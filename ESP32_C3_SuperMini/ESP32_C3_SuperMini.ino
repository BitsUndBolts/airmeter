/*
 * Copyright (c) 2026 Alexander Fuchs
 * Licensed under the MIT License.
 */

// =============================================================================
// ESP32 Wireless Multimeter Bridge
// Firmware Version: 1.1
//
// Architecture:
//   HC-12 (9600 baud, UART1) → ESP32 → Wi-Fi (STA or AP) → Browser (SSE)
//
// Pin Map:
//   GPIO 8  — Onboard LED  (LOW = ON, HIGH = OFF on most ESP32-C3 boards)
//   GPIO 9  — BOOT button  (INPUT_PULLUP, active LOW)
//   GPIO 20 — Receiving:   HC-12 TXD        → ESP32 UART1 RX
//   GPIO 21 — Sending:     ESP32 UART1 TX   → HC-12 RXD
// =============================================================================

#include <WiFi.h>
#include <Update.h>
#include <esp_wifi.h>
#include <ESPAsyncWebServer.h>
#include <LittleFS.h>
#include <ESPmDNS.h>
#include <Preferences.h>
#include <DNSServer.h>
#include <ArduinoJson.h>
#include "esp_bt.h"

//#define DEBUG_LCD_VISUAL  // Comment out to suppress per-packet LCD debug output

// =============================================================================
// STORED METER DATA SINCE BOOT - REFRESHES WITH EACH PACKET ARRIVAL
// =============================================================================
struct MeterData {
    unsigned long lastSeen    = 0;
    uint8_t lastFpsIndex      = 0;
    uint8_t lastPowerSave     = 0;  // 1 = power conservation enabled, 0 = disabled
    bool    registered        = false; // true once this slot has been populated
};

// Fixed 32-slot array indexed by channel (0–31). No heap allocation, O(1)
// access. Slots are zeroed at boot; `registered` marks which are active.
// This replaces std::map<uint8_t, MeterData> to avoid per-entry heap nodes
// on a microcontroller with a fixed, small channel space.
static MeterData meterRegistry[32];


// =============================================================================
// IMMUTABLE ESP32 HARDWARE PROFILE - POPULATED ONCE AT BOOT
// =============================================================================
struct ESPStaticDetails {
    const char* model = nullptr;
    const char* ver   = nullptr;
    uint32_t    rev   = 0;
    uint8_t     cores = 0;
};

// Global static instance allocated entirely on the BSS segment (stack/RAM).
// Eliminates runtime heap allocations (`new` / `malloc`) for metadata.
// Serves as a lightning-fast RAM cache to avoid reading hardware registers
// or generating temporary strings during frequent JSON serialization loops.
static ESPStaticDetails espStaticDetails;


// =============================================================================
// METER UPDATE TICKET
//
// A single in-RAM pending update slot. Only one config update is allowed at
// a time — a new /api/meter/update call while a ticket is active replaces it.
//
// Lifecycle:
//   EMPTY    — no pending update; /api/meter/status returns 503
//   PENDING  — RF sent, waiting for confirming packet from RP2040
//   SUCCESS  — confirming packet received and committed; /api/meter/status
//              returns 200 once, then deletes the ticket (EMPTY again)
//
// Retry logic (inside tickMeterUpdate(), called every loop()):
//   Every TICKET_RETRY_INTERVAL_MS the RF command is resent if no confirming
//   packet has arrived. After TICKET_MAX_RETRIES resends without confirmation
//   the ticket is deleted (EMPTY) and the next status poll returns 503.
// =============================================================================

static const unsigned long TICKET_RETRY_INTERVAL_MS = 2000; // 2s per attempt
static const uint8_t       TICKET_MAX_RETRIES        = 3;   // 3 attempts → ~6s total

enum TicketStatus : uint8_t {
  TICKET_EMPTY,
  TICKET_PENDING,
  TICKET_SUCCESS
};

struct MeterUpdateTicket {
  TicketStatus status      = TICKET_EMPTY;
  uint32_t     id          = 0;        // opaque ticket ID returned to the UI
  uint8_t      oldChannel  = 0;
  uint8_t      newChannel  = 0;
  uint8_t      fpsIdx      = 0;
  uint8_t      flagsByte   = 0;
  uint8_t      powerSave   = 0;        // expected powerSave value (bit 0 of flagsByte)
  uint8_t      retryCount  = 0;
  unsigned long nextRetryAt = 0;       // millis() timestamp for next RF resend
  // Confirmed values stored here on success, ready for the UI to consume
  uint8_t      confirmedChannel  = 0;
  uint8_t      confirmedFpsIdx   = 0;
  uint8_t      confirmedPowerSave = 0;
};

static MeterUpdateTicket activeTicket;
static uint32_t          ticketIdCounter = 1; // monotonic; never 0 (0 = "no ticket")


// =============================================================================
// FIRMWARE METADATA
// =============================================================================

const char* FIRMWARE_VERSION = "1.2";

// ── Helpers ───────────────────────────────────────────────────────────────────

// Shared flag so the upload handler and the response handler can coordinate.
// ESPAsyncWebServer calls the upload handler in chunks, then the request
// handler once at the end — we need to pass the error state between them.
struct UploadContext {
  bool     error    = false;
  String   errorMsg = "";
  File     fsFile;            // used by the LittleFS handler only
};

// One context per concurrent request. For a single-user OTA page this is fine.
// If you ever need to support parallel uploads, use a map keyed on request ptr.
static UploadContext otaCtx;
static UploadContext fileCtx;

// =============================================================================
// HARDWARE CONFIGURATION
// =============================================================================

static const int  ONBOARD_LED_PIN  = 8;
static const int  BOOT_BUTTON_PIN  = 9;

// =============================================================================
// HC-12 UART CONFIGURATION
// =============================================================================

static const int HC12_RX_PIN = 20;
static const int HC12_TX_PIN = 21;
static const int HC12_BAUD   = 9600;

HardwareSerial HC12(1); // UART1

// =============================================================================
// PROTOCOL CONSTANTS (must match the transmitter firmware)
// =============================================================================

static const uint8_t PREAMBLE_1       = 0x55;
static const uint8_t PREAMBLE_2       = 0xAA;
static const uint8_t PAYLOAD_LEN_IN   = 0x09;  // 9 payload bytes received from RP2040
static const uint8_t PAYLOAD_LEN_OUT  = 0x03;  // 3 payload bytes sent to RP2040

// =============================================================================
// NETWORK & SERVER OBJECTS
// =============================================================================

static const byte DNS_PORT = 53;  // Standard DNS port


AsyncWebServer    server(80);
AsyncEventSource  events("/events"); // Server-Sent Events endpoint
DNSServer         dnsServer;
Preferences       preferences;

// =============================================================================
// RUNTIME STATE — NETWORK
// =============================================================================

String            wifi_ssid              = "";
String            wifi_password          = "";
bool              is_ap_mode             = false;
bool              littlefs_available     = false;

bool              pendingReboot          = false;
unsigned long     rebootTimer            = 0;
unsigned long     connectionAttemptStart = 0;

// =============================================================================
// RUNTIME STATE — HARDWARE
// =============================================================================

bool              isBootButtonPressed    = false;
unsigned long     bootButtonPressedTime  = 0;

// =============================================================================
// PACKET RECEIVE STATE MACHINE
// =============================================================================

enum RxState : uint8_t {
  WAIT_PREAMBLE_1,
  WAIT_PREAMBLE_2,
  WAIT_LEN,
  WAIT_SEQ,
  WAIT_PAYLOAD,
  WAIT_CRC
};

static RxState  rx_state        = WAIT_PREAMBLE_1;
static uint8_t  rx_payload[PAYLOAD_LEN_IN];
static uint8_t  rx_seq          = 0;
static uint8_t  rx_len          = 0;
static uint8_t  rx_payload_idx  = 0;
static uint32_t rx_packet_count = 0;
static uint32_t rx_error_count  = 0;

// =============================================================================
// SETTINGS CACHE
//
// loadSettings() / saveSettings() keep an in-RAM copy so that frequent async
// API handlers (e.g. /api/settings GET, /api/meter/delete) never block the
// network task waiting for NVS flash reads. Flash is only written when the
// value actually changes. The cache is primed once during setup().
// =============================================================================

static String settingsCache = "";

// =============================================================================
// FORWARD DECLARATIONS
// =============================================================================

void    startAccessPoint();
void    setupWebServer();
void    saveSettings(const char* settings);
String  loadSettings();
void    wipe(bool fullReset);
void    factoryReset(bool fullReset, const char* source);
uint8_t calculate_crc8(const uint8_t* data, uint8_t len);
void    decodePacket(const uint8_t* payload);
void    processHC12();
String  getNetworksJSON();
void    sendMeterUpdateRF(uint8_t oldCh, uint8_t newCh, uint8_t fpsIdx, uint8_t flagsByte);
void    commitChannelChange(uint8_t oldChannel, uint8_t newChannel);
void    tickMeterUpdate();
void    handleFirmwareUpload(AsyncWebServerRequest*, const String&, size_t, uint8_t*, size_t, bool);
void    handleFileUpload(AsyncWebServerRequest*, const String&, size_t, uint8_t*, size_t, bool);

// =============================================================================
// CRC-8  (Dallas/Maxim — must match transmitter)
// =============================================================================

uint8_t calculate_crc8(const uint8_t* data, uint8_t len) {
  uint8_t crc = 0x00;
  for (uint8_t i = 0; i < len; i++) {
    crc ^= data[i];
    for (uint8_t j = 0; j < 8; j++) {
      crc = (crc & 0x80) ? (crc << 1) ^ 0x31 : (crc << 1);
    }
  }
  return crc;
}

// =============================================================================
// METER UPDATE HELPERS
// =============================================================================

// Builds and sends the 7-byte RF config packet to the RP2040.
// Called both from /api/meter/update (first send) and tickMeterUpdate() (retries).
void sendMeterUpdateRF(uint8_t oldCh, uint8_t newCh, uint8_t fpsIdx, uint8_t flagsByte) {
  uint8_t tx[7];
  tx[0] = PREAMBLE_2; // PREAMBLES flipped for outgoing messages
  tx[1] = PREAMBLE_1;
  tx[2] = PAYLOAD_LEN_OUT;
  tx[3] = oldCh;
  tx[4] = ((fpsIdx & 0x07) << 5) | (newCh & 0x1F);
  tx[5] = flagsByte;
  uint8_t crc_input[4] = { tx[2], tx[3], tx[4], tx[5] };
  tx[6] = calculate_crc8(crc_input, 4);
  HC12.write(tx, 7);
}

// Commits a confirmed channel rename on the ESP32 side:
//   1. Rename settings key on flash
//   2. Migrate runtime registry entry
// Called from decodePacket() the moment a confirming packet arrives.
// Only called when oldChannel != newChannel.
void commitChannelChange(uint8_t oldChannel, uint8_t newChannel) {
  // 1. Rename settings key on flash (goes through cache — fast path)
  String raw = loadSettings();
  JsonDocument cfg;
  deserializeJson(cfg, raw);
  const String oldKey = String(oldChannel);
  const String newKey = String(newChannel);
  if (cfg["meters"][oldKey]) {
    cfg["meters"][newKey] = cfg["meters"][oldKey];
    cfg["meters"].remove(oldKey.c_str());
    String out; serializeJson(cfg, out);
    saveSettings(out.c_str());
  }

  // 2. Migrate runtime registry entry
  if (meterRegistry[oldChannel].registered) {
    meterRegistry[newChannel] = meterRegistry[oldChannel];
    meterRegistry[oldChannel] = MeterData{};
  }

  Serial.printf("[TICKET] Channel %u → %u committed\n", oldChannel, newChannel);
}

// Called every loop() tick. Drives the retry cadence and expiry of the active
// ticket. Resends the RF command every TICKET_RETRY_INTERVAL_MS. Deletes the
// ticket (→ TICKET_EMPTY) after TICKET_MAX_RETRIES without confirmation, so
// the next /api/meter/status poll returns 503 (not found = failed).
void tickMeterUpdate() {
  if (activeTicket.status != TICKET_PENDING) return;

  const unsigned long now = millis();
  if ((long)(now - activeTicket.nextRetryAt) < 0) return; // not yet time

  if (activeTicket.retryCount >= TICKET_MAX_RETRIES) {
    // All attempts exhausted — delete ticket; next status poll returns 503
    Serial.printf("[TICKET] %u expired after %u retries — update failed\n",
                  activeTicket.id, TICKET_MAX_RETRIES);
    activeTicket.status = TICKET_EMPTY;
    return;
  }

  activeTicket.retryCount++;
  activeTicket.nextRetryAt = now + TICKET_RETRY_INTERVAL_MS;

  Serial.printf("[TICKET] %u retry %u/%u — resending RF\n",
                activeTicket.id, activeTicket.retryCount, TICKET_MAX_RETRIES);

  sendMeterUpdateRF(activeTicket.oldChannel, activeTicket.newChannel,
                    activeTicket.fpsIdx,     activeTicket.flagsByte);
}

// =============================================================================
// PACKET DECODER
// Unpacks 9 payload bytes into:
//   - 8-bit FPS INDEX and CHANNEL
//   - 60-bit LCD string  (COM0[15] + COM1[15] + COM2[15] + COM3[15])
//   - buzzer flag        (bit 15 of COM0)
// Broadcasts result immediately over SSE.
// =============================================================================

void decodePacket(const uint8_t* payload) {
  // Extract META fields from byte 0
  const uint8_t meta_byte = payload[0];
  const uint8_t fps_index = (meta_byte >> 5) & 0x07;
  const uint8_t channel   = meta_byte & 0x1F;

  // Reconstruct the four 16-bit COM rows (skipping payload[0])
  uint16_t com[4];
  for (int c = 0; c < 4; c++) {
    int base_idx = 1 + (c * 2); // payload[0] is META; COM rows start at index 1, 2 bytes each
    com[c] = ((uint16_t)payload[base_idx] << 8) | payload[base_idx + 1];
  }

  // Extract buzzer from bit 15 of COM0 BEFORE masking
  const bool buzzer = (com[0] & 0x8000) != 0;
  int buzzer_status = buzzer ? 1 : 0;

  // Extract power-conservation flag from bit 15 of COM1 BEFORE masking
  const bool powerSave = (com[1] & 0x8000) != 0;
  int power_save_status = powerSave ? 1 : 0;

  // Strip injected status bits — only SEG0-SEG14 (bits 0-14) carry valid LCD data
  com[0] &= 0x7FFF;
  com[1] &= 0x7FFF;

  // Build 60-character bit string: COM0[bit0..14] + COM1 + COM2 + COM3
  char bitString[61];
  bitString[60] = '\0';
  for (int c = 0; c < 4; c++) {
    for (int s = 0; s < 15; s++) {
      bitString[c * 15 + s] = ((com[c] >> s) & 1) ? '1' : '0';
    }
  }

  // Track per-meter last-seen timestamp, fps_index, and power conservation flag
  meterRegistry[channel].lastSeen      = millis();
  meterRegistry[channel].lastFpsIndex  = fps_index;
  meterRegistry[channel].lastPowerSave = power_save_status;
  meterRegistry[channel].registered    = true;

  // ── Ticket confirmation check ──────────────────────────────────────────────
  // If there is a pending update ticket, check whether this packet is the
  // confirmation we have been waiting for. A packet confirms delivery when:
  //   - it arrives on the expected NEW channel
  //   - fpsIdx and powerSave match what was requested
  // On confirmation: commit any channel rename on flash, mark ticket SUCCESS,
  // store the confirmed values for the UI to consume via /api/meter/status.
  if (activeTicket.status == TICKET_PENDING &&
      channel           == activeTicket.newChannel  &&
      fps_index         == activeTicket.fpsIdx      &&
      power_save_status == activeTicket.powerSave) {

    if (activeTicket.oldChannel != activeTicket.newChannel) {
      commitChannelChange(activeTicket.oldChannel, activeTicket.newChannel);
    }

    activeTicket.status              = TICKET_SUCCESS;
    activeTicket.confirmedChannel    = channel;
    activeTicket.confirmedFpsIdx     = fps_index;
    activeTicket.confirmedPowerSave  = power_save_status;

    Serial.printf("[TICKET] %u confirmed — channel %u fpsIdx %u powerSave %u\n",
                  activeTicket.id, channel, fps_index, power_save_status);
  }

  // Broadcast over SSE — CHANNEL is now encoded in the event name
  // (meter-data-<channel>) instead of the payload, since each stream
  // only ever carries one meter's data.
  char jsonPayload[140];
  snprintf(jsonPayload, sizeof(jsonPayload),
           "{\"lcd\":\"%s\",\"buzzer\":%d,\"fpsIdx\":%u,\"powerSave\":%d}",
           bitString, buzzer_status, fps_index, power_save_status);

  // Separate SSE event names per CHANNEL
  char eventName[16];
  snprintf(eventName, sizeof(eventName), "meter-data-%u", channel);
  events.send(jsonPayload, eventName);

  // Debug: 4-row visual display (1 buzzer prefix + 15 SEG bits per COM row)
#ifdef DEBUG_LCD_VISUAL
  String visual;
  visual.reserve(72);
  for (int c = 0; c < 4; c++) {
    visual += (c == 0) ? (buzzer ? '1' : '0') : '0';
    visual += ' ';
    for (int bit = 14; bit >= 0; bit--) {
      visual += ((com[c] >> bit) & 1) ? '1' : '0';
    }
    if (c < 3) visual += '\n';
  }

  Serial.printf("[HC12] Packet #%lu | SEQ:%u | Channel:%u | FPS Idx:%u | Buzzer:%d | PowerSave:%d\n%s\n",
                rx_packet_count, rx_seq, channel, fps_index, buzzer_status, power_save_status, visual.c_str());
#endif
}

// =============================================================================
// NON-BLOCKING HC-12 BYTE PARSER  — call every loop() tick
// =============================================================================

void processHC12() {
  while (HC12.available()) {
    const uint8_t b = HC12.read();

    switch (rx_state) {

      case WAIT_PREAMBLE_1:
        if (b == PREAMBLE_1) rx_state = WAIT_PREAMBLE_2;
        break;

      case WAIT_PREAMBLE_2:
        if (b == PREAMBLE_2) {
          rx_state = WAIT_LEN;
        } else if (b == PREAMBLE_1) {
          rx_state = WAIT_PREAMBLE_2;
        } else {
          rx_state = WAIT_PREAMBLE_1;
        }
        break;

      case WAIT_LEN:
        rx_len = b;
        if (rx_len == PAYLOAD_LEN_IN) {
          rx_state = WAIT_SEQ;
        } else {
          Serial.printf("[HC12] Bad LEN byte: 0x%02X — resyncing\n", b);
          rx_error_count++;
          rx_state = WAIT_PREAMBLE_1;
        }
        break;

      case WAIT_SEQ:
        rx_seq         = b;
        rx_payload_idx = 0;
        rx_state       = WAIT_PAYLOAD;
        break;

      case WAIT_PAYLOAD:
        rx_payload[rx_payload_idx++] = b;
        if (rx_payload_idx == rx_len) {
          rx_state = WAIT_CRC;
        }
        break;

      case WAIT_CRC: {
        // CRC covers LEN + SEQ + PAYLOAD (11 bytes)
        uint8_t crc_input[11];
        crc_input[0] = rx_len;
        crc_input[1] = rx_seq;
        memcpy(&crc_input[2], rx_payload, rx_len);

        const uint8_t expected_crc = calculate_crc8(crc_input, 11);

        if (b == expected_crc) {
          rx_packet_count++;
          decodePacket(rx_payload);
        } else {
          rx_error_count++;
          Serial.printf("[HC12] CRC FAIL — got 0x%02X expected 0x%02X (total errors: %lu)\n",
                        b, expected_crc, rx_error_count);
        }
        rx_state = WAIT_PREAMBLE_1;
        break;
      }
    }
  }
}

// =============================================================================
// WI-FI SCAN — returns nearby networks as a JSON array
// =============================================================================

String getNetworksJSON() {
  const int n = WiFi.scanComplete();

  if (n == WIFI_SCAN_FAILED || n == WIFI_SCAN_RUNNING) {
    return "[]";
  }

  JsonDocument doc;
  JsonArray arr = doc.to<JsonArray>();

  for (int i = 0; i < n; ++i) {
    const String ssid = WiFi.SSID(i);
    if (ssid.length() > 0) {
      JsonObject entry = arr.add<JsonObject>();
      entry["ssid"] = ssid;           // ArduinoJson escapes quotes and backslashes
      entry["rssi"] = WiFi.RSSI(i);
    }
  }

  WiFi.scanDelete();

  String out;
  serializeJson(doc, out);
  return out;
}

// =============================================================================
// NON-VOLATILE SETTINGS — save / load
//
// An in-RAM cache (settingsCache) is primed once during setup() and kept in
// sync on every write. Async request handlers call loadSettings() and get the
// cached string immediately without touching NVS flash. Flash is only written
// in saveSettings() when the value actually changes, which prevents unnecessary
// wear and eliminates the multi-millisecond NVS stall from the async task.
// =============================================================================

void saveSettings(const char* settings) {
  // Update RAM cache first so subsequent reads within the same request cycle
  // see the new value without waiting for the flash write to complete.
  settingsCache = settings;

  preferences.begin("settings", false);
  preferences.putString("data", settings);
  preferences.end();
}

String loadSettings() {
  // Return cached value if available — avoids NVS read on the hot path.
  if (settingsCache.length() > 0) return settingsCache;

  // Cold path: first call after boot before cache is primed by setup().
  preferences.begin("settings", true);
  settingsCache = preferences.getString("data", "{}");
  preferences.end();
  return settingsCache;
}

// =============================================================================
// CREDENTIAL & SETTINGS WIPE
//   fullReset = true  → wipe Wi-Fi credentials AND user settings
//   fullReset = false → wipe Wi-Fi credentials only
// =============================================================================

void wipe(bool fullReset) {
  preferences.begin("wifi-config", false);
  preferences.clear();
  preferences.end();

  if (fullReset) {
    preferences.begin("settings", false);
    preferences.clear();
    preferences.end();
    settingsCache = ""; // invalidate RAM cache so next read hits NVS
  }
}

// =============================================================================
// FACTORY RESET  (wipe + optional full clear, triggers a pending reboot)
// =============================================================================

void factoryReset(bool fullReset, const char* source) {
  Serial.printf("\n[RESET] Factory Reset triggered via %s\n", source);
  digitalWrite(ONBOARD_LED_PIN, LOW); // LED ON — visual feedback
  wipe(fullReset);
}

// =============================================================================
// DEOBFUSCATION — decodes hex-encoded XOR strings using a key
// =============================================================================

String deobfuscate(const String& hex, const char* key) {
  if (hex.length() == 0 || hex.length() % 2 != 0) return "";  // Malformed input
  size_t keyLen = strlen(key);
  String result = "";
  result.reserve(hex.length() / 2);

  for (size_t i = 0; i < hex.length(); i += 2) {
    // Parse two hex chars back into a byte
    uint8_t byte = (uint8_t)strtol(hex.substring(i, i + 2).c_str(), nullptr, 16);
    // XOR with the key
    result += (char)(byte ^ key[(i / 2) % keyLen]);
  }
  return result;
}


/**
 * Changes the Wi-Fi power saving mode instantly on-the-fly.
 * Does NOT disconnect or drop the Wi-Fi link.
 * 
 * @param enable False for ultra-low latency (<15ms), True for maximum power savings (~220ms).
 */
void setWifiPowerSaving(bool enable) {
    if (enable) {
        // Turns on max power savings. The radio sleeps when idle.
        esp_wifi_set_ps(WIFI_PS_MAX_MODEM);
        Serial.println("[Wi-Fi] Power Savings ON (~220ms latency, lower power)");
    } else {
        // Turns off power savings. The radio stays fully awake.
        esp_wifi_set_ps(WIFI_PS_NONE);
        Serial.println("[Wi-Fi] Power Savings OFF (<15ms latency, higher power)");
    }
}

// =============================================================================
// ACCESS POINT MODE — captive portal for initial Wi-Fi provisioning
//
// Brownout fix for ESP32-C3 SuperMini:
//   The SuperMini's onboard LDO cannot sustain the current surge that occurs
//   when the radio fires up at full TX power immediately after a software
//   reset (e.g. right after OTA flashing). The symptoms are the AP appearing
//   in serial output but never broadcasting a visible SSID — the radio browns
//   out silently before the first beacon can be sent.
//
//   Fix — three steps applied in order:
//     1. Explicitly tear down any prior radio state with softAPdisconnect()
//        and disconnect() before switching mode. This clears leftover state
//        from the flash/reboot cycle that can cause the radio driver to start
//        in a partially-initialised condition.
//     2. Set mode to WIFI_AP first, then clamp TX power to WIFI_POWER_8_5dBm
//        BEFORE calling softAP(). At 8.5 dBm the peak current draw stays
//        within what the SuperMini's LDO can supply.
//     3. The same TX power clamp is applied before WiFi.begin() in STA mode
//        for the same reason.
// =============================================================================

void startAccessPoint() {
  is_ap_mode = true;

  // 1. Explicitly stop any active Wi-Fi processes and clear memory
  WiFi.softAPdisconnect(true);
  WiFi.disconnect(true);
  delay(100); // Give the radio subsystem a moment to shut down

  // 2. Set the mode fresh
  WiFi.mode(WIFI_AP);

  const IPAddress local_IP(192, 168, 7, 1);
  const IPAddress gateway(192, 168, 7, 1);
  const IPAddress subnet(255, 255, 255, 0);

  WiFi.softAPConfig(local_IP, gateway, subnet);

  // 3. Lower the TX power immediately before broadcasting (Crucial for SuperMini)
  // High TX power during software reset causes severe brownouts on this specific board
  WiFi.setTxPower(WIFI_POWER_8_5dBm);

  WiFi.softAP("AirMeter-Setup", "airmeter123");

  dnsServer.start(DNS_PORT, "*", WiFi.softAPIP());

  Serial.println("\n[SYSTEM] Access Point Mode active.");
  Serial.print("[SYSTEM] Connect to 'AirMeter-Setup' — IP: ");
  Serial.println(WiFi.softAPIP());
}

// =============================================================================
// WEB SERVER — routes and SSE configuration
// =============================================================================

void setupWebServer() {

  // ── Root ──────────────────────────────────────────────────────────────────
  server.on("/", HTTP_GET, [](AsyncWebServerRequest* request) {
    const char* page = is_ap_mode ? "/setup.html" : "/index.html";
    String gzPage = String(page) + ".gz";

    if (littlefs_available && LittleFS.exists(gzPage)) {
      AsyncWebServerResponse* response = request->beginResponse(LittleFS, gzPage, "text/html");
      response->addHeader("Content-Encoding", "gzip");
      request->send(response);
    } else if (littlefs_available && LittleFS.exists(page)) {
      request->send(LittleFS, page, "text/html");
    } else {
      char msg[64];
      snprintf(msg, sizeof(msg), "Error 404: '%s' missing from storage.", page);
      request->send(404, "text/plain", msg);
    }
  });

  // ── Wi-Fi Credentials Save ────────────────────────────────────────────────
  server.on("/save", HTTP_POST, [](AsyncWebServerRequest* request) {
    if (request->hasParam("ssid", true) && request->hasParam("pass", true)) {
      const String test_ssid = request->getParam("ssid", true)->value();
      const String test_pass = deobfuscate(request->getParam("pass", true)->value(), "bitsundbolts");

      if (test_pass.isEmpty()) {
        request->send(400, "application/json", "{\"status\":\"error\",\"message\":\"Malformed password\"}");
        return;
      }

      Serial.printf("[NETWORK] Testing connection to: %s\n", test_ssid.c_str());

      preferences.begin("wifi-config", false);
      preferences.putString("ssid", test_ssid);
      preferences.putString("pass", test_pass);
      preferences.end();

      WiFi.mode(WIFI_AP_STA);
      WiFi.setTxPower(WIFI_POWER_8_5dBm);  // Clamp power immediately before begin()
      setWifiPowerSaving(false);
      WiFi.begin(test_ssid.c_str(), test_pass.c_str());
      connectionAttemptStart = millis();

      request->send(200, "application/json", "{\"status\":\"checking\"}");
    } else {
      request->send(400, "application/json", "{\"status\":\"error\"}");
    }
  });

  // ── API: Wi-Fi Scan ───────────────────────────────────────────────────────
  // Debounced to 3s to prevent rapid clicks from cancelling an in-progress
  // scan. Each call that arrives while WIFI_SCAN_RUNNING just returns 202
  // without re-triggering a new scan.
  server.on("/api/scan", HTTP_GET, [](AsyncWebServerRequest* request) {
    if (!is_ap_mode) {
      request->send(403, "text/plain", "Forbidden");
      return;
    }

    static unsigned long lastScanTrigger = 0;
    const int status = WiFi.scanComplete();

    if (status == WIFI_SCAN_RUNNING) {
      request->send(202, "application/json", "{\"status\":\"scanning\"}");
    } else if (status >= 0) {
      request->send(200, "application/json", getNetworksJSON());
    } else {
      // Only trigger a new scan if 3 s have elapsed since the last trigger.
      // This prevents rapid "Rescan" clicks from restarting and cancelling
      // each other in quick succession.
      if (millis() - lastScanTrigger > 3000) {
        lastScanTrigger = millis();
        WiFi.scanDelete();
        WiFi.scanNetworks(true, false); // background async scan
      }
      request->send(202, "application/json", "{\"status\":\"scanning\"}");
    }
  });

  // ── API: System Info ──────────────────────────────────────────────────────
  server.on("/api/system", HTTP_GET, [](AsyncWebServerRequest* request) {
    JsonDocument doc;

    doc["firmware"]  = FIRMWARE_VERSION;
    doc["ipAddress"] = WiFi.localIP().toString();
    doc["ssid"]      = WiFi.SSID();
    doc["rssi"]      = WiFi.RSSI();

    // Dedicated ESP32 object with chip details
    JsonObject espObj = doc["esp"].to<JsonObject>();
    // --- Dynamic Values (Polled live every time) ---
    espObj["memory"] = ESP.getFreeHeap();
    espObj["temp"]   = temperatureRead();
    espObj["freq"]   = ESP.getCpuFreqMHz();
    {
      wifi_ps_type_t psMode = WIFI_PS_NONE;
      esp_wifi_get_ps(&psMode);
      espObj["wifiPs"] = (psMode != WIFI_PS_NONE); // true = power saving ON
    }
    // --- Static Values (Read from your fast RAM struct cache) ---
    espObj["model"]  = espStaticDetails.model;
    espObj["cores"]  = espStaticDetails.cores;
    espObj["rev"]    = espStaticDetails.rev;
    espObj["ver"]    = espStaticDetails.ver;

    // Per-meter tracking array — only meters seen since boot
    JsonArray meters = doc["meters"].to<JsonArray>();
    const unsigned long now = millis();

    for (uint8_t ch = 0; ch < 32; ch++) {
      if (!meterRegistry[ch].registered) continue;
      JsonObject m = meters.add<JsonObject>();
      m["channel"]   = ch;
      m["seen"]      = (long)((now - meterRegistry[ch].lastSeen) / 1000);
      m["fpsIdx"]    = meterRegistry[ch].lastFpsIndex;
      m["powerSave"] = meterRegistry[ch].lastPowerSave;
    }

    if (littlefs_available) {
      size_t totalBytes = LittleFS.totalBytes();
      size_t usedBytes  = LittleFS.usedBytes();
      doc["littlefs_total"] = totalBytes;
      doc["littlefs_free"]  = totalBytes - usedBytes;
    } else {
      doc["littlefs_total"] = 0;
      doc["littlefs_free"]  = 0;
    }

    String jsonResponse;
    serializeJson(doc, jsonResponse);

    request->send(200, "application/json", jsonResponse);
  });

  // ── API: Diagnostics ─────────────────────────────────────────────────────
  // Dedicated endpoint for the diagnostics page. Exposes radio counters
  // and WiFi metrics that /api/system intentionally omits (kept lean for
  // the dashboard). Poll this at whatever rate the diagnostics page needs
  // without affecting dashboard cadence.
  server.on("/api/diagnostics", HTTP_GET, [](AsyncWebServerRequest* request) {
    JsonDocument doc;

    // ── WiFi ──────────────────────────────────────────────────────────────
    doc["rssi"]          = WiFi.RSSI();
    doc["channel"]       = WiFi.channel();       // RF channel (1-13)
    doc["bssid"]         = WiFi.BSSIDstr();      // AP MAC — useful for roaming diagnosis
    doc["txPower"]       = WiFi.getTxPower();    // dBm — confirms 8.5 dBm cap is active
    // ── HC-12 packet counters ──────────────────────────────────────────────
    doc["hc12RxTotal"]   = rx_packet_count;      // valid packets decoded since boot
    doc["hc12ErrTotal"]  = rx_error_count;       // CRC or framing errors since boot

    // ── Per-meter snapshot ────────────────────────────────────────────────
    // Mirrors the meters array from /api/system but adds nothing new —
    // the diagnostics page needs fpsIdx to compute expected packet rate,
    // and lastSeen to detect stale channels. Keeping it here means the page
    // never has to poll two endpoints simultaneously.
    JsonArray meters = doc["meters"].to<JsonArray>();
    const unsigned long now = millis();

    for (uint8_t ch = 0; ch < 32; ch++) {
      if (!meterRegistry[ch].registered) continue;
      JsonObject m = meters.add<JsonObject>();
      m["channel"]   = ch;
      m["seen"]      = (long)((now - meterRegistry[ch].lastSeen) / 1000);
      m["fpsIdx"]    = meterRegistry[ch].lastFpsIndex;
      m["powerSave"] = meterRegistry[ch].lastPowerSave;
    }

    // ── Uptime ────────────────────────────────────────────────────────────
    // millis() overflows after ~49 days; cast to unsigned long is intentional.
    doc["uptimeMs"]  = (unsigned long)millis();

    String out;
    serializeJson(doc, out);
    request->send(200, "application/json", out);
  });

  // ── API: Settings Read ────────────────────────────────────────────────────
  server.on("/api/settings", HTTP_GET, [](AsyncWebServerRequest* request) {
    // loadSettings() returns from RAM cache — no NVS flash read on this path
    request->send(200, "application/json", loadSettings());
  });

  // ── API: List all files on LittleFS ──────────────────────────────────────
  server.on("/api/files/all", HTTP_GET, [](AsyncWebServerRequest* request) {
    if (!littlefs_available) {
      request->send(200, "application/json", "[]");
      return;
    }

    JsonDocument doc;
    JsonArray arr = doc.to<JsonArray>();

    File root = LittleFS.open("/");
    File file = root.openNextFile();
    while (file) {
      if (!file.isDirectory()) {
        JsonObject entry = arr.add<JsonObject>();
        entry["name"] = String(file.name());
        entry["size"] = file.size();
      }
      file = root.openNextFile();
    }

    String out;
    serializeJson(doc, out);
    request->send(200, "application/json", out);
  });

  // ── API: List WebP images of Multimeters on LittleFS ─────────────────────
  server.on("/api/files/meters", HTTP_GET, [](AsyncWebServerRequest* request) {
    if (!littlefs_available) {
      request->send(200, "application/json", "[]");
      return;
    }

    JsonDocument doc;
    JsonArray arr = doc.to<JsonArray>();

    File root = LittleFS.open("/");
    File file = root.openNextFile();
    while (file) {
      String name = String(file.name());
      if (name.startsWith("mm_") && name.endsWith(".webp")) {
        arr.add(name);
      }
      file = root.openNextFile();
    }

    String out;
    serializeJson(doc, out);
    request->send(200, "application/json", out);
  });

  // ── API: Delete a file on LittleFS ───────────────────────────────────────
  server.on("/api/files/delete", HTTP_POST, [](AsyncWebServerRequest* request) {}, NULL,
    [](AsyncWebServerRequest* request, uint8_t* data, size_t len, size_t, size_t) {
      JsonDocument doc;
      if (deserializeJson(doc, data, len)) {
        request->send(400, "application/json", "{\"status\":\"error\",\"message\":\"Invalid JSON\"}");
        return;
      }

      if (!doc["path"].is<const char*>()) {
        request->send(400, "application/json", "{\"status\":\"error\",\"message\":\"Missing path\"}");
        return;
      }

      const String path = doc["path"].as<String>();
      if (path.isEmpty() || path == "/") {
        request->send(400, "application/json", "{\"status\":\"error\",\"message\":\"Invalid path\"}");
        return;
      }

      if (!littlefs_available) {
        request->send(503, "application/json", "{\"status\":\"error\",\"message\":\"Storage unavailable\"}");
        return;
      }

      if (!LittleFS.exists(path)) {
        request->send(404, "application/json", "{\"status\":\"error\",\"message\":\"File not found\"}");
        return;
      }

      if (LittleFS.remove(path)) {
        Serial.printf("[FS] Deleted: %s\n", path.c_str());
        request->send(200, "application/json", "{\"status\":\"success\"}");
      } else {
        request->send(500, "application/json", "{\"status\":\"error\",\"message\":\"Delete failed\"}");
      }
    }
  );

  // ── API: Rename a file on LittleFS ───────────────────────────────────────
  server.on("/api/files/rename", HTTP_POST, [](AsyncWebServerRequest* request) {}, NULL,
    [](AsyncWebServerRequest* request, uint8_t* data, size_t len, size_t, size_t) {
      JsonDocument doc;
      if (deserializeJson(doc, data, len)) {
        request->send(400, "application/json", "{\"status\":\"error\",\"message\":\"Invalid JSON\"}");
        return;
      }

      if (!doc["from"].is<const char*>() || !doc["to"].is<const char*>()) {
        request->send(400, "application/json", "{\"status\":\"error\",\"message\":\"Missing from/to\"}");
        return;
      }

      const String fromPath = doc["from"].as<String>();
      const String toPath   = doc["to"].as<String>();

      if (fromPath.isEmpty() || fromPath == "/" || toPath.isEmpty() || toPath == "/") {
        request->send(400, "application/json", "{\"status\":\"error\",\"message\":\"Invalid path\"}");
        return;
      }

      if (fromPath == toPath) {
        request->send(200, "application/json", "{\"status\":\"success\"}");
        return;
      }

      if (!littlefs_available) {
        request->send(503, "application/json", "{\"status\":\"error\",\"message\":\"Storage unavailable\"}");
        return;
      }

      if (!LittleFS.exists(fromPath)) {
        request->send(404, "application/json", "{\"status\":\"error\",\"message\":\"File not found\"}");
        return;
      }

      if (LittleFS.exists(toPath)) {
        request->send(409, "application/json", "{\"status\":\"error\",\"message\":\"Target name already exists\"}");
        return;
      }

      if (LittleFS.rename(fromPath, toPath)) {
        Serial.printf("[FS] Renamed: %s → %s\n", fromPath.c_str(), toPath.c_str());
        request->send(200, "application/json", "{\"status\":\"success\"}");
      } else {
        request->send(500, "application/json", "{\"status\":\"error\",\"message\":\"Rename failed\"}");
      }
    }
  );

  // ── API: Settings Write ───────────────────────────────────────────────────
  server.on("/api/settings", HTTP_POST,
    [](AsyncWebServerRequest* request) {
      request->send(200, "application/json", "{\"status\":\"success\"}");
    },
    NULL,
    [](AsyncWebServerRequest* request, uint8_t* data, size_t len,
       size_t index, size_t total) {
      if (index == 0) {
        request->_tempObject = malloc(total + 1);
        if (request->_tempObject) {
          memset(request->_tempObject, 0, total + 1);

          request->onDisconnect([request]() {
            if (request->_tempObject) {
              free(request->_tempObject);
              request->_tempObject = nullptr;
            }
          });
        }
      }
      if (request->_tempObject) {
        memcpy((uint8_t*)request->_tempObject + index, data, len);
      }
      if (index + len == total && request->_tempObject) {
        saveSettings((char*)request->_tempObject);
        free(request->_tempObject);
        request->_tempObject = nullptr;
      }
    }
  );

  // ── API: Connection Status (AP mode only) ─────────────────────────────────
  server.on("/api/status", HTTP_GET, [](AsyncWebServerRequest* request) {
    if (!is_ap_mode) {
      request->send(403, "text/plain", "Forbidden");
      return;
    }

    const wl_status_t status = WiFi.status();

    if (status == WL_CONNECTED && !pendingReboot) {
      request->send(200, "application/json", "{\"status\":\"success\"}");
      rebootTimer   = millis();
      pendingReboot = true;
      return;
    }

    if (status == WL_CONNECT_FAILED || status == WL_NO_SSID_AVAIL) {
      WiFi.disconnect();
      WiFi.mode(WIFI_AP);
      wipe(false);
      request->send(200, "application/json", "{\"status\":\"fail\"}");
      return;
    }

    // Time-based failsafe: give up after 10 s in the retry loop
    if (connectionAttemptStart > 0 &&
        millis() - connectionAttemptStart > 10000) {
      Serial.println("[TIMEOUT] Auth threshold exceeded — forcing fail state.");
      WiFi.disconnect(true);
      WiFi.mode(WIFI_AP);
      wipe(false);
      request->send(200, "application/json", "{\"status\":\"fail\"}");
      return;
    }

    request->send(200, "application/json", "{\"status\":\"still_checking\"}");
  });

  // ── API: Factory Reset (STA mode only) ───────────────────────────────────
  server.on("/api/factory-reset", HTTP_POST, [](AsyncWebServerRequest* request) {
    if (is_ap_mode) {
      request->send(403, "application/json",
                    "{\"status\":\"error\",\"message\":\"Forbidden in setup mode\"}");
      return;
    }
    factoryReset(false, "Web UI");
    request->send(200, "application/json", "{\"status\":\"success\"}");
    rebootTimer   = millis();
    pendingReboot = true;
  });

  // ── API: Restart ──────────────────────────────────────────────────────────
  server.on("/api/restart", HTTP_POST, [](AsyncWebServerRequest* request) {
    if (is_ap_mode) {
      request->send(403, "application/json",
                    "{\"status\":\"error\",\"message\":\"Forbidden in setup mode\"}");
      return;
    }

    request->send(200, "application/json", "{\"status\":\"success\"}");
    rebootTimer   = millis();
    pendingReboot = true;
  });

  // ── API: CPU Frequency ────────────────────────────────────────────────────
  // POST /api/system/freq  body: { "freq": 80 } or { "freq": 160 }
  // Applies the new clock speed immediately (Wi-Fi stays up at both values)
  // and persists it to Preferences so it survives reboots.
  server.on("/api/system/freq", HTTP_POST, [](AsyncWebServerRequest* request) {}, NULL,
    [](AsyncWebServerRequest* request, uint8_t* data, size_t len, size_t, size_t) {
      JsonDocument doc;
      if (deserializeJson(doc, data, len) || !doc["freq"].is<int>()) {
        request->send(400, "application/json", "{\"status\":\"error\",\"message\":\"Invalid JSON or missing freq\"}");
        return;
      }

      const int requested = doc["freq"].as<int>();

      // Only 80 and 160 MHz are safe with Wi-Fi active on ESP32-C3.
      if (requested != 80 && requested != 160) {
        request->send(400, "application/json",
                      "{\"status\":\"error\",\"message\":\"freq must be 80 or 160\"}");
        return;
      }

      const uint32_t newFreq = (uint32_t)requested;

      // Apply immediately — setCpuFrequencyMhz() is safe to call at runtime.
      setCpuFrequencyMhz(newFreq);

      // Persist into the shared JSON settings blob so factory reset covers it.
      {
        JsonDocument cfg;
        deserializeJson(cfg, loadSettings());
        cfg["cpu_freq_mhz"] = newFreq;
        String out; serializeJson(cfg, out);
        saveSettings(out.c_str());
      }

      Serial.printf("[CPU] Frequency changed to %u MHz\n", newFreq);

      request->send(200, "application/json", "{\"status\":\"success\"}");
    }
  );

  // ── API: Wi-Fi Power Saving ───────────────────────────────────────────────
  // POST /api/system/wifi-ps  body: { "ps": true } or { "ps": false }
  //   true  → WIFI_PS_MAX_MODEM (~220ms latency, lower power draw)
  //   false → WIFI_PS_NONE      (<15ms latency,  higher power draw)
  // Applied immediately without a reboot; persisted to the JSON settings blob.
  server.on("/api/system/wifi-ps", HTTP_POST, [](AsyncWebServerRequest* request) {}, NULL,
    [](AsyncWebServerRequest* request, uint8_t* data, size_t len, size_t, size_t) {
      JsonDocument doc;
      if (deserializeJson(doc, data, len) || !doc["ps"].is<bool>()) {
        request->send(400, "application/json",
                      "{\"status\":\"error\",\"message\":\"Invalid JSON or missing ps field\"}");
        return;
      }

      const bool psOn = doc["ps"].as<bool>(); // true = power saving ON (MAX_MODEM)
      setWifiPowerSaving(psOn);

      // Persist: wifi_ps_none = !psOn (we store the inverse so false is the safe default)
      {
        JsonDocument cfg;
        deserializeJson(cfg, loadSettings());
        cfg["wifi_ps"] = !psOn;
        String out; serializeJson(cfg, out);
        saveSettings(out.c_str());
      }

      Serial.printf("[Wi-Fi] Power saving set to %s via Web UI\n", psOn ? "ON" : "OFF");
      request->send(200, "application/json", "{\"status\":\"success\"}");
    }
  );

  // ── API: Meter Update ─────────────────────────────────────────────────────
  // Sends the RF config command to the target RP2040, creates a pending ticket,
  // and returns 202 + the ticket ID immediately. The ESP32 backend drives all
  // retry logic via tickMeterUpdate() in loop(). The UI polls
  // /api/meter/status/<id> every second to learn the outcome.
  //
  // Only one ticket is active at a time. A new request while a ticket is
  // pending replaces it (last writer wins — the old update is abandoned).
  server.on("/api/meter/update", HTTP_POST, [](AsyncWebServerRequest* request) {}, NULL,
    [](AsyncWebServerRequest* request, uint8_t* data, size_t len, size_t, size_t) {
      JsonDocument doc;
      if (deserializeJson(doc, data, len)) {
        request->send(400, "application/json", "{\"status\":\"error\"}");
        return;
      }

      if (!doc["oldChannel"].is<uint8_t>() || doc["oldChannel"].as<uint8_t>() > 31 ||
          !doc["newChannel"].is<uint8_t>() || doc["newChannel"].as<uint8_t>() > 31 ||
          !doc["fpsIdx"].is<uint8_t>()     || doc["fpsIdx"].as<uint8_t>() > 7) {
        request->send(400, "application/json",
                      "{\"status\":\"error\",\"message\":\"Invalid field value\"}");
        return;
      }

      const uint8_t oldChannel = doc["oldChannel"].as<uint8_t>();
      const uint8_t newChannel = doc["newChannel"].as<uint8_t>();
      const uint8_t fpsIdx     = doc["fpsIdx"].as<uint8_t>();
      const uint8_t flagsByte  = doc["flags"].is<uint8_t>() ? doc["flags"].as<uint8_t>() : 0x00;
      const uint8_t powerSave  = flagsByte & 0x01;

      // Create (or replace) the active ticket
      activeTicket.status      = TICKET_PENDING;
      activeTicket.id          = ticketIdCounter++;
      activeTicket.oldChannel  = oldChannel;
      activeTicket.newChannel  = newChannel;
      activeTicket.fpsIdx      = fpsIdx;
      activeTicket.flagsByte   = flagsByte;
      activeTicket.powerSave   = powerSave;
      activeTicket.retryCount  = 0;
      activeTicket.nextRetryAt = millis() + TICKET_RETRY_INTERVAL_MS;

      // Send first RF attempt immediately
      sendMeterUpdateRF(oldChannel, newChannel, fpsIdx, flagsByte);

      Serial.printf("[TICKET] %u created — ch %u→%u fpsIdx %u flags 0x%02X\n",
                    activeTicket.id, oldChannel, newChannel, fpsIdx, flagsByte);

      char resp[48];
      snprintf(resp, sizeof(resp), "{\"status\":\"pending\",\"ticket\":%lu}",
               (unsigned long)activeTicket.id);
      request->send(202, "application/json", resp);
    }
  );

  // ── API: Meter Status ─────────────────────────────────────────────────────
  // GET /api/meter/status/<ticket>
  //
  //   202 → ticket still pending; poll again in 1s
  //   200 → confirmed; body contains new channel, fpsIdx, powerSave.
  //         Ticket is consumed and deleted immediately (one delivery).
  //   503 → ticket not found (expired / never existed / already consumed).
  //         The update failed — show failure UI.
  server.on("/api/meter/status", HTTP_GET, [](AsyncWebServerRequest* request) {
    // Extract ticket ID from the URL path: /api/meter/status/<id>
    const String path      = request->url();          // e.g. "/api/meter/status/42"
    const int    lastSlash = path.lastIndexOf('/');
    const uint32_t requestedId = (lastSlash >= 0)
      ? (uint32_t)path.substring(lastSlash + 1).toInt()
      : 0;

    if (requestedId == 0) {
      request->send(400, "application/json", "{\"status\":\"error\",\"message\":\"Missing ticket ID\"}");
      return;
    }

    // Ticket not found or already consumed → 503
    if (activeTicket.status == TICKET_EMPTY || activeTicket.id != requestedId) {
      request->send(503, "application/json", "{\"status\":\"failed\"}");
      return;
    }

    // Still waiting → 202
    if (activeTicket.status == TICKET_PENDING) {
      request->send(202, "application/json", "{\"status\":\"pending\"}");
      return;
    }

    // Success → 200 with confirmed values; consume ticket immediately
    char resp[96];
    snprintf(resp, sizeof(resp),
             "{\"status\":\"success\",\"channel\":%u,\"fpsIdx\":%u,\"powerSave\":%u}",
             activeTicket.confirmedChannel,
             activeTicket.confirmedFpsIdx,
             activeTicket.confirmedPowerSave);

    activeTicket.status = TICKET_EMPTY; // consumed — next poll returns 503

    request->send(200, "application/json", resp);
  });

  // ── API: Meter Delete ─────────────────────────────────────────────────────
  server.on("/api/meter/delete", HTTP_POST, [](AsyncWebServerRequest* request) {}, NULL,
    [](AsyncWebServerRequest* request, uint8_t* data, size_t len, size_t, size_t) {
      JsonDocument doc;
      if (deserializeJson(doc, data, len)) {
        request->send(400, "application/json", "{\"status\":\"error\"}");
        return;
      }

      if (!doc["channel"].is<uint8_t>() || doc["channel"].as<uint8_t>() > 31) {
        request->send(400, "application/json",
                      "{\"status\":\"error\",\"message\":\"Invalid channel\"}");
        return;
      }
      const uint8_t channel = doc["channel"].as<uint8_t>();

      // Remove from stored settings (goes through cache — fast path)
      String raw = loadSettings();
      JsonDocument cfg;
      deserializeJson(cfg, raw);
      cfg["meters"].remove(String(channel).c_str());
      String out; serializeJson(cfg, out);
      saveSettings(out.c_str());

      // Remove from runtime stored meter data (won't show on dashboard anymore)
      meterRegistry[channel] = MeterData{};  // clear slot

      request->send(200, "application/json", "{\"status\":\"success\"}");
    }
  );

  // ── API: File Upload ──────────────────────────────────────────────────────
  server.on("/ota/file", HTTP_POST, [](AsyncWebServerRequest* request) {
      if (fileCtx.error) {
        request->send(500, "text/plain", fileCtx.errorMsg);
        return;
      }
      request->send(200, "text/plain", "OK");
    },
    handleFileUpload
  );

  // ── API: Firmware Upload ──────────────────────────────────────────────────
  server.on("/ota/firmware", HTTP_POST, [](AsyncWebServerRequest* request) {
      if (otaCtx.error) {
        request->send(500, "text/plain", otaCtx.errorMsg);
        return;
      }

      // Send 200 first, then reboot — gives the browser time to receive the
      // response before the device disappears off the network.
      request->send(200, "text/plain", "OK");

      // Schedule restart so the TCP stack can flush the response.
      rebootTimer   = millis();
      pendingReboot = true;
    },
    handleFirmwareUpload
  );

  // ── Static Assets (LittleFS) ──────────────────────────────────────────────
  // Cache-Control: max-age=3600 tells browsers to serve CSS/JS/images from
  // their local cache for up to one hour. This eliminates redundant HTTP
  // round-trips on every page load. OTA updates force a full page reload
  // after flashing, so stale caches are not a concern in practice.
  if (littlefs_available) {
    server.serveStatic("/", LittleFS, "/").setCacheControl("max-age=3600");
  }

  // ── SSE Handler ───────────────────────────────────────────────────────────
  events.onConnect([](AsyncEventSourceClient* client) {
    if (client->lastId()) {
      Serial.printf("[SSE] Client reconnected. Last event ID: %u\n", client->lastId());
    }
    client->send("Connected to Multimeter Core Stream Engine", nullptr, millis(), 10000);
  });
  server.addHandler(&events);

  // ── Catch-All / Dynamic HTML Rewrite ─────────────────────────────────────
  server.onNotFound([](AsyncWebServerRequest* request) {
    String path = request->url();

    // Reserved/dynamic routes — never let these fall through to file serving.
    // Prefix checks auto-cover every current and future /api/ and /ota/ route;
    // the exact entries cover the remaining standalone, non-prefixed routes.
    if (path == "/" || path == "/save" || path == "/events" ||
        path.startsWith("/api/") || path.startsWith("/ota/")) {
      request->send(404, "text/plain", "404: Not Found");
      return;
    }

    if (!littlefs_available) {
      request->send(404, "text/plain", "Error 404: '" + path + "' not found on this device.");
      return;
    }

    // Extensionless path → try appending .html (e.g., /meter → /meter.html)
    if (path.indexOf('.') == -1) {
      path += ".html";
    }

    const String contentType = getContentType(path);
    const String gzPath = path + ".gz";

    if (LittleFS.exists(gzPath)) {
      AsyncWebServerResponse* response = request->beginResponse(LittleFS, gzPath, contentType);
      response->addHeader("Content-Encoding", "gzip");
      request->send(response);
      return;
    }

    if (LittleFS.exists(path)) {
      request->send(LittleFS, path, contentType);
      return;
    }

    request->send(404, "text/plain", "Error 404: '" + path + "' not found on this device.");
  });

  // Clear out any stale boot-up garbage bytes
  while (HC12.available() > 0) {
    HC12.read();
  }

  server.begin();
  Serial.println("[SYSTEM] Web server started.");
}

String getContentType(const String& path) {
  if (path.endsWith(".html")) return "text/html";
  if (path.endsWith(".css"))  return "text/css";
  if (path.endsWith(".js"))   return "application/javascript";
  if (path.endsWith(".json")) return "application/json";
  if (path.endsWith(".svg"))  return "image/svg+xml";
  if (path.endsWith(".png"))  return "image/png";
  if (path.endsWith(".jpg") || path.endsWith(".jpeg")) return "image/jpeg";
  if (path.endsWith(".ico"))  return "image/x-icon";
  return "text/plain";
}

// =============================================================================
// OTA UPLOAD HANDLERS
//   POST /ota/firmware  — streams a .bin into the OTA partition, then reboots
//   POST /ota/file      — writes any file to LittleFS
// =============================================================================

// ── /ota/firmware ─────────────────────────────────────────────────────────────

void handleFirmwareUpload(AsyncWebServerRequest* request,
                          const String& filename,
                          size_t index,
                          uint8_t* data,
                          size_t len,
                          bool final)
{
  if (index == 0) {
    // First chunk — initialise
    otaCtx = UploadContext{};

    Serial.printf("[OTA] Firmware upload started: %s\n", filename.c_str());

    // content_len() is -1 when the browser doesn't send Content-Length inside
    // the multipart part, but the overall request length is usually available.
    size_t fileSize = request->contentLength();

    if (!Update.begin(fileSize > 0 ? fileSize : UPDATE_SIZE_UNKNOWN)) {
      otaCtx.error    = true;
      otaCtx.errorMsg = "Update.begin() failed: " + String(Update.errorString());
      Serial.println("[OTA] " + otaCtx.errorMsg);
      return;
    }
  }

  if (otaCtx.error) return;   // skip remaining chunks after an error

  if (Update.write(data, len) != len) {
    otaCtx.error    = true;
    otaCtx.errorMsg = "Write error: " + String(Update.errorString());
    Serial.println("[OTA] " + otaCtx.errorMsg);
    Update.abort();
    return;
  }

  if (final) {
    if (!Update.end(true)) {
      otaCtx.error    = true;
      otaCtx.errorMsg = "Finalise error: " + String(Update.errorString());
      Serial.println("[OTA] " + otaCtx.errorMsg);
    } else {
      Serial.printf("[OTA] Firmware flashed successfully (%u bytes)\n",
                    index + len);
    }
  }
}

// ── /ota/file ─────────────────────────────────────────────────────────────────

void handleFileUpload(AsyncWebServerRequest* request,
                      const String& filename,
                      size_t index,
                      uint8_t* data,
                      size_t len,
                      bool final)
{
  if (index == 0) {
    fileCtx = UploadContext{};

    // The HTML sends the desired target path in a 'path' field.
    // Fall back to /filename if not provided.
    String targetPath = "/" + filename;
    if (request->hasParam("path", true)) {
      targetPath = request->getParam("path", true)->value();
      // Ensure leading slash
      if (!targetPath.startsWith("/")) targetPath = "/" + targetPath;
    }

    // Sanity-check free space
    size_t fileSize = request->contentLength();
    if (fileSize > 0 && fileSize > LittleFS.totalBytes() - LittleFS.usedBytes()) {
      fileCtx.error    = true;
      fileCtx.errorMsg = "Insufficient LittleFS space";
      Serial.println("[OTA] " + fileCtx.errorMsg);
      return;
    }

    Serial.printf("[OTA] File upload started: %s → %s\n",
                  filename.c_str(), targetPath.c_str());

    fileCtx.fsFile = LittleFS.open(targetPath, "w");
    if (!fileCtx.fsFile) {
      fileCtx.error    = true;
      fileCtx.errorMsg = "Failed to open " + targetPath + " for writing";
      Serial.println("[OTA] " + fileCtx.errorMsg);
      return;
    }
  }

  if (fileCtx.error) return;

  if (len > 0) {
    size_t written = fileCtx.fsFile.write(data, len);
    if (written != len) {
      fileCtx.error    = true;
      fileCtx.errorMsg = "Write error (disk full?)";
      fileCtx.fsFile.close();
      Serial.println("[OTA] " + fileCtx.errorMsg);
      return;
    }
  }

  if (final) {
    fileCtx.fsFile.close();
    Serial.printf("[OTA] File written successfully (%u bytes)\n", index + len);
  }
}

// =============================================================================
// SETUP
// =============================================================================

void setup() {
  Serial.begin(115200);
  Serial.println("\n--- ESP32 Booting ---");

  // ── Completely Deactivate and Purge Bluetooth ──────────────────────────
  // 1. Release the BLE controller memory back to the global heap allocator.
  //    This must be called BEFORE initializing or using any BLE functionality.
  esp_bt_mem_release(ESP_BT_MODE_BLE);
  Serial.println("[SYSTEM] Bluetooth hardware purged. RAM released to heap.");

  // Cache static hardware specs once
  espStaticDetails.model = ESP.getChipModel();
  espStaticDetails.cores = ESP.getChipCores();
  espStaticDetails.rev   = ESP.getChipRevision();
  espStaticDetails.ver   = ESP.getCoreVersion();

  // ── GPIO ──────────────────────────────────────────────────────────────────
  pinMode(BOOT_BUTTON_PIN,  INPUT_PULLUP);
  pinMode(ONBOARD_LED_PIN,  OUTPUT);
  digitalWrite(ONBOARD_LED_PIN, LOW); // LED ON during boot

  // ── HC-12 UART ────────────────────────────────────────────────────────────
  HC12.begin(HC12_BAUD, SERIAL_8N1, HC12_RX_PIN, HC12_TX_PIN);
  Serial.println("[HC12] UART1 RX active on GPIO20 @ 9600 baud.");

  // ── LittleFS ──────────────────────────────────────────────────────────────
  if (LittleFS.begin(true)) {
    littlefs_available = true;
    Serial.println("[STORAGE] LittleFS mounted.");
  } else {
    Serial.println("[CRITICAL] LittleFS mount failed!");
  }

  // ── Prime settings cache from NVS ─────────────────────────────────────────
  // Done once here so every subsequent loadSettings() call in async handlers
  // returns immediately from RAM without touching flash.
  preferences.begin("settings", true);
  settingsCache = preferences.getString("data", "{}");
  preferences.end();
  Serial.println("[SETTINGS] Cache primed from NVS.");

  // ── Load Wi-Fi Credentials ────────────────────────────────────────────────
  preferences.begin("wifi-config", true);
  wifi_ssid      = preferences.getString("ssid", "");
  wifi_password  = preferences.getString("pass", "");
  preferences.end();

  // ── Apply saved CPU frequency and Wi-Fi power saving from JSON settings ──
  // Both settings live in the "settings" JSON blob so factory reset covers them.
  // Safe CPU values with Wi-Fi on ESP32-C3: 80 MHz and 160 MHz only.
  // Defaults: 80 MHz (low power start), Wi-Fi power saving ON (MAX_MODEM).
  // The device always boots into low-power mode; the user opts in to performance
  // via the dashboard toggles.
  {
    JsonDocument cfgDoc;
    deserializeJson(cfgDoc, settingsCache);

    const uint32_t savedFreq  = cfgDoc["cpu_freq_mhz"] | 80;
    const uint32_t safeFreq   = (savedFreq == 160) ? 160 : 80;
    setCpuFrequencyMhz(safeFreq);
    Serial.printf("[CPU] Frequency set to %u MHz\n", safeFreq);

    // Wi-Fi power saving: boot default is MAX_MODEM (true).
    // Only disable (WIFI_PS_NONE) when the user has explicitly saved wifi_ps_none=true.
    const bool wifiPsNone = cfgDoc["wifi_ps"] | false;
    setWifiPowerSaving(!wifiPsNone);
  }

  // ── Network Strategy ──────────────────────────────────────────────────────
  if (wifi_ssid.isEmpty()) {
    startAccessPoint();
  } else {
    Serial.printf("[NETWORK] Connecting to: %s\n", wifi_ssid.c_str());

    WiFi.mode(WIFI_STA);                  // Explicitly initialise Station Mode first
    WiFi.setTxPower(WIFI_POWER_8_5dBm);   // Cap the radio power immediately
    // Use WIFI_PS_NONE during the connection attempt for reliability —
    // re-apply the saved setting once connected.
    setWifiPowerSaving(false);
    WiFi.begin(wifi_ssid.c_str(), wifi_password.c_str());

    int retries = 0;
    while (WiFi.status() != WL_CONNECTED && retries < 120) { // 60s max
      delay(500);
      Serial.print('.');
      retries++;
    }

    if (WiFi.status() == WL_CONNECTED) {
      // Re-apply the saved power saving preference now that the link is up.
      {
        JsonDocument cfgDoc;
        deserializeJson(cfgDoc, settingsCache);
        const bool wifiPsNone = cfgDoc["wifi_ps"] | false;
        setWifiPowerSaving(!wifiPsNone);
      }
      digitalWrite(ONBOARD_LED_PIN, HIGH); // LED OFF — connected
      Serial.printf("\n[NETWORK] Connected! IP: %s\n",
                    WiFi.localIP().toString().c_str());

      if (MDNS.begin("airmeter")) {
        MDNS.addService("http", "tcp", 80);
        Serial.println("[DNS] mDNS active: http://airmeter.local");
      }
    } else {
      Serial.println("\n[WARNING] Connection failed — dropping to AP mode.");
      startAccessPoint();
    }
  }

  // ── HTTP Routes ───────────────────────────────────────────────────────────
  setupWebServer();
}

// =============================================================================
// LOOP
// =============================================================================

void loop() {
  // ── BOOT Button: 5-second hold → factory reset ───────────────────────────
  const int bootButtonState = digitalRead(BOOT_BUTTON_PIN);

  if (bootButtonState == LOW && !isBootButtonPressed) {
    bootButtonPressedTime = millis();
    isBootButtonPressed   = true;
  } else if (bootButtonState == LOW && isBootButtonPressed) {
    if (millis() - bootButtonPressedTime >= 5000) {
      factoryReset(true, "BOOT Button");
      while (digitalRead(BOOT_BUTTON_PIN) == LOW) delay(50); // wait for release
      ESP.restart();
    }
  } else if (bootButtonState == HIGH && isBootButtonPressed) {
    isBootButtonPressed = false; // Released before threshold — cancel
  }

  // ── Captive Portal DNS ────────────────────────────────────────────────────
  if (is_ap_mode) {
    dnsServer.processNextRequest();
  }

  // ── Pending Reboot ────────────────────────────────────────────────────────
  if (pendingReboot && millis() - rebootTimer > 500) {
    Serial.println("[SYSTEM] Executing reboot...");
    ESP.restart();
  }

  // ── Wi-Fi Reconnection Watchdog ───────────────────────────────────────────
  // In STA mode, check every 10s whether the Wi-Fi connection is still up.
  // If it has dropped (router restart, DHCP lease expiry, signal loss) call
  // WiFi.reconnect() to re-associate without a full reboot. The watchdog is
  // skipped while pendingReboot is set so it doesn't interfere with OTA
  // restart timing.
  if (!is_ap_mode && !pendingReboot) {
    static unsigned long lastWifiCheck = 0;
    if (millis() - lastWifiCheck > 10000) {
      lastWifiCheck = millis();
      if (WiFi.status() != WL_CONNECTED) {
        Serial.println("[NETWORK] Wi-Fi lost — reconnecting...");
        WiFi.reconnect();
      }
    }
  }

  // ── SSE Keep-Alive ────────────────────────────────────────────────────────
  // Send a ping event every 25s so routers and reverse proxies do not
  // close idle SSE connections before the next meter packet arrives.
  // Browsers treat unknown event types as no-ops, so this is invisible
  // to the UI event listeners which only subscribe to "meter-data-<N>".
  {
    static unsigned long lastSSEPing = 0;
    if (millis() - lastSSEPing > 25000) {
      lastSSEPing += 25000;
      events.send("", "ping");
    }
  }

  // ── Meter Update Ticket ───────────────────────────────────────────────────
  tickMeterUpdate();

  // ── HC-12 Receive ─────────────────────────────────────────────────────────
  processHC12();

  yield();
}