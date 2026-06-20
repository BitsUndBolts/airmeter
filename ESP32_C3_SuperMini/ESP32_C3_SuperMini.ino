/*
 * Copyright (c) 2026 Alexander Fuchs
 * Licensed under the MIT License.
 */

// =============================================================================
// ESP32 Wireless Multimeter Bridge
// Firmware Version: 1.0
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
// CHANNEL-CHANGE COOL-OFF
//
// When a CHANNEL is changed via /api/meter/update, packets already in flight
// (or sitting in the RP2040's UART buffer) may still arrive tagged with the
// OLD channel for a short while after the rename command goes out. Without this
// guard, decodePacket() would treat that straggler as a brand-new meter and
// silently re-create meterRegistry[oldChannel].
//
// We track a short list of "recently retired" Channels with an expiry timestamp.
// Packets for a Channel still in this list are dropped (not re-registered)
// until the cool-off window elapses — after which the Channel is open again for
// genuine re-registration (e.g. a different physical meter later reusing
// that Channel, or wrap-around re-use).
// =============================================================================

struct ChannelCooloff {
  uint8_t        channel   = 0xFF;  // 0xFF == slot unused
  unsigned long  expiresAt = 0;
};

static const uint8_t COOLOFF_SLOTS = 8; // generous headroom for rapid re-renames
static ChannelCooloff channelCooloffList[COOLOFF_SLOTS];

static const unsigned long CHANNEL_COOLOFF_MS = 5000; // 5s window

// Mark `channel` as retired for CHANNEL_COOLOFF_MS. If the list is full, overwrite the
// slot with the soonest expiry (oldest/least relevant entry).
void startChannelCooloff(uint8_t channel) {
  const unsigned long now = millis();
  int freeSlot = -1;
  int oldestSlot = 0;

  for (int i = 0; i < COOLOFF_SLOTS; i++) {
    if (channelCooloffList[i].channel == channel) {
      // Already cooling off (e.g. re-renamed again quickly) — just refresh it.
      channelCooloffList[i].expiresAt = now + CHANNEL_COOLOFF_MS;
      return;
    }
    if (channelCooloffList[i].channel == 0xFF && freeSlot == -1) {
      freeSlot = i;
    }
    if (channelCooloffList[i].expiresAt < channelCooloffList[oldestSlot].expiresAt) {
      oldestSlot = i;
    }
  }

  int slot = (freeSlot != -1) ? freeSlot : oldestSlot;
  channelCooloffList[slot].channel        = channel;
  channelCooloffList[slot].expiresAt = now + CHANNEL_COOLOFF_MS;
}

// Returns true if `channel` is currently within its cool-off window.
// Lazily expires entries it walks past, so no separate cleanup task is needed.
bool isChannelCoolingOff(uint8_t channel) {
  const unsigned long now = millis();
  for (int i = 0; i < COOLOFF_SLOTS; i++) {
    if (channelCooloffList[i].channel == channel) {
      if ((long)(channelCooloffList[i].expiresAt - now) > 0) {
        return true;
      }
      // Expired — free the slot.
      channelCooloffList[i].channel = 0xFF;
      return false;
    }
  }
  return false;
}

// =============================================================================
// FIRMWARE METADATA
// =============================================================================

const char* FIRMWARE_VERSION = "1.0";

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
// PROTOCOL CONSTANTS  (must match the transmitter firmware)
// =============================================================================

static const uint8_t PREAMBLE_1  = 0x55;
static const uint8_t PREAMBLE_2  = 0xAA;
static const uint8_t PAYLOAD_LEN = 0x09;  // 9 payload bytes received from RP2040

// =============================================================================
// NETWORK & SERVER OBJECTS
// =============================================================================

static const byte DNS_PORT               = 53;  // Standard DNS port


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

static RxState  rx_state       = WAIT_PREAMBLE_1;
static uint8_t  rx_payload[PAYLOAD_LEN];
static uint8_t  rx_seq         = 0;
static uint8_t  rx_len         = 0;
static uint8_t  rx_payload_idx = 0;
static uint32_t rx_packet_count = 0;
static uint32_t rx_error_count  = 0;

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
  const uint8_t channel  = meta_byte & 0x1F;

  // This Channel was just freed — almost certainly a straggler packet
  // sent under the old channel before the RP2040 applied the change. Drop it
  // silently rather than re-registering meterRegistry[channel].
  if (isChannelCoolingOff(channel)) {
#ifdef DEBUG_LCD_VISUAL
    Serial.printf("[COOLOFF] Dropped packet for retired channel %u\n", channel);
#endif
    return;
  }

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
        if (rx_len == PAYLOAD_LEN) {
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
// =============================================================================

void saveSettings(const char* settings) {
  preferences.begin("settings", false);
  preferences.putString("data", settings);
  preferences.end();
}

String loadSettings() {
  preferences.begin("settings", true);
  String saved = preferences.getString("data", "{}");
  preferences.end();
  return saved;
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

// =============================================================================
// ACCESS POINT MODE — captive portal for initial Wi-Fi provisioning
// =============================================================================

void startAccessPoint() {
  is_ap_mode = true;
  WiFi.mode(WIFI_AP);

  const IPAddress local_IP(192, 168, 7, 1);
  const IPAddress gateway(192, 168, 7, 1);
  const IPAddress subnet(255, 255, 255, 0);

  WiFi.softAPConfig(local_IP, gateway, subnet);
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
    if (littlefs_available && LittleFS.exists(page)) {
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
      WiFi.begin(test_ssid.c_str(), test_pass.c_str());
      connectionAttemptStart = millis();

      request->send(200, "application/json", "{\"status\":\"checking\"}");
    } else {
      request->send(400, "application/json", "{\"status\":\"error\"}");
    }
  });

  // ── API: Wi-Fi Scan ───────────────────────────────────────────────────────
  server.on("/api/scan", HTTP_GET, [](AsyncWebServerRequest* request) {
    if (!is_ap_mode) {
      request->send(403, "text/plain", "Forbidden");
      return;
    }

    const int status = WiFi.scanComplete();
    if (status == WIFI_SCAN_RUNNING) {
      request->send(202, "application/json", "{\"status\":\"scanning\"}");
    } else if (status >= 0) {
      request->send(200, "application/json", getNetworksJSON());
    } else {
      WiFi.scanDelete();
      WiFi.scanNetworks(true, false); // background async scan
      request->send(202, "application/json", "{\"status\":\"scanning\"}");
    }
  });

  // ── API: System Info ──────────────────────────────────────────────────────
  server.on("/api/system", HTTP_GET, [](AsyncWebServerRequest* request) {
    JsonDocument doc;

    doc["firmware"] = FIRMWARE_VERSION;
    doc["ipAddress"] = WiFi.localIP().toString();
    doc["ssid"] = WiFi.SSID();
    doc["rssi"] = WiFi.RSSI();
    doc["memory"] = ESP.getFreeHeap();

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

  // ── API: Settings Read ────────────────────────────────────────────────────
  server.on("/api/settings", HTTP_GET, [](AsyncWebServerRequest* request) {
    request->send(200, "application/json", loadSettings());
  });

  // ── API: List all files on LittleFS ────────────────────────────────────
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

  // ── API: List WebP images of Multimeters on LittleFS ────────────────────────────────────
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

  // ── API: Deletes a file on LittleFS ────────────────────────────────────
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

  // ── API: Renames a file on LittleFS ────────────────────────────────────
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

  // ── API: Restart ─────────────────────────────────────────────────────────
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

  // ── API: Meter Update ─────────────────────────────────────────────────────
  // Sends an RF config command to the target RP2040 and, when the caller
  // passes "commit":true, immediately migrates the ESP32-side state too
  // (settings key rename + registry move + cooloff).
  //
  // Two-phase design — why commit is caller-controlled:
  //
  //   ONLINE meter (canVerify=true in the UI):
  //     The UI's pushMeterUpdate() retry loop polls /api/system until the
  //     RP2040 echoes back the new fpsIdx/powerSave, proving delivery.
  //     Only then does the UI call this endpoint with "commit":true.
  //     Committing here is safe because RF delivery is already confirmed.
  //
  //   OFFLINE meter (canVerify=false in the UI):
  //     The RP2040 is not transmitting, so there is nothing to verify
  //     against. The UI sends "commit":false — the RF command goes out
  //     but ESP32 state is left unchanged (old channel key stays on flash,
  //     meterRegistry slot keeps its old index). When the meter powers back
  //     on it transmits on its old channel, the ESP32 sees it, and the UI
  //     detects the mismatch via /api/system. The user (or a future
  //     auto-detect flow) then calls /api/meter/commit to finalize.
  //     Without this guard, committing while the meter is off would orphan
  //     the old channel: the RP2040 wakes up transmitting on oldChannel,
  //     but the ESP32 has already renamed that key — producing two entries.
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

      // FLAGS_BYTE — full byte forwarded to the RP2040 and persisted there.
      // Bit 0 = power conservation enable. Bits 7-1 are reserved for future
      // use; callers that omit the field default to 0x00 (all flags off).
      const uint8_t flagsByte = doc["flags"].is<uint8_t>() ? doc["flags"].as<uint8_t>() : 0x00;

      // "commit":true  → caller has already verified RF delivery; migrate state now.
      // "commit":false → offline/unverified send; leave ESP32 state untouched.
      // Omitting the field defaults to false (safe).
      const bool doCommit = doc["commit"].is<bool>() && doc["commit"].as<bool>();

      // Build the 7-byte config packet
      // [ 0xAA ][ 0x55 ][ 0x03 ][ CHANNEL ][ DATA_BYTE ][ FLAGS_BYTE ][ CRC8 ]
      uint8_t tx[7];
      tx[0] = 0xAA;                                            // Preamble 1 (flipped for ESP→RP direction)
      tx[1] = 0x55;                                            // Preamble 2
      tx[2] = 0x03;                                            // Payload length (3 bytes)
      tx[3] = oldChannel;                                      // Target: who should act on this
      tx[4] = ((fpsIdx & 0x07) << 5) | (newChannel & 0x1F);  // DATA_BYTE: fpsIdx | newChannel
      tx[5] = flagsByte;                                       // FLAGS_BYTE: bit0=power conservation, bits7-1=reserved

      // CRC covers LEN + CHANNEL + DATA_BYTE + FLAGS_BYTE (4 bytes)
      uint8_t crc_input[4] = { tx[2], tx[3], tx[4], tx[5] };
      tx[6] = calculate_crc8(crc_input, 4);

      HC12.write(tx, 7);

      if (doCommit && oldChannel != newChannel) {
        // 1. Rename settings key on flash
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

        // 2. Move runtime registry entry
        if (meterRegistry[oldChannel].registered) {
          meterRegistry[newChannel] = meterRegistry[oldChannel];
          meterRegistry[oldChannel] = MeterData{};
        }

        // 3. Cool-off old channel so straggler packets are ignored
        startChannelCooloff(oldChannel);

        // Clear any cool-off on newChannel so its packets are accepted
        for (int i = 0; i < COOLOFF_SLOTS; i++) {
          if (channelCooloffList[i].channel == newChannel) {
            channelCooloffList[i].channel = 0xFF;
            break;
          }
        }

        Serial.printf("[METER] Channel %u → %u committed (fpsIdx=%u flags=0x%02X)\n",
                      oldChannel, newChannel, fpsIdx, flagsByte);
      } else {
        Serial.printf("[METER] Channel %u RF sent%s (fpsIdx=%u flags=0x%02X)\n",
                      oldChannel, doCommit ? " [no rename needed]" : " [commit deferred]",
                      fpsIdx, flagsByte);
      }

      request->send(200, "application/json", "{\"status\":\"success\"}");
    }
  );

  // ── API: Meter Commit (offline-path finalizer) ────────────────────────────
  // Called by the UI only for the offline path — after the meter has woken up
  // and the UI has confirmed (via /api/system) that the RP2040 is now echoing
  // the new channel/fpsIdx/powerSave. This finalizes the ESP32-side rename
  // that was deliberately deferred while the meter was off.
  //
  // For the online path this endpoint is never called — /api/meter/update
  // already committed with "commit":true after RF delivery was verified.
  server.on("/api/meter/commit", HTTP_POST, [](AsyncWebServerRequest* request) {}, NULL,
    [](AsyncWebServerRequest* request, uint8_t* data, size_t len, size_t, size_t) {
      JsonDocument doc;
      if (deserializeJson(doc, data, len)) {
        request->send(400, "application/json", "{\"status\":\"error\"}");
        return;
      }

      if (!doc["oldChannel"].is<uint8_t>() || doc["oldChannel"].as<uint8_t>() > 31 ||
          !doc["newChannel"].is<uint8_t>() || doc["newChannel"].as<uint8_t>() > 31) {
        request->send(400, "application/json",
                      "{\"status\":\"error\",\"message\":\"Invalid channel\"}");
        return;
      }

      const uint8_t oldChannel = doc["oldChannel"].as<uint8_t>();
      const uint8_t newChannel = doc["newChannel"].as<uint8_t>();

      if (oldChannel == newChannel) {
        request->send(200, "application/json", "{\"status\":\"success\"}");
        return;
      }

      // 1. Rename settings key on flash
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

      // 2. Move runtime registry entry
      if (meterRegistry[oldChannel].registered) {
        meterRegistry[newChannel] = meterRegistry[oldChannel];
        meterRegistry[oldChannel] = MeterData{};
      }

      // 3. Cool-off old channel so straggler packets are ignored
      startChannelCooloff(oldChannel);

      // Clear any cool-off on newChannel so its packets are accepted
      for (int i = 0; i < COOLOFF_SLOTS; i++) {
        if (channelCooloffList[i].channel == newChannel) {
          channelCooloffList[i].channel = 0xFF;
          break;
        }
      }

      Serial.printf("[METER] Offline commit: channel %u → %u finalized\n",
                    oldChannel, newChannel);

      request->send(200, "application/json", "{\"status\":\"success\"}");
    }
  );

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

      // Remove from stored settings
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

  // ── API: File Upload ─────────────────────────────────────────────────────────
  server.on("/ota/file", HTTP_POST, [](AsyncWebServerRequest* request) {
      if (fileCtx.error) {
        request->send(500, "text/plain", fileCtx.errorMsg);
        return;
      }
      request->send(200, "text/plain", "OK");
    },
 
    // ── Upload handler ────────────────────────────────────────────────────
    handleFileUpload
  );

  // ── API: Firmware Upload ─────────────────────────────────────────────────────────
  server.on("/ota/firmware", HTTP_POST, [](AsyncWebServerRequest* request) {
      if (otaCtx.error) {
        request->send(500, "text/plain", otaCtx.errorMsg);
        return;
      }
 
      // Send 200 first, then reboot — gives the browser time to receive the
      // response before the device disappears off the network.
      request->send(200, "text/plain", "OK");
 
      // Schedule restart so the TCP stack can flush the response.
      rebootTimer = millis(); 
      pendingReboot = true;   
    },
 
    // ── Upload handler (called per chunk) ─────────────────────────────────
    handleFirmwareUpload
  );

  // ── Static Assets (LittleFS) ──────────────────────────────────────────────
  if (littlefs_available) {
    server.serveStatic("/", LittleFS, "/");
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
    const String path = request->url();

    // Block direct access to known system paths without extensions
    if (path == "/" || path == "/save" ||
        path.startsWith("/api/") || path == "/events") {
      request->send(404, "text/plain", "404: Not Found");
      return;
    }

    // Extensionless path → try appending .html (e.g., /meter → /meter.html)
    if (path.indexOf('.') == -1) {
      const String htmlPath = path + ".html";
      if (littlefs_available && LittleFS.exists(htmlPath)) {
        request->send(LittleFS, htmlPath, "text/html");
        return;
      }
    }

    // Exact file match with extension
    if (littlefs_available && LittleFS.exists(path)) {
      request->send(LittleFS, path);
      return;
    }

    // True 404
    request->send(404, "text/plain",
                  "Error 404: '" + path + "' not found on this device.");
  });

  // Clear out any stale boot-up garbage bytes
  while(HC12.available() > 0) {
    HC12.read();
  }

  server.begin();
  Serial.println("[SYSTEM] Web server started.");
}

// =============================================================================
// SETUP
// =============================================================================

void setup() {
  Serial.begin(115200);
  Serial.println("\n--- ESP32 Booting ---");

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

  // ── Load Wi-Fi Credentials ────────────────────────────────────────────────
  preferences.begin("wifi-config", true);
  wifi_ssid      = preferences.getString("ssid", "");
  wifi_password  = preferences.getString("pass", "");
  preferences.end();

  // ── Network Strategy ──────────────────────────────────────────────────────
  if (wifi_ssid.isEmpty()) {
    startAccessPoint();
  } else {
    Serial.printf("[NETWORK] Connecting to: %s\n", wifi_ssid.c_str());
    WiFi.begin(wifi_ssid.c_str(), wifi_password.c_str());
    WiFi.setSleep(false);
    esp_wifi_set_ps(WIFI_PS_NONE); // Disable radio sleep for lowest latency
    Serial.println("[SYSTEM] Wi-Fi power saving disabled.");

    int retries = 0;
    while (WiFi.status() != WL_CONNECTED && retries < 120) { // 60 s max
      delay(500);
      Serial.print('.');
      retries++;
    }

    if (WiFi.status() == WL_CONNECTED) {
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
    otaCtx = UploadContext{};   // reset
 
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
    fileCtx = UploadContext{};  // reset
 
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

  // ── HC-12 Receive ─────────────────────────────────────────────────────────
  processHC12();

  yield();
}