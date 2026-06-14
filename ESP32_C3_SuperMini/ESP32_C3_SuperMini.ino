// =============================================================================
// ESP32 Wireless Multimeter Bridge
// Firmware Version: 1.0
//
// Architecture:
//   HC-12 (2400 baud, UART1) → ESP32 → Wi-Fi (STA or AP) → Browser (SSE)
//
// Pin Map:
//   GPIO 8  — Onboard LED  (LOW = ON, HIGH = OFF on most ESP32-C3 boards)
//   GPIO 9  — BOOT button  (INPUT_PULLUP, active LOW)
//   GPIO 20 — HC-12 TXD → ESP32 UART1 RX (receive-only)
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
// HC-12 TXD → ESP32 GPIO20 (UART1 RX). No TX needed — receive-only.

#define HC12_RX_PIN  20
#define HC12_BAUD    9600

HardwareSerial HC12(1); // UART1

// =============================================================================
// PROTOCOL CONSTANTS  (must match the transmitter firmware)
// =============================================================================

#define PREAMBLE_1   0x55
#define PREAMBLE_2   0xAA
#define PAYLOAD_LEN  0x09
#define PACKET_SIZE  14   // 2 preamble + 1 len + 1 seq + 1 META + 8 payload + 1 crc

// =============================================================================
// NETWORK & SERVER OBJECTS
// =============================================================================

static const byte DNS_PORT               = 53;  // Standard DNS port
static const int  SSE_BROADCAST_INTERVAL = 500; // ms (reserved for future throttling)

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
// RUNTIME STATE — MULTIMETER DATA
// =============================================================================

int               buzzer_status          = 0;
unsigned long     lastPacketTime         = 0;

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
static uint8_t  rx_payload[9];
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
//   - 8-bit FPS and METER_ID
//   - 60-bit LCD string  (COM0[15] + COM1[15] + COM2[15] + COM3[15])
//   - buzzer flag        (bit 15 of COM0)
// Broadcasts result immediately over SSE.
// =============================================================================

void decodePacket(const uint8_t* payload) {
  // Extract META fields from byte 0
  const uint8_t meta_byte = payload[0];
  const uint8_t fps_index = (meta_byte >> 5) & 0x07;
  const uint8_t meter_id  = meta_byte & 0x1F;

  // Reconstruct the four 16-bit COM rows (skipping payload[0])
  uint16_t com[4];
  for (int c = 0; c < 4; c++) {
    int base_idx = 1 + (c * 2); // Shift past the META byte
    com[c] = ((uint16_t)payload[base_idx] << 8) | payload[base_idx + 1];
  }

  // Extract buzzer from bit 15 of COM0 BEFORE masking
  const bool buzzer = (com[0] & 0x8000) != 0;
  buzzer_status = buzzer ? 1 : 0;

  // Strip buzzer bit — only SEG0-SEG14 (bits 0-14) carry valid LCD data
  com[0] &= 0x7FFF;

  // Build 60-character bit string: COM0[bit0..14] + COM1 + COM2 + COM3
  char bitString[61]; 
  bitString[60] = '\0';
  for (int c = 0; c < 4; c++) {
    for (int s = 0; s < 15; s++) {
      bitString[c * 15 + s] = ((com[c] >> s) & 1) ? '1' : '0';
    }
  }

  // Broadcast over SSE — increased JSON buffer sizing to 120 bytes safely 
  // to account for added "fps" and "meterId" keys
  char jsonPayload[120];
  snprintf(jsonPayload, sizeof(jsonPayload),
           "{\"lcd\":\"%s\",\"buzzer\":%d,\"fps\":%u,\"meterId\":%u}", 
           bitString, buzzer_status, fps_index, meter_id);
  events.send(jsonPayload, "multimeter-update");

  // Debug: visual 64-bit block with buzzer on COM0
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

  Serial.printf("[HC12] Packet #%lu | SEQ:%u | Meter ID:%u | FPS Idx:%u | Buzzer:%d\n%s\n",
                rx_packet_count, rx_seq, meter_id, fps_index, buzzer_status, visual.c_str());
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
          lastPacketTime = millis();
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

  String json = "[";
  int validCount = 0;

  for (int i = 0; i < n; ++i) {
    const String ssid = WiFi.SSID(i);
    if (ssid.length() > 0) {
      if (validCount > 0) json += ',';
      json += "{\"ssid\":\"" + ssid + "\",\"rssi\":" + String(WiFi.RSSI(i)) + "}";
      validCount++;
    }
  }

  json += ']';
  WiFi.scanDelete(); // Free scan memory
  return json;
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
// DEOBFUSCATION — decrypts hex-encoded XOR strings using a key
// =============================================================================
String deobfuscate(const String& hex, const char* key) {
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
    const long secondsSinceLastPacket =
        (lastPacketTime > 0) ? (long)((millis() - lastPacketTime) / 1000) : -1;

    JsonDocument doc;

    doc["firmware"] = FIRMWARE_VERSION;
    doc["ipAddress"] = WiFi.localIP().toString();
    doc["ssid"] = WiFi.SSID();
    doc["rssi"] = WiFi.RSSI();
    doc["memory"] = ESP.getFreeHeap();
    doc["last_seen"] = secondsSinceLastPacket;

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
 
      // Small delay so the TCP stack can flush the response.
      delay(300);
      ESP.restart();
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
  HC12.begin(HC12_BAUD, SERIAL_8N1, HC12_RX_PIN, -1); // -1 = no TX
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

// ─────────────────────────────────────────────────────────────────────────────
// OTA Update Handler for ESP32 (Arduino / ESPAsyncWebServer)
//
// Endpoints:
//   POST /ota/firmware  — streams a .bin directly into the OTA partition
//   POST /ota/file      — writes any file into LittleFS
//
// Dependencies:
//   - ESPAsyncWebServer  (https://github.com/me-no-dev/ESPAsyncWebServer)
//   - LittleFS           (built-in with ESP32 Arduino core >= 2.x)
//   - Update             (built-in with ESP32 Arduino core)
//
// Add to your main .ino / .cpp where you configure your AsyncWebServer.
// ─────────────────────────────────────────────────────────────────────────────

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