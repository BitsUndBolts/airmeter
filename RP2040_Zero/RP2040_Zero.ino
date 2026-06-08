#include <Arduino.h>
#include <Adafruit_NeoPixel.h>

// ==========================================
// 0. Protocol Definition
// ==========================================
// [ HEADER: 0x55 0xAA ] → [ LEN: 0x08 ] → [ SEQ NO ] → [ PAYLOAD (8 Bytes) ] → [ CHECKSUM (CRC-8) ]

// ==========================================
// 1. HARDWARE PIN DEFINITIONS
// ==========================================

// COM lines: COM0-COM3
const int COM_PINS[4] = {28, 29, 26, 27}; // COM0, COM1, COM2, COM3

// SEG lines: SEG0-SEG14
// Index = segment number, value = GPIO pin
const int SEG_PINS[15] = {
  3,   // SEG0  → GP3
  0,   // SEG1  → GP0
  1,   // SEG2  → GP1
  2,   // SEG3  → GP2
  15,  // SEG4  → GP15
  14,  // SEG5  → GP14
  13,  // SEG6  → GP13
  4,   // SEG7  → GP4
  5,   // SEG8  → GP5
  7,   // SEG9  → GP7
  6,   // SEG10 → GP6
  11,  // SEG11 → GP11
  10,  // SEG12 → GP10
  8,   // SEG13 → GP8
  9,   // SEG14 → GP9
};

const int BUZZER_PIN = 20;

// Robust Protocol Framing Definitions
const uint8_t PREAMBLE_1  = 0x55;
const uint8_t PREAMBLE_2  = 0xAA;
const uint8_t PAYLOAD_LEN = 0x08;

// ── Global display state (last valid mux pass per COM row) ──────────────────────────────────────
uint16_t display_matrix[4] = {0, 0, 0, 0};
bool buzzer_active = false;

Adafruit_NeoPixel rgb_led(1, 16, NEO_GRB + NEO_KHZ800);

// Global protocol trackers
uint8_t sequence_number = 0;
int     heartbeat_count = 0;
bool    led_active      = true;

// CRC-8 (Dallas/Maxim: X^8 + X^5 + X^4 + 1)
uint8_t calculate_crc8(uint8_t *data, uint8_t len) {
  uint8_t crc = 0x00;
  for (uint8_t i = 0; i < len; i++) {
    crc ^= data[i];
    for (uint8_t j = 0; j < 8; j++) {
      if (crc & 0x80) crc = (crc << 1) ^ 0x31;
      else            crc <<= 1;
    }
  }
  return crc;
}

void setup() {
  rgb_led.begin();
  rgb_led.setBrightness(20);
  rgb_led.show();

  for (int i = 0; i < 4;  i++) pinMode(COM_PINS[i], INPUT_PULLUP);
  for (int i = 0; i < 15; i++) pinMode(SEG_PINS[i], INPUT_PULLUP);
  pinMode(BUZZER_PIN, INPUT_PULLUP);

  Serial1.setTX(12);    // GP12 = UART0 TX → HC-12
  Serial1.setRX(17);    // GP17 = UART0 RX → NOT USED
  Serial1.begin(9600);
}

void loop() {
  static unsigned long lastTxTime = 0;
  unsigned long currentTime = millis();

  // ── 1. Sample: overwrite per-COM row when that COM is active ──────
  for (int c = 0; c < 4; c++) {
    if (digitalRead(COM_PINS[c]) == HIGH) {
      delayMicroseconds(1000);                  // settle to bit centre

      if (digitalRead(COM_PINS[c]) == HIGH) {   // still valid
        uint16_t row_bits = 0;
        for (int s = 0; s < 15; s++) {
          if (digitalRead(SEG_PINS[s]) == HIGH)
            row_bits |= (1 << s);
        }
        display_matrix[c] = row_bits;           // OVERWRITE, not OR
      }
    }
  }

  // Buzzer: last-seen wins
  buzzer_active = digitalRead(BUZZER_PIN);

  // ── 2. TX every 333ms — no clear needed, next mux pass overwrites ─
  if (currentTime - lastTxTime >= 333) {
    lastTxTime = currentTime;

    // Snapshot the matrix so buzzer bit doesn't pollute the live buffer
    uint16_t tx_matrix[4];
    memcpy(tx_matrix, display_matrix, sizeof(display_matrix));

    // Pack buzzer into bit 15 of tx_matrix[0] only
    if (buzzer_active) tx_matrix[0] |= (1 << 15);

    // Heartbeat LED
    if (led_active) {
      rgb_led.setPixelColor(0, rgb_led.Color(0, 255, 0));
      rgb_led.show();
      heartbeat_count++;
    }

    // Build 13-byte robust packet
    uint8_t tx_packet[13];

    tx_packet[0] = PREAMBLE_1;
    tx_packet[1] = PREAMBLE_2;
    tx_packet[2] = PAYLOAD_LEN;
    tx_packet[3] = sequence_number++;

    // Payload: use tx_matrix (has buzzer bit), NOT display_matrix
    tx_packet[4]  = (tx_matrix[0] >> 8) & 0xFF; // COM0 high (+ buzzer in bit15)
    tx_packet[5]  =  tx_matrix[0]       & 0xFF; // COM0 low
    tx_packet[6]  = (tx_matrix[1] >> 8) & 0xFF; // COM1 high
    tx_packet[7]  =  tx_matrix[1]       & 0xFF; // COM1 low
    tx_packet[8]  = (tx_matrix[2] >> 8) & 0xFF; // COM2 high
    tx_packet[9]  =  tx_matrix[2]       & 0xFF; // COM2 low
    tx_packet[10] = (tx_matrix[3] >> 8) & 0xFF; // COM3 high
    tx_packet[11] =  tx_matrix[3]       & 0xFF; // COM3 low

    // CRC-8 over 10 bytes (LEN + SEQ + 8 payload)
    tx_packet[12] = calculate_crc8(&tx_packet[2], 10);

    // Send data
    Serial1.write(tx_packet, 13);

    // Heartbeat LED off
    if (led_active) {
      rgb_led.setPixelColor(0, rgb_led.Color(0, 0, 0));
      rgb_led.show();
      if (heartbeat_count >= 10) led_active = false;
    }
  }
}