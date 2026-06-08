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

bool buzzer_active = false;

Adafruit_NeoPixel rgb_led(1, 16, NEO_GRB + NEO_KHZ800);

// Global protocol trackers
uint8_t sequence_number = 0;
int     heartbeat_count = 0;
bool    led_active      = true;
int     loopCounter = 0;

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
  // Use a static variable to keep track of the last time a packet was sent
  static unsigned long lastTxTime = 0;
  unsigned long currentTime = millis();

  // 1. LCD CAPTURING (Runs continuously every loop iteration)
  // Your 32ms protocol decoding logic sits here or right before transmission.
  // Because the loop is non-blocking, it can process the LCD signals smoothly.
  
  // 2. TIMING CHECK: Only execute packet compilation and TX every 333ms
  if (currentTime - lastTxTime >= 333) {
    lastTxTime = currentTime; // Reset the interval anchor

    uint16_t display_matrix[4] = {0, 0, 0, 0};

    // ── ALL BITS ON / OFF TOGGLE ──
    static bool all_bits_on = false;
    
    if (all_bits_on) {
      display_matrix[0] = 0x7FFF; 
      display_matrix[1] = 0x7FFF;
      display_matrix[2] = 0x7FFF;
      display_matrix[3] = 0x7FFF;
      buzzer_active = !buzzer_active;
    } else {
      display_matrix[0] = 0x0000;
      display_matrix[1] = 0x0000;
      display_matrix[2] = 0x0000;
      display_matrix[3] = 0x0000;
      buzzer_active = false;
    }

    // Toggle state logic (Locks perfectly to your 333ms intervals now)
    if (loopCounter < 10 && all_bits_on)  loopCounter++;
    if (loopCounter >= 10 && all_bits_on) { all_bits_on = !all_bits_on; loopCounter = 0; }
    if (loopCounter < 3 && !all_bits_on)   loopCounter++;
    if (loopCounter >= 3 && !all_bits_on)  { all_bits_on = !all_bits_on; loopCounter = 0; }
    
    // Pack buzzer state into bit 15 of COM0's high byte
    if (buzzer_active) display_matrix[0] |= (1 << 15);

    // Heartbeat LED On
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

    // Payload: unpack each COM row into 2 bytes (high byte first)
    tx_packet[4]  = (display_matrix[0] >> 8) & 0xFF;
    tx_packet[5]  =  display_matrix[0]       & 0xFF;
    tx_packet[6]  = (display_matrix[1] >> 8) & 0xFF;
    tx_packet[7]  =  display_matrix[1]       & 0xFF;
    tx_packet[8]  = (display_matrix[2] >> 8) & 0xFF;
    tx_packet[9]  =  display_matrix[2]       & 0xFF;
    tx_packet[10] = (display_matrix[3] >> 8) & 0xFF;
    tx_packet[11] =  display_matrix[3]       & 0xFF;

    // CRC-8 over 10 bytes
    tx_packet[12] = calculate_crc8(&tx_packet[2], 10);

    // Blast instantly to HC-12 at 9600 baud (Takes ~13.5ms to push down the wire)
    Serial1.write(tx_packet, 13);
    Serial1.flush();

    Serial.println("[SYSTEM] Sent Bits...");

    // Heartbeat LED off
    if (led_active) {
      rgb_led.setPixelColor(0, rgb_led.Color(0, 0, 0));
      rgb_led.show();
      if (heartbeat_count >= 10) led_active = false;
    }
  }
}