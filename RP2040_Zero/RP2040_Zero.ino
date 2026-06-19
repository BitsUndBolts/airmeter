/*
 * Copyright (c) 2026 Alexander Fuchs
 * Licensed under the MIT License.
 */
 
// =============================================================================
// RP2040 Zero — Multimeter LCD Monitor & Wireless Transmitter
// =============================================================================
//
// Overview:
//   Monitors the multiplexed LCD signals of a DTM0660-based multimeter
//   (ANENG AN870) by sampling COM and SEG lines via hardware comparators
//   (LMV339). Decoded display state is transmitted at a configurable rate
//   over a HC-12 433MHz wireless module using a framed binary protocol.
//
// LCD Multiplexing:
//   The DTM0660 drives a 4-COM × 15-SEG multiplexed LCD. Each COM line
//   is activated in sequence (COM0 → COM1 → COM2 → COM3) at roughly
//   2ms per phase. When a COM line is HIGH, the active SEG lines indicate
//   which segments are lit for that COM row.
//
//   Hardware comparators (LMV339) convert the LCD AC drive signals into
//   clean 3.3V logic levels before they reach the RP2040 GPIO pins.
//
// Sampling Strategy:
//   Each COM row is sampled whenever that COM goes HIGH, but rows are no
//   longer written into the live display matrix independently. The
//   DTM0660's displayed value can change asynchronously to our ~8ms
//   COM0..COM3 sweep (most visibly when the input is floating and the
//   reading is racing), so writing rows in one at a time risks stitching
//   segments from two different underlying values into one torn digit —
//   even though each individual row read was itself correct.
//
//   Instead, a full COM0..COM3 sweep is staged locally and validated for
//   timing continuity (a missed phase aborts that sweep). A validated
//   sweep is only committed into the live display matrix once it is
//   bit-for-bit identical to the immediately preceding validated sweep —
//   i.e. the source data has demonstrably settled rather than just been
//   caught mid-transition. See SECTION 5b. A sweep that never stabilises
//   simply leaves the previous known-good matrix in place, which remains
//   the correct behaviour for a slowly-changing display.
//
// Sleep Detection:
//   The multimeter enters a low-power sleep mode after a period of
//   inactivity. In sleep mode, the DTM0660 stops driving the COM and SEG
//   lines, so no new samples are captured. When no COM activity is detected
//   for SLEEP_TIMEOUT_MS milliseconds, the system enters sleep state:
//   HC-12 transmission is suspended. Normal operation resumes automatically
//   as soon as COM activity is detected again (e.g. the user wakes the meter).
//
// Transmission Protocol - Sending:
//   [ 0x55 ][ 0xAA ]          — 2-byte preamble (sync marker)
//   [ 0x09 ]                  — payload length (9 bytes); acts as implicit format tag
//   [ SEQ  ]                  — rolling sequence number (0x00–0xFF)
//   [ META ]                  — bits 7-5: fps index (0-7), bits 4-0: meter ID (0-31)
//   [ COM0_HI ][ COM0_LO ]    — 15 SEG bits for COM0 row (bit15 = buzzer)
//   [ COM1_HI ][ COM1_LO ]    — 15 SEG bits for COM1 row (bit15 = power conservation flag)
//   [ COM2_HI ][ COM2_LO ]    — 15 SEG bits for COM2 row
//   [ COM3_HI ][ COM3_LO ]    — 15 SEG bits for COM3 row
//   [ CRC8  ]                 — CRC-8 (Dallas/Maxim) over bytes 2-12 (LEN through COM3_LO)
//
// Transmission Protocol - Receiving:
//   [ 0xAA ][ 0x55 ]          — 2-byte preamble (sync marker)
//   [ 0x03 ]                  — payload length (3 bytes); acts as implicit format tag
//   [ CHANNEL ]               — Identifier for the RP2040 if this packet is meant for it
//   [ DATA_BYTE ]             — bits 7-5: fps index (0-7), bits 4-0: new channel (0-31)
//   [ FLAGS_BYTE ]            — bit 0: power conservation enable (1=on, 0=off); bits 7-1: reserved
//   [ CRC8  ]                 — CRC-8 (Dallas/Maxim) over bytes 2-5 (LEN through FLAGS_BYTE)
//
// Hardware:
//   MCU         : RP2040 Zero
//   Wireless    : HC-12 (9600 baud, FU1, 5dBm, channel C003)
//   Comparators : 5× LMV339 with resistor divider references
//   Target meter: ANENG AN870 (DTM0660 chip)
//
// =============================================================================

#include <Arduino.h>
#include <Adafruit_NeoPixel.h>
#include <EEPROM.h>
#include <string.h>  // memcpy / memcmp, used by frame-coherency staging (SECTION 5b)
#include "pico/stdlib.h"
#include "hardware/sync.h"
#include "hardware/structs/ioqspi.h"
#include "hardware/structs/sio.h"

// =============================================================================
// SECTION 1 — PIN DEFINITIONS
// =============================================================================

// COM lines from the DTM0660 LCD controller (via LMV339 comparator outputs).
// Each COM pin goes HIGH for ~2ms when that multiplexer phase is active.
// Array index matches COM number: COM_PINS[0] = COM0, etc.
const int COM_PINS[4] = { 28, 29, 26, 27 };  // COM0, COM1, COM2, COM3

// SEG lines from the DTM0660 LCD controller (via LMV339 comparator outputs).
// Array index matches SEG number: SEG_PINS[0] = SEG0, etc.
// A SEG pin reads HIGH when that segment is active during the current COM phase.
const int SEG_PINS[15] = {
   3,   // SEG0  → GP3
   0,   // SEG1  → GP0
   1,   // SEG2  → GP1
   2,   // SEG3  → GP2
  15,   // SEG4  → GP15
  14,   // SEG5  → GP14
  13,   // SEG6  → GP13
   4,   // SEG7  → GP4
   5,   // SEG8  → GP5
   7,   // SEG9  → GP7
   6,   // SEG10 → GP6
  11,   // SEG11 → GP11
  10,   // SEG12 → GP10
   8,   // SEG13 → GP8
   9,   // SEG14 → GP9
};

// Buzzer output from the DTM0660. Reads HIGH when the buzzer is active
// (continuity mode, diode test, etc.). Packed into bit 15 of COM0 payload.
const int BUZZER_PIN = 20;

// Onboard RGB LED (WS2812B) on GP16. Used as a heartbeat indicator during
// startup (first LED_HEARTBEAT transmissions) and to confirm frame rate
// changes via a flash sequence.
const int LED_PIN       = 16;
const int LED_HEARTBEAT = 10;   // Number of TX cycles to show heartbeat


// =============================================================================
// SECTION 2 — TRANSMISSION PROTOCOL CONSTANTS
// =============================================================================

// HC-12 UART interface — TX only
const int HC12_TX_PIN = 12;    // GP12 → HC-12 RXD (RP2040 TX → module RX)
const int HC12_RX_PIN = 17;    // GP17 → HC-12 TXD (RP2040 RX ← module TX)
const int HC12_BAUD   = 9600;

// Packet framing. Total packet size = 14 bytes.
// [ PREAMBLE×2 ][ LEN ][ SEQ ][ META ][ PAYLOAD×8 ][ CRC8 ]
const uint8_t PREAMBLE_1  = 0x55;
const uint8_t PREAMBLE_2  = 0xAA;
const uint8_t PAYLOAD_LEN_TX = 0x09;  // 9 payload bytes: 1 META + 4 COM rows × 2 bytes
const uint8_t PAYLOAD_LEN_RX = 0x03;  // 3 payload bytes: CHANNEL + DATA_BYTE + FLAGS_BYTE

// =============================================================================
// PACKET RECEIVE STATE MACHINE
// =============================================================================
enum RxState {
  WAIT_PREAMBLE_1,
  WAIT_PREAMBLE_2,
  WAIT_LEN,
  READ_PAYLOAD,
  WAIT_CRC
};

RxState rx_state = WAIT_PREAMBLE_1;
uint8_t rx_len = 0;
uint8_t rx_payload[PAYLOAD_LEN_RX];
uint8_t rx_payload_idx = 0;
unsigned long rx_error_count = 0;


// Meter protocol channel — loaded from EEPROM on boot, configurable at
// runtime via the ESP32 web UI. Defaults to 0 if no saved value exists.
// Valid range: 0-31 (5 bits, packed into the META byte low bits).
uint8_t CHANNEL = 0;


// COM line settle delay in microseconds. After a COM pin is detected HIGH,
// sampling is delayed to reach the centre of the active window (~2ms wide)
// before reading SEG lines. This avoids sampling during signal edges.
const unsigned int COM_SETTLE_US = 1000;

// Buzzer latch hold duration in milliseconds.
//
// The DTM0660 drives the buzzer as an AC tone (not a static DC HIGH), so the
// LMV339 output toggles rapidly between HIGH and LOW at the buzzer frequency.
// A single digitalRead() per loop iteration will frequently land on a LOW,
// causing the reported buzzer state to flicker even when the buzzer is truly on.
//
// To fix this, any HIGH sample sets a latch and arms a hold timer. The latch
// stays asserted until BUZZER_HOLD_MS milliseconds have elapsed with no further
// HIGH samples. This makes one real detection hold the active state across many
// subsequent LOW samples, while still clearing promptly when the buzzer stops.
//
// Tune this value to be:
//   — longer  than one buzzer PWM cycle  (typically ~0.5–1ms at ~1–2kHz)
//   — shorter than your fastest TX interval (currently 100ms at 10fps)
//
// 80ms is a safe default: it bridges any LOW gaps in the buzzer waveform
// without smearing the OFF→ON transition into the next transmission window.
const unsigned long BUZZER_HOLD_MS = 80;


// =============================================================================
// SECTION 3 — FRAME RATE CONFIGURATION
// =============================================================================

// Available transmission intervals in milliseconds, ordered from slowest to
// fastest.
//
//   Index 0 →  1 fps (1000ms)
//   Index 1 →  2 fps ( 500ms)
//   Index 2 →  3 fps ( 333ms)  ← default
//   Index 3 →  5 fps ( 200ms)
//   Index 4 → 10 fps ( 100ms)
const unsigned long FRAME_INTERVALS[] = { 1000, 500, 333, 200, 100 };
const int           FRAME_RATE_COUNT  = 5;
const int           FRAME_RATE_DEFAULT_INDEX = 2;   // 3 fps

// Compile-time guard: the META byte allocates 3 bits for the fps index, giving
// a maximum of 8 positions (0-7). Adding more than 8 entries to FRAME_INTERVALS
// without widening the field would silently corrupt the transmitted index.
static_assert(FRAME_RATE_COUNT <= 8,
  "FRAME_RATE_COUNT exceeds 8 — fps index field in META byte is only 3 bits wide");

// EEPROM address where the selected frame rate index is stored.
// A single byte is sufficient (values 0–4).
const int EEPROM_ADDR_FRAME_INDEX = 0;
const int EEPROM_MAGIC_ADDR       = 1;    // Address used to detect initialised EEPROM
const uint8_t EEPROM_MAGIC_VALUE  = 0xA5; // Sentinel — if present, EEPROM data is valid
const int EEPROM_ADDR_CHANNEL     = 2;
const int EEPROM_ADDR_FLAGS       = 3;    // Full FLAGS_BYTE (bit 0 = power conservation)

// Active frame rate index and interval. Set during setup and updated on button press.
int           frameRateIndex    = FRAME_RATE_DEFAULT_INDEX;
unsigned long txIntervalMs      = FRAME_INTERVALS[FRAME_RATE_DEFAULT_INDEX];

// =============================================================================
// SECTION 3b — POWER CONSERVATION
// =============================================================================
//
// When enabled, the RP2040 suppresses transmission of any packet whose LCD
// payload (display_matrix + buzzer bit) is bit-for-bit identical to the
// payload sent in the immediately preceding packet. This eliminates redundant
// RF traffic when the meter display is stable, saving power on both ends.
//
// The flag is stored as a full byte in EEPROM (EEPROM_ADDR_FLAGS, bit 0),
// leaving bits 7-1 available for future feature flags without a protocol
// change. The complete FLAGS_BYTE is echoed back to the ESP32 in each
// transmitted packet by setting bit 15 of the COM1 word, allowing the UI
// to reflect the currently active flag state.
//
// Important: suppression is based on the *outgoing* TX snapshot (after
// buzzer injection), not the raw display_matrix, so a buzzer state change
// alone is sufficient to force a packet even if the LCD digits are unchanged.

// FLAGS_BYTE bit mask for power conservation
const uint8_t FLAG_POWER_CONSERVATION = 0x01;  // bit 0

// Runtime FLAGS_BYTE — loaded from EEPROM on boot. Only bit 0 is currently
// used; the remaining 7 bits are preserved and echoed back to the ESP32 for
// forward compatibility.
uint8_t flags_byte = 0x00;

// Convenience accessor — true when power conservation is active.
inline bool powerConservationEnabled() { return (flags_byte & FLAG_POWER_CONSERVATION) != 0; }

// Last TX snapshot used for power-conservation suppression.
// Holds the full tx_matrix (including injected buzzer / flag bits) from the
// most recently transmitted packet. Initialised to all-zeros so the very
// first packet is always sent regardless of display content.
uint16_t last_tx_matrix[4] = { 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF }; // force first send


// =============================================================================
// SECTION 4 — SLEEP CONFIGURATION
// =============================================================================

// Duration of COM line inactivity (in milliseconds) before the system enters
// sleep mode. The DTM0660 stops driving COM/SEG lines when the multimeter
// enters its auto-shutoff low-power state. 10 seconds provides enough margin
// to distinguish genuine sleep from occasional missed samples.
const unsigned long SLEEP_TIMEOUT_MS = 10000;


// =============================================================================
// SECTION 5 — DISPLAY STATE
// =============================================================================

// Live display matrix. Each element holds the 15-bit SEG state for one COM row.
// Indexed by COM number: display_matrix[0] = COM0 row, etc.
// Updated only as a whole (all 4 rows at once) by handleCompletedSweep() in
// SECTION 9, and only once a full sweep has been validated for both timing
// continuity and stability against the previous sweep — see SECTION 5b.
// A sweep that isn't yet stable leaves this matrix untouched, so a still-
// settling reading simply holds the last known-good value rather than
// flickering or tearing.
uint16_t display_matrix[4] = { 0, 0, 0, 0 };

// Buzzer latch state.
//
// buzzer_active      — the latched output used at TX time. Set to true on any
//                      HIGH sample; cleared only after BUZZER_HOLD_MS of silence.
// buzzerLatchTime    — timestamp (millis) of the most recent HIGH buzzer sample.
//                      The latch expires when (now - buzzerLatchTime) > BUZZER_HOLD_MS.
bool          buzzer_active   = false;
unsigned long buzzerLatchTime = 0;


// =============================================================================
// SECTION 5b — FRAME COHERENCY STATE
// =============================================================================
//
// Problem: COM0..COM3 are sampled over the course of one mux sweep, but the
// DTM0660 can update the displayed value asynchronously to that sweep. If
// the value changes mid-sweep, rows captured before vs. after the change
// reflect two different "moments" — assembling them into one digit produces
// a torn / illegible glyph, even though every individual row read was
// itself a correct snapshot at the instant it was taken.
//
// Fix, in two parts:
//   1. Completeness check — collect 4 *distinct* COM rows, in whatever order
//      they actually arrive (no assumption about COM0→1→2→3 ordering, which
//      this code has no independent way to verify against the real wiring).
//      If the same COM index shows up again before all 4 distinct rows have
//      been seen, or if assembling the 4 rows takes longer than
//      SWEEP_MAX_DURATION_US, something was missed — discard and restart
//      the sweep from the row that just arrived.
//   2. Stability check — a complete sweep is only committed into the live
//      display_matrix if it is bit-for-bit identical to the immediately
//      preceding complete sweep. Two independently captured sweeps agreeing
//      is good evidence the source had actually settled, not just been
//      caught between transitions. A full sweep takes on the order of a few
//      ms and the fastest TX interval is 100ms, so there is ample headroom
//      to wait an extra sweep or two before committing.
// =============================================================================

// Generous ceiling on how long assembling one 4-row sweep may take before
// it's considered stale (e.g. due to a long gap after waking from sleep).
// The real mux cycle should be roughly 8ms; this is intentionally much
// looser than that so it never rejects a real sweep on a timing guess.
const unsigned long SWEEP_MAX_DURATION_US = 50000;

uint16_t      sweep_staging[4]        = { 0, 0, 0, 0 };          // rows captured so far in the in-progress sweep
bool          sweep_row_seen[4]       = { false, false, false, false }; // which COM indices have been captured this sweep
uint8_t       sweep_rows_captured     = 0;                        // how many distinct rows captured in the in-progress sweep
unsigned long sweep_start_us          = 0;                        // micros() timestamp of this sweep's first row

uint16_t      last_validated_sweep[4]    = { 0, 0, 0, 0 }; // most recent complete sweep
bool          have_last_validated_sweep  = false;          // false until at least one sweep has validated


// =============================================================================
// SECTION 6 — SLEEP STATE
// =============================================================================

// Timestamp of the last successfully captured COM sample. Used to detect
// when the multimeter has entered sleep mode (COM lines go inactive).
unsigned long lastActivityTime = 0;

// True when the system is in sleep mode. In sleep mode, HC-12 transmission
// is suspended. The system wakes automatically when COM activity resumes.
bool systemSleeping = false;


// =============================================================================
// SECTION 7 — PROTOCOL STATE
// =============================================================================

// Rolling sequence number incremented with each transmitted packet (0x00–0xFF).
// Allows the receiver to detect dropped or out-of-order packets.
uint8_t sequence_number = 0;

// Heartbeat LED state. Counts transmitted packets and disables the LED
// after LED_HEARTBEAT transmissions to save power.
int  heartbeat_count = 0;
bool led_active      = true;


// =============================================================================
// SECTION 8 — PERIPHERAL OBJECTS
// =============================================================================

Adafruit_NeoPixel rgb_led(1, LED_PIN, NEO_GRB + NEO_KHZ800);


// =============================================================================
// SECTION 9 — HELPER FUNCTIONS
// =============================================================================

uint8_t makeMetaByte(uint8_t fps_idx, uint8_t channel) {
  return ((fps_idx & 0x07) << 5) | (channel & 0x1F);
}

// Discard any in-progress sweep so the next captured row starts a fresh one.
// Called after a sweep completes (successfully or not) to keep sweeps
// non-overlapping and self-recovering.
void resetSweep() {
  sweep_row_seen[0] = sweep_row_seen[1] = sweep_row_seen[2] = sweep_row_seen[3] = false;
  sweep_rows_captured = 0;
}

// Called once a full 4-row sweep has passed the continuity check (see
// SECTION 5b). Commits it into the live display matrix only if it matches
// the previous continuity-valid sweep bit-for-bit; otherwise the previous
// known-good matrix is left untouched and we simply wait for the next
// sweep to (hopefully) agree.
void handleCompletedSweep() {
  bool matches_previous = have_last_validated_sweep &&
    (memcmp(sweep_staging, last_validated_sweep, sizeof(sweep_staging)) == 0);

  if (matches_previous) {
    memcpy(display_matrix, sweep_staging, sizeof(display_matrix));
  }

  memcpy(last_validated_sweep, sweep_staging, sizeof(last_validated_sweep));
  have_last_validated_sweep = true;
}

// Flash the LED a given number of times in blue to confirm a frame rate change.
// Each flash is 80ms on / 80ms off. Called after a button press is registered.
void flashLED(int times, int r, int g, int b) {
  for (int i = 0; i < times; i++) {
    rgb_led.setBrightness(20);
    rgb_led.setPixelColor(0, rgb_led.Color(r, g, b));
    rgb_led.show();
    delay(80);
    rgb_led.setPixelColor(0, rgb_led.Color(0, 0, 0));
    rgb_led.show();
    delay(80);
  }
}

// CRC-8 using the Dallas/Maxim polynomial (X^8 + X^5 + X^4 + 1 = 0x31).
// Applied over 'len' bytes starting at 'data'. Used to validate packet
// integrity on the receiver side.
uint8_t calculate_crc8(uint8_t *data, uint8_t len) {
  uint8_t crc = 0x00;
  for (uint8_t i = 0; i < len; i++) {
    crc ^= data[i];
    for (uint8_t j = 0; j < 8; j++) {
      crc = (crc & 0x80) ? (crc << 1) ^ 0x31 : (crc << 1);
    }
  }
  return crc;
}

void applyNewConfiguration(uint8_t new_channel, uint8_t new_fps_idx, uint8_t new_flags) {
  bool changed = false;

  if (new_channel != CHANNEL && new_channel <= 31) {
    CHANNEL = new_channel;
    EEPROM.write(EEPROM_ADDR_CHANNEL, CHANNEL);
    changed = true;
  }

  if (new_fps_idx < FRAME_RATE_COUNT && new_fps_idx != frameRateIndex) {
    frameRateIndex = new_fps_idx;
    txIntervalMs   = FRAME_INTERVALS[frameRateIndex];
    EEPROM.write(EEPROM_ADDR_FRAME_INDEX, frameRateIndex);
    changed = true;
  }

  if (new_flags != flags_byte) {
    flags_byte = new_flags;
    EEPROM.write(EEPROM_ADDR_FLAGS, flags_byte);
    changed = true;
  }

  if (changed) {
    EEPROM.write(EEPROM_MAGIC_ADDR, EEPROM_MAGIC_VALUE);
    EEPROM.commit();
    flashLED(3, 0, 0, 180);  // 3 flashes = config received confirmation
  }
}

void decodeConfigPacket(const uint8_t* payload) {
  const uint8_t current_channel = payload[0];
  if (current_channel != CHANNEL) return;

  // Extract shared fields: frequency (bits 7-5) | new_channel (bits 4-0)
  const uint8_t shared_byte = payload[1];
  const uint8_t frequency   = (shared_byte >> 5) & 0x07;
  const uint8_t new_channel = shared_byte & 0x1F;

  // Full FLAGS_BYTE — bit 0 is power conservation, bits 7-1 reserved for future use
  const uint8_t new_flags = payload[2];

  // Print debug confirmation over USB serial
  Serial.printf("[ESP32 -> RP2040] Current Channel: %u | New Channel: %u | Freq Index: %u | Flags: 0x%02X\n",
                current_channel, new_channel, frequency, new_flags);

  applyNewConfiguration(new_channel, frequency, new_flags);
}

void processIncomingESP32() {
  while (Serial1.available()) { // Drain HC-12 RX buffer
    const uint8_t b = Serial1.read();

    switch (rx_state) {

      case WAIT_PREAMBLE_1:
        // RX preamble is reversed: [0xAA][0x55], so first byte is PREAMBLE_2
        if (b == PREAMBLE_2) rx_state = WAIT_PREAMBLE_2;
        break;

      case WAIT_PREAMBLE_2:
        if (b == PREAMBLE_1) {
          rx_state = WAIT_LEN;
        } else if (b == PREAMBLE_2) {
          rx_state = WAIT_PREAMBLE_2; // Keep state if duplicate 0xAA arrives
        } else {
          rx_state = WAIT_PREAMBLE_1;
        }
        break;

      case WAIT_LEN:
        rx_len = b;
        if (rx_len == PAYLOAD_LEN_RX) {
          rx_payload_idx = 0;
          rx_state = READ_PAYLOAD;
        } else {
          Serial.printf("[HC12] Bad LEN byte from ESP32: 0x%02X — resyncing\n", b);
          rx_error_count++;
          rx_state = WAIT_PREAMBLE_1;
        }
        break;

      case READ_PAYLOAD:
        rx_payload[rx_payload_idx++] = b;
        if (rx_payload_idx == rx_len) {
          rx_state = WAIT_CRC;
        }
        break;

      case WAIT_CRC: {
        // CRC covers LEN + PAYLOAD (1 byte length + 3 bytes data = 4 bytes total)
        uint8_t crc_input[4];
        crc_input[0] = rx_len;
        memcpy(&crc_input[1], rx_payload, rx_len);

        const uint8_t expected_crc = calculate_crc8(crc_input, 4);

        if (b == expected_crc) {
          decodeConfigPacket(rx_payload);
        } else {
          rx_error_count++;
          Serial.printf("[HC12] CRC FAIL from ESP32 — got 0x%02X expected 0x%02X (Total: %lu)\n",
                        b, expected_crc, rx_error_count);
        }
        rx_state = WAIT_PREAMBLE_1;
        break;
      }
    }
  }
}

// =============================================================================
// SECTION 10 — SETUP
// =============================================================================

void setup() {
  // Initialise onboard RGB LED (off at startup)
  rgb_led.begin();
  flashLED(2, 0, 180, 0);

  // Configure COM and SEG pins as inputs with pull-ups.
  // The LMV339 comparator outputs are open-drain compatible; pull-ups ensure
  // a defined logic level when no COM or SEG line is being driven.
  for (int i = 0; i < 4;  i++) pinMode(COM_PINS[i], INPUT_PULLUP);
  for (int i = 0; i < 15; i++) pinMode(SEG_PINS[i], INPUT_PULLUP);
  pinMode(BUZZER_PIN, INPUT_PULLUP);

  // Initialise HC-12 UART
  Serial1.setTX(HC12_TX_PIN);
  Serial1.setRX(HC12_RX_PIN);
  Serial1.begin(HC12_BAUD);

  // Load saved frame rate index from EEPROM.
  // The magic byte at EEPROM_MAGIC_ADDR confirms the EEPROM has been written
  // by this firmware before. If absent, use the default frame rate index.
  EEPROM.begin(256);
  if (EEPROM.read(EEPROM_MAGIC_ADDR) == EEPROM_MAGIC_VALUE) {
    uint8_t savedFpsIdx = EEPROM.read(EEPROM_ADDR_FRAME_INDEX);
    uint8_t savedChannel = EEPROM.read(EEPROM_ADDR_CHANNEL);
    uint8_t savedFlags   = EEPROM.read(EEPROM_ADDR_FLAGS);
    
    if (savedFpsIdx < FRAME_RATE_COUNT) {
      frameRateIndex = savedFpsIdx;
      txIntervalMs   = FRAME_INTERVALS[frameRateIndex];
    }
    if (savedChannel <= 31) CHANNEL = savedChannel;
    flags_byte = savedFlags;  // Accept the full byte; all bits preserved
  }

  // Seed activity timer so the sleep timeout does not trigger immediately
  // on boot before any COM lines have been sampled.
  lastActivityTime = millis();
}

// =============================================================================
// SECTION 11 — MAIN LOOP
// =============================================================================

void loop() {
  static unsigned long lastTxTime = 0;
  unsigned long currentTime = millis();

  // ---------------------------------------------------------------------------
  // PHASE 1: Sample LCD mux signals
  //
  // Poll all four COM lines. When a COM line is found HIGH, wait for the signal
  // to settle to the midpoint of the active window, then confirm it is still
  // HIGH before reading all 15 SEG lines.
  //
  // Rows are NOT written into display_matrix directly. Instead each row is
  // staged into sweep_staging[] and checked for timing continuity against
  // the previous row (SECTION 5b). A full, continuity-valid 4-row sweep is
  // only committed into display_matrix once it matches the immediately
  // preceding continuity-valid sweep — this is what prevents a value change
  // mid-sweep from being stitched into a torn digit.
  //
  // Raw COM activity (not sweep validity) still drives the activity timer
  // and sleep wake-up, so an unstable/changing reading is never mistaken
  // for the meter going to sleep.
  // ---------------------------------------------------------------------------
  static bool com_prev_high[4] = { false, false, false, false };

  for (int c = 0; c < 4; c++) {
    bool comNowHigh = (digitalRead(COM_PINS[c]) == HIGH);

    // Only act on the rising edge (comNowHigh && previously LOW). loop()
    // runs far faster than the ~2ms COM window, so without this guard the
    // same window gets sampled many times while still HIGH, corrupting the
    // sweep-tracking state below with duplicate same-index captures.
    if (comNowHigh && !com_prev_high[c]) {
      delayMicroseconds(COM_SETTLE_US);           // Wait for signal centre

      if (digitalRead(COM_PINS[c]) == HIGH) {     // Confirm still active
        uint16_t row_bits = 0;
        for (int s = 0; s < 15; s++) {
          if (digitalRead(SEG_PINS[s]) == HIGH)
            row_bits |= (1 << s);
        }

        unsigned long captureTimeUs = micros();

        // Reset inactivity timer on every valid COM sample, regardless of
        // whether the sweep it belongs to ends up validated/committed.
        lastActivityTime = currentTime;

        // Wake from sleep if COM activity has resumed
        if (systemSleeping) {
          systemSleeping = false;
          Serial1.begin(HC12_BAUD);               // Re-enable HC-12 transmission
        }

        // --- Frame coherency: does this row fit the in-progress sweep? ---
        bool tooLongSinceSweepStart = sweep_rows_captured > 0 &&
          ((captureTimeUs - sweep_start_us) > SWEEP_MAX_DURATION_US);
        bool rowAlreadySeenThisSweep = sweep_row_seen[c];

        if (sweep_rows_captured == 0 || tooLongSinceSweepStart || rowAlreadySeenThisSweep) {
          // Starting fresh: either we were idle, the in-progress sweep has
          // been open too long (something was missed), or this exact COM
          // index already showed up earlier in the current sweep (meaning
          // we cycled back around without ever seeing one of the others).
          resetSweep();
          sweep_start_us = captureTimeUs;
        }

        sweep_staging[c]    = row_bits;
        sweep_row_seen[c]   = true;
        sweep_rows_captured++;

        if (sweep_rows_captured == 4) {
          handleCompletedSweep();
          resetSweep();                            // next COM starts a fresh sweep
        }
      }
    }

    com_prev_high[c] = comNowHigh;
  }

  // ---------------------------------------------------------------------------
  // Buzzer latch — sticky HIGH with configurable hold-off timer
  //
  // The DTM0660 drives the buzzer as an AC tone, so the LMV339 output toggles
  // between HIGH and LOW at the buzzer frequency (~1–2kHz). The loop runs at
  // roughly 250–1000 iterations/second, so a plain digitalRead() will frequently
  // land on a LOW half-cycle even when the buzzer is genuinely active.
  //
  // Instead of using the raw pin level, we maintain a latch:
  //   • Any HIGH sample sets buzzer_active = true and stamps buzzerLatchTime.
  //   • The latch is only cleared once BUZZER_HOLD_MS have elapsed since the
  //     last HIGH sample, ensuring brief LOW samples between buzzer pulses do
  //     not extinguish the active indication prematurely.
  // ---------------------------------------------------------------------------
  if (digitalRead(BUZZER_PIN) == HIGH) {
    buzzer_active   = true;
    buzzerLatchTime = currentTime;              // Arm / refresh the hold timer
  } else if (buzzer_active &&
             (currentTime - buzzerLatchTime >= BUZZER_HOLD_MS)) {
    buzzer_active = false;                     // Hold expired — buzzer is truly off
  }

  // ---------------------------------------------------------------------------
  // PHASE 2: Sleep detection
  //
  // If no COM activity has been detected for SLEEP_TIMEOUT_MS, the multimeter
  // has entered its auto-shutoff low-power state. Suspend HC-12 transmission
  // to conserve power. The system wakes automatically in Phase 1 when COM
  // lines become active again.
  // ---------------------------------------------------------------------------
  if (!systemSleeping && (currentTime - lastActivityTime >= SLEEP_TIMEOUT_MS)) {
    systemSleeping = true;
    Serial1.end();                                // Suspend HC-12 transmission
  }

  // ---------------------------------------------------------------------------
  // PHASE 3: Process incoming config packets from ESP32
  //
  // Receive and decode configuration commands sent by the ESP32 web UI over
  // the HC-12 reverse channel. Valid packets update CHANNEL and/or
  // frameRateIndex and persist both to EEPROM immediately.
  // ---------------------------------------------------------------------------
  processIncomingESP32();

  // ---------------------------------------------------------------------------
  // PHASE 4: Transmit packet every txIntervalMs (active mode only)
  //
  // Snapshot the current display matrix into a local TX buffer so the buzzer
  // bit and power-conservation flag can be packed into reserved bits without
  // modifying live state. Build and send a 14-byte framed packet over UART
  // to the HC-12 module. Transmission is skipped entirely while the system
  // is in sleep mode.
  //
  // Power conservation suppression:
  //   When power_conservation is enabled, the snapshot is compared against
  //   the snapshot transmitted in the immediately preceding packet (stored
  //   in last_tx_matrix, *after* all bit-packing). If they are identical,
  //   the packet is not sent, saving RF bandwidth when the meter display is
  //   stable. The comparison includes the buzzer bit (packed into COM0 bit15)
  //   and the power-conservation flag itself (COM1 bit15), so any change to
  //   either forces an immediate retransmit.
  //   The timer is still reset on a suppressed tick so the cadence is
  //   maintained: when the display eventually changes, the next tick fires
  //   at the configured interval rather than immediately.
  // ---------------------------------------------------------------------------
  if (!systemSleeping && (currentTime - lastTxTime >= txIntervalMs)) {
    lastTxTime = currentTime;

    // Snapshot display matrix into TX buffer (isolates live state from packing)
    uint16_t tx_matrix[4];
    memcpy(tx_matrix, display_matrix, sizeof(display_matrix));

    // Pack buzzer state into bit 15 of COM0 word (spare bit, not used by display)
    if (buzzer_active) tx_matrix[0] |= (1 << 15);

    // Pack power-conservation flag into bit 15 of COM1 word (spare bit).
    // The ESP32 extracts this to reflect the active flag state in the UI.
    if (powerConservationEnabled()) tx_matrix[1] |= (1 << 15);

    // Power conservation: suppress if the payload is unchanged since last TX
    bool payloadChanged = (memcmp(tx_matrix, last_tx_matrix, sizeof(tx_matrix)) != 0);

    if (powerConservationEnabled() && !payloadChanged) {
      // Nothing changed — skip transmission this tick
    } else {
      // Build 14-byte packet
      uint8_t tx_packet[14];

      tx_packet[0]  = PREAMBLE_1;                        // Sync byte 1
      tx_packet[1]  = PREAMBLE_2;                        // Sync byte 2
      tx_packet[2]  = PAYLOAD_LEN_TX;                    // Payload length
      tx_packet[3]  = sequence_number++;                 // Rolling sequence number

      // META byte: upper 3 bits = current fps index, lower 5 bits = CHANNEL.
      // Lets the receiver display the active frame rate and route packets from
      // multiple meters to separate UI panels without any extra protocol overhead.
      tx_packet[4]  = makeMetaByte(frameRateIndex, CHANNEL);    // [F2 F1 F0 | M4 M3 M2 M1 M0]

      tx_packet[5]  = (tx_matrix[0] >> 8) & 0xFF;       // COM0 high byte (bit15 = buzzer)
      tx_packet[6]  =  tx_matrix[0]       & 0xFF;        // COM0 low byte
      tx_packet[7]  = (tx_matrix[1] >> 8) & 0xFF;        // COM1 high byte (bit15 = power conservation flag)
      tx_packet[8]  =  tx_matrix[1]       & 0xFF;        // COM1 low byte
      tx_packet[9]  = (tx_matrix[2] >> 8) & 0xFF;        // COM2 high byte
      tx_packet[10] =  tx_matrix[2]       & 0xFF;        // COM2 low byte
      tx_packet[11] = (tx_matrix[3] >> 8) & 0xFF;        // COM3 high byte
      tx_packet[12] =  tx_matrix[3]       & 0xFF;        // COM3 low byte

      // CRC-8 over 11 bytes: LEN + SEQ + META + 8 COM bytes (indices 2-12)
      tx_packet[13] = calculate_crc8(&tx_packet[2], 11);

      Serial1.write(tx_packet, 14);

      // Record this snapshot for next-tick suppression comparison
      memcpy(last_tx_matrix, tx_matrix, sizeof(last_tx_matrix));

      // Heartbeat: flash LED for the first LED_HEARTBEAT transmissions
      if (led_active) {
        flashLED(1, 180, 0, 0);
        heartbeat_count++;
        if (heartbeat_count >= LED_HEARTBEAT) led_active = false;
      }
    }
  }
}