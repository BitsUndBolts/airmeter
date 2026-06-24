# AirMeter — ANENG AN870 Wireless Monitor

## Architecture
RP2040 Zero → HC-12 (433MHz, 9600 baud) → ESP32-C3 SuperMini → Browser (SSE)

## Hardware
- RP2040 reads DTM0660 LCD via 5× LM339LVPWR comparators (4 COM + 15 SEG lines)
- ESP32 hosts web UI from LittleFS, bridges RF to browser via Server-Sent Events
- HC-12 is bidirectional: RP2040 transmits LCD frames, ESP32 sends config commands back
- HC-12 config: 9600 baud, channel C003 (434.6 MHz), 5 dBm, FU1 mode

## Active files
ESP32_C3_SuperMini.ino   — ESP32 firmware (Wi-Fi, SSE, HC-12 RX, OTA, web server)
RP2040_Zero.ino          — RP2040 firmware (LCD sampling, HC-12 TX, sleep detection)
ESP32_C3_SuperMini/data/ — Web UI served from LittleFS
  airmeter.js/.css       — shared utilities and design system, used by all pages

## Key constraints
- ESP32-C3 CPU: 80 or 160 MHz only (160 max, 80 min for Wi-Fi stability)
- Meter channel 0–31 (5 bits), FPS index 0–4 maps to [1, 2, 3, 5, 10] Hz
- Packet formats must stay in sync across both .ino files:
    TX (RP2040→ESP32): 0x55 0xAA LEN SEQ META COM0×2 COM1×2 COM2×2 COM3×2 CRC8
    RX (ESP32→RP2040): 0xAA 0x55 0x03 OLD_CH DATA_BYTE FLAGS CRC8
    CRC8: Dallas/Maxim polynomial 0x31

## Ignore
ANENG_AN870/ PCB/ Screens/ *.pdf *.docx LICENSE README.md