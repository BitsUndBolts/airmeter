<div align="center">

<img src="AirMeter.png" alt="AirMeter Logo" width="220">

<br>
**Wireless LCD streaming for the ANENG AN870 multimeter — straight into OBS, your browser, or anywhere else you need live readings.**

</div>

---

## Table of Contents

- [What is this?](#what-is-this)
- [The Backstory](#the-backstory)
- [System Architecture](#system-architecture)
  - [1. The Probe & Segment Mapping](#1-the-probe--segment-mapping)
  - [2. The Custom PCB & Comparators](#2-the-custom-pcb--comparators)
  - [3. The Transmitter Node (RP2040 Zero)](#3-the-transmitter-node-rp2040-zero)
  - [4. Power Delivery](#4-power-delivery)
  - [5. The Receiver Node & Web UI (ESP32)](#5-the-receiver-node--web-ui-esp32)
- [Wireless Protocol (HC-12)](#wireless-protocol-hc-12)
  - [HC-12 Radio Configuration](#hc-12-radio-configuration)
  - [Meter → ESP32 (Telemetry Packet)](#meter--esp32-telemetry-packet)
  - [ESP32 → Meter (Config Packet)](#esp32--meter-config-packet)
  - [CRC-8 Details](#crc-8-details)
- [LCD Segment Map](#lcd-segment-map)
- [Web Interface](#web-interface)
- [Hardware You'll Need](#hardware-youll-need)
- [Repository Layout](#repository-layout)
- [Getting Started](#getting-started)
- [Roadmap / Ideas](#roadmap--ideas)
- [License](#license)

---

## What is this?

AirMeter is an open-source, wireless hardware modification kit that taps directly into the multiplexed LCD glass of an **ANENG AN870** multimeter, decodes every segment in real time, and streams the live reading over a low-power 433 MHz radio link to an ESP32 web server — which then re-renders the exact multimeter display in your browser (or as an OBS overlay) via Server-Sent Events.

No UART hack, no proprietary app, no Bluetooth pairing dance. Just probes on glass traces, a comparator board, and two cheap radio modules.

<p align="center">
  <img src="Screens/8 Amber.png" width="260" alt="AirMeter live view — Amber theme">
  &nbsp;&nbsp;
  <img src="Screens/13 Red Alert.png" width="260" alt="AirMeter live view — Red Alert theme">
</p>

## The Backstory

This started as a much smaller project: pull data off an AN870 multimeter and show it as an overlay in OBS for YouTube videos. The obvious path was to find the UART pins on the multimeter's settings EEPROM and tap them directly.

Except they don't exist — or rather, they're sealed under an epoxy blob with no way to reach them without destroying the board.

So the only data source left was the LCD itself: **19 physical traces** running to the glass. That's where the real rabbit hole started.

It turns out the AN870 doesn't drive each of its ~60 segments with its own wire. Instead it uses a **multiplexed LCD**: a 4×15 matrix (4 COM lines × 15 SEG lines = 60 controllable segments) where each trace is reused across many segments by toggling polarity in sequence.

Probing the glass with an oscilloscope revealed the timing: each COM phase holds for **2 ms** before the driver inverts polarity, which is what prevents DC bias from degrading the liquid crystal over time. A segment is "on" whenever there's a 3V differential between its COM and SEG line — and that 3V can show up as `COM:3V / SEG:0V` or the inverted `COM:0V / SEG:3V`, alternating every cycle.

That explains *how* the display works, but not *which* COM+SEG pair lights up *which* segment. Figuring that out needed an active probe rather than passive observation — which is where the ESP32 decoder script comes in (see [The Probe & Segment Mapping](#1-the-probe--segment-mapping) below).

Once every segment was mapped, the rest of the project was "just" wiring: a comparator board to turn the analog multiplexed wave into clean digital logic, a microcontroller to sample it, a radio link to get the data off the meter, and a web server to put it all somewhere useful.

<p align="center">
  <img src="Screens/Oscilloscope Readings.png" width="420" alt="Oscilloscope capture of multiplexed LCD signal">
</p>

## System Architecture

```
  ┌─────────────────┐      ┌──────────────────┐       ┌─────────────┐       ┌─────────────────┐      
  │ AN870 LCD glass │ ───▶│ LMV339 Comparator │ ───▶ │ RP2040 Zero │ ───▶ │   HC-12 (TX)    │ 
  │ (4 COM × 15 SEG)│      │  PCB (digitize)  │       │ (sample +   │       │  9600 / C003 /  │ 
  └─────────────────┘      └──────────────────┘       │  frame data)│       │  5dBm / FU1     │     
                                                      └─────────────┘       └─────────────────┘  

 ···▶   ···▶   ···▶    WIRELESS   ···▶   ···▶   ···▶

   ┌─────────────────┐       ┌────────────────────┐      ┌───────────────────┐
   │   HC-12 (RX)    │ ───▶ │  ESP32-C3 Bridge   │ ───▶ │  Browser / OBS UI │
   │    on ESP32     │       │ decode → SSE → Web │      │ (recreated LCD)   │
   └─────────────────┘       └────────────────────┘      └───────────────────┘

```

### 1. The Probe & Segment Mapping

Before any of the "real" hardware could be designed, every one of the 60 COM+SEG combinations had to be mapped to a physical symbol or digit segment on the AN870's screen.

[`ESP32-C3_SuperMini_Multiplexed_Screen_Decoder.txt`](./ESP32-C3_SuperMini_Multiplexed_Screen_Decoder.txt) is a tiny throwaway sketch flashed onto an ESP32-C3 SuperMini for exactly this purpose. It drives one COM probe pin and one SEG probe pin in alternating polarity at the same ~2 ms cadence the real driver uses, replicating the AN870's own multiplexing scheme:

```cpp
// Phase 1: 3.3V differential across the display glass
digitalWrite(COM_PROBE, HIGH);
digitalWrite(SEG_PROBE, LOW);
delay(2); // 2ms timing matching the AN870 scope capture

// Phase 2: Complete inversion to prevent DC screen burn
digitalWrite(COM_PROBE, LOW);
digitalWrite(SEG_PROBE, HIGH);
delay(2);
```

By manually moving the two probe wires across all 4 COM and 15 SEG traces and noting which symbol lit up each time, the full mapping in [`Screen Signal Decoding.txt`](./Screen_Signal_Decoding.txt) was built — the canonical reference for which bit corresponds to which symbol or digit segment (see [LCD Segment Map](#lcd-segment-map) below).

### 2. The Custom PCB & Comparators

Reading the multiplexed lines directly from a microcontroller GPIO doesn't work well: the signal swings through **0V, 1V, 2V, and 3V** rather than cleanly between two logic levels, and distinguishing "2V" from "3V" — the actual difference between segment-off and segment-on — is unreliable on a standard digital input.

<p align="center">
  <img src="Screens/PCB.jpg" width="380" alt="Custom comparator PCB">
</p>

The fix is a custom PCB built around **LMV339 comparators**, each referenced to fixed voltage thresholds (~0.5V and ~2.5V) so that the messy analog waveform on each trace gets cleanly digitized into a proper HIGH/LOW signal before it ever reaches a microcontroller pin.

### 3. The Transmitter Node (RP2040 Zero)

[`RP2040_Zero.ino`](./RP2040_Zero.ino) sits right on top of the comparator PCB and does the actual sampling and framing:

- Polls all 4 COM lines every loop iteration; when a COM line goes HIGH, it waits for the signal to settle to the centre of its ~2 ms window before sampling all 15 SEG lines, avoiding edge artifacts.
- Each COM row is **overwritten independently** rather than cleared every cycle, so a single missed multiplexing pass doesn't blank that row — it just keeps the last known-good state, which is the right behaviour for a slowly changing display.
- The buzzer line is handled specially: the DTM0660 drives the buzzer as an AC tone (not a steady DC level), so a single `digitalRead()` would flicker between true/false. A latch with an 80ms hold-off timer smooths this into a stable on/off flag.
- **Sleep detection**: if no COM activity is seen for 10 seconds (the meter's own auto-shutoff kicking in), the RP2040 stops transmitting over the HC-12 to save battery, and resumes automatically the moment COM activity returns.
- Transmits a framed binary packet over HC-12 at a configurable rate (1, 2, 3, 5, or 10 Hz — set from the web UI, persisted to EEPROM).
- Also listens for a reverse-direction config packet from the ESP32, letting the web UI remotely change the meter's **channel** (0–31) and **refresh rate** without touching the hardware.

### 4. Power Delivery

Since the project is designed to run on **2× AA batteries** (rechargeable 1.2V NiMH cells included), a 3.3V boost converter supplies a steady rail to every component on the transmitter side, regardless of how depleted the cells get.

### 5. The Receiver Node & Web UI (ESP32)

[`ESP32_C3_SuperMini.ino`](./ESP32_C3_SuperMini.ino) is the always-on bridge between the radio link and the network. Its responsibilities:

- **Dual Wi-Fi modes** — boots into a captive-portal Access Point (`AirMeter-Setup` / `airmeter123`) for first-time provisioning, then switches to client mode and is reachable at `airmeter.local` or its DHCP-assigned IP.
- **HC-12 packet decoding** — a non-blocking byte-level state machine parses the framed packets coming from the RP2040, validates them with CRC-8, and unpacks the 60-bit segment string plus buzzer flag and frame-rate index.
- **Server-Sent Events (SSE)** — every decoded frame is pushed immediately to connected browsers via `/events`, with a separate event name per meter channel (`meter-data-<channel>`) so each UI page only subscribes to the meter it cares about.
- **Multi-meter support** — up to **32 meters** (channels 0–31) can be configured, though only one transmits at a time per HC-12 frequency to avoid radios talking over each other. Meters register themselves automatically the first time a packet is seen on their channel.
- **Channel-change cool-off** — when a meter's channel is renamed via the UI, packets already in flight under the old channel number are deliberately dropped for a 5-second window so they don't get mistaken for a brand-new meter re-registering on the old channel.
- **Web UI hosting via LittleFS** — serves the dashboard, meter config page, live view, and file manager straight from flash.
- **OTA updates** — both the compiled ESP32 firmware (`.bin`) and the web UI files themselves can be uploaded and flashed entirely from the browser. No Arduino IDE required for day-to-day updates.
- **File manager** — full CRUD (upload, download, rename, delete) over anything stored on the ESP32's LittleFS partition, including custom multimeter face images (prefixed `mm_*.webp`) that show up automatically in the meter configuration dropdown.

## Wireless Protocol (HC-12)

### HC-12 Radio Configuration

| Setting | Value | Why |
|---|---|---|
| Baud rate | `9600` | Reliable at this range, plenty of headroom for a 14-byte frame |
| Channel | `C003` | Avoids the crowded default Channel 1 while staying within legal limits |
| Power | `5 dBm` | Not maximum power, but enough range while running off 2× AA batteries |
| Mode | `FU1` | Best balance of fast wake/response time vs. power saving |

[`Arduino_UNO_HT-12.ino`](./Arduino_UNO_HT-12.ino) is a small standalone utility (flashed to any spare Arduino Uno) used to program a pair of HC-12 modules into this exact configuration via AT commands, and to read back the current settings for verification.

### Meter → ESP32 (Telemetry Packet)

Sent continuously by the RP2040 at the configured frame rate. Total size: **14 bytes**.

| Offset | Field | Size | Description |
|---|---|---|---|
| 0 | `PREAMBLE_1` | 1 byte | `0x55` — sync marker |
| 1 | `PREAMBLE_2` | 1 byte | `0xAA` — sync marker |
| 2 | `LEN` | 1 byte | `0x09` — payload length (also acts as an implicit format tag) |
| 3 | `SEQ` | 1 byte | Rolling sequence number (`0x00`–`0xFF`), increments per packet |
| 4 | `META` | 1 byte | Bits 7–5: FPS index (0–7) · Bits 4–0: meter channel (0–31) |
| 5–6 | `COM0` | 2 bytes | 15 SEG bits for COM0 row. **Bit 15 of byte 5 is repurposed for the buzzer flag** |
| 7–8 | `COM1` | 2 bytes | 15 SEG bits for COM1 row |
| 9–10 | `COM2` | 2 bytes | 15 SEG bits for COM2 row |
| 11–12 | `COM3` | 2 bytes | 15 SEG bits for COM3 row |
| 13 | `CRC8` | 1 byte | CRC-8 (Dallas/Maxim) computed over bytes 2–12 (`LEN` through `COM3` low byte) |

The receiving ESP32 reconstructs a 60-character bit string (`COM0[0..14] + COM1[0..14] + COM2[0..14] + COM3[0..14]`) and ships it to the browser as JSON over SSE:

```json
{"lcd":"010100000000000...","buzzer":0,"fpsIdx":2}
```

### ESP32 → Meter (Config Packet)

Sent on-demand when a user changes a meter's channel or refresh rate in the web UI. Total size: **6 bytes**. Note the preamble bytes are intentionally flipped relative to the telemetry direction, so a receiver can immediately tell which direction a packet belongs to even before checking the length byte.

| Offset | Field | Size | Description |
|---|---|---|---|
| 0 | `PREAMBLE_1` | 1 byte | `0xAA` (flipped vs. telemetry direction) |
| 1 | `PREAMBLE_2` | 1 byte | `0x55` |
| 2 | `LEN` | 1 byte | `0x02` — payload length |
| 3 | `CHANNEL` | 1 byte | The **current** channel the target meter is listening on — acts as an address filter so only the intended meter applies the change |
| 4 | `DATA_BYTE` | 1 byte | Bits 7–5: new FPS index (0–7) · Bits 4–0: new channel (0–31) |
| 5 | `CRC8` | 1 byte | CRC-8 (Dallas/Maxim) over bytes 2–4 (`LEN`, `CHANNEL`, `DATA_BYTE`) |

The RP2040 only acts on a config packet if byte 3 matches its **current** channel; it then persists the new channel and FPS index to EEPROM and confirms with a 3-flash LED sequence.

### CRC-8 Details

Both directions use the same CRC-8 Dallas/Maxim polynomial (`x⁸ + x⁵ + x⁴ + 1`, i.e. `0x31`):

```cpp
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
```

## LCD Segment Map

The full bit-to-symbol mapping derived from the probing step above lives in [`Screen Signal Decoding.txt`](./Screen_Signal_Decoding.txt). Summary:

| COM row | SEG0 | SEG1 | SEG2 | SEG3 | SEG12 | SEG13 | SEG14 |
|---|---|---|---|---|---|---|---|
| **COM0** | m | Ω | BUZZER | HOLD | REL | APO | AUTO |
| **COM1** | F | Hz | MAX | DIODE | *(unused)* | NEGATIVE | DC |
| **COM2** | V | n (nano) | M (Mega) | °C | DP‑2 | Half‑digit "1" | MIN |
| **COM3** | A | µ (micro) | k (kilo) | °F | Square Wave | Battery | AC + TRUE RMS |

SEG4–SEG11 on every COM row map to the seven-segment elements (A–G) of the four numeric digits — see the source file for the exact per-digit breakdown.

## Web Interface

| Page | Purpose |
|---|---|
| [`index.html`](./index.html) | Dashboard — active meter cards, firmware/IP/Wi-Fi/storage status, restart & Wi-Fi reset controls |
| [`meterConfig.html`](./meterConfig.html) | Per-meter configuration — name, face image, channel, refresh rate |
| [`meter.html`](./meter.html) | Live recreated multimeter display for a single channel, themeable, zoomable, designed to be used directly as an OBS Browser Source |
| [`setup.html`](./setup.html) | First-boot Wi-Fi provisioning wizard (served while the ESP32 is in AP mode) |
| [`files.html`](./files.html) | File manager + OTA firmware/UI uploader |
| [`airmeter.css`](./airmeter.css) | Shared design system used across all pages |

The live meter view ([`meter.html`](./meter.html)) ships with a dozen+ built-in color themes (Classic, Amber, Red Alert, Neon Cyan, Glass Dark/Bright, and more), adjustable zoom, and per-meter persistence of both settings — making it drop straight into OBS as a transparent or colored overlay.

<p align="center">
  <img src="Screens/6 Meter Configuration.png" width="420" alt="Meter configuration screen">
</p>

## Hardware You'll Need

- 1× ANENG AN870 multimeter (or any multimeter sharing the same DTM0660-driven 4×15 multiplexed LCD)
- 1× RP2040 Zero
- 1× ESP32-C3 SuperMini (receiver side)
- 1× ESP32-C3 SuperMini (only needed temporarily, for the segment-mapping probe step)
- 2× HC-12 433 MHz wireless modules
- 5× LMV339 quad comparators (custom PCB)
- 1× Arduino Uno (or any AVR board) — only needed temporarily, to program the HC-12 modules
- 3.3V boost converter
- 2× AA battery holder (rechargeable NiMH recommended)

## Repository Layout

```
.
├── ESP32-C3_SuperMini_Multiplexed_Screen_Decoder.txt   # One-off segment-mapping probe firmware
├── Screen_Signal_Decoding.txt                          # COM/SEG → symbol mapping reference
├── RP2040_Zero.ino                                     # Transmitter firmware (meter side)
├── ESP32_C3_SuperMini.ino                              # Receiver + web server firmware
├── Arduino_UNO_HT-12.ino                                # HC-12 module configuration utility
├── index.html / meterConfig.html / meter.html /
│   setup.html / files.html                             # Web UI pages (served from LittleFS)
├── airmeter.css                                         # Shared UI styling
└── Screens/                                             # Logo, PCB photos, oscilloscope captures, screenshots
```

## Getting Started

1. **Map your display** (skip if you already trust the included segment map for your exact meter model): flash [`ESP32-C3_SuperMini_Multiplexed_Screen_Decoder.txt`](./ESP32-C3_SuperMini_Multiplexed_Screen_Decoder.txt) to a spare ESP32-C3, probe COM/SEG trace pairs on the LCD glass, and record which symbol lights up for each combination.
2. **Build the comparator PCB** using the LMV339 reference design, tuned so its two thresholds sit cleanly between the 0V/1V/2V/3V levels your oscilloscope shows on the glass traces.
3. **Configure the HC-12 modules** to `9600 / C003 / 5dBm / FU1` using [`Arduino_UNO_HT-12.ino`](./Arduino_UNO_HT-12.ino) — do this for both modules before wiring them into the final circuit.
4. **Flash the RP2040 Zero** with [`RP2040_Zero.ino`](./RP2040_Zero.ino) and wire it to the comparator PCB outputs per the pin map at the top of the file.
5. **Flash the ESP32-C3 SuperMini** (receiver) with [`ESP32_C3_SuperMini.ino`](./ESP32_C3_SuperMini.ino).
6. **Upload the web UI** — either flash LittleFS with the HTML/CSS files directly, or boot the ESP32, connect to the `AirMeter-Setup` access point, and use the file manager at `/files` once on your network.
7. **Join your Wi-Fi** through the setup wizard, then open `http://airmeter.local` (or the assigned IP) to see your meter on the dashboard.
8. **Add the live view as an OBS Browser Source**: point it at `http://airmeter.local/meter?channel=<your channel>`.

## Roadmap / Ideas

- Additional meter model support (different COM×SEG geometries)
- Optional MQTT/Home Assistant bridge alongside SSE
- Battery level reporting from the RP2040 side

## License

*(Add your preferred license here — e.g. MIT)*
