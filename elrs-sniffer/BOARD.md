# BOARD.md — pin the exact hardware before flashing

The sniffer firmware is written; the **radio pinout is not yet pinned to a
physical unit**. This file says exactly what is verified, what is guessed, and
what information closes the gap.

## Why this file exists

The goal hardware is "a LilyGo SX1280 (2.4 GHz) board with onboard display".
Three families match parts of that sentence, and they are **not**
interchangeable:

| Candidate | MCU | Display | 2.4 GHz radio |
|---|---|---|---|
| **LilyGo T-Embed** (Konrad's ecosystem — KonradIT/C5VRX `tembed-link`) | ESP32-S3 | ST7789V 170×320 SPI ✅ | ❌ **stock is a sub-GHz CC1101**, not SX1280 |
| **LilyGo T3S3 2.4G** | ESP32-S3 | 0.96" SSD1306 I2C OLED | ✅ SX1280 (no-PA or PA variant) |
| T-Beam Supreme / others | ESP32-S3 | various | SX1280 or LR1121 — check silkscreen |

So the display and the SX1280 almost certainly come from **different board
families** unless the T-Embed in question is a modded/newer unit.

## What IS verified

- **T-Embed display pinout** — taken from the manufacturer's own factory
  sketch (`Xinyuan-LilyGO/T-Embed`, `examples/factory/pin_config.h`):
  `CS=10 DC=13 CLK=12 MOSI=11 RES=9 BL=15`, power-hold `PIN_POWER_ON=46`
  (must stay HIGH), panel 320×170 in the factory's landscape setup.
  Pre-filled in `src/board_pins.h` under `BOARD_LILYGO_T_EMBED_S3`.
- **T-Embed stock radio = CC1101** (`RADIO_CS_PIN=17` in the same file) —
  sub-GHz only. It cannot see ELRS 2.4 GHz, full stop.

## What is GUESSED (clearly marked in `src/board_pins.h`)

The SX1280 section under `BOARD_LILYGO_T_EMBED_S3` currently holds the
**T3S3-typical LilyGo wiring** (`NSS=7 SCK=5 MOSI=6 MISO=3 BUSY=13 DIO1=14
RST=12`, no TXEN/RXEN). Treat every one of those numbers as wrong until
proven.

## EXACTLY what I need from you

1. **The model silkscreen** on the actual unit (e.g. "T-Embed", "T3S3",
   revision numbers) — a photo of both sides is ideal.
2. **Which chip the 2.4 GHz antenna actually feeds** — if the board was
   modded or is a newer T-Embed variant with SX1280: the chip marking near
   the RF switch (SX1280 vs CC1101 vs SX1262 vs LR1121).
3. **Radio↔MCU wiring**, any of:
   - the schematic/README of the exact product page, or
   - continuity/probe readings: NSS, SCK, MOSI, MISO, BUSY, DIO1, RST,
     TXEN/RXEN (PA boards), and **which SPI peripheral** they hang off
     (on T-Embed the SD slot uses 38–41 and the CC1101 uses 17/43/44 — an
     add-on SX1280 often lands on one of those buses).
4. **Display orientation preference** — the T-Embed panel is natively
   170×320 portrait; if the image comes up shifted, note it (CGRAM offset
   fix is a one-liner in `board_pins.h`).
5. Whether the SX1280 module has a **PA** (power amplifier, needs TXEN/RXEN
   enables — fine for a receiver, but RXEN sometimes must be driven).

## Filling in `board_pins.h`

Once (3) is known, edit the `SX1280 radio [GUESS]` block under
`BOARD_LILYGO_T_EMBED_S3`. If the radio shares the display's SPI bus, delete
`SNIFFER_RADIO_HSPI` handling; if it sits on the SD bus, set
`PIN_LORA_{SCK,MOSI,MISO}` to 40/41/38 and pick a free CS. If the board turns
out to be a T3S3, switch the env to `BOARD_LILYGO_T3S3_2G4` and expect to
finish the SSD1306 UI stub (marked TODO in `src/ui.cpp`).

## Legal note

Passive receive only. This firmware never calls a transmit API. Check local
regs on *any* RF work regardless; receiving ISM telemetry is passive
eavesdropping on an unlicensed band, but publication/direction-finding use of
third parties' data may have its own rules where you operate.
