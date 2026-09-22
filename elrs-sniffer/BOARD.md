# BOARD.md — hardware pinning: DONE (LilyGo T3-S3 SX1280)

**Status: pinned.** The unit is a **LilyGo T3-S3, ESP32-S3, SX1280 2.4 GHz
variant** with the 0.96" SSD1306 OLED. All pins below are **[VERIFIED]**
against the manufacturer's own source:
`Xinyuan-LilyGO/LilyGo-LoRa-Series` → `examples/T3S3Factory/utilities.h`
(T3_S3_V1_2_SX1280 / T3_S3_V1_2_SX1280_PA blocks) and
`examples/LoRa/T3S3/SX1280PA_PingPong/SX1280PA_PingPong.ino` (RF-switch
pins), fetched 2026-09-22. Encoded in `src/board_pins.h`.

## Verified pinout

| Function | GPIO | Source |
|---|---|---|
| SX1280 NSS | 7 | RADIO_CS_PIN |
| SX1280 SCK | 5 | RADIO_SCLK_PIN |
| SX1280 MISO | 3 | RADIO_MISO_PIN |
| SX1280 MOSI | 6 | RADIO_MOSI_PIN |
| SX1280 RST | 8 | RADIO_RST_PIN |
| SX1280 DIO1 (IRQ) | 9 | RADIO_DIO1_PIN |
| SX1280 BUSY | 36 | RADIO_BUSY_PIN |
| PA antenna switch RX enable | 21 | RADIO_RX_PIN — **PA variant only** |
| PA antenna switch TX enable | 10 | RADIO_TX_PIN — **PA variant only** |
| OLED SDA (SSD1306 @ 0x3C) | 18 | I2C_SDA, factory sketch uses addr 0x3c |
| OLED SCL | 17 | I2C_SCL |
| Onboard LED (HIGH = on) | 37 | BOARD_LED — lock indicator |
| VBAT divider | 1 | ADC_PIN (100k/100k) |
| BOOT button | 0 | BUTTON_PIN |

## Non-PA vs PA

The common "T3-S3 2.4G **without PA**" has no antenna switch — the
RXEN/TXEN pins don't exist; `board_pins.h` sets them to -1 and nothing is
driven. The **PA** variant needs its switch enabled or it is deaf:
add `-D T3S3_SX1280_PA` to `build_flags` (there is a marked spot in
`platformio.ini`); the firmware then calls RadioLib `setRfSwitchPins(21, 10)`
— the exact call from LilyGo's own SX1280PA example.

If the OLED shows column junk instead of text, the panel is an SH1106:
swap the constructor in `src/ui.cpp` (`U8G2_SSD1306_...` →
`U8G2_SH1106_128X64_NONAME_F_HW_I2C`) — one line.

## Flashing

Use `./flash.sh` — full erase + all images at explicit offsets + verify.
Details and the offsets table: [flash.md](flash.md). If the board doesn't
enumerate as `/dev/cu.usbmodem*`, see the port-troubleshooting section
there (cable/driver/download-mode).

## Legality

Passive receive only. This firmware never calls a transmit API. Receiving
in the license-free 2.4 GHz ISM band is broadly permitted; *using* decoded
third-party telemetry may be restricted where you operate (privacy,
direction-finding, interception laws). You are responsible for compliance.
The UI honesty note applies in the field too: "link fingerprint" is not
"identified pilot".
