# BOARD.md — hardware pinning: DONE (LilyGo T3-S3 V1.2 SX1280)

**Status: pinned.** The unit is a **LilyGo T3-S3 V1.2, ESP32-S3R2**
(QFN56 rev v0.2: 4 MB embedded XMC flash + 2 MB embedded QSPI PSRAM,
esptool-identified), **SX1280 2.4 GHz "ranging" listing = PA variant**,
with the 0.96" SSD1306 OLED. Chip implications: QSPI-PSRAM-only (never an
`*_opi` SDK memory variant) and XMC flash → **dio** flash mode (known S3
QIO quirks) — both encoded in `platformio.ini`. All pins below are
**[VERIFIED]** against the manufacturer's own source:
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

The confirmed board ("ranging" listing) is the **PA** variant, so the
firmware now **defaults to `-D T3S3_SX1280_PA`** (in `platformio.ini`
`[common] build_flags`): the antenna-switch enables (RX=21/TX=10) are
driven via RadioLib `setRfSwitchPins()` — the exact call from LilyGo's own
SX1280PA example. For a non-PA board, remove that define and rebuild; the
boot banner and the `boot` JSON event state which variant the binary
contains, so a wrong guess is visible on serial + OLED instead of a
silently deaf front end. Note the chip has **4 MB embedded flash** —
`board_build.flash_size = 4MB` / `default.csv` are set in the env, and
`flash.sh` flashes with `--flash-size 4MB` (see flash.md).

If the OLED shows column junk instead of text, or stays dark while the
boot log says `oled:1`, the panel is an SH1106 shipped as "SSD1306" (or
vice versa) — the firmware **defaults to SH1106** and both drivers are
compiled in. To switch: send `D` on the serial console, or **hold the
BOOT button (GPIO0) for ≥1.5 s**; the splash names the active driver
(`OLED:SH1106` / `OLED:SSD1306`) for 1.5 s, the choice persists across
reboots, and a `{"t":"event","what":"oled_driver",...}` line is emitted.

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
