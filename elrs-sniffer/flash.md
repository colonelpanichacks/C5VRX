# flash.md — flashing the T3-S3 ELRS sniffer (FULL ERASE, always)

```bash
./flash.sh                 # build + erase + flash + verify (lilygo-t3s3-sx1280)
./flash.sh <env> [port]    # other env / explicit port
```

The script runs five steps: build → port auto-detect → **full `erase_flash`**
→ write all images at explicit offsets → verify. The full erase is a hard
requirement (carried over from the OUI-SPY project rule): incremental
flashes leave stale Arduino/PHY state that looks like firmware bugs.

## Exact flash offsets used by flash.sh

| image | offset | source / why |
|---|---|---|
| `bootloader.bin` | **0x0** | ESP32-S3 bootloader offset is 0x0 (verified in `~/.platformio/platforms/espressif32/builder/frameworks/espidf.py`: `"0x1000" if mcu in [esp32,esp32s2] else ("0x2000" if c5/p4 else "0x0")` → esp32s3 → 0x0) |
| `partitions.bin` | **0x8000** | standard partition-table offset (platform default `upload.partition_table_offset`) |
| `boot_app0.bin`  | **0xe000** | Arduino OTA-rollback marker; part of the canonical merged image (platform `esp32_create_combined_bin` layout) |
| `firmware.bin`   | **0x10000** | app offset from parsing the partition table (`ESP32_APP_OFFSET`; also the platform's merged-image layout) |

Flash params: `--flash-mode dio --flash-freq 80m --flash-size 4MB`.
- Size: the confirmed board is an **ESP32-S3R2** — 4 MB embedded XMC
  flash, 2 MB embedded QSPI PSRAM (esptool: "Embedded Flash 4MB (XMC),
  Embedded PSRAM 2MB (AP_3v3)").
- Mode **dio, not qio**: the XMC embedded flash has known QIO-mode quirks
  on ESP32-S3 (boot-stage flash access faults after the bootloader switches
  read mode). dio costs nothing at 80 MHz for this firmware. QIO can be
  re-tried later once the board is healthy. The PlatformIO env pins the
  same choice (`board_build.flash_mode = dio`) so the SDK libraries, image
  headers, and flash.sh all agree.
- PSRAM: `board_build.arduino.memory_type = dio_qspi` (S3R2 embedded PSRAM
  is **QSPI**; the framework derives `<flash_mode>_qspi` by default, so dio
  flash keeps the qspi PSRAM half — verified in
  `tools/platformio-build-esp32s3.py`). Do **not** use an `*_opi` variant:
  octal-PSRAM libs on a QSPI-only chip panic early in boot exactly like the
  observed reset loop.

> Why not `pio run -t upload`: PlatformIO's Arduino-framework upload writes
> **only** `firmware.bin @ 0x10000` (`UPLOADCMD='$UPLOADER $UPLOADERFLAGS
> $ESP32_APP_OFFSET $SOURCE'`, no `FLASH_EXTRA_IMAGES` under the Arduino
> framework). That assumes an existing bootloader — false after a full
> erase — so this script writes the complete set itself.

If PlatformIO is not on PATH, the script falls back to
`python3 -m platformio` for the build and `python3 -m esptool` (or the
PlatformIO-bundled `tool-esptoolpy`) for flashing.

## Port problems (the board wasn't enumerating at one point)

The T3-S3's USB-C is the **native USB** of the ESP32-S3 (VID:PID
`303A:1001`); a working unit appears as `/dev/cu.usbmodem*` (macOS),
`/dev/ttyACM*` (Linux). If nothing shows up:

1. **Cable first.** Charge-only USB-C cables are the most common cause —
   the board powers up (LED on GPIO37 may still light) but never enumerates.
2. **Driver:** none needed on macOS; on Linux add your user to `dialout`.
3. **Force download mode:** hold **BOOT** (GPIO0, the button), tap **RST**,
   release BOOT — the port should appear even if the firmware is bad.
4. `pio device list` (or `ls /dev/cu.usbmodem*` on macOS) to confirm.
5. After flashing, the JSON serial stream is on the **same USB port** at
   **460800 8N1** (native USB CDC — no UART adapter needed).

## PA-variant note

The confirmed board ("ranging" store listing) is the **PA** variant, so the
`t3s3` env defaults to `-D T3S3_SX1280_PA` (drives the antenna switch
RX=21/TX=10). For a non-PA board, remove that define from
`[common] build_flags` in `platformio.ini` and rebuild — the banner and the
`boot` JSON event state which variant the firmware was *built* for, so a
wrong guess is visible on both serial and OLED instead of silently deaf.

## Onboard LED (GPIO37) — boot & fault patterns, no serial needed

| phase | LED pattern | meaning |
|---|---|---|
| reset → ~1 s | **solid ON** | app started (2nd-stage bootloader handed off to our image) |
| ~1 s → radio init done | OFF, **200 ms blink** while waiting | radio init in progress |
| steady state, healthy, unlocked | **1 Hz heartbeat** (50 ms ON each second) | sweeping, no link |
| steady state, healthy, locked | **solid ON** | ELRS link locked |
| steady state, radio fault | **fast blink (~5 Hz)** forever | no SX1280 — see `radio:0` stats |

Boot-stage triage without any terminal:
- **Never lights**: wrong/reset-looping image at bootloader level, hardware
  power, or LED pin mismatch — re-check `verify_flash`, try download mode
  (BOOT+RST).
- **Lights 1 s then fast-blink**: app runs; SX1280 not answering → wrong
  pins or dead radio module.
- **Lights 1 s then heartbeat**: fully healthy; it is sweeping for ELRS.

## Boot observability (v0.2.2+): what you should see, where

The firmware prints a banner within ~1.5 s of reset, **before** any radio
or display init, and the 1 Hz `stats` JSON never stops (even with a dead
radio: `radio:0, pps:0, lock:0`). Failure signatures after a good flash:

| situation | USB serial | OLED |
|---|---|---|
| healthy | banner + `boot`/`ready` + 1 Hz stats | splash, then sweep UI |
| radio fault (no SX1280 ACK) | banner + `error{what:"radio_init",detail,pins}` + stats with `radio:0`, retried every 15 s (`radio_up` on recovery) | splash, then inverted **RADIO FAULT** banner with the detail line |
| OLED dead | banner + `error{what:"oled_init",detail}` + stats | (dark) |
| both dead | banner + both errors + 1 Hz stats | (dark) |
| truly silent (no banner) | — | — |

A truly silent board after this flash = still a boot-stage problem (bad
image, flash-mode mismatch, or hardware), not an app init stall: re-check
`verify_flash` output and try download mode (BOOT+RST).

## OLED driver toggle (SH1106 vs SSD1306)

LilyGo ships the 0.96" OLED interchangeably and sometimes mislabelled: an
SH1106 panel answers the I2C ACK probe fine but stays dark on the SSD1306
init sequence (and vice versa for the column-offset symptom). The firmware
compiles **both** U8G2 drivers and **defaults to SH1106**. If the panel is
dark or garbled after flashing:

- send `D` on the serial console (460800 baud), **or**
- **hold the BOOT button for ≥ 1.5 s** (edge-armed, one toggle per press).

Each toggle re-inits the other driver, shows a 1.5 s splash naming it in
the corner (`OLED:SH1106` / `OLED:SSD1306`), persists it in flash, and
prints `{"t":"event","what":"oled_driver","drv":"..."}`. The next boot
uses the persisted driver; `ready` reports it as `oled_drv`.

## Radio pin auto-probe

At boot the firmware probes a small table of known T3-S3 SX1280 pin sets
(5 s bound each, LED blinking while it works), logs each attempt as a
`probe` JSON line, stops at the first success, and caches the winner in
NVS so later boots skip the sweep (cached set tried first; if it fails,
the full table is swept again). Send `P` on the serial console to force a
full re-probe. If every set fails, the fault screen now also hints the
module may not be an SX1280-class chip at all — check the RF can marking
(the T3-S3 also ships in an LR1121 variant, which this firmware does not
drive).
