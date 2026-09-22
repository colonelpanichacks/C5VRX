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
| `firmware.bin`   | **0x10000** | app offset from parsing `default_8MB.csv` (`ESP32_APP_OFFSET`; also the platform's merged-image layout) |

Flash params: `--flash-mode qio --flash-freq 80m --flash-size 8MB` — taken
from the `esp32-s3-devkitc-1` board manifest (`build.flash_mode=qio`,
`f_flash=80MHz`, `upload.flash_size=8MB`).

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

If the board is the SX1280 **PA** variant, build with `-D T3S3_SX1280_PA`
(add it to `build_flags` in `platformio.ini` or in `common.build_flags`
before running flash.sh) so the antenna-switch enables (RX=21/TX=10) are
driven. See `src/board_pins.h`.
