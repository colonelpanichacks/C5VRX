#!/usr/bin/env bash
# =============================================================================
# flash.sh — build + FULL-ERASE + verified flash for the ELRS sniffer.
#
#   ./flash.sh [env] [port]        env defaults to lilygo-t3s3-sx1280
#
# Why not `pio run -t upload`: PlatformIO's Arduino upload writes ONLY
# firmware.bin @ 0x10000, assuming a bootloader already exists. After the
# required FULL ERASE that leaves an unbootable board, so this script writes
# the complete canonical image set at explicit offsets (see flash.md).
# =============================================================================
set -euo pipefail
cd "$(dirname "$0")"

ENV_NAME="${1:-lilygo-t3s3-sx1280}"
PORT="${2:-}"
BUILD_DIR=".pio/build/$ENV_NAME"

# --- esptool resolution (any of: PATH, python module, PlatformIO package) ---
find_esptool() {
    if command -v esptool.py >/dev/null 2>&1; then echo "esptool.py"; return; fi
    if python3 -m esptool version >/dev/null 2>&1; then echo "python3 -m esptool"; return; fi
    local pio_esptool="$HOME/.platformio/packages/tool-esptoolpy/esptool.py"
    if [ -f "$pio_esptool" ]; then echo "python3 $pio_esptool"; return; fi
    echo "ERROR: esptool not found (tried esptool.py, python3 -m esptool, $pio_esptool)" >&2
    echo "       -> pip install esptool   (or: pip install platformio)" >&2
    exit 1
}

# --- port auto-detect ---
detect_port() {
    local ports=()
    for p in /dev/cu.usbmodem* /dev/cu.wchusbserial* /dev/cu.usbserial* \
             /dev/cu.SLAB_USBtoUART* /dev/ttyACM* /dev/ttyUSB*; do
        [ -e "$p" ] && ports+=("$p")
    done
    case ${#ports[@]} in
        0) return 1 ;;
        1) echo "${ports[0]}"; return 0 ;;
        *)
            echo "Multiple serial ports found:" >&2
            local i=1
            for p in "${ports[@]}"; do echo "  $i) $p" >&2; done
            printf "Select port [1]: " >&2
            local choice; read -r choice
            choice="${choice:-1}"
            echo "${ports[$((choice-1))]}"; return 0 ;;
    esac
}

# --- step 1: build ---
echo "==> [1/5] Building $ENV_NAME"
if command -v pio >/dev/null 2>&1; then
    pio run -e "$ENV_NAME"
else
    python3 -m platformio run -e "$ENV_NAME"
fi

# image sanity
BOOTLOADER="$BUILD_DIR/bootloader.bin"
PARTITIONS="$BUILD_DIR/partitions.bin"
FIRMWARE="$BUILD_DIR/firmware.bin"
BOOTAPP0="$HOME/.platformio/packages/framework-arduinoespressif32/tools/partitions/boot_app0.bin"
for f in "$BOOTLOADER" "$PARTITIONS" "$FIRMWARE" "$BOOTAPP0"; do
    if [ ! -f "$f" ]; then
        echo "ERROR: missing build image: $f (build failed?)" >&2
        exit 1
    fi
done

# --- step 2: port ---
echo "==> [2/5] Finding serial port"
if [ -z "$PORT" ]; then
    if ! PORT="$(detect_port)"; then
        cat >&2 <<'EOF'
ERROR: no serial port found.
  The T3-S3 enumerates as /dev/cu.usbmodem* (USB VID:PID 303A:1001). If the
  board is not showing up at all:
    * use a DATA-capable USB-C cable (charge-only cables are the #1 cause)
    * macOS needs no driver; Linux: ensure user is in dialout group
    * force download mode: hold BOOT (GPIO0), tap RST, release BOOT
    * check `ls /dev/cu.usbmodem*` / `pio device list` again
EOF
        exit 1
    fi
fi
echo "    port: $PORT"

ESPTOOL="$(find_esptool)"

# --- step 3: FULL ERASE (required; incremental flashes leave stale state) ---
echo "==> [3/5] FULL FLASH ERASE ($ESPTOOL)"
# shellcheck disable=SC2086
$ESPTOOL --chip esp32s3 -p "$PORT" -b 460800 erase_flash

# --- step 4: write the full canonical image set (offsets in flash.md) ---
echo "==> [4/5] Writing images"
# shellcheck disable=SC2086
$ESPTOOL --chip esp32s3 -p "$PORT" -b 460800 \
    --before default-reset --after hard-reset \
    write-flash -z \
    --flash-mode dio --flash-freq 80m --flash-size 4MB \
    0x0     "$BOOTLOADER" \
    0x8000  "$PARTITIONS" \
    0xe000  "$BOOTAPP0" \
    0x10000 "$FIRMWARE"

# --- step 5: verify ---
echo "==> [5/5] Verifying"
# shellcheck disable=SC2086
$ESPTOOL --chip esp32s3 -p "$PORT" -b 460800 verify_flash \
    0x0     "$BOOTLOADER" || true   # some esptool builds verify one image per call
# shellcheck disable=SC2086
$ESPTOOL --chip esp32s3 -p "$PORT" -b 460800 verify_flash \
    0x10000 "$FIRMWARE"

echo "DONE. Attach a serial monitor for the JSON stream:"
echo "    pio device monitor -b 460800 -p $PORT"
