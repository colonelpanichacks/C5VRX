#!/usr/bin/env bash
# SPDX-License-Identifier: GPL-3.0-only
set -euo pipefail

if [[ -z "${IDF_PATH:-}" ]]; then
  echo "IDF_PATH is not set. Run the ESP-IDF export script first." >&2
  exit 1
fi

LIBDIR="$IDF_PATH/components/esp_phy/lib/esp32c5"
NM="$(command -v riscv32-esp-elf-nm || true)"
if [[ -z "$NM" ]]; then
  echo "riscv32-esp-elf-nm not found. Source the ESP-IDF environment." >&2
  exit 1
fi

for lib in "$LIBDIR/librftest.a" "$LIBDIR/libphy.a"; do
  echo "==== $lib ===="
  "$NM" -A -C --defined-only "$lib" | \
    grep -Ei 'dump|fedump|adc|iq|sample|rxmem|rx_buffer|rx_data|fft|pbus.*freq|chan.*freq' || true
  echo
done
