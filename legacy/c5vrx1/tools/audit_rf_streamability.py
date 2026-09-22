#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-only
"""Reproducible static audit of the ESP32-C5 RF dump/stream boundary."""

from __future__ import annotations

import argparse
import os
from pathlib import Path
import re
import shutil
import subprocess


def run(*argv: str) -> str:
    return subprocess.run(argv, check=True, text=True,
                          stdout=subprocess.PIPE).stdout


def section(disassembly: str, name: str) -> str:
    match = re.search(
        rf"Disassembly of section \.text\.{re.escape(name)}:.*?"
        rf"(?=\nDisassembly of section|\Z)", disassembly, re.S)
    if not match:
        raise SystemExit(f"missing {name} disassembly")
    return match.group(0)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--idf", default=os.environ.get("IDF_PATH"))
    parser.add_argument("--out", default="rf-streamability-audit.md")
    args = parser.parse_args()
    if not args.idf:
        raise SystemExit("IDF_PATH is unset; pass --idf")

    objdump = shutil.which("riscv32-esp-elf-objdump")
    nm = shutil.which("riscv32-esp-elf-nm")
    if not objdump or not nm:
        raise SystemExit("ESP RISC-V binutils not found; source export.sh")

    root = Path(args.idf)
    archive = root / "components/esp_phy/lib/esp32c5/librftest.a"
    gdma = root / "components/esp_hal_dma/esp32c5/include/hal/gdma_channel.h"
    attach = root / "components/esp_hal_dma/esp32c5/include/hal/bitscrambler_peri_select.h"
    hp_system = root / "components/soc/esp32c5/register/soc/hp_system_reg.h"
    modem_clock = root / "components/soc/esp32c5/include/modem/modem_syscon_reg.h"
    if not all(p.exists() for p in (archive, gdma, attach, hp_system, modem_clock)):
        raise SystemExit("required ESP32-C5 IDF artifacts are missing")

    dis = run(objdump, "-dr", "-C", str(archive))
    symbols = run(nm, "-A", "-S", str(archive))
    adc = section(dis, "adctrig")
    dump_mode = section(dis, "set_dump_mode")
    gdma_text = gdma.read_text()
    attach_text = attach.read_text()
    hp_system_text = hp_system.read_text()
    modem_clock_text = modem_clock.read_text()

    facts = {
        "packed IQ producer function exists": bool(re.search(r"\bT adctrig$", symbols, re.M)),
        "finite dump base 0x40830000 is materialized": "408306b7" in adc,
        "reported dump size is 0x10000 bytes": "lui\ta4,0x10" in adc,
        "hardware write pointer is read at 0x600a9008":
            bool(re.search(r"lui\s+\w+,0x600a9", adc) and
                 re.search(r"lw\s+\w+,8\(\w+\)", adc)),
        "completion bit 18 is polled at 0x600a9004": "600a9004" in adc and "0x40" in adc,
        "vendor wrapper has a 1,000,000 us timeout": "f4240" in adc,
        "vendor wrapper disables capture before return": "lui\ta3,0x80000" in adc and "not\ta3,a3" in adc,
        "adctrig has no GDMA call relocation": "GDMA" not in adc.upper(),
        "public C5 GDMA trigger list has no Wi-Fi/FE source": not re.search(r"WIFI|MODEM|\bFE\b", gdma_text),
        "public C5 BitScrambler attach list has no Wi-Fi/FE source": not re.search(r"WIFI|MODEM|\bFE\b", attach_text),
        "dump mode is an FE/AGC register configuration": "600a08cc" in dump_mode and "600a70b8" in dump_mode,
        "0x60095004 is named HP SRAM usage, not RX clock":
            "HP_SYSTEM_SRAM_USAGE_CONF_REG" in hp_system_text and
            "HP_SYSTEM_MAC_DUMP_ALLOC" in hp_system_text,
        "C5 public header names a data-dump clock mux":
            "MODEM_SYSCON_CLK_DATA_DUMP_MUX" in modem_clock_text,
    }

    passed = all(facts.values())
    lines = [
        "# ESP32-C5 RF streamability static audit", "",
        f"Archive: `{archive}`", "",
        "| Static fact | Result |", "|---|---|",
    ]
    lines += [f"| {name} | {'yes' if value else 'NO'} |"
              for name, value in facts.items()]
    lines += ["", "## Verdict", "",
              "The blob contains a hardware-driven circular/pre-trigger dump candidate, "
              "but the shipped wrapper is finite and tears the engine down. No public "
              "GDMA or BitScrambler attachment exposes the Wi-Fi RF/FE producer. Static "
              "evidence alone therefore does not establish a sustainable application "
              "sample stream. A guarded ring reader is now implemented but remains "
              "`EXPERIMENTAL_RING_SOURCE_UNPROVEN` until the on-device cadence, wrap, "
              "phase and contention probes pass; repeated finite captures are never "
              "labeled continuous.", ""]
    Path(args.out).write_text("\n".join(lines))
    print(f"RF streamability audit {'PASS' if passed else 'INCOMPLETE'}: {args.out}")
    return 0 if passed else 1


if __name__ == "__main__":
    raise SystemExit(main())
