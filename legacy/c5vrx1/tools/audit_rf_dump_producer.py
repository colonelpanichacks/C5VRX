#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-only
"""Verify source reconstruction constants against the audited C5 machine code."""
from __future__ import annotations
import argparse, hashlib, json, pathlib, re, shutil, subprocess

EXPECTED = "0f9680b41612762d854accf2334412e3e01b206a3ec290bbbb232842fdaec7ba"

def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--idf", required=True)
    args = ap.parse_args()
    root = pathlib.Path(args.idf)
    archive = root / "components/esp_phy/lib/esp32c5/librftest.a"
    digest = hashlib.sha256(archive.read_bytes()).hexdigest()
    if digest != EXPECTED:
        raise SystemExit(f"refusing SHA-256 {digest}")
    objdump = shutil.which("riscv32-esp-elf-objdump")
    if not objdump:
        raise SystemExit("riscv32-esp-elf-objdump missing")
    dis = subprocess.run([objdump, "-dr", "-C", str(archive)], check=True,
                         text=True, stdout=subprocess.PIPE).stdout
    producer = pathlib.Path("main/c5vrx_rf_dump_producer.c").read_text()
    required_dis = ["ff200637", "ffe20737", "006c06b7", "005c05b7",
                    "002c05b7", "00160737", "00120737", "800006b7"]
    required_source = ["0x01fe0000", "0x006c0000",
                       "0x005c0000", "0x00160000", "0x80000000",
                       "CTRL_SW_TRIGGER_BIT"]
    failures = [x for x in required_dis if x not in dis]
    failures += [x for x in required_source if x not in producer]
    rate = json.loads(pathlib.Path("research/rf-dump-rate-fields.json").read_text())
    if [e["field"] for e in rate["entries"]] != list(range(8)):
        failures.append("rate-table-fields")
    if not re.search(r"REG32\(DUMP_CTRL\) \|= CTRL_ENABLE_BIT", producer):
        failures.append("start-enable")
    if not re.search(
        r"REG32\(DUMP_CTRL\) \|= CTRL_SW_TRIGGER_BIT;\s*"
        r"REG32\(DUMP_CTRL\) &= ~CTRL_SW_TRIGGER_BIT;", producer
    ):
        failures.append("mode0-software-trigger-pulse")
    if not re.search(r"REG32\(DUMP_CTRL\) &= ~CTRL_ENABLE_BIT", producer):
        failures.append("stop-disable")
    if not re.search(
        r"SOC_RESERVE_MEMORY_REGION\(C5VRX_RF_DUMP_PRE_GUARD_ADDR,\s*"
        r"C5VRX_RF_DUMP_POST_GUARD_ADDR\s*\+\s*"
        r"C5VRX_RF_DUMP_POST_GUARD_BYTES,\s*c5vrx_rf_dump_ram\)",
        producer,
    ):
        failures.append("dump-ram-not-reserved")
    if "if (s_configured || s_running)" not in producer:
        failures.append("double-owner-not-rejected")
    if not re.search(r"li\s+a5,12.{0,400}R_RISCV_CALL\s+ble_rx_start", dis,
                     re.DOTALL):
        failures.append("vendor-mode12-ble-rx-start")
    if "mode != C5VRX_RF_DUMP_MODE_11)" not in producer or \
            "mode == C5VRX_RF_DUMP_MODE_12" in producer:
        failures.append("mode12-split-producer-not-fail-closed")
    print("RF dump producer audit", "PASS" if not failures else "FAIL",
          "failures=" + ",".join(failures))
    return 1 if failures else 0

if __name__ == "__main__":
    raise SystemExit(main())
