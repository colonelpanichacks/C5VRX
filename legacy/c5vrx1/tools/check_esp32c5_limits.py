#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-only
"""Fail CI when the XIAO ESP32-C5 firmware outgrows its resource budget."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any


XIAO_APP_PARTITION_BYTES = 0x600000
C5_BOOTLOADER_REGION_BYTES = 0x6000
MIN_HP_SRAM_FREE_BYTES = 160 * 1024
MIN_LP_SRAM_FREE_BYTES = 8 * 1024


def memory_region(report: dict[str, Any], name: str) -> dict[str, Any]:
    for region in report.get("layout", []):
        if region.get("name") == name:
            return region
    raise ValueError(f"esp_idf_size report has no {name!r} region")


def validate(report: dict[str, Any], app_bytes: int,
             bootloader_bytes: int) -> list[str]:
    hp = memory_region(report, "HP SRAM")
    lp = memory_region(report, "LP SRAM")
    errors: list[str] = []

    hp_free = int(hp["free"])
    lp_free = int(lp["free"])
    if hp_free < MIN_HP_SRAM_FREE_BYTES:
        errors.append(
            f"HP SRAM free {hp_free} < runtime safety budget "
            f"{MIN_HP_SRAM_FREE_BYTES}")
    if lp_free < MIN_LP_SRAM_FREE_BYTES:
        errors.append(
            f"LP SRAM free {lp_free} < safety budget {MIN_LP_SRAM_FREE_BYTES}")
    if app_bytes > XIAO_APP_PARTITION_BYTES:
        errors.append(
            f"application {app_bytes} > partition {XIAO_APP_PARTITION_BYTES}")
    if bootloader_bytes > C5_BOOTLOADER_REGION_BYTES:
        errors.append(
            f"bootloader {bootloader_bytes} > region {C5_BOOTLOADER_REGION_BYTES}")
    return errors


def self_test() -> None:
    good = {
        "layout": [
            {"name": "HP SRAM", "free": MIN_HP_SRAM_FREE_BYTES},
            {"name": "LP SRAM", "free": MIN_LP_SRAM_FREE_BYTES},
        ]
    }
    assert not validate(
        good, XIAO_APP_PARTITION_BYTES, C5_BOOTLOADER_REGION_BYTES)

    bad = {
        "layout": [
            {"name": "HP SRAM", "free": MIN_HP_SRAM_FREE_BYTES - 1},
            {"name": "LP SRAM", "free": MIN_LP_SRAM_FREE_BYTES - 1},
        ]
    }
    errors = validate(
        bad, XIAO_APP_PARTITION_BYTES + 1, C5_BOOTLOADER_REGION_BYTES + 1)
    assert len(errors) == 4
    try:
        memory_region({"layout": []}, "HP SRAM")
    except ValueError:
        pass
    else:
        raise AssertionError("missing memory region was accepted")
    print("C5VRX_ESP32C5_LIMITS_SELF_TEST result=PASS")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--size-json", type=Path,
                        help="esp_idf_size --format json2 output")
    parser.add_argument("--binary", type=Path,
                        help="built application .bin")
    parser.add_argument("--bootloader", type=Path,
                        help="built bootloader .bin")
    parser.add_argument("--self-test", action="store_true")
    args = parser.parse_args()

    if args.self_test:
        self_test()
        return 0
    if not args.size_json or not args.binary or not args.bootloader:
        parser.error("--size-json, --binary and --bootloader are required")

    report = json.loads(args.size_json.read_text(encoding="utf-8"))
    hp = memory_region(report, "HP SRAM")
    lp = memory_region(report, "LP SRAM")
    app_bytes = args.binary.stat().st_size
    bootloader_bytes = args.bootloader.stat().st_size
    errors = validate(report, app_bytes, bootloader_bytes)

    result = "FAIL" if errors else "PASS"
    print(
        f"C5VRX_ESP32C5_LIMITS result={result} "
        f"app={app_bytes}/{XIAO_APP_PARTITION_BYTES} "
        f"bootloader={bootloader_bytes}/{C5_BOOTLOADER_REGION_BYTES} "
        f"hp_sram_used={int(hp['used'])}/{int(hp['total'])} "
        f"hp_sram_free={int(hp['free'])}/{MIN_HP_SRAM_FREE_BYTES} "
        f"lp_sram_used={int(lp['used'])}/{int(lp['total'])} "
        f"lp_sram_free={int(lp['free'])}/{MIN_LP_SRAM_FREE_BYTES}")
    for error in errors:
        print(f"ERROR: {error}")
    return 1 if errors else 0


if __name__ == "__main__":
    raise SystemExit(main())
