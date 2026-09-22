#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-only
"""Fail CI if a packaged board identity drifts from its sdkconfig pin map."""

from __future__ import annotations

import json
import re
from pathlib import Path


ROOT = Path(__file__).resolve().parent.parent

PROFILES = {
    "devkit-wroom": {
        "json": "firmware_profiles/devkit-wroom.json",
        "config": "sdkconfig.defaults.flasher",
        "partition": "partitions_receiver_console.csv",
        "flash_mb": 4,
        "factory_size": 0x300000,
        "pins": [0, 1, 6, 8, 9, 10],
    },
    "xiao-esp32c5": {
        "json": "firmware_profiles/xiao-esp32c5.json",
        "config": "sdkconfig.defaults.xiao_receiver_console",
        "partition": "partitions_xiao_receiver_console.csv",
        "flash_mb": 8,
        "factory_size": 0x600000,
        "pins": [23, 24, 11, 12, 8, 9],
    },
}


def config_values(path: Path) -> dict[str, str]:
    values: dict[str, str] = {}
    for line in path.read_text(encoding="utf-8").splitlines():
        if line.startswith("CONFIG_") and "=" in line:
            key, value = line.split("=", 1)
            values[key] = value.strip().strip('"')
    return values


def factory_partition_size(path: Path) -> int:
    for line in path.read_text(encoding="utf-8").splitlines():
        if line.lstrip().startswith("factory,"):
            columns = [column.strip() for column in line.split(",")]
            return int(columns[4], 0)
    raise AssertionError(f"No factory partition in {path}")


def validate() -> None:
    seen_pin_maps: set[tuple[int, ...]] = set()
    for profile_id, expected in PROFILES.items():
        metadata = json.loads((ROOT / expected["json"]).read_text(encoding="utf-8"))
        config = config_values(ROOT / expected["config"])

        assert metadata["schema_version"] == 1
        assert metadata["profile_id"] == profile_id
        assert metadata["flash_size_mb"] == expected["flash_mb"]
        assert config["CONFIG_C5VRX_BOARD_PROFILE"] == profile_id
        assert config[f"CONFIG_ESPTOOLPY_FLASHSIZE_{expected['flash_mb']}MB"] == "y"

        pins = tuple(int(config[f"CONFIG_C5VRX_CVBS_D{bit}_GPIO"])
                     for bit in range(6))
        assert pins == tuple(expected["pins"]), (profile_id, pins)
        assert len(set(pins)) == len(pins), (profile_id, "duplicate GPIO")
        assert not ({13, 14} & set(pins)), (profile_id, "native USB collision")
        assert pins not in seen_pin_maps, (profile_id, "profile pin maps must differ")
        seen_pin_maps.add(pins)

        assert factory_partition_size(ROOT / expected["partition"]) == expected["factory_size"]
        pin_text = "/".join(str(pin) for pin in pins)
        assert re.search(rf"GPIO(?:s)?[^\n]*{re.escape(pin_text)}", metadata["av_pin_summary"])


if __name__ == "__main__":
    validate()
    print("firmware profile validation: PASS")
