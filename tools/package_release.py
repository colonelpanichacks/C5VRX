#!/usr/bin/env python3
"""Package C5VRX-3 production binaries for distribution, web flashers, and desktop."""

import os
import sys
import shutil
import subprocess
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
BUILD = ROOT / "build"
PACKAGES = ROOT / "packages"
DESKTOP = Path("C:/Users/leonb/Desktop")
DESKTOP_PACKAGES = DESKTOP / "packages"
STAGE_DIR = Path("C:/Users/leonb/Twotoz/C5VRX/_package_stage")

bootloader = BUILD / "bootloader/bootloader.bin"
ptable = BUILD / "partition_table/partition-table.bin"
app = BUILD / "c5vrx3.bin"
flasher_args = BUILD / "flasher_args.json"


def main():
    if not app.exists():
        print(f"Error: {app} does not exist. Run idf.py build first.")
        sys.exit(1)

    print("=======================================================")
    print(" PACKAGING C5VRX-3 PRODUCTION BINARIES")
    print("=======================================================")

    PACKAGES.mkdir(exist_ok=True)
    DESKTOP_PACKAGES.mkdir(exist_ok=True)
    STAGE_DIR.mkdir(exist_ok=True)

    # 1. Desktop root
    dest_desktop_app = DESKTOP / "c5vrx3.bin"
    shutil.copy2(app, dest_desktop_app)
    print(f"[DESKTOP] Copied to: {dest_desktop_app} ({dest_desktop_app.stat().st_size:,} bytes)")

    # 2. Repo packages & Desktop packages
    for target in [PACKAGES, DESKTOP_PACKAGES]:
        shutil.copy2(app, target / "c5vrx3.bin")
        shutil.copy2(bootloader, target / "bootloader.bin")
        shutil.copy2(ptable, target / "partition-table.bin")
        if flasher_args.exists():
            shutil.copy2(flasher_args, target / "flasher_args.json")

    # 3. Create merged binary (0x0 universal one-click flash)
    merged_bin = PACKAGES / "c5vrx3_merged.bin"
    cmd = [
        sys.executable, "-m", "esptool",
        "--chip", "esp32c5",
        "merge-bin",
        "-o", str(merged_bin),
        "--flash-mode", "dio",
        "--flash-size", "8MB",
        "--flash-freq", "80m",
        "0x2000", str(bootloader),
        "0x8000", str(ptable),
        "0x10000", str(app),
    ]
    res = subprocess.run(cmd, capture_output=True, text=True)
    if res.returncode == 0:
        print(f"[MERGED] Created:  {merged_bin} ({merged_bin.stat().st_size:,} bytes)")
        shutil.copy2(merged_bin, DESKTOP_PACKAGES / "c5vrx3_merged.bin")
        shutil.copy2(merged_bin, DESKTOP / "c5vrx3_merged.bin")
        print(f"[DESKTOP] Copied merged: {DESKTOP / 'c5vrx3_merged.bin'}")
    else:
        print(f"[WARNING] merge-bin output: {res.stderr}")

    # 4. Update legacy C5VRX stage
    shutil.copy2(app, STAGE_DIR / "c5vrx3.bin")
    shutil.copy2(app, STAGE_DIR / "C5VRX.bin")
    shutil.copy2(bootloader, STAGE_DIR / "bootloader.bin")
    shutil.copy2(ptable, STAGE_DIR / "partition-table.bin")
    if flasher_args.exists():
        shutil.copy2(flasher_args, STAGE_DIR / "flasher_args.json")

    print("\n=======================================================")
    print(" ALL PACKAGES CREATED SUCCESSFULLY!")
    print(f" Desktop App:    {DESKTOP / 'c5vrx3.bin'}")
    print(f" Desktop Merged: {DESKTOP / 'c5vrx3_merged.bin'}")
    print(f" Desktop Folder: {DESKTOP_PACKAGES}")
    print(f" Repo Packages:  {PACKAGES}")
    print("=======================================================")


if __name__ == "__main__":
    main()
