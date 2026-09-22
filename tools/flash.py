#!/usr/bin/env python3
"""Flash C5VRX-3 firmware to ESP32-C5 board."""
import sys
import subprocess
from pathlib import Path
import serial.tools.list_ports

ROOT = Path(__file__).resolve().parent.parent
BUILD = ROOT / "build"


def find_esp_port():
    ports = serial.tools.list_ports.comports()
    for p in ports:
        desc = (p.description or "").upper()
        hwid = (p.hwid or "").upper()
        if "303A" in hwid or "ESPRESSIF" in desc or "USB JTAG" in desc or "USB-SERIAL" in desc:
            return p.device
    for p in ports:
        if not (p.hwid or "").startswith("BTHENUM"):
            return p.device
    return "COM10"


def main():
    port = sys.argv[1] if len(sys.argv) > 1 else find_esp_port()

    bootloader = BUILD / "bootloader/bootloader.bin"
    ptable = BUILD / "partition_table/partition-table.bin"
    app = BUILD / "c5vrx3.bin"

    for f in (bootloader, ptable, app):
        if not f.exists():
            print(f"Error: {f} not found! Run build first.")
            sys.exit(1)

    print(f"=======================================================")
    print(f" FLASHING C5VRX-3 (Seamless16K Phase5 Production)")
    print(f" Port: {port}")
    print(f" App:  {app}")
    print(f"=======================================================")

    cmd = [
        sys.executable, "-m", "esptool",
        "--chip", "esp32c5",
        "-p", port,
        "-b", "460800",
        "--before", "usb-reset",
        "--after", "watchdog-reset",
        "write-flash",
        "--flash-mode", "dio",
        "--flash-size", "8MB",
        "--flash-freq", "80m",
        "0x2000", str(bootloader),
        "0x8000", str(ptable),
        "0x10000", str(app),
    ]

    res = subprocess.run(cmd)
    sys.exit(res.returncode)


if __name__ == "__main__":
    main()
