import sys
import time
import subprocess
from pathlib import Path
import serial.tools.list_ports

ROOT = Path(__file__).resolve().parent.parent
BUILD = ROOT / "build"

bootloader = BUILD / "bootloader" / "bootloader.bin"
ptable = BUILD / "partition_table" / "partition-table.bin"
app = BUILD / "c5vrx3.bin"

print("=" * 60)
print(" AUTO-FLASH WATCHER GESTART")
print(" Wachten tot ESP32-C5 wordt aangesloten...")
print(" (Zodra de USB-kabel contact maakt, begint het flashen direct!)")
print("=" * 60)
sys.stdout.flush()

BT_PORTS = {"COM3", "COM4", "COM5", "COM6", "COM7", "COM8"}

def find_esp_port():
    ports = [p.device for p in serial.tools.list_ports.comports()]
    # Check non-BT ports first
    candidates = [p for p in ports if p not in BT_PORTS]
    if "COM10" in candidates:
        return "COM10"
    if candidates:
        return candidates[0]
    return None

for attempt in range(120): # 2 minutes
    port = find_esp_port()
    if port:
        print(f"\n[GEVONDEN] Poort gedetecteerd: {port}! Starten met flashen...")
        sys.stdout.flush()
        cmd = [
            sys.executable, "-m", "esptool",
            "--chip", "esp32c5",
            "-p", port,
            "-b", "460800",
            "--before", "default-reset",
            "--after", "watchdog-reset",
            "write-flash",
            "--flash-mode", "dio",
            "--flash-size", "8MB",
            "--flash-freq", "80m",
            "0x2000", str(bootloader),
            "0x8000", str(ptable),
            "0x10000", str(app),
        ]
        res = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
        print(res.stdout)
        if res.returncode == 0:
            print("\n" + "=" * 60)
            print(f" >>> FLASH VOLTOOID OP {port}! <<<")
            print("=" * 60)
            sys.stdout.flush()
            sys.exit(0)
        else:
            print("[HERHAAL] Flash poging mislukt, probeer opnieuw (houd eventueel Boot ingedrukt)...")
            sys.stdout.flush()
    time.sleep(1.0)

print("\nTimeout: Geen ESP32-C5 poort gedetecteerd.")
sys.exit(1)
