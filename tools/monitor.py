#!/usr/bin/env python3
"""Interactive serial monitor for C5VRX-3 with real-time reception & channel controls."""

import sys
import time
import threading
import msvcrt
import serial
import serial.tools.list_ports

def find_default_port():
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

PORT = sys.argv[1] if len(sys.argv) > 1 else find_default_port()
BAUD = 115200

print("=" * 60)
print(f" C5VRX-3 INTERACTIVE SERIAL MONITOR")
print(f" Port: {PORT} @ {BAUD} baud")
print("=" * 60)
print(" Hotkeys:")
print("  'c' / 'C'   : Cycle FPV Channel / Band (A1..A8, R1..R8, B1..B8, F1..F8)")
print("  '+' / '-'   : Manual RF gain step (+/- 2)")
print("  'a'/'s'/'m' : AGC mode (Active / Shadow / Manual)")
print("  'b'         : Cycle Bandwidth Gear (Auto Gearbox / Forced BW40 / Forced BW20)")
print("  'f'         : Cycle AFC Mode (Auto Centering / Hold / Off)")
print("  ',' / '.'   : Fine-tune carrier offset (-50 / +50 kHz)")
print("  '0'         : Reset frequency offset to +0 kHz")
print("  'o'         : Toggle OSD On-Screen Display Menu (console)")
print("  'O'         : Toggle BOOT button menu trigger (Safe Flight Mode)")
print("  ' ' / 'n'   : Move menu cursor (when menu active)")
print("  'x' / Enter : Select / change menu setting (when menu active)")
print("  'd'         : Dump real-time hardware reception diagnostics")
print("  Ctrl+C      : Exit")
print("-" * 60)
sys.stdout.flush()

ser = serial.Serial()
ser.port = PORT
ser.baudrate = BAUD
ser.dtr = False
ser.rts = False
ser.timeout = 0.1

try:
    ser.open()
    ser.dtr = False
    ser.rts = False
except Exception as e:
    print(f"Error opening {PORT}: {e}")
    sys.exit(1)

def reader():
    is_in_status_line = False
    while True:
        try:
            raw = ser.readline()
            if not raw:
                continue
            text = raw.decode("utf-8", errors="replace").rstrip("\r\n")
            if not text:
                continue

            # In-place status line for 1 Hz AGC telemetry (no endless scrolling!)
            if text.startswith("[AGC:"):
                sys.stdout.write(f"\r{text.ljust(95)}\r")
                sys.stdout.flush()
                is_in_status_line = True
            else:
                if is_in_status_line:
                    sys.stdout.write("\n")
                    is_in_status_line = False
                sys.stdout.write(text + "\n")
                sys.stdout.flush()
        except Exception:
            break

t = threading.Thread(target=reader, daemon=True)
t.start()

# Query current channel and diagnostics on connect
time.sleep(0.1)
try:
    ser.write(b"d\n")
    ser.flush()
except Exception:
    pass

try:
    while True:
        if msvcrt.kbhit():
            ch = msvcrt.getch()
            if ch == b"\x03":  # Ctrl+C
                break
            try:
                ser.write(ch)
                ser.flush()
            except Exception:
                pass
        time.sleep(0.02)
except KeyboardInterrupt:
    pass
finally:
    ser.close()
    print("\nClosed serial port.")
