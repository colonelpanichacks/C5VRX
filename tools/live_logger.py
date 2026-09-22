#!/usr/bin/env python3
"""Live logger for C5VRX-3 Dual-Loop Adaptive Gain Controller."""
import sys
import time
import serial

PORT = sys.argv[1] if len(sys.argv) > 1 else "COM10"
BAUD = 115200
DURATION = int(sys.argv[2]) if len(sys.argv) > 2 else 120

print(f"Connecting to {PORT} (DTR=False, RTS=False) for {DURATION}s...")

ser = serial.Serial()
ser.port = PORT
ser.baudrate = BAUD
ser.dtr = False
ser.rts = False
ser.timeout = 0.5
try:
    ser.open()
except Exception as e:
    print(f"Error opening {PORT}: {e}")
    sys.exit(1)

# Ensure chip is running
ser.dtr = False
ser.rts = False

# Send space to get startup diagnostic frame
time.sleep(0.2)
ser.write(b" ")
time.sleep(0.2)

start_time = time.time()
print(f"--- Live Telemetry Started at {time.strftime('%H:%M:%S')} ---")

log_path = "live_walkaround_log.txt"
with open(log_path, "w", encoding="utf-8") as f_log:
    while time.time() - start_time < DURATION:
        line = ser.readline().decode("utf-8", errors="replace").strip()
        if line:
            ts = time.strftime("%H:%M:%S")
            msg = f"{ts} | {line}"
            print(msg)
            f_log.write(msg + "\n")
            f_log.flush()
        else:
            time.sleep(0.05)

ser.close()
print(f"--- Logging finished. Saved to {log_path} ---")
