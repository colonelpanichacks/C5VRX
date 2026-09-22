"""Bounded serial control/logging for the diagnostic (never live pacing)."""
import argparse
import time
from pathlib import Path
import serial

p = argparse.ArgumentParser()
p.add_argument('--port', default='COM10')
p.add_argument('--seconds', type=float, default=15)
p.add_argument('--capture', action='store_true')
p.add_argument('--log', type=Path, required=True)
a = p.parse_args()
buf = bytearray()
s = serial.Serial(port=None, baudrate=115200, timeout=.1, write_timeout=2)
s.dtr = False
s.rts = False
s.port = a.port
with s:
    stop = time.monotonic() + a.seconds
    while time.monotonic() < stop:
        block = s.read(s.in_waiting or 1)
        buf.extend(block)
        if b'ISSUE11 CAPTURE COMPLETE' in buf:
            break
a.log.write_bytes(buf)
print(buf.decode(errors='replace')[-7000:])
