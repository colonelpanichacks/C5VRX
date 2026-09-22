#!/usr/bin/env python3
"""C5VRX-3 USB video preview viewer.

Connects to the receiver's USB Serial/JTAG console, sends 'p' to start the
preview stream (firmware console key), parses v1 packets with the exact
legacy magic/CRC-32 framing, and displays 160x120 GRAY8 frames.

Dependencies:
    required: pyserial          (pip install pyserial)
    optional: pygame            (live window; pip install pygame)
    optional: Pillow            (periodic PNG dumps; pip install Pillow)
    fallback: if neither optional package is present, prints frame stats.

Usage:
    python3 tools/preview_viewer.py /dev/cu.usbmodemXXXX
    python3 tools/preview_viewer.py COM7 --scale 4
    python3 tools/preview_viewer.py /dev/cu.usbmodemXXXX --png-dir frames

Ctrl+C stops the viewer and sends 'p' again to turn the firmware preview off.
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

# Reuse the exact legacy v1 wire-format parser (magic, dual CRC-32, resync).
sys.path.insert(
    0,
    str(Path(__file__).resolve().parent.parent / "legacy" / "c5vrx1" / "tools"),
)
try:
    from c5vrx_usb_protocol import (
        FRAME_DESCRIPTOR,
        PACKET_GRAY8_FRAME,
        PACKET_STREAM_END,
        PACKET_STREAM_INFO,
        PIXEL_FORMAT_GRAY8,
        StreamDecoder,
    )
except ImportError as exc:  # pragma: no cover - defensive, repo layout moved
    sys.exit(f"cannot import legacy c5vrx_usb_protocol.py: {exc}")

try:
    import serial
except ImportError:
    sys.exit("pyserial is required: pip install pyserial")

WIDTH = 160
HEIGHT = 120
SCALE_DEFAULT = 4


def make_display(scale: int, png_dir: Path | None):
    """Return a display callback: pygame window, PIL PNG dump, or stats."""
    try:
        import pygame

        pygame.init()
        screen = pygame.display.set_mode((WIDTH * scale, HEIGHT * scale))
        pygame.display.set_caption("C5VRX-3 USB preview")

        def show(frame: bytes, seq: int) -> bool:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    return False
            rgb = bytes(c for v in frame for c in (v, v, v))
            surf = pygame.image.frombuffer(rgb, (WIDTH, HEIGHT), "RGB")
            surf = pygame.transform.scale(surf, (WIDTH * scale, HEIGHT * scale))
            screen.blit(surf, (0, 0))
            pygame.display.flip()
            return True

        print("display: pygame window")
        return show
    except ImportError:
        pass

    if png_dir is not None:
        try:
            from PIL import Image

            png_dir.mkdir(parents=True, exist_ok=True)
            state = {"count": 0}

            def show(frame: bytes, seq: int) -> bool:
                state["count"] += 1
                if state["count"] % 30 == 1:
                    img = Image.frombytes("L", (WIDTH, HEIGHT), frame)
                    img = img.resize(
                        (WIDTH * scale, HEIGHT * scale), Image.NEAREST
                    )
                    path = png_dir / f"preview_{seq:06d}.png"
                    img.save(path)
                    print(f"saved {path}")
                return True

            print(f"display: PIL, dumping every 30th frame to {png_dir}/")
            return show
        except ImportError:
            pass

    def show(frame: bytes, seq: int) -> bool:
        lo = min(frame)
        hi = max(frame)
        mean = sum(frame) // len(frame)
        print(f"frame seq={seq} min={lo} max={hi} mean={mean}")
        return True

    print("display: none (install pygame or Pillow); printing frame stats")
    return show


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("port", help="serial port, e.g. /dev/cu.usbmodemXXXX or COM7")
    ap.add_argument("--baud", type=int, default=115200,
                    help="baud rate (ignored by USB Serial/JTAG, kept for UART bridges)")
    ap.add_argument("--scale", type=int, default=SCALE_DEFAULT)
    ap.add_argument("--png-dir", type=Path, default=None,
                    help="force the Pillow PNG-dump display mode into this directory")
    args = ap.parse_args()

    show = make_display(args.scale, args.png_dir)
    decoder = StreamDecoder()

    ser = serial.Serial(args.port, args.baud, timeout=0.1)
    ser.reset_input_buffer()
    ser.write(b"p")  # toggle firmware preview ON
    ser.flush()
    print(f"sent 'p' to {args.port}; waiting for preview packets...")

    frames = 0
    started = time.monotonic()
    try:
        while True:
            data = ser.read(4096)
            if not data:
                continue
            for kind, value in decoder.feed(data):
                if kind == "line":
                    print(value)
                elif kind == "error":
                    print(f"[protocol] {value}", file=sys.stderr)
                elif kind == "packet":
                    if value.packet_type == PACKET_STREAM_INFO:
                        (w, h, stride, fmt, flags) = FRAME_DESCRIPTOR.unpack(
                            value.payload
                        )
                        print(f"[stream] info: {w}x{h} stride={stride} "
                              f"fmt={fmt} flags={flags}")
                    elif value.packet_type == PACKET_GRAY8_FRAME:
                        (w, h, stride, fmt, flags) = FRAME_DESCRIPTOR.unpack_from(
                            value.payload
                        )
                        frame = value.payload[FRAME_DESCRIPTOR.size:]
                        if (w, h, fmt) != (WIDTH, HEIGHT, PIXEL_FORMAT_GRAY8):
                            print(f"[stream] unexpected geometry {w}x{h} fmt={fmt}")
                            continue
                        if len(frame) != stride * h:
                            print(f"[stream] short frame: {len(frame)} bytes")
                            continue
                        frames += 1
                        locked = bool(flags & 1)
                        if not show(frame, value.sequence):
                            raise KeyboardInterrupt
                        if frames % 10 == 1:
                            rate = frames / max(time.monotonic() - started, 1e-6)
                            print(f"[stream] frame {frames} seq={value.sequence} "
                                  f"locked={locked} ({rate:.1f} fps avg)")
                    elif value.packet_type == PACKET_STREAM_END:
                        dropped = int.from_bytes(value.payload, "little")
                        print(f"[stream] end, device-dropped frames: {dropped}")
    except KeyboardInterrupt:
        pass
    finally:
        ser.write(b"p")  # toggle firmware preview OFF
        ser.flush()
        ser.close()
        rate = frames / max(time.monotonic() - started, 1e-6)
        print(f"done: {frames} frames in {time.monotonic() - started:.1f}s "
              f"({rate:.1f} fps avg)")


if __name__ == "__main__":
    main()
