#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-only
"""Capture one bounded CRC-framed C5VRX IQ block and report honest metrics."""

from __future__ import annotations

import argparse
import json
import math
import statistics
import struct
import sys
import time
from datetime import datetime, timezone
from pathlib import Path

import serial

from c5vrx_usb_protocol import (
    PACKET_IQ_U32_BLOCK,
    PACKET_IQ_U32_CHUNK,
    Packet,
    StreamDecoder,
    decode_iq_block,
    decode_iq_chunk,
)

MAX_DEVICE_WORDS = 16384


def signed10(value: int) -> int:
    value &= 0x3FF
    return value - 0x400 if value & 0x200 else value


def iq_metrics(words: list[int]) -> dict[str, float | int | str]:
    i_values = [signed10(word >> 10) for word in words]
    q_values = [signed10(word) for word in words]
    mean_i = statistics.fmean(i_values)
    mean_q = statistics.fmean(q_values)
    ac_power = statistics.fmean(
        (i - mean_i) ** 2 + (q - mean_q) ** 2
        for i, q in zip(i_values, q_values)
    )
    radii = [
        math.hypot(i - mean_i, q - mean_q)
        for i, q in zip(i_values, q_values)
    ]
    discriminator = []
    for index in range(1, len(words)):
        pi, pq = i_values[index - 1], q_values[index - 1]
        ci, cq = i_values[index], q_values[index]
        discriminator.append(math.atan2(cq * pi - ci * pq, ci * pi + cq * pq))
    return {
        "words": len(words),
        "packing": "I=bits10..19_signed10,Q=bits0..9_signed10",
        "i_mean": mean_i,
        "i_std": statistics.pstdev(i_values),
        "q_mean": mean_q,
        "q_std": statistics.pstdev(q_values),
        "complex_ac_rms": math.sqrt(ac_power),
        "radius_mean": statistics.fmean(radii),
        "radius_std": statistics.pstdev(radii),
        "fm_mean_rad": statistics.fmean(discriminator),
        "fm_std_rad": statistics.pstdev(discriminator),
    }


def main() -> int:
    if hasattr(sys.stdout, "reconfigure"):
        sys.stdout.reconfigure(encoding="utf-8", errors="backslashreplace")
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", default="COM10")
    parser.add_argument("--words", type=int, default=MAX_DEVICE_WORDS)
    parser.add_argument("--label", required=True)
    parser.add_argument("--output-dir", type=Path, default=Path("captures"))
    parser.add_argument("--timeout", type=float, default=15.0)
    args = parser.parse_args()
    if not 256 <= args.words <= MAX_DEVICE_WORDS:
        parser.error(f"--words must be 256..{MAX_DEVICE_WORDS}")

    decoder = StreamDecoder()
    transport_ready = False
    capture_done = False
    block: list[int] | None = None
    packet_timestamp_us = 0
    capture_started = 0.0
    packet_received = 0.0
    chunk_capture_id: int | None = None
    chunk_total_words: int | None = None
    iq_chunks: dict[int, tuple[int, ...]] = {}

    # Keep the USB Serial/JTAG reset/boot control lines inactive. Opening the
    # port with pySerial defaults can otherwise return a C5 that was manually
    # flashed straight to the ROM download loop instead of the application.
    ser = serial.Serial()
    ser.port = args.port
    ser.baudrate = 115200
    ser.timeout = 0.1
    ser.write_timeout = 2
    ser.dtr = False
    ser.rts = False
    ser.open()
    # Native USB Serial/JTAG may be openable slightly before its bulk OUT
    # endpoint is ready after enumeration. Match the proven Receiver Console
    # sequencing and do not write immediately after CreateFile/open().
    time.sleep(0.6)
    wire_capture = bytearray(ser.read_all())
    session_ok = False

    def read_wire(size: int = 65536) -> bytes:
        data = ser.read(size)
        wire_capture.extend(data)
        return data

    def send(command: str) -> None:
        ser.write((command + "\n").encode("ascii"))
        ser.flush()
        print(f"> {command}", flush=True)

    try:
        send("PING")
        pong = False
        deadline = time.monotonic() + 4.0
        while time.monotonic() < deadline and not pong:
            for kind, value in decoder.feed(read_wire()):
                if kind == "line":
                    line = str(value)
                    print(line, flush=True)
                    pong = line.startswith("C5VRX_PONG")
                elif kind == "error":
                    print(f"C5VRX_HOST_PROTOCOL_ERROR reason={value}", flush=True)
        if not pong:
            raise RuntimeError("PING was not acknowledged")

        # Tuning is runtime state and resets to the firmware default after a
        # reboot. Always select and verify A1 immediately before each capture.
        send("SET A 1")
        a1_set = False
        deadline = time.monotonic() + 5.0
        while time.monotonic() < deadline and not a1_set:
            for kind, value in decoder.feed(read_wire()):
                if kind == "line":
                    line = str(value)
                    print(line, flush=True)
                    a1_set = (line.startswith("C5VRX_OK set") and
                              "band=A" in line and "channel=1" in line and
                              "wifi=173" in line)
                elif kind == "error":
                    print(f"C5VRX_HOST_PROTOCOL_ERROR reason={value}", flush=True)
        if not a1_set:
            raise RuntimeError("SET A 1 was not acknowledged with Wi-Fi channel 173")

        send("STATUS")
        status_a1 = False
        deadline = time.monotonic() + 4.0
        while time.monotonic() < deadline and not status_a1:
            for kind, value in decoder.feed(read_wire()):
                if kind == "line":
                    line = str(value)
                    print(line, flush=True)
                    status_a1 = (line.startswith("C5VRX_STATUS") and
                                 "band=A" in line and "channel=1" in line and
                                 "mhz=5865" in line and "readback=173" in line)
                elif kind == "error":
                    print(f"C5VRX_HOST_PROTOCOL_ERROR reason={value}", flush=True)
        if not status_a1:
            raise RuntimeError("STATUS did not verify A1/5865 MHz/readback 173")

        send("USB PREVIEW START")
        deadline = time.monotonic() + args.timeout
        while time.monotonic() < deadline and not transport_ready:
            for kind, value in decoder.feed(read_wire()):
                if kind == "line":
                    line = str(value)
                    print(line, flush=True)
                    if (line.startswith("C5VRX_USB_PREVIEW state=START") and
                            "code=0" in line):
                        transport_ready = True
                elif kind == "error":
                    print(f"C5VRX_HOST_PROTOCOL_ERROR reason={value}", flush=True)
        if not transport_ready:
            raise RuntimeError("binary IQ transport did not become ready")

        capture_started = time.monotonic()
        send(f"CAPTURE {args.words}")
        deadline = time.monotonic() + args.timeout
        while time.monotonic() < deadline and not (capture_done and block is not None):
            chunk = read_wire()
            if not chunk:
                continue
            for kind, value in decoder.feed(chunk):
                if kind == "line":
                    line = str(value)
                    print(line, flush=True)
                    if line.startswith("C5VRX_CAPTURE_DONE"):
                        if "code=0" not in line:
                            raise RuntimeError(line)
                        capture_done = True
                elif kind == "packet":
                    packet = value
                    assert isinstance(packet, Packet)
                    if packet.packet_type == PACKET_IQ_U32_BLOCK:
                        if block is not None:
                            raise RuntimeError("duplicate IQ packet")
                        block = decode_iq_block(packet)
                        packet_timestamp_us = packet.timestamp_us
                        packet_received = time.monotonic()
                    elif packet.packet_type == PACKET_IQ_U32_CHUNK:
                        iq_chunk = decode_iq_chunk(packet)
                        if chunk_capture_id is None:
                            chunk_capture_id = iq_chunk.capture_id
                            chunk_total_words = iq_chunk.total_words
                            packet_timestamp_us = packet.timestamp_us
                        if (iq_chunk.capture_id != chunk_capture_id or
                                iq_chunk.total_words != chunk_total_words):
                            raise RuntimeError("mixed IQ captures in chunk stream")
                        existing = iq_chunks.get(iq_chunk.offset_words)
                        if existing is not None and existing != iq_chunk.words:
                            raise RuntimeError("conflicting duplicate IQ chunk")
                        iq_chunks[iq_chunk.offset_words] = iq_chunk.words
                        received_words = sum(len(words) for words in iq_chunks.values())
                        if received_words == chunk_total_words:
                            assembled: list[int] = []
                            offset = 0
                            for chunk_offset in sorted(iq_chunks):
                                if chunk_offset != offset:
                                    break
                                words = iq_chunks[chunk_offset]
                                assembled.extend(words)
                                offset += len(words)
                            if offset == chunk_total_words:
                                block = assembled
                                packet_received = time.monotonic()
                else:
                    print(f"C5VRX_HOST_PROTOCOL_ERROR reason={value}", flush=True)
        if block is None:
            raise RuntimeError("IQ packet not received")
        if len(block) != args.words:
            raise RuntimeError(f"short IQ block: {len(block)}/{args.words}")
        if not capture_done:
            raise RuntimeError("capture packet arrived without CAPTURE_DONE")
        session_ok = True
    finally:
        if session_ok:
            try:
                send("USB PREVIEW STOP")
            except Exception:
                pass
        ser.close()
        if not session_ok:
            args.output_dir.mkdir(parents=True, exist_ok=True)
            stamp = datetime.now().strftime("%Y%m%d-%H%M%S")
            wire_path = args.output_dir / f"{stamp}-{args.label}.wire.bin"
            wire_path.write_bytes(wire_capture)
            print(f"C5VRX_HOST_WIRE_SAVED path={wire_path.resolve()}", flush=True)

    elapsed = max(1e-9, packet_received - capture_started)
    metrics = iq_metrics(block)
    metrics.update({
        "label": args.label,
        "captured_utc": datetime.now(timezone.utc).isoformat(),
        "device_block_words": args.words,
        "continuity": "ONE_FINITE_VENDOR_DUMP",
        "packet_timestamp_us": packet_timestamp_us,
        "capture_command_to_packet_s": elapsed,
        "host_transaction_mwords_per_s": len(block) / elapsed / 1_000_000.0,
        "producer_sample_rate_sps": "UNKNOWN_NOT_INFERRED_FROM_USB_TRANSFER",
    })

    args.output_dir.mkdir(parents=True, exist_ok=True)
    stem = f"{datetime.now().strftime('%Y%m%d-%H%M%S')}-{args.label}-{len(block)}"
    raw_path = args.output_dir / f"{stem}.u32le"
    json_path = args.output_dir / f"{stem}.json"
    raw_path.write_bytes(struct.pack(f"<{len(block)}I", *block))
    json_path.write_text(json.dumps(metrics, indent=2) + "\n", encoding="utf-8")

    print("C5VRX_HOST_CAPTURE_OK " + " ".join(
        f"{key}={value}" for key, value in metrics.items()
        if key in {
            "label", "words", "complex_ac_rms", "radius_mean", "radius_std",
            "fm_std_rad", "host_transaction_mwords_per_s",
        }
    ))
    print(f"C5VRX_HOST_RAW path={raw_path.resolve()}")
    print(f"C5VRX_HOST_METRICS path={json_path.resolve()}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
