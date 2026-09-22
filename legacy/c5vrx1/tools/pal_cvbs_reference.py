#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-only
"""Reference model for the C5VRX streamed PAL 625/50 CVBS test pattern.

This mirrors the firmware timing at 20 MS/s. It validates the half-line
vertical-sync sequence, 288 active lines per field, horizontal timing, chunk
wraparound and the optional 4.43361875 MHz swinging-burst stress signal.

It is a generator/engineering check, not a claim of broadcast compliance.
Real GPIO levels, edge shape and 75-ohm loading still need a scope.
"""
from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np

FS = 20_000_000
LINE_US = 64.0
LINE_SAMPLES = 1280
HALF = 640
FIELD_HALF_LINES = 625
FRAME_HALF_LINES = 1250

HSYNC = 94
EQ = 47
BROAD = 546
ACTIVE_START = 210
ACTIVE_END = 1250
ACTIVE_SAMPLES = ACTIVE_END - ACTIVE_START

NORMAL_START_HALF = 15
ACTIVE_START_HALF = 49
CHUNK_HALF_LINES = 64
BURST_START = 112
BURST_SAMPLES = 45
PAL_FSC = 4_433_618.75


def code_from_mv(mv: np.ndarray | float, bits: int = 6) -> np.ndarray:
    max_code = (1 << bits) - 1
    x = np.asarray(mv, dtype=np.float64)
    return np.rint(np.clip(x, 0.0, 1000.0) * max_code / 1000.0).astype(np.uint8)


def grayscale(active_x: np.ndarray, bits: int = 6) -> np.ndarray:
    bar = np.minimum(active_x // (ACTIVE_SAMPLES // 8), 7)
    mv = 1000.0 - (1000.0 - 320.0) * bar / 7.0
    return code_from_mv(mv, bits)


def templates(bits: int = 6, burst: bool = True) -> dict[str, np.ndarray]:
    sync = int(code_from_mv(0, bits))
    blank = int(code_from_mv(300, bits))

    eq = np.full(HALF, blank, dtype=np.uint8)
    eq[:EQ] = sync

    broad = np.full(HALF, blank, dtype=np.uint8)
    broad[:BROAD] = sync

    blank_first = np.full(HALF, blank, dtype=np.uint8)
    blank_first[:HSYNC] = sync
    blank_second = np.full(HALF, blank, dtype=np.uint8)

    active_first: list[np.ndarray] = []
    for phase_idx in range(2):
        a = blank_first.copy()
        if burst:
            n = np.arange(BURST_SAMPLES, dtype=np.float64)
            phase0 = (-3.0 * np.pi / 4.0) if phase_idx else (3.0 * np.pi / 4.0)
            mv = 300.0 + 150.0 * np.sin(phase0 + 2.0 * np.pi * PAL_FSC * n / FS)
            a[BURST_START:BURST_START + BURST_SAMPLES] = code_from_mv(mv, bits)
        x = np.arange(HALF - ACTIVE_START, dtype=np.int32)
        a[ACTIVE_START:] = grayscale(x, bits)
        active_first.append(a)

    active_second = np.full(HALF, blank, dtype=np.uint8)
    second_n = ACTIVE_END - HALF
    x = np.arange(HALF - ACTIVE_START, HALF - ACTIVE_START + second_n, dtype=np.int32)
    active_second[:second_n] = grayscale(x, bits)

    return {
        "eq": eq,
        "broad": broad,
        "blank_first": blank_first,
        "blank_second": blank_second,
        "active_first_0": active_first[0],
        "active_first_1": active_first[1],
        "active_second": active_second,
    }


def template_name(frame_half_line: int) -> str:
    field_pos = frame_half_line % FIELD_HALF_LINES
    if field_pos < 5:
        return "eq"
    if field_pos < 10:
        return "broad"
    if field_pos < NORMAL_START_HALF:
        return "eq"

    normal_offset = field_pos - NORMAL_START_HALF
    first_half = (normal_offset & 1) == 0
    active = field_pos >= ACTIVE_START_HALF

    if not active:
        return "blank_first" if first_half else "blank_second"
    if not first_half:
        return "active_second"
    line_index = normal_offset // 2
    return f"active_first_{line_index & 1}"


def render_frame(bits: int = 6, burst: bool = True) -> np.ndarray:
    t = templates(bits, burst)
    out = np.empty(FRAME_HALF_LINES * HALF, dtype=np.uint8)
    for h in range(FRAME_HALF_LINES):
        out[h * HALF:(h + 1) * HALF] = t[template_name(h)]
    return out


def render_chunks(count: int, bits: int = 6, burst: bool = True) -> np.ndarray:
    t = templates(bits, burst)
    out = np.empty(count * CHUNK_HALF_LINES * HALF, dtype=np.uint8)
    h = 0
    pos = 0
    for _ in range(count):
        for _ in range(CHUNK_HALF_LINES):
            out[pos:pos + HALF] = t[template_name(h)]
            pos += HALF
            h = (h + 1) % FRAME_HALF_LINES
    return out


def self_test() -> None:
    assert LINE_SAMPLES == int(round(FS * 64e-6))
    assert HALF * 2 == LINE_SAMPLES
    assert HSYNC == int(round(FS * 4.7e-6))
    assert EQ == int(round(FS * 2.35e-6))
    assert BROAD == int(round(FS * 27.3e-6))
    assert ACTIVE_START == int(round(FS * 10.5e-6))
    assert ACTIVE_END == int(round(FS * 62.5e-6))
    assert FIELD_HALF_LINES * HALF / FS == 0.020
    assert FRAME_HALF_LINES * HALF / FS == 0.040

    for field in range(2):
        base = field * FIELD_HALF_LINES
        names = [template_name(base + i) for i in range(15)]
        assert names[:5] == ["eq"] * 5
        assert names[5:10] == ["broad"] * 5
        assert names[10:15] == ["eq"] * 5

    active_names = [template_name(h) for h in range(FRAME_HALF_LINES)]
    active_half_count = sum(name.startswith("active_") for name in active_names)
    assert active_half_count == 576 * 2, active_half_count
    # There are 576 active half-lines per field = 288 complete active lines.
    for field in range(2):
        lo = field * FIELD_HALF_LINES
        hi = lo + FIELD_HALF_LINES
        count = sum(template_name(h).startswith("active_") for h in range(lo, hi))
        assert count == 576, count

    t = templates(6, True)
    sync = int(code_from_mv(0, 6))
    blank = int(code_from_mv(300, 6))
    assert np.all(t["eq"][:EQ] == sync)
    assert np.all(t["eq"][EQ:] == blank)
    assert np.all(t["broad"][:BROAD] == sync)
    assert np.all(t["blank_first"][:HSYNC] == sync)
    assert np.all(t["blank_second"] == blank)
    assert not np.array_equal(t["active_first_0"], t["active_first_1"])

    # Check that chunk generation reproduces the exact frame sequence through
    # the 40 ms frame wrap, not just when a chunk happens to align with it.
    chunks_needed = (FRAME_HALF_LINES + CHUNK_HALF_LINES * 2 - 1) // CHUNK_HALF_LINES
    streamed = render_chunks(chunks_needed, 6, True)
    frame = render_frame(6, True)
    compare = np.tile(frame, 2)[: streamed.size]
    assert np.array_equal(streamed, compare)

    burst0 = t["active_first_0"][BURST_START:BURST_START + BURST_SAMPLES].astype(np.float64)
    burst1 = t["active_first_1"][BURST_START:BURST_START + BURST_SAMPLES].astype(np.float64)
    assert np.std(burst0) > 2.0
    assert np.std(burst1) > 2.0
    assert abs(np.mean(burst0) - blank) < 2.0
    assert abs(np.mean(burst1) - blank) < 2.0

    print("PAL CVBS reference self-test PASS")
    print(f"  sample rate: {FS / 1e6:.1f} MS/s")
    print(f"  line: {LINE_SAMPLES} samples = {LINE_SAMPLES / FS * 1e6:.3f} us")
    print(f"  field: {FIELD_HALF_LINES * HALF / FS * 1e3:.3f} ms")
    print(f"  frame: {FRAME_HALF_LINES * HALF / FS * 1e3:.3f} ms")
    print("  vertical sync: 5 equalizing + 5 broad + 5 equalizing half-lines / field")
    print("  active: 288 full lines / field, 576 / frame")
    print(f"  DMA chunk: {CHUNK_HALF_LINES} half-lines = {CHUNK_HALF_LINES * HALF} bytes")


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--self-test", action="store_true")
    ap.add_argument("--bits", type=int, default=6, choices=range(2, 9))
    ap.add_argument("--no-burst", action="store_true")
    ap.add_argument("--dump-frame", type=Path, help="write one 40 ms byte-coded frame")
    args = ap.parse_args()

    if args.self_test:
        self_test()

    if args.dump_frame:
        frame = render_frame(args.bits, not args.no_burst)
        frame.tofile(args.dump_frame)
        print(f"wrote {frame.size} byte samples to {args.dump_frame}")

    if not args.self_test and not args.dump_frame:
        self_test()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
