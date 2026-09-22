#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-only
"""Decode ESP32-C5 RF-test FE/ADC dump words into signed 10-bit I/Q.

The lower-20-bit packing is confirmed by C5 ESP-IDF v6.0.2 `print_dump_data`
and `accumiq` disassembly and matches historical Espressif RF-test tooling:
Q = bits 0..9, I = bits 10..19, both signed 10-bit two's complement.
"""

from __future__ import annotations

import argparse
import csv
import re
import struct
import sys
from dataclasses import dataclass
from pathlib import Path

IQ_LINE_RE = re.compile(r"\bIQ:([0-9a-fA-F]{8})\b")
HEX_TOKEN_RE = re.compile(r"(?:0x)?([0-9a-fA-F]{8})")


def sign10(v: int) -> int:
    v &= 0x3FF
    return v - 0x400 if v & 0x200 else v


def pack10(i: int, q: int, upper: int = 0) -> int:
    return ((upper & 0xFFF) << 20) | ((i & 0x3FF) << 10) | (q & 0x3FF)


@dataclass(frozen=True)
class Sample:
    index: int
    raw: int
    i: int
    q: int
    rx_gain_bits: int
    rx_err_bit: int
    agc_state_bits: int


def decode_word(word: int, index: int = 0) -> Sample:
    word &= 0xFFFFFFFF
    return Sample(
        index=index,
        raw=word,
        q=sign10(word),
        i=sign10(word >> 10),
        # These upper-bit interpretations match historical Espressif tooling;
        # treat them as provisional on C5 until hardware captures confirm them.
        rx_gain_bits=(word >> 20) & 0x7F,
        rx_err_bit=(word >> 27) & 0x01,
        agc_state_bits=(word >> 28) & 0x0F,
    )


def parse_words(text: str) -> list[int]:
    # Prefer machine-readable lines emitted by c5vrx_adc_dump.c so timestamps
    # and unrelated log numbers cannot accidentally become samples.
    marked = [int(m.group(1), 16) for m in IQ_LINE_RE.finditer(text)]
    if marked:
        return marked
    return [int(m.group(1), 16) for m in HEX_TOKEN_RE.finditer(text)]


def self_test() -> None:
    vectors = [(-512, -512), (-511, -1), (-1, 0), (0, 1), (123, -234), (511, 511)]
    for n, (i, q) in enumerate(vectors):
        word = pack10(i, q, upper=0xA55)
        s = decode_word(word, n)
        assert s.i == i, (i, s)
        assert s.q == q, (q, s)
        assert s.rx_gain_bits == (0xA55 & 0x7F)
        assert s.rx_err_bit == ((0xA55 >> 7) & 1)
        assert s.agc_state_bits == ((0xA55 >> 8) & 0xF)
    print(f"ADC dump decoder self-test passed ({len(vectors)} vectors)")


def write_csv(samples: list[Sample], path: Path | None) -> None:
    output = open(path, "w", newline="", encoding="utf-8") if path else sys.stdout
    try:
        writer = csv.writer(output)
        writer.writerow(["index", "raw_hex", "i", "q", "rx_gain_bits", "rx_err_bit", "agc_state_bits"])
        for s in samples:
            writer.writerow([
                s.index,
                f"0x{s.raw:08x}",
                s.i,
                s.q,
                s.rx_gain_bits,
                s.rx_err_bit,
                s.agc_state_bits,
            ])
    finally:
        if path:
            output.close()


def write_interleaved_i16(samples: list[Sample], path: Path) -> None:
    with path.open("wb") as f:
        for s in samples:
            f.write(struct.pack("<hh", s.i, s.q))


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("input", nargs="?", help="Serial/log file containing IQ:xxxxxxxx lines or raw 8-digit hex words")
    ap.add_argument("--csv", dest="csv_path", type=Path, help="Write decoded samples to CSV")
    ap.add_argument(
        "--iq-bin",
        dest="iq_bin_path",
        type=Path,
        help="Write little-endian int16 I,Q interleaved binary for tools/wbfm_demod.py",
    )
    ap.add_argument("--self-test", action="store_true")
    args = ap.parse_args()

    if args.self_test:
        self_test()
        if not args.input:
            return 0

    if not args.input:
        ap.error("input file required unless --self-test is used")

    words = parse_words(Path(args.input).read_text(encoding="utf-8", errors="replace"))
    samples = [decode_word(w, i) for i, w in enumerate(words)]
    if not samples:
        raise SystemExit("no IQ dump words found")

    if args.csv_path or not args.iq_bin_path:
        write_csv(samples, args.csv_path)
    if args.iq_bin_path:
        write_interleaved_i16(samples, args.iq_bin_path)
        print(f"wrote {len(samples)} interleaved int16 IQ samples to {args.iq_bin_path}", file=sys.stderr)

    print(f"decoded {len(samples)} words", file=sys.stderr)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
