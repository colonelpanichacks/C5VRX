#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-only
"""Offline WBFM discriminator for future C5VRX IQ captures.

Input is interleaved signed int16 IQ by default:
    I0, Q0, I1, Q1, ...
"""

from __future__ import annotations

import argparse
from pathlib import Path
import numpy as np


def load_iq(path: Path, dtype: str) -> np.ndarray:
    dt = np.dtype(dtype)
    raw = np.fromfile(path, dtype=dt)
    if raw.size < 4 or raw.size % 2:
        raise ValueError("IQ file must contain an even number of interleaved I/Q samples")
    raw = raw.astype(np.float32)
    return raw[0::2] + 1j * raw[1::2]


def fm_discriminator(iq: np.ndarray) -> np.ndarray:
    if iq.size < 2:
        return np.empty(0, dtype=np.float32)
    prod = iq[1:] * np.conj(iq[:-1])
    return np.angle(prod).astype(np.float32)


def moving_average(x: np.ndarray, taps: int) -> np.ndarray:
    if taps <= 1:
        return x
    kernel = np.ones(taps, dtype=np.float32) / taps
    return np.convolve(x, kernel, mode="same").astype(np.float32)


def self_test() -> None:
    fs = 20_000_000
    n = 200_000
    t = np.arange(n, dtype=np.float64) / fs
    f_base = 100_000.0
    deviation = 2_000_000.0
    inst_freq = deviation * np.sin(2 * np.pi * f_base * t)
    phase = 2 * np.pi * np.cumsum(inst_freq) / fs
    iq = np.exp(1j * phase).astype(np.complex64)
    demod = fm_discriminator(iq)
    recovered_hz = demod * fs / (2 * np.pi)
    expected = inst_freq[1:]
    rmse = float(np.sqrt(np.mean((recovered_hz - expected) ** 2)))
    print(f"self-test RMSE: {rmse:.1f} Hz")
    if rmse > 2_000:
        raise SystemExit("self-test failed")
    print("self-test PASS")


def main() -> None:
    p = argparse.ArgumentParser()
    p.add_argument("input", nargs="?", type=Path, help="interleaved IQ input")
    p.add_argument("--dtype", default="<i2", help="numpy dtype, default little-endian int16")
    p.add_argument("--sample-rate", type=float, default=20_000_000)
    p.add_argument("--smooth", type=int, default=1, help="moving-average taps")
    p.add_argument("--decimate", type=int, default=1)
    p.add_argument("--output", type=Path, default=Path("demod.f32"))
    p.add_argument("--self-test", action="store_true")
    args = p.parse_args()

    if args.self_test:
        self_test()
        return
    if args.input is None:
        p.error("input is required unless --self-test is used")
    if args.decimate < 1:
        p.error("--decimate must be >= 1")

    iq = load_iq(args.input, args.dtype)
    demod = fm_discriminator(iq)
    demod = moving_average(demod, args.smooth)
    demod = demod[::args.decimate]
    hz = demod * args.sample_rate / (2 * np.pi)
    hz.astype("<f4").tofile(args.output)
    out_rate = args.sample_rate / args.decimate
    print(f"wrote {hz.size} float32 samples to {args.output}")
    print(f"output sample rate: {out_rate:.3f} Hz")


if __name__ == "__main__":
    main()
