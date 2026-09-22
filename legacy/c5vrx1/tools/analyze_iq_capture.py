#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-only
"""Quick diagnostics for a decoded C5VRX interleaved int16 I/Q capture.

This is intentionally a proof-of-concept analyzer, not a video decoder. It
answers the first hardware questions: is the buffer complex, is there a carrier,
how wide is it, what is its frequency offset, and does the FM discriminator show
roughly line-periodic analog-video structure?
"""

from __future__ import annotations

import argparse
from pathlib import Path
import numpy as np


def load_iq(path: Path) -> np.ndarray:
    raw = np.fromfile(path, dtype="<i2")
    if raw.size < 8 or raw.size % 2:
        raise ValueError("expected little-endian int16 interleaved I,Q samples")
    f = raw.astype(np.float32)
    return (f[0::2] + 1j * f[1::2]).astype(np.complex64)


def fm_discriminator(iq: np.ndarray) -> np.ndarray:
    return np.angle(iq[1:] * np.conj(iq[:-1])).astype(np.float32)


def estimate_frequency_offset(iq: np.ndarray, fs: float) -> float:
    prod = iq[1:] * np.conj(iq[:-1])
    mean_phasor = np.mean(prod / np.maximum(np.abs(prod), 1e-12))
    return float(np.angle(mean_phasor) * fs / (2 * np.pi))


def spectrum_metrics(iq: np.ndarray, fs: float) -> tuple[float, float, float]:
    x = iq - np.mean(iq)
    window = np.hanning(x.size).astype(np.float32)
    spec = np.fft.fftshift(np.fft.fft(x * window))
    power = np.abs(spec) ** 2
    freq = np.fft.fftshift(np.fft.fftfreq(x.size, 1.0 / fs))
    peak_hz = float(freq[int(np.argmax(power))])

    total = float(power.sum())
    if total <= 0:
        return peak_hz, 0.0, 0.0
    cdf = np.cumsum(power) / total
    lo = int(np.searchsorted(cdf, 0.005))
    hi = min(power.size - 1, int(np.searchsorted(cdf, 0.995)))
    return peak_hz, float(freq[lo]), float(freq[hi])


def estimate_video_line_period(demod: np.ndarray, fs: float) -> tuple[float, float]:
    """Return best 50-80 us autocorrelation period and normalized score."""
    x = demod.astype(np.float64)
    x -= np.mean(x)
    std = np.std(x)
    if std < 1e-12:
        return 0.0, 0.0
    x /= std

    min_lag = max(1, int(fs * 50e-6))
    max_lag = min(x.size - 2, int(fs * 80e-6))
    if max_lag <= min_lag:
        return 0.0, 0.0

    # A direct dot-product search is fine for the tiny <=16k vendor dump.
    best_lag = min_lag
    best_score = -1.0
    for lag in range(min_lag, max_lag + 1):
        score = float(np.mean(x[:-lag] * x[lag:]))
        if score > best_score:
            best_score = score
            best_lag = lag
    return best_lag / fs, best_score


def report(iq: np.ndarray, fs: float) -> dict[str, float]:
    centered = iq - np.mean(iq)
    power = np.abs(centered) ** 2
    rms = float(np.sqrt(np.mean(power)))
    freq_offset = estimate_frequency_offset(iq, fs)
    peak_hz, bw_lo, bw_hi = spectrum_metrics(iq, fs)
    demod = fm_discriminator(iq)
    line_period, line_score = estimate_video_line_period(demod, fs)

    return {
        "samples": float(iq.size),
        "duration_us": iq.size / fs * 1e6,
        "mean_i": float(np.mean(iq.real)),
        "mean_q": float(np.mean(iq.imag)),
        "rms_ac": rms,
        "freq_offset_hz": freq_offset,
        "spectrum_peak_hz": peak_hz,
        "bw99_lo_hz": bw_lo,
        "bw99_hi_hz": bw_hi,
        "bw99_hz": bw_hi - bw_lo,
        "line_period_us": line_period * 1e6,
        "line_score": line_score,
    }


def self_test() -> None:
    fs = 20_000_000.0
    n = 16_384
    line_hz = 15_625.0
    t = np.arange(n, dtype=np.float64) / fs
    # Synthetic WBFM: 600 kHz carrier offset plus a line-rate baseband tone.
    inst_freq = 600_000.0 + 1_500_000.0 * np.sin(2 * np.pi * line_hz * t)
    phase = 2 * np.pi * np.cumsum(inst_freq) / fs
    iq = np.exp(1j * phase).astype(np.complex64)
    r = report(iq, fs)
    assert abs(r["freq_offset_hz"] - 600_000.0) < 30_000.0, r
    assert 55.0 < r["line_period_us"] < 72.0, r
    assert r["line_score"] > 0.7, r
    print("capture analyzer self-test PASS")
    print(f"  offset: {r['freq_offset_hz'] / 1e3:.1f} kHz")
    print(f"  line period: {r['line_period_us']:.2f} us (score {r['line_score']:.3f})")


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("input", nargs="?", type=Path, help="interleaved int16 I/Q file")
    ap.add_argument("--sample-rate", type=float, default=80_000_000.0)
    ap.add_argument("--self-test", action="store_true")
    args = ap.parse_args()

    if args.self_test:
        self_test()
        if args.input is None:
            return 0
    if args.input is None:
        ap.error("input is required unless --self-test is used")

    iq = load_iq(args.input)
    r = report(iq, args.sample_rate)
    print(f"samples:            {int(r['samples'])}")
    print(f"duration:           {r['duration_us']:.2f} us")
    print(f"mean I/Q:           {r['mean_i']:.2f} / {r['mean_q']:.2f}")
    print(f"AC RMS magnitude:   {r['rms_ac']:.2f}")
    print(f"phase freq offset:  {r['freq_offset_hz'] / 1e6:+.4f} MHz")
    print(f"FFT peak:           {r['spectrum_peak_hz'] / 1e6:+.4f} MHz")
    print(f"99% power interval: {r['bw99_lo_hz'] / 1e6:+.3f} .. {r['bw99_hi_hz'] / 1e6:+.3f} MHz")
    print(f"99% width:          {r['bw99_hz'] / 1e6:.3f} MHz")
    if r["line_period_us"]:
        print(f"50-80us autocorr:   {r['line_period_us']:.2f} us (score {r['line_score']:.3f})")
        print("PAL line period is ~64.00 us; NTSC is ~63.56 us. A high score near either is encouraging, not proof.")
    else:
        print("50-80us autocorr:   unavailable (capture too short / constant)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
