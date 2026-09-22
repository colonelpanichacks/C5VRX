#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-only
"""Evaluate hardware-friendly FM discriminator ideas for ESP32-C5 BitScrambler.

C5VRX's expensive live operation is not PAL/NTSC decoding; it is turning a
wide complex I/Q stream into composite-video FM baseband. The ESP32-C5 has a
BitScrambler in the GDMA path with only eight instructions but a 2 KiB LUT.
That makes a quantized phase discriminator interesting.

Pass 1 (common to both experiments):
    signed 10-bit I/Q -> phase8 + (-phase8) via a 1024 x 16-bit LUT (2 KiB)

Pass 2A (quality path):
    exact phase8[n] + (-phase8[n-1]) modulo 256 using BitScrambler counters.
    This needs no trigonometry or multiply and models a multi-cycle second pass.

Pass 2B (throughput path):
    current phase6 + previous phase5 -> delta8 via a 2048 x 8-bit LUT (2 KiB).
    This intentionally discards a few phase bits so the pair fits the 11-bit
    address available when the BitScrambler LUT is configured 8-bit wide.

The tool validates approximation error numerically. It does NOT prove the C5
can sustain the required DMA/BitScrambler throughput or that the recovered RF
dump can be made continuous.
"""
from __future__ import annotations

import argparse
import math
from pathlib import Path
import numpy as np

IQ_BITS = 10
IQ_COARSE_BITS = 5
PHASE_BITS = 8
FAST_CURRENT_BITS = 6
FAST_PREVIOUS_BITS = 5
PHASE_LUT_SIZE = 1 << (IQ_COARSE_BITS * 2)
FAST_DELTA_LUT_SIZE = 1 << (FAST_CURRENT_BITS + FAST_PREVIOUS_BITS)


def _signed(code: int, bits: int) -> int:
    sign = 1 << (bits - 1)
    return code - (1 << bits) if code & sign else code


def _coarse_center(code: int, bits: int, source_bits: int = IQ_BITS) -> float:
    coarse = _signed(code, bits)
    scale = 1 << (source_bits - bits)
    return coarse * scale + (scale - 1) / 2.0


def build_phase_lut() -> np.ndarray:
    """1024 x uint16: low byte phase8, high byte two's-complement -phase8."""
    lut = np.empty(PHASE_LUT_SIZE, dtype=np.uint16)
    for i5 in range(1 << IQ_COARSE_BITS):
        i = _coarse_center(i5, IQ_COARSE_BITS)
        for q5 in range(1 << IQ_COARSE_BITS):
            q = _coarse_center(q5, IQ_COARSE_BITS)
            phase = math.atan2(q, i) % (2.0 * math.pi)
            p = int(round(phase * 256.0 / (2.0 * math.pi))) & 0xFF
            neg = (-p) & 0xFF
            lut[(i5 << IQ_COARSE_BITS) | q5] = p | (neg << 8)
    return lut


def build_fast_delta_lut() -> np.ndarray:
    """2048 x uint8: phase6 current + phase5 previous -> signed delta8."""
    lut = np.empty(FAST_DELTA_LUT_SIZE, dtype=np.uint8)
    for current6 in range(1 << FAST_CURRENT_BITS):
        current_phase = current6 * (2.0 * math.pi / (1 << FAST_CURRENT_BITS))
        for previous5 in range(1 << FAST_PREVIOUS_BITS):
            previous_phase = (previous5 * 2.0 + 0.5) * (
                2.0 * math.pi / (1 << FAST_CURRENT_BITS)
            )
            delta = (current_phase - previous_phase + math.pi) % (2.0 * math.pi) - math.pi
            q = int(round(delta * 256.0 / (2.0 * math.pi)))
            q = max(-128, min(127, q))
            lut[(current6 << FAST_PREVIOUS_BITS) | previous5] = q & 0xFF
    return lut


def phase8_codes(i: np.ndarray, q: np.ndarray, phase_lut: np.ndarray) -> np.ndarray:
    if i.size != q.size:
        raise ValueError("I and Q arrays must have equal length")
    mask = (1 << IQ_BITS) - 1
    iu = i.astype(np.int32) & mask
    qu = q.astype(np.int32) & mask
    idx = ((iu >> (IQ_BITS - IQ_COARSE_BITS)) << IQ_COARSE_BITS) | (
        qu >> (IQ_BITS - IQ_COARSE_BITS)
    )
    return (phase_lut[idx] & 0xFF).astype(np.uint8)


def demod_counter_path(i: np.ndarray, q: np.ndarray, phase_lut: np.ndarray | None = None) -> np.ndarray:
    """Model the high-quality phase8 + neg(previous) BitScrambler-counter path."""
    if i.size < 2 or i.size != q.size:
        raise ValueError("I and Q arrays must have equal length >= 2")
    if phase_lut is None:
        phase_lut = build_phase_lut()
    p = phase8_codes(i, q, phase_lut).astype(np.int16)
    d = ((p[1:] - p[:-1] + 128) & 0xFF) - 128
    return d.astype(np.float32) * np.float32(2.0 * math.pi / 256.0)


def demod_fast_lut_path(i: np.ndarray, q: np.ndarray,
                        phase_lut: np.ndarray | None = None,
                        delta_lut: np.ndarray | None = None) -> np.ndarray:
    """Model the faster 11-bit second LUT path."""
    if i.size < 2 or i.size != q.size:
        raise ValueError("I and Q arrays must have equal length >= 2")
    if phase_lut is None:
        phase_lut = build_phase_lut()
    if delta_lut is None:
        delta_lut = build_fast_delta_lut()

    p = phase8_codes(i, q, phase_lut)
    current6 = p[1:].astype(np.int32) >> (PHASE_BITS - FAST_CURRENT_BITS)
    previous5 = p[:-1].astype(np.int32) >> (PHASE_BITS - FAST_PREVIOUS_BITS)
    idx = (current6 << FAST_PREVIOUS_BITS) | previous5
    raw = delta_lut[idx].astype(np.int16)
    raw[raw >= 128] -= 256
    return raw.astype(np.float32) * np.float32(2.0 * math.pi / 256.0)


def exact_demod(i: np.ndarray, q: np.ndarray) -> np.ndarray:
    z = i.astype(np.float32) + 1j * q.astype(np.float32)
    return np.angle(z[1:] * np.conj(z[:-1])).astype(np.float32)


def quantize_iq(z: np.ndarray, peak: float = 430.0) -> tuple[np.ndarray, np.ndarray]:
    mag = float(np.max(np.abs(z)))
    if mag <= 0:
        raise ValueError("zero signal")
    scale = peak / mag
    i = np.clip(np.rint(z.real * scale), -512, 511).astype(np.int16)
    q = np.clip(np.rint(z.imag * scale), -512, 511).astype(np.int16)
    return i, q


def synthetic_video_like_iq(fs: float = 40_000_000.0, n: int = 200_000) -> tuple[np.ndarray, np.ndarray]:
    """Constant-envelope FM with sync-, luma- and PAL-chroma-like rates."""
    t = np.arange(n, dtype=np.float64) / fs
    line_phase = np.mod(t, 64e-6)
    sync = np.where(line_phase < 4.7e-6, -1.0, 0.0)
    luma = 0.42 * np.sin(2.0 * np.pi * 1_050_000.0 * t)
    chroma = 0.18 * np.sin(2.0 * np.pi * 4_433_618.75 * t)
    baseband = 0.55 * sync + luma + chroma
    inst_freq = 3_800_000.0 * np.clip(baseband, -1.0, 1.0)
    phase = 2.0 * np.pi * np.cumsum(inst_freq) / fs
    return quantize_iq(np.exp(1j * phase))


def metrics(reference: np.ndarray, approx: np.ndarray) -> dict[str, float]:
    if reference.shape != approx.shape:
        raise ValueError("shape mismatch")
    err = np.angle(np.exp(1j * (approx.astype(np.float64) - reference.astype(np.float64))))
    rmse = float(np.sqrt(np.mean(err * err)))
    mae = float(np.mean(np.abs(err)))
    corr = float(np.corrcoef(reference, approx)[0, 1])
    signal_rms = float(np.sqrt(np.mean(reference.astype(np.float64) ** 2)))
    snr_db = float(20.0 * np.log10(signal_rms / max(rmse, 1e-12)))
    return {"rmse_rad": rmse, "mae_rad": mae, "corr": corr, "snr_db": snr_db}


def emit_luts(directory: Path) -> None:
    directory.mkdir(parents=True, exist_ok=True)
    phase = build_phase_lut()
    fast = build_fast_delta_lut()
    phase.astype("<u2").tofile(directory / "phase8_and_neg_from_iq5x5.bin")
    fast.tofile(directory / "delta8_from_phase6x5.bin")
    print(f"wrote phase LUT: {phase.nbytes} bytes")
    print(f"wrote fast delta LUT: {fast.nbytes} bytes")


def self_test() -> None:
    phase_lut = build_phase_lut()
    fast_lut = build_fast_delta_lut()
    assert phase_lut.nbytes == 2048
    assert fast_lut.nbytes == 2048

    i, q = synthetic_video_like_iq()
    ref = exact_demod(i, q)
    quality = metrics(ref, demod_counter_path(i, q, phase_lut))
    fast = metrics(ref, demod_fast_lut_path(i, q, phase_lut, fast_lut))

    assert quality["rmse_rad"] < 0.05, quality
    assert quality["corr"] > 0.98, quality
    assert fast["rmse_rad"] < 0.10, fast
    assert fast["corr"] > 0.93, fast

    print("BitScrambler FM concept self-test PASS")
    print("  phase LUT: 1024 x 16-bit = 2048 bytes")
    print("  fast delta LUT: 2048 x 8-bit = 2048 bytes")
    print("  quality path (phase8 + counter subtract):")
    print(f"    RMSE {quality['rmse_rad']:.4f} rad/sample, corr {quality['corr']:.5f}, equiv SNR {quality['snr_db']:.1f} dB")
    print("  throughput path (phase6 + previous phase5 LUT):")
    print(f"    RMSE {fast['rmse_rad']:.4f} rad/sample, corr {fast['corr']:.5f}, equiv SNR {fast['snr_db']:.1f} dB")


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--self-test", action="store_true")
    ap.add_argument("--emit-dir", type=Path, help="write the two 2 KiB LUT images")
    args = ap.parse_args()

    if args.emit_dir:
        emit_luts(args.emit_dir)
    if args.self_test or not args.emit_dir:
        self_test()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
