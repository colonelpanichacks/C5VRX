#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-only
"""Validate the C5VRX 4:1 BitScrambler WBFM architecture."""
from __future__ import annotations

import math
from pathlib import Path

import numpy as np

BSASM = Path(__file__).resolve().parents[1] / "main" / "c5vrx_wbfm_4to1.bsasm"
BIAS = 32


def _signed(code: int, bits: int) -> int:
    sign = 1 << (bits - 1)
    return code - (1 << bits) if code & sign else code


def _coarse_center(code: int, bits: int, source_bits: int = 10) -> float:
    coarse = _signed(code, bits)
    scale = 1 << (source_bits - bits)
    return coarse * scale + (scale - 1) / 2.0


def expected_lut() -> np.ndarray:
    out = np.empty(1024, dtype=np.uint16)
    for i5 in range(32):
        i = _coarse_center(i5, 5)
        for q5 in range(32):
            q = _coarse_center(q5, 5)
            phase = math.atan2(q, i) % (2.0 * math.pi)
            p6 = int(round(phase * 64.0 / (2.0 * math.pi))) & 0x3F
            bias_minus_phase = (BIAS - p6) & 0x3F
            out[(i5 << 5) | q5] = p6 | (bias_minus_phase << 8)
    return out


def validate_program_text() -> None:
    text = BSASM.read_text(encoding="utf-8")
    lowered = text.lower()
    assert "cfg lut_width_bits 16" in lowered
    assert "addctibl" in lowered
    assert "ldctibl" in lowered
    assert "set 22..26 5..9" in lowered
    assert "set 27..31 15..19" in lowered
    assert "set 0..5 b0..b5" in lowered
    assert "write 8" in lowered
    # LUT data is intentionally loaded at runtime through bitscrambler_load_lut.
    assert not any(line.strip().lower().startswith("lut ") for line in text.splitlines())


def phase6_codes(i: np.ndarray, q: np.ndarray, lut: np.ndarray) -> np.ndarray:
    iu = i.astype(np.int32) & 0x3FF
    qu = q.astype(np.int32) & 0x3FF
    idx = ((iu >> 5) << 5) | (qu >> 5)
    return (lut[idx] & 0x3F).astype(np.uint8)


def synthetic_iq(fs: float = 80_000_000.0, n: int = 400_000) -> tuple[np.ndarray, np.ndarray]:
    t = np.arange(n, dtype=np.float64) / fs
    line = np.mod(t, 64e-6)
    sync = np.where(line < 4.7e-6, -1.0, 0.0)
    luma = 0.42 * np.sin(2.0 * np.pi * 1_050_000.0 * t)
    chroma = 0.18 * np.sin(2.0 * np.pi * 4_433_618.75 * t)
    baseband = 0.55 * sync + luma + chroma
    inst_freq = 3_800_000.0 * np.clip(baseband, -1.0, 1.0)
    phase = 2.0 * np.pi * np.cumsum(inst_freq) / fs
    z = np.exp(1j * phase)
    i = np.clip(np.rint(z.real * 430.0), -512, 511).astype(np.int16)
    q = np.clip(np.rint(z.imag * 430.0), -512, 511).astype(np.int16)
    return i, q


def self_test() -> None:
    validate_program_text()
    lut = expected_lut()
    assert lut.size == 1024 and lut.nbytes == 2048

    i80, q80 = synthetic_iq()
    i20 = i80[::4]
    q20 = q80[::4]

    p = phase6_codes(i20, q20, lut).astype(np.int16)
    dac_codes = ((BIAS + p[1:] - p[:-1]) & 0x3F).astype(np.uint8)
    signed_codes = ((dac_codes.astype(np.int16) - BIAS + 32) & 0x3F) - 32
    approx = signed_codes.astype(np.float64) * (2.0 * math.pi / 64.0)

    z = i20.astype(np.float64) + 1j * q20.astype(np.float64)
    exact = np.angle(z[1:] * np.conj(z[:-1]))
    err = np.angle(np.exp(1j * (approx - exact)))
    rmse = float(np.sqrt(np.mean(err * err)))
    corr = float(np.corrcoef(exact, approx)[0, 1])

    assert rmse < 0.06, (rmse, corr)
    assert corr > 0.99, (rmse, corr)
    assert len(i80) == 4 * len(i20)

    print("4:1 BitScrambler WBFM architecture self-test PASS")
    print("  runtime LUT: 1024 x 16-bit = 2048 bytes")
    print("  design-case stream ratio: candidate 80 MS/s IQ -> 20 MS/s phase delta")
    print(f"  decimated synthetic FM: RMSE {rmse:.4f} rad, corr {corr:.5f}")


if __name__ == "__main__":
    self_test()
