#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-only
"""Estimate minimal passive CVBS output hardware for C5VRX.

The model compares 2-8 bit uniform resistor-DAC quantization and a one-bit
noise-shaped research path. It also calculates two resistor-network families:

1. direct-loaded: absolute minimum for a short 75-ohm input, without 75-ohm
   source matching;
2. source-matched: the analog-first reference network, designed for a chosen
   Thevenin source impedance and loaded full-scale voltage.

This is an engineering sizing tool, not a substitute for scope measurements of
real ESP32-C5 GPIO VOH/output resistance, cable parasitics and the actual load.
"""
from __future__ import annotations

import argparse
import math

import numpy as np

PAL_CHROMA_HZ = 4_433_618.75
DEFAULT_FS = 40_000_000.0
DEFAULT_LOAD_OHM = 75.0
DEFAULT_SOURCE_OHM = 75.0
DEFAULT_VDD = 3.3
DEFAULT_FULL_SCALE_V = 1.0


def synthetic_cvbs(fs: float = DEFAULT_FS, lines: int = 8) -> np.ndarray:
    line_n = int(round(fs * 64e-6))
    sync_n = int(round(fs * 4.7e-6))
    active_start = int(round(fs * 10.4e-6))
    active_n = int(round(fs * 52.0e-6))

    out = np.full(line_n * lines, 0.30, dtype=np.float64)
    for line in range(lines):
        base = line * line_n
        out[base:base + sync_n] = 0.0

        t = np.arange(active_n, dtype=np.float64) / fs
        x = np.linspace(0.0, 1.0, active_n, endpoint=False)
        luma = 0.38 + 0.42 * (0.5 + 0.5 * np.sin(2.0 * np.pi * (2.0 + 0.15 * line) * x))
        chroma_phase = (line & 1) * np.pi
        chroma = 0.12 * np.sin(2.0 * np.pi * PAL_CHROMA_HZ * t + chroma_phase)
        active = np.clip(luma + chroma, 0.30, 1.0)
        out[base + active_start:base + active_start + active_n] = active
    return out


def reconstruction_filter(x: np.ndarray, fs: float, pass_hz: float = 5.5e6, stop_hz: float = 7.0e6) -> np.ndarray:
    spec = np.fft.rfft(x)
    f = np.fft.rfftfreq(x.size, 1.0 / fs)
    h = np.ones_like(f)
    h[f >= stop_hz] = 0.0
    transition = (f > pass_hz) & (f < stop_hz)
    h[transition] = 0.5 * (1.0 + np.cos(np.pi * (f[transition] - pass_hz) / (stop_hz - pass_hz)))
    return np.fft.irfft(spec * h, n=x.size)


def quantize_uniform(x: np.ndarray, bits: int) -> np.ndarray:
    levels = (1 << bits) - 1
    return np.rint(np.clip(x, 0.0, 1.0) * levels) / levels


def sigma_delta_1bit(x: np.ndarray) -> np.ndarray:
    y = np.empty_like(x)
    integrator = 0.0
    previous = 0.0
    for i, v in enumerate(np.clip(x, 0.0, 1.0)):
        integrator += float(v) - previous
        bit = 1.0 if integrator >= 0.5 else 0.0
        y[i] = bit
        previous = bit
    return y


def metrics(reference: np.ndarray, candidate: np.ndarray) -> tuple[float, float, float]:
    err = candidate - reference
    signal = reference - np.mean(reference)
    mse = float(np.mean(err * err))
    snr = math.inf if mse <= 1e-30 else 10.0 * math.log10(float(np.mean(signal * signal)) / mse)
    corr = float(np.corrcoef(reference, candidate)[0, 1])
    rmse = math.sqrt(mse)
    return snr, corr, rmse


def direct_loaded_dac_resistors(
    bits: int,
    vdd: float = DEFAULT_VDD,
    full_scale_v: float = DEFAULT_FULL_SCALE_V,
    load_ohm: float = DEFAULT_LOAD_OHM,
) -> list[float]:
    """Ideal branch resistors (MSB -> LSB) directly into a shunt load.

    This is the absolute-minimum short-trace experiment and deliberately does
    NOT provide a specified 75-ohm source impedance.
    """
    if bits < 1 or full_scale_v <= 0 or full_scale_v >= vdd or load_ohm <= 0:
        raise ValueError("invalid DAC parameters")
    total_g = (full_scale_v / (vdd - full_scale_v)) / load_ohm
    unit_g = total_g / ((1 << bits) - 1)
    return [1.0 / (unit_g * (1 << k)) for k in reversed(range(bits))]


def source_matched_dac_resistors(
    bits: int,
    vdd: float = DEFAULT_VDD,
    full_scale_v: float = DEFAULT_FULL_SCALE_V,
    load_ohm: float = DEFAULT_LOAD_OHM,
    source_ohm: float = DEFAULT_SOURCE_OHM,
) -> tuple[list[float], float]:
    """Binary branch resistors (MSB -> LSB) plus shunt for a Thevenin source."""
    if bits < 1 or min(vdd, full_scale_v, load_ohm, source_ohm) <= 0:
        raise ValueError("invalid DAC parameters")

    open_circuit_v = full_scale_v * (source_ohm + load_ohm) / load_ohm
    if open_circuit_v >= vdd:
        raise ValueError("requested loaded full scale needs open-circuit voltage >= VDD")

    total_g = 1.0 / source_ohm
    branch_g = (open_circuit_v / vdd) * total_g
    shunt_g = total_g - branch_g
    if shunt_g <= 0:
        raise ValueError("no positive shunt conductance for requested parameters")

    unit_g = branch_g / ((1 << bits) - 1)
    branches = [1.0 / (unit_g * (1 << k)) for k in reversed(range(bits))]
    shunt = 1.0 / shunt_g
    return branches, shunt


def network_result(
    branches_msb_first: list[float],
    shunt_ohm: float,
    vdd: float = DEFAULT_VDD,
    load_ohm: float = DEFAULT_LOAD_OHM,
) -> tuple[float, float, float]:
    branch_g = sum(1.0 / r for r in branches_msb_first)
    shunt_g = 1.0 / shunt_ohm
    source_r = 1.0 / (branch_g + shunt_g)
    open_v = vdd * branch_g / (branch_g + shunt_g)
    loaded_v = open_v * load_ohm / (source_r + load_ohm)
    return source_r, open_v, loaded_v


def evaluate(fs: float = DEFAULT_FS) -> dict[str, tuple[float, float, float]]:
    source = synthetic_cvbs(fs)
    ref = reconstruction_filter(source, fs)
    results: dict[str, tuple[float, float, float]] = {}

    one = reconstruction_filter(sigma_delta_1bit(source), fs)
    results["1-bit PDM"] = metrics(ref, one)

    for bits in range(2, 9):
        q = quantize_uniform(source, bits)
        rec = reconstruction_filter(q, fs)
        results[f"{bits}-bit DAC"] = metrics(ref, rec)
    return results


def self_test() -> None:
    r = evaluate(DEFAULT_FS)
    assert r["3-bit DAC"][1] > 0.99, r
    assert r["3-bit DAC"][0] > 18.0, r
    assert r["5-bit DAC"][0] > 28.0, r
    assert r["1-bit PDM"][0] < r["3-bit DAC"][0], r

    branches, shunt = source_matched_dac_resistors(6)
    source_r, open_v, loaded_v = network_result(branches, shunt)
    assert abs(source_r - 75.0) < 1e-9
    assert abs(open_v - 2.0) < 1e-9
    assert abs(loaded_v - 1.0) < 1e-9

    expected_lsb_to_msb = [7796.25, 3898.125, 1949.0625, 974.53125, 487.265625, 243.6328125]
    actual_lsb_to_msb = list(reversed(branches))
    assert np.allclose(actual_lsb_to_msb, expected_lsb_to_msb)
    assert abs(shunt - 190.3846153846154) < 1e-9

    print("minimal CVBS DAC model self-test PASS")
    for name, (snr, corr, rmse) in r.items():
        print(f"  {name:10s}: SNR {snr:5.1f} dB  corr {corr:.5f}  RMSE {rmse:.4f} V")

    print("  6-bit 75R-source reference (MSB->LSB): " +
          ", ".join(f"{v:.2f} ohm" for v in branches))
    print(f"  reference shunt: {shunt:.2f} ohm")


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--sample-rate", type=float, default=DEFAULT_FS)
    ap.add_argument("--self-test", action="store_true")
    ap.add_argument("--bits", type=int, default=6, choices=range(2, 9))
    args = ap.parse_args()

    if args.self_test:
        self_test()
        return 0

    r = evaluate(args.sample_rate)
    for name, (snr, corr, rmse) in r.items():
        print(f"{name:10s}: SNR {snr:5.1f} dB  corr {corr:.5f}  RMSE {rmse:.4f} V")

    direct = direct_loaded_dac_resistors(args.bits)
    print(f"\nIdeal {args.bits}-bit direct-loaded branches into 75 ohm (MSB -> LSB):")
    for i, value in enumerate(direct):
        print(f"  bit {args.bits - 1 - i}: {value:.1f} ohm")
    print("  WARNING: this variant does not source-match the cable.")

    branches, shunt = source_matched_dac_resistors(args.bits)
    source_r, open_v, loaded_v = network_result(branches, shunt)
    print(f"\nIdeal {args.bits}-bit source-matched reference (MSB -> LSB):")
    for i, value in enumerate(branches):
        print(f"  bit {args.bits - 1 - i}: {value:.1f} ohm")
    print(f"  video node -> GND: {shunt:.1f} ohm")
    print(f"  Thevenin source: {source_r:.2f} ohm")
    print(f"  open/full-scale: {open_v:.3f} V")
    print(f"  loaded/full-scale into 75R: {loaded_v:.3f} V")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
