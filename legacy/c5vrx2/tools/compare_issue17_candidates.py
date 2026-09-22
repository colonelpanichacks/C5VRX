#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'tools'))
import bs_model


import argparse
import json
import math
from pathlib import Path
import numpy as np

TAU = 2.0 * math.pi
PEDESTAL = 20
GAIN = 2

_CENT = np.array([
       0,    8,   15,   24,   32,   40,   49,   56,
      64,   72,   79,   87,   96,  104,  113,  120,
    -128, -120, -113, -104,  -96,  -88,  -79,  -72,
     -64,  -56,  -49,  -40,  -32,  -23,  -15,   -8,
], dtype=np.int32)


def sbc(code: np.ndarray, bits: int) -> np.ndarray:
    w = 1 << (10 - bits)
    c = code.astype(np.float64) * w + (w - 1) * 0.5
    return np.where(c >= 512.0, c - 1024.0, c)


def unpack(raw: np.ndarray):
    q = sbc((raw & 0x0F).astype(np.uint8), 4)
    i = sbc((raw >> 4).astype(np.uint8), 4)
    return q, i


def exact_phase(raw: np.ndarray) -> np.ndarray:
    q, i = unpack(raw)
    return np.arctan2(q, i)


def phase5_state(raw: np.ndarray) -> np.ndarray:
    return (np.rint(exact_phase(raw) * 32.0 / TAU).astype(np.int32) & 0x1F).astype(np.int8)


def wrap_r(phi: np.ndarray) -> np.ndarray:
    return (phi + math.pi) % TAU - math.pi


def scale(d: np.ndarray, gain: int = GAIN, pedestal: int = PEDESTAL) -> np.ndarray:
    n = d.astype(np.int32) * (gain + 1)
    return np.clip(np.where(n < 0, -(-n + 2) // 4, (n + 2) // 4) + pedestal, 0, 63).astype(np.uint8)


def scale_40m(d: np.ndarray, pedestal: int = PEDESTAL) -> np.ndarray:
    """Scale 25 ns adjacent deltas (which have half the deviation of 50 ns deltas)."""
    # 50 ns delta: n = d * 3 / 4.
    # 25 ns delta has 1/2 the radians, so multiply by 6 instead of 3 to match DAC scale.
    n = d.astype(np.int32) * 6
    return np.clip(np.where(n < 0, -(-n + 2) // 4, (n + 2) // 4) + pedestal, 0, 63).astype(np.uint8)


def demod_phase5_hold40(raw: np.ndarray) -> np.ndarray:
    """Baseline: n->n+2 on odd bytes with all 32 phase states, held 2x."""
    sel = raw[1::2]
    st = phase5_state(sel)
    p, c = st[:-1], st[1:]
    d = ((_CENT[c.astype(np.int32)] - _CENT[p.astype(np.int32)] + 128) % 256 - 128).astype(np.int32)
    dac20 = scale(d)
    return np.repeat(dac20, 2)


def demod_linear40(raw: np.ndarray) -> np.ndarray:
    """Linear40: Phase5 20 MS/s with midpoint reconstruction [A, (A+B+1)//2]."""
    sel = raw[1::2]
    st = phase5_state(sel)
    p, c = st[:-1], st[1:]
    d = ((_CENT[c.astype(np.int32)] - _CENT[p.astype(np.int32)] + 128) % 256 - 128).astype(np.int32)
    dac20 = scale(d)
    
    prev = dac20[:-1].astype(int)
    curr = dac20[1:].astype(int)
    mid = (prev + curr + 1) // 2
    # Pair [prev, mid]
    linear = np.column_stack((prev, mid)).ravel()
    return linear.astype(np.uint8)


def demod_polyphase40(raw: np.ndarray) -> np.ndarray:
    """Polyphase40: 4-tap cubic Hermite / Catmull-Rom fractional interpolation."""
    sel = raw[1::2]
    st = phase5_state(sel)
    p, c = st[:-1], st[1:]
    d = ((_CENT[c.astype(np.int32)] - _CENT[p.astype(np.int32)] + 128) % 256 - 128).astype(np.int32)
    dac20 = scale(d).astype(float)
    
    # y[n+0.5] = -0.0625 y[n-1] + 0.5625 y[n] + 0.5625 y[n+1] - 0.0625 y[n+2]
    n = len(dac20)
    if n < 4:
        return demod_linear40(raw)
    
    y = dac20
    mid = -0.0625 * y[:-3] + 0.5625 * y[1:-2] + 0.5625 * y[2:-1] - 0.0625 * y[3:]
    mid = np.clip(np.rint(mid), 0, 63).astype(np.uint8)
    
    curr = y[1:-2].astype(np.uint8)
    poly = np.column_stack((curr, mid)).ravel()
    return poly.astype(np.uint8)


def demod_true40(raw: np.ndarray) -> np.ndarray:
    """True adjacent 40->40 MS/s demodulator: one output per raw byte."""
    st = phase5_state(raw)
    p, c = st[:-1], st[1:]
    d = ((_CENT[c.astype(np.int32)] - _CENT[p.astype(np.int32)] + 128) % 256 - 128).astype(np.int32)
    return scale_40m(d)



def demod_hw_true40(raw: np.ndarray) -> np.ndarray:
    """Issue 17: 2-bundle hardware True 40 MS/s adjacent BitScrambler core."""
    asm_path = ROOT / "main/c5vrx2_wbfm_q4_true40_2to1.bsasm"
    asm_text = asm_path.read_text()
    sim_out = bs_model.simulate(asm_text, bytes(raw), len(raw))
    # Aligned with 3-sample pipeline delay relative to ideal np.diff
    return np.array(sim_out[3:], dtype=np.uint8)


def ideal_reference_40m(raw: np.ndarray) -> np.ndarray:
    """High-precision continuous floating-point adjacent discriminator reference."""
    phi = exact_phase(raw)
    d = wrap_r(np.diff(phi))  # radians per 25 ns
    # In phase8 units (scaled by 256 / 2pi):
    d_phase8 = d * 256.0 / TAU
    # Scaled with 40M slope (equivalent to 50M * 2):
    n = d_phase8 * 6.0
    ref = np.clip(np.where(n < 0, -(-n + 2) / 4.0, (n + 2) / 4.0) + PEDESTAL, 0.0, 63.0)
    return ref


def analyze_candidate(dac: np.ndarray, ref: np.ndarray, label: str) -> dict:
    min_len = min(len(dac), len(ref))
    d = dac[:min_len].astype(np.float64)
    r = ref[:min_len].astype(np.float64)
    
    err = np.abs(d - r)
    mae = float(np.mean(err))
    median_err = float(np.median(err))
    rms_err = float(np.sqrt(np.mean(err ** 2)))
    p95 = float(np.percentile(err, 95))
    p99 = float(np.percentile(err, 99))
    max_err = float(np.max(err))
    
    hard_8 = float(np.mean(err >= 8.0))
    hard_16 = float(np.mean(err >= 16.0))
    hard_32 = float(np.mean(err >= 32.0))
    
    n_levels = int(len(np.unique(dac)))
    clip_low = float(np.mean(dac == 0))
    clip_high = float(np.mean(dac == 63))
    
    return {
        "label": label,
        "samples": min_len,
        "mae": mae,
        "median": median_err,
        "rms": rms_err,
        "p95": p95,
        "p99": p99,
        "max": max_err,
        "hard_ge_8_pct": hard_8 * 100.0,
        "hard_ge_16_pct": hard_16 * 100.0,
        "hard_ge_32_pct": hard_32 * 100.0,
        "levels": n_levels,
        "clip_low_pct": clip_low * 100.0,
        "clip_high_pct": clip_high * 100.0,
    }


def analyze_spectral(y: np.ndarray, fs: float, tone_hz: float) -> dict:
    n = len(y)
    n_pow2 = 1 << int(math.log2(n))
    y_sub = y[:n_pow2]
    spec = 2.0 * np.abs(np.fft.rfft((y_sub - np.mean(y_sub)))) / n_pow2
    freqs = np.fft.rfftfreq(n_pow2, 1.0 / fs)
    
    k = int(round(tone_hz * n_pow2 / fs))
    k_fund = max(1, min(len(spec) - 1, k))
    peak_fund = float(spec[k_fund])
    
    k_img = int(round((20e6 - tone_hz) * n_pow2 / fs))
    if 0 <= k_img < len(spec) and peak_fund > 1e-6:
        img_dBc = float(20.0 * math.log10(max(1e-9, spec[k_img]) / peak_fund))
    else:
        img_dBc = float("nan")
        
    return {
        "fundamental_peak": peak_fund,
        "first_image_dBc": img_dBc,
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--capture", type=Path, default=Path("measurements/issue-11-cvbs/vtx_real_capture_v3.bin"))
    args = parser.parse_args()
    
    print("==================================================================")
    print("ISSUE #17: 40 MS/s DEMOD & RECONSTRUCTION COMPARATIVE BENCHMARK")
    print("==================================================================")
    
    # 1. Coherent synthetic tone test (NTSC chroma: 3.579545 MHz)
    print("\n--- TEST A: Synthetic NTSC Color Burst Carrier (3.5795 MHz) ---")
    fs_rf = 40e6
    fs_dac = 40e6
    f_chroma = 3579545.0
    t = np.arange(16384) / fs_rf
    
    dev_hz = 500e3
    phi_rf = 2.0 * np.pi * np.cumsum(dev_hz * np.cos(2.0 * np.pi * f_chroma * t)) / fs_rf
    amp = 7.0
    q = np.clip(np.rint(amp * np.sin(phi_rf)), -8, 7).astype(np.int8) & 0x0F
    i = np.clip(np.rint(amp * np.cos(phi_rf)), -8, 7).astype(np.int8) & 0x0F
    synthetic_raw = (q | (i << 4)).astype(np.uint8)
    
    ref_synth = ideal_reference_40m(synthetic_raw)
    
    c1_synth = demod_phase5_hold40(synthetic_raw)
    c2_synth = demod_hw_true40(synthetic_raw)
    c3_synth = demod_true40(synthetic_raw)
    c4_synth = demod_linear40(synthetic_raw)
    c5_synth = demod_polyphase40(synthetic_raw)
    
    for cand, name in [
        (c1_synth, "Phase5 Hold40 [A, A] (Baseline)"),
        (c2_synth, "HW True40 2-Bundle (Issue 17 Core)"),
        (c3_synth, "True 40->40 Demod (Adjacent Reference)"),
        (c4_synth, "Linear40 [A, (A+B)/2]"),
        (c5_synth, "Polyphase40 (Catmull-Rom)"),
    ]:
        res = analyze_candidate(cand, ref_synth, name)
        spec = analyze_spectral(cand, fs_dac, f_chroma)
        print(f"\n{name}:")
        print(f"  MAE: {res['mae']:.2f} codes | RMS: {res['rms']:.2f} | Median: {res['median']:.2f} | P99: {res['p99']:.2f} | Max: {res['max']:.2f}")
        print(f"  Hard Error Tail: >=8 codes: {res['hard_ge_8_pct']:.2f}% | >=16 codes: {res['hard_ge_16_pct']:.2f}%")
        print(f"  Levels: {res['levels']} | 1st Image (16.42 MHz): {spec['first_image_dBc']:.2f} dBc")

    # 2. Real Physical RF Capture Test
    if args.capture.exists():
        print(f"\n--- TEST B: Physical RF Capture ({args.capture.name}) ---")
        raw_bytes = args.capture.read_bytes()
        if len(raw_bytes) > 64 and raw_bytes[:4] == b'CBV1':
            capture_raw = np.frombuffer(raw_bytes[64:], dtype=np.uint8)
        else:
            capture_raw = np.frombuffer(raw_bytes, dtype=np.uint8)
            
        print(f"Loaded {len(capture_raw)} raw Q4/I4 bytes from {args.capture}")
        ref_cap = ideal_reference_40m(capture_raw)
        
        c1_cap = demod_phase5_hold40(capture_raw)
        c2_cap = demod_hw_true40(capture_raw)
        c3_cap = demod_true40(capture_raw)
        c4_cap = demod_linear40(capture_raw)
        c5_cap = demod_polyphase40(capture_raw)
        
        results = []
        for cand, name in [
            (c1_cap, "Phase5 Hold40 [A, A] (Baseline)"),
            (c2_cap, "HW True40 2-Bundle (Issue 17 Core)"),
            (c3_cap, "True 40->40 Demod (Adjacent Reference)"),
            (c4_cap, "Linear40 [A, (A+B)/2] (Reconstruction)"),
            (c5_cap, "Polyphase40 (Band-limited Spline)"),
        ]:
            res = analyze_candidate(cand, ref_cap, name)
            results.append(res)
            print(f"\n{name}:")
            print(f"  MAE: {res['mae']:.3f} codes | RMS: {res['rms']:.3f} | Median: {res['median']:.3f} | P99: {res['p99']:.2f} | Max: {res['max']:.2f}")
            print(f"  Hard Error Tail: >=8 codes: {res['hard_ge_8_pct']:.2f}% | >=16: {res['hard_ge_16_pct']:.2f}% | >=32: {res['hard_ge_32_pct']:.2f}%")
            print(f"  Levels: {res['levels']} | Clipping: low(0): {res['clip_low_pct']:.2f}%, high(63): {res['clip_high_pct']:.2f}%")
            
        out_json = Path("measurements/issue17_comparison.json")
        out_json.parent.mkdir(parents=True, exist_ok=True)
        out_json.write_text(json.dumps(results, indent=2))
        print(f"\nSaved full benchmark results to {out_json}")


if __name__ == "__main__":
    main()
