#!/usr/bin/env python3
"""
Active NTSC chroma analysis comparing Phase 5 (pedestal 20) with Ideal Float FM demodulation.

Measures per video line:
1. Line length and H-sync edge timing
2. 3.579545 MHz burst amplitude and phase (Ideal and Phase 5)
3. 3.579545 MHz active-video chroma amplitude and phase (Ideal and Phase 5)
4. Sample-for-sample Phase 5 error vs Ideal Float (MAE, RMS, correlation)
5. Color phase referenced to burst (Phi_active - Phi_burst) for decoder perspective
6. Line-to-line phase increments
7. Rigorous test for deterministic ~3-line / ~120 deg phase walk
8. Burst clipping / zero-count analysis at pedestal 20
"""

from pathlib import Path
import math
import numpy as np

NTSC_FSC = 3579545.454545  # Hz
FS_RAW = 40_000_000.0      # Hz
FS_OUT = 20_000_000.0      # Hz
TAU = 2.0 * math.pi

def sbc(code: np.ndarray, bits: int) -> np.ndarray:
    width = 1 << (10 - bits)
    center = (code.astype(np.float64) * width) + (width - 1) * 0.5
    center[center >= 512.0] -= 1024.0
    return center

def load_rf(path: Path):
    blob = path.read_bytes()
    raw = np.frombuffer(blob[64:64+32768], dtype=np.uint8)
    q = sbc(raw & 0x0F, 4)
    i = sbc(raw >> 4, 4)
    return raw, q, i

def demod_ideal_float(q, i, ped=20.0):
    phase = np.arctan2(q, i)
    delta = (np.diff(phase) + np.pi) % TAU - math.pi
    n_pairs = (len(delta) - 1) // 2
    # Pair sum delta[1] + delta[2] matches the exact time interval of Phase 5 (odd bytes)
    d_odd = delta[1:1+2*n_pairs:2] + delta[2:1+2*n_pairs:2]
    steps_phase8 = d_odd * (256.0 / TAU)
    scaled = steps_phase8 * (3.0 / 4.0)
    out = np.clip(ped + scaled, 0.0, 63.0)
    return out

def demod_phase5(raw, ped=20):
    import sys
    sys.path.append('tools')
    import validate_phase5_quality as p5
    lut = p5.build_lut(calibration_gain=2, pedestal=ped)
    sel = raw[1::2]
    out = []
    prev_p = 0
    for b in sel:
        curr_p = (lut[b] >> 8) & 0x1F
        dac = lut[(prev_p << 5) | curr_p] & 0x3F
        out.append(dac)
        prev_p = curr_p
    # sel[0] was raw[1] (priming step); sel[1] was raw[3] (first valid delta)
    return np.array(out[1:], dtype=np.float64)

def fit_sinusoid(t, y, freq):
    phase_ref = TAU * freq * t
    M = np.c_[np.ones(len(t)), np.cos(phase_ref), np.sin(phase_ref)]
    c0, c1, c2 = np.linalg.lstsq(M, y, rcond=None)[0]
    amp = math.hypot(c1, c2)
    phi = math.degrees(math.atan2(-c2, c1))
    return c0, amp, phi

def circ_diff(a, b):
    return (a - b + 180.0) % 360.0 - 180.0

def circ_mean(deg_list):
    if not deg_list:
        return float('nan')
    rads = np.radians(deg_list)
    return math.degrees(math.atan2(np.sum(np.sin(rads)), np.sum(np.cos(rads))))

def circ_std(deg_list):
    if not deg_list:
        return float('nan')
    rads = np.radians(deg_list)
    R = math.hypot(np.sum(np.sin(rads)), np.sum(np.cos(rads))) / len(deg_list)
    return math.degrees(math.sqrt(max(0.0, -2.0 * math.log(max(R, 1e-10)))))

def main():
    cap_path = Path('measurements/issue-11-cvbs/vtx_real_capture_v3.bin')
    raw, q, i = load_rf(cap_path)

    ideal = demod_ideal_float(q, i, ped=20.0)
    p5 = demod_phase5(raw, ped=20)

    n_samples = min(len(ideal), len(p5))
    ideal = ideal[:n_samples]
    p5 = p5[:n_samples]

    diff_all = p5 - ideal
    mae_all = np.mean(np.abs(diff_all))
    rms_all = np.sqrt(np.mean(diff_all**2))
    corr_all = np.corrcoef(p5, ideal)[0, 1]

    print("="*95)
    print(" NTSC CHROMA SAMPLE-FOR-SAMPLE VALIDATION: PHASE 5 vs IDEAL FLOAT")
    print("="*95)
    print(f"Capture: vtx_real_capture_v3.bin (32 KiB raw Q4/I4 @ 40 MS/s)")
    print(f"Demodulated CVBS: {n_samples} samples @ 20 MS/s ({n_samples/20e3:.2f} ms)")
    print(f"Global Sample-for-Sample Correlation: {corr_all*100:.2f}%")
    print(f"Global Sample-for-Sample MAE:         {mae_all:.3f} DAC codes (out of 64)")
    print(f"Global Sample-for-Sample RMS:         {rms_all:.3f} DAC codes")
    print("="*95)

    # Detect H-sync edges
    smooth = np.convolve(ideal, np.ones(5)/5, 'same')
    edges = np.flatnonzero((smooth[:-1] >= 11.0) & (smooth[1:] < 11.0))
    valid_edges = []
    for e in edges:
        if len(valid_edges) == 0 or (e - valid_edges[-1] > 1100):
            valid_edges.append(e)

    print("\nTABLE 1: LINE-BY-LINE BURST & ACTIVE CHROMA MEASUREMENTS")
    print("-"*108)
    print(f"{'Line':<5} {'Period':<10} {'Burst Amp (P5/Id)':<20} {'Burst Phase (P5/Id)':<24} {'Act Amp (P5/Id)':<20} {'Color Phi (P5/Id)':<22} {'Err':<6}")
    print("-"*108)

    lines = []
    for idx, e in enumerate(valid_edges):
        b_start = e + 106
        b_end   = e + 156
        a_start = e + 200
        a_end   = e + 1200
        if a_end >= n_samples:
            break

        period_us = (e - valid_edges[idx-1]) / FS_OUT * 1e6 if idx > 0 else 63.55

        # Burst fit (using global time reference to preserve absolute subcarrier phase)
        t_b = np.arange(b_start, b_end) / FS_OUT
        _, b_amp_p5, b_phi_p5 = fit_sinusoid(t_b, p5[b_start:b_end], NTSC_FSC)
        _, b_amp_id, b_phi_id = fit_sinusoid(t_b, ideal[b_start:b_end], NTSC_FSC)

        # Active chroma fit
        t_a = np.arange(a_start, a_end) / FS_OUT
        _, a_amp_p5, a_phi_p5 = fit_sinusoid(t_a, p5[a_start:a_end], NTSC_FSC)
        _, a_amp_id, a_phi_id = fit_sinusoid(t_a, ideal[a_start:a_end], NTSC_FSC)

        # Color phase relative to burst (decoder perspective: Phi_active - Phi_burst)
        col_p5 = circ_diff(a_phi_p5, b_phi_p5)
        col_id = circ_diff(a_phi_id, b_phi_id)
        col_err = circ_diff(col_p5, col_id)

        # Burst zero clipping count
        zeros_b = int(np.sum(p5[b_start:b_end] == 0))

        lines.append({
            'line': idx + 1,
            'period_us': period_us,
            'b_amp_p5': b_amp_p5, 'b_amp_id': b_amp_id,
            'b_phi_p5': b_phi_p5, 'b_phi_id': b_phi_id,
            'a_amp_p5': a_amp_p5, 'a_amp_id': a_amp_id,
            'a_phi_p5': a_phi_p5, 'a_phi_id': a_phi_id,
            'col_p5': col_p5, 'col_id': col_id,
            'col_err': col_err,
            'zeros_b': zeros_b,
        })

        print(f"L{idx+1:<4d} {period_us:5.2f} us   {b_amp_p5:4.2f} / {b_amp_id:4.2f} codes   {b_phi_p5:+7.1f}° / {b_phi_id:+7.1f}°   {a_amp_p5:4.2f} / {a_amp_id:4.2f} codes   {col_p5:+7.1f}° / {col_id:+7.1f}°   {col_err:+5.1f}°")

    print("-"*108)

    # Table 2: Line-to-line phase increments
    print("\nTABLE 2: LINE-TO-LINE INCREMENTS (TESTING FOR 3-LINE / 120° WALK)")
    print("-"*90)
    print(f"{'Line pair':<12} {'Burst dPhi (P5)':<18} {'Burst dPhi (Id)':<18} {'Color dPhi (P5)':<18} {'Color dPhi (Id)':<18}")
    print("-"*90)
    for i in range(len(lines) - 1):
        l1 = lines[i]
        l2 = lines[i+1]
        b_d_p5 = circ_diff(l2['b_phi_p5'], l1['b_phi_p5'])
        b_d_id = circ_diff(l2['b_phi_id'], l1['b_phi_id'])
        c_d_p5 = circ_diff(l2['col_p5'], l1['col_p5'])
        c_d_id = circ_diff(l2['col_id'], l1['col_id'])
        print(f"L{l1['line']:<2d} -> L{l2['line']:<2d}     {b_d_p5:+8.2f}°         {b_d_id:+8.2f}°         {c_d_p5:+8.2f}°         {c_d_id:+8.2f}°")

    print("-"*90)

    # Modulo-3 grouping
    print("\nTABLE 3: MODULO-3 LINE GROUPING (CHECKING FOR RED-GREEN-BLUE LAYER ROTATION)")
    print("-"*80)
    for mod in range(3):
        grp = [l for l in lines if l['line'] % 3 == mod]
        cols_p5 = [l['col_p5'] for l in grp]
        cols_id = [l['col_id'] for l in grp]
        print(f"Line % 3 == {mod} (Lines {', '.join(str(l['line']) for l in grp)}):")
        print(f"  Phase 5 Color Phase:  Mean = {circ_mean(cols_p5):+7.2f}°,  Std = {circ_std(cols_p5):.2f}°")
        print(f"  Ideal   Color Phase:  Mean = {circ_mean(cols_id):+7.2f}°,  Std = {circ_std(cols_id):.2f}°")

    # Burst clipping stats
    print("\nTABLE 4: COLOR BURST ZERO-CLIPPING AT PEDESTAL 20")
    print("-"*80)
    for l in lines:
        pct = (l['zeros_b'] / 50.0) * 100.0
        print(f"  Line {l['line']:2d}: {l['zeros_b']:2d} / 50 samples clipped to code 0 ({pct:4.1f}% of burst interval)")

if __name__ == '__main__':
    main()
