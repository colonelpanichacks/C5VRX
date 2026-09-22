import math
from pathlib import Path
import numpy as np

NTSC_FSC = 3579545.454545
FS = 20_000_000.0

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

def demod_ideal_float(q, i):
    phase = np.arctan2(q, i)
    delta = (np.diff(phase) + np.pi) % (2 * np.pi) - np.pi
    n_out = len(delta) // 2
    boxcar = 0.5 * (delta[0:2*n_out:2] + delta[1:2*n_out:2])
    scaled = boxcar * (256.0 / (2.0 * np.pi)) * (3.0 / 4.0)
    out = np.clip(20.0 + scaled, 0, 63)
    return out

def demod_trajectory(raw, traj_lut):
    p4_table = (traj_lut[:256] >> 8) & 0x0F
    out = []
    prev_p4 = 0
    for k in range(len(raw) // 2):
        m = raw[2*k]
        c = raw[2*k+1]
        qm = ((int(m) >> 3) & 1) | ((int(m) >> 6) & 2)
        cp4 = p4_table[c]
        addr = prev_p4 | (qm << 4) | (cp4 << 6)
        out.append(traj_lut[addr] & 0x3F)
        prev_p4 = cp4
    return np.array(out, dtype=np.float64)

def demod_phase5(raw, ped=28):
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
    return np.array(out, dtype=np.float64)

def load_traj_lut():
    lut_lines = open('main/trajectory_lut.h').read().splitlines()
    nums = []
    for line in lut_lines:
        line = line.strip()
        if line.startswith('/*') or line.startswith('#') or 'c5vrx2_trajectory_lut' in line or '};' in line:
            continue
        nums.extend([int(x.strip()) for x in line.split(',') if x.strip()])
    return np.array(nums, dtype=np.uint16)

def analyze_bursts(samples, label):
    smooth = np.convolve(samples, np.ones(5)/5, 'same')
    edges = np.flatnonzero((smooth[:-1] >= 10.5) & (smooth[1:] < 10.5))
    valid = []
    for e in edges:
        if len(valid) == 0 or (e - valid[-1] > 1100):
            valid.append(e)

    print(f"\n=================================================================")
    print(f" Demodulator: {label}")
    print(f"=================================================================")
    print(f"{'Line':<8} {'Edge (sample)':<15} {'Line len (us)':<15} {'Burst Amp (codes)':<18} {'Burst Phase (deg)':<18}")
    print("-" * 75)

    results = []
    for idx, edge in enumerate(valid):
        if idx > 0:
            llen_us = (edge - valid[idx-1]) / FS * 1e6
        else:
            llen_us = float('nan')

        a = int(edge + 5.3e-6 * FS)
        b = int(edge + 7.8e-6 * FS)
        if b >= len(samples):
            continue
        t = (np.arange(a, b) - edge) / FS
        y = samples[a:b]
        
        phase_ref = 2.0 * np.pi * NTSC_FSC * t
        fit = np.linalg.lstsq(np.c_[np.ones(len(t)), np.cos(phase_ref), np.sin(phase_ref)], y, rcond=None)[0]
        amp = np.hypot(fit[1], fit[2])
        phi = np.degrees(np.arctan2(-fit[2], fit[1]))
        results.append((idx + 1, edge, llen_us, amp, phi))
        print(f"line {idx+1:<4} {edge:<15} {llen_us:<15.2f} {amp:<18.2f} {phi:<+18.2f}")

    odd_phases = [r[4] for r in results if r[0] % 2 == 1 and r[3] > 1.0]
    even_phases = [r[4] for r in results if r[0] % 2 == 0 and r[3] > 1.0]

    def circ_mean(deg_list):
        if not deg_list:
            return float('nan')
        rads = np.radians(deg_list)
        return np.degrees(np.arctan2(np.sum(np.sin(rads)), np.sum(np.cos(rads))))

    def circ_std(deg_list):
        if not deg_list:
            return float('nan')
        rads = np.radians(deg_list)
        R = np.hypot(np.sum(np.sin(rads)), np.sum(np.cos(rads))) / len(deg_list)
        return np.degrees(np.sqrt(max(0, -2.0 * np.log(max(R, 1e-10)))))

    mean_odd = circ_mean(odd_phases)
    mean_even = circ_mean(even_phases)
    std_odd = circ_std(odd_phases)
    std_even = circ_std(even_phases)
    diff = (mean_even - mean_odd + 180.0) % 360.0 - 180.0

    print("-" * 75)
    print(f"odd-line  gemiddelde burstfase: {mean_odd:+7.2f}°  (std = {std_odd:.2f}°, N={len(odd_phases)})")
    print(f"even-line gemiddelde burstfase: {mean_even:+7.2f}°  (std = {std_even:.2f}°, N={len(even_phases)})")
    print(f"verschil  odd <-> even:        {diff:+7.2f}°")
    print("=================================================================")
    return results, mean_odd, mean_even, diff

if __name__ == '__main__':
    cap_path = Path('measurements/issue-11-cvbs/vtx_real_capture_v3.bin')
    raw, q, i = load_rf(cap_path)
    
    # 1. Ideal float
    ideal_out = demod_ideal_float(q, i)
    analyze_bursts(ideal_out, "1. Ideal Floating-Point Adjacent FM (40 MS/s -> boxcar 2:1)")

    # 2. PR16 Trajectory LUT
    traj_lut = load_traj_lut()
    traj_out = demod_trajectory(raw, traj_lut)
    analyze_bursts(traj_out, "2. PR16 Trajectory LUT (Hardware Production)")

    # 3. Phase 5 Polar (pedestal 28 - what was running when user saw bars)
    p5_out = demod_phase5(raw, ped=28)
    analyze_bursts(p5_out, "3. Phase 5 Polar LUT (pedestal 28 - commit e0f53c8)")
