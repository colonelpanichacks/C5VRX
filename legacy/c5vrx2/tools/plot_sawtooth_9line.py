import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from pathlib import Path
import numpy as np
import sys
sys.path.insert(0, '.')
sys.path.insert(0, 'tools')
import analyze_active_chroma as aac

cap_path = Path('measurements/issue-11-cvbs/vtx_real_capture_v3.bin')
raw, q, i = aac.load_rf(cap_path)

phase = np.arctan2(q, i)
delta = (np.diff(phase) + np.pi) % (2.0 * np.pi) - np.pi
n_pairs = (len(delta) - 1) // 2
d_odd = delta[1:1+2*n_pairs:2] + delta[2:1+2*n_pairs:2]
ideal = np.clip(20.0 + d_odd * (256.0 / (2*np.pi)) * (3.0 / 4.0), 0, 63)
p5 = aac.demod_phase5(raw, ped=20)[:len(ideal)]

sync_lpf = np.ones(21) / 21.0
id_lpf = np.convolve(ideal, sync_lpf, 'same')
p5_lpf = np.convolve(p5, sync_lpf, 'same')

thresh = 7.5

def get_exact_edges(sig_lpf):
    edges = np.flatnonzero((sig_lpf[:-1] >= thresh) & (sig_lpf[1:] < thresh))
    valid = []
    for e in edges:
        if len(valid) == 0 or (e - valid[-1] > 1100):
            y0, y1 = sig_lpf[e], sig_lpf[e+1]
            frac = (y0 - thresh) / (y0 - y1)
            valid.append(e + frac)
    return valid

edges_id = get_exact_edges(id_lpf)
edges_p5 = get_exact_edges(p5_lpf)

N = min(len(edges_id), len(edges_p5))
t0_id = edges_id[0]
t0_p5 = edges_p5[0]

k = np.arange(N)
mod9 = k % 9

H_samples = 11440.0 / 9.0  # exactly 1271 + 1/9

theo_frac_sample = (k * (1.0 / 9.0) + (t0_id % 1.0)) % 1.0
theo_sawtooth_ns = ((k * (1.0 / 9.0)) % 1.0) * 50.0

meas_frac_id = np.array(edges_id) % 1.0
meas_frac_p5 = np.array(edges_p5) % 1.0

meas_sawtooth_id_ns = ((meas_frac_id - meas_frac_id[0] + 1.0) % 1.0) * 50.0
meas_sawtooth_p5_ns = ((meas_frac_p5 - meas_frac_p5[0] + 1.0) % 1.0) * 50.0

print('========================================================================================================')
print(' KILLER TEST: H-SYNC SUB-SAMPLE EDGE PHASE vs LINE_NUMBER % 9')
print('========================================================================================================')
print(f"{'Line':<5} {'Line%9':<8} {'Theo Sawtooth':<18} {'Meas Id (ns)':<16} {'Meas P5 (ns)':<16} {'Theo Frac':<14} {'Meas Id Frac':<14}")
print('--------------------------------------------------------------------------------------------------------')
for i in range(N):
    print(f"L{i+1:<4d} {mod9[i]:<8d} {theo_sawtooth_ns[i]:5.2f} ns           {meas_sawtooth_id_ns[i]:5.2f} ns         {meas_sawtooth_p5_ns[i]:5.2f} ns         {theo_frac_sample[i]:5.3f} smp       {meas_frac_id[i]:5.3f} smp")

print('========================================================================================================')

plt.figure(figsize=(12, 7))

plt.subplot(2, 1, 1)
plt.plot(k + 1, theo_sawtooth_ns, 'r--', marker='o', linewidth=2, label='Theoretical Sawtooth: (k % 9) * 5.55 ns')
plt.plot(k + 1, meas_sawtooth_id_ns, 'b-s', linewidth=2, markersize=8, label='Measured Ideal Float CVBS')
plt.plot(k + 1, meas_sawtooth_p5_ns, 'g:^', linewidth=2, markersize=8, label='Measured Phase 5 Quality CVBS')
plt.title('H-Sync Edge Sub-Sample Shift vs Line Number (50 ns / 9 = 5.55 ns per line)', fontsize=13, fontweight='bold')
plt.xlabel('Line Number', fontsize=11)
plt.ylabel('Sub-Sample Offset (ns)', fontsize=11)
plt.xticks(k + 1, [f'L{i+1}\n(m{mod9[i]})' for i in range(N)])
plt.yticks(np.linspace(0, 50, 11))
plt.grid(True, linestyle='--', alpha=0.5)
plt.legend(fontsize=11, loc='upper left')

plt.subplot(2, 1, 2)
# Average measured sawtooth per mod9 bin
mod_avg_id = [np.mean([meas_sawtooth_id_ns[i] for i in range(N) if mod9[i] == m]) for m in range(9)]
mod_avg_p5 = [np.mean([meas_sawtooth_p5_ns[i] for i in range(N) if mod9[i] == m]) for m in range(9)]
mod_theo = [m * (50.0 / 9.0) for m in range(9)]

plt.plot(range(9), mod_theo, 'r--', marker='o', linewidth=2, label='Theoretical: mod * 5.55 ns')
plt.plot(range(9), mod_avg_id, 'b-s', linewidth=2, markersize=8, label='Measured Ideal Float (mean per mod9)')
plt.plot(range(9), mod_avg_p5, 'g:^', linewidth=2, markersize=8, label='Measured Phase 5 (mean per mod9)')
plt.title('Sub-Sample Offset vs Line Number % 9 (Modulo-9 Sawtooth Average)', fontsize=13, fontweight='bold')
plt.xlabel('Line Number % 9', fontsize=11)
plt.ylabel('Sub-Sample Offset (ns)', fontsize=11)
plt.xticks(range(9), [f'mod {m}\n({m*5.55:.1f} ns)' for m in range(9)])
plt.yticks(np.linspace(0, 50, 11))
plt.grid(True, linestyle='--', alpha=0.5)
plt.legend(fontsize=11, loc='upper left')

plt.tight_layout()
out_png = Path('C:/Users/leonb/.gemini/antigravity-cli/brain/d53f0852-6c10-48a8-b452-306daa49e236/hsync_edge_vs_mod9.png')
plt.savefig(out_png, dpi=150)
print('Saved plot to:', out_png)
