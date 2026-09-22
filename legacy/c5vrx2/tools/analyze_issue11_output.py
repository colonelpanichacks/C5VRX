#!/usr/bin/env python3
"""Actual bounded PR10 DAC replay capture; never label this live RX/TX evidence.

Input is the 64-byte CO11 header, chronological Q4/I4, then DAC GPIO bytes.
Rates are configured rates. Burst amplitude is in DAC codes, not volts.
"""
import argparse
import csv
import hashlib
import json
import struct
from pathlib import Path
import numpy as np
from validate_trajectory import parse_asm, simulate


def fnv(data):
    h = 2166136261
    for v in data:
        h = ((h ^ int(v)) * 16777619) & 0xffffffff
    return h


def load(path):
    blob = path.read_bytes()
    if len(blob) < 64:
        raise ValueError('truncated header')
    h = struct.unpack('<16I', blob[:64])
    if h[:3] != (0x31314f43, 1, 64) or h[5:7] != (40000000, 20000000):
        raise ValueError('unsupported capture format/rates')
    if len(blob) < 64 + h[3] + h[4]:
        raise ValueError('truncated payload')
    raw = np.frombuffer(blob[64:64+h[3]], dtype=np.uint8)
    actual = np.frombuffer(blob[64+h[3]:64+h[3]+h[4]], dtype=np.uint8)
    if fnv(raw) != h[12] or fnv(actual) != h[13]:
        raise ValueError('payload hash mismatch')
    if any(h[k] for k in (7, 8, 9, 14)):
        raise ValueError(f'hardware acquisition failed: raw/tx/rx/result={[h[k] for k in (7,8,9,14)]}')
    return h, raw, actual & 63


def align(actual, expected):
    n, m = len(expected), len(actual)
    if m > n:
        raise ValueError('alignment requires capture shorter than replay period')
    padded = np.zeros(n)
    padded[:m] = actual.astype(float)
    cross = np.fft.ifft(np.fft.fft(expected) * np.conj(np.fft.fft(padded))).real
    sq = np.r_[expected.astype(float)**2, expected.astype(float)**2]
    cs = np.r_[0, np.cumsum(sq)]
    costs = cs[m:m+n] - cs[:n] + np.sum(padded**2) - 2*cross
    offsets = np.argsort(costs)[:16]
    scored = [(int(np.count_nonzero(actual != expected[(np.arange(m)+k) % n])), int(k)) for k in offsets]
    mismatch, offset = min(scored)
    return offset, mismatch, expected[(np.arange(m)+offset) % n]


def runs(mask):
    edges = np.diff(np.r_[False, mask, False].astype(int))
    return list(zip(np.flatnonzero(edges == 1), np.flatnonzero(edges == -1)))


def measure(samples, offset, period, fs, threshold=None):
    smooth = np.convolve(samples, np.ones(11)/11, 'same')
    low, high = np.percentile(smooth[20:-20], [5, 65])
    if threshold is None:
        threshold = float(low + .35*(high-low))
    candidates = [(a, b) for a, b in runs(smooth < threshold) if 50 <= b-a <= 135 and a > 20 and b < len(samples)-20]
    # Do not invent missing edges, force a grid, or bridge the replay seam.
    pulses = []
    for a, b in candidates:
        if any(1100 <= abs(a-c) <= 1450 for c, _ in candidates if c != a):
            y0, y1 = smooth[a-1:a+1]
            edge = a - 1 + (threshold-y0)/(y1-y0)
            pulses.append((float(edge), b-a))
    rows = []
    for i, (edge, width) in enumerate(pulses):
        a, b = int(np.ceil(edge + 5.5e-6*fs)), int(np.floor(edge + 7.8e-6*fs))
        if b >= len(samples):
            continue
        # Include sync and entire burst window in seam exclusion.
        if int((offset+edge-10)//period) != int((offset+b+10)//period):
            continue
        row = dict(edge_sample=edge, edge_us=edge/fs*1e6, sync_width_us=width/fs*1e6)
        if rows:
            delta = edge-rows[-1]['edge_sample']
            valid = 1100 <= delta <= 1450 and int((offset+edge)//period) == int((offset+rows[-1]['edge_sample'])//period)
            row['line_length_samples'] = delta if valid else None
            row['line_length_us'] = delta/fs*1e6 if valid else None
        else:
            row['line_length_samples'] = row['line_length_us'] = None
        t = (np.arange(a,b)-edge)/fs
        y = samples[a:b].astype(float)
        for label, freq in [('ntsc', 3579545.454545), ('pal',4433618.75)]:
            phase = 2*np.pi*freq*t
            x = np.c_[np.ones(len(t)), (t-t.mean())*fs, np.cos(phase), np.sin(phase)]
            fit = np.linalg.lstsq(x, y, rcond=None)[0]
            amp = float(np.hypot(fit[2], fit[3]))
            phi = float(np.arctan2(-fit[3], fit[2]))
            residual = float(np.sqrt(np.mean((y-x@fit)**2)))
            row.update({f'{label}_burst_peak_codes':amp,
                        f'{label}_burst_phase_at_hsync_deg':float(np.degrees(phi)),
                        f'{label}_burst_global_phase_deg':float(np.degrees(np.angle(np.exp(1j*(phi-2*np.pi*freq*edge/fs))))),
                        f'{label}_burst_residual_rms_codes':residual})
        rows.append(row)
    # Timing residual about fitted line spacing; split at missing edges/seam.
    group = []
    groups = []
    for row in rows:
        if row['line_length_samples'] is None and group:
            groups.append(group); group=[]
        group.append(row)
    if group: groups.append(group)
    for g in groups:
        edges = np.array([r['edge_sample'] for r in g])
        if len(g) >= 3:
            fitted = np.polyval(np.polyfit(np.arange(len(g)), edges, 1), np.arange(len(g)))
            for r, e in zip(g, edges-fitted): r['hsync_fit_residual_ns'] = float(e/fs*1e9)
    return rows, threshold, smooth


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('capture', type=Path)
    ap.add_argument('--threshold', type=float)
    args = ap.parse_args()
    h, raw, actual = load(args.capture)
    lut, asm = parse_asm()
    expected = simulate(np.tile(raw, 2), lut, asm)[len(raw)//2:]
    offset, mismatches, aligned = align(actual, expected)
    rows, threshold, smooth = measure(actual, offset, len(expected), h[6], args.threshold)
    lengths = [r['line_length_us'] for r in rows if r['line_length_us'] is not None]
    residuals = [r['hsync_fit_residual_ns'] for r in rows if 'hsync_fit_residual_ns' in r]
    report = {
        'capture_sha256': hashlib.sha256(args.capture.read_bytes()).hexdigest(),
        'evidence': 'actual six-bit DAC GPIO capture of bounded PR10 hardware replay; NOT simultaneous live RX/TX',
        'rate_basis': 'configured PLL-derived rates; no external timebase measurement',
        'raw_bytes':len(raw), 'output_samples':len(actual), 'raw_duration_us':len(raw)/h[5]*1e6,
        'alignment_offset':offset, 'hardware_reference_mismatches':mismatches,
        'compared_samples':len(actual), 'raw_irq_status':h[10], 'output_irq_status':h[11],
        'threshold_codes':threshold, 'detected_syncs':len(rows),
        'line_length_us': {'min':min(lengths), 'max':max(lengths), 'mean':float(np.mean(lengths)), 'std':float(np.std(lengths))} if lengths else None,
        'hsync_fit_residual_ns':{'rms':float(np.std(residuals)), 'pkpk':float(np.ptp(residuals))} if residuals else None,
        'warning':'No signal-quality conclusion if byte comparison fails. Burst fit is an estimate; weak/noisy fits do not establish color phase. Replay seam excluded; no long-run/frame claim.',
    }
    prefix = args.capture.with_suffix('')
    Path(str(prefix)+'.json').write_text(json.dumps(report, indent=2)+'\n')
    if rows:
        fields = sorted(set().union(*(r.keys() for r in rows)))
        with Path(str(prefix)+'.csv').open('w', newline='') as f:
            writer = csv.DictWriter(f, fieldnames=fields)
            writer.writeheader(); writer.writerows(rows)
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig, axes = plt.subplots(3, 1, figsize=(13,9))
    t = np.arange(len(actual))/h[6]*1e6
    axes[0].plot(t, actual, lw=.3, alpha=.5)
    axes[0].plot(t, smooth, lw=.8)
    axes[0].axhline(threshold, color='r', ls=':')
    for r in rows: axes[0].axvline(r['edge_us'], color='k', alpha=.3)
    axes[0].set(xlabel='Capture time (us; configured clock)', ylabel='DAC code', title=f'Actual PR10 replay: {mismatches}/{len(actual)} byte mismatches; NOT live RX/TX')
    for r in rows:
        start = int(r['edge_sample'])
        line = actual[start:start+1280]
        axes[1].plot(np.arange(len(line))/20, line, lw=.4, alpha=.45)
    axes[1].set(xlabel='Time since detected H-sync (us)', ylabel='DAC code', xlim=(0,12), title='H-sync and burst overlays (unaltered samples)')
    axes[2].plot(np.arange(len(actual)), actual.astype(int)-aligned.astype(int), lw=.5)
    axes[2].set(xlabel='Output sample', ylabel='Actual minus reference', title='Full capture comparison, including replay seam')
    fig.tight_layout(); fig.savefig(str(prefix)+'.png', dpi=160); plt.close(fig)
    print(json.dumps(report, indent=2))


if __name__ == '__main__':
    main()
