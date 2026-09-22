#!/usr/bin/env python3
"""Analyze a frozen LIVE Q4/I4 ring.

Three models are evaluated side-by-side (issue #6):
  1. exact  -- floating atan2 on bucket-center Cartesian coords (production geometry)
  2. prod   -- exact mirror of production q4_phase5()
  3. pr5    -- q4_phase5_state() + centroid mapping + invalid-state handling

The previous analyzer used raw signed nibbles instead of bucket-center values,
causing 105/256 phase5 states to differ from production. All models here use
the correct geometry (signed_bucket_center as in wbfm_q4.c).
"""
from __future__ import annotations
import argparse, math, struct
from pathlib import Path
import numpy as np

TAU = 2.0 * math.pi
MAGIC = 0x31564243
HEADER_BYTES = 64
RAW_RATE_HZ = 40_000_000.0
SEL_RATE_HZ = RAW_RATE_HZ / 2.0
NTSC_SC_HZ = 3_579_545.0
PAL_SC_HZ = 4_433_618.75
NTSC_LINE_HZ = 15_734.264
RAW_BLOCK = 4096
INVALID_STATE = 31
MIN_AMP2 = 5

_CENTROIDS = np.array([
      -4,    8,   15,   24,   32,   40,   49,   56,
      64,   72,   80,   87,   96,  104,  113,  120,
    -128, -120, -113, -104,  -96,  -88,  -79,  -72,
     -64,  -56,  -49,  -40,  -32,  -23,  -16,
], dtype=np.int16)


# ---- geometry (must match wbfm_q4.c) ----------------------------------------

def signed_bucket_center(code: np.ndarray, bits: int) -> np.ndarray:
    w = 1 << (10 - bits)
    c = code.astype(np.float64) * w + (w - 1) * 0.5
    return np.where(c >= 512.0, c - 1024.0, c)


def unpack_qi(raw: np.ndarray):
    q = signed_bucket_center((raw & 0x0F).astype(np.uint8), 4)
    i = signed_bucket_center((raw >> 4).astype(np.uint8), 4)
    return q, i


def phase_exact(raw: np.ndarray) -> np.ndarray:
    q, i = unpack_qi(raw)
    return np.arctan2(q, i)


def phase5_prod(raw: np.ndarray) -> np.ndarray:
    return (np.rint(phase_exact(raw) * 32.0 / TAU).astype(np.int32) & 0x1F).astype(np.uint8)


def phase5_state(raw: np.ndarray) -> np.ndarray:
    q_s = np.where((raw & 0x0F) >= 8, (raw & 0x0F).astype(np.int16) - 16, (raw & 0x0F).astype(np.int16))
    i_s = np.where((raw >> 4) >= 8, (raw >> 4).astype(np.int16) - 16, (raw >> 4).astype(np.int16))
    mag2 = (i_s * i_s + q_s * q_s).astype(np.int32)
    st = phase5_prod(raw).astype(np.int32)
    st = np.where(st == 31, 0, st)
    return np.where(mag2 < MIN_AMP2, INVALID_STATE, st).astype(np.uint8)


def centroid_delta(prev_st: np.ndarray, curr_st: np.ndarray) -> np.ndarray:
    d = _CENTROIDS[curr_st].astype(np.int32) - _CENTROIDS[prev_st].astype(np.int32)
    return ((d + 128) % 256 - 128).astype(np.int16)


def scale_real_sum(d: np.ndarray, gain: int = 2) -> np.ndarray:
    n = d.astype(np.int32) * (gain + 1)
    return np.where(n < 0, -(-n + 2) // 4, (n + 2) // 4).astype(np.int16)


def wrap_phase5(d: np.ndarray) -> np.ndarray:
    d2 = (d & 0x1F).astype(np.int16)
    return np.where(d2 >= 16, d2 - 32, d2)


def wrap_rad(phi: np.ndarray) -> np.ndarray:
    return (phi + math.pi) % TAU - math.pi


# ---- three discriminator models ----------------------------------------------

def demod_exact(sel: np.ndarray) -> np.ndarray:
    return wrap_rad(np.diff(phase_exact(sel)))


def demod_prod(sel: np.ndarray) -> np.ndarray:
    return wrap_phase5(np.diff(phase5_prod(sel).astype(np.int16)))


def demod_pr5(sel: np.ndarray) -> np.ndarray:
    st = phase5_state(sel)
    p, c = st[:-1], st[1:]
    valid = (p != INVALID_STATE) & (c != INVALID_STATE)
    d = centroid_delta(p.clip(0, 30), c.clip(0, 30))
    return np.where(valid, d, np.int16(0))


# ---- file loading ------------------------------------------------------------

def load(path: Path):
    blob = path.read_bytes()
    if len(blob) < HEADER_BYTES:
        raise ValueError("capture too short")
    fields = struct.unpack_from("<IHHIIIIII2BHII6I", blob)
    names = ["magic","version","header_bytes","payload_bytes",
             "iq_rate_hz","cvbs_rate_hz","writer_pointer","dump_control",
             "minimum","maximum","reserved0","sample_sum","transitions"] + \
            [f"r{i}" for i in range(6)]
    hdr = dict(zip(names, fields))
    if hdr["magic"] != MAGIC or hdr["version"] != 3:
        raise ValueError("not LIVE snapshot v3")
    raw = np.frombuffer(blob, dtype=np.uint8,
                        count=hdr["payload_bytes"], offset=HEADER_BYTES).copy()
    return hdr, raw


# ---- PSD helpers -------------------------------------------------------------

def band_rms(sig: np.ndarray, rate: float, f_lo: float, f_hi: float) -> float:
    n = len(sig)
    win = np.hanning(n)
    S = np.fft.rfft((sig.astype(np.float64) - sig.mean()) * win)
    freqs = np.fft.rfftfreq(n, 1.0 / rate)
    pwr = np.abs(S) ** 2 / np.sum(win ** 2)
    mask = (freqs >= f_lo) & (freqs <= f_hi)
    return float(np.sqrt(np.mean(pwr[mask]))) if np.any(mask) else float("nan")


# ---- main -------------------------------------------------------------------

def sec(title):
    print(f"\n--- {title} ---")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("capture", type=Path)
    ap.add_argument("--psd", action="store_true", help="Print chroma PSD table")
    args = ap.parse_args()
    hdr, raw = load(args.capture)

    print("=" * 60)
    print(f"  CAPTURE: {args.capture.name}  ({raw.size} bytes)")
    print("=" * 60)
    print(f"unique raw bytes:     {np.unique(raw).size}/256")
    print(f"adjacent duplicates:  {np.count_nonzero(raw[1:] == raw[:-1])}/{raw.size-1}")

    # IQ amplitude
    sec("IQ amplitude distribution (bucket-center geometry)")
    q_bc, i_bc = unpack_qi(raw)
    mag2_bc = q_bc**2 + i_bc**2
    for thr in (5, 64, 256, 1024, 4096):
        print(f"  mag2 < {thr:5d}: {np.mean(mag2_bc < thr):.3%}")

    # Phase5 geometry fix verification (issue #6)
    sec("Phase5 geometry fix — issue #6 verification")
    idx = np.arange(256, dtype=np.uint8)
    q_old = np.where(idx & 8, (idx & 0xF).astype(np.int16) - 16, (idx & 0xF).astype(np.int16)).astype(np.float64)
    i_old = np.where(idx >> 4 & 8, (idx >> 4).astype(np.int16) - 16, (idx >> 4).astype(np.int16)).astype(np.float64)
    ph_old = (np.rint(np.arctan2(q_old, i_old) * 32.0 / TAU).astype(np.int32) & 0x1F).astype(np.uint8)
    ph_new = phase5_prod(idx)
    n_differ = int(np.sum(ph_old != ph_new))
    print(f"  states differ between old (nibble) and new (bucket-center): {n_differ}/256")
    print(f"  -> issue #6 bug {'confirmed and fixed' if n_differ > 0 else 'not reproduced'}")

    # Invalid-state stats
    all_st = phase5_state(raw)
    inv_mask = all_st == INVALID_STATE
    print(f"\nInvalid-state (PR5): {np.mean(inv_mask):.3%} of all samples")

    sec("Winding loss at the actual 40 -> 20 MS/s boundary")
    ph40 = phase_exact(raw)
    adj40 = wrap_rad(np.diff(ph40))
    starts40 = np.arange(0, len(raw) - 2, 2)
    pair_sum40 = adj40[starts40] + adj40[starts40 + 1]
    endpoint40 = wrap_rad(ph40[starts40 + 2] - ph40[starts40])
    winding40 = np.abs(pair_sum40 - endpoint40) > math.pi / 2
    q40, i40 = unpack_qi(raw)
    m240 = q40**2 + i40**2
    pm240 = np.minimum.reduce((m240[starts40], m240[starts40 + 1],
                               m240[starts40 + 2]))
    strong40 = pm240 >= (8 * 64)**2
    print(f"  n->n+2 winding loss all:       {np.mean(winding40):.3%}")
    if np.any(strong40):
        print(f"  n->n+2 winding loss strong IQ: {np.mean(winding40[strong40]):.3%}")

    for parity in (0, 1):
        sel = raw[parity::2]
        print(f"\n{'='*60}")
        print(f"  PARITY {parity}  raw[{parity}::2]  {sel.size} samples")
        print("="*60)

        sec("Three-model discriminator comparison (phase5 steps @ 20 MS/s)")
        d_exact = demod_exact(sel)
        d_prod  = demod_prod(sel).astype(np.float64)
        d_pr5   = demod_pr5(sel).astype(np.float64) / 8.0  # phase8 -> phase5

        exact_steps = d_exact * 32.0 / TAU

        for name, d in (("exact (bucket-center)", exact_steps),
                        ("prod  (phase5 delta) ", d_prod),
                        ("pr5   (centroid      )", d_pr5)):
            off_r = float(np.angle(np.mean(np.exp(1j * d * TAU / 32.0))))
            off_hz = off_r / TAU * SEL_RATE_HZ
            rms = float(np.std(d))
            p99 = float(np.percentile(np.abs(d), 99))
            p999 = float(np.percentile(np.abs(d), 99.9))
            print(f"  {name}:")
            print(f"    DC offset: {off_hz:+.0f} Hz  rms={rms:.4f}  p99={p99:.4f}  p99.9={p999:.4f}")

        sec("PR5 confidence / invalid-state analysis")
        st = phase5_state(sel)
        inv = st == INVALID_STATE
        affected = np.count_nonzero(inv[:-1] | inv[1:])
        print(f"  invalid samples:           {np.mean(inv):.3%}")
        print(f"  discrim intervals hit:     {affected}/{len(inv)-1} ({affected/max(1,len(inv)-1):.3%})")

        sec("PR5 DAC histogram")
        pr5_raw = demod_pr5(sel)
        dac = np.clip(20 + scale_real_sum(pr5_raw), 0, 63)
        print(f"  distinct codes:   {len(np.unique(dac))}")
        print(f"  sync-range (<=5): {np.mean(dac <= 5):.4%}")
        print(f"  white-range(>=58):{np.mean(dac >= 58):.4%}")
        print(f"  pedestal (=20):   {np.mean(dac == 20):.4%}")

        sec("Chroma-band noise (exact model, 20 MS/s)")
        exact_dac = np.clip(20.0 + exact_steps * 0.75, 0, 63)
        for label, sc in (("NTSC 3.58 MHz", NTSC_SC_HZ), ("PAL  4.43 MHz", PAL_SC_HZ)):
            rms_val = band_rms(exact_dac, SEL_RATE_HZ, sc - 500e3, sc + 500e3)
            print(f"  noise RMS @ {label}: {rms_val:.4f} DAC LSB")

        if args.psd:
            sec("PSD comparison (40 bins)")
            freqs, _ = np.fft.rfftfreq(len(exact_dac), 1.0/SEL_RATE_HZ), None
            freqs = np.fft.rfftfreq(len(exact_dac), 1.0/SEL_RATE_HZ)
            def _pdb(s):
                w = np.hanning(len(s))
                S = np.fft.rfft((s.astype(np.float64)-s.mean())*w)
                return 10*np.log10(np.abs(S)**2/np.sum(w**2)+1e-30)
            pdb_ex = _pdb(exact_dac)
            pdb_pr = _pdb(np.clip(20.0 + d_pr5 * 0.75, 0, 63))
            step = max(1, len(freqs) // 40)
            print(f"  {'freq_kHz':>10} {'exact_dB':>10} {'pr5_dB':>10}")
            for idx in range(0, len(freqs), step):
                print(f"  {freqs[idx]/1e3:10.1f} {pdb_ex[idx]:10.2f} {pdb_pr[idx]:10.2f}")

        sec("Block-boundary events")
        ph5 = phase5_prod(sel)
        d5 = wrap_phase5(np.diff(ph5.astype(np.int16)))
        bnd = []
        for rb in range(RAW_BLOCK, raw.size, RAW_BLOCK):
            k = (rb - parity + 1) // 2
            if 1 <= k < len(d5):
                bnd.append((rb, int(d5[k-1])))
        cyc = int(((int(ph5[0]) - int(ph5[-1]) + 16) & 31) - 16)
        print(f"  block deltas: {bnd}")
        print(f"  cyclic end->start: {cyc:+d}")

        sec("NTSC line-freq correlation")
        d_f = d_prod - float(np.mean(d_prod))
        nom = int(round(SEL_RATE_HZ / NTSC_LINE_HZ))
        best_lag, best_corr = nom, -2.0
        for lag in range(nom - 24, nom + 25):
            if lag <= 0 or lag >= len(d_f):
                continue
            l, r = d_f[:-lag], d_f[lag:]
            den = float(np.sqrt(np.dot(l,l)*np.dot(r,r)))
            sc = float(np.dot(l,r)/den) if den else 0.0
            if sc > best_corr:
                best_lag, best_corr = lag, sc
        print(f"  lag={best_lag} ({SEL_RATE_HZ/best_lag:.1f} Hz)  corr={best_corr:.6f}")


if __name__ == "__main__":
    main()
