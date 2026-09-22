#!/usr/bin/env python3
"""Golden offline FM demodulator for issue #6.

Compares three demodulation strategies on the same frozen Q4/I4 capture:

  A. production  -- n->n+2 discriminator (current BitScrambler)
  B. adjacent    -- true adjacent 40MS/s deltas, boxcar-decimated to 20MS/s
  C. adjacent+lp -- adjacent with 3-tap symmetric FIR before decimation

Strategy B/C is what the BitScrambler would produce after a three-bundle
redesign. This script proves the quality gain before any hardware change.

Usage:
  python tools/golden_demod.py capture.bin [--write-cvbs out.bin]
"""
from __future__ import annotations
import argparse, math, struct, sys
from pathlib import Path
import numpy as np

TAU = 2.0 * math.pi
MAGIC = 0x31564243
HEADER_BYTES = 64
RAW_RATE_HZ  = 40_000_000.0
OUT_RATE_HZ  = 20_000_000.0
NTSC_SC_HZ   = 3_579_545.0
PAL_SC_HZ    = 4_433_618.75
PEDESTAL     = 20
GAIN         = 2   # calibration_gain matching firmware default
# Centroid table from wbfm_q4.c
_CENT = np.array([
       0,    8,   15,   24,   32,   40,   49,   56,
      64,   72,   79,   87,   96,  104,  113,  120,
    -128, -120, -113, -104,  -96,  -88,  -79,  -72,
     -64,  -56,  -49,  -40,  -32,  -23,  -15,   -8,
], dtype=np.int32)

# ---------------------------------------------------------------------------
# Core geometry (wbfm_q4.c production)
# ---------------------------------------------------------------------------

def sbc(code: np.ndarray, bits: int) -> np.ndarray:
    """signed_bucket_center: exact mirror of C production code."""
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


def phase5_prod(raw: np.ndarray) -> np.ndarray:
    return (np.rint(exact_phase(raw) * 32.0 / TAU).astype(np.int32) & 0x1F
            ).astype(np.int8)


def phase5_state(raw: np.ndarray) -> np.ndarray:
    return phase5_prod(raw)


def wrap5(d: np.ndarray) -> np.ndarray:
    d2 = (d.astype(np.int32) & 0x1F).astype(np.int8)
    return np.where(d2 >= 16, d2 - 32, d2).astype(np.int8)


def wrap_r(phi: np.ndarray) -> np.ndarray:
    return (phi + math.pi) % TAU - math.pi


def scale(d: np.ndarray) -> np.ndarray:
    n = d.astype(np.int32) * (GAIN + 1)
    return np.clip(np.where(n < 0, -(-n + 2) // 4, (n + 2) // 4) + PEDESTAL,
                   0, 63).astype(np.uint8)


# ---------------------------------------------------------------------------
# Three demodulation strategies
# ---------------------------------------------------------------------------

def demod_production(raw: np.ndarray) -> np.ndarray:
    """Current firmware: n->n+2 on odd bytes with all 32 phase states."""
    sel = raw[1::2]           # bytes 1,3,5,... (production parity)
    st = phase5_state(sel)
    p, c = st[:-1], st[1:]
    d = ((_CENT[c.astype(np.int32)]
          - _CENT[p.astype(np.int32)] + 128) % 256 - 128
         ).astype(np.int32)
    return scale(d)


def demod_adjacent(raw: np.ndarray,
                   lp_taps: np.ndarray | None = None) -> np.ndarray:
    """Demodulate every adjacent 40MS/s pair, then decimate 2:1.

    If lp_taps is given, apply a symmetric FIR at 40MS/s before decimation.
    This is what a three-bundle BitScrambler redesign would produce.
    """
    phi = exact_phase(raw)              # phase at 40MS/s
    d40 = wrap_r(np.diff(phi))          # adjacent delta in radians @ 40MS/s

    # optional anti-alias FIR at 40MS/s
    if lp_taps is not None:
        d40 = np.convolve(d40, lp_taps, mode='same')

    # decimate: sum pairs (equivalent to boxcar + 2:1 downsample)
    n_pairs = len(d40) // 2
    d20 = d40[:n_pairs * 2].reshape(n_pairs, 2).sum(axis=1)  # radians @ 20MS/s

    # scale() consumes phase8 units, matching wbfm_q4.c.  The old model used
    # phase5 units here and understated the adjacent result by a factor of 8.
    d20_steps = d20 * 256.0 / TAU
    d20_int = np.clip(np.round(d20_steps), -128, 127).astype(np.int32)
    return scale(d20_int)


def demod_adjacent_lp(raw: np.ndarray) -> np.ndarray:
    """Adjacent + 3-tap [0.25, 0.5, 0.25] anti-alias FIR at 40MS/s."""
    taps = np.array([0.25, 0.5, 0.25])
    return demod_adjacent(raw, lp_taps=taps)


# ---------------------------------------------------------------------------
# Metrics
# ---------------------------------------------------------------------------

def metrics(dac: np.ndarray, rate_hz: float, label: str) -> None:
    d = dac.astype(np.float64)
    rms = float(np.std(d))
    p99 = float(np.percentile(d, 99))
    p1  = float(np.percentile(d, 1))
    sync_frac = float(np.mean(dac <= 5))
    white_frac = float(np.mean(dac >= 58))
    ped_frac   = float(np.mean(dac == 20))
    n_levels   = len(np.unique(dac))

    # chroma-band noise via FFT
    n = len(d)
    win = np.hanning(n)
    S   = np.fft.rfft((d - d.mean()) * win)
    pwr = np.abs(S) ** 2 / np.sum(win ** 2)
    freqs = np.fft.rfftfreq(n, 1.0 / rate_hz)

    def band_rms(f_lo, f_hi):
        m = (freqs >= f_lo) & (freqs <= f_hi)
        return float(np.sqrt(np.mean(pwr[m]))) if np.any(m) else float('nan')

    ntsc_rms = band_rms(NTSC_SC_HZ - 500e3, NTSC_SC_HZ + 500e3)
    pal_rms  = band_rms(PAL_SC_HZ  - 500e3, PAL_SC_HZ  + 500e3)

    print(f"  {label}:")
    print(f"    levels={n_levels:3d}  rms={rms:.3f}  p1={p1:.1f}  p99={p99:.1f}")
    print(f"    sync<=5: {sync_frac:.4%}  ped=20: {ped_frac:.4%}  white>=58: {white_frac:.4%}")
    print(f"    chroma noise RMS  NTSC 3.58M: {ntsc_rms:.4f}  PAL 4.43M: {pal_rms:.4f}")


# ---------------------------------------------------------------------------
# File I/O
# ---------------------------------------------------------------------------

def load(path: Path):
    blob = path.read_bytes()
    fields = struct.unpack_from("<IHHIIIIII2BHII6I", blob)
    names  = ["magic","version","header_bytes","payload_bytes",
              "iq_rate_hz","cvbs_rate_hz","writer_pointer","dump_control",
              "minimum","maximum","reserved0","sample_sum","transitions",
              ] + [f"r{i}" for i in range(6)]
    hdr = dict(zip(names, fields))
    if hdr["magic"] != MAGIC or hdr["version"] != 3:
        raise ValueError("not a LIVE snapshot v3")
    raw = np.frombuffer(blob, dtype=np.uint8,
                        count=hdr["payload_bytes"],
                        offset=HEADER_BYTES).copy()
    return hdr, raw


def write_cvbs(path: Path, dac: np.ndarray) -> None:
    """Write raw 8-bit DAC codes for external inspection."""
    path.write_bytes(bytes(dac.clip(0, 63).astype(np.uint8)))


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawTextHelpFormatter)
    ap.add_argument("capture", type=Path)
    ap.add_argument("--write-cvbs", type=Path, metavar="OUT",
                    help="Write demodulated DAC codes as raw bytes")
    args = ap.parse_args()

    hdr, raw = load(args.capture)
    print(f"Capture: {args.capture.name}  {raw.size} bytes @ {hdr['iq_rate_hz']/1e6:.0f} MS/s")
    print()

    dac_prod  = demod_production(raw)
    dac_adj   = demod_adjacent(raw)
    dac_adj_lp = demod_adjacent_lp(raw)

    n = min(len(dac_prod), len(dac_adj), len(dac_adj_lp))
    dac_prod   = dac_prod[:n]
    dac_adj    = dac_adj[:n]
    dac_adj_lp = dac_adj_lp[:n]

    print("=== Three-strategy comparison ===")
    metrics(dac_prod,   OUT_RATE_HZ, "A: production (n->n+2, centroid)")
    metrics(dac_adj,    OUT_RATE_HZ, "B: adjacent 40MS/s, boxcar 2:1")
    metrics(dac_adj_lp, OUT_RATE_HZ, "C: adjacent 40MS/s, [.25 .5 .25] FIR")

    # Winding-loss comparison
    print()
    print("=== Winding-loss analysis ===")
    phi = exact_phase(raw)
    adj = wrap_r(np.diff(phi))
    starts = np.arange(0, len(raw) - 2, 2)
    # Preserve the unwrapped adjacent sum: its +/-2pi branch is exactly the
    # information that an endpoint-only n->n+2 discriminator loses.
    pair_sum = adj[starts] + adj[starts + 1]
    endpoint = wrap_r(phi[starts + 2] - phi[starts])
    winding  = np.abs(pair_sum - endpoint) > math.pi / 2
    q_bc, i_bc = unpack(raw)
    m2 = q_bc**2 + i_bc**2
    pm2 = np.minimum.reduce((m2[starts], m2[starts+1], m2[starts+2]))
    strong = pm2 >= (8 * 64)**2
    print(f"  n->n+2 winding loss (all):    {np.mean(winding):.3%}")
    if np.any(strong):
        print(f"  n->n+2 winding loss (strong): {np.mean(winding[strong]):.3%}")
    adj_err = np.abs(adj[starts] * 32.0 / TAU - np.round(adj[starts] * 32.0 / TAU))
    print(f"  adjacent 40MS/s quantisation error rms: {float(np.std(adj_err)):.4f} steps")

    # SNR improvement estimate (chroma band)
    prod_chroma = _chroma_rms(dac_prod, OUT_RATE_HZ)
    adj_chroma  = _chroma_rms(dac_adj,  OUT_RATE_HZ)
    if prod_chroma and adj_chroma:
        snr_gain_db = 20.0 * math.log10(prod_chroma / adj_chroma) if adj_chroma > 0 else 0
        print(f"  chroma-band noise reduction B vs A: {snr_gain_db:+.1f} dB")

    if args.write_cvbs:
        write_cvbs(args.write_cvbs, dac_adj_lp)
        print(f"\nWrote {len(dac_adj_lp)} DAC codes -> {args.write_cvbs}")


def _chroma_rms(dac: np.ndarray, rate_hz: float) -> float:
    d = dac.astype(np.float64) - dac.mean()
    n = len(d)
    win = np.hanning(n)
    S   = np.fft.rfft(d * win)
    pwr = np.abs(S)**2 / np.sum(win**2)
    freqs = np.fft.rfftfreq(n, 1.0 / rate_hz)
    m = (freqs >= NTSC_SC_HZ - 500e3) & (freqs <= NTSC_SC_HZ + 500e3)
    return float(np.sqrt(np.mean(pwr[m]))) if np.any(m) else 0.0


if __name__ == "__main__":
    main()
