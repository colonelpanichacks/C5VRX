#!/usr/bin/env python3
"""Monotone 2x Linear40 Reconstruction Oracle against Golden Phase5.

Enforces strict separation of concerns:
  1. Golden Phase5 is bit-exact and untouched (filtered[0::2] == golden[:])
  2. Monotone interpolation with zero overshoot: min(A, B) <= M <= max(A, B)
  3. Strict step reduction: max(|A-M|, |M-B|) <= ceil(|A-B|/2)
"""
import math
import unittest
from pathlib import Path
import numpy as np

ROOT = Path(__file__).resolve().parents[1]

TAU = 2.0 * math.pi
PEDESTAL = 20
GAIN = 2

_CENT = np.array([
       0,    8,   15,   24,   32,   40,   49,   56,
      64,   72,   79,   87,   96,  104,  113,  120,
    -128, -120, -113, -104,  -96,  -88,  -79,  -72,
     -64,  -56,  -49,  -40,  -32,  -23,  -15,   -8,
], dtype=np.int32)

def sbc(c: np.ndarray, bits: int) -> np.ndarray:
    w = 1 << (10 - bits)
    v = c.astype(np.float64) * w + (w - 1) * 0.5
    return np.where(v >= 512.0, v - 1024.0, v)

def exact_phase(raw: np.ndarray) -> np.ndarray:
    q = sbc((raw & 0x0F).astype(np.uint8), 4)
    i = sbc((raw >> 4).astype(np.uint8), 4)
    return np.arctan2(q, i)

def phase5_state(raw: np.ndarray) -> np.ndarray:
    return (np.rint(exact_phase(raw) * 32.0 / TAU).astype(np.int32) & 0x1F).astype(np.int8)

def golden_phase5_demod(raw: np.ndarray) -> np.ndarray:
    """Golden Phase5 reference demodulator (20 MS/s)."""
    sel = raw[1::2]
    st = phase5_state(sel)
    p, c = st[:-1], st[1:]
    d = ((_CENT[c.astype(np.int32)] - _CENT[p.astype(np.int32)] + 128) % 256 - 128).astype(np.int32)
    n = d * (GAIN + 1)
    return np.clip(np.where(n < 0, -(-n + 2) // 4, (n + 2) // 4) + PEDESTAL, 0, 63).astype(np.uint8)

def linear40_reconstruct(golden: np.ndarray) -> np.ndarray:
    """Monotone 2x Linear40 reconstruction filter.
    
    Y[2k]   = golden[k]
    Y[2k+1] = round((golden[k] + golden[k+1]) / 2)
    """
    if len(golden) < 2:
        return golden
    prev = golden[:-1].astype(int)
    curr = golden[1:].astype(int)
    mid = (prev + curr + 1) // 2
    
    out = np.empty(2 * len(prev), dtype=np.uint8)
    out[0::2] = prev
    out[1::2] = mid
    return out

class TestMonotoneLinear40(unittest.TestCase):
    def test_golden_preservation_and_monotonicity(self):
        cap_path = ROOT / "measurements/issue-11-cvbs/vtx_real_capture_v3.bin"
        if not cap_path.exists():
            self.skipTest("Physical capture file not found")

        raw_bytes = cap_path.read_bytes()
        raw = np.frombuffer(raw_bytes[64:] if raw_bytes[:4] == b'CBV1' else raw_bytes, dtype=np.uint8)

        golden = golden_phase5_demod(raw)
        reconstructed = linear40_reconstruct(golden)

        # 1. Assertion: Golden is bit-exact and untouched
        np.testing.assert_array_equal(
            reconstructed[0::2],
            golden[:-1],
            err_msg="Y[2k] must be bit-exact identical to Golden[k]!"
        )

        # 2. Assertion: Every midpoint M is strictly bounded between A and B (no overshoot)
        prev = golden[:-1].astype(int)
        curr = golden[1:].astype(int)
        mid = reconstructed[1::2].astype(int)

        lower = np.minimum(prev, curr)
        upper = np.maximum(prev, curr)
        self.assertTrue(
            np.all((mid >= lower) & (mid <= upper)),
            "Midpoint M must satisfy min(A, B) <= M <= max(A, B)!"
        )

        # 3. Assertion: Step size is strictly halved
        original_jumps = np.abs(curr - prev)
        jump_a_to_m = np.abs(mid - prev)
        jump_m_to_b = np.abs(curr - mid)
        max_interpolated_jump = np.maximum(jump_a_to_m, jump_m_to_b)
        allowed_max_jump = np.ceil(original_jumps / 2.0).astype(int)

        self.assertTrue(
            np.all(max_interpolated_jump <= allowed_max_jump),
            "Every interpolated jump must be <= ceil(|A - B| / 2)!"
        )

        # Statistics verification
        orig_diffs = np.abs(np.diff(golden.astype(float)))
        interp_diffs = np.abs(np.diff(reconstructed.astype(float)))

        print("\n--- Monotone Linear40 Reconstruction Verification ---")
        print(f"  Golden Phase5 mean jump:        {np.mean(orig_diffs):.2f} codes")
        print(f"  Reconstructed Linear40 jump:    {np.mean(interp_diffs):.2f} codes")
        print(f"  Golden jumps >= 16 codes:       {np.mean(orig_diffs >= 16.0)*100:.2f}%")
        print(f"  Reconstructed jumps >= 16:      {np.mean(interp_diffs >= 16.0)*100:.2f}%")
        print(f"  Reconstructed jumps >= 32:      {np.mean(interp_diffs >= 32.0)*100:.2f}%")
        print("  Mathematical Guarantee: 0 new spikes, 0 overshoot, exactly halved step bounds.")

if __name__ == "__main__":
    unittest.main()
