#!/usr/bin/env python3
"""Search hardware-cheap raw-bit projections for a winding-aware WBFM LUT.

The two-bundle TX BitScrambler can address a 1024-entry LUT directly from
three bytes already present in its input register: previous endpoint, skipped
middle sample and current endpoint.  This tool tests symmetric 4+2+4 raw-bit
addresses against the true adjacent-FM reference and cross-validates on
alternating output samples.  It is a feasibility oracle, not a LUT trainer.
"""
from __future__ import annotations

import argparse
import itertools
from pathlib import Path

import numpy as np

from golden_demod import HEADER_BYTES, exact_phase, load, scale, wrap_r


def project(values: np.ndarray, bits: tuple[int, ...]) -> np.ndarray:
    out = np.zeros(values.shape, dtype=np.uint16)
    for dst, src in enumerate(bits):
        out |= (((values >> src) & 1).astype(np.uint16) << dst)
    return out


def fit_predict(address: np.ndarray, target: np.ndarray,
                train: np.ndarray, test: np.ndarray,
                fallback: np.ndarray) -> np.ndarray:
    items = len(fallback)
    sums = np.bincount(address[train], weights=target[train], minlength=items)
    counts = np.bincount(address[train], minlength=items)
    table = fallback.astype(np.float64).copy()
    seen = counts != 0
    table[seen] = sums[seen] / counts[seen]
    return table[address[test]]


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("capture", type=Path)
    ap.add_argument("--top", type=int, default=12)
    args = ap.parse_args()
    _, raw = load(args.capture)

    # The live TX path observes triples raw[1],raw[2],raw[3], then advances
    # two input bytes per DAC output.
    starts = np.arange(1, len(raw) - 2, 2)
    prev, middle, current = raw[starts], raw[starts + 1], raw[starts + 2]
    phi = exact_phase(raw)
    adjacent = wrap_r(np.diff(phi))
    target_phase8 = ((adjacent[starts] + adjacent[starts + 1]) *
                     (256.0 / (2.0 * np.pi)))
    target_dac = scale(np.rint(target_phase8).astype(np.int32)).astype(np.float64)

    # Address-local endpoint-only fallback, fitted over the complete capture.
    # It is only used for addresses absent from one alternating training half.
    train_even = np.arange(len(starts)) % 2 == 0
    train_odd = ~train_even
    results = []
    for endpoint_width, middle_width in ((4, 2), (4, 3), (5, 1)):
      items = 1 << (2 * endpoint_width + middle_width)
      for endpoint_bits in itertools.combinations(range(8), endpoint_width):
        p = project(prev, endpoint_bits)
        c = project(current, endpoint_bits)
        for middle_bits in itertools.combinations(range(8), middle_width):
            m = project(middle, middle_bits)
            address = p | (m << endpoint_width) | \
                      (c << (endpoint_width + middle_width))
            sums = np.bincount(address, weights=target_dac, minlength=items)
            counts = np.bincount(address, minlength=items)
            fallback = np.full(items, 20.0)
            seen = counts != 0
            fallback[seen] = sums[seen] / counts[seen]

            pred_odd = fit_predict(address, target_dac, train_even, train_odd,
                                   fallback)
            pred_even = fit_predict(address, target_dac, train_odd, train_even,
                                    fallback)
            err = np.concatenate((pred_odd - target_dac[train_odd],
                                  pred_even - target_dac[train_even]))
            mae = float(np.mean(np.abs(err)))
            hard = float(np.mean(np.abs(err) >= 16.0))
            results.append((hard, mae, endpoint_bits, middle_bits, items))

    # Two-stage 16-bit LUT candidate: first lookup yields phase4; the second
    # has phase4(previous)+phase4(current)+two direct middle-byte bits.
    phase4_all = (np.rint(phi * 16.0 / (2.0 * np.pi)).astype(np.int32) & 0xF)
    p4 = phase4_all[starts].astype(np.uint16)
    c4 = phase4_all[starts + 2].astype(np.uint16)
    for middle_bits in itertools.combinations(range(8), 2):
        m = project(middle, middle_bits)
        address = p4 | (m << 4) | (c4 << 6)
        sums = np.bincount(address, weights=target_dac, minlength=1024)
        counts = np.bincount(address, minlength=1024)
        fallback = np.full(1024, 20.0)
        seen = counts != 0
        fallback[seen] = sums[seen] / counts[seen]
        pred_odd = fit_predict(address, target_dac, train_even, train_odd,
                               fallback)
        pred_even = fit_predict(address, target_dac, train_odd, train_even,
                                fallback)
        err = np.concatenate((pred_odd - target_dac[train_odd],
                              pred_even - target_dac[train_even]))
        results.append((float(np.mean(np.abs(err) >= 16.0)),
                        float(np.mean(np.abs(err))),
                        ("phase4",), middle_bits, 1024))

    results.sort()
    baseline = scale(np.rint(wrap_r(phi[starts + 2] - phi[starts]) *
                             256.0 / (2.0 * np.pi)).astype(np.int32))
    base_err = baseline.astype(np.float64) - target_dac
    print(f"capture outputs: {len(starts)}")
    print(f"endpoint baseline: MAE={np.mean(np.abs(base_err)):.3f}, "
          f"hard>=16={np.mean(np.abs(base_err) >= 16):.3%}")
    for hard, mae, ep, mid, items in results[:args.top]:
        print(f"LUT={items} endpoint={ep} middle={mid} "
              f"MAE={mae:.3f} hard>=16={hard:.3%}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
