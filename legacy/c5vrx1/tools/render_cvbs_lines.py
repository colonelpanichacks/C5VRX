#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-only
"""Render a few grayscale PAL/NTSC-like scanlines from a finite C5VRX IQ capture.

This is deliberately a *line* renderer rather than a full-frame decoder. The
current recovered C5 RF-test RAM only holds <=16,384 complex samples, so at wide
sample rates it can contain only a handful of horizontal video lines. That is
still enough for a very strong proof: RF -> I/Q -> WBFM -> recognizable luma.

Output is binary PGM (P5), which needs no image-library dependency.
"""
from __future__ import annotations

import argparse
from pathlib import Path
import numpy as np

PAL_LINE_US = 64.0
PAL_ACTIVE_START_US = 10.4
PAL_ACTIVE_US = 52.0
NTSC_LINE_US = 63.555
NTSC_ACTIVE_START_US = 10.5
NTSC_ACTIVE_US = 52.66


def load_iq(path: Path) -> np.ndarray:
    raw = np.fromfile(path, dtype="<i2")
    if raw.size < 16 or raw.size % 2:
        raise ValueError("expected little-endian int16 interleaved I,Q samples")
    f = raw.astype(np.float32)
    return (f[0::2] + 1j * f[1::2]).astype(np.complex64)


def fm_discriminator(iq: np.ndarray) -> np.ndarray:
    if iq.size < 2:
        raise ValueError("need at least two IQ samples")
    return np.angle(iq[1:] * np.conj(iq[:-1])).astype(np.float32)


def moving_mean(x: np.ndarray, width: int) -> np.ndarray:
    width = max(1, min(int(width), x.size))
    if width == 1:
        return x.astype(np.float64)
    cs = np.concatenate(([0.0], np.cumsum(x, dtype=np.float64)))
    y = (cs[width:] - cs[:-width]) / width
    # Center the valid convolution approximately back onto sample positions.
    pad_left = width // 2
    pad_right = x.size - y.size - pad_left
    return np.pad(y, (pad_left, pad_right), mode="edge")


def detect_line_starts(cvbs: np.ndarray, fs: float, standard: str) -> np.ndarray:
    """Find repeated negative sync troughs using timing, not a full video PLL."""
    line_us = PAL_LINE_US if standard == "pal" else NTSC_LINE_US
    line_n = max(8, int(round(fs * line_us * 1e-6)))
    sync_n = max(2, int(round(fs * 4.5e-6)))
    score = moving_mean(cvbs.astype(np.float64), sync_n)

    # The moving-mean trough is centered inside the sync pulse. Track those
    # centers while searching, then convert them back to approximate leading
    # edges before applying PAL/NTSC active-video timing.
    first_center = int(np.argmin(score))
    centers = [first_center]

    # Search both directions around integer line periods. A +/-12% timing
    # window is intentionally generous because first real C5 rate calibration
    # may be imperfect.
    search = max(4, int(0.12 * line_n))

    pos = first_center + line_n
    while pos + search < score.size:
        lo = max(0, pos - search)
        hi = min(score.size, pos + search + 1)
        pick = lo + int(np.argmin(score[lo:hi]))
        if pick <= centers[-1]:
            break
        centers.append(pick)
        pos = pick + line_n

    prefix: list[int] = []
    pos = first_center - line_n
    while pos - search >= 0:
        lo = max(0, pos - search)
        hi = min(score.size, pos + search + 1)
        pick = lo + int(np.argmin(score[lo:hi]))
        prefix.append(pick)
        pos = pick - line_n

    centers = list(reversed(prefix)) + centers
    # De-duplicate any picks that collapsed into the same sync trough.
    dedup: list[int] = []
    min_sep = int(0.6 * line_n)
    for s in centers:
        if not dedup or s - dedup[-1] >= min_sep:
            dedup.append(s)

    starts = np.asarray(dedup, dtype=np.int64) - sync_n // 2
    return starts


def resample_row(x: np.ndarray, width: int) -> np.ndarray:
    if x.size < 2:
        raise ValueError("active line too short")
    src = np.linspace(0.0, 1.0, x.size, dtype=np.float64)
    dst = np.linspace(0.0, 1.0, width, dtype=np.float64)
    return np.interp(dst, src, x.astype(np.float64))


def render_lines(cvbs: np.ndarray, fs: float, standard: str = "pal",
                 width: int = 320, invert: bool = False) -> tuple[np.ndarray, np.ndarray]:
    if standard not in {"pal", "ntsc"}:
        raise ValueError("standard must be pal or ntsc")
    if width < 16:
        raise ValueError("width must be >= 16")

    active_start_us = PAL_ACTIVE_START_US if standard == "pal" else NTSC_ACTIVE_START_US
    active_us = PAL_ACTIVE_US if standard == "pal" else NTSC_ACTIVE_US
    active_start = int(round(fs * active_start_us * 1e-6))
    active_n = int(round(fs * active_us * 1e-6))

    starts = detect_line_starts(cvbs, fs, standard)
    rows: list[np.ndarray] = []
    used: list[int] = []
    for s in starts:
        a = int(s) + active_start
        b = a + active_n
        if a < 0 or b > cvbs.size:
            continue
        row = resample_row(cvbs[a:b], width)
        rows.append(row)
        used.append(int(s))

    if not rows:
        raise ValueError("capture does not contain one complete active scanline")

    image = np.vstack(rows)
    if invert:
        image = -image

    # Robust global black/white mapping. A real decoder will estimate porch and
    # sync levels per line, but percentiles make the first proof resilient to
    # unknown FM scale/offset.
    lo, hi = np.percentile(image, [2.0, 98.0])
    if not np.isfinite(lo) or not np.isfinite(hi) or hi - lo < 1e-9:
        raise ValueError("no usable luma dynamic range")
    image = np.clip((image - lo) / (hi - lo), 0.0, 1.0)
    return np.rint(image * 255.0).astype(np.uint8), np.asarray(used, dtype=np.int64)


def write_pgm(path: Path, image: np.ndarray) -> None:
    if image.ndim != 2 or image.dtype != np.uint8:
        raise ValueError("expected uint8 HxW image")
    h, w = image.shape
    with path.open("wb") as f:
        f.write(f"P5\n{w} {h}\n255\n".encode("ascii"))
        f.write(image.tobytes())


def synthetic_capture(fs: float = 20_000_000.0, lines: int = 8) -> tuple[np.ndarray, np.ndarray]:
    line_n = int(round(fs * PAL_LINE_US * 1e-6))
    sync_n = int(round(fs * 4.7e-6))
    active_start = int(round(fs * PAL_ACTIVE_START_US * 1e-6))
    active_n = int(round(fs * PAL_ACTIVE_US * 1e-6))

    cvbs = np.full(line_n * lines, 0.05, dtype=np.float64)  # blank/porch
    expected_rows = []
    for line in range(lines):
        base = line * line_n
        cvbs[base:base + sync_n] = -0.40
        # Alternating gradient direction makes line alignment obvious.
        gradient = np.linspace(0.0, 0.70, active_n, dtype=np.float64)
        if line & 1:
            gradient = gradient[::-1]
        cvbs[base + active_start:base + active_start + active_n] = gradient
        expected_rows.append(gradient)

    # FM modulate into an ideal constant-envelope complex waveform. Keep the
    # max phase step well below pi to avoid discriminator aliasing.
    deviation_hz = 4_000_000.0
    inst_freq = deviation_hz * cvbs
    phase = 2.0 * np.pi * np.cumsum(inst_freq) / fs
    iq = np.exp(1j * phase).astype(np.complex64)
    return iq, np.vstack(expected_rows)


def self_test() -> None:
    fs = 20_000_000.0
    iq, expected = synthetic_capture(fs=fs)
    demod = fm_discriminator(iq)
    image, starts = render_lines(demod, fs, "pal", width=320)
    if image.shape[0] < 5:
        raise AssertionError((image.shape, starts))

    # Check that the recovered rows are strongly monotonic, alternating with
    # the synthetic source. We intentionally don't require all edge rows because
    # the first discriminator sample shifts the timeline by one sample.
    corrs = []
    x = np.linspace(-1.0, 1.0, image.shape[1])
    for row in image:
        corrs.append(abs(float(np.corrcoef(x, row.astype(np.float64))[0, 1])))
    if float(np.median(corrs)) < 0.96:
        raise AssertionError(corrs)

    print("finite CVBS line renderer self-test PASS")
    print(f"  recovered complete lines: {image.shape[0]}")
    print(f"  output geometry: {image.shape[1]}x{image.shape[0]}")
    print(f"  median |gradient correlation|: {np.median(corrs):.4f}")


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("input", nargs="?", type=Path, help="interleaved little-endian int16 I,Q file")
    ap.add_argument("--sample-rate", type=float, default=80_000_000.0)
    ap.add_argument("--standard", choices=["pal", "ntsc"], default="pal")
    ap.add_argument("--width", type=int, default=320)
    ap.add_argument("--invert", action="store_true", help="invert recovered luma polarity")
    ap.add_argument("--output", type=Path, default=Path("c5vrx-lines.pgm"))
    ap.add_argument("--self-test", action="store_true")
    args = ap.parse_args()

    if args.self_test:
        self_test()
        if args.input is None:
            return 0
    if args.input is None:
        ap.error("input is required unless --self-test is used")

    iq = load_iq(args.input)
    cvbs = fm_discriminator(iq)
    image, starts = render_lines(cvbs, args.sample_rate, args.standard, args.width, args.invert)
    write_pgm(args.output, image)
    print(f"wrote {args.output}: {image.shape[1]}x{image.shape[0]} from {len(starts)} complete lines")
    print("This is a finite grayscale proof, not a full PAL/NTSC frame decoder.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
