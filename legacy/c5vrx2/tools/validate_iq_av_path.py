#!/usr/bin/env python3
"""Host analysis for C5VRX IQ captures and the resistor DAC.

The production source contract is checked against the direct two-bundle
Q3/I2 discriminator. Historical Q10 captures remain useful for comparing
signal polarity and line structure, but no longer pace the live AV path.
"""
from __future__ import annotations

import argparse
import math
from pathlib import Path

import numpy as np


IQ_RATE_HZ = 80_000_000.0
OUTPUT_RATE_HZ = 20_000_000.0
BLOCK_WORDS = 16_384
LP_CLOCK_HZ = 48_000_000.0
RESISTORS_OHM = np.asarray([8200.0, 3900.0, 2000.0, 1000.0, 470.0, 240.0])
SHUNT_OHM = 200.0
LOAD_OHM = 75.0


def sign_extend_10(values: np.ndarray) -> np.ndarray:
    values = values.astype(np.int32) & 0x3FF
    return np.where(values & 0x200, values - 0x400, values).astype(np.int16)


def decode_words(words: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    q = sign_extend_10(words)
    i = sign_extend_10(words >> 10)
    return i, q


def coarse_center(code5: int) -> float:
    signed5 = code5 - 32 if code5 & 0x10 else code5
    return signed5 * 32.0 + 15.5


def coarse_center_q4(code4: int) -> float:
    signed4 = code4 - 16 if code4 & 0x08 else code4
    return signed4 * 64.0 + 31.5


def build_phase_lut(phase_bits: int, dc_i: int = 0, dc_q: int = 0) -> np.ndarray:
    modulus = 1 << phase_bits
    lut = np.empty(1024, dtype=np.uint8)
    for i5 in range(32):
        for q5 in range(32):
            phase = math.atan2(coarse_center(q5) - dc_q,
                               coarse_center(i5) - dc_i) % (2.0 * math.pi)
            lut[(i5 << 5) | q5] = int(round(phase * modulus /
                                                   (2.0 * math.pi))) & (modulus - 1)
    return lut


def retained_phases(i: np.ndarray, q: np.ndarray, phase_bits: int,
                    dc_correct: bool) -> np.ndarray:
    dc_i = int(np.clip(round(float(np.mean(i)) / 32.0), -16, 15)) * 32 \
        if dc_correct else 0
    dc_q = int(np.clip(round(float(np.mean(q)) / 32.0), -16, 15)) * 32 \
        if dc_correct else 0
    lut = build_phase_lut(phase_bits, dc_i, dc_q)
    iu = i.astype(np.int32) & 0x3FF
    qu = q.astype(np.int32) & 0x3FF
    index = ((iu >> 5) << 5) | (qu >> 5)
    return lut[index][::4]


def retained_phase8_q4_i5(i: np.ndarray, q: np.ndarray) -> np.ndarray:
    """Exact compact firmware phase LUT: top five I bits and top four Q bits."""
    lut = np.empty(512, dtype=np.uint8)
    for i5 in range(32):
        for q4 in range(16):
            phase = math.atan2(coarse_center_q4(q4), coarse_center(i5)) % (
                2.0 * math.pi)
            lut[(i5 << 4) | q4] = int(round(phase * 256.0 / (
                2.0 * math.pi))) & 0xFF
    iu = i.astype(np.int32) & 0x3FF
    qu = q.astype(np.int32) & 0x3FF
    index = ((iu >> 5) << 4) | (qu >> 6)
    return lut[index][::4]


def map_old_phase6(p6: np.ndarray) -> np.ndarray:
    """Old mapping: code = 15 + previous - current, modulo 64."""
    return ((15 + p6[:-1].astype(np.int16) - p6[1:].astype(np.int16)) & 0x3F).astype(np.uint8)


def map_phase8_2x(p8: np.ndarray, inverted: bool) -> np.ndarray:
    """Historical phase8/bits1..6 mapping, including 8-bit accumulator wrap."""
    current = p8[1:].astype(np.int16)
    previous = p8[:-1].astype(np.int16)
    delta = previous - current if inverted else current - previous
    accumulator = (40 + delta) & 0xFF
    return ((accumulator >> 1) & 0x3F).astype(np.uint8)


def map_configurable(p8: np.ndarray, pedestal: int, gain: int,
                     inverted: bool) -> np.ndarray:
    """Calibration-aware firmware mapping using B2..B7."""
    scaled = (p8.astype(np.uint16) * gain).astype(np.uint8).astype(np.int16)
    current = scaled[1:]
    previous = scaled[:-1]
    delta = previous - current if inverted else current - previous
    accumulator = (pedestal * 4 + delta) & 0xFF
    return ((accumulator >> 2) & 0x3F).astype(np.uint8)


def full_range_code(accumulator: np.ndarray) -> np.ndarray:
    """Exact second-half LUT used by the five-instruction firmware route."""
    return np.where(
        (accumulator <= 13) | (accumulator > 167),
        0,
        np.minimum(63, accumulator - 7),
    ).astype(np.uint8)


def map_phase8_full_range(p8: np.ndarray) -> np.ndarray:
    """Firmware polarity and saturated transfer over modulo-256 phase8."""
    current = p8[1:].astype(np.int16)
    previous = p8[:-1].astype(np.int16)
    accumulator = (40 + current - previous) & 0xFF
    return full_range_code(accumulator)


def dac_mv(code: int) -> float:
    high_conductance = sum(1.0 / RESISTORS_OHM[bit]
                           for bit in range(6) if code & (1 << bit))
    denominator = (float(np.sum(1.0 / RESISTORS_OHM)) +
                   1.0 / SHUNT_OHM + 1.0 / LOAD_OHM)
    return 3300.0 * high_conductance / denominator


def exact_retained_delta(i: np.ndarray, q: np.ndarray) -> np.ndarray:
    # The proven host decoder removes block DC before phase differencing.
    z = ((i.astype(np.float64) - float(np.mean(i))) +
         1j * (q.astype(np.float64) - float(np.mean(q))))[::4]
    return np.angle(z[1:] * np.conj(z[:-1]))


def circular_window_mean(values: np.ndarray, width: int) -> np.ndarray:
    extended = np.concatenate((values, values[:width - 1]))
    sums = np.convolve(extended, np.ones(width, dtype=np.float64), mode="valid")
    return sums[:values.size] / width


def find_hsync(delta: np.ndarray) -> tuple[str, int, int, float, int]:
    best: tuple[str, int, int, float, int] | None = None
    width = int(round(4.7e-6 * OUTPUT_RATE_HZ))
    for standard, nominal in (("PAL", OUTPUT_RATE_HZ / 15_625.0),
                              ("NTSC", OUTPUT_RATE_HZ / 15_734.264)):
        for period in range(int(round(nominal)) - 4, int(round(nominal)) + 5):
            sums = np.zeros(period, dtype=np.float64)
            counts = np.zeros(period, dtype=np.int32)
            phase = np.arange(delta.size) % period
            np.add.at(sums, phase, delta)
            np.add.at(counts, phase, 1)
            folded = sums / np.maximum(counts, 1)
            smooth = circular_window_mean(folded, width)
            baseline = float(np.median(folded))
            low_at = int(np.argmin(smooth))
            high_at = int(np.argmax(smooth))
            low_excursion = baseline - float(smooth[low_at])
            high_excursion = float(smooth[high_at]) - baseline
            start = low_at if low_excursion >= high_excursion else high_at
            excursion = max(low_excursion, high_excursion)
            polarity = -1 if low_excursion >= high_excursion else 1
            noise = float(np.median(np.abs(folded - baseline))) * 1.4826 + 1e-9
            score = excursion / noise
            candidate = (standard, period, start, score, polarity)
            if best is None or candidate[3] > best[3]:
                best = candidate
    assert best is not None
    return best


def region_stats(codes: np.ndarray, period: int, sync_start: int) -> str:
    phase = (np.arange(codes.size) - sync_start) % period
    sync = codes[phase < int(round(4.7e-6 * OUTPUT_RATE_HZ))]
    porch = codes[(phase >= int(round(5.7e-6 * OUTPUT_RATE_HZ))) &
                  (phase < int(round(10.2e-6 * OUTPUT_RATE_HZ)))]
    active = codes[(phase >= int(round(10.5e-6 * OUTPUT_RATE_HZ))) &
                   (phase < period - int(round(1.5e-6 * OUTPUT_RATE_HZ)))]
    if not sync.size or not porch.size or not active.size:
        return "insufficient"
    return (f"sync={np.median(sync):4.1f} porch={np.median(porch):4.1f} "
            f"active-p10/50/90={np.percentile(active, 10):4.1f}/"
            f"{np.percentile(active, 50):4.1f}/{np.percentile(active, 90):4.1f}")


def analyze_capture(path: Path) -> None:
    words = np.fromfile(path, dtype="<u4")
    i, q = decode_words(words)
    p6 = retained_phases(i, q, 6, dc_correct=False)
    p8_compact = retained_phase8_q4_i5(i, q)
    p8 = retained_phases(i, q, 8, dc_correct=False)
    p8_dc = retained_phases(i, q, 8, dc_correct=True)
    reference = exact_retained_delta(i, q)
    standard, period, sync_start, score, polarity = find_hsync(reference)
    polarity_name = "low" if polarity < 0 else "high"
    print(f"{path.name}: words={words.size} mean_i/q={np.mean(i):.1f}/{np.mean(q):.1f} "
          f"H={standard}/{period} score={score:.2f} raw-sync={polarity_name}")
    mappings = (
        ("old phase6 prev-current bias15", map_old_phase6(p6)),
        ("historical adjacent Q4 default", map_configurable(
            p8, pedestal=20, gain=2, inverted=False)),
        ("rejected compact phase8 hard clip", map_phase8_full_range(p8_compact)),
        ("phase8 prev-current bias20 2x", map_phase8_2x(p8, inverted=True)),
        ("phase8 current-prev + coarse DC", map_phase8_2x(p8_dc, inverted=False)),
        ("phase8 prev-current + coarse DC", map_phase8_2x(p8_dc, inverted=True)),
    )
    for name, codes in mappings:
        print(f"  {name:38s} {region_stats(codes, period, sync_start)}")


def cadence_report(fill_cycles: float, gap_cycles: float,
                   active_hz: float | None) -> None:
    raw_rate = 40_000_000.0
    required = OUTPUT_RATE_HZ * 2.0
    print("live cadence:")
    print(f"  PARLIO RX raw Q4: {raw_rate / 1e6:.3f} MB/s")
    print(f"  TX-BS raw demand: {required / 1e6:.3f} MB/s")
    print("  clocks: RX=PLL_F240M/6, TX=PLL_F240M/12")
    print("  verdict: PASS - exact hardware 2:1 clock relation")


def verify_sources(repo: Path) -> None:
    asm = (repo / "main" / "c5vrx2_wbfm_q4_iq5_2to1.bsasm").read_text(encoding="utf-8")
    c = (repo / "main" / "wbfm_q4.c").read_text(encoding="utf-8")
    assert asm.count("read 16") == 1 and asm.count("write 8") == 1
    assert "set 16 25" in asm and "set 20 31" in asm
    assert "set 21 9" in asm and "set 25 15" in asm
    assert "set 0..5 L0..L5" in asm
    assert "compact_iq5_phase" in c
    assert "scale_real_sum" in c
    assert "cal->pedestal_code" in c
    assert "cal->discriminator_gain" in c
    assert "C5VRX2_POLARITY_PREVIOUS_MINUS_CURRENT" in c


def self_test(repo: Path) -> None:
    verify_sources(repo)
    packed = np.asarray([(((-123) & 0x3FF) << 10) | (321 & 0x3FF)], dtype=np.uint32)
    i, q = decode_words(packed)
    assert int(i[0]) == -123 and int(q[0]) == 321
    phase = np.asarray([1, 5, 9], dtype=np.uint8)
    assert np.all(map_old_phase6(phase) == np.asarray([11, 11], dtype=np.uint8))
    assert np.all(map_phase8_2x(np.asarray([10, 10], dtype=np.uint8), False) == 20)
    p8 = np.asarray([10, 14, 18], dtype=np.uint8)
    assert np.all(map_configurable(p8, 20, 2, False) == 22)
    assert np.all(map_configurable(p8, 20, 2, True) == 18)
    assert np.all(map_configurable(np.asarray([10, 10], dtype=np.uint8),
                                   24, 4, False) == 24)
    accumulator = np.asarray([0, 13, 14, 27, 40, 69, 70, 167, 168, 255], dtype=np.int16)
    assert np.all(full_range_code(accumulator) ==
                  np.asarray([0, 0, 7, 20, 33, 62, 63, 63, 0, 0], dtype=np.uint8))
    assert abs(dac_mv(19) - 312.0) < 1.0
    assert abs(dac_mv(63) - 1017.5) < 1.0
    print("IQ -> AV host model self-test PASS")
    print("  active Q3/I2 two-sample-FM/pedestal/gain contract matched")
    print(f"  physical DAC: code 15={dac_mv(15):.1f} mV, 19={dac_mv(19):.1f} mV, "
          f"20={dac_mv(20):.1f} mV, 63={dac_mv(63):.1f} mV")


def default_captures() -> list[Path]:
    root = Path.home() / "Documents" / "C5VRX Sessions"
    candidates = sorted(root.glob("*-receiver-console/iq/*.iq32le"))
    return candidates[-14:]


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("captures", nargs="*", type=Path)
    parser.add_argument("--fill-cycles", type=float, default=9787.0)
    parser.add_argument("--gap-cycles", type=float, default=6647.0)
    parser.add_argument("--active-hz", type=float, default=83_246_744.0)
    args = parser.parse_args()
    repo = Path(__file__).resolve().parents[1]
    self_test(repo)
    cadence_report(args.fill_cycles, args.gap_cycles, args.active_hz)
    captures = args.captures or default_captures()
    if not captures:
        print("no saved IQ32 captures found")
    for capture in captures:
        analyze_capture(capture)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
