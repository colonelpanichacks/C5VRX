#!/usr/bin/env python3
"""Validate the dual-purpose five-bit polar WBFM LUT and its assembly map."""

from __future__ import annotations

import argparse
import functools
import math
from pathlib import Path


TAU = 2.0 * math.pi
def signed_bucket_center(code: int, bits: int) -> float:
    width = 1 << (10 - bits)
    center = code * width + (width - 1) * 0.5
    return center - 1024.0 if center >= 512.0 else center


def wrapped(value: float) -> float:
    return (value + math.pi) % TAU - math.pi


def exact_phase(packed: int) -> float:
    q = signed_bucket_center(packed & 0x0F, 4)
    i = signed_bucket_center(packed >> 4, 4)
    return math.atan2(q, i)


def compact_iq5_phase(packed: int) -> float:
    compact = ((packed >> 1) & 0x07) | (((packed >> 6) & 0x03) << 3)
    q = signed_bucket_center(compact & 0x07, 3)
    i = signed_bucket_center(compact >> 3, 2)
    return math.atan2(q, i)


def phase5(packed: int) -> int:
    return round(exact_phase(packed) * 32.0 / TAU) & 0x1F


def phase5_state(packed: int) -> int:
    return phase5(packed)


@functools.lru_cache(maxsize=None)
def phase5_centroids() -> list[float]:
    result: list[float] = []
    for state in range(32):
        members = [exact_phase(packed) for packed in range(256)
                   if phase5_state(packed) == state]
        sine = sum(math.sin(value) for value in members)
        cosine = sum(math.cos(value) for value in members)
        result.append(math.atan2(sine, cosine))
    return result


@functools.lru_cache(maxsize=None)
def phase5_centroid_phase8() -> list[int]:
    return [round(value * 256.0 / TAU) for value in phase5_centroids()]


def centroid_delta_phase8(previous: int, current: int) -> int:
    centers = phase5_centroid_phase8()
    delta = (centers[current] - centers[previous] + 128) % 256 - 128
    return delta


def scale_real_sum(value: int, calibration_gain: int = 2) -> int:
    numerator = value * (calibration_gain + 1)
    return -((-numerator + 2) // 4) if numerator < 0 else (numerator + 2) // 4


def build_lut(calibration_gain: int = 2, pedestal: int = 20) -> list[int]:
    lut = [0] * 1024
    for previous in range(32):
        for current in range(32):
            delta = centroid_delta_phase8(previous, current)
            code = max(0, min(63, pedestal + scale_real_sum(delta, calibration_gain)))
            lut[(previous << 5) | current] = code
    for packed in range(256):
        lut[packed] |= phase5_state(packed) << 8
    return lut


def rms_degrees(errors: list[float]) -> float:
    return math.degrees(math.sqrt(sum(value * value for value in errors) /
                                  len(errors)))


def validate_sources(repo: Path) -> None:
    asm = (repo / "main" / "c5vrx2_wbfm_q4_phase5_2to1.bsasm").read_text()
    source = (repo / "main" / "wbfm_q4.c").read_text()
    realtime = (repo / "main" / "realtime.c").read_text()
    defaults = (repo / "sdkconfig.quality.defaults").read_text()
    # Restored two-bundle baseline: 40 MHz bus, repeated 20 MS/s values.
    # Check the checked-in assembly, not the abandoned five-bundle algorithm.
    assert asm.count("read 16") == 2
    assert asm.count("write 16") == 1
    assert "jmp address_delta" in asm
    from bs_model import simulate
    raw = [(i*37+(i>>2)*19+7)&255 for i in range(32772)]
    expected = []
    previous = 0
    lut = build_lut()
    for packed in raw[1::2]:
        current = phase5_state(packed)
        code = lut[(previous<<5)|current]&63
        expected.extend([code,code])
        previous = current
    assert simulate(asm,raw,len(expected)) == expected
    assert "lut " + " ".join(map(str, build_lut())) in asm
    assert "q4_phase5" in source
    assert "q4_phase5_state" in source
    assert "c5vrx2_wbfm_q4_phase5_program" in realtime
    assert "CONFIG_C5VRX2_MODE_LIVE=y" in defaults
    assert "CONFIG_C5VRX2_WBFM_PHASE5_QUALITY=y" in defaults


def rewrite_embedded_lut(repo: Path) -> None:
    path = repo / "main" / "c5vrx2_wbfm_q4_phase5_2to1.bsasm"
    lines = path.read_text().splitlines()
    replacement = "lut " + " ".join(map(str, build_lut()))
    lut_lines = [index for index, line in enumerate(lines)
                 if line.startswith("lut ")]
    if len(lut_lines) != 1:
        raise RuntimeError("expected exactly one embedded LUT")
    lines[lut_lines[0]] = replacement
    path.write_text("\n".join(lines) + "\n")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--rewrite-lut", action="store_true",
                        help="regenerate the embedded assembly LUT")
    args = parser.parse_args()
    repo = Path(__file__).resolve().parents[1]
    if args.rewrite_lut:
        rewrite_embedded_lut(repo)
    validate_sources(repo)
    lut = build_lut()
    assert len(lut) == 1024

    # Prove that overlapping raw-phase and pair-delta addresses do not collide:
    # they occupy disjoint bits of the same 16-bit word.
    for packed in range(256):
        assert ((lut[packed] >> 8) & 0x1F) == phase5_state(packed)
    for previous in range(32):
        for current in range(32):
            index = (previous << 5) | current
            delta = centroid_delta_phase8(previous, current)
            expected = max(0, min(63, 20 + scale_real_sum(delta, 2)))
            assert (lut[index] & 0x3F) == expected

    cartesian_errors: list[float] = []
    polar_errors: list[float] = []
    for packed in range(256):
        q = signed_bucket_center(packed & 0x0F, 4)
        i = signed_bucket_center(packed >> 4, 4)
        if i * i + q * q <= 128.0 * 128.0:
            continue
        reference = exact_phase(packed)
        cartesian_errors.append(abs(wrapped(compact_iq5_phase(packed) -
                                            reference)))
        quantized = phase5_centroids()[phase5_state(packed)]
        polar_errors.append(abs(wrapped(quantized - reference)))

    cartesian_rms = rms_degrees(cartesian_errors)
    polar_rms = rms_degrees(polar_errors)
    cartesian_max = math.degrees(max(cartesian_errors))
    polar_max = math.degrees(max(polar_errors))
    assert polar_rms < cartesian_rms * 0.4
    # Keep the worst reliable-vector error comfortably below half of the old
    # Cartesian quantizer's maximum.
    assert polar_max < cartesian_max * 0.5

    # Explicit branch-cut check around +pi/-pi remains a small positive step.
    assert centroid_delta_phase8(15, 16) == 8

    output_codes = sorted({value & 0x3F for value in lut})
    assert len(output_codes) >= 32


    print("five-bit polar WBFM quality validation PASS")
    print(f"  Q3/I2 Cartesian phase error: {cartesian_rms:.2f} deg RMS, "
          f"{cartesian_max:.2f} deg max")
    print(f"  phase5 polar phase error:     {polar_rms:.2f} deg RMS, "
          f"{polar_max:.2f} deg max")
    print(f"  centroid delta DAC levels:    {len(output_codes)}")
    print("  circular phase states:        32/32 preserved")
    print("  all 1024 dual-purpose LUT entries and modulo wrap verified")
    print("  actual assembly verified: repeated 20 MS/s values on 40 MHz bus")
    return 0


def validate_reconstruction_simulation(lut: list[int]) -> None:
    """Historical arithmetic example for the reverted core; not a source validator."""
    test_iq = [(i * 37 + (i >> 2) * 19 + 7) & 0xFF for i in range(256)]
    expected = []
    prev_p = 0
    prev_dac = 0
    for packed in test_iq:
        curr_p = (lut[packed] >> 8) & 0x1F
        dac = lut[(prev_p << 5) | curr_p] & 0x3F
        mid = (prev_dac + dac + 1) // 2
        expected.extend([prev_dac, mid])
        prev_dac = dac
        prev_p = curr_p

    O = 0
    ctra = 1
    simulated = []

    def get_bits(val: int, low: int, high: int) -> int:
        mask = (1 << (high - low + 1)) - 1
        return (val >> low) & mask

    for packed in test_iq:
        a_from_prev_b = get_bits(O, 16, 21)
        p_prev = get_bits(O, 26, 30)
        lut_addr_phase = packed
        new_O = (a_from_prev_b & 0x3F) | ((lut_addr_phase & 0xFF) << 16) | ((p_prev & 0x1F) << 26)
        ctra = 1
        O = new_O
        lut_out = lut[lut_addr_phase]

        p_curr = (lut_out >> 8) & 0x1F
        a_val = get_bits(O, 0, 5)
        new_O = (a_val & 0x3F) | ((p_curr & 0x1F) << 16) | ((p_prev & 0x1F) << 21) | ((p_curr & 0x1F) << 26)
        O = new_O
        lut_addr_dac = (p_prev << 5) | p_curr
        lut_out = lut[lut_addr_dac]

        dac_b = lut_out & 0x3F
        new_O = (a_val & 0x3F) | ((dac_b & 0x3F) << 6) | ((a_val & 0x3F) << 16) | ((p_curr & 0x1F) << 26)
        ctra = (ctra + a_val) & 0xFFFF
        O = new_O

        new_O = (a_val & 0x3F) | ((dac_b & 0x3F) << 6) | ((dac_b & 0x3F) << 16) | ((p_curr & 0x1F) << 26)
        ctra = (ctra + dac_b) & 0xFFFF
        O = new_O

        midpoint = get_bits(ctra, 1, 6)
        byte0 = a_val & 0x3F
        byte1 = midpoint & 0x3F
        simulated.extend([byte0, byte1])
        new_O = byte0 | (byte1 << 8) | ((dac_b & 0x3F) << 16) | ((p_curr & 0x1F) << 26)
        O = new_O

    assert simulated == expected, "BitScrambler simulation output differs from analytical [A, round((A+B)/2)]"


if __name__ == "__main__":
    raise SystemExit(main())
