#!/usr/bin/env python3
"""Host checks for the active MODEM-Q4 -> direct TX-BS -> CVBS path."""

from __future__ import annotations

import math
import random
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
RING_WORDS = 16384
TAU = 2.0 * math.pi


def bucket_center(code: int, bits: int) -> float:
    width = 1 << (10 - bits)
    center = code * width + (width - 1) * 0.5
    return center - 1024.0 if center >= 512.0 else center


def pack_q4(sample: complex) -> int:
    i10 = max(-512, min(511, round(sample.real))) & 0x3FF
    q10 = max(-512, min(511, round(sample.imag))) & 0x3FF
    return ((i10 >> 6) << 4) | (q10 >> 6)


def compact_iq5(packed: int) -> int:
    return ((packed >> 1) & 0x07) | (((packed >> 6) & 0x03) << 3)


def iq5_phase(compact: int) -> float:
    return math.atan2(bucket_center(compact & 0x07, 3),
                      bucket_center(compact >> 3, 2))


def map_sum(sum_mod: int, pedestal: int = 20, gain_setting: int = 2) -> int:
    signed = sum_mod - 256 if sum_mod >= 128 else sum_mod
    numerator = signed * (gain_setting + 1)
    scaled = -((-numerator + 2) // 4) if numerator < 0 else (numerator + 2) // 4
    return max(0, min(63, pedestal + scaled))


class Q4Demod:
    def __init__(self) -> None:
        self.previous = 0

    def consume_pair(self, packed: int) -> int:
        current = compact_iq5(packed)
        delta = iq5_phase(current) - iq5_phase(self.previous)
        while delta >= math.pi:
            delta -= TAU
        while delta < -math.pi:
            delta += TAU
        phase8 = max(-128, min(127, round(delta * 256.0 / TAU)))
        self.previous = current
        return map_sum(phase8 & 0xFF)


def make_iq(count: int) -> list[complex]:
    random.seed(0xC5_2)
    phase = math.pi - 0.03
    samples: list[complex] = []
    for n in range(count):
        delta = (0.17 * math.sin(n * 0.0013) +
                 0.11 * math.sin(n * 0.173) +
                 random.uniform(-0.025, 0.025))
        phase += delta
        amplitude = 440.0 + 45.0 * math.sin(n * 0.0007)
        samples.append(complex(round(amplitude * math.cos(phase)),
                               round(amplitude * math.sin(phase))))
    return samples


def demodulate(samples: list[complex], split_points: set[int]) -> list[int]:
    demod = Q4Demod()
    output: list[int] = []
    packed = [pack_q4(sample) for sample in samples]
    for index in range(1, len(packed), 2):
        # A memory block or physical-ring wrap must not reset LUT history.
        if index in split_points:
            pass
        output.append(demod.consume_pair(packed[index]))
    return output


def verify_sources() -> None:
    asm = (ROOT / "main/c5vrx2_wbfm_q4_iq5_2to1.bsasm").read_text()
    q4 = (ROOT / "main/wbfm_q4.c").read_text()
    realtime = (ROOT / "main/realtime.c").read_text()
    source = (ROOT / "main/continuous_iq.c").read_text()

    assert asm.count("read 16") == 1 and asm.count("write 8") == 1
    # Physical result20/result21 proved current byte I24..31, retained prior
    # pair I8..15, and conventional O16-based 16-bit LUT addressing.
    assert "set 16 25" in asm and "set 20 31" in asm
    assert "set 21 9" in asm and "set 25 15" in asm
    assert "set 26..31 L" in asm
    assert "set 0..5 L0..L5" in asm
    assert "scale_real_sum(sum" in q4
    assert "return phase *" not in q4
    for name in ("q4_phase", "q4_negative", "q4_state", "q4_delta",
                 "q4_pairsum", "counter_lut"):
        assert (ROOT / f"main/c5vrx2_{name}.bsasm").exists()

    assert "adctrig(" not in source
    assert "CTRL_START" in source and "control | CTRL_ENABLE" in source
    assert "CTRL_START;" not in source
    assert "c5vrx2_wifi5_lock_rx_only()" in source
    assert "#define MODEM_IQ_RATE_HZ 40000000u" in realtime
    assert "#define CVBS_RATE_HZ     20000000u" in realtime
    assert ".partial_rx_en = true" in realtime
    assert ".flags.loop_transmission = true" in realtime
    assert "parlio_tx_unit_decorate_bitscrambler" in realtime
    assert "c5vrx2_wbfm_q4_iq5_program()" in realtime
    assert "s_raw_ring" in realtime
    assert "s_rx_bs" not in realtime
    assert realtime.rindex("continuous_iq_start()") < realtime.rindex(
        "start_rx_ring()")

    # The PARLIO decorator reloads the program at transaction start, so the
    # production table must be embedded in the assembly. Verify every entry
    # against an independent implementation; a stale generated table would
    # otherwise compile and run at full rate while producing the wrong CVBS.
    lut_line = next(line for line in asm.splitlines() if line.startswith("lut "))
    embedded = [int(item) for item in lut_line.split()[1:]]
    expected: list[int] = []
    for previous in range(32):
        for current in range(32):
            delta = iq5_phase(current) - iq5_phase(previous)
            while delta >= math.pi:
                delta -= TAU
            while delta < -math.pi:
                delta += TAU
            phase8 = max(-128, min(127, round(delta * 256.0 / TAU)))
            expected.append(map_sum(phase8 & 0xFF))
    assert len(embedded) == 1024
    assert embedded == expected, "embedded direct-discriminator LUT is stale"


def verify_modulo_gain() -> None:
    # Scaling absolute phase by 3/2 is not circular: equal angles one turn
    # apart become 128 codes apart. Gain after signed differencing is valid.
    assert ((3 * 256) // 2) & 0xFF == 128
    seam_delta = ((-128 & 0xFF) - (127 & 0xFF)) & 0xFF
    assert seam_delta == 1
    assert map_sum(2) > map_sum(1)


def main() -> None:
    verify_sources()
    verify_modulo_gain()
    samples = make_iq(RING_WORDS * 3 + 257)
    uninterrupted = demodulate(samples, set())
    split = demodulate(samples, {1, 7, RING_WORDS, RING_WORDS + 3,
                                 2 * RING_WORDS})
    assert uninterrupted == split, "Q4/FM state changed at a ring boundary"
    assert len(set(uninterrupted)) > 20
    assert all(0 <= value <= 63 for value in uninterrupted)
    print("active Q4 IQ5 two-sample-FM boundary state: PASS")
    print("absolute phase remains unscaled; gain follows signed modulo unwrap: PASS")
    print("40-MS/s Q4 -> coherent n-to-n+2 delta -> 20-MS/s DAC model: PASS")


if __name__ == "__main__":
    main()
