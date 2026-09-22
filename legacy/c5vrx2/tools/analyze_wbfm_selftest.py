#!/usr/bin/env python3
"""Decode a C5VRX staged BitScrambler self-test flash record."""

from __future__ import annotations

import argparse
import struct
from pathlib import Path

MAGIC = 0x31544257
INPUT_BYTES = 1024
RESULT_BYTES = 1024
STAGE_NAMES = (
    "phase address and LUT low byte",
    "negative phase LUT high byte",
    "previous-output state carry",
    "ADDCTIAL adjacent phase delta",
    "two-delta pair accumulator",
    "counter-to-LUT identity address",
    "counter-to-production LUT output",
    "complete production program",
)


def old_record(data: bytes) -> None:
    fields = struct.unpack_from("<IHH i 5I 8I", data)
    print("legacy self-test record v1")
    print(f"run_error={fields[3]} written={fields[4]} "
          f"mismatches={fields[5]} first={fields[6]}")
    print(f"phase_error={fields[9]} phase_written={fields[10]} "
          f"phase_mismatches={fields[11]} phase_first={fields[12]}")


def new_record(data: bytes) -> None:
    magic, version, header_bytes, result, input_bytes = struct.unpack_from(
        "<IHHII", data)
    print(f"staged self-test v{version}: result={result} "
          f"input={input_bytes} header={header_bytes}")
    record_fmt = "<iIIIII"
    record_size = struct.calcsize(record_fmt)
    data_base = header_bytes + input_bytes
    cycles = struct.unpack_from("<12I", data, 16 + 8 * record_size)
    cpu_hz = cycles[8] if version >= 3 else 0
    for index, name in enumerate(STAGE_NAMES):
        values = struct.unpack_from(record_fmt, data, 16 + index * record_size)
        error, written, compared, mismatches, first, offset = values
        actual_at = data_base + index * 2 * RESULT_BYTES
        expected_at = actual_at + RESULT_BYTES
        actual = data[actual_at:actual_at + min(written, RESULT_BYTES)]
        expected = data[expected_at + offset:
                        expected_at + offset + min(compared, RESULT_BYTES)]
        local = sum(a != b for a, b in zip(actual, expected))
        timing = ""
        if cpu_hz and cycles[index]:
            seconds = cycles[index] / cpu_hz
            input_rate = input_bytes / seconds
            output_rate = written / seconds
            timing = (f" time_us={seconds * 1e6:.2f} "
                      f"input_MBps={input_rate / 1e6:.3f} "
                      f"output_MBps={output_rate / 1e6:.3f}")
        print(f"{index + 1}: {name}: error={error} written={written} "
              f"compared={compared} mismatches={mismatches} first={first} "
              f"expected_offset={offset} verify={local} "
              f"unique_actual={len(set(actual))}{timing}")


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("record", type=Path)
    args = parser.parse_args()
    data = args.record.read_bytes()
    if len(data) < 64:
        raise SystemExit("record is too short")
    magic, version = struct.unpack_from("<IH", data)
    if magic != MAGIC:
        raise SystemExit(f"bad magic 0x{magic:08x}")
    if version == 1:
        old_record(data)
    elif version in (2, 3):
        new_record(data)
    else:
        raise SystemExit(f"unsupported record version {version}")


if __name__ == "__main__":
    main()
