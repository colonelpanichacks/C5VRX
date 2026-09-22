#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-only
"""Run and record the first C5VRX A4 hardware proof over native USB serial."""

from __future__ import annotations

import argparse
import hashlib
import json
import re
import time
from datetime import datetime, timezone
from pathlib import Path


DONE_MARKERS = {
    "PING": "C5VRX_PONG",
    "STATUS": "C5VRX_STATUS",
    "WBFM HWTEST": "C5VRX_WBFM_HWTEST_DONE",
    "CAPTURE 16384": "C5VRX_CAPTURE_DONE",
    "WBFM CAPTURE 16384": "C5VRX_WBFM_CAPTURE_DONE",
    "CHAIN 32 16384": "C5VRX_CHAIN_DONE",
}


def parse_fields(line: str) -> dict[str, int | str]:
    fields: dict[str, int | str] = {}
    for key, value in re.findall(r"([A-Za-z_]+)=([^\s]+)", line):
        try:
            fields[key] = int(value, 0)
        except ValueError:
            fields[key] = value
    return fields


def summarize_capture(lines: list[str]) -> dict[str, int | str]:
    words = [line[3:].strip().lower() for line in lines if line.startswith("IQ:")]
    blob = bytes.fromhex("".join(words)) if words else b""
    return {
        "words": len(words),
        "sha256": hashlib.sha256(blob).hexdigest(),
        "unique_words": len(set(words)),
    }


def result_from_lines(command: str, lines: list[str]) -> dict[str, object]:
    marker = DONE_MARKERS[command]
    final = next((line for line in reversed(lines) if marker in line), "")
    fields = parse_fields(final)
    passed = bool(final)
    if "code" in fields:
        passed = passed and fields["code"] == 0
    result: dict[str, object] = {
        "command": command,
        "passed": passed,
        "final": final,
        "fields": fields,
        "lines": lines,
    }
    if command.startswith("CAPTURE "):
        result["capture"] = summarize_capture(lines)
        result["passed"] = passed and result["capture"]["words"] == 16384
    return result


def read_command(serial_port, command: str, timeout_s: float) -> dict[str, object]:
    serial_port.reset_input_buffer()
    serial_port.write((command + "\n").encode("ascii"))
    serial_port.flush()
    marker = DONE_MARKERS[command]
    deadline = time.monotonic() + timeout_s
    lines: list[str] = []
    while time.monotonic() < deadline:
        raw = serial_port.readline()
        if not raw:
            continue
        line = raw.decode("utf-8", errors="replace").strip()
        if line:
            print(line)
            lines.append(line)
        if marker in line:
            break
    return result_from_lines(command, lines)


def require_prompt(message: str, assume_yes: bool) -> None:
    if assume_yes:
        return
    answer = input(f"{message} Type YES to continue: ").strip()
    if answer != "YES":
        raise SystemExit("bench test cancelled")


def run_bench(port: str, output: Path, baud: int, timeout_s: float,
              assume_yes: bool) -> int:
    try:
        import serial
    except ImportError as exc:
        raise SystemExit("install pyserial first: python -m pip install pyserial") from exc

    report: dict[str, object] = {
        "schema": 1,
        "created_utc": datetime.now(timezone.utc).isoformat(),
        "port": port,
        "baud": baud,
        "target": "A4/5805 MHz",
        "results": [],
    }
    results: list[dict[str, object]] = report["results"]  # type: ignore[assignment]

    with serial.Serial(port, baudrate=baud, timeout=0.25) as ser:
        time.sleep(1.0)
        for command in ("PING", "STATUS", "WBFM HWTEST"):
            results.append(read_command(ser, command, timeout_s))

        require_prompt("Turn the 5805 MHz VTX OFF and keep the antenna/load safe.", assume_yes)
        off = read_command(ser, "CAPTURE 16384", timeout_s)
        off["label"] = "vtx_off"
        results.append(off)

        require_prompt("Turn the A4/5805 MHz VTX ON with a static test image.", assume_yes)
        on = read_command(ser, "CAPTURE 16384", timeout_s)
        on["label"] = "vtx_on"
        results.append(on)
        results.append(read_command(ser, "WBFM CAPTURE 16384", timeout_s))
        results.append(read_command(ser, "CHAIN 32 16384", max(timeout_s, 30.0)))

    off_hash = off.get("capture", {}).get("sha256")  # type: ignore[union-attr]
    on_hash = on.get("capture", {}).get("sha256")  # type: ignore[union-attr]
    report["off_on_capture_changed"] = bool(off_hash and on_hash and off_hash != on_hash)
    report["all_commands_passed"] = all(bool(item["passed"]) for item in results)
    report["bench_passed"] = bool(
        report["all_commands_passed"] and report["off_on_capture_changed"]
    )
    output.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    print(f"report: {output}")
    print("BENCH PASS" if report["bench_passed"] else "BENCH INCOMPLETE/FAIL")
    return 0 if report["bench_passed"] else 1


def self_test() -> None:
    words = [f"IQ:{value:08x}" for value in range(16384)]
    capture = result_from_lines("CAPTURE 16384", words + ["C5VRX_CAPTURE_DONE code=0"])
    assert capture["passed"] is True
    assert capture["capture"]["words"] == 16384  # type: ignore[index]
    assert capture["capture"]["unique_words"] == 16384  # type: ignore[index]
    chain = result_from_lines(
        "CHAIN 32 16384",
        ["C5VRX_CHAIN_DONE code=0 blocks=32 total=524288 repeated_hashes=0 boundary_jump_power=31"],
    )
    assert chain["passed"] is True
    assert chain["fields"]["blocks"] == 32  # type: ignore[index]
    failed = result_from_lines("WBFM HWTEST", ["C5VRX_WBFM_HWTEST_DONE code=259"])
    assert failed["passed"] is False
    print("c5vrx_bench self-test PASS")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", help="serial port, for example COM5 or /dev/ttyACM0")
    parser.add_argument("--output", type=Path, default=Path("c5vrx-bench-report.json"))
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument(
        "--timeout",
        type=float,
        default=60.0,
        help="per-command timeout; raw USB captures can take tens of seconds",
    )
    parser.add_argument("--yes", action="store_true", help="skip VTX safety prompts")
    parser.add_argument("--self-test", action="store_true")
    args = parser.parse_args()
    if args.self_test:
        self_test()
        return 0
    if not args.port:
        parser.error("--port is required unless --self-test is used")
    return run_bench(args.port, args.output, args.baud, args.timeout, args.yes)


if __name__ == "__main__":
    raise SystemExit(main())
