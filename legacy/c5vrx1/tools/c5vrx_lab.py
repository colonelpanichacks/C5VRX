#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-only
"""Session-oriented USB-C hardware lab runner for C5VRX.

Stdout contains one final JSON object. Live device output and operator prompts
go to stderr so automation can parse the result without losing live feedback.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
import re
import struct
import subprocess
import sys
import tempfile
import threading
import time
import zipfile
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Callable

from c5vrx_usb_protocol import (
    FRAME_DESCRIPTOR,
    PACKET_GRAY8_FRAME,
    PACKET_YUV411_FRAME,
    PIXEL_FORMAT_GRAY8,
    PIXEL_FORMAT_YUV411,
    Packet,
    StreamDecoder,
    yuv411_to_rgb,
)


SCHEMA_VERSION = 1
DEFAULT_BAUD = 115200
DEFAULT_CAPTURE_WORDS = 16384
SESSION_ROOT = Path("c5vrx-sessions")
FIELD_RE = re.compile(r"([A-Za-z][A-Za-z0-9_]*)=([^\s]+)")
IQ_RE = re.compile(r"^IQ:([0-9a-fA-F]{8})$")


def utc_now() -> str:
    return datetime.now(timezone.utc).isoformat()


def json_value(value: str) -> int | float | str:
    try:
        return int(value, 0)
    except ValueError:
        try:
            return float(value)
        except ValueError:
            return value


def parse_measurement(line: str) -> dict[str, Any] | None:
    marker = line.find("C5VRX_")
    if marker < 0:
        return None
    record_text = line[marker:].strip()
    name = record_text.split(maxsplit=1)[0]
    return {
        "timestamp_utc": utc_now(),
        "name": name,
        "fields": {key: json_value(value) for key, value in FIELD_RE.findall(record_text)},
        "raw": line,
    }


def local_git_info(root: Path | None = None) -> dict[str, Any]:
    repo = root or Path(__file__).resolve().parent.parent

    def git(*args: str) -> str | None:
        try:
            return subprocess.check_output(
                ["git", "-C", str(repo), *args],
                stderr=subprocess.DEVNULL,
                text=True,
                timeout=3,
            ).strip()
        except (OSError, subprocess.SubprocessError):
            return None

    commit = git("rev-parse", "HEAD")
    return {
        "repository": git("config", "--get", "remote.origin.url"),
        "commit": commit,
        "describe": git("describe", "--always", "--dirty", "--tags"),
        "branch": git("branch", "--show-current"),
        "dirty": bool(git("status", "--porcelain")),
    }


def default_session_parent() -> Path:
    configured = os.environ.get("C5VRX_SESSION_ROOT")
    if configured:
        return Path(configured).expanduser()
    return SESSION_ROOT


class SessionRecorder:
    """Thread-safe durable recorder shared by the CLI and Receiver Console."""

    def __init__(
        self,
        kind: str,
        parent: Path | None = None,
        board_profile: dict[str, Any] | None = None,
        test_config: dict[str, Any] | None = None,
        maximum_preview_frames: int = 250,
    ) -> None:
        stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
        base = (parent or default_session_parent()).expanduser().resolve()
        base.mkdir(parents=True, exist_ok=True)
        candidate = base / f"{stamp}-{kind}"
        suffix = 1
        while candidate.exists():
            candidate = base / f"{stamp}-{kind}-{suffix}"
            suffix += 1
        candidate.mkdir()
        self.path = candidate
        self.iq_path = candidate / "iq"
        self.preview_path = candidate / "preview"
        self.iq_path.mkdir()
        self.preview_path.mkdir()
        self._lock = threading.RLock()
        self._raw = (candidate / "raw-serial.bin").open("ab")
        self._text = (candidate / "raw-serial.log").open(
            "a", encoding="utf-8", newline="\n")
        self.measurements: list[dict[str, Any]] = []
        self.commands: list[dict[str, Any]] = []
        self.errors: list[dict[str, Any]] = []
        self.iq_captures: list[dict[str, Any]] = []
        self.preview_frames: list[dict[str, Any]] = []
        self._iq_words: list[int] | None = None
        self._iq_label = "capture"
        self._iq_count = 0
        self._preview_limit = maximum_preview_frames
        self._preview_limit_reported = False
        self.started_utc = utc_now()
        self.host_git = local_git_info()
        self.board_profile = board_profile or {"profile_id": "unknown"}
        self.test_config = test_config or {}
        self.firmware: dict[str, Any] = {}
        self.outcome: dict[str, Any] = {"status": "RUNNING", "passed": False}
        self._write_json("board-profile.json", self.board_profile)
        self._write_json("test-configuration.json", self.test_config)
        self._write_json("firmware-git-version.json", {
            "host_git": self.host_git, "device": self.firmware,
        })
        self.snapshot()

    def _write_json(self, name: str, value: Any) -> None:
        path = self.path / name
        temporary = path.with_suffix(path.suffix + ".tmp")
        temporary.write_text(json.dumps(value, indent=2, sort_keys=True) + "\n", encoding="utf-8")
        temporary.replace(path)

    def update_test_config(self, **values: Any) -> None:
        with self._lock:
            self.test_config.update(values)
            self._write_json("test-configuration.json", self.test_config)

    def record_raw(self, data: bytes) -> None:
        with self._lock:
            self._raw.write(data)
            self._raw.flush()

    def record_command(self, command: str) -> None:
        with self._lock:
            self.commands.append({"timestamp_utc": utc_now(), "command": command})
            self._text.write(f"> {command}\n")
            self._text.flush()

    def next_iq_label(self, label: str) -> None:
        with self._lock:
            self._iq_label = re.sub(r"[^A-Za-z0-9_.-]+", "-", label).strip("-") or "capture"

    def record_line(self, line: str) -> dict[str, Any] | None:
        with self._lock:
            self._text.write(line + "\n")
            self._text.flush()
            measurement = parse_measurement(line)
            if measurement:
                self.measurements.append(measurement)
                if measurement["name"] == "C5VRX_STATUS":
                    fields = measurement["fields"]
                    self.firmware.update({
                        key: fields[key] for key in
                        ("firmware", "version", "idf", "protocol", "profile")
                        if key in fields
                    })
                    profile_id = fields.get("profile")
                    if isinstance(profile_id, str) and profile_id:
                        self._set_detected_board_profile(profile_id)
                fields = measurement["fields"]
                classification = str(fields.get("classification", ""))
                if measurement["name"] == "C5VRX_ERR":
                    self.record_error("DEVICE_ERROR", line)
                elif fields.get("code") not in (None, 0):
                    self.record_error("DEVICE_FAILURE", line)
                elif classification.startswith(("FAILED", "REJECTED")):
                    self.record_error("DEVICE_FAILURE", line)
            if line.startswith("C5VRX_IQ_BEGIN"):
                self._iq_words = []
            elif self._iq_words is not None:
                match = IQ_RE.match(line)
                if match:
                    self._iq_words.append(int(match.group(1), 16))
                elif line == "C5VRX_IQ_END":
                    self._save_iq_capture(self._iq_label, self._iq_words)
                    self._iq_words = None
            return measurement

    def _set_detected_board_profile(self, profile_id: str) -> None:
        if self.board_profile.get("profile_id") == profile_id:
            return
        roots = [
            Path(__file__).resolve().parent.parent / "firmware_profiles",
            Path(__file__).resolve().parent,
            Path.cwd(),
        ]
        selected: dict[str, Any] | None = None
        for root in roots:
            candidates = list(root.glob("*.json")) if root.name == "firmware_profiles" else [root / "profile.json"]
            for path in candidates:
                try:
                    candidate = json.loads(path.read_text(encoding="utf-8"))
                except (OSError, ValueError):
                    continue
                if candidate.get("profile_id") == profile_id:
                    selected = candidate
                    break
            if selected:
                break
        self.board_profile = selected or {"profile_id": profile_id}
        self._write_json("board-profile.json", self.board_profile)

    def _save_iq_capture(self, label: str, words: list[int]) -> None:
        self._iq_count += 1
        stem = f"{self._iq_count:03d}-{label}"
        packed_path = self.iq_path / f"{stem}.iq32le"
        decoded_path = self.iq_path / f"{stem}.iq16le"
        with packed_path.open("wb") as packed, decoded_path.open("wb") as decoded:
            for word in words:
                packed.write(struct.pack("<I", word))
                i, q = decode_iq_word(word)
                decoded.write(struct.pack("<hh", i, q))
        item = {
            "label": label,
            "words": len(words),
            "packed_file": str(packed_path.relative_to(self.path)),
            "decoded_iq_file": str(decoded_path.relative_to(self.path)),
            "sha256": hashlib.sha256(b"".join(struct.pack("<I", word) for word in words)).hexdigest(),
            "metrics": iq_metrics(words),
        }
        self.iq_captures.append(item)
        self._write_json("iq/index.json", self.iq_captures)

    def record_packet(self, packet: Packet) -> None:
        if packet.packet_type not in {PACKET_GRAY8_FRAME, PACKET_YUV411_FRAME}:
            return
        with self._lock:
            if len(self.preview_frames) >= self._preview_limit:
                if not self._preview_limit_reported:
                    self.record_error(
                        "PREVIEW_ARCHIVE_LIMIT",
                        f"preview archive limited to {self._preview_limit} frames",
                    )
                    self._preview_limit_reported = True
                return
            if len(packet.payload) < FRAME_DESCRIPTOR.size:
                self.record_error("PREVIEW_SHORT_DESCRIPTOR", "GRAY8 packet descriptor is truncated")
                return
            width, height, stride, pixel_format, flags = FRAME_DESCRIPTOR.unpack_from(packet.payload)
            pixels = packet.payload[FRAME_DESCRIPTOR.size:]
            if (
                not width or not height or width > 640 or height > 480
                or pixel_format not in {PIXEL_FORMAT_GRAY8, PIXEL_FORMAT_YUV411}
                or (pixel_format == PIXEL_FORMAT_GRAY8 and stride < width)
                or (pixel_format == PIXEL_FORMAT_YUV411 and stride < width * 3 // 2)
                or len(pixels) != stride * height
            ):
                self.record_error("PREVIEW_INVALID_FRAME", f"sequence={packet.sequence}")
                return
            color = pixel_format == PIXEL_FORMAT_YUV411
            if color:
                try:
                    pixels = yuv411_to_rgb(pixels, width, height, stride)
                except ValueError as exc:
                    self.record_error("PREVIEW_INVALID_YUV411", str(exc))
                    return
            elif stride != width:
                pixels = b"".join(
                    pixels[row * stride:row * stride + width] for row in range(height)
                )
            suffix = "ppm" if color else "pgm"
            filename = f"frame-{len(self.preview_frames) + 1:05d}-seq-{packet.sequence}.{suffix}"
            frame_path = self.preview_path / filename
            magic = "P6" if color else "P5"
            frame_path.write_bytes(
                f"{magic}\n{width} {height}\n255\n".encode("ascii") + pixels)
            self.preview_frames.append({
                "file": str(frame_path.relative_to(self.path)),
                "sequence": packet.sequence,
                "timestamp_us": packet.timestamp_us,
                "width": width,
                "height": height,
                "sync_locked": bool(flags & 1),
                "sha256": hashlib.sha256(pixels).hexdigest(),
            })
            self._write_json("preview/index.json", self.preview_frames)

    def record_error(self, kind: str, detail: str, **fields: Any) -> None:
        with self._lock:
            item = {"timestamp_utc": utc_now(), "kind": kind, "detail": detail, **fields}
            self.errors.append(item)
            self._write_json("errors-failures.json", self.errors)

    def snapshot(self) -> None:
        with self._lock:
            self._write_json("measurements.json", {
                "schema": SCHEMA_VERSION,
                "records": self.measurements,
                "commands": self.commands,
                "iq_captures": self.iq_captures,
                "preview_frames": self.preview_frames,
            })
            self._write_json("errors-failures.json", self.errors)
            self._write_json("results.json", self.outcome)
            self._write_json("firmware-git-version.json", {
                "host_git": self.host_git, "device": self.firmware,
            })
            self._write_json("session.json", {
                "schema": SCHEMA_VERSION,
                "started_utc": self.started_utc,
                "updated_utc": utc_now(),
                "session_folder": self.path.name,
                "outcome": self.outcome,
                "artifacts": {
                    "raw_serial_binary": "raw-serial.bin",
                    "raw_serial_log": "raw-serial.log",
                    "structured_results": "results.json",
                    "measurements": "measurements.json",
                    "firmware_git_version": "firmware-git-version.json",
                    "board_profile": "board-profile.json",
                    "test_configuration": "test-configuration.json",
                    "iq_index": "iq/index.json",
                    "preview_index": "preview/index.json",
                    "errors_failures": "errors-failures.json",
                },
            })
            # Always materialize indexes, even when the hardware produced none.
            self._write_json("iq/index.json", self.iq_captures)
            self._write_json("preview/index.json", self.preview_frames)

    def finalize(self, outcome: dict[str, Any]) -> None:
        with self._lock:
            self.outcome = outcome
            self.snapshot()

    def create_bundle(self, destination: Path | None = None) -> Path:
        with self._lock:
            self.snapshot()
            target = destination or self.path.with_name(self.path.name + "-codex-bundle.zip")
            target = target.expanduser().resolve()
            target.parent.mkdir(parents=True, exist_ok=True)
            with zipfile.ZipFile(target, "w", compression=zipfile.ZIP_DEFLATED) as archive:
                for path in sorted(self.path.rglob("*")):
                    if path.is_file() and path.resolve() != target:
                        archive.write(path, path.relative_to(self.path))
            return target

    def close(self) -> None:
        with self._lock:
            self.snapshot()
            self._raw.close()
            self._text.close()


def decode_iq_word(word: int) -> tuple[int, int]:
    def sign10(value: int) -> int:
        value &= 0x3FF
        return value - 0x400 if value & 0x200 else value

    return sign10(word >> 10), sign10(word)


def iq_metrics(words: list[int], sample_rate_hz: float | None = None) -> dict[str, Any]:
    if len(words) < 3:
        return {"samples": len(words), "available": False}
    samples = [decode_iq_word(word) for word in words]
    count = len(samples)
    mean_i = sum(i for i, _ in samples) / count
    mean_q = sum(q for _, q in samples) / count
    ac_power = sum((i - mean_i) ** 2 + (q - mean_q) ** 2 for i, q in samples) / count
    phases: list[float] = []
    unit_real = 0.0
    unit_imag = 0.0
    for (previous_i, previous_q), (current_i, current_q) in zip(samples, samples[1:]):
        real = current_i * previous_i + current_q * previous_q
        imag = current_q * previous_i - current_i * previous_q
        phase = math.atan2(imag, real)
        phases.append(phase)
        magnitude = math.hypot(real, imag)
        if magnitude:
            unit_real += real / magnitude
            unit_imag += imag / magnitude
    phase_mean = math.atan2(unit_imag, unit_real)
    phase_coherence = math.hypot(unit_real, unit_imag) / len(phases)
    discriminator_std = math.sqrt(
        sum(wrapped_phase(value - phase_mean) ** 2 for value in phases) / len(phases)
    )
    result: dict[str, Any] = {
        "available": True,
        "samples": count,
        "unique_words": len(set(words)),
        "mean_i": mean_i,
        "mean_q": mean_q,
        "rms_ac": math.sqrt(ac_power),
        "mean_magnitude": sum(math.hypot(i, q) for i, q in samples) / count,
        "mean_phase_increment": phase_mean,
        "phase_coherence": phase_coherence,
        "wbfm_discriminator_std_rad": discriminator_std,
    }
    if sample_rate_hz and sample_rate_hz > 0:
        period_us, score = line_period_candidate(phases, sample_rate_hz)
        result.update({
            "sample_rate_hz": sample_rate_hz,
            "frequency_offset_hz": phase_mean * sample_rate_hz / (2 * math.pi),
            "video_line_period_candidate_us": period_us,
            "video_line_correlation": score,
        })
    return result


def wrapped_phase(value: float) -> float:
    return (value + math.pi) % (2 * math.pi) - math.pi


def line_period_candidate(demod: list[float], sample_rate_hz: float) -> tuple[float, float]:
    if len(demod) < 32:
        return 0.0, 0.0
    mean = sum(demod) / len(demod)
    centered = [value - mean for value in demod]
    variance = sum(value * value for value in centered) / len(centered)
    if variance < 1e-12:
        return 0.0, 0.0
    low = max(1, int(sample_rate_hz * 55e-6))
    high = min(len(centered) - 2, int(sample_rate_hz * 72e-6))
    if high <= low:
        return 0.0, 0.0
    # Bound the cost while covering the PAL/NTSC range at high measured rates.
    stride = max(1, (high - low) // 500)
    best_lag, best_score = low, -1.0
    for lag in range(low, high + 1, stride):
        pairs = len(centered) - lag
        covariance = sum(centered[n] * centered[n + lag] for n in range(pairs)) / pairs
        score = covariance / variance
        if score > best_score:
            best_lag, best_score = lag, score
    return best_lag * 1_000_000.0 / sample_rate_hz, best_score


def ratio_changed(a: float, b: float, low: float, high: float) -> bool:
    if abs(a) < 1e-12:
        return abs(b) > 1e-12
    ratio = abs(b / a)
    return ratio < low or ratio > high


def compare_vtx_metrics(off: dict[str, Any], on: dict[str, Any]) -> dict[str, Any]:
    iq_changed = (
        ratio_changed(float(off["rms_ac"]), float(on["rms_ac"]), 0.80, 1.25)
        or abs(float(on["phase_coherence"]) - float(off["phase_coherence"])) >= 0.15
        or math.hypot(
            float(on["mean_i"]) - float(off["mean_i"]),
            float(on["mean_q"]) - float(off["mean_q"]),
        ) >= 32.0
    )
    wbfm_changed = (
        ratio_changed(
            float(off["wbfm_discriminator_std_rad"]),
            float(on["wbfm_discriminator_std_rad"]),
            0.80,
            1.25,
        )
        or abs(float(on["mean_phase_increment"]) - float(off["mean_phase_increment"])) >= 0.05
    )
    off_score = float(off.get("video_line_correlation", 0.0))
    on_score = float(on.get("video_line_correlation", 0.0))
    on_period = float(on.get("video_line_period_candidate_us", 0.0))
    sync_candidate_changed = (
        55.0 <= on_period <= 72.0 and on_score >= 0.35 and on_score - off_score >= 0.10
    )
    return {
        "rf_iq": {
            "changed": iq_changed,
            "reason": "bounded IQ feature delta exceeds host heuristic" if iq_changed
            else "no bounded IQ feature delta exceeded host heuristic",
        },
        "wbfm": {
            "changed": wbfm_changed,
            "reason": "phase-discriminator feature delta exceeds host heuristic" if wbfm_changed
            else "no phase-discriminator feature delta exceeded host heuristic",
        },
        "video_sync_candidate": {
            "changed": sync_candidate_changed,
            "reason": "on-capture has a stronger 55-72 us correlation candidate" if sync_candidate_changed
            else "no stronger bounded PAL/NTSC line-period candidate",
            "machine_video_proof": False,
        },
        "thresholds": {
            "iq_rms_ratio": [0.80, 1.25],
            "iq_coherence_absolute_delta": 0.15,
            "iq_dc_centroid_delta": 32.0,
            "wbfm_std_ratio": [0.80, 1.25],
            "wbfm_mean_phase_absolute_delta_rad": 0.05,
            "line_period_us": [55.0, 72.0],
            "line_correlation_min": 0.35,
            "line_correlation_improvement": 0.10,
        },
    }


@dataclass(frozen=True)
class CommandResult:
    command: str
    marker: str
    passed: bool
    reason: str
    records: list[dict[str, Any]]
    elapsed_seconds: float

    def as_json(self) -> dict[str, Any]:
        return {
            "command": self.command,
            "marker": self.marker,
            "passed": self.passed,
            "reason": self.reason,
            "elapsed_seconds": round(self.elapsed_seconds, 3),
            "records": self.records,
        }


class LabConnection:
    def __init__(self, serial_port: Any, recorder: SessionRecorder, live: bool = True) -> None:
        self.serial = serial_port
        self.recorder = recorder
        self.decoder = StreamDecoder()
        self.live = live
        self.records: list[dict[str, Any]] = []

    def pump(self, duration: float = 0.2) -> None:
        deadline = time.monotonic() + duration
        while time.monotonic() < deadline:
            raw = self.serial.read(4096)
            if not raw:
                continue
            self.recorder.record_raw(raw)
            for kind, value in self.decoder.feed(raw):
                if kind == "line":
                    line = str(value)
                    if self.live:
                        print(line, file=sys.stderr, flush=True)
                    record = self.recorder.record_line(line)
                    if record:
                        self.records.append(record)
                elif kind == "packet":
                    assert isinstance(value, Packet)
                    self.recorder.record_packet(value)
                else:
                    self.recorder.record_error("USB_PROTOCOL", str(value))

    def run(
        self,
        command: str,
        marker: str,
        timeout: float,
        accept: Callable[[dict[str, Any]], bool] | None = None,
    ) -> CommandResult:
        start_index = len(self.records)
        started = time.monotonic()
        self.recorder.record_command(command)
        self.serial.write((command + "\n").encode("ascii"))
        self.serial.flush()
        deadline = started + timeout
        matched: dict[str, Any] | None = None
        while time.monotonic() < deadline:
            self.pump(min(0.25, max(0.01, deadline - time.monotonic())))
            for record in self.records[start_index:]:
                if marker in record["name"] or marker in record["raw"]:
                    matched = record
                    break
            if matched or any(
                record["name"] == "C5VRX_ERR" for record in self.records[start_index:]
            ):
                break
        records = self.records[start_index:]
        errors = [record for record in records if record["name"] == "C5VRX_ERR"]
        if errors:
            passed, reason = False, errors[-1]["raw"]
        elif not matched:
            passed, reason = False, f"timeout waiting for {marker}"
        elif accept is not None:
            passed = accept(matched)
            reason = "completion record accepted" if passed else f"completion record rejected: {matched['raw']}"
        else:
            code = matched["fields"].get("code")
            passed = code in (None, 0)
            reason = "completion marker received" if passed else f"device code={code}"
        result = CommandResult(command, marker, passed, reason, records, time.monotonic() - started)
        if not passed:
            self.recorder.record_error("COMMAND_FAILED", reason, command=command, marker=marker)
        self.recorder.snapshot()
        return result


def candidate_ports() -> list[dict[str, Any]]:
    try:
        from serial.tools import list_ports
    except ImportError as exc:
        raise RuntimeError("pyserial is required: python -m pip install pyserial") from exc
    candidates = []
    for port in list_ports.comports():
        description = port.description or ""
        text = " ".join(filter(None, [description, port.manufacturer or "", port.product or ""])).lower()
        score = 0
        if port.vid == 0x303A:
            score += 100
        if "esp32" in text or "espressif" in text:
            score += 50
        if "usb jtag" in text or "usb serial" in text or "usb-c" in text:
            score += 20
        candidates.append({
            "device": port.device,
            "description": description,
            "vid": port.vid,
            "pid": port.pid,
            "serial_number": port.serial_number,
            "score": score,
        })
    return sorted(candidates, key=lambda item: (-item["score"], item["device"]))


def choose_port(explicit: str | None) -> tuple[str, list[dict[str, Any]]]:
    ports = candidate_ports()
    if explicit:
        return explicit, ports
    if not ports:
        raise RuntimeError("no serial ports detected; connect the C5 with a data-capable USB-C cable")
    preferred = [item for item in ports if item["score"] >= 50]
    if len(preferred) == 1:
        return str(preferred[0]["device"]), ports
    plausible = preferred or [
        item for item in ports
        if str(item["device"]).upper().startswith("COM")
        or any(token in str(item["device"]) for token in ("ttyACM", "ttyUSB", "cu.usb"))
    ]
    if len(plausible) == 1:
        return str(plausible[0]["device"]), ports
    if not plausible:
        raise RuntimeError("no USB serial port detected; connect the C5 with a data-capable USB-C cable")

    # Probe ambiguous ports with the documented, read-only PING command.
    try:
        import serial
    except ImportError as exc:
        raise RuntimeError("pyserial is required: python -m pip install pyserial") from exc
    matches: list[str] = []
    for item in plausible:
        try:
            with serial.Serial(item["device"], DEFAULT_BAUD, timeout=0.15, write_timeout=0.5) as probe:
                probe.write(b"PING\n")
                probe.flush()
                deadline = time.monotonic() + 1.0
                wire = b""
                while time.monotonic() < deadline:
                    wire += probe.read(512)
                    if b"C5VRX_PONG" in wire:
                        matches.append(str(item["device"]))
                        break
        except Exception:
            continue
    if len(matches) == 1:
        return matches[0], ports
    raise RuntimeError(
        "could not select one C5VRX port automatically; use --port. Candidates: "
        + ", ".join(str(item["device"]) for item in ports)
    )


def open_lab(args: argparse.Namespace, kind: str) -> tuple[SessionRecorder, LabConnection, Any]:
    config = {
        "command": kind,
        "port": args.port or "AUTO",
        "baud": args.baud,
        "timeout_seconds": args.timeout,
        "detected_ports": [],
        "target": "A4 / 5805 MHz",
        "rf_safety": {
            "bounded_capture_only": True,
            "no_rf_register_overrides": True,
            "live_start_validation_preserved": True,
        },
    }
    recorder = SessionRecorder(
        kind,
        parent=args.sessions_dir,
        test_config=config,
        maximum_preview_frames=args.max_preview_frames,
    )
    recorder.update_test_config(**{
        key: getattr(args, key) for key in
        ("capture_words", "preview_mode", "include_vtx_proof", "yes")
        if hasattr(args, key)
    })
    try:
        port, ports = choose_port(args.port)
        recorder.update_test_config(port=port, detected_ports=ports)
    except Exception as exc:
        recorder.record_error("PORT_DETECTION", str(exc))
        recorder.finalize({"status": "ERROR", "passed": False, "reason": str(exc)})
        session = recorder.path
        recorder.close()
        raise RuntimeError(f"{exc}; session={session}") from exc
    try:
        import serial
    except ImportError as exc:
        recorder.record_error("DEPENDENCY", "pyserial is not installed")
        recorder.finalize({"status": "ERROR", "passed": False, "reason": "pyserial is not installed"})
        session = recorder.path
        recorder.close()
        raise RuntimeError(
            f"pyserial is required: python -m pip install pyserial; session={session}") from exc
    try:
        serial_port = serial.Serial(port, args.baud, timeout=0.15, write_timeout=1.0)
    except Exception:
        recorder.record_error("SERIAL_OPEN", f"could not open {port}")
        recorder.finalize({"status": "ERROR", "passed": False, "reason": "serial open failed"})
        recorder.close()
        raise
    connection = LabConnection(serial_port, recorder, live=not args.quiet)
    time.sleep(0.3)
    connection.pump(0.7)
    return recorder, connection, serial_port


def find_record(records: list[dict[str, Any]], name: str, **fields: Any) -> dict[str, Any] | None:
    for record in reversed(records):
        if record["name"] != name:
            continue
        if all(record["fields"].get(key) == value for key, value in fields.items()):
            return record
    return None


def auto_test(connection: LabConnection, args: argparse.Namespace) -> dict[str, Any]:
    results: list[CommandResult] = []
    timeout = args.timeout

    def command(text: str, marker: str, seconds: float | None = None) -> CommandResult:
        result = connection.run(text, marker, seconds or timeout)
        results.append(result)
        return result

    command("PING", "C5VRX_PONG", 5.0)
    command("BW 40", "C5VRX_OK", 5.0)
    command("SET A 4", "C5VRX_OK", 10.0)
    status = command("STATUS", "C5VRX_STATUS", 10.0)
    command("WBFM HWTEST", "C5VRX_WBFM_HWTEST_DONE", 20.0)
    cadence = command("PRODUCER CADENCE PROBE ALL", "C5VRX_PRODUCER_CADENCE_DONE", 20.0)
    mode0 = find_record(cadence.records, "C5VRX_PRODUCER_CADENCE", mode=0)
    measured_rate = int(mode0["fields"].get("complex_samples_per_sec", 0)) if mode0 else 0
    if measured_rate:
        command(f"FINE TUNE VERIFY 5805 5807 {measured_rate}", "C5VRX_FINE_TUNE_VERIFY", 20.0)
    else:
        connection.recorder.record_error(
            "MEASURED_RATE_MISSING", "mode-0 cadence did not provide a sample rate; fine-tune verification skipped")
    command("WRAP FLAG PROBE 0", "C5VRX_WRAP_FLAG_PROBE_DONE", 15.0)
    command("PHASE CONTINUITY PROBE 0", "C5VRX_PHASE_CONTINUITY_DONE", 15.0)
    command("PRODUCER SOAK 0 30000", "C5VRX_PRODUCER_SOAK_DONE", max(45.0, timeout))
    for factor in (2, 4, 8):
        command(f"BENCH SPARSE {factor}", "C5VRX_BENCH_DONE", 15.0)
    command("BENCH BITSCRAMBLER", "C5VRX_BENCH_DONE", 15.0)
    command("BENCH PARLIO", "C5VRX_BENCH_DONE", 15.0)
    command("BENCH PIPELINE", "C5VRX_BENCH_DONE", 20.0)
    command("BENCH USB PREVIEW", "C5VRX_BENCH_DONE", 15.0)
    for block_words in (1024, 2048, 4096):
        command(
            f"BENCH RING PIPELINE 0 1000 {block_words}",
            "C5VRX_BENCH_RING_PIPELINE",
            20.0,
        )
    connection.recorder.next_iq_label("auto-test")
    command(f"CAPTURE {args.capture_words}", "C5VRX_CAPTURE_DONE", timeout)
    command(f"WBFM CAPTURE {args.capture_words}", "C5VRX_WBFM_CAPTURE_DONE", timeout)
    command(f"CHAIN 32 {args.capture_words}", "C5VRX_CHAIN_DONE", max(30.0, timeout))
    command("USB PREVIEW STOP", "C5VRX_USB_PREVIEW", 10.0)
    command("CAPABILITIES", "C5VRX_CAPABILITIES", 10.0)
    command("STATUS", "C5VRX_STATUS", 10.0)

    failures = [result.reason for result in results if not result.passed]
    status_record = find_record(status.records, "C5VRX_STATUS")
    if not status_record:
        failures.append("STATUS record missing")
    else:
        fields = status_record["fields"]
        if fields.get("mhz") != 5805 or fields.get("inside") != 1 or fields.get("exact") != 1:
            failures.append("device is not on exact in-window A4 / 5805 MHz")
        if not fields.get("profile"):
            failures.append("device did not report a board profile")
    if not mode0 or mode0["fields"].get("classification") != "MEASURED":
        failures.append("mode-0 producer cadence is not classified MEASURED")
    elif mode0["fields"].get("ambiguous_intervals") != 0:
        failures.append("mode-0 producer cadence contains ambiguous intervals")
    phase = find_record(connection.records, "C5VRX_PHASE_CONTINUITY", mode=0)
    if not phase or phase["fields"].get("classification") != "MEASURED_CONTINUOUS":
        failures.append("mode-0 phase continuity is not MEASURED_CONTINUOUS")
    soak = find_record(connection.records, "C5VRX_PRODUCER_SOAK_DONE", mode=0, requested_ms=30000)
    if not soak or any(soak["fields"].get(field) != 1 for field in (
        "continuous_wrap_valid", "writer_position_valid", "adjacent_memory_valid"
    )):
        failures.append("30-second continuous ring/writer/canary proof did not pass")
    rings = [
        find_record(
            connection.records,
            "C5VRX_BENCH_RING_PIPELINE",
            mode=0,
            duration_ms=1000,
            block_words=block_words,
        )
        for block_words in (1024, 2048, 4096)
    ]
    if any(not ring for ring in rings):
        failures.append("1K/2K/4K real-ring benchmark matrix is incomplete")
    elif not any(
        ring["fields"].get("classification") == "MEASURED_ON_HARDWARE" and
        ring["fields"].get("two_x_deadline_headroom_pass") == 1
        for ring in rings if ring
    ):
        failures.append("no ring block size proved >=2x service-deadline headroom")
    saved_capture = next(
        (item for item in reversed(connection.recorder.iq_captures) if item["label"] == "auto-test"),
        None,
    )
    if not saved_capture or saved_capture["words"] != args.capture_words:
        failures.append("finite auto-test IQ capture is incomplete or missing")
    return {
        "schema": SCHEMA_VERSION,
        "test": "auto-test",
        "passed": not failures,
        "status": "PASS" if not failures else "FAIL",
        "reasons": ["all existing bounded hardware-test gates passed"] if not failures else failures,
        "measured_source_rate_hz": measured_rate or None,
        "commands": [result.as_json() for result in results],
        "iq_artifact": saved_capture,
        "note": "source bandwidth remains an external RF-sweep gate and is never asserted by this tool",
    }


def operator_checkpoint(message: str, assume_yes: bool) -> None:
    if assume_yes:
        print(f"AUTOMATED FIXTURE CHECKPOINT: {message}", file=sys.stderr, flush=True)
        return
    print(message, file=sys.stderr, flush=True)
    print("Type YES after the hardware is in that state:", file=sys.stderr, flush=True)
    if sys.stdin.readline().strip() != "YES":
        raise RuntimeError("operator cancelled VTX proof test")


def actual_sync_probe(
    connection: LabConnection, state: str, mode: str, timeout: float,
) -> dict[str, Any]:
    if mode == "none":
        return {"available": False, "reason": "preview probe disabled"}
    start_preview = connection.run("USB PREVIEW START", "C5VRX_USB_PREVIEW", 10.0)
    live_command = "LIVE START" if mode == "guarded" else "LIVE EXPERIMENTAL START 0"
    live_marker = "C5VRX_LIVE_START" if mode == "guarded" else "C5VRX_LIVE_EXPERIMENTAL_START"
    start_live = connection.run(live_command, live_marker, 15.0)
    if not start_preview.passed or not start_live.passed:
        connection.run("USB PREVIEW STOP", "C5VRX_USB_PREVIEW", 10.0)
        return {
            "available": False,
            "reason": start_live.reason,
            "mode": mode,
            "production_gate_preserved": mode == "guarded",
        }
    before_frames = len(connection.recorder.preview_frames)
    probe = connection.run(
        "CVBS LOCK PROBE 1000",
        "C5VRX_CVBS_LOCK_PROBE",
        max(timeout, 10.0),
        accept=lambda _record: True,
    )
    connection.pump(0.5)
    connection.run("LIVE STOP", "C5VRX_LIVE_STOP", 10.0)
    connection.run("USB PREVIEW STOP", "C5VRX_USB_PREVIEW", 10.0)
    record = find_record(probe.records, "C5VRX_CVBS_LOCK_PROBE")
    return {
        "available": record is not None,
        "state": state,
        "mode": mode,
        "experimental_unproven": mode == "experimental",
        "record": record,
        "frames_saved": len(connection.recorder.preview_frames) - before_frames,
    }


def vtx_proof(connection: LabConnection, args: argparse.Namespace) -> dict[str, Any]:
    commands: list[CommandResult] = []

    def command(text: str, marker: str, seconds: float | None = None) -> CommandResult:
        result = connection.run(text, marker, seconds or args.timeout)
        commands.append(result)
        return result

    command("PING", "C5VRX_PONG", 5.0)
    command("BW 40", "C5VRX_OK", 5.0)
    command("SET A 4", "C5VRX_OK", 10.0)
    status = command("STATUS", "C5VRX_STATUS", 10.0)
    cadence = command("PRODUCER CADENCE PROBE 0", "C5VRX_PRODUCER_CADENCE_DONE", 20.0)
    mode0 = find_record(cadence.records, "C5VRX_PRODUCER_CADENCE", mode=0)
    rate = int(mode0["fields"].get("complex_samples_per_sec", 0)) if mode0 else 0

    state_results: dict[str, Any] = {}
    for state, prompt in (
        ("vtx-off", "Turn the VTX OFF. Keep its antenna/load attached and the RF setup safe."),
        ("vtx-on", "Turn the VTX ON at A4 / 5805 MHz with a known moving PAL test card."),
    ):
        operator_checkpoint(prompt, args.yes)
        connection.recorder.next_iq_label(state)
        capture = command(
            f"CAPTURE {args.capture_words}", "C5VRX_CAPTURE_DONE", args.timeout)
        wbfm = command(
            f"WBFM CAPTURE {args.capture_words}", "C5VRX_WBFM_CAPTURE_DONE", args.timeout)
        wbfm_metrics = find_record(wbfm.records, "C5VRX_WBFM_METRICS")
        saved = next(
            (item for item in reversed(connection.recorder.iq_captures) if item["label"] == state),
            None,
        )
        if saved and rate:
            packed = connection.recorder.path / saved["packed_file"]
            words = [item[0] for item in struct.iter_unpack("<I", packed.read_bytes())]
            saved["metrics"] = iq_metrics(words, rate)
            connection.recorder._write_json("iq/index.json", connection.recorder.iq_captures)
        state_results[state] = {
            "capture": capture.as_json(),
            "hardware_wbfm": wbfm.as_json(),
            "hardware_wbfm_metrics": wbfm_metrics,
            "iq_artifact": saved,
            "actual_video_sync": actual_sync_probe(
                connection, state, args.preview_mode, args.timeout),
        }

    failures = [result.reason for result in commands if not result.passed]
    off_capture = state_results["vtx-off"]["iq_artifact"]
    on_capture = state_results["vtx-on"]["iq_artifact"]
    comparison: dict[str, Any] = {}
    if not off_capture or not on_capture:
        failures.append("both bounded IQ artifacts were not saved")
    elif off_capture["words"] != args.capture_words or on_capture["words"] != args.capture_words:
        failures.append("one or both IQ captures are incomplete")
    else:
        comparison = compare_vtx_metrics(off_capture["metrics"], on_capture["metrics"])
        comparison["raw_capture_hash_changed"] = off_capture["sha256"] != on_capture["sha256"]
        comparison["raw_hash_is_rf_proof"] = False
        off_wbfm = state_results["vtx-off"].get("hardware_wbfm_metrics")
        on_wbfm = state_results["vtx-on"].get("hardware_wbfm_metrics")
        device_wbfm_changed = False
        if off_wbfm and on_wbfm:
            keys = ("min", "max", "mean", "mean_abs_dev_from_bias")
            device_wbfm_changed = any(
                off_wbfm["fields"].get(key) != on_wbfm["fields"].get(key) for key in keys
            )
        comparison["hardware_wbfm_summary"] = {
            "available": bool(off_wbfm and on_wbfm),
            "changed": device_wbfm_changed,
            "reason": "device BitScrambler summary fields changed" if device_wbfm_changed
            else "device BitScrambler summaries unavailable or unchanged",
        }
        off_sync = state_results["vtx-off"]["actual_video_sync"]
        on_sync = state_results["vtx-on"]["actual_video_sync"]
        actual_changed = False
        if off_sync.get("available") and on_sync.get("available"):
            off_record = off_sync.get("record") or {}
            on_record = on_sync.get("record") or {}
            actual_changed = (
                off_record.get("fields", {}).get("analog_vtx_usable_iq") == 0
                and on_record.get("fields", {}).get("analog_vtx_usable_iq") == 1
            )
        comparison["actual_video_sync"] = {
            "changed": actual_changed,
            "available": bool(off_sync.get("available") and on_sync.get("available")),
            "reason": "off rejected and on reached MEASURED_CVBS_LOCK" if actual_changed
            else "guarded on/off CVBS lock transition was not observed",
        }
        sync_changed = actual_changed or comparison["video_sync_candidate"]["changed"]
        if not comparison["raw_capture_hash_changed"]:
            failures.append("VTX off/on raw capture hashes are identical")
        if not comparison["rf_iq"]["changed"]:
            failures.append(comparison["rf_iq"]["reason"])
        if not comparison["wbfm"]["changed"]:
            failures.append(comparison["wbfm"]["reason"])
        if not sync_changed:
            failures.append("neither guarded CVBS lock nor bounded line-period candidate changed")
    if not rate:
        failures.append("measured mode-0 sample rate unavailable; video-sync comparison cannot be calibrated")
    status_record = find_record(status.records, "C5VRX_STATUS")
    if not status_record or status_record["fields"].get("mhz") != 5805:
        failures.append("device did not confirm A4 / 5805 MHz")
    return {
        "schema": SCHEMA_VERSION,
        "test": "vtx-proof",
        "passed": not failures,
        "status": "PASS" if not failures else "FAIL",
        "reasons": ["bounded VTX-off/on RF, IQ, WBFM and sync evidence changed"] if not failures else failures,
        "measured_source_rate_hz": rate or None,
        "states": state_results,
        "comparison": comparison,
        "commands": [result.as_json() for result in commands],
        "safety_note": "no bandwidth gate was asserted and no RF register value was supplied by the host",
    }


def add_connection_args(parser: argparse.ArgumentParser) -> None:
    parser.add_argument("--port", help="serial port; omitted means auto-detect C5 USB Serial/JTAG")
    parser.add_argument("--baud", type=int, default=DEFAULT_BAUD)
    parser.add_argument("--timeout", type=float, default=60.0, help="per-command timeout in seconds")
    parser.add_argument("--sessions-dir", type=Path, default=default_session_parent())
    parser.add_argument("--max-preview-frames", type=int, default=250)
    parser.add_argument("--quiet", action="store_true", help="do not mirror live device lines to stderr")


def self_test() -> None:
    def word(i: int, q: int) -> int:
        return ((i & 0x3FF) << 10) | (q & 0x3FF)

    sample_rate = 20_000_000.0
    count = 4096
    off = [word((n * 13) % 41 - 20, (n * 7) % 37 - 18) for n in range(count)]
    phase = 0.0
    on: list[int] = []
    for n in range(count):
        phase += 2 * math.pi * (300_000 + 1_200_000 * math.sin(2 * math.pi * n / 1280)) / sample_rate
        on.append(word(int(250 * math.cos(phase)), int(250 * math.sin(phase))))
    compared = compare_vtx_metrics(iq_metrics(off, sample_rate), iq_metrics(on, sample_rate))
    assert compared["rf_iq"]["changed"]
    assert compared["wbfm"]["changed"]
    with tempfile.TemporaryDirectory() as temporary:
        recorder = SessionRecorder("self-test", Path(temporary), {"profile_id": "test"})
        recorder.record_command("CAPTURE 3")
        recorder.next_iq_label("synthetic")
        recorder.record_line("C5VRX_IQ_BEGIN samples=3")
        for value in on[:3]:
            recorder.record_line(f"IQ:{value:08x}")
        recorder.record_line("C5VRX_IQ_END")
        recorder.record_line("C5VRX_STATUS profile=test firmware=C5VRX version=abc idf=v6.0.2 protocol=8")
        descriptor = FRAME_DESCRIPTOR.pack(2, 2, 2, PIXEL_FORMAT_GRAY8, 1)
        recorder.record_packet(Packet(PACKET_GRAY8_FRAME, 1, 100, descriptor + b"\x00\x40\x80\xff"))
        recorder.finalize({"status": "PASS", "passed": True})
        bundle = recorder.create_bundle()
        recorder.close()
        with zipfile.ZipFile(bundle) as archive:
            names = set(archive.namelist())
        required = {
            "raw-serial.bin", "raw-serial.log", "measurements.json",
            "results.json",
            "firmware-git-version.json", "board-profile.json",
            "test-configuration.json", "iq/index.json", "preview/index.json",
            "errors-failures.json", "session.json",
        }
        assert required <= names, required - names
    print("c5vrx_lab self-test PASS", file=sys.stderr)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    subparsers = parser.add_subparsers(dest="subcommand", required=True)
    auto = subparsers.add_parser("auto-test", help="run the existing bounded first-hardware suite")
    add_connection_args(auto)
    auto.add_argument("--include-vtx-proof", action="store_true", help="append the interactive A4 VTX proof")
    auto.add_argument("--yes", action="store_true", help="fixture controls VTX state; skip operator checkpoints")
    auto.add_argument("--capture-words", type=int, default=DEFAULT_CAPTURE_WORDS, choices=[1024, 2048, 4096, 8192, 16384])
    auto.add_argument("--preview-mode", choices=["guarded", "experimental", "none"], default="guarded")
    vtx = subparsers.add_parser("vtx-proof", help="compare bounded A4 captures with VTX off and on")
    add_connection_args(vtx)
    vtx.add_argument("--yes", action="store_true", help="fixture controls VTX state; skip operator checkpoints")
    vtx.add_argument("--capture-words", type=int, default=DEFAULT_CAPTURE_WORDS, choices=[1024, 2048, 4096, 8192, 16384])
    vtx.add_argument("--preview-mode", choices=["guarded", "experimental", "none"], default="guarded")
    ports = subparsers.add_parser("ports", help="list and rank serial ports")
    export = subparsers.add_parser("export", help="create a Codex bundle from an existing session")
    export.add_argument("session", type=Path)
    export.add_argument("--output", type=Path)
    subparsers.add_parser("self-test", help="exercise parsers, metrics, artifacts and ZIP export")
    args = parser.parse_args()

    if args.subcommand == "ports":
        print(json.dumps({"schema": SCHEMA_VERSION, "ports": candidate_ports()}, sort_keys=True))
        return 0
    if args.subcommand == "self-test":
        self_test()
        print(json.dumps({"schema": SCHEMA_VERSION, "test": "self-test", "status": "PASS", "passed": True}))
        return 0
    if args.subcommand == "export":
        session = args.session.expanduser().resolve()
        if not session.is_dir():
            parser.error(f"session folder does not exist: {session}")
        destination = (args.output or session.with_name(session.name + "-codex-bundle.zip")).resolve()
        with zipfile.ZipFile(destination, "w", compression=zipfile.ZIP_DEFLATED) as archive:
            for path in sorted(session.rglob("*")):
                if path.is_file() and path.resolve() != destination:
                    archive.write(path, path.relative_to(session))
        print(json.dumps({"schema": SCHEMA_VERSION, "status": "PASS", "passed": True, "bundle": str(destination)}))
        return 0

    recorder: SessionRecorder | None = None
    serial_port: Any = None
    try:
        recorder, connection, serial_port = open_lab(args, args.subcommand)
        result = auto_test(connection, args) if args.subcommand == "auto-test" else vtx_proof(connection, args)
        if args.subcommand == "auto-test" and args.include_vtx_proof:
            proof = vtx_proof(connection, args)
            result["vtx_proof"] = proof
            if not proof["passed"]:
                result["passed"] = False
                result["status"] = "FAIL"
                result["reasons"].append("included VTX proof failed")
        result["session"] = str(recorder.path)
        if not result["passed"]:
            for reason in result.get("reasons", []):
                recorder.record_error("TEST_FAILURE", str(reason))
        recorder.finalize(result)
        bundle = recorder.create_bundle()
        result["bundle"] = str(bundle)
        recorder.finalize(result)
        print(json.dumps(result, sort_keys=True))
        return 0 if result["passed"] else 1
    except KeyboardInterrupt:
        if recorder:
            recorder.record_error("INTERRUPTED", "operator interrupted test")
            recorder.finalize({"status": "ERROR", "passed": False, "reason": "interrupted"})
        print(json.dumps({"schema": SCHEMA_VERSION, "status": "ERROR", "passed": False, "reason": "interrupted"}))
        return 2
    except Exception as exc:
        if recorder:
            recorder.record_error("LAB_ERROR", str(exc))
            recorder.finalize({"status": "ERROR", "passed": False, "reason": str(exc)})
        print(json.dumps({"schema": SCHEMA_VERSION, "status": "ERROR", "passed": False, "reason": str(exc)}))
        return 2
    finally:
        if serial_port is not None:
            try:
                serial_port.close()
            except Exception:
                pass
        if recorder:
            recorder.close()


if __name__ == "__main__":
    raise SystemExit(main())
