#!/usr/bin/env python3
"""C5VRX-3 foxhunt HUD: MJPEG video + radio meters + receiver control over LAN.

Serves the USB video preview (224x168 GRAY8, row-streamed binary packets on
the USB Serial/JTAG console; legacy 128x96 whole frames still accepted) as
an MJPEG stream any browser can watch, with a
HUD-style dashboard: animated carrier-level meter with peak-hold, Q_phase
and gain bars, a 60 s level-history graph, a 48-channel direct-tune grid,
and console-key control buttons. Mobile-friendly, so an iPhone on the LAN
can drive the receiver. No external JS/CSS - works offline on a LAN.

Dependencies (pip3 install --user ... if missing):
    pyserial   - serial port access
    flask      - web framework
    Pillow     - JPEG encoding / placeholder rendering

Usage:
    python3 tools/flask_app.py [serial_port] [--host 0.0.0.0] [--port 5000]

The console key protocol is the firmware's: 'g' scanner, 'c' channel,
'C' band, 'u' buzzer mute, 'b' BW gear, 'a' AGC mode, 'k' dead-channel
skip. The preview stream is always-on in current firmware ('p' is ignored
and never sent). Direct tune is the firmware's two-byte command: 't' arms,
the next byte is the channel index 0..47 (R1=0, A1=8, B1=16, E1=24, F1=32,
L1=40); on firmware without 't' support the bytes degrade to harmless
console input. Binary frames use the v1 packet protocol parsed by the
unmodified legacy tools/c5vrx_usb_protocol.py (shared with preview_viewer.py).

Detection episodes (carrier lock -> lost) are appended as one row each to
tools/detections.csv; the DETECTIONS tab lists them (/api/detections), the
EXPORT CSV button downloads the file (/api/detections/export), and CLEAR
truncates it to header-only (/api/detections/clear). The camera button on
the video card saves the current frame to tools/shots/ (episode-named when
a detection is open); rows with a matching shot show a thumbnail.
"""

from __future__ import annotations

import argparse
import csv
import glob
import io
import os
import re
import sys
import threading
import time
from datetime import datetime
from pathlib import Path

sys.path.insert(
    0,
    str(Path(__file__).resolve().parent.parent / "legacy" / "c5vrx1" / "tools"),
)
from c5vrx_usb_protocol import (  # noqa: E402
    FRAME_DESCRIPTOR,
    PACKET_GRAY8_FRAME,
    PACKET_GRAY8_ROWS,
    PACKET_STREAM_END,
    PACKET_STREAM_INFO,
    PIXEL_FORMAT_GRAY8,
    StreamDecoder,
    decode_gray8_rows,
)

import serial  # noqa: E402
from flask import Flask, Response, jsonify, send_from_directory  # noqa: E402
from PIL import Image, ImageDraw, ImageFont  # noqa: E402

WIDTH = 224                 # default canvas: firmware now streams 224x168
HEIGHT = 168                # as GRAY8_ROWS packets (old 128x96 type-2 frames
SCALE = 2                   # still accepted; STREAM_INFO can override dims)
JPEG_QUALITY = 70
PLACEHOLDER_AFTER_S = 15.0  # serve "NO SIGNAL" only when disconnected, or
                            # no frames at all for this long; live snow
                            # (even stale by a few seconds) always shows
FRAME_HEARTBEAT_S = 30.0    # idle wake-up only; no frame is pushed unless
                            # frame_seq or the connection state changed
MJPEG_KEEPALIVE_S = 1.0     # re-push the last REAL frame at ~1 Hz when the
                            # firmware frame flow stalls (never placeholder)
ALLOWED_KEYS = set("gcCubamsd+-0fjkoOvx")
KEY_MIN_INTERVAL_S = 0.1    # ~10/s max serial-writing rate across all endpoints
DESIRED_TIMEOUT_S = 6.0     # give up on a toggle the firmware never confirms
TUNE_BYTE_GAP_S = 0.15      # gap between 't' arm byte and index byte
CHANNEL_COUNT = 48

# Standard FPV channel table: 6 bands x 8, band*8+channel indexing,
# mirroring s_fpv_channels in main/rf.c (R, A, B, E, F, L).
CHANNELS = [
    ("R", [5658, 5695, 5732, 5769, 5806, 5843, 5880, 5917]),
    ("A", [5865, 5845, 5825, 5805, 5785, 5765, 5745, 5725]),
    ("B", [5733, 5752, 5771, 5790, 5809, 5828, 5847, 5866]),
    ("E", [5705, 5685, 5665, 5645, 5885, 5905, 5925, 5945]),
    ("F", [5740, 5760, 5780, 5800, 5820, 5840, 5860, 5880]),
    ("L", [5362, 5399, 5436, 5473, 5510, 5547, 5584, 5621]),
]

# [AGC:ACT] TRACK BW=BW40 | R6:5843.000MHz +0kHz (CFO= +12 kHz) | G=34 | P=25 Q=96% | c=0 | L=28.4dB
AGC_RE = re.compile(
    r"\[AGC:(?P<mode>\w+)\]\s+(?P<state>\w+)\s+BW=(?P<bw>\w+)\s*\|\s*"
    r"(?P<chan>\w+):(?P<mhz>[\d.]+)MHz\s+(?P<off>[+-]?\d+)kHz\s+"
    r"\(CFO=\s*(?P<cfo>[+-]?\d+)\s*kHz\)\s*\|\s*G=\s*(?P<gain>\d+)\s*\|\s*"
    r"P=\s*(?P<p>\d+)\s+Q=\s*(?P<q>\d+)%\s*\|\s*c=\s*(?P<clip>\d+)"
    r"(?:\s*\|\s*L=\s*(?P<level>[-\d.]+)dB)?"
)


def parse_level_db(raw):
    """Decode the firmware L= field to a float dB value.

    video.c prints it as %2d.%d from s_last_level_dbc/10 and %10, so
    negative values come out malformed by C's truncating division:
    dbc=-23 -> "L=-2.-3dB", dbc=-5 -> "L= 0.-5dB".  Rebuild those as
    int + frac/10.  Returns None when the text is undecodable so the
    caller can keep the last-known value.
    """
    try:
        return float(raw)
    except (TypeError, ValueError):
        pass
    m = re.match(r"^(-?\d+)\.(-?\d+)$", raw or "")
    if m:
        return int(m.group(1)) + int(m.group(2)) / 10.0
    return None
LOCKED_RE = re.compile(r"\[CARRIER\] Locked on (\w+) \((\d+) MHz\) in (\S+)")
LOST_RE = re.compile(r"\[CARRIER\] Lost")
EPISODE_RE = re.compile(
    r"\[EPISODE\] cfo_ppm=(?P<ppm>\S+) video_std=(?P<std>\S*) line_us=(?P<line>\S*)")
SCAN_ON_RE = re.compile(r"\[SCAN\] ON")
SCAN_OFF_RE = re.compile(r"\[SCAN\] OFF")
# Firmware-driven scan resume: after a lost lock, or after a candidate that
# did not confirm — both mean the hop loop is running again.
SCAN_AUTO_RE = re.compile(r"\[SCAN\] AUTO - resuming")
SCAN_RESUME_RE = re.compile(r"\[SCAN\] Candidate not confirmed")
SCAN_CAND_RE = re.compile(r"\[SCAN\] Carrier candidate")
AUDIO_RE = re.compile(r"\[AUDIO\] Buzzer (ENABLED|MUTED)")
SKIP_DEAD_RE = re.compile(r"\[SWEEP\] dead-channel skip (ON|OFF)")
SWEEP_RE = re.compile(r"\[SWEEP\] (?P<chan>\w+) P=(?P<p>\d+) Q=(?P<q>\d+)")
CHANNEL_RE = re.compile(r"\[(?:CHANNEL|MENU: CHANNEL)\].*?(\w+) \((\d+) MHz\) in (\S+)")
TUNE_RE = re.compile(r"\[TUNE\] -> (\w+) \((\d+) MHz\) in (\S+)")
BAND_RE = re.compile(r"\[(?:BAND|MENU: BAND)\].*?(\S+)\s*$")
PREVIEW_ON_RE = re.compile(r"\[PREVIEW\] ON")
PREVIEW_UNAVAIL_RE = re.compile(r"\[PREVIEW\] UNAVAILABLE")
PREVIEW_OFF_RE = re.compile(
    r"\[PREVIEW\] OFF - frames completed=(\d+) sent=(\d+) dropped=(\d+)"
    r" \(lines=(\d+), vsyncs=(\d+)\)")
BOOT_RE = re.compile(r"Seamless 16K Phase5 receiver")


class State:
    """Shared state between the serial reader thread and Flask workers."""

    def __init__(self) -> None:
        self.lock = threading.Lock()
        self.frame_cond = threading.Condition(self.lock)
        self.frame = None            # latest complete GRAY8 frame bytes
        self.frame_w = WIDTH         # live canvas dims (STREAM_INFO can
        self.frame_h = HEIGHT        # override; default 224x168)
        # Row-accumulation canvas for PACKET_GRAY8_ROWS: rows may skip
        # indices (grabber give-ups), so unwritten rows keep their previous
        # content -- the same "display keeps old row" contract as the old
        # whole-frame format.
        self.row_buf = bytearray(WIDTH * HEIGHT)
        self.frame_seq = -1
        self.frame_time = 0.0
        self.frame_times = []        # timestamps for FPS estimation
        self.connected = False
        self.preview_on = False      # tracked from firmware prints/packets;
                                     # read-only: firmware streams from boot,
                                     # 'p' is ignored and never sent
        self.preview_available = True  # firmware prints UNAVAILABLE if not
        self.scan = False
        self.scan_desired = None     # None = no user preference yet: the
        self.skip_dead = True        # firmware default: dead-channel skip ON
        self.skip_desired = None     # supervisor leaves it alone until touched
        # Toggle name -> monotonic time of the last user-initiated desired
        # change. If the firmware has not confirmed within DESIRED_TIMEOUT_S
        # (e.g. old firmware lacking the toggle), the supervisor clears the
        # latch instead of retrying forever.
        self.desired_since = {}
        self.buzzer_enabled = True   # supervisor leaves firmware scan/buzzer
        self.buzzer_desired = None   # alone until the user touches a toggle
        self.locked = False
        # Per-channel sweep record, fixed order R1..R8,A1..A8,...,L1..L8.
        # p/q stay None until the firmware emits a "[SWEEP] ch P=.. Q=.." hop
        # line; the UI renders those as empty baseline bars.
        self.spectrum = [
            {"name": f"{band}{i + 1}", "band": band, "freq_mhz": freqs[i],
             "p": None, "q": None, "age_s": None, "sweeps_count": 0}
            for band, freqs in CHANNELS for i in range(8)
        ]
        # Packet/parser instrumentation: which pipeline stage is alive.
        self.pkts_info = 0           # STREAM_INFO packets (preview task opened)
        self.pkts_frame = 0          # GRAY8_FRAME packets (legacy whole frames)
        self.pkts_rows = 0           # GRAY8_ROWS packets (row-streamed video)
        self.pkts_end = 0            # STREAM_END packets
        self.parse_errors = 0        # v1 framing/CRC errors on the wire
        self.last_frame_flags = None # GRAY8 descriptor flags (bit0 = fw h_locked)
        self.preview_stats = None    # parsed from the "[PREVIEW] OFF" print
        self.serial_port = None      # port actually opened (may be discovered)
        self.last_tune_time = 0.0    # monotonic time of last [TUNE] print
        self.telemetry = {
            "agc_mode": "-", "agc_state": "-", "bw": "-",
            "channel": "-", "freq_mhz": 0.0, "offset_khz": 0, "cfo_khz": 0,
            "gain": 0, "p_median": 0, "q_phase": 0, "clip": 0,
            "level_db": 0.0, "band": "-",
        }

    def put_frame(self, payload: bytes, seq: int) -> None:
        now = time.monotonic()
        with self.frame_cond:
            self.frame = payload
            self.frame_seq = seq
            self.frame_time = now
            self.preview_on = True   # flowing frames prove the stream is on
            self.frame_times = [t for t in self.frame_times if now - t < 5.0]
            self.frame_times.append(now)
            self.frame_cond.notify_all()

    def fps(self) -> float:
        with self.lock:
            if len(self.frame_times) < 2:
                return 0.0
            span = max(self.frame_times[-1] - self.frame_times[0], 1e-6)
            return (len(self.frame_times) - 1) / span


STATE = State()


# ----- Detection episode logging (tools/detections.csv) -----
#
# One CSV row per detection episode (lock acquire -> lock lost), plus
# standalone event rows (boot). Episode state is mutated only under
# STATE.lock (parse_line / SerialManager / packet hook); the file write is a
# single small append guarded by its own lock so the serial reader thread
# never blocks on anything bigger than one row.

DETECTIONS_CSV = Path(__file__).resolve().parent / "detections.csv"
DETECTIONS_HEADER = [
    "start_iso", "end_iso", "duration_s",
    "channel", "band", "freq_mhz",
    "level_peak_db", "level_mean_db", "level_min_db", "level_samples",
    "video_sync", "frames_received", "max_fps",
    "skip_dead", "dwell_ms", "lock_type", "end_reason",
    "cfo_ppm", "video_std", "line_us",
]
DEFAULT_DWELL_MS = 200            # scanner full dwell (firmware video.c)

_dets_lock = threading.Lock()
EPISODE = None                    # open episode dict, or None (STATE.lock)


def _det_count_on_disk() -> int:
    try:
        with open(DETECTIONS_CSV, newline="") as f:
            return max(0, sum(1 for _ in f) - 1)
    except OSError:
        return 0


_dets_total = _det_count_on_disk()


def _det_coerce(v):
    """CSV cell -> JSON value: numbers where numeric, '' for blank."""
    if v is None or v == "":
        return ""
    try:
        return int(v)
    except ValueError:
        try:
            return float(v)
        except ValueError:
            return v


def _det_read_rows(limit: int = 500):
    """(rows, total): newest-first dicts in CSV column order, capped at
    `limit`; ([], 0) when no CSV exists yet."""
    with _dets_lock:
        try:
            with open(DETECTIONS_CSV, newline="") as f:
                rows = list(csv.DictReader(f))
        except OSError:
            return [], 0
    total = len(rows)
    rows.reverse()
    return [{k: _det_coerce(v) for k, v in r.items()} for r in rows[:limit]], total


# ----- Video screenshots (tools/shots/) -----
#
# One JPEG per camera-button press. Named <episode-start>__<HHMMSS>.jpg
# when an episode is open so the DETECTIONS tab can match shots to rows
# (row start_iso, both sanitized to YYYY-MM-DDTHH-MM-SS); otherwise the
# capture time is the base. Plain files, served back via /shots/<name>.

SHOTS_DIR = Path(__file__).resolve().parent / "shots"


def _shot_stamp(ts: float) -> str:
    """Filename-safe local timestamp matching the CSV start_iso prefix."""
    return _iso(ts)[:19].replace(":", "-")


def _shot_list() -> list:
    try:
        return sorted(p.name for p in SHOTS_DIR.iterdir()
                      if p.suffix == ".jpg")
    except OSError:
        return []


def _iso(ts: float) -> str:
    return datetime.fromtimestamp(ts).astimezone().isoformat(timespec="seconds")


def _det_write_row(row: dict) -> None:
    """Append one row (and the header on first creation). Small and fast:
    a single ~200-byte append per episode, never on a hot path."""
    global _dets_total
    with _dets_lock:
        new = not DETECTIONS_CSV.exists()
        with open(DETECTIONS_CSV, "a", newline="") as f:
            w = csv.DictWriter(f, fieldnames=DETECTIONS_HEADER)
            if new:
                w.writeheader()
            w.writerow(row)
        _dets_total += 1


def _fps_unlocked() -> float:
    """STATE.fps() without taking STATE.lock (caller already holds it)."""
    ft = STATE.frame_times
    if len(ft) < 2:
        return 0.0
    span = max(ft[-1] - ft[0], 1e-6)
    return (len(ft) - 1) / span


def _episode_start(chan: str, freq_mhz: str, band: str) -> None:
    """Caller holds STATE.lock."""
    global EPISODE
    if EPISODE is not None:
        _episode_end("relock")
    EPISODE = {
        "start": time.time(),
        "channel": chan, "band": band, "freq_mhz": freq_mhz,
        "peak": None, "min": None, "sum": 0.0, "n": 0,
        "video_sync": False,
        "frames0": STATE.pkts_frame,
        "max_fps": 0.0,
        "skip_dead": STATE.skip_dead,
        # A lock right after a user/direct tune is a manual lock, not a
        # scanner find.
        "lock_type": ("manual"
                      if time.monotonic() - STATE.last_tune_time < 4.0
                      else "scanner"),
        # OSINT fields, filled by the firmware [EPISODE] line emitted just
        # before [CARRIER] Lost; blank when it never arrives (relock/retune
        # endings) or when a fact is unknown.
        "cfo_ppm": "", "video_std": "", "line_us": "",
    }


def _episode_tick(level_db, fps: float) -> None:
    """Per telemetry line while locked. Caller holds STATE.lock."""
    ep = EPISODE
    if ep is None:
        return
    if level_db is not None:
        ep["peak"] = level_db if ep["peak"] is None else max(ep["peak"], level_db)
        ep["min"] = level_db if ep["min"] is None else min(ep["min"], level_db)
        ep["sum"] += level_db
        ep["n"] += 1
    if fps > ep["max_fps"]:
        ep["max_fps"] = fps


def _episode_video_sync() -> None:
    """A GRAY8 frame arrived with descriptor flags bit0 (h_locked) set."""
    if EPISODE is not None:
        EPISODE["video_sync"] = True


def _episode_end(reason: str) -> None:
    """Close the open episode and append its row. Caller holds STATE.lock."""
    global EPISODE
    ep = EPISODE
    if ep is None:
        return
    EPISODE = None
    end = time.time()
    row = {
        "start_iso": _iso(ep["start"]),
        "end_iso": _iso(end),
        "duration_s": round(end - ep["start"], 1),
        "channel": ep["channel"],
        "band": ep["band"],
        "freq_mhz": ep["freq_mhz"],
        "level_peak_db": "" if ep["peak"] is None else round(ep["peak"], 2),
        "level_mean_db": "" if not ep["n"] else round(ep["sum"] / ep["n"], 2),
        "level_min_db": "" if ep["min"] is None else round(ep["min"], 2),
        "level_samples": ep["n"],
        "video_sync": int(ep["video_sync"]),
        "frames_received": STATE.pkts_frame - ep["frames0"],
        "max_fps": round(ep["max_fps"], 1),
        "skip_dead": int(ep["skip_dead"]),
        "dwell_ms": DEFAULT_DWELL_MS,
        "lock_type": ep["lock_type"],
        "end_reason": reason,
        "cfo_ppm": ep["cfo_ppm"],
        "video_std": ep["video_std"],
        "line_us": ep["line_us"],
    }
    _det_write_row(row)


def _det_event(name: str) -> None:
    """Standalone notable event as its own row (e.g. firmware boot)."""
    now = _iso(time.time())
    _det_write_row({
        "start_iso": now, "end_iso": now, "duration_s": 0,
        "channel": "", "band": "", "freq_mhz": "",
        "level_peak_db": "", "level_mean_db": "", "level_min_db": "",
        "level_samples": 0, "video_sync": "", "frames_received": "",
        "max_fps": "", "skip_dead": "", "dwell_ms": "",
        "lock_type": "event", "end_reason": name,
        "cfo_ppm": "", "video_std": "", "line_us": "",
    })


def parse_line(line: str) -> None:
    m = AGC_RE.search(line)
    with STATE.lock:
        if PREVIEW_UNAVAIL_RE.search(line):
            # Preview task missing on the firmware (buffer alloc failed at
            # boot). Pin the read-only preview flags off.
            STATE.preview_available = False
            STATE.preview_on = False
            return
        if PREVIEW_ON_RE.search(line):
            if STATE.preview_available:
                STATE.preview_on = True
            return
        pm = PREVIEW_OFF_RE.search(line)
        if pm:
            STATE.preview_on = False
            STATE.preview_stats = {
                "frames_completed": int(pm.group(1)),
                "frames_sent": int(pm.group(2)),
                "frames_dropped": int(pm.group(3)),
                "lines_captured": int(pm.group(4)),
                "vsyncs": int(pm.group(5)),
            }
            return
        if BOOT_RE.search(line):
            # Firmware rebooted: every console-toggle state resets to default.
            _episode_end("firmware reboot")
            _det_event("boot")
            STATE.preview_on = False
            STATE.preview_available = True
            STATE.scan = False
            STATE.buzzer_enabled = True
            STATE.locked = False
            return
        if m:
            t = STATE.telemetry
            t["agc_mode"] = m.group("mode")
            t["agc_state"] = m.group("state")
            t["bw"] = m.group("bw")
            t["channel"] = m.group("chan")
            t["freq_mhz"] = float(m.group("mhz"))
            t["offset_khz"] = int(m.group("off"))
            t["cfo_khz"] = int(m.group("cfo"))
            t["gain"] = int(m.group("gain"))
            t["p_median"] = int(m.group("p"))
            t["q_phase"] = int(m.group("q"))
            t["clip"] = int(m.group("clip"))
            lv = None
            if m.group("level") is not None:
                lv = parse_level_db(m.group("level"))
                if lv is not None:
                    t["level_db"] = lv
            STATE.locked = m.group("state") == "TRACK"
            _episode_tick(lv, _fps_unlocked())
            return
        sm = SWEEP_RE.search(line)
        if sm:
            STATE.scan = True  # a hop line is proof the scan loop is running
            name = sm.group("chan")
            for entry in STATE.spectrum:
                if entry["name"] == name:
                    entry["p"] = int(sm.group("p"))
                    entry["q"] = int(sm.group("q"))
                    entry["age_s"] = 0.0
                    entry["sweeps_count"] += 1
                    break
            return
        m = LOCKED_RE.search(line)
        if m:
            STATE.locked = True
            STATE.telemetry["channel"] = m.group(1)
            STATE.telemetry["freq_mhz"] = float(m.group(2))
            STATE.telemetry["band"] = m.group(3)
            _episode_start(m.group(1), m.group(2), m.group(3))
            return
        if LOST_RE.search(line):
            STATE.locked = False
            _episode_end("signal lost")
            return
        em = EPISODE_RE.search(line)
        if em:
            # Emitted by the firmware just before [CARRIER] Lost: attach the
            # end-of-episode facts to the still-open episode.
            if EPISODE is not None:
                EPISODE["cfo_ppm"] = em.group("ppm")
                EPISODE["video_std"] = em.group("std")
                EPISODE["line_us"] = em.group("line")
            return
        if SCAN_ON_RE.search(line) or SCAN_AUTO_RE.search(line) \
                or SCAN_RESUME_RE.search(line):
            # Firmware says the hop loop runs: adopt it as the preference too,
            # so the supervisor never fights firmware-driven scanning.
            _episode_end("scan restarted")
            STATE.scan = True
            STATE.scan_desired = True
            return
        if SCAN_OFF_RE.search(line) or SCAN_CAND_RE.search(line):
            STATE.scan = False
            STATE.scan_desired = False
            return
        am = AUDIO_RE.search(line)
        if am:
            STATE.buzzer_enabled = am.group(1) == "ENABLED"
            STATE.buzzer_desired = STATE.buzzer_enabled
            return
        km = SKIP_DEAD_RE.search(line)
        if km:
            STATE.skip_dead = km.group(1) == "ON"
            STATE.skip_desired = STATE.skip_dead
            return
        m = TUNE_RE.search(line)
        if m:
            STATE.telemetry["channel"] = m.group(1)
            STATE.telemetry["freq_mhz"] = float(m.group(2))
            STATE.telemetry["band"] = m.group(3)
            STATE.locked = False
            STATE.last_tune_time = time.monotonic()
            _episode_end("user retuned")
            return
        m = CHANNEL_RE.search(line)
        if m:
            STATE.telemetry["channel"] = m.group(1)
            STATE.telemetry["freq_mhz"] = float(m.group(2))
            STATE.telemetry["band"] = m.group(3)
            return
        m = BAND_RE.search(line)
        if m and "Switched" in line:
            STATE.telemetry["band"] = m.group(1)


class SerialManager(threading.Thread):
    """Owns the serial port: reconnect loop, packet parsing, rate-limited writes."""

    def __init__(self, port: str, baud: int) -> None:
        super().__init__(daemon=True)
        self.port = port
        self.baud = baud
        self.ser = None
        self.write_lock = threading.Lock()
        self.stop_event = threading.Event()
        self._last_write = 0.0
        self._last_supervisor = 0.0
        self._last_age = time.monotonic()

    def _resolve_port(self) -> str:
        """Use the configured port if present; otherwise discover any
        /dev/cu.usbmodem* node (the usbmodem suffix changes when the board
        is replugged into a different physical USB port)."""
        if os.path.exists(self.port):
            return self.port
        candidates = sorted(glob.glob("/dev/cu.usbmodem*"))
        if candidates:
            if candidates[0] != self.port:
                print(f"[serial] {self.port} missing; discovered {candidates[0]}")
            return candidates[0]
        return self.port  # let the open fail and retry with backoff

    def _open(self) -> None:
        port = self._resolve_port()
        ser = serial.Serial(port, self.baud, timeout=0.2)
        ser.reset_input_buffer()
        self.ser = ser
        with STATE.lock:
            STATE.connected = True
            STATE.serial_port = port
        with STATE.frame_cond:
            STATE.frame_cond.notify_all()

    def _supervise(self) -> None:
        """Converge the firmware toggles onto their desired states.

        'g' (scan), 'u' (buzzer) and 'k' (dead-channel skip) are all toggles,
        so state must be tracked, not assumed: the firmware prints "[SCAN]
        ON/OFF/AUTO - resuming/Carrier candidate", "[AUDIO] Buzzer
        ENABLED/MUTED" and "[SWEEP] dead-channel skip ON/OFF" on every change
        (parsed into the STATE flags), and a boot banner resets them. If
        tracked != desired, send one key and wait for the firmware print to
        confirm before retrying (3 s guard). Worst case (board state unknown
        after app restart) costs two toggles; it can never oscillate because
        the firmware always acknowledges. If no acknowledgement arrives
        within DESIRED_TIMEOUT_S of a user-initiated change (e.g. old
        firmware that lacks the toggle), the desired latch is cleared back to
        hands-off instead of retrying forever. Preview is NOT supervised:
        the firmware streams from boot and ignores 'p'."""
        now = time.monotonic()
        with STATE.lock:
            for entry in STATE.spectrum:
                if entry["age_s"] is not None:
                    entry["age_s"] = round(entry["age_s"] + now - self._last_age, 1)
            raw = []
            # scan/buzzer/skip desired start as None: no preference means leave
            # the firmware as-is until the user touches the toggle. Firmware
            # prints ("[SCAN] AUTO - resuming" etc.) sync desired to the real
            # state.
            if STATE.scan_desired is not None:
                raw.append(("scan", "g", STATE.scan, STATE.scan_desired))
            if STATE.buzzer_desired is not None:
                raw.append(("buzzer", "u", STATE.buzzer_enabled, STATE.buzzer_desired))
            if STATE.skip_desired is not None:
                raw.append(("skip", "k", STATE.skip_dead, STATE.skip_desired))
            pairs = []
            for name, key, actual, desired in raw:
                if actual == desired:
                    STATE.desired_since.pop(name, None)  # confirmed
                    continue
                since = STATE.desired_since.get(name)
                if since is not None and now - since > DESIRED_TIMEOUT_S:
                    # Never confirmed (e.g. old firmware without this
                    # toggle): drop the latch, back to hands-off.
                    if name == "scan":
                        STATE.scan_desired = None
                    elif name == "buzzer":
                        STATE.buzzer_desired = None
                    else:
                        STATE.skip_desired = None
                    STATE.desired_since.pop(name, None)
                    continue
                pairs.append((key, actual, desired))
        self._last_age = now
        if now - self._last_supervisor < 3.0:
            return
        for key, actual, desired in pairs:
            if actual != desired and self.send_key(key):
                self._last_supervisor = now
                return

    def _write(self, data: bytes) -> bool:
        try:
            if self.ser is None:
                return False
            self.ser.write(data)
            self.ser.flush()
            return True
        except (serial.SerialException, OSError):
            return False

    def send_key(self, key: str) -> bool:
        """Single console key, rate-limited with every other write endpoint."""
        with self.write_lock:
            now = time.monotonic()
            if now - self._last_write < KEY_MIN_INTERVAL_S:
                return False
            self._last_write = now
            return self._write(key.encode("ascii"))

    def send_park(self, index: int) -> bool:
        """Click-to-lock: stop the scanner (only if running), then the
        two-byte direct tune ('t' arm + index byte, firmware console
        protocol from main/video.c console_diag_task). The whole sequence
        holds the write lock so no supervisor/key write can interleave,
        and scan_desired is latched OFF first so the toggle supervisor
        converges to the parked state instead of re-enabling the scan."""
        with self.write_lock:
            now = time.monotonic()
            if now - self._last_write < KEY_MIN_INTERVAL_S:
                return False
            with STATE.lock:
                scanning = STATE.scan
                if scanning:
                    STATE.scan_desired = False
                    STATE.desired_since["scan"] = now
            if scanning:
                if not self._write(b"g"):
                    return False
                time.sleep(KEY_MIN_INTERVAL_S)
            if not self._write(b"t"):
                return False
            time.sleep(TUNE_BYTE_GAP_S)
            self._last_write = time.monotonic()
            return self._write(bytes([index]))

    def run(self) -> None:
        backoff = 0.5
        decoder = StreamDecoder()
        while not self.stop_event.is_set():
            try:
                if self.ser is None:
                    self._open()
                    backoff = 0.5
                    print(f"[serial] connected to {STATE.serial_port}")
                self._supervise()
                data = self.ser.read(4096)
                if not data:
                    continue
                for kind, value in decoder.feed(data):
                    if kind == "line":
                        parse_line(value)
                    elif kind == "packet":
                        self._packet(value)
                    elif kind == "error":
                        with STATE.lock:
                            STATE.parse_errors += 1
            except (serial.SerialException, OSError) as exc:
                print(f"[serial] lost: {exc}; retrying in {backoff:.1f}s")
                try:
                    if self.ser is not None:
                        self.ser.close()
                except OSError:
                    pass
                self.ser = None
                decoder = StreamDecoder()  # resync framing on the new connection
                with STATE.lock:
                    STATE.connected = False
                    _episode_end("serial disconnect")
                with STATE.frame_cond:
                    STATE.frame_cond.notify_all()
                self.stop_event.wait(backoff)
                backoff = min(backoff * 2.0, 5.0)

    @staticmethod
    def _packet(pkt) -> None:
        if pkt.packet_type == PACKET_GRAY8_FRAME:
            # Legacy whole-frame path (old 128x96 firmware). Validated
            # against the live canvas dims learned from STREAM_INFO.
            with STATE.lock:
                STATE.pkts_frame += 1
            if len(pkt.payload) < FRAME_DESCRIPTOR.size:
                return
            w, h, stride, fmt, flags = FRAME_DESCRIPTOR.unpack_from(pkt.payload)
            frame = pkt.payload[FRAME_DESCRIPTOR.size:]
            with STATE.lock:
                ok = (fmt == PIXEL_FORMAT_GRAY8 and stride == w and
                      (w, h) == (STATE.frame_w, STATE.frame_h) and
                      len(frame) == stride * h)
                if ok:
                    STATE.last_frame_flags = flags
                    if flags & 1:
                        _episode_video_sync()
            if ok:
                STATE.put_frame(frame, pkt.sequence)
        elif pkt.packet_type == PACKET_GRAY8_ROWS:
            # Row-streamed video (current firmware, 224x168). Rows land on
            # the persistent canvas; missing indices keep stale content.
            # A row with flags bit0 set completes a frame -> publish. On
            # frame-final rows, flags bit1 is the grabber-locked state
            # (set = real video sync, clear = snow) and drives the badge.
            with STATE.lock:
                STATE.pkts_rows += 1
                row_bytes = STATE.frame_w
            try:
                rows = decode_gray8_rows(pkt, row_bytes)
            except ValueError:
                with STATE.lock:
                    STATE.parse_errors += 1
                return
            frame = None
            locked = False
            with STATE.frame_cond:
                for r in rows:
                    if r.row_index >= STATE.frame_h:
                        continue
                    start = r.row_index * STATE.frame_w
                    STATE.row_buf[start:start + STATE.frame_w] = r.pixels
                    if r.flags & 1:
                        frame = bytes(STATE.row_buf)
                        locked = bool(r.flags & 2)
            if frame is not None:
                with STATE.lock:
                    STATE.last_frame_flags = 1 if locked else 0
                    if locked:
                        _episode_video_sync()
                STATE.put_frame(frame, pkt.sequence)
        elif pkt.packet_type == PACKET_STREAM_INFO:
            with STATE.lock:
                STATE.pkts_info += 1
                STATE.preview_on = True
                # Descriptor advertises the stream's canvas size (224x168 on
                # current firmware); adopt it and reset the row canvas.
                if len(pkt.payload) >= FRAME_DESCRIPTOR.size:
                    w, h, stride, fmt, flags = FRAME_DESCRIPTOR.unpack_from(
                        pkt.payload)
                    STATE.last_frame_flags = flags
                    if (fmt == PIXEL_FORMAT_GRAY8 and stride == w and
                            16 <= w <= 512 and 16 <= h <= 512 and
                            (w, h) != (STATE.frame_w, STATE.frame_h)):
                        STATE.frame_w = w
                        STATE.frame_h = h
                        STATE.row_buf = bytearray(w * h)
            with STATE.frame_cond:
                STATE.frame_cond.notify_all()
        elif pkt.packet_type == PACKET_STREAM_END:
            with STATE.lock:
                STATE.pkts_end += 1
                STATE.preview_on = False
            with STATE.frame_cond:
                STATE.frame_cond.notify_all()

    def close(self) -> None:
        # Clean shutdown: stop the reader thread. The firmware stream is
        # always-on and ignores 'p', so there is nothing to stop over serial.
        self.stop_event.set()
        try:
            if self.ser is not None:
                self.ser.close()
        except OSError:
            pass


def make_jpeg(image: Image.Image) -> bytes:
    buf = io.BytesIO()
    image.save(buf, format="JPEG", quality=JPEG_QUALITY)
    return buf.getvalue()


PLACEHOLDER_JPEG = None


def placeholder_jpeg() -> bytes:
    global PLACEHOLDER_JPEG
    if PLACEHOLDER_JPEG is None:
        img = Image.new("L", (WIDTH * SCALE, HEIGHT * SCALE), 16)
        draw = ImageDraw.Draw(img)
        # Pillow >= 10.1 scalable default font; anchor='mm' centers on the
        # canvas (448x336) instead of relying on hardcoded top-left offsets.
        big = ImageFont.load_default(size=48)
        small = ImageFont.load_default(size=28)
        draw.text((WIDTH * SCALE // 2, HEIGHT * SCALE // 2 - 14),
                  "NO SIGNAL", font=big, fill=255, anchor="mm")
        draw.text((WIDTH * SCALE // 2, HEIGHT * SCALE // 2 + 18),
                  "standby", font=small, fill=150, anchor="mm")
        PLACEHOLDER_JPEG = make_jpeg(img)
    return PLACEHOLDER_JPEG


def current_jpeg() -> bytes:
    with STATE.lock:
        frame = STATE.frame
        dims = (STATE.frame_w, STATE.frame_h)
        connected = STATE.connected
        age = time.monotonic() - STATE.frame_time if frame else None
    # Placeholder only when genuinely disconnected or frameless for a long
    # stretch; as long as any frames flow (snow included), show the canvas.
    if frame is None or not connected or age > PLACEHOLDER_AFTER_S:
        return placeholder_jpeg()
    img = Image.frombytes("L", dims, frame)
    img = img.resize((dims[0] * SCALE, dims[1] * SCALE), Image.NEAREST)
    return make_jpeg(img)


def mjpeg_stream():
    # Push exactly one image per actual change: a new firmware frame or a
    # connection-state flip.  The placeholder is sent once and then the
    # stream goes quiet — a periodically re-sent placeholder makes the
    # browser re-decode and visibly flash.  Exception: while a REAL firmware
    # frame is the current image (received and not yet stale), a stalled
    # frame flow re-pushes that frame at ~1 Hz so the browser feed never
    # looks dead; the placeholder path stays push-once.
    last_seq = -1
    last_conn = None
    last_push = 0.0
    while True:
        with STATE.frame_cond:
            if STATE.frame_seq == last_seq and STATE.connected == last_conn:
                STATE.frame_cond.wait(timeout=MJPEG_KEEPALIVE_S)
            changed = (STATE.frame_seq != last_seq or
                       STATE.connected != last_conn)
            now = time.monotonic()
            has_real = (STATE.frame is not None and STATE.connected and
                        now - STATE.frame_time <= PLACEHOLDER_AFTER_S)
            keepalive = (not changed and has_real and
                         now - last_push >= MJPEG_KEEPALIVE_S)
            if not changed and not keepalive:
                continue
            last_seq = STATE.frame_seq
            last_conn = STATE.connected
        last_push = now
        yield (b"--frame\r\nContent-Type: image/jpeg\r\n\r\n"
               + current_jpeg() + b"\r\n")


def channel_grid_js() -> str:
    """Render CHANNELS as a JS literal for the tune grid."""
    rows = []
    for band, freqs in CHANNELS:
        cells = ",".join(f"{{n:'{band}{i+1}',f:{f}}}" for i, f in enumerate(freqs))
        rows.append(f"[{cells}]")
    return "[" + ",\n".join(rows) + "]"


PAGE = """<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1, user-scalable=no">
<title>OUI-SPY // C5VRX</title>
<style>
  :root { --grn:#39ff6a; --amb:#ffb000; --red:#ff4a3d; --bg:#050805;
          --panel:#0a120a; --dim:#33502f; --txt:#9fdc9f; }
  * { box-sizing:border-box; -webkit-tap-highlight-color:transparent; }
  button, a { touch-action:manipulation; }   /* no double-tap zoom delay */
  html, body { height:100%; }
  body { background:var(--bg); color:var(--txt);
         font-family:ui-monospace,Menlo,Consolas,monospace;
         margin:0; padding:10px 10px 0; height:100dvh;
         overflow:hidden; display:flex; flex-direction:column; gap:8px; }
  body::before { content:""; position:fixed; inset:0; pointer-events:none; z-index:9;
      background:repeating-linear-gradient(0deg,rgba(0,0,0,.18) 0 1px,transparent 1px 3px); }
  h1 { font-size:.85rem; letter-spacing:.18em; color:var(--amb); margin:0;
       text-shadow:0 0 8px rgba(255,176,0,.5); display:flex;
       justify-content:space-between; align-items:center; flex:0 0 auto; }
  #lock { font-size:.72rem; letter-spacing:.1em; padding:1px 8px; border-radius:4px;
          border:1px solid var(--dim); color:var(--dim); }
  #lock.on  { color:var(--grn); border-color:var(--grn);
              box-shadow:0 0 12px rgba(57,255,106,.6); text-shadow:0 0 6px var(--grn); }
  #lock.scan { color:var(--amb); border-color:var(--amb); animation:pulse 1s infinite; }
  @keyframes pulse { 50% { opacity:.35; } }
  .top { flex:1 1 0; min-height:0; display:flex; flex-direction:column; gap:8px; overflow:hidden; }
  .cols { flex:1 1 auto; min-height:0; display:flex; flex-direction:row; gap:10px;
          overflow:hidden; align-items:stretch; }
  .col { display:flex; flex-direction:column; gap:8px; min-height:0; min-width:0; overflow:hidden; }
  .col.left { flex:1 1 0; }
  .col.mid { flex:1.3 1 0; }
  .col.right { flex:1 1 0; }
  .panel { background:var(--panel); border:1px solid var(--dim); border-radius:6px;
           padding:8px 10px; flex:0 0 auto;
           background-image:repeating-linear-gradient(90deg,rgba(57,255,106,.03) 0 1px,transparent 1px 24px),
                            repeating-linear-gradient(0deg,rgba(57,255,106,.03) 0 1px,transparent 1px 24px); }
  .vidcard { flex:1 1 auto; min-height:0; display:flex; flex-direction:column; }
  .grow { flex:1 1 auto; min-height:0; overflow:auto; }
  .vwrap { flex:1 1 auto; min-height:0; min-width:0; display:flex; overflow:hidden;
           justify-content:center; align-items:center; }
  .screen { display:block; box-sizing:border-box; width:100%; height:100%;
            min-width:0; min-height:0;
            object-fit:contain; object-position:center; image-rendering:pixelated;
            border:1px solid var(--dim);
            background:#000; border-radius:6px; }
  .specwrap { flex:1 1 auto; min-height:0; display:flex; flex-direction:column; }
  #spec { flex:1 1 auto; min-height:70px; width:100%; }
  .lbl { color:var(--dim); font-size:.62rem; letter-spacing:.1em; margin:3px 0 1px; }
  .vmode { float:right; padding:0 6px; border-radius:3px; border:1px solid var(--dim);
           color:var(--dim); letter-spacing:.12em; }
  .vmode.sync { color:var(--grn); border-color:var(--grn);
                text-shadow:0 0 6px rgba(57,255,106,.6); }
  .vmode.static { color:var(--amb); border-color:var(--amb); animation:pulse 1.2s infinite; }
  .vmode.nosignal { color:var(--dim); border-color:var(--dim); }
  .hopchip { padding:0 5px; border-radius:3px; border:1px solid var(--dim);
             color:var(--dim); letter-spacing:.08em; }
  .hopchip.lock { color:var(--grn); border-color:var(--grn);
                  text-shadow:0 0 6px rgba(57,255,106,.6); }
  .hopchip.dwell { color:var(--amb); border-color:var(--amb);
                   animation:pulse .5s infinite; }
  .hopchip.parked { color:#31c8ff; border-color:#31c8ff;
                    text-shadow:0 0 6px rgba(49,200,255,.5); }
  canvas { display:block; width:100%; }
  .bar { height:7px; border:1px solid var(--dim); border-radius:3px; background:#000;
         overflow:hidden; }
  .bar > div { height:100%; width:0%; background:var(--grn); transition:width .3s; }
  .grid48 { display:grid; grid-template-columns:repeat(8,1fr); gap:3px; }
  .ch { font:inherit; font-size:.56rem; line-height:1.2; padding:2px 0; min-height:27px;
        background:#0a0f0a; color:var(--txt); border:1px solid var(--dim);
        border-radius:3px; cursor:pointer; }
  .ch b { display:block; font-size:.64rem; }
  .ch.cur { border-color:var(--grn); color:var(--grn);
            box-shadow:0 0 8px rgba(57,255,106,.5); }
  .ch.sweep { border-color:var(--amb); color:var(--amb); animation:pulse .5s infinite; }
  .ch.parked { border-color:#31c8ff; color:#31c8ff;
               box-shadow:0 0 8px rgba(49,200,255,.5); }
  .bandrow { color:var(--amb); font-size:.58rem; letter-spacing:.12em; margin:4px 0 1px; }
  /* Controls live inside the channel card: a 3-up grid row exactly the
   * grid's width, every button the same generous height, each with a real
   * iOS-style switch (26px track, sliding knob) and a readable label. */
  .cardbar { display:grid; grid-template-columns:repeat(3,1fr); gap:6px;
             margin-top:8px; }
  .cardbar .tg { box-sizing:border-box; height:38px; min-height:0;
                 padding:2px 4px; font-size:.68rem; gap:5px;
                 letter-spacing:.05em; white-space:nowrap;
                 transition:border-color .2s, color .2s, box-shadow .2s; }
  .cardbar .tg .sw { width:26px; height:14px; border-radius:8px;
                     transition:border-color .2s, box-shadow .2s; }
  .cardbar .tg .sw::after { width:10px; height:10px; top:1px; left:1px;
                            transition:left .2s ease, background .2s; }
  .cardbar .tg.on { box-shadow:0 0 10px rgba(57,255,106,.25); }
  .cardbar .tg.on .sw::after { left:13px; }
  .tg { font:inherit; font-size:.6rem; letter-spacing:.08em; display:flex;
        align-items:center; justify-content:center; gap:6px; padding:7px 2px;
        min-height:34px; min-width:0; background:var(--panel); color:var(--dim);
        border:1px solid var(--dim); border-radius:6px; cursor:pointer; }
  .tg .sw { width:22px; height:12px; border:1px solid var(--dim); border-radius:7px;
            position:relative; flex:0 0 auto; }
  .tg .sw::after { content:""; position:absolute; top:1px; left:1px; width:8px;
            height:8px; border-radius:50%; background:var(--dim);
            transition:left .2s, background .2s; }
  .tg.on { color:var(--grn); border-color:var(--grn); }
  .tg.on .sw { border-color:var(--grn); box-shadow:0 0 8px rgba(57,255,106,.5); }
  .tg.on .sw::after { left:11px; background:var(--grn); box-shadow:0 0 6px var(--grn); }
  .tg.pend { color:var(--amb); border-color:var(--amb); animation:pulse .7s infinite; }
  .tg.pend .sw { border-color:var(--amb); }
  .tg.pend .sw::after { background:var(--amb); }
  .tg.dis { opacity:.35; pointer-events:none; }
  #export { text-decoration:none; }
  #detcount { font-size:.6rem; letter-spacing:.1em; color:var(--dim);
              align-self:center; }
  #toast { min-height:1em; margin-top:4px; color:var(--amb); font-size:.6rem;
           text-align:left; }
  .stats { display:grid; grid-template-columns:repeat(auto-fit,minmax(86px,1fr));
           gap:2px 10px; font-size:.66rem; }
  .stats .k { color:var(--dim); }
  .botsplit { flex:1 1 0; min-height:45dvh; display:flex; width:100vw;
              margin:5px 0 0 calc(50% - 50vw); }
  #gwrap { flex:11 1 0; min-width:0; position:relative;
           border-top:1px solid var(--dim);
           background:#071007;
           background-image:repeating-linear-gradient(90deg,rgba(57,255,106,.04) 0 1px,transparent 1px 24px),
                            repeating-linear-gradient(0deg,rgba(57,255,106,.04) 0 1px,transparent 1px 24px); }
  #swrap { flex:9 1 0; min-width:0; position:relative;
           border-top:1px solid var(--dim); border-left:1px solid var(--dim);
           background:#071007;
           background-image:repeating-linear-gradient(90deg,rgba(57,255,106,.04) 0 1px,transparent 1px 24px),
                            repeating-linear-gradient(0deg,rgba(57,255,106,.04) 0 1px,transparent 1px 24px); }
  #fspec { position:absolute; left:0; right:0; bottom:0; width:100%; height:35%;
           border-top:1px solid var(--dim); }
  #wfall { position:absolute; left:0; right:0; top:0; width:100%; height:65%; }
  #holdw { position:absolute; top:5px; right:10px; font-size:.56rem;
           letter-spacing:.1em; color:var(--dim); cursor:pointer;
           user-select:none; z-index:1; }
  #holdw input { accent-color:var(--grn); width:11px; height:11px;
                 vertical-align:-1px; cursor:pointer; }
  #holdw:has(input:checked) { color:var(--grn); }
  #shot { float:right; font:inherit; font-size:.54rem; letter-spacing:.1em;
          margin-left:6px; padding:0 6px; background:var(--panel);
          color:var(--dim); border:1px solid var(--dim); border-radius:3px;
          cursor:pointer; }
  #shot:active { color:var(--grn); border-color:var(--grn); }
  .shotth { width:64px; border:1px solid var(--dim); border-radius:3px;
            cursor:zoom-in; image-rendering:pixelated; vertical-align:middle; }
  #lightbox { display:none; position:fixed; inset:0; z-index:30;
              background:rgba(0,0,0,.88); cursor:zoom-out;
              align-items:center; justify-content:center; }
  #lightbox img { max-width:94vw; max-height:92dvh; image-rendering:pixelated;
                  border:1px solid var(--dim); }
  #fstitle { position:absolute; top:7px; left:14px; font-size:.62rem;
             letter-spacing:.18em; color:var(--dim); pointer-events:none; }
  #hist { position:absolute; inset:0; width:100%; height:100%; }
  #gtitle { position:absolute; top:7px; left:38px; font-size:.62rem;
            letter-spacing:.18em; color:var(--dim); pointer-events:none; }
  #gcur { position:absolute; top:4px; right:14px; font-size:.62rem;
          letter-spacing:.12em; color:var(--dim); pointer-events:none; text-align:right; }
  #gcur b { font-size:1.5rem; color:#2ba84f; text-shadow:0 0 6px rgba(57,255,106,.4); }
  #gcur.lock b { color:var(--grn); text-shadow:0 0 8px rgba(57,255,106,.8); }
  #gcur.scan b { color:var(--amb); text-shadow:0 0 8px rgba(255,176,0,.7);
                 animation:pulse 1s infinite; }
  .tabs { flex:0 0 auto; display:flex; gap:6px; align-items:flex-end;
          justify-content:center; }
  .tab { font:inherit; font-size:.62rem; letter-spacing:.14em; padding:4px 14px;
         background:var(--panel); color:var(--dim); border:1px solid var(--dim);
         border-radius:5px; cursor:pointer; display:flex; align-items:center;
         gap:8px; }
  .tab.on { color:var(--grn); border-color:var(--grn);
            box-shadow:0 0 8px rgba(57,255,106,.35);
            text-shadow:0 0 6px rgba(57,255,106,.6); }
  #detchip { font-size:.52rem; letter-spacing:.08em; color:var(--amb);
             border:1px solid var(--dim); border-radius:3px; padding:0 5px; }
  #view-dets { flex:1 1 0; min-height:0; display:none; flex-direction:column;
               gap:0; background:var(--panel); border:1px solid var(--dim);
               border-radius:6px; overflow:hidden;
               background-image:repeating-linear-gradient(90deg,rgba(57,255,106,.03) 0 1px,transparent 1px 24px),
                                repeating-linear-gradient(0deg,rgba(57,255,106,.03) 0 1px,transparent 1px 24px); }
  .detbar { flex:0 0 auto; display:flex; align-items:center; gap:10px;
            padding:6px 10px; border-bottom:1px solid var(--dim); }
  .detbar .lbl { margin:0; flex:1 1 auto; }
  #dettotal { font-size:.6rem; letter-spacing:.1em; color:var(--dim); }
  .detbar .tg { flex:0 0 auto; height:22px; min-height:0; padding:1px 10px;
                font-size:.54rem; }
  .detwrap { flex:1 1 0; min-height:0; overflow:auto; }
  #dettable { width:100%; border-collapse:collapse; font-size:.6rem; }
  #dettable th { position:sticky; top:0; z-index:1; background:#0d160d;
                 color:var(--amb); font-weight:normal; letter-spacing:.08em;
                 text-align:left; padding:4px 8px; white-space:nowrap;
                 border-bottom:1px solid var(--dim); }
  #dettable td { padding:2px 8px; white-space:nowrap; color:var(--txt);
                 border-bottom:1px solid rgba(51,80,47,.25); }
  #dettable tbody tr:nth-child(even) { background:rgba(57,255,106,.04); }
  #dettable td.na { color:var(--dim); }
  .dch { color:var(--grn); text-shadow:0 0 6px rgba(57,255,106,.5); }
  .vsb { padding:0 5px; border-radius:3px; border:1px solid var(--dim);
         color:var(--dim); letter-spacing:.08em; }
  .vsb.y { color:var(--grn); border-color:var(--grn);
           text-shadow:0 0 6px rgba(57,255,106,.6); }
  /* Detection rows as cards (mobile only; #detcards is display:none on
     desktop, where the sticky-header table is used instead). */
  #detcards { display:none; }
  .detcard { border:1px solid rgba(51,80,47,.5); border-radius:5px;
             padding:5px 8px; margin:0 0 6px; background:rgba(57,255,106,.03); }
  .dcline { display:flex; align-items:center; gap:10px; font-size:.66rem;
            margin-bottom:3px; }
  .dcline .dch { font-size:.68rem; }
  .dcts { color:var(--amb); letter-spacing:.06em; }
  .dcdur { color:var(--dim); }
  .dcline .vsb { margin-left:auto; }
  .dcgrid { display:grid; grid-template-columns:1fr 1fr; gap:1px 12px;
            font-size:.6rem; }
  .dcgrid i { font-style:normal; color:var(--dim); font-size:.56rem;
              margin-right:5px; }
  /* Detections data-table dressing: row accents by episode type, band
     chips, split timestamps, right-aligned numerics, level spark strip. */
  #dettable tr.vs { background:rgba(57,255,106,.05); }
  #dettable tr.vs td:first-child { border-left:2px solid var(--grn); }
  #dettable tr.ev { background:rgba(255,176,0,.04); }
  #dettable tr.ev td:first-child { border-left:2px solid var(--amb); }
  #dettable td.num { text-align:right; font-variant-numeric:tabular-nums; }
  #dettable th.sps { min-width:64px; }
  .dt-d { color:var(--dim); }
  .dt-t { color:var(--txt); }
  .bchip { display:inline-block; min-width:22px; text-align:center;
           padding:0 4px; border-radius:3px; border:1px solid;
           font-size:.58rem; letter-spacing:.05em; }
  .bchip.bR { color:#ffb000; border-color:#ffb000; }
  .bchip.bA { color:#39ff6a; border-color:#39ff6a; }
  .bchip.bB { color:#31c8ff; border-color:#31c8ff; }
  .bchip.bE { color:#c96bff; border-color:#c96bff; }
  .bchip.bF { color:#ff8c3a; border-color:#ff8c3a; }
  .bchip.bL { color:#8a958a; border-color:#8a958a; }
  #detsum { font-size:.58rem; letter-spacing:.05em; color:var(--dim);
            white-space:nowrap; overflow:hidden; text-overflow:ellipsis; }
  .dchip { display:inline-block; padding:0 4px; border-radius:3px;
           border:1px solid; font-size:.56rem; letter-spacing:.06em; }
  .stdb { padding:0 3px; border-radius:3px; border:1px solid var(--dim);
          color:var(--txt); font-size:.54rem; letter-spacing:.08em; }
  .ltb { padding:0 4px; border-radius:3px; border:1px solid var(--dim);
         color:var(--dim); font-size:.56rem; letter-spacing:.05em; }
  .ltb.scanner { color:var(--grn); border-color:var(--grn); }
  .ltb.manual { color:#31c8ff; border-color:#31c8ff; }
  .ltb.event { color:var(--amb); border-color:var(--amb); }
  .spark { display:inline-block; position:relative; width:60px; height:14px;
           background:#050805; border:1px solid rgba(51,80,47,.5);
           border-radius:2px; vertical-align:middle; }
  .sp-range { position:absolute; top:3px; bottom:3px; border-radius:1px;
              background:linear-gradient(90deg,#1a5f2a,#39ff6a); }
  .sp-mean { position:absolute; top:1px; bottom:1px; width:2px;
             margin-left:-1px; background:#e8ffe8; }
  .detcard.vs { border-left:2px solid var(--grn);
                background:rgba(57,255,106,.05); }
  .detcard.ev { border-left:2px solid var(--amb); }
  .dcspark { margin:2px 0 3px; }
  /* ============ mobile / narrow layout (<=899px) ============
     The 3-column card row un-nests (.col -> display:contents) into a
     2-column grid: video spans the left (tap toggles a fixed full-viewport
     zoom), meters + 48ch spectrum stack right, stats full-width, channel
     card last with its own internal scroll. Bottom graphs stack vertical.
     Tabs become full-width 40px touch targets; the LIVE-tab chip carries
     the detection count so the h1 counter goes away. */
  @media (max-width: 899px) {
    body { padding:6px 6px 0; gap:6px; }
    .tabs { gap:6px; }
    .tab { flex:1 1 0; justify-content:center; min-height:40px;
           font-size:.66rem; padding:6px 8px; }
    #detchip { font-size:.56rem; padding:1px 5px; }
    h1 { font-size:.72rem; letter-spacing:.1em; }
    #detcount { display:none; }
    #lock { font-size:.6rem; padding:1px 6px; }
    .top { gap:6px; }
    .cols { display:grid; grid-template-columns:1fr 1fr; gap:6px;
            grid-template-rows:auto auto auto minmax(0,1fr); }
    .col { display:contents; }
    .vidcard { grid-column:1; grid-row:1 / span 2; min-height:0; }
    .metercard { grid-column:2; grid-row:1; }
    .specwrap { grid-column:2; grid-row:2; }
    .statscard { grid-column:1 / -1; grid-row:3; }
    .grow { grid-column:1 / -1; grid-row:4; min-height:0; }
    .panel { padding:6px 8px; }
    .lbl { font-size:.6rem; }
    #spec { min-height:54px; }
    .stats { grid-template-columns:repeat(auto-fit,minmax(58px,1fr));
             font-size:.58rem; }
    .ch { min-height:32px; padding:1px 0; font-size:0; }
    .ch b { font-size:.58rem; }
    .bandrow { margin:1px 0 0; font-size:.58rem; }
    .cardbar { grid-template-columns:repeat(3,1fr); gap:4px; margin-top:6px; }
    .cardbar .tg { height:36px; font-size:.64rem; letter-spacing:.02em; gap:4px; }
    .screen { cursor:pointer; }
    .vidcard.zoom { position:fixed; inset:0; z-index:20; border-radius:0; }
    .botsplit { flex:0 0 auto; flex-direction:column; height:34dvh;
                min-height:0; margin-top:6px; }
    #gwrap, #swrap { flex:1 1 0; min-height:0; }
    #swrap { border-left:none; }
    #gtitle, #fstitle { font-size:.58rem; }
    #gcur b { font-size:1.1rem; }
    .detbar { gap:8px; padding:6px 8px; }
    .detbar .lbl { overflow:hidden; text-overflow:ellipsis; white-space:nowrap; }
    .detbar .tg { height:36px; font-size:.6rem; padding:2px 12px; }
    #dettable { display:none; }
    #detcards { display:block; padding:6px; }
  }
  /* Landscape phone: wide but ~390px tall. 4-column grid -- video, meters
     over stats, 48ch spectrum, channel card full height; graphs back to
     side-by-side (plenty of width). */
  @media (max-width: 899px) and (orientation: landscape) {
    .cols { grid-template-columns:1.2fr 1fr 1fr 1.1fr;
            grid-template-rows:minmax(0,1fr) auto; }
    .vidcard { grid-column:1; grid-row:1 / 3; }
    .metercard { grid-column:2; grid-row:1; }
    .statscard { grid-column:2; grid-row:2; }
    .specwrap { grid-column:3; grid-row:1 / 3; }
    .grow { grid-column:4; grid-row:1 / 3; }
    .grid48 { grid-template-columns:repeat(4,1fr); }  /* cells stay >=32px */
    .cardbar .tg .sw { display:none; }   /* .on/.pend border+color carry state */
    .botsplit { flex-direction:row; height:32dvh; }
    #swrap { border-left:1px solid var(--dim); }
  }
  @media (max-width: 899px) and (orientation: portrait) and (max-height: 700px) {
    .botsplit { height:28dvh; }        /* small phones: room for the grid */
  }
</style>
</head>
<body>
<div class="tabs">
  <button id="tab-live" class="tab on" onclick="showTab('live')">LIVE <span id="detchip">detections: --</span></button>
  <button id="tab-dets" class="tab" onclick="showTab('dets')">DETECTIONS</button>
</div>
<div class="top" id="view-live">
<h1>OUI-SPY // C5VRX <span id="detcount" title="detections logged to tools/detections.csv">-- DET</span><span id="lock">OFFLINE</span></h1>
<div class="cols">
<div class="col left">
<div class="panel metercard">
  <div class="lbl">CARRIER LEVEL <span id="lvlval">--</span> dB</div>
  <canvas id="meter" height="28"></canvas>
  <div class="lbl">Q PHASE COHERENCE <span id="qval">--</span>%</div>
  <div class="bar"><div id="qbar"></div></div>
  <div class="lbl">RX GAIN <span id="gval">--</span> / 62 (higher = farther)</div>
  <div class="bar"><div id="gbar" style="background:var(--amb)"></div></div>
  <div class="lbl">HOP <span id="hop">--</span> <span id="hopchip" class="hopchip"></span></div>
</div>
<div class="panel specwrap">
  <div class="lbl">48CH PWR</div>
  <canvas id="spec"></canvas>
</div>
</div>
<div class="col mid">
<div class="panel vidcard">
  <div class="lbl">VIDEO PREVIEW <span id="vmode" class="vmode">--</span><button id="shot" title="save the current frame to tools/shots/">CAM</button></div>
  <div class="vwrap"><img class="screen" src="/video_feed" alt="video"></div>
</div>
<div class="panel statscard">
  <div class="stats">
    <span><span class="k">CH </span><b id="chan">-</b></span>
    <span><span class="k">FREQ </span><b id="freq">-</b></span>
    <span><span class="k">BAND </span><b id="band">-</b></span>
    <span><span class="k">AGC </span><b id="agc">-</b></span>
    <span><span class="k">BW </span><b id="bw">-</b></span>
    <span><span class="k">CFO </span><b id="cfo">-</b></span>
    <span><span class="k">P </span><b id="p">-</b></span>
    <span><span class="k">CLIP </span><b id="clip">-</b></span>
    <span><span class="k">FPS </span><b id="fps">-</b></span>
  </div>
</div>
</div>
<div class="col right">
<div class="panel grow">
  <div class="lbl">DIRECT TUNE</div>
  <div id="grid"></div>
  <div class="cardbar">
    <button id="tg-scan" class="tg" onclick="key('g')"><span class="sw"></span>SCAN</button>
    <button id="tg-mute" class="tg" onclick="key('u')"><span class="sw"></span>MUTE</button>
    <button id="tg-skip" class="tg" onclick="key('k')"><span class="sw"></span>SKIP DEAD</button>
  </div>
  <div id="toast"></div>
</div>
</div>
</div>
<div class="botsplit">
<div id="gwrap">
<canvas id="hist"></canvas>
<div id="gtitle">LEVEL HISTORY // 60S</div>
<div id="gcur"><b id="glvl">--</b> dB</div>
</div>
<div id="swrap">
<canvas id="wfall"></canvas>
<canvas id="fspec"></canvas>
<div id="fstitle">CHAN SPECTRUM</div>
<label id="holdw" title="peak-hold trace on the power plot"><input type="checkbox" id="hold" checked>HOLD</label>
</div>
</div>
</div>
<div id="view-dets">
  <div class="detbar">
    <span class="lbl">DETECTION LOG // tools/detections.csv</span>
    <span id="dettotal">0 ROWS</span>
    <span id="detsum"></span>
    <a id="export" class="tg" href="/api/detections/export" download="detections.csv">EXPORT CSV</a>
    <button id="detclear" class="tg" title="truncate tools/detections.csv to header-only">CLEAR</button>
  </div>
  <div class="detwrap">
    <table id="dettable"><thead id="dethead"></thead><tbody id="detbody"></tbody></table>
    <div id="detcards"></div>
  </div>
</div>
<div id="lightbox"><img id="lbimg" alt="captured frame"></div>
<script>
const CH = __CHANNEL_GRID__;
/* UI-state persistence (tab, HOLD, video zoom). Firmware toggles stay
   firmware-synced; only local view prefs live here. */
const lsGet = k => { try { return localStorage.getItem(k); } catch (e) { return null; } };
const lsSet = (k, v) => { try { localStorage.setItem(k, v); } catch (e) {} };
const grid = document.getElementById('grid');
CH.forEach((row, bi) => {
  const lbl = document.createElement('div');
  lbl.className = 'bandrow'; lbl.textContent = 'BAND ' + row[0].n[0];
  grid.appendChild(lbl);
  const r = document.createElement('div'); r.className = 'grid48';
  row.forEach((c, ci) => {
    const b = document.createElement('button');
    b.className = 'ch'; b.dataset.name = c.n;
    b.innerHTML = '<b>' + c.n + '</b>' + c.f;
    b.onclick = () => tune(bi * 8 + ci, c.n);
    r.appendChild(b);
  });
  grid.appendChild(r);
});

/* Mobile: tap the video to zoom it to a fixed full-viewport overlay
   (works where requestFullscreen on a div doesn't, e.g. iPhone Safari);
   tap again to close. Desktop keeps the click-through behaviour. */
const vidcard = document.querySelector('.vidcard');
document.querySelector('.screen').addEventListener('click', () => {
  if (window.innerWidth <= 899) {
    vidcard.classList.toggle('zoom');
    lsSet('c5_zoom', vidcard.classList.contains('zoom') ? '1' : '0');
  }
});
if (lsGet('c5_zoom') === '1' && window.innerWidth <= 899)
  vidcard.classList.add('zoom');

let S = null;               // last /api/state snapshot
let hist = [];              // {t:ms, lvl:dB, lock:bool}
const HIST_MS = 60000;

/* Auto-range target helper: min/max of the given values with 15% padding
 * (floor 2 dB), minimum span so a dead-flat signal still gets a usable
 * plot. Returns [lo, hi] in dB; defaults when no usable values. */
function rangeTarget(vals, dfltLo, dfltHi, minSpan) {
  let lo = Infinity, hi = -Infinity;
  vals.forEach(v => {
    if (typeof v !== 'number' || !isFinite(v)) return;
    if (v < lo) lo = v;
    if (v > hi) hi = v;
  });
  if (!isFinite(lo)) return [dfltLo, dfltHi];
  const pad = Math.max(2, (hi - lo) * 0.15);
  lo -= pad; hi += pad;
  if (hi - lo < minSpan) { const m = (hi + lo) / 2; lo = m - minSpan / 2; hi = m + minSpan / 2; }
  return [lo, hi];
}
function yTarget(now) {
  const vals = [];
  hist.forEach(p => { if (now - p.t < HIST_MS) vals.push(p.lvl); });
  return rangeTarget(vals, 0, 45, 6);
}
let yLo = 0, yHi = 0;         // smoothed range; yHi<=yLo = snap on first data
let fsLo = 0, fsHi = 0;       // same for the sweep-spectrum panel
let fPeaks = {};              // channel name -> {v:dB, t:ms} peak-hold
let holdOn = lsGet('c5_hold') !== '0';   // peak-hold trace toggle (persisted)
{
  const cb = document.getElementById('hold');
  cb.checked = holdOn;
  cb.addEventListener('change', () => {
    holdOn = cb.checked;
    lsSet('c5_hold', holdOn ? '1' : '0');
  });
}
document.getElementById('shot').addEventListener('click', async () => {
  try {
    const r = await (await fetch('/api/shot', {method: 'POST'})).json();
    toast(r.ok ? 'SHOT -> ' + r.name : 'shot failed');
  } catch (e) { toast('shot failed'); }
});

/* Waterfall (gqrx-style): poll() drops one new spectrum row into wfNew;
   draw() shifts the offscreen up and paints it at the bottom. Colors come
   from a heat LUT (dark blue -> cyan -> green -> yellow -> red) mapped
   through the same smoothed dB range as the power trace below it. */
const WFLUT = (() => {
  const stops = [[0.00,   5,  10,  42], [0.30,   0, 150, 160],
                 [0.50,  57, 255, 106], [0.75, 255, 225,  74],
                 [1.00, 255,  74,  61]];
  const lut = [];
  for (let i = 0; i < 256; i++) {
    const t = i / 255;
    let a = stops[0], b = stops[stops.length - 1];
    for (let s = 0; s < stops.length - 1; s++)
      if (t <= stops[s + 1][0]) { a = stops[s]; b = stops[s + 1]; break; }
    const f = (t - a[0]) / Math.max(1e-6, b[0] - a[0]);
    lut.push('rgb(' + Math.round(a[1] + (b[1] - a[1]) * f) + ',' +
                      Math.round(a[2] + (b[2] - a[2]) * f) + ',' +
                      Math.round(a[3] + (b[3] - a[3]) * f) + ')');
  }
  return lut;
})();
const wfOff = document.createElement('canvas');
const wfc = wfOff.getContext('2d');
let wfNew = null;               // pending waterfall row from poll()

async function key(k) {
  try {
    const r = await fetch('/api/key/' + encodeURIComponent(k), {method:'POST'});
    const j = await r.json();
    toast(j.ok ? 'sent [' + k + ']' : 'busy / rejected');
  } catch (e) { toast('request failed'); }
  setTimeout(poll, 300);
}
async function tune(idx, name) {
  try {
    const r = await fetch('/api/tune/' + idx, {method:'POST'});
    const j = await r.json();
    toast(j.ok ? 'TUNE -> ' + name : (r.status === 429 ? 'rate limited' : 'tune failed'));
  } catch (e) { toast('request failed'); }
  setTimeout(poll, 400);
}
function toast(t) { document.getElementById('toast').textContent = t; }

/* Tabs: LIVE dashboard vs the DETECTIONS log. No reload; the detections
   table polls /api/detections on the same 500 ms cadence, but only while
   its tab is active. Active tab persists in localStorage. */
let tabLive = true;
function showTab(t) {
  tabLive = (t !== 'dets');
  lsSet('c5_tab', tabLive ? 'live' : 'dets');
  document.getElementById('view-live').style.display = tabLive ? '' : 'none';
  document.getElementById('view-dets').style.display = tabLive ? 'none' : 'flex';
  document.getElementById('tab-live').classList.toggle('on', tabLive);
  document.getElementById('tab-dets').classList.toggle('on', !tabLive);
  if (!tabLive) pollDets();
}
const DETCOLS = [
  ['start_iso', 'START'], ['end_iso', 'END'], ['duration_s', 'DUR s'],
  ['channel', 'CH'], ['drone', 'DRONE'], ['band', 'BAND'], ['freq_mhz', 'FREQ'],
  ['cfo_ppm', 'CFO'], ['video_std', 'STD'], ['line_us', 'LINE'],
  ['level_peak_db', 'PEAK'], ['level_mean_db', 'MEAN'], ['level_min_db', 'MIN'],
  ['video_sync', 'SYNC'], ['frames_received', 'FRAMES'], ['max_fps', 'MAXFPS'],
  ['lock_type', 'LOCK'], ['end_reason', 'END REASON'],
];
const esc = s => String(s).replace(/&/g, '&amp;').replace(/</g, '&lt;')
                          .replace(/>/g, '&gt;');
const fmtDur = v => {
  const s = Math.round(Number(v) || 0);
  return s < 60 ? s + 's' : Math.floor(s / 60) + 'm ' + (s % 60) + 's';
};
const fmtISO = v => (typeof v === 'string' && v.length >= 19 && v.charAt(10) === 'T')
  ? '<span class="dt-d">' + esc(v.slice(0, 10)) + '</span> <span class="dt-t">' +
    esc(v.slice(11, 19)) + '</span>' : esc(v);
const bchip = (band, name) =>
  '<span class="bchip b' + esc(band || '?') + '">' + esc(name) + '</span>';
/* Mini level strip: min..peak range bar with a mean tick, on the graph's
   fixed 0..45 dB scale. '' when the episode has no level samples. */
const sparkSpan = r => {
  const lo = r.level_min_db, hi = r.level_peak_db, mean = r.level_mean_db;
  if (typeof lo !== 'number' || typeof hi !== 'number') return '';
  const X = v => Math.max(0, Math.min(100, v / 45 * 100));
  const left = X(lo), width = Math.max(2, X(hi) - left);
  return '<span class="spark">' +
    '<span class="sp-range" style="left:' + left.toFixed(1) +
      '%;width:' + width.toFixed(1) + '%"></span>' +
    (typeof mean === 'number'
      ? '<span class="sp-mean" style="left:' + X(mean).toFixed(1) + '%"></span>'
      : '') + '</span>';
};
/* Row accent class: video-synced episodes green, standalone events amber. */
const detRowCls = r => r.video_sync === 1 ? 'vs'
                     : r.lock_type === 'event' ? 'ev' : '';
/* DRONE-ID clustering: fingerprint = cfo_ppm rounded to the nearest
   0.5 ppm + video_std ('' when the firmware left it blank). Rows with a
   blank/non-numeric cfo_ppm carry no fingerprint and get no ID. */
const droneFp = r => {
  if (r.cfo_ppm === '' || r.cfo_ppm === undefined || r.cfo_ppm === null)
    return null;
  const p = Number(r.cfo_ppm);
  if (!isFinite(p)) return null;
  return (Math.round(p * 2) / 2).toFixed(1) + '|' + (r.video_std || '');
};
/* Deterministic fp -> DRONE-XXX map: clusters are numbered by first-seen
   (earliest start_iso), so the same row set always yields the same IDs. */
function droneMap(rows) {
  const first = {};
  rows.forEach(r => {
    const fp = droneFp(r);
    if (!fp) return;
    const t = String(r.start_iso || '');
    if (!(fp in first) || t < first[fp]) first[fp] = t;
  });
  const fps = Object.keys(first).sort((a, b) =>
    first[a] < first[b] ? -1 : first[a] > first[b] ? 1 : a < b ? -1 : 1);
  const m = {};
  fps.forEach((fp, i) => { m[fp] = 'DRONE-' + String(i + 1).padStart(3, '0'); });
  return m;
}
/* Stable per-ID hue so each drone keeps its own chip color. */
const droneHue = id => {
  let h = 0;
  for (let i = 0; i < id.length; i++) h = (h * 31 + id.charCodeAt(i)) >>> 0;
  return h % 360;
};
const droneChip = id =>
  '<span class="dchip" style="color:hsl(' + droneHue(id) + ',80%,62%);' +
  'border-color:hsl(' + droneHue(id) + ',80%,40%)">' + esc(id) + '</span>';
const fmtCfo = v => (v > 0 ? '+' : '') + Number(v).toFixed(1);
const fmtLine = v => (typeof v === 'number' ? v.toFixed(2) : esc(v)) + 'us';
/* Header summary strip: "N drones fingerprinted · M encounters ·
   strongest: DRONE-X (peak dB)" from the loaded rows. */
function renderDetSum(rows, dm) {
  const ids = Object.keys(dm).length;
  const enc = rows.reduce((n, r) => n + (r.drone ? 1 : 0), 0);
  let best = null;
  rows.forEach(r => {
    if (r.drone && typeof r.level_peak_db === 'number' &&
        (!best || r.level_peak_db > best.peak)) best = { id: r.drone, peak: r.level_peak_db };
  });
  document.getElementById('detsum').textContent =
    ids + ' drone' + (ids === 1 ? '' : 's') + ' fingerprinted · ' +
    enc + ' encounter' + (enc === 1 ? '' : 's') +
    (best ? ' · strongest: ' + best.id + ' (' + best.peak + 'dB)' : '');
}
/* Match a detection row to its screenshot: capture names are
   <sanitized episode start>__<HHMMSS>.jpg, sanitized the same way
   (YYYY-MM-DDTHH:MM:SS -> dashes). */
const shotFor = (r, shots) => {
  if (typeof r.start_iso !== 'string' || r.start_iso.length < 19) return null;
  const san = r.start_iso.slice(0, 19).replace(/:/g, '-');
  return shots.find(n => n.startsWith(san + '__')) || null;
};
function showShot(src) {
  document.getElementById('lbimg').src = src;
  document.getElementById('lightbox').style.display = 'flex';
}
let detSig = '';                       // column signature of the built header
function renderDets(d) {
  const rows = d.rows || [];
  const shots = d.shots || [];
  document.getElementById('dettotal').textContent = (d.total || 0) + ' ROWS';
  // Cluster fingerprinted rows and inject the display ID, then summarize.
  const dm = droneMap(rows);
  rows.forEach(r => {
    const fp = droneFp(r);
    if (fp) r.drone = dm[fp];
  });
  renderDetSum(rows, dm);
  // Skip columns the CSV lacks entirely (or that are blank on every row).
  const cols = DETCOLS.filter(c => rows.some(r => r[c[0]] !== undefined && r[c[0]] !== ''));
  if (rows.some(r => typeof r.level_peak_db === 'number')) {
    const li = cols.findIndex(c => c[0] === 'level_min_db');
    cols.splice(li >= 0 ? li + 1 : cols.length, 0, ['spark', 'LEVEL']);
  }
  if (rows.some(r => shotFor(r, shots))) cols.push(['shot', 'SHOT']);
  if (window.innerWidth <= 899) { renderDetCards(rows, cols, shots); return; }
  const sig = cols.map(c => c[0]).join(',');
  if (sig !== detSig) {
    detSig = sig;
    document.getElementById('dethead').innerHTML =
      '<tr>' + cols.map(c => '<th' + (c[0] === 'spark' ? ' class="sps"' : '') + '>' +
                             c[1] + '</th>').join('') + '</tr>';
  }
  document.getElementById('detbody').innerHTML = rows.map(r => {
    const rcls = detRowCls(r);
    return '<tr' + (rcls ? ' class="' + rcls + '"' : '') + '>' + cols.map(c => {
      const k = c[0], v = r[k];
      if (k === 'shot') {
        const nm = shotFor(r, shots);
        return nm ? '<td><img class="shotth" src="/shots/' + nm + '"></td>'
                  : '<td class="na">--</td>';
      }
      if (k === 'spark') { const sp = sparkSpan(r); return sp ? '<td>' + sp + '</td>'
                                                              : '<td class="na">--</td>'; }
      if (k === 'video_sync')
        return v === 1 ? '<td><span class="vsb y">YES</span></td>'
                       : '<td class="na">--</td>';
      if (v === undefined || v === '') return '<td class="na">--</td>';
      if (k === 'channel') return '<td>' + bchip(r.band, v) + '</td>';
      if (k === 'drone') return '<td>' + droneChip(v) + '</td>';
      if (k === 'cfo_ppm') return '<td class="num">' + esc(fmtCfo(v)) + '</td>';
      if (k === 'video_std') return '<td><span class="stdb">' + esc(v) + '</span></td>';
      if (k === 'line_us') return '<td class="num">' + fmtLine(v) + '</td>';
      if (k === 'start_iso' || k === 'end_iso')
        return '<td class="ts">' + fmtISO(v) + '</td>';
      if (k === 'duration_s') return '<td class="num">' + esc(fmtDur(v)) + '</td>';
      if (k === 'freq_mhz')
        return '<td class="num">' + (typeof v === 'number' ? v.toFixed(1) : esc(v)) + '</td>';
      if (k === 'level_peak_db' || k === 'level_mean_db' || k === 'level_min_db')
        return '<td class="num">' + esc(v) + '</td>';
      if (k === 'lock_type')
        return '<td><span class="ltb ' + esc(v) + '">' + esc(v) + '</span></td>';
      return '<td>' + esc(v) + '</td>';
    }).join('') + '</tr>';
  }).join('');
}
/* Mobile: one compact card per row instead of the wide table, same visual
   language (accent bar, band chip, spark strip). Title line is channel
   chip + timestamp + duration + shot + sync; the rest stacks as
   label:value pairs in a 2-column grid. */
function renderDetCards(rows, cols, shots) {
  const TITLE = ['channel', 'drone', 'start_iso', 'duration_s', 'video_sync', 'shot'];
  document.getElementById('detcards').innerHTML = rows.map(r => {
    let h = '<div class="dcline">';
    if (r.channel !== undefined && r.channel !== '') h += bchip(r.band, r.channel);
    if (r.drone) h += droneChip(r.drone);
    if (r.start_iso) h += '<span class="dcts">' + fmtISO(r.start_iso) + '</span>';
    if (r.duration_s !== undefined && r.duration_s !== '')
      h += '<span class="dcdur">' + esc(fmtDur(r.duration_s)) + '</span>';
    const nm = shotFor(r, shots);
    if (nm) h += '<img class="shotth" src="/shots/' + nm + '">';
    if (r.video_sync === 1) h += '<span class="vsb y">YES</span>';
    h += '</div>';
    const sp = sparkSpan(r);
    if (sp) h += '<div class="dcspark">' + sp + '</div>';
    h += '<div class="dcgrid">';
    cols.forEach(c => {
      if (TITLE.indexOf(c[0]) >= 0 || c[0] === 'spark') return;
      const v = r[c[0]];
      if (v === undefined || v === '') return;
      let val;
      if (c[0] === 'end_iso') val = fmtISO(v);
      else if (c[0] === 'freq_mhz' && typeof v === 'number') val = v.toFixed(1);
      else if (c[0] === 'cfo_ppm') val = esc(fmtCfo(v));
      else if (c[0] === 'video_std') val = '<span class="stdb">' + esc(v) + '</span>';
      else if (c[0] === 'line_us') val = fmtLine(v);
      else if (c[0] === 'lock_type') val = '<span class="ltb ' + esc(v) + '">' + esc(v) + '</span>';
      else val = esc(v);
      h += '<span><i>' + c[1] + '</i>' + val + '</span>';
    });
    return '<div class="detcard ' + detRowCls(r) + '">' + h + '</div></div>';
  }).join('');
}
document.querySelector('.detwrap').addEventListener('click', e => {
  if (e.target.classList && e.target.classList.contains('shotth'))
    showShot(e.target.getAttribute('src'));
});
document.getElementById('lightbox').addEventListener('click', () => {
  document.getElementById('lightbox').style.display = 'none';
});
document.getElementById('detclear').addEventListener('click', async () => {
  if (!confirm('Delete ALL logged detections (tools/detections.csv)?')) return;
  try {
    await fetch('/api/detections/clear', {method: 'POST'});
    detSig = '';                       // force header rebuild
    pollDets();
  } catch (e) { /* next poll retries the render anyway */ }
});
async function pollDets() {
  try { renderDets(await (await fetch('/api/detections')).json()); }
  catch (e) { /* keep last good table */ }
}

async function poll() {
  let s;
  try { s = await (await fetch('/api/state')).json(); }
  catch (e) { return; }
  S = s;
  const t = s.telemetry, set = (id, v) => document.getElementById(id).textContent = v;
  const num = v => (typeof v === 'number' && isFinite(v)) ? v : 0;
  const lk = document.getElementById('lock');
  if (!s.connected)      { lk.textContent = 'OFFLINE';    lk.className = ''; }
  else if (s.locked)     { lk.textContent = 'LOCKED';     lk.className = 'on'; }
  else if (s.scan)       { lk.textContent = 'SCANNING';   lk.className = 'scan'; }
  else                   { lk.textContent = 'SEARCHING';  lk.className = ''; }
  set('chan', t.channel);
  set('freq', num(t.freq_mhz) ? num(t.freq_mhz).toFixed(1) + ' MHz' : '-');
  set('band', t.band); set('agc', t.agc_mode + '/' + t.agc_state); set('bw', t.bw);
  set('cfo', (num(t.cfo_khz) >= 0 ? '+' : '') + num(t.cfo_khz) + ' kHz');
  set('p', num(t.p_median)); set('clip', num(t.clip)); set('fps', num(s.fps).toFixed(1));
  set('lvlval', num(t.level_db).toFixed(1));
  set('glvl', num(t.level_db).toFixed(1));
  document.getElementById('gcur').className = s.locked ? 'lock' : (s.scan ? 'scan' : '');
  set('qval', num(t.q_phase)); set('gval', num(t.gain));
  document.getElementById('qbar').style.width = Math.min(100, num(t.q_phase)) + '%';
  document.getElementById('gbar').style.width = Math.min(100, num(t.gain) / 62 * 100) + '%';
  const tg = (id, actual, desired) => {
    // Knob always shows the firmware-confirmed state; amber pulse while the
    // desired target is unconfirmed (pending).
    const el = document.getElementById(id);
    el.classList.toggle('on', !!actual);
    el.classList.toggle('pend', desired !== null && desired !== undefined && desired !== actual);
  };
  tg('tg-scan', s.scan, s.scan_desired);
  tg('tg-mute', s.buzzer_enabled, s.buzzer_desired);
  tg('tg-skip', s.skip_dead, s.skip_desired);
  set('detcount', num(s.detections_total) + ' DET');
  set('detchip', 'detections: ' + num(s.detections_total));
  const vm = document.getElementById('vmode');
  // Badge only exists while video-locked (SYNC); otherwise it hides --
  // the lock state is obvious from the picture itself.
  vm.textContent = s.video_mode === 'sync' ? 'SYNC' : '';
  vm.className = 'vmode ' + (s.video_mode === 'sync' ? 'sync' : 'nosignal');
  vm.style.display = s.video_mode === 'sync' ? '' : 'none';
  // Freshest sweep entry (min age_s): the scanner's current dwell channel
  // -- same source the spectrum/waterfall dwell markers use.
  let fresh = null;
  (s.spectrum || []).forEach(e => {
    if (e && e.age_s !== null && e.age_s !== undefined &&
        (!fresh || e.age_s < fresh.age_s)) fresh = e;
  });
  // Hop line on the meters card: dwell channel while scanning (pulsing),
  // the tuned channel with its state word otherwise.
  const tf = num(t.freq_mhz) ? num(t.freq_mhz).toFixed(1) + ' MHz' : '--';
  const hopChip = document.getElementById('hopchip');
  if (s.locked) {
    set('hop', t.channel + ' · ' + tf);
    hopChip.textContent = 'LOCKED'; hopChip.className = 'hopchip lock';
  } else if (s.scan) {
    set('hop', fresh ? fresh.name + ' · ' + num(fresh.freq_mhz).toFixed(1) + ' MHz'
                     : '--');
    hopChip.textContent = 'DWELL'; hopChip.className = 'hopchip dwell';
  } else {
    set('hop', t.channel + ' · ' + tf);
    hopChip.textContent = 'PARKED'; hopChip.className = 'hopchip parked';
  }
  document.querySelectorAll('.ch').forEach(b => {
    const here = b.dataset.name === t.channel;
    b.classList.toggle('cur', !!(s.locked && here));
    b.classList.toggle('sweep', !!(s.scan && here));
    b.classList.toggle('parked', !!(!s.scan && !s.locked && here));
  });
  /* Level history: locked keeps the locked-channel level stream (byte-
     identical to before). While scanning, each poll's freshest dwell
     power becomes an instantaneous sample instead, so the graph jumps
     the moment a dwell sees energy; insta samples get amber dots. */
  let lvl = num(t.level_db), insta = false;
  if (s.scan && !s.locked && fresh &&
      typeof fresh.p === 'number' && isFinite(fresh.p) && fresh.age_s < 2) {
    lvl = fresh.p; insta = true;
  }
  hist.push({t: Date.now(), lvl: lvl, lock: s.locked, ch: t.channel, insta: insta});
  hist = hist.filter(h => Date.now() - h.t < HIST_MS);
  // Waterfall: one new row per poll (drawn by draw() on its own cadence).
  wfNew = (s.spectrum || []).filter(e => e && e.freq_mhz >= 5560 && e.freq_mhz <= 6000)
                            .sort((a, b) => a.freq_mhz - b.freq_mhz);
  if (!s.connected) toast('SERIAL DISCONNECTED');
}

/* Animated meters: smoothed needle with peak-hold, lock-marked history. */
let shown = 0, peak = 0, peakT = 0;
function draw() {
  try {
  const num = v => (typeof v === 'number' && isFinite(v)) ? v : 0;
  const target = num(S && S.telemetry.level_db);
  shown += (target - shown) * 0.15;
  if (shown > peak) { peak = shown; peakT = Date.now(); }
  else if (Date.now() - peakT > 2000) { peak = Math.max(shown, peak - 0.15); }

  const m = document.getElementById('meter'), mc = m.getContext('2d');
  const w = m.width = m.clientWidth * devicePixelRatio, h = m.height = 28 * devicePixelRatio;
  const sx = v => v / 45 * w;
  const zones = [[0, 15, '#ffb000'], [15, 30, '#39ff6a'], [30, 45, '#ff4a3d']];
  zones.forEach(([a, b, c]) => {
    mc.fillStyle = c + '22'; mc.fillRect(sx(a), 0, sx(b) - sx(a), h);
    mc.fillStyle = c; mc.fillRect(sx(a), h - 3 * devicePixelRatio, sx(b) - sx(a), 2 * devicePixelRatio);
  });
  for (let v = 0; v <= 45; v += 5) {
    mc.fillStyle = '#33502f'; mc.fillRect(sx(v), 0, 1, 5 * devicePixelRatio);
    mc.font = (8 * devicePixelRatio) + 'px monospace'; mc.fillText(v, sx(v) + 2, 8 * devicePixelRatio);
  }
  const zoneCol = zones.find(z => shown < z[1]) || zones[2];
  mc.fillStyle = zoneCol[2];
  mc.fillRect(0, h * .3, sx(Math.max(0, shown)), h * .55);
  mc.fillStyle = '#fff';
  mc.fillRect(sx(peak) - 1, h * .2, 2 * devicePixelRatio, h * .75);

  /* 48CH PWR: one bar per channel from [SWEEP] hop data (same sweep P
     values as the big CHAN SPECTRUM panel). Bar height = P, auto-scaled;
     band color-coded; current scanner position an amber glow, locked
     channel a bright outline, strongest channel a brighter green bar with
     its name lit. Every channel name is printed vertically under its bar
     -- one row at desktop width, two alternating rows when cramped, so no
     bar is ever unlabeled. No data yet draws as a bare baseline. */
  const BANDC = {R:'#ff4a3d', A:'#ffb000', B:'#39ff6a',
                 E:'#31c8ff', F:'#c96bff', L:'#ffe14a'};
  const sp = document.getElementById('spec'), spc = sp.getContext('2d');
  const spw = sp.width = sp.clientWidth * devicePixelRatio,
        sph = sp.height = sp.clientHeight * devicePixelRatio;
  const spec = (S && S.spectrum) || [];
  const cur = S && S.telemetry.channel, isLock = !!(S && S.locked);
  let pmax = 8, best = -1, bestP = -1;
  spec.forEach((e, i) => {
    if (e && typeof e.p === 'number' && isFinite(e.p)) {
      if (e.p > pmax) pmax = e.p;
      if (e.p > bestP) { bestP = e.p; best = i; }
    }
  });
  const scpt = sp.clientWidth < 300;                 // cramped: 2-row labels
  const labH = (scpt ? 27 : 14) * devicePixelRatio;
  const n = Math.max(spec.length, 1), bw2 = spw / n, base = 2 * devicePixelRatio;
  const plotB = sph - labH;                          // bars end above the labels
  spec.forEach((e, i) => {
    const x = i * bw2, bwid = Math.max(1, bw2 - devicePixelRatio * .5);
    /* Full-height faint track per channel: the panel reads as a filled
       instrument even when every bar is short (weak sweep powers). */
    spc.fillStyle = '#0d1a0d';
    spc.fillRect(x, 2 * devicePixelRatio, bwid, plotB - base - 2 * devicePixelRatio);
    spc.fillStyle = '#1a2f1a';
    spc.fillRect(x, plotB - base, bwid, devicePixelRatio);
    if (!e || typeof e.p !== 'number' || !isFinite(e.p)) return;
    const bh = Math.min(1, e.p / pmax) * (plotB - 8 * devicePixelRatio);
    const c = BANDC[e.band] || '#39ff6a';
    const grad = spc.createLinearGradient(0, plotB - base - bh, 0, plotB - base);
    if (i === best) {                       // strongest: brighter green fill
      grad.addColorStop(0, '#d8ffe0'); grad.addColorStop(1, c + '55');
    } else {
      grad.addColorStop(0, c + 'ee'); grad.addColorStop(1, c + '22');
    }
    spc.fillStyle = grad;
    if (e.name === cur) { spc.shadowColor = '#ffb000'; spc.shadowBlur = 8 * devicePixelRatio; }
    else if (i === best) { spc.shadowColor = '#39ff6a'; spc.shadowBlur = 8 * devicePixelRatio; }
    spc.fillRect(x, plotB - base - bh, bwid, bh);
    spc.shadowBlur = 0;
    if (isLock && e.name === cur) {
      spc.strokeStyle = '#39ff6a'; spc.lineWidth = devicePixelRatio;
      spc.strokeRect(x, plotB - base - bh, bwid, bh);
    }
  });
  // Channel names rotated -90 degrees under their bars; cramped panels
  // stagger odd/even channels onto two rows so all 48 stay legible.
  spc.font = (9 * devicePixelRatio) + 'px monospace';
  spc.textAlign = 'left'; spc.textBaseline = 'middle';
  spec.forEach((e, i) => {
    if (!e) return;
    const cx = i * bw2 + bw2 / 2;
    const ly = plotB + (scpt ? (i % 2) + 1 : 1) * 13 * devicePixelRatio;
    spc.save();
    spc.translate(cx, ly);
    spc.rotate(-Math.PI / 2);
    if (i === best) {
      spc.fillStyle = '#e8ffe8'; spc.shadowColor = '#39ff6a';
      spc.shadowBlur = 5 * devicePixelRatio;
    } else {
      spc.fillStyle = '#4a6a4f';
    }
    spc.fillText(e.name, 0, 0);
    spc.restore();
  });
  spc.shadowBlur = 0;
  spc.fillStyle = '#33502f';
  spc.textAlign = 'right'; spc.textBaseline = 'top';
  spc.fillText('P/' + pmax, spw - 2 * devicePixelRatio, 2 * devicePixelRatio);
  spc.textAlign = 'left'; spc.textBaseline = 'alphabetic';

  /* Big history graph: dB grid, gradient area, glow trace, peak-hold,
     lock/scan hue, baseline lock ticks, scan sweep at the leading edge. */
  const he = document.getElementById('hist'), hc = he.getContext('2d');
  const hw = he.width = he.clientWidth * devicePixelRatio, hh = he.height = he.clientHeight * devicePixelRatio;
  const hcpt = he.clientWidth < 520 || he.clientHeight < 200;   // cramped: slim axes
  const padL = (hcpt ? 24 : 30) * devicePixelRatio, padR = (hcpt ? 8 : 10) * devicePixelRatio;
  const padT = (hcpt ? 24 : 34) * devicePixelRatio, padB = (hcpt ? 15 : 18) * devicePixelRatio;
  const gw = hw - padL - padR, gh = hh - padT - padB;
  const now = Date.now();
  const X = t => padL + (1 - (now - t) / HIST_MS) * gw;
  /* Auto-ranging Y: glide toward yTarget so the trace always fills most of
   * the plot height -- a strong signal never flatlines the top and a weak
   * one never hugs the bottom. */
  const tgt = yTarget(now);
  if (yHi <= yLo) { yLo = tgt[0]; yHi = tgt[1]; }
  else { yLo += (tgt[0] - yLo) * 0.12; yHi += (tgt[1] - yHi) * 0.12; }
  const Y = v => padT + gh - (Math.max(yLo, Math.min(yHi, v)) - yLo) / (yHi - yLo) * gh;
  const locked = !!(S && S.locked), scanning = !!(S && S.scan);
  const col = locked ? '#39ff6a' : scanning ? '#ffb000' : '#2ba84f';

  const gzones = [[0, 15, '#ffb000'], [15, 30, '#39ff6a'], [30, 45, '#ff4a3d']];
  gzones.forEach(([a, b, c]) => {
    hc.fillStyle = c + '0d'; hc.fillRect(padL, Y(b), gw, Y(a) - Y(b));
  });

  hc.font = (9 * devicePixelRatio) + 'px monospace';
  hc.textAlign = 'left'; hc.textBaseline = 'middle';
  const span = yHi - yLo, step = span > 30 ? 10 : span > 12 ? 5 : 2;
  for (let v = Math.ceil(yLo / step) * step; v <= yHi + 1e-9; v += step) {
    const y = Y(v), major = Math.round(v / step) % 2 === 0;   // dB gridlines
    hc.strokeStyle = major ? '#33502f' : '#1a2f1a'; hc.lineWidth = 1;
    hc.beginPath(); hc.moveTo(padL, y); hc.lineTo(hw - padR, y); hc.stroke();
    if (major) { hc.fillStyle = '#33502f'; hc.fillText(Math.round(v), 5 * devicePixelRatio, y); }
  }
  hc.fillStyle = '#33502f'; hc.fillText('dB', 5 * devicePixelRatio, padT - 12 * devicePixelRatio);

  hc.textAlign = 'center';
  for (let s = 0; s <= 60; s += hcpt ? 20 : 10) {  // time ticks (fewer when cramped)
    const x = X(now - s * 1000);
    hc.strokeStyle = '#1a2f1a';
    hc.beginPath(); hc.moveTo(x, padT); hc.lineTo(x, padT + gh); hc.stroke();
    hc.fillStyle = '#33502f';
    hc.fillText(s ? '-' + s + 's' : 'now', x, padT + gh + 9 * devicePixelRatio);
  }
  hc.textAlign = 'left';

  hist.forEach(p => {                                // lock ticks along the baseline
    hc.fillStyle = p.lock ? '#39ff6a' : '#3a2a20';
    hc.fillRect(X(p.t), padT + gh - 2 * devicePixelRatio, 2, 2 * devicePixelRatio);
  });

  const pts = hist.filter(p => now - p.t < HIST_MS).map(p => [X(p.t), Y(num(p.lvl))]);
  const trace = () => {                              // smoothed path through samples
    hc.beginPath();
    hc.moveTo(pts[0][0], pts[0][1]);
    for (let i = 1; i < pts.length - 1; i++) {
      const xc = (pts[i][0] + pts[i + 1][0]) / 2, yc = (pts[i][1] + pts[i + 1][1]) / 2;
      hc.quadraticCurveTo(pts[i][0], pts[i][1], xc, yc);
    }
    hc.lineTo(pts[pts.length - 1][0], pts[pts.length - 1][1]);
  };
  if (pts.length > 1) {
    const grad = hc.createLinearGradient(0, padT, 0, padT + gh);
    grad.addColorStop(0, col + '55'); grad.addColorStop(1, col + '00');
    trace();
    hc.lineTo(pts[pts.length - 1][0], padT + gh);
    hc.lineTo(pts[0][0], padT + gh); hc.closePath();
    hc.fillStyle = grad; hc.fill();
    trace();
    hc.strokeStyle = col; hc.lineWidth = 1.8 * devicePixelRatio;
    hc.shadowColor = col; hc.shadowBlur = 10 * devicePixelRatio;
    hc.stroke(); hc.shadowBlur = 0;
    hc.fillStyle = '#fff'; hc.shadowColor = col; hc.shadowBlur = 8 * devicePixelRatio;
    hc.beginPath();
    hc.arc(pts[pts.length - 1][0], pts[pts.length - 1][1], 2.5 * devicePixelRatio, 0, 7);
    hc.fill(); hc.shadowBlur = 0;
  }

  hist.forEach(p => {                                // instantaneous sweep samples:
    if (!p.insta || now - p.t >= HIST_MS) return;    // amber dots vs the green
    hc.fillStyle = '#ffb000';                        // locked trace
    hc.beginPath(); hc.arc(X(p.t), Y(num(p.lvl)), 1.6 * devicePixelRatio, 0, 7);
    hc.fill();
  });

  let pk = 0;                                        // peak-hold over the window
  hist.forEach(p => { if (now - p.t < HIST_MS) pk = Math.max(pk, num(p.lvl)); });
  if (pk > 0.5) {
    const y = Y(pk);
    hc.setLineDash([4 * devicePixelRatio, 4 * devicePixelRatio]);
    hc.strokeStyle = '#e8ffe8'; hc.lineWidth = 1;
    hc.beginPath(); hc.moveTo(padL, y); hc.lineTo(hw - padR, y); hc.stroke();
    hc.setLineDash([]);
    // Label at the LEFT end of the dashed line: the right edge already has
    // the #gcur current-value overlay, and drawing both there overlaps.
    hc.fillStyle = '#e8ffe8'; hc.textAlign = 'left';
    hc.fillText('PEAK ' + pk.toFixed(1), padL + 4 * devicePixelRatio, y - 8 * devicePixelRatio);
  }

  if (scanning) {                                    // sweep at the leading edge
    const a = 0.35 + 0.35 * Math.sin(now / 140);
    hc.fillStyle = 'rgba(255,176,0,' + a.toFixed(3) + ')';
    hc.fillRect(hw - padR - 2 * devicePixelRatio, padT, 2 * devicePixelRatio, gh);
  }

  /* Channel labels, no clutter: while a lock run is in the window, its
     channel name with a dashed marker at the lock start and a triangle
     at the current (right) edge; otherwise the top-1 recent peak gets
     its channel name (when the sample carries one). */
  const winH = hist.filter(p => now - p.t < HIST_MS);
  const lockRun = winH.filter(p => p.lock);
  if (lockRun.length) {
    const lx = X(lockRun[0].t), lname = lockRun[lockRun.length - 1].ch || '';
    hc.strokeStyle = '#39ff6a'; hc.lineWidth = 1;
    hc.setLineDash([2 * devicePixelRatio, 2 * devicePixelRatio]);
    hc.beginPath(); hc.moveTo(lx, padT); hc.lineTo(lx, padT + gh); hc.stroke();
    hc.setLineDash([]);
    if (locked) {
      const xe = hw - padR;
      hc.fillStyle = '#39ff6a';
      hc.beginPath();
      hc.moveTo(xe, padT + 5 * devicePixelRatio);
      hc.lineTo(xe - 4 * devicePixelRatio, padT);
      hc.lineTo(xe - 4 * devicePixelRatio, padT + 10 * devicePixelRatio);
      hc.closePath(); hc.fill();
    }
    if (lname) {
      const startVis = lx >= padL - 2 * devicePixelRatio;
      hc.fillStyle = '#39ff6a'; hc.textBaseline = 'top';
      hc.textAlign = startVis ? 'left' : 'right';
      hc.fillText(lname, startVis ? lx + 3 * devicePixelRatio
                                  : hw - padR - 6 * devicePixelRatio,
                  padT + 2 * devicePixelRatio);
    }
  } else {
    let pkP = null;
    winH.forEach(p => { if (p.ch && (!pkP || num(p.lvl) > num(pkP.lvl))) pkP = p; });
    if (pkP) {
      hc.fillStyle = '#ffb000'; hc.textAlign = 'center'; hc.textBaseline = 'bottom';
      hc.fillText(pkP.ch, X(pkP.t), Y(num(pkP.lvl)) - 3 * devicePixelRatio);
    }
  }

  /* SDR-style sweep spectrum: per-channel P vs actual frequency from
     /api/state's spectrum array (the same [SWEEP] data the channel grid
     uses). Auto-ranged dB Y (min span 10), gradient area, slowly-fading
     peak-hold trace, band separators, dwell scanline while scanning. */
  const fe = document.getElementById('fspec'), fc = fe.getContext('2d');
  const fw = fe.width = fe.clientWidth * devicePixelRatio,
        fh = fe.height = fe.clientHeight * devicePixelRatio;
  const fcpt = fe.clientWidth < 520 || fe.clientHeight < 200;   // cramped: slim axes
  const fpadL = (fcpt ? 24 : 30) * devicePixelRatio, fpadR = 8 * devicePixelRatio;
  const fpadT = (fcpt ? 18 : 22) * devicePixelRatio, fpadB = (fcpt ? 14 : 16) * devicePixelRatio;
  const fgw = fw - fpadL - fpadR, fgh = fh - fpadT - fpadB;
  const F_LO = 5560, F_HI = 6000;                  // 5.6-6.0 GHz window
  const FX = f => fpadL + (f - F_LO) / (F_HI - F_LO) * fgw;
  const spec2 = (S && S.spectrum) || [];
  const fvals = [];
  spec2.forEach(e => { if (e && typeof e.p === 'number' && isFinite(e.p)) fvals.push(e.p); });
  const ftgt = rangeTarget(fvals, 0, 45, 10);
  if (fsHi <= fsLo) { fsLo = ftgt[0]; fsHi = ftgt[1]; }
  else { fsLo += (ftgt[0] - fsLo) * 0.12; fsHi += (ftgt[1] - fsHi) * 0.12; }
  const FY = v => fpadT + fgh - (Math.max(fsLo, Math.min(fsHi, v)) - fsLo) / (fsHi - fsLo) * fgh;

  const fspan = fsHi - fsLo, fstep = fspan > 30 ? 10 : fspan > 12 ? 5 : 2;
  fc.font = (9 * devicePixelRatio) + 'px monospace';
  fc.textAlign = 'left'; fc.textBaseline = 'middle';
  for (let v = Math.ceil(fsLo / fstep) * fstep; v <= fsHi + 1e-9; v += fstep) {
    const y = FY(v), major = Math.round(v / fstep) % 2 === 0;   // dB gridlines
    fc.strokeStyle = major ? '#33502f' : '#1a2f1a'; fc.lineWidth = 1;
    fc.beginPath(); fc.moveTo(fpadL, y); fc.lineTo(fw - fpadR, y); fc.stroke();
    if (major) { fc.fillStyle = '#33502f'; fc.fillText(Math.round(v), 4 * devicePixelRatio, y); }
  }
  fc.textAlign = 'center'; fc.textBaseline = 'top';
  for (let f = 5600; f <= F_HI; f += 100) {        // frequency ticks, 100 MHz
    const x = FX(f);
    fc.strokeStyle = '#1a2f1a';
    fc.beginPath(); fc.moveTo(x, fpadT); fc.lineTo(x, fpadT + fgh); fc.stroke();
    fc.fillStyle = '#33502f';
    fc.fillText((f / 1000).toFixed(1), x, fpadT + fgh + 3 * devicePixelRatio);
  }
  const BF = {R: 5658, A: 5725, B: 5733, E: 5645, F: 5740, L: 5362};
  Object.keys(BF).forEach(bd => {                  // band separators
    const f = BF[bd];
    if (f < F_LO || f > F_HI) return;
    const x = FX(f);
    fc.strokeStyle = '#2a4030';
    fc.beginPath(); fc.moveTo(x, fpadT); fc.lineTo(x, fpadT + fgh); fc.stroke();
    fc.fillStyle = '#4a6a4f'; fc.textAlign = 'left';
    fc.fillText(bd, x + 2 * devicePixelRatio, fpadT + 1 * devicePixelRatio);
  });

  // Data in frequency order; a null-p entry breaks the trace into a gap.
  const allF = spec2.filter(e => e && e.freq_mhz >= F_LO && e.freq_mhz <= F_HI)
                    .sort((a, b) => a.freq_mhz - b.freq_mhz);
  const segs = []; let seg = [];
  allF.forEach(e => {
    if (typeof e.p === 'number' && isFinite(e.p)) seg.push([FX(e.freq_mhz), FY(e.p), e]);
    else if (seg.length) { segs.push(seg); seg = []; }
  });
  if (seg.length) segs.push(seg);
  const pts2 = segs.reduce((a, s) => a.concat(s), []);
  segs.forEach(sg => {
    if (sg.length === 1) {                          // lone sample: dot
      fc.fillStyle = '#39ff6a';
      fc.beginPath(); fc.arc(sg[0][0], sg[0][1], 1.6 * devicePixelRatio, 0, 7); fc.fill();
      return;
    }
    fc.beginPath(); fc.moveTo(sg[0][0], fpadT + fgh);
    sg.forEach(p => fc.lineTo(p[0], p[1]));
    fc.lineTo(sg[sg.length - 1][0], fpadT + fgh); fc.closePath();
    const fgrad = fc.createLinearGradient(0, fpadT, 0, fpadT + fgh);
    fgrad.addColorStop(0, '#39ff6a44'); fgrad.addColorStop(1, '#39ff6a00');
    fc.fillStyle = fgrad; fc.fill();
    fc.beginPath(); fc.moveTo(sg[0][0], sg[0][1]);
    sg.forEach(p => fc.lineTo(p[0], p[1]));
    fc.strokeStyle = '#39ff6a'; fc.lineWidth = 1.4 * devicePixelRatio;
    fc.shadowColor = '#39ff6a'; fc.shadowBlur = 6 * devicePixelRatio;
    fc.stroke(); fc.shadowBlur = 0;
  });
  if (holdOn) {
    spec2.forEach(e => {                           // peak-hold bookkeeping
      if (!e || typeof e.p !== 'number' || !isFinite(e.p)) return;
      const r = fPeaks[e.name];
      if (!r || e.p >= r.v) fPeaks[e.name] = {v: e.p, t: now};
      else if (now - r.t > 5000) r.v = Math.max(e.p, r.v - 0.05);  // slow fade
    });
    const pk2 = spec2.filter(e => e && fPeaks[e.name] &&
                                  e.freq_mhz >= F_LO && e.freq_mhz <= F_HI)
                     .map(e => [FX(e.freq_mhz), FY(fPeaks[e.name].v)])
                     .sort((a, b) => a[0] - b[0]);
    if (pk2.length > 1) {
      fc.beginPath(); fc.moveTo(pk2[0][0], pk2[0][1]);
      pk2.forEach(p => fc.lineTo(p[0], p[1]));
      fc.setLineDash([3 * devicePixelRatio, 3 * devicePixelRatio]);
      fc.strokeStyle = '#e8ffe8aa'; fc.lineWidth = 1;
      fc.stroke(); fc.setLineDash([]);
    }
  }
  // Top-3 peak labels, strongest first; re-sorted by x, then overlapping
  // neighbours stagger onto a second row (or drop) so they never collide.
  const lbls = pts2.slice().sort((a, b) => b[2].p - a[2].p).slice(0, 3)
                   .sort((a, b) => a[0] - b[0]);
  let lx0 = -1e9, lx1 = -1e9;
  lbls.forEach(p => {
    let row = 0;
    if (p[0] - lx0 < 26 * devicePixelRatio) {
      if (p[0] - lx1 < 26 * devicePixelRatio) return;
      row = 1;
    }
    if (row) lx1 = p[0]; else lx0 = p[0];
    fc.fillStyle = '#fff'; fc.textAlign = 'center'; fc.textBaseline = 'bottom';
    fc.shadowColor = '#39ff6a'; fc.shadowBlur = 4 * devicePixelRatio;
    fc.fillText(p[2].name, p[0],
                p[1] - 2 * devicePixelRatio - row * 11 * devicePixelRatio);
    fc.shadowBlur = 0;
  });
  if (S && S.scan) {                               // dwell scanline at the
    let fresh = null;                              // freshest [SWEEP] entry
    spec2.forEach(e => {
      if (e && e.age_s !== null && e.age_s !== undefined &&
          (!fresh || e.age_s < fresh.age_s)) fresh = e;
    });
    if (fresh && fresh.freq_mhz >= F_LO && fresh.freq_mhz <= F_HI) {
      const a = 0.10 + 0.08 * Math.sin(now / 160);
      fc.fillStyle = 'rgba(255,176,0,' + a.toFixed(3) + ')';
      fc.fillRect(FX(fresh.freq_mhz) - 4 * devicePixelRatio, fpadT,
                  8 * devicePixelRatio, fgh);
    }
  }

  /* Waterfall (top of this panel): offscreen buffer shifted up one row
     per poll, newest at the bottom (gqrx convention). Each channel's
     strip spans its midpoint boundaries on the true-frequency axis;
     colors through WFLUT on the trace's smoothed dB range; channel
     names rotated 90 degrees along the bottom. */
  const we = document.getElementById('wfall'), wc = we.getContext('2d');
  const ww = we.width = we.clientWidth * devicePixelRatio,
        wh = we.height = we.clientHeight * devicePixelRatio;
  const wcpt = we.clientWidth < 520 || we.clientHeight < 200;
  const wpadL = (wcpt ? 24 : 30) * devicePixelRatio, wpadR = 8 * devicePixelRatio;
  const wpadT = (wcpt ? 18 : 20) * devicePixelRatio;
  const wgw = ww - wpadL - wpadR;
  const wstag = wgw / Math.max(allF.length, 1) < 9 * devicePixelRatio;
  const wpadB = (wstag ? 30 : 16) * devicePixelRatio;  // label strip below
  const wgh = wh - wpadT - wpadB;
  const WX = f => wpadL + (f - F_LO) / (F_HI - F_LO) * wgw;
  if (wgw > 0 && wgh > 0) {
    if (wfOff.width !== wgw || wfOff.height !== wgh) {
      wfOff.width = wgw; wfOff.height = wgh;
      wfc.fillStyle = '#050805'; wfc.fillRect(0, 0, wgw, wgh);
    }
    const rowH = Math.max(1, Math.round(2 * devicePixelRatio));
    if (wfNew && wfNew.length) {
      const row = wfNew; wfNew = null;
      wfc.drawImage(wfOff, 0, -rowH);              // scroll up (snapshot copy)
      wfc.fillStyle = '#050805';                   // gaps stay dark
      wfc.fillRect(0, wgh - rowH, wgw, rowH);
      const wspan = Math.max(1e-6, fsHi - fsLo);
      row.forEach((e, i) => {
        if (typeof e.p !== 'number' || !isFinite(e.p)) return;
        const x0 = i ? (WX(e.freq_mhz) + WX(row[i - 1].freq_mhz)) / 2 - wpadL : 0;
        const x1 = i < row.length - 1
                 ? (WX(e.freq_mhz) + WX(row[i + 1].freq_mhz)) / 2 - wpadL : wgw;
        const v = Math.max(0, Math.min(1, (e.p - fsLo) / wspan));
        wfc.fillStyle = WFLUT[(v * 255) | 0];
        wfc.fillRect(x0, wgh - rowH, Math.max(1, x1 - x0), rowH);
      });
    }
    wc.clearRect(0, 0, ww, wh);
    wc.drawImage(wfOff, wpadL, wpadT);
    Object.keys(BF).forEach(bd => {                // band separators
      const f = BF[bd];
      if (f < F_LO || f > F_HI) return;
      const x = WX(f);
      wc.strokeStyle = '#2a4030'; wc.lineWidth = 1;
      wc.beginPath(); wc.moveTo(x, wpadT); wc.lineTo(x, wpadT + wgh); wc.stroke();
    });
    if (S && S.scan) {                             // dwell marker, newest row
      let wfresh = null;
      spec2.forEach(e => {
        if (e && e.age_s !== null && e.age_s !== undefined &&
            (!wfresh || e.age_s < wfresh.age_s)) wfresh = e;
      });
      if (wfresh && wfresh.freq_mhz >= F_LO && wfresh.freq_mhz <= F_HI) {
        wc.fillStyle = '#ffb000';
        wc.fillRect(WX(wfresh.freq_mhz) - 4 * devicePixelRatio,
                    wpadT + wgh - 2 * devicePixelRatio,
                    8 * devicePixelRatio, 2 * devicePixelRatio);
      }
    }
    /* Dedicated label strip below the plot: every channel name at its
       frequency position, rotated vertical, fully inside the strip (never
       over data). Two staggered rows when the per-channel pitch is too
       tight; the active channel's label is lit. */
    wc.fillStyle = '#050805';
    wc.fillRect(wpadL, wpadT + wgh, wgw, wh - wpadT - wgh);
    wc.strokeStyle = '#1a2f1a'; wc.lineWidth = 1;
    wc.beginPath(); wc.moveTo(wpadL, wpadT + wgh + 0.5);
    wc.lineTo(ww - wpadR, wpadT + wgh + 0.5); wc.stroke();
    wc.font = (9 * devicePixelRatio) + 'px monospace';
    wc.textAlign = 'left'; wc.textBaseline = 'middle';
    const curCh = S && S.telemetry.channel;
    allF.forEach((e, i) => {
      const ly = wpadT + wgh + (wstag ? (i % 2) * 14 + 14 : 14) * devicePixelRatio;
      wc.save();
      wc.translate(WX(e.freq_mhz), ly);
      wc.rotate(-Math.PI / 2);
      if (e.name === curCh) {
        wc.fillStyle = '#e8ffe8'; wc.shadowColor = '#39ff6a';
        wc.shadowBlur = 5 * devicePixelRatio;
      } else {
        wc.fillStyle = '#4a6a4f';
      }
      wc.fillText(e.name, 0, 0);
      wc.restore();
    });
    wc.shadowBlur = 0;
  }
  } catch (e) { /* one bad frame must not kill the loop */ }
  finally { requestAnimationFrame(draw); }
}
setInterval(poll, 500); poll(); requestAnimationFrame(draw);
setInterval(() => { if (!tabLive) pollDets(); }, 500);
if (lsGet('c5_tab') === 'dets') showTab('dets');
</script>
</body>
</html>"""


def create_app() -> Flask:
    app = Flask(__name__)
    app.json.sort_keys = False  # keep CSV column order in /api/detections

    @app.get("/")
    def index():
        return PAGE.replace("__CHANNEL_GRID__", channel_grid_js())

    @app.get("/video_feed")
    def video_feed():
        return Response(mjpeg_stream(),
                        mimetype="multipart/x-mixed-replace; boundary=frame")

    @app.get("/api/state")
    def api_state():
        with STATE.lock:
            snap = dict(STATE.telemetry)
            frame_age = (time.monotonic() - STATE.frame_time
                         if STATE.frame is not None else None)
            # Video-pane classification: nosignal only when disconnected or
            # frameless for a long stretch; otherwise sync/snow from the
            # lock flag (rows bit1, or legacy descriptor flags bit0).
            if frame_age is None or not STATE.connected or \
                    frame_age > PLACEHOLDER_AFTER_S:
                video_mode = "nosignal"
            elif STATE.last_frame_flags is not None and \
                    (STATE.last_frame_flags & 1):
                video_mode = "sync"
            else:
                video_mode = "snow"
            body = {
                "connected": STATE.connected,
                "serial_port": STATE.serial_port,
                "preview_on": STATE.preview_on,
                "preview_available": STATE.preview_available,
                "scan": STATE.scan,
                "scan_desired": STATE.scan_desired,
                "skip_dead": STATE.skip_dead,
                "skip_desired": STATE.skip_desired,
                "buzzer_enabled": STATE.buzzer_enabled,
                "buzzer_desired": STATE.buzzer_desired,
                "locked": STATE.locked,
                "detections_total": _dets_total,
                "spectrum": [dict(e) for e in STATE.spectrum],
                "telemetry": snap,
                "frame_age_s": None if frame_age is None else round(frame_age, 2),
                "video_mode": video_mode,
                "pipeline": {
                    "pkts_stream_info": STATE.pkts_info,
                    "pkts_gray8_frame": STATE.pkts_frame,
                    "pkts_gray8_rows": STATE.pkts_rows,
                    "pkts_stream_end": STATE.pkts_end,
                    "parse_errors": STATE.parse_errors,
                    "last_frame_flags": STATE.last_frame_flags,
                    "preview_stats": STATE.preview_stats,
                    "canvas": [STATE.frame_w, STATE.frame_h],
                },
            }
        body["fps"] = round(STATE.fps(), 2)
        return jsonify(body)

    @app.post("/api/key/<k>")
    def api_key(k):
        if len(k) != 1 or k not in ALLOWED_KEYS:
            return jsonify({"ok": False, "error": "key not allowed"}), 400
        if not SERIAL.send_key(k):
            return jsonify({"ok": False, "error": "serial busy or unavailable"}), 503
        if k == "g":
            with STATE.lock:
                STATE.scan_desired = not STATE.scan
                STATE.desired_since["scan"] = time.monotonic()
        elif k == "u":
            with STATE.lock:
                STATE.buzzer_desired = not STATE.buzzer_enabled
                STATE.desired_since["buzzer"] = time.monotonic()
        elif k == "k":
            with STATE.lock:
                STATE.skip_desired = not STATE.skip_dead
                STATE.desired_since["skip"] = time.monotonic()
        return jsonify({"ok": True, "key": k})

    @app.post("/api/tune/<int:idx>")
    def api_tune(idx):
        if not 0 <= idx < CHANNEL_COUNT:
            return jsonify({"ok": False, "error": "index out of range 0..47"}), 400
        if not SERIAL.send_park(idx):
            return jsonify({"ok": False, "error": "rate limited or serial unavailable"}), 429
        band, ch = divmod(idx, 8)
        name = f"{CHANNELS[band][0]}{ch + 1}"
        with STATE.lock:
            STATE.locked = False  # a retune always drops the carrier first
        return jsonify({"ok": True, "index": idx, "channel": name})

    @app.post("/api/shot")
    def api_shot():
        data = current_jpeg()
        with STATE.lock:
            ep_start = EPISODE["start"] if EPISODE is not None else None
        base = _shot_stamp(ep_start if ep_start is not None else time.time())
        SHOTS_DIR.mkdir(exist_ok=True)
        name = f"{base}__{time.strftime('%H%M%S')}.jpg"
        path = SHOTS_DIR / name
        n = 2
        while path.exists():
            name = f"{base}__{time.strftime('%H%M%S')}_{n}.jpg"
            path = SHOTS_DIR / name
            n += 1
        path.write_bytes(data)
        return jsonify({"ok": True, "name": name})

    @app.get("/shots/<path:filename>")
    def shot_file(filename):
        # send_from_directory resolves through safe_join: no traversal.
        return send_from_directory(SHOTS_DIR, filename)

    @app.get("/api/detections")
    def api_detections():
        rows, total = _det_read_rows()
        return jsonify({"total": total, "rows": rows, "shots": _shot_list()})

    @app.post("/api/detections/clear")
    def clear_detections():
        # Truncate to header-only; a still-open episode appends normally
        # when it ends.
        global _dets_total
        with _dets_lock:
            with open(DETECTIONS_CSV, "w", newline="") as f:
                csv.DictWriter(f, fieldnames=DETECTIONS_HEADER).writeheader()
            _dets_total = 0
        return jsonify({"ok": True, "total": 0})

    @app.get("/api/detections/export")
    def export_detections():
        # Header-only CSV when no detections have been logged yet.
        with _dets_lock:
            if DETECTIONS_CSV.exists():
                data = DETECTIONS_CSV.read_bytes()
            else:
                data = (",".join(DETECTIONS_HEADER) + "\r\n").encode()
        return Response(
            data, mimetype="text/csv",
            headers={"Content-Disposition":
                     "attachment; filename=detections.csv"})

    return app


SERIAL = None


def main() -> None:
    global SERIAL
    ap = argparse.ArgumentParser(description="C5VRX-3 foxhunt HUD")
    ap.add_argument("serial_port", nargs="?", default="/dev/cu.usbmodem312301")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--host", default="0.0.0.0")
    ap.add_argument("--port", type=int, default=5000)
    args = ap.parse_args()

    SERIAL = SerialManager(args.serial_port, args.baud)
    SERIAL.start()
    app = create_app()
    try:
        app.run(host=args.host, port=args.port, threaded=True, use_reloader=False)
    except KeyboardInterrupt:
        pass
    finally:
        print("\nshutting down: stopping preview stream")
        SERIAL.close()
        SERIAL.join(timeout=2.0)


if __name__ == "__main__":
    main()
