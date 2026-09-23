#!/usr/bin/env python3
"""C5VRX-3 foxhunt HUD: MJPEG video + radio meters + receiver control over LAN.

Serves the USB video preview (224x168 GRAY8, row-streamed binary packets on
the USB Serial/JTAG console; legacy 128x96 whole frames still accepted) as
an MJPEG stream any browser can watch, with a
HUD-style dashboard: animated carrier-level meter with peak-hold, Q_phase
and gain bars, a 60 s level-history graph, a 48-channel direct-tune grid,
console-key control buttons, and an ELRS sniffer card + full tab
(auto-discovered LilyGo T3-S3 running elrs-sniffer/, JSON lines at 460800
on its own USB port). Mobile-friendly, so an iPhone on the LAN
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
import json
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
    MAGIC,
    PACKET_GRAY8_FRAME,
    PACKET_GRAY8_ROWS,
    PACKET_STREAM_END,
    PACKET_STREAM_INFO,
    PIXEL_FORMAT_GRAY8,
    StreamDecoder,
    decode_gray8_rows,
)

import serial  # noqa: E402
from flask import Flask, Response, jsonify, request, send_from_directory  # noqa: E402
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
ROW_SNAPSHOT_AFTER_S = 0.4  # partial frames: if rows keep arriving but no
                            # bit0 completes a frame, snapshot the canvas
                            # after this long so new video always surfaces
ROWS_STALE_S = 3.0          # rows older than this mark the canvas stale
                            # (amber border) instead of showing bright pixels
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
        self.row_time = 0.0          # monotonic time of the last row bytes
        self.snap_seq = 0            # decreasing synthetic seq for snapshots
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

    def put_frame(self, payload: bytes, seq: int, count_fps: bool = True) -> None:
        now = time.monotonic()
        with self.frame_cond:
            self.frame = payload
            self.frame_seq = seq
            self.frame_time = now
            self.preview_on = True   # flowing frames prove the stream is on
            if count_fps:
                self.frame_times = [t for t in self.frame_times if now - t < 5.0]
                self.frame_times.append(now)
            self.frame_cond.notify_all()

    def publish_snapshot(self) -> None:
        """Publish the current row canvas as a partial-frame snapshot.

        Used by the MJPEG loop when rows keep arriving but no bit0 row
        completes a frame (reconnecting/marginal video): the display must
        track the newest received rows instead of freezing on the last
        complete frame. Not counted in FPS -- only bit0 completions are.
        """
        self.snap_seq -= 1   # negative seqs can never collide with packet seqs
        self.put_frame(bytes(self.row_buf), self.snap_seq, count_fps=False)

    def fps(self) -> float:
        with self.lock:
            if len(self.frame_times) < 2:
                return 0.0
            span = max(self.frame_times[-1] - self.frame_times[0], 1e-6)
            return (len(self.frame_times) - 1) / span


STATE = State()


# ----- Unified serial port arbitration -----
#
# Two USB devices may be present: the OUI-SPY board (binary C5VRX-magic
# packets + "[TAG]" console text at the configured baud) and the ELRS
# sniffer dongle (JSON lines at 460800). Either reader blindly opening the
# other's port wedges both (field case: the dongle alone on the configured
# default port was grabbed by the main reader, leaving elrs offline), so
# NO reader may open a port without positive evidence. PortArbiter owns
# discovery for both: claim() probes each unclaimed port at both bauds
# (~1.1 s per baud) and classifies by content; a port is claimed only on a
# positive match for the caller's kind. Unclassified ports are closed and
# retried on the caller's next backoff cycle. Readers release their claim
# on disconnect; a port classified as the other device is remembered
# (known) so its reader tries it first next pass.

ELRS_BAUD = 460800
BOARD_TAGS = (b"C5VRX-3", b"[SWEEP]", b"[AGC:", b"[CARRIER]", b"[PREVIEW]")


def classify_evidence(buf: bytes):
    """bytes -> "board" | "elrs" | None. ELRS = an LF-delimited line that
    parses as a JSON object with a string "t" key; board = the binary
    C5VRX magic, the boot banner, or a console tag."""
    for line in buf.split(b"\n"):
        line = line.strip()
        if not line.startswith(b"{"):
            continue
        try:
            obj = json.loads(line.decode("utf-8", "replace"))
        except ValueError:
            continue
        if isinstance(obj, dict) and isinstance(obj.get("t"), str):
            return "elrs"
    if MAGIC in buf:
        return "board"
    for tag in BOARD_TAGS:
        if tag in buf:
            return "board"
    return None


class PortArbiter:
    """See the notes above. Probing happens outside the lock; claimed and
    known are the only shared state."""

    PROBE_PER_BAUD_S = 1.1

    def __init__(self, board_baud: int) -> None:
        self.board_baud = board_baud
        self.lock = threading.Lock()
        self.claimed = {}            # port -> "board" | "elrs"
        self.known = {}              # port -> last classification
        self.last_class = None       # (port, kind, monotonic) debug

    def _order(self, kind: str, hint) -> list:
        """Unclaimed ports: configured hint first, then ports previously
        classified as this kind, then the rest."""
        with self.lock:
            free = [p for p in sorted(glob.glob("/dev/cu.usbmodem*"))
                    if p not in self.claimed]
            known_match = [p for p in free if self.known.get(p) == kind]
        rest = [p for p in free if p != hint and p not in known_match]
        return ([hint] if hint in free else []) + known_match + rest

    @staticmethod
    def _open_raw(port: str, baud: int):
        # dtr/rts deasserted before open: probing must not reset either
        # device with a control-line pulse.
        ser = serial.Serial()
        ser.port = port
        ser.baudrate = baud
        ser.timeout = 0.1
        ser.dtr = False
        ser.rts = False
        ser.open()
        return ser

    def _probe(self, port: str):
        """(kind, open serial at the matching baud) or (None)."""
        for baud in (ELRS_BAUD, self.board_baud):
            try:
                ser = self._open_raw(port, baud)
            except (serial.SerialException, OSError):
                continue
            try:
                deadline = time.monotonic() + self.PROBE_PER_BAUD_S
                buf = b""
                while time.monotonic() < deadline:
                    chunk = ser.read(1024)
                    if chunk:
                        buf += chunk
                        kind = classify_evidence(buf)
                        if kind:
                            return kind, ser
            except (serial.SerialException, OSError):
                pass
            try:
                ser.close()
            except OSError:
                pass
        return None

    def claim(self, kind: str, hint=None):
        """First free port with positive evidence for `kind`, open at the
        right baud and claimed; None when nothing classifies right now."""
        for port in self._order(kind, hint):
            found = self._probe(port)
            if found is None:
                continue
            cls, ser = found
            with self.lock:
                self.known[port] = cls
                self.last_class = (port, cls, time.monotonic())
            print(f"[arbiter] classified {port} as {cls}")
            if cls != kind:
                try:
                    ser.close()
                except OSError:
                    pass
                continue
            with self.lock:
                if port in self.claimed:   # lost a claim race while probing
                    try:
                        ser.close()
                    except OSError:
                        pass
                    continue
                self.claimed[port] = kind
            return ser
        return None

    def release(self, port) -> None:
        with self.lock:
            self.claimed.pop(port, None)

    def snapshot(self) -> dict:
        now = time.monotonic()
        with self.lock:
            last = None
            if self.last_class:
                p, k, t = self.last_class
                last = {"port": p, "kind": k, "age_s": round(now - t, 1)}
            return {"claimed": dict(self.claimed), "known": dict(self.known),
                    "last": last}


ARBITER = PortArbiter(115200)


# ----- Detection episode logging (tools/detections.csv) -----
#
# One CSV row per detection episode, unified across both RF bands: 5.8
# video episodes (lock acquire -> lock lost, state mutated only under
# STATE.lock) and 2.4 ELRS link encounters (sniffer lock rise -> fall, own
# lock, see "ELRS encounter episodes" below), plus standalone event rows
# (boot). The file write is a single small append guarded by its own lock
# so the serial reader thread never blocks on anything bigger than one row.
# _det_read_rows migrates legacy files on read: a header whose first column
# is not "band" predates the RF-band column (its "band" was the Boscam
# sub-band), and rows may predate trailing columns.

DETECTIONS_CSV = Path(__file__).resolve().parent / "detections.csv"
DETECTIONS_HEADER = [
    "band",                        # RF band: "5.8" video | "2.4" ELRS link
    "start_iso", "end_iso", "duration_s",
    "channel", "subband", "freq_mhz",
    "level_peak_db", "level_mean_db", "level_min_db", "level_samples",
    "video_sync", "frames_received", "max_fps",
    "skip_dead", "dwell_ms", "lock_type", "end_reason",
    "cfo_ppm", "video_std", "line_us", "uid",
    "geo_lat", "geo_lon",
]
# Columns coerced to numbers on read; everything else stays a string (so
# band "5.8" never becomes float 5.8, iso strings never get mangled).
_DET_NUMERIC = frozenset({
    "duration_s", "freq_mhz", "level_peak_db", "level_mean_db",
    "level_min_db", "level_samples", "video_sync", "frames_received",
    "max_fps", "skip_dead", "dwell_ms", "cfo_ppm", "line_us",
    "geo_lat", "geo_lon",
})
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
    `limit`; ([], 0) when no CSV exists yet. Migrates legacy files: a header
    whose first column is not "band" predates the RF-band column — its own
    "band" column was the Boscam sub-band — and rows may also predate
    trailing columns (overflow cells map onto the missing tail in order)."""
    with _dets_lock:
        try:
            with open(DETECTIONS_CSV, newline="") as f:
                rdr = csv.reader(f)
                hdr = next(rdr, None)
                data = list(rdr) if hdr else []
        except OSError:
            return [], 0
    if not hdr:
        return [], 0
    legacy = hdr[0] != "band"
    if legacy:
        names = ["subband" if h == "band" else h for h in hdr]
    else:
        names = hdr[1:]
    total = len(data)
    data.reverse()
    out = []
    for row in data[:limit]:
        if legacy and row and row[0] in ("5.8", "2.4"):
            # New-style row appended to an old-header file.
            band, cells = row[0], row[1:]
        elif legacy:
            band, cells = "5.8", row
        else:
            band, cells = (row[0] if row else ""), row[1:]
        r = {"band": band}
        r.update(zip(names, cells))
        if len(cells) > len(names):
            extra = cells[len(names):]
            missing = [k for k in DETECTIONS_HEADER if k not in r]
            for k, v in zip(missing, extra):
                r[k] = v
        out.append({k: (_det_coerce(r.get(k, "")) if k in _DET_NUMERIC
                        else r.get(k, ""))
                    for k in DETECTIONS_HEADER})
    return out, total


# ----- Drone aliases (tools/drone_aliases.json) -----
#
# User-named display aliases for fingerprinted drones, keyed by the same
# fingerprint the DETECTIONS tab clusters on (line_us rounded to the
# nearest 0.02 us + "|" + video_std, e.g. "63.98|PAL"). Display-only: the
# fingerprint stays the identity; detections.csv is never rewritten.
# Same pattern as _dets_lock: state mutated and file written under one
# lock; missing/corrupt file reads as {}.

DRONE_ALIASES_JSON = Path(__file__).resolve().parent / "drone_aliases.json"
ALIAS_FP_RE = re.compile(r"^(\d+\.\d{2}\|\w{0,8}|elrs:[0-9A-Fa-f]+)$")
ALIAS_MAX = 40

_aliases_lock = threading.Lock()


def _load_aliases() -> dict:
    try:
        with open(DRONE_ALIASES_JSON) as f:
            data = json.load(f)
        if not isinstance(data, dict):
            return {}
        return {str(k): str(v) for k, v in data.items()}
    except (OSError, ValueError):
        return {}


DRONE_ALIASES = _load_aliases()


def _save_aliases() -> None:
    """Write the alias map atomically (caller holds _aliases_lock)."""
    tmp = DRONE_ALIASES_JSON.with_suffix(".tmp")
    with open(tmp, "w") as f:
        json.dump(DRONE_ALIASES, f, indent=1, sort_keys=True)
    tmp.replace(DRONE_ALIASES_JSON)


# ----- Browser geolocation (X-Geo header on /api/state polls) -----
#
# The HUD page asks for the browser's geolocation (permission-gated,
# remembered in localStorage) and ships the fix as an "X-Geo: lat,lon,acc"
# header on every state poll. The fix is sampled when a detection episode
# opens and lands in its CSV row (geo_lat/geo_lon); a fix older than
# GEO_STALE_S counts as absent.

GEO_STALE_S = 30.0
GEO = {"lat": None, "lon": None, "acc": None, "t": 0.0}
_geo_lock = threading.Lock()


def _geo_set(lat: float, lon: float, acc) -> None:
    with _geo_lock:
        GEO.update(lat=lat, lon=lon, acc=acc, t=time.monotonic())


def _geo_current():
    """(lat, lon) or None when there is no fix or it has gone stale."""
    with _geo_lock:
        if GEO["lat"] is None or time.monotonic() - GEO["t"] > GEO_STALE_S:
            return None
        return GEO["lat"], GEO["lon"]


# ----- Video screenshots (tools/shots/) -----
#
# One JPEG per camera-button press, plus an auto-shot on the first synced
# frame of each episode and a best-OSD frame at episode end. Named
# <episode-start>__<tag>.jpg when an episode is open so the DETECTIONS tab
# can match shots to rows (row start_iso, both sanitized to
# YYYY-MM-DDTHH-MM-SS); otherwise the capture time is the base. Plain
# files, served back via /shots/<name>.

SHOTS_DIR = Path(__file__).resolve().parent / "shots"

AUTO_SHOT_SUPPRESS_S = 3.0    # manual shot this recent suppresses the auto one
OSD_MIN_SCORE = 24            # high-contrast blocks needed to keep an OSD shot

_last_shot_mono = 0.0         # last shot (manual or auto), monotonic


def _shot_stamp(ts: float) -> str:
    """Filename-safe local timestamp matching the CSV start_iso prefix."""
    return _iso(ts)[:19].replace(":", "-")


def _shot_list() -> list:
    try:
        return sorted(p.name for p in SHOTS_DIR.iterdir()
                      if p.suffix == ".jpg")
    except OSError:
        return []


def _save_shot(data: bytes, base: str, tag: str) -> str:
    """Write one JPEG as <base>__<tag>.jpg (collision-suffixed); returns
    the file name."""
    SHOTS_DIR.mkdir(exist_ok=True)
    name = f"{base}__{tag}.jpg"
    path = SHOTS_DIR / name
    n = 2
    while path.exists():
        name = f"{base}__{tag}_{n}.jpg"
        path = SHOTS_DIR / name
        n += 1
    path.write_bytes(data)
    return name


def _frame_jpeg(frame: bytes, dims) -> bytes:
    """JPEG of a raw GRAY8 canvas, scaled like the live feed."""
    img = Image.frombytes("L", dims, frame)
    img = img.resize((dims[0] * SCALE, dims[1] * SCALE), Image.NEAREST)
    return make_jpeg(img)


def _osd_score(frame: bytes, w: int, h: int) -> int:
    """OSD-text likelihood: count of high-contrast dark-on-light 8x8 blocks
    in the center 60% of the frame (OSD glyphs sit center-screen and are
    near-black text on a near-white outline)."""
    if not frame or w < 40 or h < 40:
        return 0
    x0, x1 = w // 5, w * 4 // 5
    y0, y1 = h // 5, h * 4 // 5
    score = 0
    for by in range(y0, y1 - 7, 8):
        for bx in range(x0, x1 - 7, 8):
            block = b"".join(frame[y * w + bx: y * w + bx + 8]
                             for y in range(by, by + 8))
            if max(block) - min(block) >= 96 and sum(block) >= 64 * 128:
                score += 1
    return score


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
    geo = _geo_current()
    EPISODE = {
        "start": time.time(),
        "channel": chan, "band": band, "freq_mhz": freq_mhz,
        "peak": None, "min": None, "sum": 0.0, "n": 0,
        "video_sync": False,
        "osd_best": None,          # (score, frame bytes) while synced
        "frames0": STATE.pkts_frame,
        "max_fps": 0.0,
        "skip_dead": STATE.skip_dead,
        # Browser fix sampled at episode open; "" when absent/stale.
        "geo_lat": "" if geo is None else round(geo[0], 6),
        "geo_lon": "" if geo is None else round(geo[1], 6),
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


def _episode_video_sync(frame: bytes) -> None:
    """A GRAY8 frame arrived while the grabber reports video sync. Caller
    holds STATE.lock. The first synced frame of an episode triggers an
    auto-shot (suppressed when a manual shot just happened); every synced
    frame is OSD-scored and the best is kept for the episode's __osd.jpg."""
    global _last_shot_mono
    ep = EPISODE
    if ep is None:
        return
    dims = (STATE.frame_w, STATE.frame_h)
    score = _osd_score(frame, *dims)
    best = ep["osd_best"]
    if best is None or score > best[0]:
        ep["osd_best"] = (score, bytes(frame))
    if ep["video_sync"]:
        return
    ep["video_sync"] = True
    now = time.monotonic()
    if now - _last_shot_mono < AUTO_SHOT_SUPPRESS_S:
        return
    _last_shot_mono = now
    try:
        _save_shot(_frame_jpeg(frame, dims), _shot_stamp(ep["start"]),
                   time.strftime("%H%M%S"))
    except OSError:
        pass


def _episode_end(reason: str) -> None:
    """Close the open episode and append its row. Caller holds STATE.lock."""
    global EPISODE
    ep = EPISODE
    if ep is None:
        return
    EPISODE = None
    end = time.time()
    row = {
        "band": "5.8",
        "start_iso": _iso(ep["start"]),
        "end_iso": _iso(end),
        "duration_s": round(end - ep["start"], 1),
        "channel": ep["channel"],
        "subband": ep["band"],
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
        "uid": "",
        "geo_lat": ep["geo_lat"],
        "geo_lon": ep["geo_lon"],
    }
    _det_write_row(row)
    best = ep["osd_best"]
    if best is not None and best[0] > OSD_MIN_SCORE:
        try:
            _save_shot(_frame_jpeg(best[1], (STATE.frame_w, STATE.frame_h)),
                       _shot_stamp(ep["start"]), "osd")
        except OSError:
            pass


def _det_event(name: str) -> None:
    """Standalone notable event as its own row (e.g. firmware boot)."""
    now = _iso(time.time())
    _det_write_row({
        "band": "5.8",
        "start_iso": now, "end_iso": now, "duration_s": 0,
        "channel": "", "subband": "", "freq_mhz": "",
        "level_peak_db": "", "level_mean_db": "", "level_min_db": "",
        "level_samples": 0, "video_sync": "", "frames_received": "",
        "max_fps": "", "skip_dead": "", "dwell_ms": "",
        "lock_type": "event", "end_reason": name,
        "cfo_ppm": "", "video_std": "", "line_us": "", "uid": "",
        "geo_lat": "", "geo_lon": "",
    })


# ----- ELRS encounter episodes -----
#
# The sniffer's link-lock rises/falls are detection episodes too, logged to
# the same detections.csv as band "2.4" rows under their own lock, so the
# sniffer thread appends without ever touching STATE. freq_mhz is the ELRS
# 2.4 GHz sync channel the sniffer parks on; levels are sniffer RSSI.

ELRS_FREQ_MHZ = 2441.4
ELRS_EP = None                    # open ELRS episode dict, or None
_elrs_ep_lock = threading.Lock()


def _elrs_ep_open_locked(obj: dict) -> None:
    """Caller holds _elrs_ep_lock."""
    global ELRS_EP
    rate = str(obj.get("rate") or "-").removeprefix("LoRa ")
    iq = str(obj.get("iq") or "-")
    geo = _geo_current()
    ELRS_EP = {
        "start": time.time(),
        "channel": f"{rate}/{iq}",
        "peak": None, "min": None, "sum": 0.0, "n": 0,
        # Browser fix sampled at episode open; "" when absent/stale.
        "geo_lat": "" if geo is None else round(geo[0], 6),
        "geo_lon": "" if geo is None else round(geo[1], 6),
    }


def _elrs_ep_close_locked(reason: str) -> None:
    """Caller holds _elrs_ep_lock."""
    global ELRS_EP
    ep = ELRS_EP
    if ep is None:
        return
    ELRS_EP = None
    with ELRS.lock:
        uid = ELRS.uid
    end = time.time()
    _det_write_row({
        "band": "2.4",
        "start_iso": _iso(ep["start"]),
        "end_iso": _iso(end),
        "duration_s": round(end - ep["start"], 1),
        "channel": ep["channel"],
        "subband": "ELRS",
        "freq_mhz": ELRS_FREQ_MHZ,
        "level_peak_db": "" if ep["peak"] is None else round(ep["peak"], 2),
        "level_mean_db": "" if not ep["n"] else round(ep["sum"] / ep["n"], 2),
        "level_min_db": "" if ep["min"] is None else round(ep["min"], 2),
        "level_samples": ep["n"],
        "video_sync": "", "frames_received": "", "max_fps": "",
        "skip_dead": "", "dwell_ms": "",
        "lock_type": "elrs",
        "end_reason": reason,
        "cfo_ppm": "", "video_std": "", "line_us": "",
        "uid": uid or "",
        "geo_lat": ep["geo_lat"],
        "geo_lon": ep["geo_lon"],
    })


def _elrs_ep_tick(obj: dict) -> None:
    """Track sniffer objects: open an encounter on lock rise, accumulate
    RSSI while locked, close on lock fall / explicit unlock. Called from
    ElrsManager right after ELRS.update(obj)."""
    t = obj.get("t")
    with _elrs_ep_lock:
        if t == "unlock":
            _elrs_ep_close_locked("unlock:" + str(obj.get("why") or "?"))
            return
        if t != "stats":
            return
        if obj.get("lock"):
            if ELRS_EP is None:
                _elrs_ep_open_locked(obj)
            rssi = obj.get("rssi")
            if isinstance(rssi, (int, float)):
                ep = ELRS_EP
                ep["peak"] = rssi if ep["peak"] is None else max(ep["peak"], rssi)
                ep["min"] = rssi if ep["min"] is None else min(ep["min"], rssi)
                ep["sum"] += rssi
                ep["n"] += 1
        else:
            _elrs_ep_close_locked("signal lost")


def _elrs_ep_end(reason: str) -> None:
    """Lock-taking wrapper for callers outside the tick path."""
    with _elrs_ep_lock:
        _elrs_ep_close_locked(reason)


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

    def _open(self) -> None:
        """Claim a port via the shared arbitrator. The configured port is
        only a preference HINT: no port is opened without positive board
        evidence (banner/console tag/binary magic), so an ELRS-only port
        can never be grabbed by this reader."""
        ser = ARBITER.claim("board", hint=self.port)
        if ser is None:
            raise serial.SerialException("no port with board evidence")
        ser.timeout = 0.2
        ser.reset_input_buffer()
        self.ser = ser
        with STATE.lock:
            STATE.connected = True
            STATE.serial_port = ser.port
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
                if self.ser is not None:
                    ARBITER.release(self.ser.port)
                    try:
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
                        _episode_video_sync(frame)
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
                    STATE.row_time = time.monotonic()
                    if r.flags & 1:
                        frame = bytes(STATE.row_buf)
                        locked = bool(r.flags & 2)
            if frame is not None:
                with STATE.lock:
                    STATE.last_frame_flags = 1 if locked else 0
                    if locked:
                        _episode_video_sync(frame)
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
        if self.ser is not None:
            ARBITER.release(self.ser.port)
            try:
                self.ser.close()
            except OSError:
                pass


# ----- ELRS sniffer (second serial port, elrs-sniffer/) -----
#
# A LilyGo T3-S3 running the elrs-sniffer firmware streams one JSON object
# per line at 460800 8N1 on its own /dev/cu.usbmodem* node (contract:
# elrs-sniffer/PROTOCOL.md, v0.2.x). Port discovery/claim is handled by
# the shared PortArbiter above (JSON "t"-line evidence); this thread only
# reads the claimed port, mirrors the main reader's reconnect/backoff
# shape, and never writes to the dongle or touches STATE/frame paths.

ELRS_MAX_EVENTS = 8           # last_events ring exposed via /api/state
ELRS_TLM_TYPES = ("gps", "batt", "atti", "fm", "tlm", "linkstats", "sync")
ELRS_WRITE_MIN_INTERVAL_S = 0.5  # console command rate limit (U/R/P/D/V)
ELRS_PHRASE_MAX = 64             # bind-phrase input cap


class ElrsState:
    """Latest ELRS link state + telemetry, mutated only by ElrsManager."""

    def __init__(self) -> None:
        self.lock = threading.Lock()
        self.connected = False
        self.port = None
        self.radio = 0
        self.link_lock = 0
        self.rate = "-"
        self.iq = "-"
        self.rssi = None
        self.rssi_now = None
        self.snr10 = None
        self.pps = 0
        self.rx_per_s = None
        self.lq_permille = 0
        self.uid = None
        self.arm = 0
        self.ch = [0, 0, 0, 0]
        self.freq = None        # hop-follow tuned Hz (optional, fhss mode)
        self.fhss = None        # hop sequence index; 255 = not following
        self.stats_time = None  # monotonic of the last stats line
        self.tlm = {}           # type -> (payload dict, monotonic)
        self.events = []        # newest-last [(monotonic, seq, payload)]
        self.event_seq = 0
        self.crack_state = None  # stats.crack mirror (keep-last)
        self.mode = None         # stats.mode: sweep|park|follow|discovery
        self.crack_ev = None     # (payload, monotonic) latest t=="crack" line
        self.uid_src = None      # uid event src: stored|set|default(-nvs-unavailable)
        self.uid_full = None     # spaced full UID from uid events
        self.uid_set_time = None  # monotonic of the last src=="set" uid event
        self.last_sync = None    # (payload, monotonic) latest t=="sync" line
        self.tails = {}          # uid tail hex -> {count, rssi, band, validated, ts}

    def update(self, obj: dict) -> None:
        now = time.monotonic()
        t = obj.get("t")
        with self.lock:
            if t == "stats":
                self.radio = obj.get("radio", 1)
                self.link_lock = obj.get("lock", 0)
                self.rate = obj.get("rate", "-")
                self.iq = obj.get("iq", "-")
                self.rssi = obj.get("rssi")
                self.rssi_now = obj.get("rssi_now", self.rssi_now)
                self.snr10 = obj.get("snr10")
                self.pps = obj.get("pps", 0)
                self.rx_per_s = obj.get("rx_per_s", self.rx_per_s)
                self.lq_permille = obj.get("lq_permille", 0)
                # Hop-follow fields are emitted only while following; keep
                # the last seen values otherwise (fhss 255 = not following).
                self.freq = obj.get("freq", self.freq)
                self.fhss = obj.get("fhss", self.fhss)
                if obj.get("uid"):
                    self.uid = obj["uid"]
                self.arm = obj.get("arm", 0)
                # Crack machine mirrors (dashboard contract): crack = last state
                # string, mode = sweep|park|follow|discovery. Keep-last so a lost crack
                # event line still leaves the panel stateable from stats alone.
                self.crack_state = obj.get("crack", self.crack_state)
                self.mode = obj.get("mode", self.mode)
                ch = obj.get("ch")
                if isinstance(ch, list) and len(ch) == 4:
                    self.ch = [int(c) for c in ch]
                self.stats_time = now
            else:
                if t == "crack":
                    # Keep-last full crack payload (state/uid_tail/done/
                    # valids_best[/uid_full]) — survives the events ring.
                    self.crack_ev = (obj, now)
                if t == "sync":
                    # Keep-last sync (band/len/freq + forward-compatible
                    # validated/model_id/tail_src pass through verbatim), and
                    # count the heard tail. FLRC syncs arrive ok:0 — that is
                    # NORMAL (structural validation, no ELRS CRC), never a
                    # failure; only a LoRa ok:0 means a CRC miss.
                    self.last_sync = (obj, now)
                    band = obj.get("band")
                    validated = obj.get("validated")
                    if validated is None:
                        if obj.get("ok"):
                            validated = "crc"
                        elif band == "flrc":
                            validated = "structural"
                    self._note_tail(obj.get("uid"),
                                    self.rssi_now if self.rssi_now is not None
                                    else self.rssi,
                                    band, validated, now)
                if t == "event" and obj.get("what") == "uid":
                    # Bind-phrase source: stored|set|default (dashboard hints +
                    # apply feedback). The phrase-derived tail identifies "our"
                    # link among heard neighbors before any air traffic.
                    self.uid_src = obj.get("src")
                    self.uid_full = obj.get("uid")
                    parts = str(obj.get("uid") or "").split()
                    if len(parts) == 6:
                        self.uid = "".join(parts[3:])
                    if obj.get("src") == "set":
                        self.uid_set_time = now
                if t == "event" and obj.get("what") == "flrc_sync":
                    # FLRC phrase-check line: uid_pkt is an air-heard tail
                    # (counted even when never adopted); tail_src says how the
                    # hearing was validated (structural | syncword+crc24).
                    self._note_tail(obj.get("uid_pkt"),
                                    self.rssi_now if self.rssi_now is not None
                                    else self.rssi,
                                    "flrc", obj.get("tail_src"), now)
                if t in ELRS_TLM_TYPES:
                    self.tlm[t] = (obj, now)
                    if t == "sync" and obj.get("uid"):
                        self.uid = obj["uid"]
                if t:
                    self.event_seq += 1
                    self.events.append((now, self.event_seq, obj))
                    del self.events[:-ELRS_MAX_EVENTS]

    def _note_tail(self, tail, rssi, band, validated, now: float) -> None:
        """Caller holds self.lock. One entry per heard UID tail (the
        link-identity bytes every sync leaks), capped at 16 by recency."""
        if not tail or not isinstance(tail, str):
            return
        ent = self.tails.get(tail)
        if ent is None:
            ent = {"count": 0, "rssi": None, "band": None,
                   "validated": None, "ts": 0.0}
            self.tails[tail] = ent
        ent["count"] += 1
        if isinstance(rssi, (int, float)):
            ent["rssi"] = rssi
        if band:
            ent["band"] = band
        if validated:
            ent["validated"] = validated
        ent["ts"] = now
        if len(self.tails) > 16:
            oldest = min(self.tails, key=lambda k: self.tails[k]["ts"])
            del self.tails[oldest]

    def snapshot(self) -> dict:
        now = time.monotonic()
        with self.lock:
            return {
                "connected": self.connected,
                "port": self.port,
                "radio": self.radio,
                "lock": self.link_lock,
                "rate": self.rate,
                "iq": self.iq,
                "rssi": self.rssi,
                "rssi_now": self.rssi_now,
                "snr": None if self.snr10 is None else round(self.snr10 / 10.0, 1),
                "pps": self.pps,
                "rx_per_s": self.rx_per_s,
                "lq": round(self.lq_permille / 10.0, 1),
                "uid": self.uid,
                "arm": self.arm,
                "ch": list(self.ch),
                "freq": self.freq,
                "fhss": self.fhss,
                "mode": self.mode,
                "crack_state": self.crack_state,
                "crack": (None if self.crack_ev is None else
                          {"age_s": round(now - self.crack_ev[1], 1),
                           **{k: v for k, v in self.crack_ev[0].items()
                              if k != "t"}}),
                "uid_src": self.uid_src,
                "uid_full": self.uid_full,
                "uid_set_age_s": (None if self.uid_set_time is None
                                  else round(now - self.uid_set_time, 1)),
                "last_sync": (None if self.last_sync is None else
                              {"age_s": round(now - self.last_sync[1], 1),
                               **{k: v for k, v in self.last_sync[0].items()
                                  if k != "t"}}),
                "tails": [{"tail": tail, "count": e["count"],
                           "rssi": e["rssi"], "band": e["band"],
                           "validated": e["validated"],
                           "own": tail == self.uid,
                           "age_s": round(now - e["ts"], 1)}
                          for tail, e in sorted(
                              self.tails.items(),
                              key=lambda kv: -kv[1]["ts"])[:8]],
                "stats_age_s": (None if self.stats_time is None
                                else round(now - self.stats_time, 1)),
                "tlm": {k: {"age_s": round(now - ts, 1),
                            **{kk: vv for kk, vv in v.items() if kk != "t"}}
                        for k, (v, ts) in self.tlm.items()},
                "last_events": [{"seq": seq, "age_s": round(now - ts, 1), **ev}
                                for ts, seq, ev in self.events[-ELRS_MAX_EVENTS:]],
            }


ELRS = ElrsState()


class ElrsManager(threading.Thread):
    """Reads the ELRS sniffer dongle on the port the arbiter classified
    as "elrs" (see the arbitration notes above)."""

    def __init__(self) -> None:
        super().__init__(daemon=True)
        self.ser = None
        self.stop_event = threading.Event()
        self.write_lock = threading.Lock()
        self._last_write = 0.0

    def send_line(self, line: str) -> bool:
        """Console command line (U <phrase>, R [step], P, D, V) to the
        sniffer dongle, rate-limited against itself; the reader thread keeps
        owning reads. Mirrors SerialManager._write's failure shape."""
        with self.write_lock:
            now = time.monotonic()
            if now - self._last_write < ELRS_WRITE_MIN_INTERVAL_S:
                return False
            try:
                if self.ser is None:
                    return False
                self.ser.write(line.encode("ascii", "ignore") + b"\n")
                self.ser.flush()
            except (serial.SerialException, OSError):
                return False
            self._last_write = now
            return True

    def run(self) -> None:
        backoff = 0.5
        buf = b""
        while not self.stop_event.is_set():
            try:
                if self.ser is None:
                    ser = ARBITER.claim("elrs")
                    if ser is None:
                        self.stop_event.wait(min(backoff, 3.0))
                        backoff = min(backoff * 2.0, 5.0)
                        continue
                    backoff = 0.5
                    buf = b""
                    ser.timeout = 0.2
                    self.ser = ser
                    with ELRS.lock:
                        ELRS.connected = True
                        ELRS.port = ser.port
                    print(f"[elrs] connected to {ser.port}")
                chunk = self.ser.read(1024)
                if not chunk:
                    continue
                buf += chunk
                while b"\n" in buf:
                    line, buf = buf.split(b"\n", 1)
                    line = line.strip()
                    # Protocol: skip any line that does not begin with '{'
                    # (boot banners, fault text); ignore unknown keys.
                    if not line.startswith(b"{"):
                        continue
                    try:
                        obj = json.loads(line)
                    except ValueError:
                        continue
                    if isinstance(obj, dict):
                        ELRS.update(obj)
                        _elrs_ep_tick(obj)
            except (serial.SerialException, OSError) as exc:
                print(f"[elrs] lost: {exc}; rediscovering in {backoff:.1f}s")
                if self.ser is not None:
                    ARBITER.release(self.ser.port)
                    try:
                        self.ser.close()
                    except OSError:
                        pass
                self.ser = None
                buf = b""
                with ELRS.lock:
                    ELRS.connected = False
                _elrs_ep_end("serial disconnect")
                self.stop_event.wait(backoff)
                backoff = min(backoff * 2.0, 5.0)

    def close(self) -> None:
        self.stop_event.set()
        if self.ser is not None:
            ARBITER.release(self.ser.port)
            try:
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
        # Canvas freshness is measured from the ROW bytes, not the snapshot
        # publishes: a partial stream snapshots every >=400 ms, which would
        # keep `age` forever young while the picture itself is frozen.
        rows_stale = bool(frame) and             (time.monotonic() - STATE.row_time) > ROWS_STALE_S
    # Placeholder only when genuinely disconnected or frameless for a long
    # stretch; as long as any frames flow (snow included), show the canvas.
    if frame is None or not connected or age > PLACEHOLDER_AFTER_S:
        return placeholder_jpeg()
    img = Image.frombytes("L", dims, frame)
    if rows_stale:
        # Old channel / gone drone: dim the picture and frame it amber so
        # "this is stale" is obvious at a glance (no flicker during normal
        # flow -- the marking appears once at the ROWS_STALE_S boundary).
        img = img.point(lambda p: (p * 3) // 4)
    img = img.resize((dims[0] * SCALE, dims[1] * SCALE), Image.NEAREST)
    if rows_stale:
        img = img.convert("RGB")
        draw = ImageDraw.Draw(img)
        draw.rectangle([0, 0, img.width - 1, img.height - 1],
                       outline=(255, 176, 0), width=SCALE)
    return make_jpeg(img)


def mjpeg_stream():
    # Push exactly one image per actual change: a new firmware frame or a
    # connection-state flip.  The placeholder is sent once and then the
    # stream goes quiet — a periodically re-sent placeholder makes the
    # browser re-decode and visibly flash.  Exception: while a REAL firmware
    # frame is the current image (received and not yet stale), a stalled
    # frame flow re-pushes that frame at ~1 Hz so the browser feed never
    # looks dead; the placeholder path stays push-once.
    #
    # Partial frames: reconnecting video whose frames never complete (no
    # bit0 row) must still surface -- if rows have arrived since the last
    # publish and ROW_SNAPSHOT_AFTER_S has passed, snapshot the canvas now.
    # The wait is short enough (250 ms) that the gate fires within one tick
    # of its deadline even when no further packets arrive.
    last_seq = -1
    last_conn = None
    last_push = 0.0
    while True:
        with STATE.frame_cond:
            if STATE.frame_seq == last_seq and STATE.connected == last_conn:
                STATE.frame_cond.wait(timeout=0.25)
            changed = (STATE.frame_seq != last_seq or
                       STATE.connected != last_conn)
            now = time.monotonic()
            if not changed and STATE.connected and                     STATE.row_time > STATE.frame_time and                     now - STATE.frame_time >= ROW_SNAPSHOT_AFTER_S:
                STATE.publish_snapshot()
                changed = True
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
  .hopchip { display:inline-block; min-width:46px; text-align:center;
             padding:0 5px; border-radius:3px; border:1px solid var(--dim);
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
  #gwrap { flex:1 1 0; min-width:0; position:relative;
           border-top:1px solid var(--dim);
           background:#071007;
           background-image:repeating-linear-gradient(90deg,rgba(57,255,106,.04) 0 1px,transparent 1px 24px),
                            repeating-linear-gradient(0deg,rgba(57,255,106,.04) 0 1px,transparent 1px 24px); }
  #swrap { flex:1 1 0; min-width:0; position:relative;
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
  /* ELRS card (third bottom card) + ELRS tab: same grid backdrop, dense. */
  #ewrap { flex:1 1 0; min-width:0; position:relative; overflow:hidden;
           border-top:1px solid var(--dim); border-left:1px solid var(--dim);
           background:#071007; padding:24px 12px 6px;
           display:flex; flex-direction:column; gap:4px;
           background-image:repeating-linear-gradient(90deg,rgba(57,255,106,.04) 0 1px,transparent 1px 24px),
                            repeating-linear-gradient(0deg,rgba(57,255,106,.04) 0 1px,transparent 1px 24px); }
  #etitle { position:absolute; top:7px; left:14px; font-size:.62rem;
            letter-spacing:.18em; color:var(--dim); pointer-events:none; }
  #edot, #xedot { display:inline-block; width:8px; height:8px; border-radius:50%;
          background:#4a5a4a; vertical-align:1px; }
  #edot.lock, #xedot.lock { background:var(--grn); box-shadow:0 0 8px rgba(57,255,106,.8); }
  #edot.sweep, #xedot.sweep { background:var(--amb); box-shadow:0 0 8px rgba(255,176,0,.7);
                animation:pulse 1s infinite; }
  #ebig { display:flex; gap:14px; align-items:baseline; }
  .eb .ek, .egauge .ek { color:var(--dim); font-size:.56rem; letter-spacing:.1em; }
  .eb b { font-size:1.3rem; color:var(--txt); margin:0 3px; font-variant-numeric:tabular-nums; }
  .eb .eu, .egauge .eu { color:var(--dim); font-size:.54rem; }
  #erate { font-size:.6rem; color:var(--txt); letter-spacing:.06em; flex:0 0 14px; height:14px; white-space:nowrap;
         overflow:hidden; text-overflow:ellipsis;
         font-variant-numeric:tabular-nums; }
  #eintel { font-size:.6rem; color:var(--amb); letter-spacing:.04em;
            white-space:nowrap; overflow:hidden; text-overflow:ellipsis;
            flex:0 0 14px; height:14px; font-variant-numeric:tabular-nums; }
  #euid { display:flex; gap:6px; align-items:center; align-self:flex-start;
          flex:0 0 18px; height:18px; overflow:hidden; }
  .armb { color:#fff; background:var(--red); border-radius:3px; padding:0 6px;
          font-size:.56rem; letter-spacing:.1em; animation:pulse 1s infinite; }
  .armb.sm { font-size:.5rem; padding:0 4px; }
  #etick { flex:1 1 0; min-height:0; overflow:hidden; font-size:.54rem;
           line-height:1.5; color:var(--dim); }
  #etick .evl { white-space:nowrap; overflow:hidden; text-overflow:ellipsis; }
  .sticks { display:flex; gap:6px; }
  .stk { flex:1 1 0; min-width:0; }
  .stk i { font-style:normal; color:var(--dim); font-size:.5rem;
           letter-spacing:.08em; }
  .stk .stkbar { position:relative; height:8px; background:#0d160d;
                 border:1px solid var(--dim); border-radius:2px; }
  .stk .stkbar::after { content:""; position:absolute; left:50%; top:0;
                        bottom:0; width:1px; background:var(--dim); }
  .stkfill { position:absolute; top:1px; bottom:1px; background:var(--grn);
             border-radius:1px; }
  .stk.dim .stkfill { background:#4a5a4a; }
  .stk.dim i, .stk.dim b { opacity:.5; }
  .stk b { font-size:.5rem; color:var(--txt); font-weight:normal;
           font-variant-numeric:tabular-nums; }
  .evl .etag { display:inline-block; min-width:34px; margin-right:5px; }
  /* Event color code: sync/flrc_sync cyan, lock green, unlock/linkstats
     amber, aircraft telemetry white, OSINT events magenta, errors red. */
  .evl.k-sync .etag, .evl.k-flrc_sync .etag,
  .evl.k-modelId .etag { color:#31c8ff; }
  .evl.k-sync.flrc .etag { color:#ff3ac8; }  /* structural FLRC sync = OSINT */
  .evl.k-lock .etag { color:var(--grn); }
  .evl.k-unlock .etag, .evl.k-linkstats .etag { color:var(--amb); }
  .evl.k-gps .etag, .evl.k-batt .etag, .evl.k-atti .etag,
  .evl.k-fm .etag { color:var(--txt); }
  .evl.k-fp .etag, .evl.k-uid_cracked .etag, .evl.k-uid2_crack .etag,
  .evl.k-uid2_crack_failed .etag, .evl.k-crack .etag { color:#ff3ac8; }
  .evl.k-error .etag { color:var(--red); }
  .evl .eage { color:var(--dim); margin-left:5px; }
  /* ELRS tab: gauges on top, sparkline middle, intel + log bottom. */
  #view-elrs { flex:1 1 0; min-height:0; display:none; flex-direction:column;
               background:var(--panel); border:1px solid var(--dim);
               border-radius:6px; overflow:hidden;
               background-image:repeating-linear-gradient(90deg,rgba(57,255,106,.03) 0 1px,transparent 1px 24px),
                                repeating-linear-gradient(0deg,rgba(57,255,106,.03) 0 1px,transparent 1px 24px); }
  .etop { flex:0 0 96px; height:96px; display:flex; align-items:center; gap:22px;
          flex-wrap:nowrap; overflow:hidden; padding:0 18px;
          border-bottom:1px solid var(--dim); }
  .egauge { flex:0 0 84px; text-align:center; overflow:hidden; }
  .egauge b { display:block; font-size:1.9rem; color:var(--txt);
              font-variant-numeric:tabular-nums; }
  .gsub { display:block; font-style:normal; font-size:.54rem; color:var(--dim);
          letter-spacing:.06em; min-height:.7rem; white-space:nowrap;
          overflow:hidden; font-variant-numeric:tabular-nums; }
  /* ELRS tab link header: state dot, rate/iq, lock badge, ARMED tag, UID
     alias chip, hop-follow indicator. FIXED height; conditional items keep
     their slots via visibility + reserved widths (never display:none), so
     badges appearing never push neighbors. */
  #elinkhdr { flex:0 0 34px; height:34px; display:flex; align-items:center;
              gap:12px; flex-wrap:nowrap; overflow:hidden; padding:0 18px;
              font-size:.66rem; letter-spacing:.08em;
              border-bottom:1px solid var(--dim);
              font-variant-numeric:tabular-nums; }
  #elinkhdr .esp { flex:1 1 auto; }
  #xehinfo { color:var(--txt); white-space:nowrap; }
  #xeuidwrap { flex:0 0 auto; min-width:110px; min-height:18px;
               text-align:right; }
  #xearm { flex:0 0 54px; box-sizing:border-box; text-align:center; }
  .ebadge { padding:0 8px; border-radius:3px; border:1px solid var(--dim);
            color:var(--dim); font-size:.58rem; letter-spacing:.12em; }
  .ebadge.lock { color:var(--grn); border-color:var(--grn);
                 text-shadow:0 0 6px rgba(57,255,106,.6); }
  .ebadge.sweep { color:var(--amb); border-color:var(--amb); }
  .followb { flex:0 0 auto; min-width:196px; box-sizing:border-box;
             display:inline-block; text-align:center;
             padding:0 8px; border-radius:3px; border:1px solid #31c8ff;
             color:#31c8ff; font-size:.58rem; letter-spacing:.1em; }
  #xestickwrap { flex:1 1 220px; max-width:420px; position:relative; }
  /* Crack panel: the FLRC acquisition pipeline as stage chips between the
     link header and the gauges. FIXED-height strip that always reserves its
     slot (the panel itself is visibility-toggled, never display:none); every
     chip has a fixed width + reserved sub-line, so state changes, progress
     numbers and the CRACKED/FAILED swap never reflow the tab. Passed stages
     solid lit, current stage pulses, future stages dim. */
  #ecrack { flex:0 0 40px; height:40px; display:flex; align-items:center;
            gap:8px; flex-wrap:nowrap; overflow:hidden; padding:0 18px;
            box-sizing:border-box; border-bottom:1px solid var(--dim);
            font-size:.58rem; letter-spacing:.1em; white-space:nowrap;
            font-variant-numeric:tabular-nums; }
  .ckst { flex:0 0 auto; height:30px; box-sizing:border-box; overflow:hidden;
          display:inline-flex; flex-direction:column; justify-content:center;
          padding:2px 8px; border:1px solid var(--dim); border-radius:3px;
          color:var(--dim); text-align:center; }
  .ckst b { font-weight:normal; letter-spacing:.14em; }
  .ckst i { display:block; font-style:normal; font-size:.5rem; color:var(--dim);
            letter-spacing:.06em; min-height:.66rem; line-height:.66rem;
            overflow:hidden; text-overflow:ellipsis; }
  #cks-sweep { width:86px; } #cks-sync { width:110px; }
  #cks-ident { width:96px; } #cks-crack { width:186px; }
  #cks-done { width:220px; }
  .ckarr { flex:0 0 auto; color:var(--dim); }
  .ckst.pass { color:var(--grn); border-color:var(--grn); opacity:.7; }
  .ckst.on { color:var(--grn); border-color:var(--grn);
             text-shadow:0 0 6px rgba(57,255,106,.6);
             animation:ckpulse 1.1s ease-in-out infinite; }
  .ckst.on.ok { animation:none; }               /* CRACKED holds solid, machine done */
  .ckst.on.fail { color:var(--amb); border-color:var(--amb);
                  text-shadow:0 0 6px rgba(255,176,32,.6); }
  @keyframes ckpulse { 50% { opacity:.45; } }
  .ckbar { display:block; height:3px; margin-top:2px; background:rgba(51,80,47,.5);
           border-radius:2px; overflow:hidden; }
  #ckfill { display:block; height:100%; width:0%; background:var(--grn); }
  #ckdone-sub b { font-weight:normal; color:var(--grn); }
  /* Tails-heard + bind-phrase strip: per-tail chips (count, last RSSI,
     validated state) on the left, uid-src hint + phrase apply on the right.
     FIXED height; the hint is visibility-toggled, chips refill in place, so
     the strip never reflows the tab. */
  #etails { flex:0 0 34px; height:34px; display:flex; align-items:center;
            gap:8px; flex-wrap:nowrap; overflow:hidden; padding:0 18px;
            box-sizing:border-box; border-bottom:1px solid var(--dim);
            font-size:.58rem; letter-spacing:.08em; white-space:nowrap;
            font-variant-numeric:tabular-nums; }
  #etails .tlbl { flex:0 0 auto; color:var(--dim); letter-spacing:.14em; }
  #etailchips { flex:1 1 auto; min-width:0; display:flex; gap:6px;
                align-items:center; overflow:hidden; }
  .tchip { flex:0 0 auto; padding:1px 6px; border:1px solid var(--dim);
           border-radius:3px; color:var(--txt); }
  .tchip b { color:var(--grn); font-weight:normal; }
  .tchip.own { border-color:var(--grn);
               box-shadow:0 0 6px rgba(57,255,106,.25); }
  .tchip .tcnt, .tchip .trssi, .tchip .tband { color:var(--dim); }
  .tchip .tval { color:#31c8ff; }
  .tchip .tval.na { color:var(--dim); }
  #euidhint { flex:0 0 auto; min-width:190px; text-align:right;
              color:var(--amb); visibility:hidden; overflow:hidden;
              text-overflow:ellipsis; }
  #euidhint.ok { color:var(--grn); }
  #ephrase { flex:0 0 170px; width:170px; font:inherit; font-size:.6rem;
             letter-spacing:.06em; background:var(--panel); color:var(--txt);
             border:1px solid var(--dim); border-radius:3px; padding:2px 6px; }
  #ephrase:focus { border-color:var(--grn); outline:none; }
  #ephrasebtn { flex:0 0 auto; height:22px; font-size:.56rem; padding:0 10px; }
  .staletag { position:absolute; top:-7px; right:0; font-size:.5rem;
              letter-spacing:.14em; color:var(--amb); border:1px solid var(--amb);
              border-radius:3px; padding:0 4px; background:var(--panel); }
  .sticks.big { width:100%; }
  .sticks.big .stk .stkbar { height:14px; }
  .sticks.big .stk i, .sticks.big .stk b { font-size:.56rem; }
  .emid { flex:1 1 0; min-height:80px; position:relative;
          border-bottom:1px solid var(--dim); }
  #espark { position:absolute; inset:0; width:100%; height:100%; }
  .esptitle { position:absolute; top:6px; left:14px; font-size:.58rem;
              letter-spacing:.16em; color:var(--dim); pointer-events:none; }
  /* Telemetry grid: label:value cards (GPS/batt/atti/fm + linkstats), each
     with its value age. ALL cards always render (dim — placeholder when
     empty) at a FIXED height; the note slot is always reserved
     (visibility-toggled), so arrivals never reflow the grid. Stacks 2-col
     on mobile (media query below). */
  .etlm { flex:0 0 auto; display:grid; grid-template-columns:repeat(3,1fr);
          gap:8px; padding:8px 14px; border-bottom:1px solid var(--dim); }
  .etcard { min-width:0; height:56px; overflow:hidden;
            border:1px solid rgba(51,80,47,.5); border-radius:5px; padding:5px 10px;
            box-sizing:border-box;
            background:rgba(57,255,106,.03); font-size:.6rem; line-height:1.5; }
  .etcard .etk { color:var(--dim); font-size:.56rem; letter-spacing:.12em; }
  .etcard .etv { color:var(--txt); font-size:.66rem; letter-spacing:.04em; margin-top:1px;
               white-space:nowrap; overflow:hidden; text-overflow:ellipsis;
               font-variant-numeric:tabular-nums; }
  .etcard .etv b { color:var(--grn); }
  .etcard .etna { color:var(--dim); }
  .etcard .etage { color:var(--dim); margin-left:6px; letter-spacing:.04em;
                   font-size:.52rem; }
  .etcard .etsub { color:var(--dim); font-size:.52rem; letter-spacing:.04em;
                   white-space:nowrap; overflow:hidden; text-overflow:ellipsis; }
  .ecopy { font:inherit; font-size:.56rem; padding:0 4px; margin-left:4px;
           background:var(--panel); color:var(--dim);
           border:1px solid var(--dim); border-radius:3px; cursor:pointer;
           vertical-align:1px; }
  .ecopy:active { color:var(--grn); border-color:var(--grn); }
  #etnote { grid-column:1 / -1; height:26px; box-sizing:border-box; overflow:hidden;
          visibility:hidden; white-space:nowrap; text-overflow:ellipsis;
          border:1px solid var(--amb); border-radius:5px;
          padding:4px 10px; color:var(--amb); font-size:.62rem;
          letter-spacing:.06em; }
  #etnote.sweep { border-color:var(--dim); color:var(--dim); }
  /* Event log + encounters: FIXED-height row, each pane scrolls internally;
     entry count never changes the container. */
  .ebot { flex:0 0 30%; min-height:110px; display:flex;
          border-top:none; }
  #elog { flex:1 1 0; min-width:0; padding:8px 14px; font-size:.58rem;
          line-height:1.6; overflow-y:auto; }
  #elog .evl { white-space:nowrap; overflow:hidden; text-overflow:ellipsis; }
  #elog .ets { color:var(--dim); margin-right:6px; }
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
  #elrschip { font-size:.52rem; letter-spacing:.08em; color:var(--dim);
              border:1px solid var(--dim); border-radius:3px; padding:0 5px;
              display:inline-block; min-width:44px; text-align:center; }
  #elrschip.sweep { color:var(--amb); border-color:var(--amb); }
  #elrschip.lock { color:var(--grn); border-color:var(--grn); }
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
  #dettable tr.nz { opacity:.45; }
  .detcard.nz { opacity:.55; }
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
           border:1px solid; font-size:.56rem; letter-spacing:.06em;
           cursor:pointer; }
  .dchip:hover::after { content:'\270E'; margin-left:3px; opacity:.65; }
  .dchip.aliased { font-weight:bold; }
  .dedit { width:90px; background:#0d160d; color:var(--txt);
           border:1px solid var(--grn); border-radius:3px;
           font:inherit; font-size:.56rem; padding:0 3px; }
  .stdb { padding:0 3px; border-radius:3px; border:1px solid var(--dim);
          color:var(--txt); font-size:.54rem; letter-spacing:.08em; }
  .ltb { padding:0 4px; border-radius:3px; border:1px solid var(--dim);
         color:var(--dim); font-size:.56rem; letter-spacing:.05em; }
  .ltb.scanner { color:var(--grn); border-color:var(--grn); }
  .ltb.manual { color:#31c8ff; border-color:#31c8ff; }
  .ltb.event { color:var(--amb); border-color:var(--amb); }
  .ltb.elrs { color:#31f5ff; border-color:#31f5ff; }
  .bchip.b24 { color:#31f5ff; border-color:#31f5ff; }
  .bchip.b58 { color:var(--grn); border-color:var(--grn); }
  .newmark { color:var(--amb); margin-right:2px; }
  #geobtn.on { color:var(--grn); border-color:var(--grn); }
  #eenc { flex:1 1 0; min-width:0; padding:8px 14px; font-size:.58rem;
          line-height:1.6; overflow:auto; border-right:1px solid var(--dim); }
  #eenc .eenc-h { color:var(--dim); letter-spacing:.14em; margin-bottom:3px; }
  #eenc .een { white-space:nowrap; overflow:hidden; text-overflow:ellipsis; }
  #eenc .een .ets { color:var(--dim); margin-right:4px; }
  #eenc .een.open { color:var(--grn); }
  #eenc .euid { color:#31f5ff; }
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
    .botsplit { flex:0 0 auto; flex-direction:row; height:34dvh;
                min-height:0; margin-top:6px; overflow-x:auto;
                scroll-snap-type:x mandatory; }
    #gwrap, #swrap, #ewrap { flex:0 0 86%; min-height:0;
                             scroll-snap-align:center; }
    #swrap { border-left:none; }
    .etop { gap:12px; padding:0 12px; }
    .egauge { flex:0 0 62px; }
    .egauge b { font-size:1.3rem; }
    .etlm { grid-template-columns:repeat(2,1fr); gap:6px; padding:6px 10px; }
    #elinkhdr { gap:8px; padding:0 12px; font-size:.6rem; }
    .followb { min-width:160px; }
    #ecrack { gap:5px; padding:0 12px; font-size:.5rem; }
    #ecrack .ckst { padding:2px 5px; }
    #cks-sweep { width:64px; } #cks-sync { width:84px; }
    #cks-ident { width:72px; } #cks-crack { width:140px; }
    #cks-done { width:160px; }
    #etails { gap:6px; padding:0 12px; font-size:.5rem; }
    #etails .tlbl { display:none; }
    #euidhint { min-width:110px; }
    #ephrase { flex:1 1 90px; width:90px; min-width:0; }
    .ebot { flex:0 0 220px; flex-direction:column; overflow:auto; }
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
    .botsplit { flex-direction:row; height:32dvh; overflow-x:visible; }
    #gwrap, #swrap, #ewrap { flex:1 1 0; }
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
  <button id="tab-elrs" class="tab" onclick="showTab('elrs')">ELRS <span id="elrschip" class="off">--</span></button>
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
<div id="ewrap">
  <div id="etitle">ELRS 2.4G <span id="edot"></span></div>
  <div id="ebig">
    <div class="eb"><span class="ek">RSSI</span><b id="erssi">--</b><span class="eu">dBm</span></div>
    <div class="eb"><span class="ek">LQ</span><b id="elq">--</b><span class="eu">%</span></div>
  </div>
  <div id="erate">--</div>
  <div id="esticks" class="sticks"></div>
  <div id="eintel">--</div>
  <div id="euid" style="visibility:hidden"></div>
  <div id="etick"></div>
</div>
</div>
</div>
<div id="view-elrs">
  <div id="elinkhdr">
    <span id="xedot"></span>
    <span id="xehinfo">--</span>
    <span id="xelockb" class="ebadge">--</span>
    <span class="esp"></span>
    <span id="xeuidwrap"></span>
    <span id="xearm" class="armb" style="visibility:hidden">ARMED</span>
    <span id="xefollow" class="followb" style="visibility:hidden"></span>
  </div>
  <div id="ecrack" style="visibility:hidden">
    <span class="ckst" id="cks-sweep"><b>SWEEP</b><i id="cksweep-sub"></i></span>
    <span class="ckarr">&#8250;</span>
    <span class="ckst" id="cks-sync"><b>SYNC</b><i id="cktail"></i></span>
    <span class="ckarr">&#8250;</span>
    <span class="ckst" id="cks-ident"><b>IDENTITY</b><i id="ckident-sub"></i></span>
    <span class="ckarr">&#8250;</span>
    <span class="ckst" id="cks-crack"><b>CRACKING</b><i id="ckprog"></i><span class="ckbar"><span id="ckfill"></span></span></span>
    <span class="ckarr">&#8250;</span>
    <span class="ckst" id="cks-done"><b id="ckdone-lbl">CRACKED</b><i id="ckdone-sub"></i></span>
  </div>
  <div id="etails">
    <span class="tlbl">TAILS</span>
    <span id="etailchips"></span>
    <span id="euidhint"></span>
    <input id="ephrase" maxlength="64" placeholder="bind phrase"
           spellcheck="false" autocomplete="off">
    <button id="ephrasebtn" class="tg"
            title="send 'U &lt;phrase&gt;' to the sniffer dongle (persisted in its flash; it answers with a uid event)">APPLY</button>
  </div>
  <div class="etop">
    <div class="egauge"><span class="ek">RSSI</span><b id="xerssi">--</b><span class="eu">dBm</span><i class="gsub" id="xerssinow"></i></div>
    <div class="egauge"><span class="ek">LQ</span><b id="xelq">--</b><span class="eu">%</span></div>
    <div class="egauge"><span class="ek">SNR</span><b id="xesnr">--</b><span class="eu">dB</span></div>
    <div class="egauge"><span class="ek">PPS</span><b id="xepps">--</b><span class="eu">pkt/s</span><i class="gsub" id="xerx"></i></div>
    <div id="xestickwrap">
      <div id="xesticks" class="sticks big"></div>
      <div id="xestale" class="staletag" style="visibility:hidden">STALE</div>
    </div>
  </div>
  <div class="emid"><canvas id="espark"></canvas><div class="esptitle">RSSI / LQ // 60S</div></div>
  <div class="etlm">
    <div class="etcard" id="etgps"></div>
    <div class="etcard" id="etbatt"></div>
    <div class="etcard" id="etatti"></div>
    <div class="etcard" id="etfm"></div>
    <div class="etcard" id="etls"></div>
    <div id="etnote"></div>
  </div>
  <div class="ebot">
    <div id="eenc"></div>
    <div id="elog"></div>
  </div>
</div>
<div id="view-dets">
  <div class="detbar">
    <span class="lbl">DETECTION LOG // tools/detections.csv</span>
    <span id="dettotal">0 ROWS</span>
    <span id="detsum"></span>
    <button id="geobtn" class="tg"
            title="share this browser's geolocation: the fix rides state polls (X-Geo header) and is stamped on detection rows as geo_lat/geo_lon">&#128205; OFF</button>
    <button id="detnoise" class="tg" style="display:none"
            title="blip locks (under 2s, no video sync, under 15 dB) are hidden; the raw CSV keeps everything">SHOW NOISE</button>
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

/* Tabs: LIVE dashboard, ELRS link page, DETECTIONS log. No reload; every
   data source (main board poll, detections, ELRS via /api/state) keeps
   updating on the 500 ms cadence regardless of the active tab — switching
   only changes what is RENDERED (canvas work is skipped for hidden tabs).
   Active tab persists in localStorage. */
let tabCur = 'live';
function showTab(t) {
  tabCur = (t === 'elrs' || t === 'dets') ? t : 'live';
  lsSet('c5_tab', tabCur);
  document.getElementById('view-live').style.display = tabCur === 'live' ? '' : 'none';
  document.getElementById('view-elrs').style.display = tabCur === 'elrs' ? 'flex' : 'none';
  document.getElementById('view-dets').style.display = tabCur === 'dets' ? 'flex' : 'none';
  ['live', 'elrs', 'dets'].forEach(x =>
    document.getElementById('tab-' + x).classList.toggle('on', x === tabCur));
}
const DETCOLS = [
  ['band', 'RF'],
  ['start_iso', 'START'], ['end_iso', 'END'], ['duration_s', 'DUR s'],
  ['channel', 'CH'], ['drone', 'DRONE'], ['geo', 'GEO'], ['subband', 'BAND'],
  ['freq_mhz', 'FREQ'],
  ['cfo_ppm', 'CFO'], ['video_std', 'STD'], ['line_us', 'LINE'], ['uid', 'UID'],
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
/* ELRS encounter rows show the link rate (e.g. "250Hz/i") as their CH
   value, in cyan, tooltip-marked as the ELRS link rather than a channel. */
const echip = name =>
  '<span class="bchip b24" title="ELRS link">' + esc(name) + '</span>';
const chCell = r => r.band === '2.4' ? echip(r.channel)
                                     : bchip(r.subband, canonCh(r.channel));
/* Canonical display name for carriers the grid knows under two channels
   (A1 5865 vs B8 5866 are 1 MHz apart): within 2 MHz of another Boscam
   grid channel, the alphabetically-first name wins, so the same carrier
   reads consistently. Display-only — the logged channel is untouched;
   names outside the grid pass through. */
const CANON = {};
CH.forEach(row => row.forEach(c => {
  let best = c.n;
  CH.forEach(r2 => r2.forEach(d => {
    if (Math.abs(d.f - c.f) <= 2 && d.n < best) best = d.n;
  }));
  CANON[c.n] = best;
}));
const canonCh = name => CANON[name] || name;
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
/* DRONE-ID clustering: ELRS rows fingerprint on the link UID
   ("elrs:<uid>"); analog rows on line_us rounded to the nearest 0.02 us +
   "|" + video_std ('' when the firmware left it blank). The line period
   comes from the camera crystal and is stable per device — unlike cfo_ppm,
   which bounces with video content and stays display-only. Rounds via
   integer cents (x100, snap to even) to dodge binary-float ties. Analog
   rows with a blank/non-numeric line_us get no fingerprint, no ID. */
const droneFp = r => {
  if (r.uid !== undefined && r.uid !== null && r.uid !== '')
    return 'elrs:' + String(r.uid).toLowerCase();
  if (r.line_us === '' || r.line_us === undefined || r.line_us === null)
    return null;
  const us = Number(r.line_us);
  if (!isFinite(us) || us <= 0) return null;
  const cents = Math.round(Math.round(us * 100) / 2) * 2;
  return (cents / 100).toFixed(2) + '|' + (r.video_std || '');
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
/* Stable per-fingerprint hue so a drone keeps its chip color even when
   renamed (the alias is display-only; the fingerprint is the identity). */
const droneHue = fp => {
  let h = 0;
  for (let i = 0; i < fp.length; i++) h = (h * 31 + fp.charCodeAt(i)) >>> 0;
  return h % 360;
};
let detAliases = {};                   // fingerprint -> user alias (sidecar)
let detLast = null;                    // last /api/detections payload
let showNoise = false;                 // noise rows hidden by default
/* A row is NOISE when it looks like a blip lock: under 2 s, no video
   sync, peak under 15 dB. ELRS rows are exempt — link RSSI is a negative
   dBm peak, so the rule would hide every encounter. Hidden rows don't
   count in the counter or the summary; the CSV/export keep everything
   (raw data). */
const isNoise = r => r.band !== '2.4' && Number(r.duration_s || 0) < 2 &&
                     !r.video_sync && Number(r.level_peak_db || 0) < 15;
const droneChip = (fp, id, isNew) => {
  const h = droneHue(fp), al = detAliases[fp];
  return '<span class="dchip' + (al ? ' aliased' : '') + '" data-fp="' + esc(fp) + '"' +
    (al ? ' title="' + esc(id) + ' (' + esc(fp) + ')"' : '') +
    ' style="color:hsl(' + h + ',80%,62%);' +
    'border-color:hsl(' + h + ',80%,40%)">' +
    (isNew ? '<span class="newmark" title="first seen in the last 24 h">✦</span>' : '') +
    esc(al || id) + '</span>';
};
const fmtCfo = v => (v > 0 ? '+' : '') + Number(v).toFixed(1);
const fmtLine = v => (typeof v === 'number' ? v.toFixed(2) : esc(v)) + 'us';
/* Header summary strip: "N drones fingerprinted · M encounters ·
   strongest: DRONE-X (peak dB)" — aliased drones show the alias with the
   fingerprint's line period instead ("strongest: MY RIG (63.98us)"), and
   when both RF bands are present a per-band encounter count is appended
   ("· 3× 5.8 · 2× ELRS"). Counts only the rows passed in (noise-filtered
   view). */
function renderDetSum(rows) {
  const seen = {};
  rows.forEach(r => { if (r.drone) seen[r.drone] = 1; });
  const ids = Object.keys(seen).length;
  const enc = rows.reduce((n, r) => n + (r.drone ? 1 : 0), 0);
  let best = null;
  rows.forEach(r => {
    if (r.drone && typeof r.level_peak_db === 'number' &&
        (!best || r.level_peak_db > best.peak))
      best = { id: r.drone, fp: r.dronefp, peak: r.level_peak_db };
  });
  let strong = '';
  if (best) {
    const al = detAliases[best.fp];
    strong = al ? ' · strongest: ' + al + ' (' + best.fp.split('|')[0] + 'us)'
                : ' · strongest: ' + best.id + ' (' + best.peak + 'dB)';
  }
  const n24 = rows.reduce((n, r) => n + (r.band === '2.4' ? 1 : 0), 0);
  const bands = n24 ? ' · ' + (rows.length - n24) + '× 5.8 · ' + n24 + '× ELRS'
                    : '';
  document.getElementById('detsum').textContent =
    ids + ' drone' + (ids === 1 ? '' : 's') + ' fingerprinted · ' +
    enc + ' encounter' + (enc === 1 ? '' : 's') + strong + bands;
}
/* Inline rename: swap a drone chip for a small input; Enter saves (empty
   clears the alias), Esc cancels. All rows of the fingerprint update
   together on the re-render. */
function droneRename(chip) {
  const fp = chip.getAttribute('data-fp');
  if (!fp) return;
  const inp = document.createElement('input');
  inp.className = 'dedit';
  inp.value = detAliases[fp] || chip.textContent;
  inp.maxLength = 40;
  chip.replaceWith(inp);
  inp.focus(); inp.select();
  let done = false;
  const finish = save => {
    if (done) return;
    done = true;
    if (!save) { if (detLast) renderDets(detLast); return; }
    const alias = inp.value.trim();
    fetch('/api/alias', { method: 'POST',
                          headers: { 'Content-Type': 'application/json' },
                          body: JSON.stringify({ fp, alias }) })
      .then(r => r.json()).then(j => {
        if (j.ok) {
          detAliases = j.aliases || {};
          if (detLast) detLast.aliases = detAliases;
        }
        toast(j.ok ? (alias ? 'alias saved' : 'alias cleared') : 'alias rejected');
        if (detLast) renderDets(detLast);
      })
      .catch(() => { toast('alias save failed'); if (detLast) renderDets(detLast); });
  };
  inp.addEventListener('keydown', e => {
    if (e.key === 'Enter') finish(true);
    else if (e.key === 'Escape') finish(false);
  });
  inp.addEventListener('blur', () => finish(true));
}
/* Match a detection row to its screenshots: capture names are
   <sanitized episode start>__<tag>.jpg, sanitized the same way
   (YYYY-MM-DDTHH:MM:SS -> dashes). A row can have several (auto-shot,
   manual shots, best-OSD frame). */
const shotsFor = (r, shots) => {
  if (typeof r.start_iso !== 'string' || r.start_iso.length < 19) return [];
  const san = r.start_iso.slice(0, 19).replace(/:/g, '-');
  return shots.filter(n => n.startsWith(san + '__'));
};
function showShot(src) {
  document.getElementById('lbimg').src = src;
  document.getElementById('lightbox').style.display = 'flex';
}
let detSig = '';                       // column signature of the built header
function renderDets(d) {
  const rows = d.rows || [];
  const shots = d.shots || [];
  detLast = d;
  detAliases = d.aliases || {};
  // Noise filter: blip locks hidden by default; the header chip reveals
  // them (dimmed). Hidden rows count neither in the counter nor the
  // summary. The CSV/export always keep everything.
  const noiseN = rows.reduce((n, r) => n + (isNoise(r) ? 1 : 0), 0);
  const view = showNoise ? rows : rows.filter(r => !isNoise(r));
  const nzBtn = document.getElementById('detnoise');
  nzBtn.style.display = noiseN ? '' : 'none';
  nzBtn.textContent = (showNoise ? 'HIDE NOISE (' : 'SHOW NOISE (') + noiseN + ')';
  document.getElementById('dettotal').textContent = view.length + ' ROWS';
  // Cluster ALL rows (noise included) so drone IDs stay stable when the
  // noise toggle flips, then summarize the visible view only. Also stamp
  // per-row: droneNew (cluster first seen <24 h ago -> ✦ on the chip) and
  // the virtual geo cell (📍 when the row carries a browser fix).
  const dm = droneMap(rows);
  const firstSeen = {};
  rows.forEach(r => {
    const fp = droneFp(r);
    if (fp) {
      r.drone = dm[fp]; r.dronefp = fp;
      const t = String(r.start_iso || '');
      if (!(fp in firstSeen) || t < firstSeen[fp]) firstSeen[fp] = t;
    }
  });
  const nowMs = Date.now();
  rows.forEach(r => {
    if (r.dronefp) {
      const t0 = Date.parse(firstSeen[r.dronefp] || '');
      r.droneNew = isFinite(t0) && nowMs - t0 < 86400000;
    }
    if (typeof r.geo_lat === 'number' && typeof r.geo_lon === 'number')
      r.geo = '📍';
  });
  renderDetSum(view);
  // Skip columns the CSV lacks entirely (or that are blank on every row).
  const cols = DETCOLS.filter(c => view.some(r => r[c[0]] !== undefined && r[c[0]] !== ''));
  if (view.some(r => typeof r.level_peak_db === 'number')) {
    const li = cols.findIndex(c => c[0] === 'level_min_db');
    cols.splice(li >= 0 ? li + 1 : cols.length, 0, ['spark', 'LEVEL']);
  }
  if (view.some(r => shotsFor(r, shots).length)) cols.push(['shot', 'SHOT']);
  if (window.innerWidth <= 899) { renderDetCards(view, cols, shots); return; }
  const sig = cols.map(c => c[0]).join(',');
  if (sig !== detSig) {
    detSig = sig;
    document.getElementById('dethead').innerHTML =
      '<tr>' + cols.map(c => '<th' + (c[0] === 'spark' ? ' class="sps"' : '') +
        (c[0] === 'cfo_ppm' ? ' title="per-measurement carrier offset — drifts ' +
                             'with video content; NOT a fingerprint"' : '') + '>' +
                             c[1] + '</th>').join('') + '</tr>';
  }
  document.getElementById('detbody').innerHTML = view.map(r => {
    let rcls = detRowCls(r);
    if (showNoise && isNoise(r)) rcls += (rcls ? ' ' : '') + 'nz';
    return '<tr' + (rcls ? ' class="' + rcls + '"' : '') + '>' + cols.map(c => {
      const k = c[0], v = r[k];
      if (k === 'shot') {
        const nms = shotsFor(r, shots);
        return nms.length
          ? '<td>' + nms.map(nm =>
              '<img class="shotth" src="/shots/' + nm + '">').join('') + '</td>'
          : '<td class="na">--</td>';
      }
      if (k === 'spark') { const sp = sparkSpan(r); return sp ? '<td>' + sp + '</td>'
                                                              : '<td class="na">--</td>'; }
      if (k === 'video_sync')
        return v === 1 ? '<td><span class="vsb y">YES</span></td>'
                       : '<td class="na">--</td>';
      if (v === undefined || v === '') return '<td class="na">--</td>';
      if (k === 'band')
        return '<td><span class="bchip ' + (v === '2.4' ? 'b24' : 'b58') + '">' +
               esc(v) + '</span></td>';
      if (k === 'geo')
        return '<td title="' + esc(r.geo_lat) + ', ' + esc(r.geo_lon) +
               '">📍</td>';
      if (k === 'channel') return '<td>' + chCell(r) + '</td>';
      if (k === 'drone') return '<td>' + droneChip(r.dronefp, v, r.droneNew) + '</td>';
      if (k === 'cfo_ppm')
        return '<td class="num" title="per-measurement carrier offset — drifts ' +
               'with video content; NOT a fingerprint">' + esc(fmtCfo(v)) + '</td>';
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
  const TITLE = ['channel', 'drone', 'geo', 'start_iso', 'duration_s', 'video_sync', 'shot'];
  document.getElementById('detcards').innerHTML = rows.map(r => {
    let h = '<div class="dcline">';
    if (r.channel !== undefined && r.channel !== '') h += chCell(r);
    if (r.drone) h += droneChip(r.dronefp, r.drone, r.droneNew);
    if (r.geo) h += '<span title="' + esc(r.geo_lat) + ', ' + esc(r.geo_lon) +
                    '">📍</span>';
    if (r.start_iso) h += '<span class="dcts">' + fmtISO(r.start_iso) + '</span>';
    if (r.duration_s !== undefined && r.duration_s !== '')
      h += '<span class="dcdur">' + esc(fmtDur(r.duration_s)) + '</span>';
    shotsFor(r, shots).forEach(nm => {
      h += '<img class="shotth" src="/shots/' + nm + '">';
    });
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
    let rcls = detRowCls(r);
    if (showNoise && isNoise(r)) rcls += (rcls ? ' ' : '') + 'nz';
    return '<div class="detcard ' + rcls + '">' + h + '</div></div>';
  }).join('');
}
document.querySelector('.detwrap').addEventListener('click', e => {
  if (e.target.classList && e.target.classList.contains('shotth')) {
    showShot(e.target.getAttribute('src'));
    return;
  }
  if (e.target.closest && !e.target.closest('.dedit')) {
    const chip = e.target.closest('.dchip');
    if (chip) droneRename(chip);
  }
});
document.getElementById('lightbox').addEventListener('click', () => {
  document.getElementById('lightbox').style.display = 'none';
});
document.getElementById('detnoise').addEventListener('click', () => {
  showNoise = !showNoise;
  if (detLast) renderDets(detLast);
});
/* Browser geolocation (permission-gated, remembered in c5_geo): while ON,
   the latest fix rides every /api/state poll as an X-Geo header and the
   backend stamps it onto detection rows opened since. */
let geoOn = lsGet('c5_geo') === '1';
let geoFix = null, geoWatch = null;
const geoBtn = document.getElementById('geobtn');
function geoUi() {
  geoBtn.textContent = geoOn ? '📍 ON' : '📍 OFF';
  geoBtn.classList.toggle('on', geoOn);
}
function geoStart() {
  if (!('geolocation' in navigator)) {
    geoOn = false; lsSet('c5_geo', '0'); geoUi();
    toast('no geolocation API');
    return;
  }
  navigator.geolocation.getCurrentPosition(p => {
    geoFix = { lat: p.coords.latitude, lon: p.coords.longitude,
               acc: p.coords.accuracy };
    if (geoWatch === null)
      geoWatch = navigator.geolocation.watchPosition(q => {
        geoFix = { lat: q.coords.latitude, lon: q.coords.longitude,
                   acc: q.coords.accuracy };
      }, () => {}, { maximumAge: 5000, timeout: 10000 });
  }, () => {
    geoOn = false; lsSet('c5_geo', '0'); geoUi();
    toast('geolocation denied');
  });
}
geoBtn.addEventListener('click', () => {
  geoOn = !geoOn;
  lsSet('c5_geo', geoOn ? '1' : '0');
  if (geoOn) geoStart();
  else if (geoWatch !== null) {
    navigator.geolocation.clearWatch(geoWatch);
    geoWatch = null; geoFix = null;
  }
  geoUi();
});
if (geoOn) geoStart();
geoUi();
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

/* ---- ELRS sniffer card + tab (data rides /api/state as s.elrs) ---- */
const STICK_NAMES = ['R', 'P', 'T', 'Y'];
/* Stick deflection: 988-2012 us -> -100..+100 % around 1500. 0 us (never
   decoded) or lock=0 greys the bar: frozen/stale per the protocol notes. */
const stickPct = us => us > 0 ? Math.max(-100, Math.min(100, (us - 1500) / 512 * 100)) : 0;
function stickHtml(ch, lock) {
  return STICK_NAMES.map((n, i) => {
    const us = (ch && ch[i]) || 0, dim = !lock || !us;
    const p = stickPct(us), l = p >= 0 ? 50 : 50 + p / 2, w = Math.abs(p) / 2;
    return '<div class="stk' + (dim ? ' dim' : '') + '"><i>' + n + '</i>' +
      '<div class="stkbar"><div class="stkfill" style="left:' + l.toFixed(1) +
      '%;width:' + w.toFixed(1) + '%"></div></div><b>' + (us || '----') + '</b></div>';
  }).join('');
}
/* Detail text per event type for the ticker/log (the colored tag carries
   the type name). Unknown types degrade to a JSON shrug, per protocol. */
function fmtEv(ev) {
  switch (ev.t) {
    case 'sync': {
      // FLRC syncs are validated STRUCTURALLY (no ELRS CRC): ok:0 is the
      // normal case and must never read as a failure. Round-2 fields
      // (validated/model_id/tail_src) pass through when present.
      const flrc = ev.band === 'flrc';
      let s = 'uid=' + (ev.uid || '?') + ' rateIdx=' + ev.rateIdx;
      if (ev.freq) s += ' ' + (ev.freq / 1e6).toFixed(1) + 'MHz';
      if (ev.validated) s += ' ✓' + ev.validated;
      if (ev.model_id !== undefined && ev.model_id !== null)
        s += ' model' + ev.model_id;
      if (flrc) s += ' structural';
      else if (!ev.ok) s += ' (crc fail)';
      return s;
    }
    case 'lock': return 'link captured';
    case 'unlock': return ev.why || '';
    case 'linkstats': return 'lq=' + ev.lq + ' rssi=' + ev.rssi1 + '/' +
                             ev.rssi2 + ' snr=' + ev.snr;
    case 'gps': return (ev.lat_e7 / 1e7).toFixed(5) + ',' +
                       (ev.lon_e7 / 1e7).toFixed(5) + ' ' + ev.sats + 'sat ' +
                       (ev.spd_kmh10 / 10).toFixed(0) + 'km/h';
    case 'batt': return (ev.v10 / 10).toFixed(1) + 'V ' +
                        (ev.a10 / 10).toFixed(1) + 'A ' + ev.mah + 'mAh';
    case 'atti': return 'p=' + (ev.p / 10000 * 57.3).toFixed(0) +
                        ' r=' + (ev.r / 10000 * 57.3).toFixed(0) +
                        ' y=' + (ev.y / 10000 * 57.3).toFixed(0);
    case 'fm': return ev.m || '';
    case 'dwell': return (ev.rate || '') + ' iq=' + (ev.iq || '?');
    case 'tlm': return 'ft=' + ev.ft;
    case 'boot': return 'v' + (ev.v || '?') + ' ' + (ev.board || '');
    case 'ready': return 'radio=' + ev.radio + ' freq=' + (ev.sync_freq || '?');
    case 'radio_up': return 'recovered';
    case 'error': return (ev.what || '?') + ' ' + (ev.detail || '');
    case 'probe': return (ev.set || ev.info || '') + (ev.ok ? ' ok' : '');
    case 'crack':
      return ev.state + (ev.uid_tail ? ' tail=' + ev.uid_tail +
        (ev.tail_src ? '(' + ev.tail_src + ')' : '') : '') +
        (ev.state === 'cracking' ? ' ' + (ev.done || 0) + '/' + (ev.total || 256) +
         ' best=' + (ev.valids_best || 0) : '') +
        (ev.uid_full ? ' uid=' + ev.uid_full : '') +
        (ev.state === 'failed' ? ' — auto-retry' : '');
    case 'event':
      switch (ev.what) {
        case 'fp': return 'band=' + (ev.band || '?') + ' tail=' +
                          (ev.uid_tail || '?');
        case 'flrc_sync': return 'uid_pkt=' + (ev.uid_pkt || '?') +
                          ' phrase=' + (ev.uid_phrase || '?') +
                          (ev.match ? ' MATCH' : ' no-match') +
                          (ev.tail_src ? ' ' + ev.tail_src : '');
        case 'modelId': return 'recovered ELRS modelId=' +
                          (ev.id !== undefined ? ev.id : '?');
        case 'uid2_crack': return 'uid2=' + ev.uid2 + ' valids=' + ev.valids;
        case 'uid_cracked': return 'UID=' + (ev.uid || '?');
        case 'uid2_crack_failed': return 'UID[2] candidates exhausted';
        case 'uid': return 'bind phrase applied';
        default: return ev.what || '';
      }
    default: return '';
  }
}
/* Class key for color-coding: t="event" lines classify by their `what`
   (fp/uid_cracked/uid2_crack/flrc_sync + t=crack are the OSINT events -> magenta). */
const evKey = ev => ev.t === 'event' ? (ev.what || 'event') : (ev.t || '?');
/* Sync lines carry their band in the tag (SYNC·LORA cyan vs SYNC·FLRC
   magenta) so the two validation paths read distinctly at a glance. */
const evTag = ev => ev.t === 'sync'
  ? 'SYNC·' + (ev.band === 'flrc' ? 'FLRC' : 'LORA')
  : evKey(ev).toUpperCase();
const evLine = (ev, stamp) =>
  '<div class="evl k-' + esc(evKey(ev)) +
  (ev.t === 'sync' && ev.band === 'flrc' ? ' flrc' : '') + '">' +
  (stamp ? '<span class="ets">' + esc(stamp) + '</span>' : '') +
  '<span class="etag">' + esc(evTag(ev)) + '</span>' +
  esc(fmtEv(ev)) +
  (ev.age_s !== undefined ? '<span class="eage">' + ev.age_s.toFixed(0) + 's</span>' : '') +
  '</div>';
/* Latest intel items for the LIVE-tab card, preference-ordered:
   GPS -> batt -> fm -> atti -> linkstats (model-reported uplink). */
function intelItems(e) {
  const t = (e && e.tlm) || {}, out = [];
  if (t.gps) out.push('GPS ' + (t.gps.lat_e7 / 1e7).toFixed(5) + ',' +
    (t.gps.lon_e7 / 1e7).toFixed(5) + ' · ' + t.gps.sats + 'sat · ' +
    (t.gps.spd_kmh10 / 10).toFixed(0) + 'km/h');
  if (t.batt) out.push('BATT ' + (t.batt.v10 / 10).toFixed(1) + 'V · ' +
    (t.batt.a10 / 10).toFixed(1) + 'A · ' + t.batt.mah + 'mAh');
  if (t.fm) out.push('MODE ' + t.fm.m);
  if (t.atti) out.push('ATTI p' + (t.atti.p / 10000 * 57.3).toFixed(0) +
    ' r' + (t.atti.r / 10000 * 57.3).toFixed(0) +
    ' y' + (t.atti.y / 10000 * 57.3).toFixed(0));
  if (t.linkstats) out.push('UPLINK lq=' + t.linkstats.lq + ' rssi2=' +
    t.linkstats.rssi2 + ' snr=' + t.linkstats.snr);
  const ci = crackInfo(e, true), cl = ci && crackLine(ci);
  if (cl) out.unshift(cl);
  return out;
}
/* Crack machine state (dashboard contract): the last t="crack" event is
   keep-last (backend mirrors it on every poll) and stats.crack carries the
   last state string, so the panel survives an event line lost on the wire. */
let crackEv = null;                      // keep-last latest crack payload
function crackInfo(e, on) {
  if (!on || !e) return null;
  if (e.crack) crackEv = e.crack;
  const state = e.crack_state || (crackEv && crackEv.state) || null;
  return { state, ev: crackEv || {}, mode: e.mode || null,
           lora: !!(e.lock && /^LoRa/.test(e.rate || '')) };
}
/* LIVE-card intel line while the pipeline runs (cracked is shown by the UID
   chips + panel instead). */
function crackLine(ci) {
  const ck = ci.ev;
  switch (ci.state) {
    case 'listening': return ci.mode === 'discovery' ? 'LISTENING // discovery'
                                                     : 'LISTENING';
    case 'sync_seen': return 'SYNC ' + (ck.uid_tail || '?');
    case 'identity': return 'IDENTITY ' + (ck.uid_tail || '?');
    case 'cracking': return 'CRACK ' + (ck.done || 0) + '/' + (ck.total || 256) +
                         ' best=' + (ck.valids_best || 0);
    case 'failed': return 'CRACK FAILED · auto-retry';
    default: return null;
  }
}
/* Crack panel: SWEEP -> SYNC -> IDENTITY -> CRACKING -> CRACKED/FAILED stage
   chips. Passed = solid lit, current = pulsing, future = dim. LoRa locks
   never enter the pipeline: CRACKING shows a dim N/A. The strip is fixed
   height and always reserves its slot (visibility-toggled only). */
const CK_IDX = { listening: 0, sync_seen: 1, identity: 2, cracking: 3,
                 cracked: 4, failed: 4 };
function updElrsCrack(e, on, set) {
  document.getElementById('ecrack').style.visibility = on ? 'visible' : 'hidden';
  const ci = crackInfo(e, on), st = ci && ci.state;
  const ck = (ci && ci.ev) || {}, mode = ci && ci.mode;
  const idx = st !== null && st !== undefined ? CK_IDX[st] : (mode ? 0 : -1);
  ['cks-sweep', 'cks-sync', 'cks-ident', 'cks-crack', 'cks-done']
    .forEach((id, i) => {
      let cls = 'ckst';
      if (idx >= 0 && i < idx) cls += ' pass';
      else if (idx >= 0 && i === idx)
        cls += ' on' + (st === 'failed' ? ' fail' : st === 'cracked' ? ' ok' : '');
      document.getElementById(id).className = cls;
    });
  set('cksweep-sub', mode || '');
  // SYNC chip sub: tail + last sync's band, ✓ when the firmware reports a
  // validation path (round-2 `validated` field, or the derived crc/structural).
  const ls = on && e && e.last_sync ? e.last_sync : null;
  set('cktail', idx >= 1 && ck.uid_tail ?
    'tail ' + ck.uid_tail +
    (ls && ls.band ? ' · ' + String(ls.band).toUpperCase() : '') +
    (ls && (ls.validated || ls.band === 'flrc' || ls.ok) ? ' ✓' : '') : '');
  set('ckident-sub', st === 'identity' ? '2nd sync ok' : '');
  // LoRa lock that never entered the pipeline: CRACKING goes dim "N/A".
  const loraNa = ci && ci.lora && (!st || st === 'listening');
  if (loraNa) {
    document.getElementById('cks-crack').className = 'ckst';
    set('ckprog', 'N/A');
  } else {
    set('ckprog', st === 'cracking' ?
      (ck.done || 0) + '/' + (ck.total || 256) + ' best=' + (ck.valids_best || 0) :
      (idx >= 3 && ck.done ? ck.done + '/' + (ck.total || 256) : ''));
  }
  document.getElementById('ckfill').style.width =
    st === 'cracking' && ck.total ?
      Math.min(100, (ck.done || 0) / ck.total * 100) + '%' :
      (st === 'cracked' ? '100%' : '0%');
  document.getElementById('ckdone-lbl').textContent =
    st === 'failed' ? 'FAILED' : 'CRACKED';
  const ds = document.getElementById('ckdone-sub');
  if (st === 'cracked' && ck.uid_full) {
    // uid_full's first bytes are a phrase-derived guess — only the tail is
    // ground truth: dim the guess, keep the tail bright (label via phrase_crack).
    const uf = String(ck.uid_full), tail = String(ck.uid_tail || '');
    ds.innerHTML = esc(uf.slice(0, uf.length - tail.length)) +
                   '<b>' + esc(tail) + '</b>';
  } else ds.textContent = st === 'failed' ? 'auto-retry' : '';
}
/* Tails-heard strip: one chip per unique UID tail heard on syncs (count,
   last RSSI, band, validated state); the phrase-derived tail is ringed as
   "own" so the user can tell their link from neighbors at a glance. The
   hint slot reports uid-src: recent "phrase set" apply feedback wins over
   the default-phrase nag. */
function updElrsTails(e, on) {
  const tails = on && e && Array.isArray(e.tails) ? e.tails : [];
  document.getElementById('etailchips').innerHTML = tails.map(t =>
    '<span class="tchip' + (t.own ? ' own' : '') + '"' +
    (t.own ? ' title="your bind phrase UID tail"' : '') + '>' +
    '<b>' + esc(t.tail) + '</b>' +
    '<span class="tcnt"> ×' + t.count + '</span> ' +
    '<span class="trssi">' +
      (typeof t.rssi === 'number' ? t.rssi : '--') + 'dBm</span> ' +
    (t.band ? '<span class="tband">' + esc(String(t.band).toUpperCase()) +
              '</span> ' : '') +
    (t.validated ? '<span class="tval">✓' + esc(t.validated) + '</span>'
                 : '<span class="tval na">—</span>') +
    '</span>').join('');
  const hint = document.getElementById('euidhint');
  const src = on && e ? e.uid_src : null;
  const setAge = on && e ? e.uid_set_age_s : null;
  if (src === 'set' && typeof setAge === 'number' && setAge < 15) {
    hint.textContent = 'phrase set ✓ tail ' + (e.uid || '?');
    hint.className = 'ok';
    hint.style.visibility = 'visible';
  } else if (src && String(src).startsWith('default')) {
    hint.textContent = 'default bind phrase — set yours';
    hint.className = '';
    hint.style.visibility = 'visible';
  } else hint.style.visibility = 'hidden';
}
let elrsHist = [];                     // {t, rssi, lq} per poll, 60 s window
let elogSeen = 0;                      // last event seq in the tab log
let elogRows = [];                     // newest-last HTML lines, capped
/* Session encounter list (this page load only): one entry per link-lock
   rise->fall with open/close wall times, uid and peak rssi/lq. The
   persistent log is the CSV (band 2.4 rows); this is the live view. */
let elrsEnc = [];                      // closed encounters, newest-last
let elrsEncOpen = null;                // {t0, uid, peakRssi, peakLq}
const encLine = en =>
  '<div class="een' + (en.t1 ? '' : ' open') + '">' +
  '<span class="ets">' + new Date(en.t0).toTimeString().slice(0, 8) +
  (en.t1 ? '–' + new Date(en.t1).toTimeString().slice(0, 8) : '–…') + '</span>' +
  '<span class="euid">' + esc(en.uid || 'uid?') + '</span> ' +
  'rssi ' + (en.peakRssi === null || en.peakRssi === undefined ? '--' : en.peakRssi) +
  ' · lq ' + (en.peakLq === null || en.peakLq === undefined ? '--' : en.peakLq) +
  '</div>';
function updElrsEnc(on, lock, rssi, lq, uid) {
  if (lock && !elrsEncOpen) {
    elrsEncOpen = { t0: Date.now(), uid: uid || null,
                    peakRssi: rssi, peakLq: lq };
  } else if (lock && elrsEncOpen) {
    if (uid) elrsEncOpen.uid = uid;
    if (typeof rssi === 'number')
      elrsEncOpen.peakRssi = Math.max(elrsEncOpen.peakRssi ?? -Infinity, rssi);
    if (typeof lq === 'number')
      elrsEncOpen.peakLq = Math.max(elrsEncOpen.peakLq ?? -Infinity, lq);
  } else if (elrsEncOpen) {
    elrsEnc.push(Object.assign({ t1: Date.now() }, elrsEncOpen));
    elrsEnc = elrsEnc.slice(-40);
    elrsEncOpen = null;
  }
  document.getElementById('eenc').innerHTML =
    '<div class="eenc-h">ENCOUNTERS // session</div>' +
    (elrsEncOpen ? encLine(elrsEncOpen) : '') +
    elrsEnc.slice().reverse().map(encLine).join('');
}
/* Telemetry cards: each label:value card carries its value age (dim "Ns ago" beyond 5 s)
   and degrades to a dim placeholder; values are latest-of-type from the backend
   (tlm dict never expires, so presence == arrived at least once since connect). */
const etAge = a => (typeof a === 'number' && a > 5)
  ? '<span class="etage">' + Math.round(a) + 's ago</span>' : '';
function etCard(id, label, valHtml, sub, age) {
  document.getElementById(id).innerHTML =
    '<div class="etk">' + esc(label) + etAge(age) + '</div>' +
    '<div class="etv">' + valHtml + '</div>' +
    (sub ? '<div class="etsub">' + esc(sub) + '</div>' : '');
}
function updElrsTlm(e, on) {
  const t = (e && e.tlm) || {}, na = '<span class="etna">—</span>';
  let gpsHtml, gpsAge = t.gps && t.gps.age_s;
  if (!t.gps) gpsHtml = '<span class="etna">—</span>';
  else if (!t.gps.lat_e7 && !t.gps.lon_e7)
    gpsHtml = '<span class="etna">no GPS lock</span>';
  else {
    const la = (t.gps.lat_e7 / 1e7).toFixed(5),
          lo = (t.gps.lon_e7 / 1e7).toFixed(5);
    gpsHtml = esc(la + ',' + lo) + ' · ' + esc(t.gps.sats) + 'sat · ' +
      (t.gps.spd_kmh10 / 10).toFixed(0) + 'km/h' +
      ' <button class="ecopy" data-coords="' + la + ',' + lo +
      '" title="copy coordinates">&#128203;</button>';
  }
  etCard('etgps', 'GPS', gpsHtml, '', gpsAge);
  etCard('etbatt', 'BATTERY', t.batt ?
    '<b>' + (t.batt.v10 / 10).toFixed(1) + 'V</b> · ' +
    (t.batt.a10 / 10).toFixed(1) + 'A · ' + esc(t.batt.mah) + 'mAh' : na,
    '', t.batt && t.batt.age_s);
  etCard('etatti', 'ATTITUDE', t.atti ?
    'p<b>' + (t.atti.p / 10000 * 57.3).toFixed(0) + '°</b> r<b>' +
    (t.atti.r / 10000 * 57.3).toFixed(0) + '°</b> y<b>' +
    (t.atti.y / 10000 * 57.3).toFixed(0) + '°</b>' : na,
    '', t.atti && t.atti.age_s);
  etCard('etfm', 'FLIGHT MODE', t.fm ? '<b>' + esc(t.fm.m) + '</b>' : na,
    '', t.fm && t.fm.age_s);
  etCard('etls', 'LINKSTATS // MODEL', t.linkstats ?
    'lq <b>' + esc(t.linkstats.lq) + '</b> · rssi <b>' +
    esc(t.linkstats.rssi1) + '/' + esc(t.linkstats.rssi2) + '</b> · snr <b>' +
    esc(t.linkstats.snr) + '</b>' : na,
    "reported by the model's receiver (uplink at the aircraft, not the sniffer)",
    t.linkstats && t.linkstats.age_s);
  // Empty-state honesty: locked with zero aircraft telemetry ever, or
  // sweeping. The note slot is always reserved (visibility, fixed height).
  const note = document.getElementById('etnote');
  if (on && !e.lock) {
    note.style.visibility = 'visible'; note.className = 'sweep';
    note.textContent = 'sweeping — waiting for link';
  } else if (on && !(t.gps || t.batt || t.atti || t.fm)) {
    note.style.visibility = 'visible'; note.className = 'warn';
    note.textContent = "pilot's telemetry is OFF — control link data only " +
                       '(sticks + linkstats)';
  } else note.style.visibility = 'hidden';
}
function updElrs(e, set) {
  const dot = document.getElementById('edot');
  const chip = document.getElementById('elrschip');
  const on = e && e.connected;
  dot.className = on ? (e.lock ? 'lock' : 'sweep') : '';
  chip.textContent = on ? (e.lock ? 'LOCK' : 'sweep') : 'offline';
  chip.className = on ? (e.lock ? 'lock' : 'sweep') : 'off';
  const numv = v => (typeof v === 'number' && isFinite(v)) ? v : null;
  const rssi = on ? numv(e.rssi) : null;
  set('erssi', on ? (rssi === null ? '--' : rssi) : '--');
  set('elq', on ? numv(e.lq) ?? '--' : '--');
  set('erate', on ? (e.rate || '--') + ' · iq ' + (e.iq || '-') + ' · ' +
        (e.pps || 0) + ' pps · SNR ' + (numv(e.snr) === null ? '--' : e.snr) + ' dB'
                  : 'no dongle');
  document.getElementById('esticks').innerHTML = stickHtml(on ? e.ch : null, on && e.lock);
  const items = intelItems(e);
  set('eintel', on ? (items.length ?
        items[Math.floor(Date.now() / 2000) % items.length] : 'listening...')
                   : '--');
  // Card: UID alias chip + ARMED marker (slot is fixed-height and kept via
  // visibility, so the card layout never shifts when they appear).
  const euid = document.getElementById('euid');
  euid.style.visibility = on && (e.uid || e.arm) ? 'visible' : 'hidden';
  euid.innerHTML = on ?
    (e.uid ? droneChip('elrs:' + String(e.uid).toLowerCase(), e.uid) : '') +
    (e.arm ? '<span class="armb sm">ARMED</span>' : '') : '';
  document.getElementById('etick').innerHTML =
    on ? (e.last_events || []).slice(-3).reverse().map(ev => evLine(ev)).join('') : '';
  // ELRS tab link header: dot, rate/iq, lock badge, ARMED tag, UID alias chip, FOLLOWING chip.
  document.getElementById('xedot').className = on ? (e.lock ? 'lock' : 'sweep') : '';
  set('xehinfo', on ? (e.rate || '--') + ' · iq ' + (e.iq || '-') : 'no dongle');
  const lb = document.getElementById('xelockb');
  lb.textContent = on ? (e.lock ? 'LOCK' : 'SWEEP') : 'OFFLINE';
  lb.className = 'ebadge ' + (on ? (e.lock ? 'lock' : 'sweep') : 'off');
  document.getElementById('xearm').style.visibility =
    on && e.arm ? 'visible' : 'hidden';
  const fp = e.uid ? 'elrs:' + String(e.uid).toLowerCase() : null;
  document.getElementById('xeuidwrap').innerHTML = on && fp ?
    droneChip(fp, 'elrs:' + e.uid) : '';
  const follow = !!(on && e.lock && typeof e.freq === 'number' && e.freq > 0 &&
                    e.fhss !== null && e.fhss !== undefined && e.fhss !== 255);
  const fol = document.getElementById('xefollow');
  fol.style.visibility = follow ? 'visible' : 'hidden';
  if (follow)
    fol.textContent = 'FOLLOWING ' + (e.freq / 1e6).toFixed(1) + ' MHz · fhss ' + e.fhss;
  // Tab gauges (+ secondary lines: rssi_now, raw rx/s) + sticks.
  set('xerssi', on ? (rssi === null ? '--' : rssi) : '--');
  set('xerssinow', on ? 'now ' + (numv(e.rssi_now) === null ? '--' : e.rssi_now) : '');
  set('xelq', on ? numv(e.lq) ?? '--' : '--');
  set('xesnr', on ? (numv(e.snr) === null ? '--' : e.snr) : '--');
  set('xepps', on ? (e.pps || 0) : '--');
  set('xerx', on && numv(e.rx_per_s) !== null ? e.rx_per_s + ' rx/s raw' : '');
  document.getElementById('xesticks').innerHTML = stickHtml(on ? e.ch : null, on && e.lock);
  const allZero = !(e && e.ch || []).some(v => v > 0);
  document.getElementById('xestale').style.visibility =
    on && (!e.lock || allZero) ? 'visible' : 'hidden';
  updElrsTlm(e, on);
  updElrsCrack(e, on, set);
  updElrsTails(e, on);
  // Scrolling event log: append only events newer than the last seen seq.
  (on ? e.last_events || [] : []).forEach(ev => {
    if (ev.seq > elogSeen) {
      elogSeen = ev.seq;
      elogRows.push(evLine(ev, new Date().toTimeString().slice(0, 8)));
    }
  });
  elogRows = elogRows.slice(-60);
  document.getElementById('elog').innerHTML = elogRows.slice().reverse().join('');
  // Sparkline history: 60 s rolling window, updated every poll.
  if (on) {
    elrsHist.push({ t: Date.now(), rssi: rssi, lq: numv(e.lq) });
    elrsHist = elrsHist.filter(h => Date.now() - h.t < 60000);
  }
  updElrsEnc(on, !!(on && e.lock), rssi, numv(e.lq), e.uid);
}
/* ELRS tab sparkline: RSSI auto-range (green) + LQ 0..100 (amber), 60 s. */
function drawElrsSpark() {
  const c = document.getElementById('espark');
  if (!c || !c.clientWidth) return;
  const x = c.getContext('2d');
  const w = c.width = c.clientWidth * devicePixelRatio,
        h = c.height = c.clientHeight * devicePixelRatio;
  const now = Date.now(), dpr = devicePixelRatio;
  const pts = elrsHist.filter(p => now - p.t < 60000);
  x.clearRect(0, 0, w, h);
  x.strokeStyle = 'rgba(51,80,47,.3)'; x.lineWidth = 1;
  for (let i = 1; i < 4; i++) {
    x.beginPath(); x.moveTo(0, h * i / 4); x.lineTo(w, h * i / 4); x.stroke();
  }
  if (pts.length < 2) return;
  const X = t => (1 - (now - t) / 60000) * w;
  const rs = pts.map(p => p.rssi).filter(v => typeof v === 'number');
  if (rs.length > 1) {
    let lo = Math.min.apply(null, rs), hi = Math.max.apply(null, rs);
    if (hi - lo < 6) { const m = (hi + lo) / 2; lo = m - 3; hi = m + 3; }
    const Y = v => h - (v - lo) / (hi - lo) * (h - 16 * dpr) - 8 * dpr;
    x.strokeStyle = '#39ff6a'; x.lineWidth = 1.5 * dpr;
    x.shadowColor = '#39ff6a'; x.shadowBlur = 4 * dpr;
    x.beginPath();
    let started = false;
    pts.forEach(p => {
      if (typeof p.rssi !== 'number') return;
      const px = X(p.t), py = Y(p.rssi);
      if (!started) { x.moveTo(px, py); started = true; } else x.lineTo(px, py);
    });
    x.stroke(); x.shadowBlur = 0;
    x.fillStyle = '#4a6a4f'; x.font = (9 * dpr) + 'px monospace';
    x.fillText(hi.toFixed(0) + ' dBm', 6 * dpr, 10 * dpr);
    x.fillText(lo.toFixed(0), 6 * dpr, h - 4 * dpr);
  }
  const Yq = v => h - v / 100 * (h - 16 * dpr) - 8 * dpr;
  x.strokeStyle = '#ffb000'; x.lineWidth = dpr;
  x.beginPath();
  let qs = false;
  pts.forEach(p => {
    if (typeof p.lq !== 'number') return;
    const px = X(p.t), py = Yq(p.lq);
    if (!qs) { x.moveTo(px, py); qs = true; } else x.lineTo(px, py);
  });
  x.stroke();
}

/* GPS card: 📋 copies "lat,lon" to the clipboard. */
document.getElementById('etgps').addEventListener('click', e => {
  const b = e.target.closest && e.target.closest('.ecopy');
  if (!b) return;
  const txt = b.getAttribute('data-coords') || '';
  if (navigator.clipboard && navigator.clipboard.writeText)
    navigator.clipboard.writeText(txt).then(() => toast('GPS copied: ' + txt),
                                           () => toast('copy failed'));
  else toast('clipboard unavailable');
});
/* UID chips (ELRS tab header + LIVE card) rename through the same alias
   system as the DETECTIONS table (fingerprint elrs:<uid>). */
['elinkhdr', 'euid'].forEach(id =>
  document.getElementById(id).addEventListener('click', e => {
    if (e.target.closest && !e.target.closest('.dedit')) {
      const chip = e.target.closest('.dchip');
      if (chip) droneRename(chip);
    }
  }));
/* Bind-phrase apply: POST the phrase, the dongle's confirming uid event
   (src:"set") shows up as panel feedback via the state poll. */
async function applyPhrase() {
  const inp = document.getElementById('ephrase');
  const phrase = (inp.value || '').trim();
  if (!phrase) { toast('enter a bind phrase'); return; }
  try {
    const r = await (await fetch('/api/elrs/phrase', {
      method: 'POST', headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ phrase })
    })).json();
    toast(r.ok ? 'phrase sent — waiting for dongle confirm'
               : 'phrase failed: ' + (r.error || '?'));
    if (r.ok) inp.value = '';
  } catch (e) { toast('phrase failed'); }
}
document.getElementById('ephrasebtn').addEventListener('click', applyPhrase);
document.getElementById('ephrase').addEventListener('keydown', e => {
  if (e.key === 'Enter') applyPhrase();
});

async function poll() {
  let s;
  const hdrs = geoFix ? { 'X-Geo': geoFix.lat + ',' + geoFix.lon + ',' +
                                   Math.round(geoFix.acc || 0) } : {};
  try { s = await (await fetch('/api/state', { headers: hdrs })).json(); }
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
  vm.style.visibility = s.video_mode === 'sync' ? 'visible' : 'hidden';
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
  updElrs(s.elrs, set);
  if (!s.connected) toast('SERIAL DISCONNECTED');
}

/* Animated meters: smoothed needle with peak-hold, lock-marked history. */
let shown = 0, peak = 0, peakT = 0;
function draw() {
  try {
  /* Every data source updates regardless of the active tab; only canvas
     work is skipped while a tab is hidden. The ELRS tab draws just its
     sparkline. */
  if (tabCur !== 'live') {
    if (tabCur === 'elrs') drawElrsSpark();
    return;
  }
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
setInterval(pollDets, 500);          // all tabs live: no per-tab fetch pauses
const savedTab = lsGet('c5_tab');
if (savedTab && savedTab !== 'live') showTab(savedTab);
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
        # Browser geolocation fix, if the page has permission and sends it.
        geo_hdr = request.headers.get("X-Geo", "")
        try:
            parts = [float(p) for p in geo_hdr.split(",")]
        except ValueError:
            parts = []
        if len(parts) >= 2 and -90.0 <= parts[0] <= 90.0 \
                and -180.0 <= parts[1] <= 180.0:
            acc = parts[2] if len(parts) >= 3 and parts[2] >= 0 else None
            _geo_set(parts[0], parts[1], acc)
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
        body["elrs"] = ELRS.snapshot()   # own lock; independent of STATE
        body["arbiter"] = ARBITER.snapshot()   # port classification debug
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

    @app.post("/api/elrs/phrase")
    def api_elrs_phrase():
        """Set the ELRS bind phrase on the sniffer dongle (console 'U
        <phrase>'; firmware persists it to NVS and answers with a uid event,
        src:"set" — the panel picks that up as apply feedback)."""
        body = request.get_json(force=True, silent=True) or {}
        phrase = str(body.get("phrase") or "").strip()
        if not phrase or len(phrase) > ELRS_PHRASE_MAX or \
                not all(32 <= ord(c) < 127 for c in phrase):
            return jsonify({"ok": False, "error": "bad phrase"}), 400
        if ELRS_MGR is None or not ELRS_MGR.send_line("U " + phrase):
            return jsonify({"ok": False,
                            "error": "dongle busy or unavailable"}), 503
        return jsonify({"ok": True})

    @app.post("/api/shot")
    def api_shot():
        global _last_shot_mono
        data = current_jpeg()
        with STATE.lock:
            ep_start = EPISODE["start"] if EPISODE is not None else None
        _last_shot_mono = time.monotonic()
        base = _shot_stamp(ep_start if ep_start is not None else time.time())
        name = _save_shot(data, base, time.strftime("%H%M%S"))
        return jsonify({"ok": True, "name": name})

    @app.get("/shots/<path:filename>")
    def shot_file(filename):
        # send_from_directory resolves through safe_join: no traversal.
        return send_from_directory(SHOTS_DIR, filename)

    @app.get("/api/detections")
    def api_detections():
        rows, total = _det_read_rows()
        with _aliases_lock:
            aliases = dict(DRONE_ALIASES)
        return jsonify({"total": total, "rows": rows, "shots": _shot_list(),
                        "aliases": aliases})

    @app.get("/api/aliases")
    def api_aliases():
        with _aliases_lock:
            return jsonify(dict(DRONE_ALIASES))

    @app.post("/api/alias")
    def api_alias():
        body = request.get_json(force=True, silent=True) or {}
        fp = body.get("fp")
        alias = str(body.get("alias") or "").strip()
        if not isinstance(fp, str) or not ALIAS_FP_RE.match(fp):
            return jsonify({"ok": False, "error": "bad fingerprint"}), 400
        if len(alias) > ALIAS_MAX:
            return jsonify({"ok": False, "error": "alias too long"}), 400
        with _aliases_lock:
            if alias:
                DRONE_ALIASES[fp] = alias
            else:
                DRONE_ALIASES.pop(fp, None)
            try:
                _save_aliases()
            except OSError:
                return jsonify({"ok": False, "error": "save failed"}), 500
            return jsonify({"ok": True, "aliases": dict(DRONE_ALIASES)})

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
ELRS_MGR = None


def main() -> None:
    global SERIAL, ELRS_MGR
    ap = argparse.ArgumentParser(description="C5VRX-3 foxhunt HUD")
    ap.add_argument("serial_port", nargs="?", default="/dev/cu.usbmodem312301")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--host", default="0.0.0.0")
    ap.add_argument("--port", type=int, default=5000)
    args = ap.parse_args()

    SERIAL = SerialManager(args.serial_port, args.baud)
    ARBITER.board_baud = args.baud
    SERIAL.start()
    # Second reader: the shared arbiter classifies usbmodem ports by
    # evidence, so both readers always land on their own device.
    elrs_mgr = ElrsManager()
    ELRS_MGR = elrs_mgr
    elrs_mgr.start()
    app = create_app()
    try:
        app.run(host=args.host, port=args.port, threaded=True, use_reloader=False)
    except KeyboardInterrupt:
        pass
    finally:
        print("\nshutting down: stopping preview stream")
        SERIAL.close()
        elrs_mgr.close()
        SERIAL.join(timeout=2.0)
        elrs_mgr.join(timeout=2.0)


if __name__ == "__main__":
    main()
