#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-only
"""C5VRX Windows flasher + runtime USB control panel.

The one-file executable bundles the ESP32-C5 firmware, flashes it, reconnects
through USB Serial/JTAG, allows FPV band/channel selection without reflashing,
and can trigger finite IQ captures for the reverse-engineering workflow.
"""

from __future__ import annotations

import json
import math
import queue
import statistics
import sys
import threading
import time
import traceback
from contextlib import redirect_stderr, redirect_stdout
from pathlib import Path
import tkinter as tk
from tkinter import filedialog, messagebox, ttk

import esptool
import numpy as np
import serial
from serial.tools import list_ports

from c5vrx_lab import SessionRecorder

from c5vrx_usb_protocol import (
    FRAME_DESCRIPTOR,
    PACKET_GRAY8_FRAME,
    PACKET_YUV411_FRAME,
    PACKET_IQ_U32_BLOCK,
    PACKET_IQ_U32_CHUNK,
    PACKET_PHASE8_CHUNK,
    PACKET_STREAM_END,
    PACKET_STREAM_INFO,
    PIXEL_FORMAT_GRAY8,
    PIXEL_FORMAT_YUV411,
    Packet,
    StreamDecoder,
    decode_iq_block,
    decode_iq_chunk,
    decode_phase8_chunk,
    yuv411_to_rgb,
)

APP_TITLE = "C5VRX Receiver Console"
APP_BUILD = "video-proof-26-always-on-av"
C5_RX_MAX_MHZ = 5885
VIDEO_LINE_RATES_HZ = {
    "PAL": 15_625.0,
    "NTSC": 15_734.264,
}
VIDEO_FIELD_PERIOD_US = {
    "PAL": 20_000.0,
    "NTSC": 1_000_000.0 / 59.94,
}
VIDEO_SYNC_MIN_SCORE = 0.65
VIDEO_POLARITY_LOCK_VOTES = 3.0
VIDEO_VERTICAL_PHASE_GATE_US = 750.0
VIDEO_VERTICAL_PHASE_GAIN = 0.05
VIDEO_VERTICAL_FREQUENCY_GAIN = 0.10
VIDEO_VERTICAL_MIN_CALIBRATION_FIELDS = 100
VIDEO_ACTIVE_START_LINES = {
    "NTSC": 20.0,
    "PAL": 23.0,
}
VIDEO_COLOR_SUBCARRIER_HZ = {
    "PAL": 4_433_618.75,
    "NTSC": 3_579_545.0,
}
# Fractions of one line measured from the leading edge of horizontal sync.
# Both standards place the reference burst on the back porch before active
# video. Keeping this window clear of sync and active luma is important: false
# burst lock would turn ordinary monochrome detail into colored noise.
VIDEO_COLOR_BURST_WINDOW = {
    "PAL": (0.083, 0.135),
    "NTSC": (0.082, 0.135),
}
VIDEO_COLOR_MIN_BURST_LEVEL = 0.018
VIDEO_COLOR_MIN_COHERENCE = 0.30

FPV_BANDS = {
    "A": [5865, 5845, 5825, 5805, 5785, 5765, 5745, 5725],
    "B": [5733, 5752, 5771, 5790, 5809, 5828, 5847, 5866],
    "E": [5705, 5685, 5665, 5645, 5885, 5905, 5925, 5945],
    "F": [5740, 5760, 5780, 5800, 5820, 5840, 5860, 5880],
    "R": [5658, 5695, 5732, 5769, 5806, 5843, 5880, 5917],
}


def resource_dir() -> Path:
    base = getattr(sys, "_MEIPASS", None)
    if base:
        return Path(base) / "firmware"
    return Path(__file__).resolve().parent.parent / "firmware"


def load_firmware_profile() -> dict[str, object]:
    """Load the identity of the firmware bundled into this executable."""
    profile_path = resource_dir() / "profile.json"
    if not profile_path.exists():
        return {
            "schema_version": 0,
            "profile_id": "legacy-unknown",
            "display_name": "legacy/unknown ESP32-C5 profile",
            "av_pin_summary": "Check the firmware bundle documentation before wiring AV",
            "flash_size_mb": 0,
        }

    profile = json.loads(profile_path.read_text(encoding="utf-8"))
    if profile.get("schema_version") != 1:
        raise RuntimeError(f"Unsupported firmware profile schema: {profile_path}")
    for key in ("profile_id", "display_name", "av_pin_summary"):
        if not isinstance(profile.get(key), str) or not profile[key]:
            raise RuntimeError(f"Invalid firmware profile field {key}: {profile_path}")
    if not isinstance(profile.get("flash_size_mb"), int):
        raise RuntimeError(f"Invalid firmware profile flash_size_mb: {profile_path}")
    return profile


def load_flash_plan() -> tuple[list[str], list[tuple[str, Path]]]:
    fw = resource_dir()
    manifest = fw / "flasher_args.json"
    if not manifest.exists():
        raise FileNotFoundError(f"Firmware manifest not found: {manifest}")

    data = json.loads(manifest.read_text(encoding="utf-8"))
    extra = data.get("extra_esptool_args", {})

    args: list[str] = ["--chip", extra.get("chip", "esp32c5")]
    if extra.get("before"):
        args += ["--before", str(extra["before"])]
    if extra.get("after"):
        args += ["--after", str(extra["after"])]
    if extra.get("baud"):
        args += ["--baud", str(extra["baud"])]

    files: list[tuple[str, Path]] = []
    for offset, rel in data.get("flash_files", {}).items():
        p = fw / rel
        if not p.exists():
            p = fw / Path(rel).name
        if not p.exists():
            raise FileNotFoundError(f"Firmware file missing for {offset}: {rel}")
        files.append((offset, p))

    if not files:
        raise RuntimeError("No flash files found in flasher_args.json")
    return args, files


class TextSink:
    def __init__(self, text: tk.Text):
        self.text = text
        self.pending: queue.Queue[str] = queue.Queue(maxsize=2048)
        self.dropped = 0
        self.text.after(25, self._drain)

    def write(self, s: str) -> int:
        original_length = len(s)
        if original_length > 16384:
            s = (f"[on-screen line truncated from {original_length} chars; "
                 "full bytes remain in the session bundle]\n" + s[-4096:])
        if s:
            try:
                self.pending.put_nowait(s)
            except queue.Full:
                # The durable session recorder remains authoritative. Keep the
                # on-screen console recent without ever stalling USB draining.
                try:
                    self.pending.get_nowait()
                    self.pending.task_done()
                except queue.Empty:
                    pass
                self.dropped += 1
                try:
                    self.pending.put_nowait(s)
                except queue.Full:
                    self.dropped += 1
        return original_length

    def flush(self) -> None:
        pass

    def _drain(self) -> None:
        chunks: list[str] = []
        size = 0
        while size < 131072:
            try:
                chunk = self.pending.get_nowait()
            except queue.Empty:
                break
            chunks.append(chunk)
            size += len(chunk)
            self.pending.task_done()
        if chunks:
            self.text.configure(state="normal")
            self.text.insert("end", "".join(chunks))
            line_count = int(self.text.index("end-1c").split(".")[0])
            if line_count > 4000:
                self.text.delete("1.0", f"{line_count - 4000 + 1}.0")
            char_count = int(self.text.count(
                "1.0", "end-1c", "chars")[0])
            if char_count > 300000:
                self.text.delete(
                    "1.0", f"1.0 + {char_count - 300000} chars")
            self.text.see("end")
            self.text.configure(state="disabled")
        self.text.after(25, self._drain)


class C5VRXApp(tk.Tk):
    def __init__(self):
        super().__init__()
        self.firmware_profile = load_firmware_profile()
        self.expected_profile = str(self.firmware_profile["profile_id"])
        self.session = SessionRecorder(
            "receiver-console",
            parent=Path.home() / "Documents" / "C5VRX Sessions",
            board_profile=self.firmware_profile,
            test_config={
                "application": APP_TITLE,
                "baud": 115200,
                "rf_safety": {
                    "bounded_capture_only": True,
                    "no_rf_register_overrides": True,
                    "live_start_validation_preserved": True,
                },
            },
        )
        self.profile_mismatch_warned = False
        self.title(f"{APP_TITLE} — {self.firmware_profile['display_name']}")
        self.geometry("860x650")
        self.minsize(760, 580)

        self._busy = False
        self.ser: serial.Serial | None = None
        self.serial_thread: threading.Thread | None = None
        self.serial_stop = threading.Event()
        self.serial_write_lock = threading.Lock()
        self.usb_write_retries = 0
        self.usb_write_failures = 0
        self.usb_transport_stalled = False
        self.iq_words: list[int] = []
        self.iq_capture_active = False
        self.preview_frame: bytes | None = None
        self.preview_width = 160
        self.preview_height = 120
        self.preview_image: tk.PhotoImage | None = None
        self.preview_display_image: tk.PhotoImage | None = None
        self.preview_sequence: int | None = None
        self.usb_preview_active = False
        self.usb_preview_receiving = False
        self.experimental_live_pending = False
        self.experimental_live_active = False
        self.live_iq_active = False
        self.live_iq_capture_done = False
        self.live_iq_packet_done = False
        self.live_iq_capture_id: int | None = None
        self.live_iq_capture_timestamp_us: int | None = None
        self.live_iq_total_words = 0
        self.live_iq_chunks: dict[int, tuple[int, ...]] = {}
        self.live_phase8_chunks: dict[int, bytes] = {}
        self.live_iq_source_msps: float | None = None
        self.live_iq_usb_started = 0.0
        self.live_iq_usb_bytes = 0
        self.live_iq_blocks = 0
        self.live_video_host_frames = 0
        self.live_iq_request_started = 0.0
        self.live_iq_transport_ready = False
        self.live_iq_capture_retries = 0
        self.live_iq_max_retries = 3
        self.live_iq_fast_mode = False
        self.live_iq_pipeline_target = 1
        self.live_iq_commands_outstanding = 0
        self.live_iq_refill_pending = False
        self.live_iq_last_transport_progress = 0.0
        self.live_iq_watchdog_generation = 0
        self.live_iq_recoveries = 0
        self.live_iq_transport_losses = 0
        self.live_iq_processing_generation = 0
        self.live_iq_processing_queue: queue.Queue[
            tuple[str, object, int | None]] = queue.Queue(maxsize=4)
        self.live_iq_processing_drops = 0
        self.live_video_width = 160
        self.live_video_height = 120
        self.live_video_pixels = bytearray(
            self.live_video_width * self.live_video_height)
        self.live_video_rgb_pixels = bytearray(
            self.live_video_width * self.live_video_height * 3)
        self.live_video_row = 0
        self.live_video_rows_seen: set[int] = set()
        self.live_video_standard_scores = {"PAL": 0.0, "NTSC": 0.0}
        self.live_video_rejected_blocks = 0
        self.live_video_black: float | None = None
        self.live_video_white: float | None = None
        self.live_video_display_offset = 0
        self.live_video_vertical_pending: int | None = None
        self.live_video_vertical_pending_hits = 0
        self.live_video_vertical_anchor_us: float | None = None
        self.live_video_vertical_last_event_us: float | None = None
        self.live_video_vertical_lock_events = 0
        self.live_video_vertical_candidate_us: float | None = None
        self.live_video_vertical_candidate_hits = 0
        self.live_video_vertical_candidate_standard: str | None = None
        self.live_video_field_period_us = VIDEO_FIELD_PERIOD_US["NTSC"]
        self.live_video_vertical_standard: str | None = None
        self.live_video_requested_standard = "Auto"
        self.live_video_requested_sample_rate = 80.0
        self.live_video_fm_polarity_votes = 0.0
        self.live_video_fm_polarity_lock: int | None = None
        self.live_video_color_oscillators: dict[
            tuple[str, float, int], tuple[list[float], list[float]]] = {}
        self.live_video_color_locked_lines = 0
        self.live_video_monochrome_lines = 0
        self.live_video_color_burst_level = 0.0
        self.live_video_color_burst_coherence = 0.0
        self.live_video_color_line_counter = 0
        self.live_video_color_bias = {"PAL": 0j, "NTSC": 0j}
        self.live_video_color_bias_samples = {"PAL": 0, "NTSC": 0}
        self.live_video_color_valid_rows: set[int] = set()
        self.live_video_render_pending = False
        self.live_video_pending_frame: tuple[
            bytes, str, float, float, str] | None = None
        self.live_video_pending_status: str | None = None
        self.device_phase8_supported = False
        self.live_transport_name = "raw-IQ32"
        self.first_test_active = False
        self.first_test_fine_sent = False
        self.first_test_center = 5805

        root = ttk.Frame(self, padding=14)
        root.pack(fill="both", expand=True)

        header = ttk.Frame(root)
        header.pack(fill="x")
        ttk.Label(header, text="C5VRX", font=("Segoe UI", 25, "bold")).pack(side="left")
        ttk.Label(header, text="ESP32-C5 analog FPV receiver & first-hardware console").pack(side="left", padx=12, pady=(8, 0))
        ttk.Label(
            root,
            text=(f"Firmware: {self.firmware_profile['display_name']}  •  "
                  f"AV: {self.firmware_profile['av_pin_summary']}  •  "
                  f"GUI: {APP_BUILD}"),
            foreground="#2457a6",
        ).pack(anchor="w", pady=(4, 0))

        port_row = ttk.Frame(root)
        port_row.pack(fill="x", pady=(12, 8))
        ttk.Label(port_row, text="USB / COM port:").pack(side="left")
        self.port_var = tk.StringVar()
        self.port_combo = ttk.Combobox(port_row, textvariable=self.port_var, state="readonly", width=48)
        self.port_combo.pack(side="left", padx=8, fill="x", expand=True)
        ttk.Button(port_row, text="Refresh", command=self.refresh_ports).pack(side="left", padx=(0, 6))
        self.connect_btn = ttk.Button(port_row, text="Connect", command=self.toggle_connect)
        self.connect_btn.pack(side="left")

        self.connection_var = tk.StringVar(value="Disconnected")
        ttk.Label(root, textvariable=self.connection_var).pack(anchor="w", pady=(0, 8))

        notebook = ttk.Notebook(root)
        notebook.pack(fill="both", expand=True)

        control_tab = ttk.Frame(notebook, padding=12)
        preview_tab = ttk.Frame(notebook, padding=12)
        notebook.add(control_tab, text="Flash & Control")
        notebook.add(preview_tab, text="USB Preview")

        self._build_control_tab(control_tab)
        self._build_preview_tab(preview_tab)

        self.log = tk.Text(root, height=12, wrap="word", state="disabled", font=("Consolas", 9))
        self.log.pack(fill="both", expand=False, pady=(10, 0))
        self.sink = TextSink(self.log)
        self.after(33, self._poll_live_iq_frame)

        export_row = ttk.Frame(root)
        export_row.pack(fill="x", pady=(8, 0))
        ttk.Button(
            export_row,
            text="EXPORT CODEX BUNDLE",
            command=self.export_codex_bundle,
        ).pack(side="left")
        ttk.Label(
            export_row,
            text=f"Session: {self.session.path}",
        ).pack(side="left", padx=10)

        self.protocol("WM_DELETE_WINDOW", self.on_close)
        self.refresh_ports()
        self.update_channel_label()

    def _build_control_tab(self, tab: ttk.Frame) -> None:
        flash_box = ttk.LabelFrame(tab, text="Firmware", padding=10)
        flash_box.pack(fill="x")

        self.flash_btn = ttk.Button(flash_box, text="FLASH C5VRX", command=self.flash)
        self.flash_btn.pack(side="left", fill="x", expand=True, ipady=7)
        self.progress = ttk.Progressbar(flash_box, mode="indeterminate", length=170)
        self.progress.pack(side="left", padx=(10, 0))

        ttk.Label(
            flash_box,
            text=(f"Board-specific {self.firmware_profile['flash_size_mb']} MB firmware. "
                  "Band/channel changes happen live over USB after flashing."),
        ).pack(anchor="w", pady=(8, 0))

        channel_box = ttk.LabelFrame(tab, text="Analog FPV channel", padding=10)
        channel_box.pack(fill="x", pady=(12, 0))

        row = ttk.Frame(channel_box)
        row.pack(fill="x")

        ttk.Label(row, text="Band").pack(side="left")
        self.band_var = tk.StringVar(value="A")
        band_combo = ttk.Combobox(row, textvariable=self.band_var, state="readonly", values=list(FPV_BANDS), width=6)
        band_combo.pack(side="left", padx=(6, 16))
        band_combo.bind("<<ComboboxSelected>>", lambda _e: self.update_channel_label())

        ttk.Label(row, text="Channel").pack(side="left")
        self.channel_var = tk.StringVar(value="1")
        channel_combo = ttk.Combobox(row, textvariable=self.channel_var, state="readonly", values=[str(i) for i in range(1, 9)], width=6)
        channel_combo.pack(side="left", padx=(6, 16))
        channel_combo.bind("<<ComboboxSelected>>", lambda _e: self.update_channel_label())

        ttk.Label(row, text="RX bandwidth").pack(side="left")
        self.bw_var = tk.StringVar(value="40")
        ttk.Combobox(row, textvariable=self.bw_var, state="readonly", values=["40", "20"], width=7).pack(side="left", padx=(6, 16))

        self.apply_btn = ttk.Button(row, text="Apply channel", command=self.apply_channel)
        self.apply_btn.pack(side="right")

        self.channel_info_var = tk.StringVar()
        ttk.Label(channel_box, textvariable=self.channel_info_var).pack(anchor="w", pady=(8, 0))

        capture_box = ttk.LabelFrame(tab, text="RF / IQ research", padding=10)
        capture_box.pack(fill="x", pady=(12, 0))

        cap_row = ttk.Frame(capture_box)
        cap_row.pack(fill="x")
        ttk.Label(cap_row, text="Finite IQ samples").pack(side="left")
        self.samples_var = tk.StringVar(value="16384")
        ttk.Combobox(
            cap_row,
            textvariable=self.samples_var,
            state="readonly",
            values=["1024", "2048", "4096", "8192", "16384"],
            width=10,
        ).pack(side="left", padx=8)
        self.capture_btn = ttk.Button(cap_row, text="Capture IQ", command=self.capture_iq)
        self.capture_btn.pack(side="left")
        ttk.Button(cap_row, text="Status", command=lambda: self.send_command("STATUS")).pack(side="left", padx=8)

        ttk.Label(
            capture_box,
            text="CAPTURE uses the recovered Espressif RF-test dump path. It is a finite diagnostic capture, not continuous video yet.",
        ).pack(anchor="w", pady=(8, 0))

        diag_box = ttk.LabelFrame(tab, text="First hardware diagnostics", padding=10)
        diag_box.pack(fill="x", pady=(12, 0))
        ttk.Button(
            diag_box,
            text="FIRST HARDWARE TEST",
            command=self.first_hardware_test,
        ).pack(side="left", ipady=5)
        ttk.Label(
            diag_box,
            text="Runs fail-closed cadence, wrap, phase, BitScrambler and staged soak tests. Use a coherent RF tone for phase evidence.",
            wraplength=610,
        ).pack(side="left", padx=12)

    def _build_preview_tab(self, tab: ttk.Frame) -> None:
        ttk.Label(tab, text="USB-C signal preview", font=("Segoe UI", 14, "bold")).pack(anchor="w")
        ttk.Label(
            tab,
            text=(
                "Displays a 160×120 PAL/NTSC video-proof raster with burst-locked "
                "color and an automatic monochrome fallback. The old blue WBFM "
                "diagnostic graph is disabled in this build."
            ),
            wraplength=760,
        ).pack(anchor="w", pady=(4, 10))

        self.preview_canvas = tk.Canvas(tab, height=280, background="black", highlightthickness=1)
        self.preview_canvas.pack(fill="both", expand=True)
        self.preview_canvas.bind("<Configure>", self._redraw_preview)

        self.preview_status_var = tk.StringVar(
            value=f"{APP_BUILD}: waiting for video-proof capture")
        ttk.Label(tab, textvariable=self.preview_status_var).pack(anchor="w", pady=(8, 0))

        preview_controls = ttk.Frame(tab)
        preview_controls.pack(fill="x", pady=(8, 0))
        self.capture_16k_btn = ttk.Button(
            preview_controls, text="Capture 16K IQ", command=self.capture_iq_16k)
        self.capture_16k_btn.pack(side="left")
        self.experimental_live_btn = ttk.Button(
            preview_controls,
            text="Start USB live preview (safe)",
            command=self.start_experimental_usb_preview,
        )
        self.experimental_live_btn.pack(side="left", padx=8)
        ttk.Button(preview_controls, text="Stop live preview", command=self.stop_usb_preview).pack(side="left")
        ttk.Button(
            preview_controls,
            text="Measure CVBS lock (5 s)",
            command=lambda: self.send_command("CVBS LOCK PROBE 5000"),
        ).pack(side="left", padx=8)
        ttk.Button(preview_controls, text="Clear", command=self.clear_preview).pack(side="left", padx=8)

        iq_live_controls = ttk.Frame(tab)
        iq_live_controls.pack(fill="x", pady=(8, 0))
        self.live_iq_start_btn = ttk.Button(
            iq_live_controls,
            text="ULTRA-SLOW VIDEO PROOF (A1)",
            command=self.start_live_iq_video,
        )
        self.live_iq_start_btn.pack(side="left")
        self.live_iq_fast_start_btn = ttk.Button(
            iq_live_controls,
            text="FAST VIDEO PROOF (A1)",
            command=self.start_fast_live_iq_video,
        )
        self.live_iq_fast_start_btn.pack(side="left", padx=(8, 0))
        self.live_iq_stop_btn = ttk.Button(
            iq_live_controls,
            text="STOP IQ VIDEO",
            command=self.stop_live_iq_video,
            state="disabled",
        )
        self.live_iq_stop_btn.pack(side="left", padx=8)
        ttk.Label(iq_live_controls, text="Raster clock").pack(side="left", padx=(8, 3))
        self.video_sample_rate_var = tk.StringVar(value="80")
        ttk.Combobox(
            iq_live_controls,
            textvariable=self.video_sample_rate_var,
            state="readonly",
            values=["20", "40", "80"],
            width=5,
        ).pack(side="left")
        ttk.Label(iq_live_controls, text="Standard").pack(
            side="left", padx=(8, 3))
        self.video_standard_var = tk.StringVar(value="Auto")
        ttk.Combobox(
            iq_live_controls,
            textvariable=self.video_standard_var,
            state="readonly",
            values=["Auto", "PAL", "NTSC"],
            width=6,
        ).pack(side="left")
        ttk.Label(
            iq_live_controls,
            text="MS/s; rejects unlocked noise and slowly fills a detected video raster.",
        ).pack(side="left", padx=8)

    def refresh_ports(self) -> None:
        selected_device = self.selected_port()
        ports = list(list_ports.comports())
        values = [f"{p.device}  —  {p.description}" for p in ports]
        self.port_combo["values"] = values
        if selected_device:
            for i, value in enumerate(values):
                if value.split()[0] == selected_device:
                    self.port_combo.current(i)
                    return
        if values:
            self.port_combo.current(0)
        else:
            self.port_var.set("")

    def selected_port(self) -> str | None:
        value = self.port_var.get().strip()
        return value.split()[0] if value else None

    def update_channel_label(self) -> None:
        try:
            band = self.band_var.get()
            ch = int(self.channel_var.get())
            mhz = FPV_BANDS[band][ch - 1]
        except Exception:
            return
        if mhz <= C5_RX_MAX_MHZ:
            exact_note = "inside ESP32-C5 specified RX window"
        else:
            exact_note = "OUTSIDE ESP32-C5 specified RX window"
        self.channel_info_var.set(f"{band}{ch}  •  {mhz} MHz  •  {exact_note}")

    def toggle_connect(self) -> None:
        if self.ser and self.ser.is_open:
            self.disconnect_serial()
        else:
            self.connect_serial(show_errors=True)

    def connect_serial(self, show_errors: bool = False) -> bool:
        port = self.selected_port()
        if not port:
            if show_errors:
                messagebox.showerror(APP_TITLE, "No USB/COM port selected.")
            return False

        self.disconnect_serial()
        self.profile_mismatch_warned = False
        # Capabilities belong to the newly connected firmware.  Do not carry
        # Phase8 support over when the user reconnects an older build.
        self.device_phase8_supported = False
        candidate: serial.Serial | None = None
        try:
            # Configure native USB-Serial/JTAG control lines before opening.
            # serial.Serial(port, ...) opens first with the pyserial defaults;
            # on ESP32-C5 that DTR/RTS transition can reset the device and make
            # the just-opened Windows handle stale before the first PING.
            candidate = serial.Serial()
            candidate.port = port
            candidate.baudrate = 115200
            # A finite capture ends with much less than a 4 KiB read.  The old
            # 150 ms timeout therefore became a 150 ms dead period whenever a
            # single lost marker drained the command queue.  Fast proof needs
            # a short tail timeout and immediate draining of available bytes.
            candidate.timeout = 0.01
            # Commands are tiny, but Windows can briefly backpressure OUT
            # transfers while native USB is delivering a large binary frame.
            # A short timeout plus bounded retry keeps the UI responsive.
            candidate.write_timeout = 0.5
            candidate.dtr = False
            candidate.rts = False
            candidate.open()
            self.ser = candidate
            self.usb_transport_stalled = False
            self.serial_stop.clear()
            self.serial_thread = threading.Thread(target=self._serial_reader, daemon=True)
            self.serial_thread.start()
            self.connection_var.set(f"Connected: {port}")
            self.connect_btn.configure(text="Disconnect")
            self.sink.write(f"\n=== CONNECTED: {port} ===\n")
            self.after(250, lambda: self.send_command("PING"))
            self.after(450, lambda: self.send_command("STATUS"))
            return True
        except Exception as exc:
            if candidate is not None:
                try:
                    candidate.close()
                except Exception:
                    pass
            self.ser = None
            self.connection_var.set("Disconnected")
            self.connect_btn.configure(text="Connect")
            if show_errors:
                messagebox.showerror(APP_TITLE, f"Could not open {port}:\n\n{exc}")
            return False

    def disconnect_serial(self) -> None:
        self.serial_stop.set()
        self.live_iq_active = False
        self.experimental_live_pending = False
        self.experimental_live_active = False
        self.usb_preview_active = False
        self.preview_sequence = None
        ser = self.ser
        self.ser = None
        if ser:
            try:
                with self.serial_write_lock:
                    ser.close()
            except Exception:
                pass
        self.connection_var.set("Disconnected")
        self.connect_btn.configure(text="Connect")

    def _serial_reader(self) -> None:
        ser = self.ser
        if not ser:
            return
        decoder = StreamDecoder()
        try:
            while not self.serial_stop.is_set() and ser.is_open:
                waiting = min(65536, ser.in_waiting)
                raw = ser.read(waiting if waiting else 1)
                if not raw:
                    continue
                # Coalesce bytes that arrived with the first byte.  This keeps
                # durable session recording efficient without waiting for an
                # arbitrary fixed-size read to fill.
                waiting = min(65536 - len(raw), ser.in_waiting)
                if waiting:
                    raw += ser.read(waiting)
                self.session.record_raw(raw)
                for kind, value in decoder.feed(raw):
                    if kind == "line":
                        line = str(value)
                        self.session.record_line(line)
                        self.sink.write(line + "\n")
                        self._parse_device_line(line)
                    elif kind == "packet":
                        assert isinstance(value, Packet)
                        self.session.record_packet(value)
                        self._handle_usb_packet(value)
                    else:
                        self.session.record_error("USB_PROTOCOL", str(value))
                        self.sink.write(
                            f"C5VRX_PREVIEW_RESYNC reason={value}\n")
                        if (self.live_iq_active and
                                str(value).startswith("PAYLOAD_CRC")):
                            self.live_iq_transport_losses += 1
                            self._note_live_iq_transport_progress()
        except Exception as exc:
            if not self.serial_stop.is_set():
                self.session.record_error("SERIAL_READER", str(exc))
                self.sink.write(f"\nSerial reader stopped: {exc}\n")
                self.after(0, self._serial_lost)

    def _serial_lost(self) -> None:
        self.disconnect_serial()

    def _parse_device_line(self, line: str) -> None:
        if (self.live_iq_active and line.startswith((
                "C5VRX_CAPTURE_KERNEL",
                "C5VRX_IQ_BINARY_BEGIN",
                "C5VRX_IQ_BINARY_END",
                "C5VRX_PHASE8_CAPTURE_BEGIN",
                "C5VRX_PHASE8_BINARY_BEGIN",
                "C5VRX_PHASE8_BINARY_END",
                "C5VRX_CAPTURE_DONE",
                "C5VRX_PHASE8_CAPTURE_DONE"))):
            self._note_live_iq_transport_progress()
        if line.startswith(("C5VRX_READY", "C5VRX_STATUS")):
            fields = self._fields(line)
            self.device_phase8_supported = \
                fields.get("phase8_capture") == "1"
        if line.startswith("C5VRX_LIVE_EXPERIMENTAL_START"):
            fields = self._fields(line)
            self.experimental_live_pending = False
            if fields.get("code") == "0":
                self.experimental_live_active = True
                self.usb_preview_active = True
                self.usb_preview_receiving = False
                block_words = fields.get("block_words", "adaptive")
                self.after(
                    0, self.preview_status_var.set,
                    f"Experimental USB live started ({block_words} ring words); "
                    "opening preview transport")
                if self.send_command("USB PREVIEW START"):
                    self.after(250, self._usb_preview_keepalive)
            else:
                self.experimental_live_active = False
                self.usb_preview_active = False
                self.after(
                    0, self.preview_status_var.set,
                    f"Experimental USB live failed: {line}")
            return
        if (line.startswith("C5VRX_USB_PREVIEW state=START") and
                self.experimental_live_active):
            fields = self._fields(line)
            if fields.get("code") == "0":
                self.after(
                    0, self.preview_status_var.set,
                    "Experimental USB live preview running; acquiring CVBS lock")
            else:
                self.after(0, self.stop_usb_preview)
            return
        if (line.startswith("C5VRX_USB_PREVIEW state=START") and
                self.live_iq_active):
            fields = self._fields(line)
            if fields.get("code") == "0":
                if not self.live_iq_transport_ready:
                    self.live_iq_transport_ready = True
                    self.live_video_pending_status = (
                        "Binary IQ transport ready; requesting fresh A1 capture")
                    # This parser already runs on the serial thread. Queue the
                    # first captures directly so a busy Tk renderer cannot
                    # hold the transport idle.
                    self._refill_live_iq_pipeline()
            else:
                self.after(0, self.stop_live_iq_video,
                           f"binary transport failed ({line})")
            return
        if (self.first_test_active and not self.first_test_fine_sent
                and line.startswith("C5VRX_PRODUCER_CADENCE mode=0 ")):
            fields = self._fields(line)
            if fields.get("classification") == "MEASURED":
                rate = int(fields.get("complex_samples_per_sec", "0"))
                if rate:
                    center = self.first_test_center
                    tone = center + 2
                    self.first_test_fine_sent = True
                    fine_command = f"FINE TUNE VERIFY {center} {tone} {rate}"
                    if not self.send_command(fine_command):
                        self.sink.write(
                            "C5VRX_FINE_TUNE_AUTOMATION_FAILED "
                            "error=USB_COMMAND_WRITE_FAILED\n")
        if line.startswith("C5VRX_IQ_BEGIN"):
            self.iq_words = []
            self.iq_capture_active = True
            self.after(0, self.preview_status_var.set, "Receiving IQ samples over USB-C...")
            return
        if line.startswith("C5VRX_CAPTURE_KERNEL"):
            fields = self._fields(line)
            try:
                if fields.get("done") == "1":
                    self.live_iq_source_msps = float(fields["finite_fill_msps"])
            except (KeyError, ValueError):
                pass
        if line.startswith(("C5VRX_CAPTURE_DONE",
                            "C5VRX_PHASE8_CAPTURE_DONE")) and \
                self.live_iq_active:
            self.live_iq_capture_done = "code=0" in line
            fast_phase8_credit_pump = (
                self.live_iq_fast_mode and self.device_phase8_supported)
            if not fast_phase8_credit_pump or not self.live_iq_capture_done:
                self.live_iq_commands_outstanding = max(
                    0, self.live_iq_commands_outstanding - 1)
            self.live_iq_packet_done = False
            if not self.live_iq_capture_done:
                fields = self._fields(line)
                if (fields.get("code") == "263" and
                        self.live_iq_capture_retries < self.live_iq_max_retries):
                    self.live_iq_capture_retries += 1
                    retry = self.live_iq_capture_retries
                    self.after(
                        0, self.preview_status_var.set,
                        f"RF capture timeout; retry {retry}/{self.live_iq_max_retries}")
                    self._schedule_live_iq_refill(100)
                else:
                    self.after(0, self.stop_live_iq_video,
                               f"device capture failed after {self.live_iq_capture_retries} retries")
            else:
                self.live_iq_capture_retries = 0
                # A completion line is emitted only after every binary chunk
                # has been handed to USB.  Refill here instead of on the first
                # chunk so commands cannot accumulate behind an active dump.
                if fast_phase8_credit_pump:
                    # Fast Phase8 refills when a new capture ID is observed,
                    # not here. Completion lines can be lost during a protocol
                    # resync; using them as credits permanently drained the
                    # real firmware queue while the host still believed it was
                    # full.
                    pass
                elif self.live_iq_fast_mode:
                    # _parse_device_line runs on the serial-reader thread.
                    # Refill directly; routing this through Tk added about
                    # 230 ms of idle time whenever PhotoImage was rendering.
                    self._refill_live_iq_pipeline()
                else:
                    self._schedule_live_iq_refill(250)
        if line.startswith("IQ:") and self.iq_capture_active:
            try:
                self.iq_words.append(int(line[3:], 16))
            except ValueError:
                pass
            return
        if line == "C5VRX_IQ_END":
            self.iq_capture_active = False
            words = len(self.iq_words)
            self.after(0, self.preview_status_var.set, f"Captured {words} complex IQ samples — rendering FM discriminator")
            self.after(0, self.render_iq_preview)
            return
        if line.startswith("C5VRX_STATUS") or line.startswith("C5VRX_OK set"):
            self.after(0, self._apply_status_line, line)

    @staticmethod
    def _fields(line: str) -> dict[str, str]:
        fields: dict[str, str] = {}
        for part in line.split():
            if "=" in part:
                key, value = part.split("=", 1)
                fields[key] = value
        return fields

    def _handle_usb_packet(self, packet: Packet) -> None:
        if packet.packet_type in {PACKET_IQ_U32_BLOCK, PACKET_IQ_U32_CHUNK}:
            self._handle_iq_usb_packet(packet)
            return
        if packet.packet_type == PACKET_PHASE8_CHUNK:
            self._handle_phase8_usb_packet(packet)
            return
        if self.preview_sequence is not None and packet.sequence != (
                self.preview_sequence + 1) & 0xFFFFFFFF:
            self.sink.write(
                "C5VRX_PREVIEW_SEQUENCE_GAP "
                f"expected={(self.preview_sequence + 1) & 0xFFFFFFFF} "
                f"received={packet.sequence}\n")
        self.preview_sequence = packet.sequence

        if packet.packet_type == PACKET_STREAM_END:
            dropped = int.from_bytes(packet.payload[:8], "little") \
                if len(packet.payload) >= 8 else 0
            self.after(0, self.preview_status_var.set,
                       f"USB preview stopped; device dropped {dropped} frame(s)")
            return
        if packet.packet_type not in {
                PACKET_STREAM_INFO, PACKET_GRAY8_FRAME, PACKET_YUV411_FRAME}:
            self.sink.write(
                f"C5VRX_PREVIEW_SKIP packet_type={packet.packet_type}\n")
            return
        if len(packet.payload) < FRAME_DESCRIPTOR.size:
            self.sink.write("C5VRX_PREVIEW_DROP reason=SHORT_DESCRIPTOR\n")
            return

        width, height, stride, pixel_format, flags = \
            FRAME_DESCRIPTOR.unpack_from(packet.payload)
        if (not width or not height or width > 640 or height > 480 or
                pixel_format not in {PIXEL_FORMAT_GRAY8, PIXEL_FORMAT_YUV411} or
                (pixel_format == PIXEL_FORMAT_GRAY8 and stride < width) or
                (pixel_format == PIXEL_FORMAT_YUV411 and
                 stride < width * 3 // 2)):
            self.sink.write("C5VRX_PREVIEW_DROP reason=UNSUPPORTED_FORMAT\n")
            return
        if packet.packet_type == PACKET_STREAM_INFO:
            self.preview_sequence = packet.sequence
            self.after(0, self.preview_status_var.set,
                       f"USB preview: {width}×{height} "
                       f"{'YUV411 color' if pixel_format == PIXEL_FORMAT_YUV411 else 'GRAY8'}, waiting for lock")
            return

        pixels = packet.payload[FRAME_DESCRIPTOR.size:]
        if len(pixels) != stride * height:
            self.sink.write("C5VRX_PREVIEW_DROP reason=PAYLOAD_SIZE\n")
            return
        self.usb_preview_receiving = True
        if pixel_format == PIXEL_FORMAT_YUV411:
            try:
                rgb = yuv411_to_rgb(pixels, width, height, stride)
            except ValueError as exc:
                self.sink.write(f"C5VRX_PREVIEW_DROP reason={exc}\n")
                return
            self.after(
                0, self._show_yuv_rgb_frame, rgb, width, height,
                bool(flags & 2))
        else:
            if stride < width:
                self.sink.write("C5VRX_PREVIEW_DROP reason=STRIDE\n")
                return
            if stride != width:
                pixels = b"".join(
                    pixels[row * stride:row * stride + width]
                    for row in range(height)
                )
            self.after(0, self._show_gray_frame, pixels, width, height)
        if not (flags & 1):
            self.sink.write("C5VRX_PREVIEW_WARNING reason=SYNC_UNLOCKED\n")

    def _handle_iq_usb_packet(self, packet: Packet) -> None:
        self.live_iq_usb_bytes += 32 + len(packet.payload) + 4
        self._note_live_iq_transport_progress()
        if packet.packet_type == PACKET_IQ_U32_BLOCK:
            try:
                words = decode_iq_block(packet)
            except ValueError as exc:
                self.sink.write(f"C5VRX_IQ_PACKET_DROP reason={exc}\n")
                return
            self._finish_iq_usb_block(words, packet.timestamp_us)
            return

        try:
            chunk = decode_iq_chunk(packet)
        except ValueError as exc:
            self.sink.write(f"C5VRX_IQ_CHUNK_DROP reason={exc}\n")
            return
        if self.live_iq_capture_id != chunk.capture_id:
            self.live_iq_capture_id = chunk.capture_id
            self.live_iq_capture_timestamp_us = packet.timestamp_us
            self.live_iq_total_words = chunk.total_words
            self.live_iq_chunks = {}
        if chunk.total_words != self.live_iq_total_words:
            self.sink.write("C5VRX_IQ_CHUNK_DROP reason=MIXED_TOTAL\n")
            return
        existing = self.live_iq_chunks.get(chunk.offset_words)
        if existing is not None and existing != chunk.words:
            self.sink.write("C5VRX_IQ_CHUNK_DROP reason=CONFLICTING_DUPLICATE\n")
            return
        self.live_iq_chunks[chunk.offset_words] = chunk.words

        if sum(len(words) for words in self.live_iq_chunks.values()) != chunk.total_words:
            return
        assembled: list[int] = []
        expected_offset = 0
        for offset in sorted(self.live_iq_chunks):
            if offset != expected_offset:
                return
            words = self.live_iq_chunks[offset]
            assembled.extend(words)
            expected_offset += len(words)
        if expected_offset == chunk.total_words:
            self._finish_iq_usb_block(
                assembled, self.live_iq_capture_timestamp_us)

    def _handle_phase8_usb_packet(self, packet: Packet) -> None:
        self.live_iq_usb_bytes += 32 + len(packet.payload) + 4
        self._note_live_iq_transport_progress()
        try:
            chunk = decode_phase8_chunk(packet)
        except ValueError as exc:
            self.sink.write(f"C5VRX_PHASE8_CHUNK_DROP reason={exc}\n")
            return
        if self.live_iq_capture_id != chunk.capture_id:
            self.live_iq_capture_id = chunk.capture_id
            self.live_iq_capture_timestamp_us = packet.timestamp_us
            self.live_iq_total_words = chunk.total_samples
            self.live_phase8_chunks = {}
            if self.live_iq_active and self.live_iq_fast_mode:
                # The first valid packet from a capture proves that firmware
                # consumed one command. Replace that credit immediately. This
                # is resilient to a lost CAPTURE_DONE line and behaves like a
                # bounded version of repeatedly clicking Capture 16K.
                self.live_iq_commands_outstanding = max(
                    0, self.live_iq_commands_outstanding - 1)
                self._refill_live_iq_pipeline()
        if chunk.total_samples != self.live_iq_total_words:
            self.sink.write("C5VRX_PHASE8_CHUNK_DROP reason=MIXED_TOTAL\n")
            return
        existing = self.live_phase8_chunks.get(chunk.offset_samples)
        if existing is not None and existing != chunk.phases:
            self.sink.write(
                "C5VRX_PHASE8_CHUNK_DROP reason=CONFLICTING_DUPLICATE\n")
            return
        self.live_phase8_chunks[chunk.offset_samples] = chunk.phases
        if sum(len(data) for data in self.live_phase8_chunks.values()) != \
                chunk.total_samples:
            return
        assembled = bytearray()
        expected_offset = 0
        for offset in sorted(self.live_phase8_chunks):
            if offset != expected_offset:
                return
            data = self.live_phase8_chunks[offset]
            assembled.extend(data)
            expected_offset += len(data)
        if expected_offset == chunk.total_samples:
            self._finish_phase8_usb_block(
                bytes(assembled), self.live_iq_capture_timestamp_us)

    def _finish_iq_usb_block(
            self, words: list[int], timestamp_us: int | None = None) -> None:
        self.iq_words = words
        self.iq_capture_active = False
        self.live_iq_packet_done = True
        if self.live_iq_active:
            self.live_iq_blocks += 1
            self._enqueue_live_iq_processing("raw", words, timestamp_us)
        else:
            self.after(0, self.preview_status_var.set,
                       f"Captured {len(words)} CRC-valid IQ words")
            self.after(0, self.render_iq_preview)

    def _finish_phase8_usb_block(
            self, phases: bytes, timestamp_us: int | None = None) -> None:
        self.iq_capture_active = False
        self.live_iq_packet_done = True
        if self.live_iq_active:
            self.live_iq_blocks += 1
            self._enqueue_live_iq_processing("phase8", phases, timestamp_us)
        else:
            self.after(
                0, self.preview_status_var.set,
                f"Captured {len(phases)} CRC-valid phase8 samples")

    def _apply_status_line(self, line: str) -> None:
        fields = self._fields(line)
        device_profile = fields.get("profile")
        profile_error = None
        if (line.startswith("C5VRX_STATUS") and not device_profile and
                self.expected_profile != "legacy-unknown"):
            profile_error = "device did not report a board profile"
        elif (line.startswith("C5VRX_STATUS") and device_profile and
              self.expected_profile not in {"legacy-unknown", device_profile}):
            profile_error = f"device reports {device_profile}"

        if profile_error:
            self.connection_var.set(
                f"PROFILE MISMATCH: console={self.expected_profile}; {profile_error}")
            if not self.profile_mismatch_warned:
                self.profile_mismatch_warned = True
                messagebox.showwarning(
                    APP_TITLE,
                    "The connected firmware uses a different board profile.\n\n"
                    f"Console bundle: {self.expected_profile}\n"
                    f"Device: {profile_error}\n\n"
                    "Do not connect the AV resistor network until the firmware "
                    "and physical board mapping match.",
                )
        elif line.startswith("C5VRX_STATUS") and device_profile:
            heap_text = ""
            try:
                free_kib = int(fields.get("heap_internal_free", "0")) // 1024
                dma_kib = int(fields.get("heap_dma_largest", "0")) // 1024
                if free_kib and dma_kib:
                    heap_text = f" — heap {free_kib} KiB, largest DMA {dma_kib} KiB"
            except ValueError:
                pass
            self.connection_var.set(
                f"Connected: {self.selected_port()} — profile verified: "
                f"{device_profile}{heap_text}")
        band = fields.get("band")
        ch = fields.get("channel")
        bw = fields.get("bw")
        if band in FPV_BANDS:
            self.band_var.set(band)
        if ch in {str(i) for i in range(1, 9)}:
            self.channel_var.set(ch)
        if bw in {"20", "40"}:
            self.bw_var.set(bw)
        self.update_channel_label()

    def _write_serial_bytes(self, payload: bytes, retries: int = 2) -> bool:
        """Serialize native-USB writes and tolerate bounded Windows stalls."""
        if self.usb_transport_stalled:
            return False
        last_error: Exception | None = None
        for attempt in range(retries + 1):
            ser = self.ser
            if not ser or not ser.is_open:
                return False
            try:
                with self.serial_write_lock:
                    if ser is not self.ser or not ser.is_open:
                        return False
                    written = ser.write(payload)
                if written == len(payload):
                    return True
                last_error = serial.SerialTimeoutException(
                    f"short USB write {written}/{len(payload)}")
            except serial.SerialTimeoutException as exc:
                last_error = exc
            except (OSError, serial.SerialException) as exc:
                last_error = exc
                self.session.record_error("USB_COMMAND_WRITE", str(exc))
                self.after(0, self._serial_lost)
                return False
            if attempt < retries:
                self.usb_write_retries += 1
                time.sleep(0.025 * (attempt + 1))
        self.usb_write_failures += 1
        self.session.record_error("USB_COMMAND_WRITE_TIMEOUT", str(last_error))
        self.sink.write(
            "C5VRX_HOST_USB_WRITE_TIMEOUT "
            f"bytes={len(payload)} retries={retries} "
            f"total_failures={self.usb_write_failures}\n")
        self.usb_transport_stalled = True
        self.after(0, self._show_usb_transport_stall)
        return False

    def _show_usb_transport_stall(self) -> None:
        """Stop command producers after one wedged native-USB transfer."""
        self.live_iq_active = False
        self.experimental_live_pending = False
        self.experimental_live_active = False
        self.usb_preview_active = False
        self.usb_preview_receiving = False
        self.live_iq_transport_ready = False
        self.live_iq_commands_outstanding = 0
        self.live_iq_refill_pending = False
        self.live_iq_watchdog_generation += 1
        self.live_iq_processing_generation += 1
        if hasattr(self, "experimental_live_btn"):
            self.experimental_live_btn.configure(state="normal")
            self.live_iq_start_btn.configure(state="normal")
            self.live_iq_fast_start_btn.configure(state="normal")
            self.live_iq_stop_btn.configure(state="disabled")
            self.capture_btn.configure(state="normal")
            self.capture_16k_btn.configure(state="normal")
        self.preview_status_var.set(
            "USB transport stopped after one write timeout; reconnect or "
            "power-cycle the XIAO before retrying")

    def send_command(self, command: str) -> bool:
        normalized = command.strip()
        ser = self.ser
        if not ser or not ser.is_open:
            self.after(
                0, messagebox.showwarning,
                APP_TITLE, "Connect to C5VRX first.")
            return False
        if self.usb_transport_stalled:
            return False
        self.session.record_command(normalized)
        self.sink.write(f"> {normalized}\n")
        return self._write_serial_bytes(
            (normalized + "\n").encode("ascii"), retries=3)

    def apply_channel(self) -> None:
        band = self.band_var.get()
        ch = int(self.channel_var.get())
        mhz = FPV_BANDS[band][ch - 1]
        if mhz > C5_RX_MAX_MHZ:
            messagebox.showwarning(
                APP_TITLE,
                f"{band}{ch} is {mhz} MHz, above the current ESP32-C5 specified RX limit of {C5_RX_MAX_MHZ} MHz.",
            )
            return
        self.send_command(f"BW {self.bw_var.get()}")
        self.after(100, lambda: self.send_command(f"SET {band} {ch}"))

    def capture_iq(self) -> None:
        self.session.next_iq_label("receiver-console-capture")
        self.send_command(f"CAPTURE {int(self.samples_var.get())}")

    def capture_iq_16k(self) -> None:
        self.samples_var.set("16384")
        self.capture_iq()

    def start_experimental_usb_preview(self) -> None:
        # Continuous RF dump ownership removes CPU access to HP-SRAM on the
        # C5. Real XIAO hardware proved that issuing the old experimental ring
        # command wedges native USB. Use bounded LP-kernel Phase8 captures;
        # every capture restores CPU SRAM ownership before USB transmission.
        self._start_live_iq_video(fast=False)

    def stop_usb_preview(self) -> None:
        if self.live_iq_active:
            self.stop_live_iq_video("user")
            return
        was_experimental = (
            self.experimental_live_pending or self.experimental_live_active)
        self.experimental_live_pending = False
        self.experimental_live_active = False
        self.usb_preview_active = False
        self.usb_preview_receiving = False
        if was_experimental:
            self.send_command("LIVE STOP")
            self.after(100, lambda: self.send_command("USB PREVIEW STOP"))
        else:
            self.send_command("USB PREVIEW STOP")

    def _usb_preview_keepalive(self) -> None:
        if (not self.usb_preview_active or
                not self.ser or not self.ser.is_open):
            return
        # Missing one 250 ms keepalive is safe with the firmware's 750 ms
        # lease. Serialize it with capture commands and retry once rather than
        # racing pyserial writes from the Tk and reader threads.
        self._write_serial_bytes(b"USB PREVIEW KEEPALIVE\n", retries=1)
        self.after(250, self._usb_preview_keepalive)

    def start_live_iq_video(self) -> None:
        self._start_live_iq_video(fast=False)

    def start_fast_live_iq_video(self) -> None:
        self._start_live_iq_video(fast=True)

    def _start_live_iq_video(self, fast: bool) -> None:
        if not self.ser or not self.ser.is_open:
            messagebox.showwarning(APP_TITLE, "Connect to C5VRX first.")
            return
        if self.usb_transport_stalled:
            messagebox.showwarning(
                APP_TITLE,
                "The USB transport stalled. Reconnect or power-cycle the "
                "XIAO before starting another preview.")
            return
        if self.live_iq_active:
            return
        self.live_iq_active = True
        self.usb_preview_active = True
        self.usb_preview_receiving = False
        self.live_iq_fast_mode = fast
        # Two Phase8 credits overlap RF acquisition with USB draining without
        # saturating the native USB command endpoint with a four-command burst.
        self.live_iq_pipeline_target = 2 if fast else 1
        self.live_iq_commands_outstanding = 0
        self.live_iq_refill_pending = False
        self.live_iq_last_transport_progress = time.monotonic()
        self.live_iq_watchdog_generation += 1
        self.live_iq_recoveries = 0
        self.live_iq_transport_losses = 0
        self.live_iq_processing_generation += 1
        self.live_iq_processing_queue = queue.Queue(maxsize=4)
        self.live_iq_processing_drops = 0
        self.live_iq_capture_done = False
        self.live_iq_packet_done = False
        self.live_iq_capture_id = None
        self.live_iq_capture_timestamp_us = None
        self.live_iq_chunks = {}
        self.live_phase8_chunks = {}
        self.live_iq_source_msps = None
        self.live_iq_transport_ready = False
        self.live_iq_capture_retries = 0
        self.live_iq_usb_started = time.monotonic()
        self.live_iq_usb_bytes = 0
        self.live_iq_blocks = 0
        self.live_video_host_frames = 0
        self.live_video_pixels[:] = bytes(len(self.live_video_pixels))
        self.live_video_rgb_pixels[:] = bytes(len(self.live_video_rgb_pixels))
        self.live_video_row = 0
        self.live_video_rows_seen.clear()
        self.live_video_standard_scores = {"PAL": 0.0, "NTSC": 0.0}
        self.live_video_rejected_blocks = 0
        self.live_video_black = None
        self.live_video_white = None
        self.live_video_color_locked_lines = 0
        self.live_video_monochrome_lines = 0
        self.live_video_color_burst_level = 0.0
        self.live_video_color_burst_coherence = 0.0
        self.live_video_color_line_counter = 0
        self.live_video_color_bias = {"PAL": 0j, "NTSC": 0j}
        self.live_video_color_bias_samples = {"PAL": 0, "NTSC": 0}
        self.live_video_color_valid_rows.clear()
        self.live_video_display_offset = 0
        self.live_video_vertical_pending = None
        self.live_video_vertical_pending_hits = 0
        self.live_video_requested_standard = self.video_standard_var.get()
        self.live_video_requested_sample_rate = float(
            self.video_sample_rate_var.get())
        self.live_video_fm_polarity_votes = 0.0
        self.live_video_fm_polarity_lock = None
        self.live_video_vertical_anchor_us = None
        self.live_video_vertical_last_event_us = None
        self.live_video_vertical_lock_events = 0
        self.live_video_vertical_candidate_us = None
        self.live_video_vertical_candidate_hits = 0
        self.live_video_vertical_candidate_standard = None
        initial_standard = (
            self.live_video_requested_standard
            if self.live_video_requested_standard != "Auto" else "NTSC")
        self.live_video_field_period_us = VIDEO_FIELD_PERIOD_US[initial_standard]
        self.live_video_vertical_standard = None
        self.live_video_render_pending = False
        self.live_video_pending_frame = None
        self.live_video_pending_status = None
        self.live_iq_start_btn.configure(state="disabled")
        self.live_iq_fast_start_btn.configure(state="disabled")
        self.live_iq_stop_btn.configure(state="normal")
        self.experimental_live_btn.configure(state="disabled")
        self.capture_btn.configure(state="disabled")
        self.capture_16k_btn.configure(state="disabled")
        processing_generation = self.live_iq_processing_generation
        processing_queue = self.live_iq_processing_queue
        threading.Thread(
            target=self._live_iq_processing_worker,
            args=(processing_generation, processing_queue),
            daemon=True,
        ).start()
        # Replace any previous diagnostic drawing immediately. This also gives
        # an unmistakable visual indication that the video-only GUI build is
        # running before the first RF block arrives.
        self._show_gray_frame(
            bytes(self.live_video_pixels),
            self.live_video_width,
            self.live_video_height,
        )
        self.preview_status_var.set(
            f"{APP_BUILD}: starting {'FAST pipelined' if fast else 'single-request'} "
            "bounded Phase8 -> PAL/NTSC raster; VTX must be A1 (5865 MHz)")
        self.send_command("BW 40")
        self.after(100, lambda: self.send_command("SET A 1"))
        self.after(220, self._start_bounded_capture_transport)
        watchdog_generation = self.live_iq_watchdog_generation
        self.after(500, self._live_iq_watchdog, watchdog_generation)

    def _start_bounded_capture_transport(self) -> None:
        if (not self.live_iq_active or self.usb_transport_stalled or
                not self.ser or not self.ser.is_open):
            return
        # CAPTURE PHASE8 already uses the CRC-framed binary transport. It does
        # not require the device-side YUV preview worker or its 64 KiB buffers.
        self.live_iq_transport_ready = True
        self.live_video_pending_status = (
            "Bounded Phase8 transport ready; requesting fresh A1 capture")
        self._refill_live_iq_pipeline()

    def stop_live_iq_video(self, reason: str = "user") -> None:
        self.live_iq_active = False
        self.usb_preview_active = False
        self.usb_preview_receiving = False
        self.live_iq_watchdog_generation += 1
        self.live_iq_processing_generation += 1
        self.live_iq_transport_ready = False
        self.live_iq_commands_outstanding = 0
        self.live_iq_refill_pending = False
        self.live_video_pending_frame = None
        if hasattr(self, "live_iq_start_btn"):
            self.live_iq_start_btn.configure(state="normal")
            self.live_iq_fast_start_btn.configure(state="normal")
            self.live_iq_stop_btn.configure(state="disabled")
            self.experimental_live_btn.configure(state="normal")
            self.capture_btn.configure(state="normal")
            self.capture_16k_btn.configure(state="normal")
        self.preview_status_var.set(f"IQ video stopped: {reason}")

    def _request_live_iq_capture(self) -> bool:
        if not self.live_iq_active or not self.live_iq_transport_ready:
            return False
        self.live_iq_request_started = time.monotonic()
        if self.device_phase8_supported:
            self.live_transport_name = "phase8"
            sent = self.send_command("CAPTURE PHASE8 16384")
        else:
            self.live_transport_name = "raw-IQ32 fallback"
            sent = self.send_command("CAPTURE 16384")
        if sent:
            self.live_iq_commands_outstanding += 1
        return sent

    def _schedule_live_iq_refill(self, delay_ms: int = 0) -> None:
        if self.live_iq_refill_pending or not self.live_iq_active:
            return
        self.live_iq_refill_pending = True
        self.after(delay_ms, self._refill_live_iq_pipeline)

    def _refill_live_iq_pipeline(self) -> None:
        self.live_iq_refill_pending = False
        if not self.live_iq_active or not self.live_iq_transport_ready:
            return
        # Keep the raw-IQ fallback strictly single-request because each block
        # is four times larger than Phase8.
        target = self.live_iq_pipeline_target
        if not self.device_phase8_supported:
            target = 1
        while self.live_iq_commands_outstanding < target:
            if not self._request_live_iq_capture():
                self._schedule_live_iq_refill(100)
                break

    def _note_live_iq_transport_progress(self) -> None:
        if self.live_iq_active:
            self.live_iq_last_transport_progress = time.monotonic()

    def _enqueue_live_iq_processing(self, kind: str, payload: object,
                                    timestamp_us: int | None) -> None:
        """Keep serial draining while signal processing runs independently."""
        item = (kind, payload, timestamp_us)
        try:
            self.live_iq_processing_queue.put_nowait(item)
            return
        except queue.Full:
            pass

        # Video proof is a live diagnostic view. Prefer the newest capture to
        # blocking the USB reader behind stale rasters and losing wire bytes.
        try:
            self.live_iq_processing_queue.get_nowait()
            self.live_iq_processing_queue.task_done()
        except queue.Empty:
            pass
        self.live_iq_processing_drops += 1
        try:
            self.live_iq_processing_queue.put_nowait(item)
        except queue.Full:
            self.live_iq_processing_drops += 1

    def _live_iq_processing_worker(
            self, generation: int,
            processing_queue: queue.Queue[
                tuple[str, object, int | None]]) -> None:
        while (self.live_iq_active and
               generation == self.live_iq_processing_generation):
            try:
                kind, payload, timestamp_us = \
                    processing_queue.get(timeout=0.2)
            except queue.Empty:
                continue
            try:
                if (not self.live_iq_active or
                        generation != self.live_iq_processing_generation):
                    continue
                if kind == "phase8":
                    assert isinstance(payload, bytes)
                    self._process_live_phase8_block(payload, timestamp_us)
                else:
                    assert isinstance(payload, list)
                    self._process_live_iq_block(payload, timestamp_us)
            except Exception as exc:
                self.sink.write(
                    "C5VRX_HOST_VIDEO_PROCESSING_ERROR "
                    f"type={type(exc).__name__} detail={exc}\n")
            finally:
                processing_queue.task_done()

    def _live_iq_watchdog(self, generation: int) -> None:
        if (generation != self.live_iq_watchdog_generation or
                not self.live_iq_active):
            return
        if self.live_iq_transport_ready:
            idle_s = time.monotonic() - self.live_iq_last_transport_progress
            if idle_s >= 1.25:
                self.live_iq_recoveries += 1
                self.live_iq_commands_outstanding = 0
                self.live_iq_capture_done = False
                self.live_iq_packet_done = False
                self.live_iq_capture_id = None
                self.live_iq_chunks = {}
                self.live_phase8_chunks = {}
                self.live_iq_last_transport_progress = time.monotonic()
                self.sink.write(
                    "C5VRX_HOST_VIDEO_RECOVERY "
                    f"count={self.live_iq_recoveries} idle_ms={idle_s * 1000:.0f} "
                    f"pipeline_target={self.live_iq_pipeline_target} "
                    "reason=GHOSTED_CAPTURE_SLOTS\n")
                self.preview_status_var.set(
                    f"Recovering continuous video pipeline after {idle_s:.2f} s "
                    "without capture progress")
                self._refill_live_iq_pipeline()
        self.after(500, self._live_iq_watchdog, generation)

    @staticmethod
    def _fm_discriminator(words: list[int]) -> list[float]:
        if len(words) < 2:
            return []
        decoded = [C5VRXApp._decode_iq(raw) for raw in words]
        # The recovered dump has a sizeable, block-dependent I/Q DC offset.
        # Removing it before phase differencing prevents the carrier circle
        # from orbiting an artificial origin and makes CVBS sync plateaus much
        # easier to detect.
        mean_i = statistics.fmean(value[0] for value in decoded)
        mean_q = statistics.fmean(value[1] for value in decoded)
        values: list[float] = []
        pi = decoded[0][0] - mean_i
        pq = decoded[0][1] - mean_q
        for raw_i, raw_q in decoded[1:]:
            ci = raw_i - mean_i
            cq = raw_q - mean_q
            values.append(math.atan2(cq * pi - ci * pq,
                                     ci * pi + cq * pq))
            pi, pq = ci, cq
        return values

    @staticmethod
    def _phase8_discriminator(phases: bytes) -> list[float]:
        """Convert wrapped unsigned phase bytes into normalized FM deltas."""
        if len(phases) < 2:
            return []
        radians_per_code = 2.0 * math.pi / 256.0
        previous = phases[0]
        values: list[float] = []
        for current in phases[1:]:
            signed_delta = ((current - previous + 128) & 0xFF) - 128
            values.append(signed_delta * radians_per_code)
            previous = current
        return values

    @staticmethod
    def _sync_fold_candidates(
            binned: list[float], period_bins: int,
            limit: int = 6) -> list[tuple[int, float, int]]:
        """Return distinct sync-window candidates by normalized contrast.

        A polarity of +1 means the detected sync plateau is already below the
        blanking baseline. -1 means the RF tap is spectrally inverted and the
        discriminator must be inverted before extracting luma.
        """
        if period_bins < 8 or len(binned) < period_bins * 2:
            return [(0, 0.0, 1)]
        sums = [0.0] * period_bins
        counts = [0] * period_bins
        for index, value in enumerate(binned):
            phase = index % period_bins
            sums[phase] += value
            counts[phase] += 1
        folded = [
            sums[index] / counts[index] if counts[index] else 0.0
            for index in range(period_bins)
        ]
        sync_bins = max(2, int(round(period_bins * 0.073)))
        circular = folded + folded[:sync_bins]
        window_sum = sum(circular[:sync_bins])
        window_means = [window_sum / sync_bins]
        for phase in range(1, period_bins):
            window_sum += (
                circular[phase + sync_bins - 1] - circular[phase - 1])
            window_means.append(window_sum / sync_bins)
        baseline = statistics.fmean(binned)
        noise = max(1e-9, statistics.pstdev(binned))
        ranked: list[tuple[int, float, int]] = []
        for phase, mean in enumerate(window_means):
            low_contrast = baseline - mean
            high_contrast = mean - baseline
            if low_contrast >= high_contrast:
                ranked.append((phase, low_contrast / noise, 1))
            else:
                ranked.append((phase, high_contrast / noise, -1))
        ranked.sort(key=lambda item: item[1], reverse=True)

        # A single real sync plateau produces many adjacent high-scoring
        # windows. Keep separated alternatives so burst validation can compare
        # genuine H-sync against repeated vertical picture edges.
        selected: list[tuple[int, float, int]] = []
        for candidate in ranked:
            phase = candidate[0]
            if any(min(abs(phase - old[0]),
                       period_bins - abs(phase - old[0])) < sync_bins
                   for old in selected):
                continue
            selected.append(candidate)
            if len(selected) >= limit:
                break
        return selected or [(0, 0.0, 1)]

    @staticmethod
    def _sync_fold_candidate(
            binned: list[float], period_bins: int) -> tuple[int, float, int]:
        """Return the strongest folded candidate for compatibility/tests."""
        return C5VRXApp._sync_fold_candidates(binned, period_bins, 1)[0]

    def _color_standard_coherence(
            self, fm: list[float], standard: str, raster_msps: float,
            period: float, sync_sample: int) -> float:
        """Score PAL/NTSC using energy coherent with that standard's burst."""
        cos_values, sin_values = self._color_oscillator(
            standard, raster_msps, period)
        begin_fraction, end_fraction = VIDEO_COLOR_BURST_WINDOW[standard]
        scores: list[float] = []
        cursor = float(sync_sample)
        while cursor - period >= 0:
            cursor -= period
        while cursor + period <= len(fm):
            line_start = int(cursor)
            phasor, rms = self._quadrature_component(
                fm,
                int(cursor + period * begin_fraction),
                int(cursor + period * end_fraction),
                line_start,
                cos_values,
                sin_values)
            scores.append(abs(phasor) / max(
                1e-9, math.sqrt(2.0) * rms))
            cursor += period
        return statistics.median(scores) if scores else 0.0

    def _select_video_timing(
            self, binned: list[float], raster_msps: float,
            bin_size: int,
            fm: list[float] | None = None) -> tuple[str, float, int, float, int]:
        requested = self.live_video_requested_standard
        standards = ("PAL", "NTSC") if requested == "Auto" else (requested,)
        timing_options: dict[
            str, tuple[float, list[tuple[int, float, int]]]] = {}
        for standard in standards:
            period = raster_msps * 1_000_000.0 / VIDEO_LINE_RATES_HZ[standard]
            period_bins = max(8, int(round(period / bin_size)))
            timing_options[standard] = (
                period, self._sync_fold_candidates(binned, period_bins))

        # FM discriminator polarity is a property of the fixed RF/IF path; it
        # cannot reverse from one three-line capture to the next. Active color
        # edges can nevertheless look like a strong sync plateau and their
        # chroma can look even more coherent than the true back-porch burst.
        # Learn the session polarity from the strongest folded candidates, then
        # permanently reject opposite-polarity candidates until the user starts
        # a new proof. Waiting for three votes is harmless because picture rows
        # are already withheld until vertical sync has been acquired.
        evidence_standard = max(
            timing_options,
            key=lambda name: timing_options[name][1][0][1],
        )
        evidence_score = timing_options[evidence_standard][1][0][1]
        evidence_polarity = timing_options[evidence_standard][1][0][2]
        if (self.live_video_fm_polarity_lock is None and
                evidence_score >= VIDEO_SYNC_MIN_SCORE):
            self.live_video_fm_polarity_votes = max(
                -VIDEO_POLARITY_LOCK_VOTES,
                min(VIDEO_POLARITY_LOCK_VOTES,
                    self.live_video_fm_polarity_votes + evidence_polarity),
            )
            if abs(self.live_video_fm_polarity_votes) >= \
                    VIDEO_POLARITY_LOCK_VOTES:
                self.live_video_fm_polarity_lock = (
                    1 if self.live_video_fm_polarity_votes > 0.0 else -1)

        candidates: dict[str, tuple[float, int, float, int, float]] = {}
        for standard in standards:
            period, sync_options = timing_options[standard]
            required_polarity = (
                self.live_video_fm_polarity_lock
                if self.live_video_fm_polarity_lock is not None else
                sync_options[0][2])
            matching_options = [
                option for option in sync_options
                if option[2] == required_polarity]
            if matching_options:
                # Candidates are returned in descending sync contrast. Burst
                # remains useful for PAL/NTSC identification and color phase,
                # but it must not move H-sync into active picture content.
                phase, score, polarity = matching_options[0]
            else:
                phase, _score, polarity = sync_options[0]
                score = 0.0
            color_coherence = 0.0
            if fm is not None:
                color_coherence = self._color_standard_coherence(
                    fm, standard, raster_msps, period, phase * bin_size)
            candidates[standard] = (
                period, phase, score, polarity, color_coherence)

        if requested == "Auto":
            best_now = max(candidates, key=lambda name: candidates[name][2])
            if candidates[best_now][2] >= VIDEO_SYNC_MIN_SCORE:
                for standard in ("PAL", "NTSC"):
                    self.live_video_standard_scores[standard] = (
                        self.live_video_standard_scores[standard] * 0.85 +
                        candidates[standard][2])
            standard = max(
                self.live_video_standard_scores,
                key=self.live_video_standard_scores.__getitem__)
            if not any(self.live_video_standard_scores.values()):
                standard = best_now
            # PAL and NTSC line rates differ by less than one percent, so a
            # three-line finite capture cannot always separate them by H-sync
            # alone. Their color subcarriers are far apart. A coherent burst
            # therefore supplies a much stronger and faster standard decision.
            color_standard = max(
                candidates, key=lambda name: candidates[name][4])
            other_standard = (
                "NTSC" if color_standard == "PAL" else "PAL")
            color_score = candidates[color_standard][4]
            other_score = candidates[other_standard][4]
            if color_score >= 0.15 and color_score >= other_score + 0.08:
                standard = color_standard
        else:
            standard = requested
        period, phase, score, polarity, _color_coherence = candidates[standard]
        return standard, period, phase, score, polarity

    def _color_oscillator(
            self, standard: str, raster_msps: float,
            period: float) -> tuple[list[float], list[float]]:
        """Return one line of cached color-subcarrier quadrature samples."""
        sample_count = int(math.ceil(period)) + 2
        key = (standard, raster_msps, sample_count)
        cached = self.live_video_color_oscillators.get(key)
        if cached is not None:
            return cached
        omega = (2.0 * math.pi * VIDEO_COLOR_SUBCARRIER_HZ[standard] /
                 (raster_msps * 1_000_000.0))
        cos_values = [math.cos(omega * index)
                      for index in range(sample_count)]
        sin_values = [math.sin(omega * index)
                      for index in range(sample_count)]
        cached = (cos_values, sin_values)
        self.live_video_color_oscillators[key] = cached
        return cached

    @staticmethod
    def _quadrature_component(
            samples: list[float], begin: int, end: int, line_start: int,
            cos_values: list[float],
            sin_values: list[float]) -> tuple[complex, float]:
        """Correlate a CVBS interval with the color subcarrier.

        Removing the interval mean rejects luma and sync energy before the
        quadrature correlation. The returned complex value retains the native
        CVBS amplitude so it can be normalized against the luma span later.
        """
        begin = max(line_start, begin)
        end = min(len(samples), end, line_start + len(cos_values))
        count = end - begin
        if count < 4:
            return 0j, 0.0
        mean = statistics.fmean(samples[begin:end])
        cosine_sum = 0.0
        sine_sum = 0.0
        energy = 0.0
        for index in range(begin, end):
            value = samples[index] - mean
            phase_index = index - line_start
            cosine_sum += value * cos_values[phase_index]
            sine_sum += value * sin_values[phase_index]
            energy += value * value
        scale = 2.0 / count
        # e^-jwt correlation: the negative sine term preserves the conventional
        # positive complex phase for cos(wt + phase).
        phasor = complex(cosine_sum * scale, -sine_sum * scale)
        return phasor, math.sqrt(energy / count)

    @staticmethod
    def _color_rgb(
            standard: str, y: float, relative_chroma: complex,
            luma_span: float, line_number: int) -> tuple[int, int, int]:
        """Convert burst-relative PAL U/V or NTSC I/Q into display RGB."""
        y = max(0.0, min(1.0, y))
        chroma = relative_chroma / max(1e-6, luma_span)
        chroma *= 0.72

        if standard == "PAL":
            # PAL swings the burst +/-45 degrees around the -U axis while the
            # V component changes sign on alternate physical lines. Restore
            # that line's absolute U/V axes from its own burst reference.
            swing = 1.0 if line_number % 2 == 0 else -1.0
            burst_axis = math.radians(135.0 * swing)
            absolute = chroma * complex(
                math.cos(burst_axis), math.sin(burst_axis))
            u = max(-0.75, min(0.75, -absolute.imag))
            v = max(-0.75, min(0.75, swing * absolute.real))
            red = y + 1.140 * v
            green = y - 0.395 * u - 0.581 * v
            blue = y + 2.032 * u
        else:
            # NTSC burst is 180 degrees from the color reference. Rotate the
            # conventional I/Q axes by the 33 degree modulation offset.
            iq = (-chroma) * complex(
                math.cos(math.radians(-33.0)),
                math.sin(math.radians(-33.0)))
            raw_i = iq.real
            raw_q = -iq.imag
            # Two independent captures of the supplied SMPTE bars through the
            # Caddx Ant show the same +97 degree chroma-axis error. This matrix
            # is the least-squares observed-I/Q -> reference-I/Q calibration,
            # averaged across both screenshots. It corrects hue and the unequal
            # quadrature gains without changing luma.
            i_value = 0.31835 * raw_i - 1.84549 * raw_q
            q_value = 1.24641 * raw_i - 0.71014 * raw_q
            i_value = max(-0.75, min(0.75, i_value))
            q_value = max(-0.75, min(0.75, q_value))
            # The third locked SMPTE capture leaves only the yellow sector a
            # little orange: its negative-Q magnitude is low while the other
            # five bar hues are already correct. Apply a smooth quadrant-only
            # correction so red, green, cyan, blue and magenta do not rotate.
            quadrant_total = abs(i_value) + abs(q_value)
            if i_value > 0.0 and q_value < 0.0 and quadrant_total > 1e-9:
                yellow_weight = min(
                    1.0,
                    4.0 * i_value * (-q_value) /
                    (quadrant_total * quadrant_total),
                )
                q_value = max(-0.75, q_value * (1.0 + 0.14 * yellow_weight))
            red = y + 0.956 * i_value + 0.621 * q_value
            green = y - 0.272 * i_value - 0.647 * q_value
            blue = y - 1.106 * i_value + 1.703 * q_value

        return tuple(
            max(0, min(255, int(round(channel * 255.0))))
            for channel in (red, green, blue)
        )

    def _observe_vertical_sync(
            self, event_time_us: float, standard: str) -> None:
        """Discipline field placement and rate from sparse broad V-sync lines."""
        nominal_period = VIDEO_FIELD_PERIOD_US[standard]
        if self.live_video_vertical_standard != standard:
            self.live_video_vertical_standard = standard
            self.live_video_vertical_anchor_us = None
            self.live_video_vertical_last_event_us = None
            self.live_video_vertical_lock_events = 0
            self.live_video_field_period_us = nominal_period

        anchor = self.live_video_vertical_anchor_us
        if anchor is None:
            self.live_video_vertical_anchor_us = event_time_us
            self.live_video_vertical_last_event_us = event_time_us
            self.live_video_vertical_lock_events = 1
            # Do not retain rows assembled using an arbitrary USB timestamp
            # phase. Start the visible raster at the first proven field sync.
            self.live_video_pixels[:] = bytes(len(self.live_video_pixels))
            self.live_video_rgb_pixels[:] = bytes(
                len(self.live_video_rgb_pixels))
            self.live_video_rows_seen.clear()
            self.live_video_color_valid_rows.clear()
            self.live_video_display_offset = 0
            return

        period = self.live_video_field_period_us
        fields_from_anchor = int(round((event_time_us - anchor) / period))
        predicted = anchor + fields_from_anchor * period
        phase_error = event_time_us - predicted
        # The Caddx field clock in the measured bundle differs from nominal by
        # only a fraction of a microsecond per field. That still accumulates
        # into a rolling frame seam. The old narrow candidate gate eventually
        # rejected every real V-sync and left the raster free-running. Keep a
        # conservative gate for false picture candidates, but use accepted
        # long-baseline phase error to discipline both phase and frequency.
        if abs(phase_error) > VIDEO_VERTICAL_PHASE_GATE_US:
            return
        if fields_from_anchor:
            period_correction = (
                VIDEO_VERTICAL_FREQUENCY_GAIN * phase_error /
                fields_from_anchor)
            period_correction = max(-2.0, min(2.0, period_correction))
            period = max(
                nominal_period * 0.995,
                min(nominal_period * 1.005, period + period_correction),
            )
            self.live_video_field_period_us = period
        self.live_video_vertical_anchor_us = (
            anchor + VIDEO_VERTICAL_PHASE_GAIN * phase_error)
        self.live_video_vertical_lock_events += 1
        self.live_video_vertical_last_event_us = event_time_us

    def _observe_vertical_candidate(
            self, event_time_us: float, standard: str,
            direct_lock: bool) -> None:
        """Acquire V-lock from sparse but field-coherent broad-sync evidence."""
        nominal_period = VIDEO_FIELD_PERIOD_US[standard]
        if direct_lock:
            self._observe_vertical_sync(event_time_us, standard)
            self.live_video_vertical_candidate_us = None
            self.live_video_vertical_candidate_hits = 0
            self.live_video_vertical_candidate_standard = None
            return

        anchor = self.live_video_vertical_anchor_us
        if anchor is not None:
            self._observe_vertical_sync(event_time_us, standard)
            return

        if self.live_video_vertical_candidate_standard != standard:
            self.live_video_vertical_candidate_standard = standard
            self.live_video_vertical_candidate_us = None
            self.live_video_vertical_candidate_hits = 0

        previous = self.live_video_vertical_candidate_us
        coherent = False
        calibration_period: float | None = None
        if previous is not None and event_time_us > previous:
            delta = event_time_us - previous
            fields = max(1, int(round(delta / nominal_period)))
            coherent = abs(delta - fields * nominal_period) <= 350.0
            measured_period = delta / fields
            if (coherent and
                    fields >= VIDEO_VERTICAL_MIN_CALIBRATION_FIELDS and
                    nominal_period * 0.995 <= measured_period <=
                    nominal_period * 1.005):
                calibration_period = measured_period
        self.live_video_vertical_candidate_hits = (
            self.live_video_vertical_candidate_hits + 1 if coherent else 1)
        self.live_video_vertical_candidate_us = event_time_us
        if self.live_video_vertical_candidate_hits >= 2:
            self._observe_vertical_sync(event_time_us, standard)
            # Sparse capture often gives us a much better clock measurement
            # before it gives us a picture. When the two acquisition events
            # span enough fields, start the raster at that measured rate rather
            # than painting with nominal timing and slowly shearing it later.
            if calibration_period is not None:
                self.live_video_field_period_us = calibration_period
            self.live_video_vertical_candidate_us = None
            self.live_video_vertical_candidate_hits = 0
            self.live_video_vertical_candidate_standard = None

    def _process_live_iq_block(
            self, words: list[int], timestamp_us: int | None = None) -> None:
        clipped_words = sum(
            1 for word in words
            if any(abs(value) >= 511 for value in self._decode_iq(word))
        )
        clipped_percent = 100.0 * clipped_words / max(1, len(words))
        fm = self._fm_discriminator(words)
        self._process_live_fm_block(
            fm, timestamp_us, f"{clipped_percent:.1f}%")

    def _process_live_phase8_block(
            self, phases: bytes, timestamp_us: int | None = None) -> None:
        fm = self._phase8_discriminator(phases)
        self._process_live_fm_block(fm, timestamp_us, "n/a (Phase8)")

    def _process_live_fm_block(
            self, fm: list[float], timestamp_us: int | None,
            clipped_text: str) -> None:
        if len(fm) < 512:
            return

        # Work in 16-sample bins. The vendor call duration is not the physical
        # sample clock, so the user-selectable RF-raster hypothesis below is
        # deliberately independent from finite_fill_msps.
        bin_size = 16
        binned = [
            statistics.fmean(fm[offset:offset + bin_size])
            for offset in range(0, len(fm) - bin_size + 1, bin_size)
        ]
        raster_msps = self.live_video_requested_sample_rate
        (video_standard, period, sync_phase,
         sync_score, fm_polarity) = self._select_video_timing(
             binned, raster_msps, bin_size, fm)

        # Do not stretch ordinary RF noise into a convincing full-contrast
        # picture. Only a repeated horizontal-sync candidate may update the
        # raster; rejected blocks leave the last locked pixels untouched.
        if sync_score < VIDEO_SYNC_MIN_SCORE:
            self.live_video_rejected_blocks += 1
            self.live_video_pending_status = (
                f"{APP_BUILD}: no {video_standard} H-sync lock "
                f"(score {sync_score:.2f} < {VIDEO_SYNC_MIN_SCORE:.2f}); "
                "pixels not updated")
            return

        if fm_polarity < 0:
            fm = [-value for value in fm]

        line_starts: list[int] = []
        cursor = sync_phase * bin_size
        while cursor - period >= 0:
            cursor -= period
        while cursor + period <= len(fm):
            line_starts.append(int(cursor))
            cursor += period

        # Identify NTSC/PAL vertical serration intervals from the captured
        # waveform itself. Normal H-sync occupies about 7% of a line; vertical
        # sync holds the composite signal below the sync/porch midpoint for
        # roughly half or more of consecutive lines. Those lines are timing
        # evidence, not picture rows, and must never be painted into the image.
        vertical_line_starts: set[int] = set()
        strong_vertical_starts: list[int] = []
        for start_sample in line_starts:
            line_end = min(len(fm), int(start_sample + period))
            sync_end = int(start_sample + period * 0.073)
            porch_begin = int(start_sample + period * 0.083)
            porch_end = int(start_sample + period * 0.135)
            if (sync_end <= start_sample or porch_end <= porch_begin or
                    line_end <= start_sample):
                continue
            sync_mean = statistics.fmean(fm[start_sample:sync_end])
            porch_mean = statistics.fmean(fm[porch_begin:porch_end])
            threshold = (sync_mean + porch_mean) * 0.5
            low_fraction = sum(
                value < threshold for value in fm[start_sample:line_end]) / (
                    line_end - start_sample)
            if low_fraction >= 0.45:
                vertical_line_starts.add(start_sample)
            if low_fraction >= 0.55:
                strong_vertical_starts.append(start_sample)

        broad_vertical_starts = sorted(vertical_line_starts)
        direct_vertical_lock = (
            len(strong_vertical_starts) >= 2 and sync_score >= 1.20)
        tentative_vertical_lock = (
            bool(broad_vertical_starts) and sync_score >= 0.80)
        if (timestamp_us is not None and
                (direct_vertical_lock or tentative_vertical_lock)):
            vertical_sample = min(
                strong_vertical_starts
                if direct_vertical_lock else broad_vertical_starts)
            vertical_time_us = timestamp_us - (
                len(fm) - vertical_sample) / raster_msps
            self._observe_vertical_candidate(
                vertical_time_us, video_standard, direct_vertical_lock)

        if timestamp_us is not None and self.live_video_vertical_anchor_us is None:
            self.live_video_pending_status = (
                f"{APP_BUILD}: waiting for captured {video_standard} vertical sync; "
                f"coherent candidates {self.live_video_vertical_candidate_hits}/2; "
                "picture rows withheld to prevent jumping")
            return

        cos_values, sin_values = self._color_oscillator(
            video_standard, raster_msps, period)
        burst_begin_fraction, burst_end_fraction = \
            VIDEO_COLOR_BURST_WINDOW[video_standard]
        active_lines: list[
            tuple[int, int, list[float], list[complex], float, float]] = []
        for start_sample in line_starts:
            if start_sample in vertical_line_starts:
                continue
            line_start = int(start_sample)
            active_begin = int(start_sample + period * 0.16)
            active_end = int(start_sample + period * 0.96)
            if active_begin < 0 or active_end > len(fm) or active_end <= active_begin:
                continue
            burst_begin = int(start_sample + period * burst_begin_fraction)
            burst_end = int(start_sample + period * burst_end_fraction)
            burst_phasor, burst_rms = self._quadrature_component(
                fm, burst_begin, burst_end, line_start,
                cos_values, sin_values)
            burst_amplitude = abs(burst_phasor)
            burst_coherence = burst_amplitude / max(
                1e-9, math.sqrt(2.0) * burst_rms)
            burst_reference = (
                burst_phasor.conjugate() / burst_amplitude
                if burst_amplitude > 1e-9 else 0j)
            source = fm[active_begin:active_end]
            row: list[float] = []
            row_chroma: list[complex] = []
            for x in range(self.live_video_width):
                left = x * len(source) // self.live_video_width
                right = max(left + 1, (x + 1) * len(source) // self.live_video_width)
                row.append(statistics.fmean(source[left:right]))
                pixel_phasor, _pixel_rms = self._quadrature_component(
                    fm, active_begin + left, active_begin + right,
                    line_start, cos_values, sin_values)
                row_chroma.append(pixel_phasor * burst_reference)

            # Composite chroma bandwidth is much lower than the 160-pixel
            # luma raster. A short symmetric filter rejects fine luma edges
            # that otherwise alias into vivid magenta/green dot crawl.
            if row_chroma:
                raw_chroma = row_chroma
                row_chroma = []
                for x in range(len(raw_chroma)):
                    weighted = 0j
                    weight_total = 0
                    for delta, weight in ((-2, 1), (-1, 2), (0, 4),
                                          (1, 2), (2, 1)):
                        source_x = x + delta
                        if 0 <= source_x < len(raw_chroma):
                            weighted += raw_chroma[source_x] * weight
                            weight_total += weight
                    row_chroma.append(weighted / weight_total)

            if timestamp_us is not None:
                line_time_us = timestamp_us - (
                    len(fm) - start_sample) * 1_000_000.0 / (
                        raster_msps * 1_000_000.0)
                line_reference_us = (
                    self.live_video_vertical_anchor_us
                    if self.live_video_vertical_anchor_us is not None else 0.0)
                line_number = int(round(
                    (line_time_us - line_reference_us) *
                    VIDEO_LINE_RATES_HZ[video_standard] /
                    1_000_000.0))
            else:
                line_number = self.live_video_color_line_counter
                self.live_video_color_line_counter += 1
            active_lines.append((
                start_sample, line_number, row, row_chroma,
                burst_amplitude, burst_coherence))

        if not active_lines:
            self.live_video_pending_status = (
                "IQ received; no complete video-line candidate in this block")
            return
        scale_values = sorted(
            value for (_start_sample, _line_number, row, _row_chroma,
                       _burst_amplitude, _burst_coherence) in active_lines
            for value in row)
        block_black = scale_values[len(scale_values) // 20]
        block_white = scale_values[(len(scale_values) * 19) // 20]
        if self.live_video_black is None or self.live_video_white is None:
            self.live_video_black = block_black
            self.live_video_white = block_white
        else:
            # Every finite block contains only a few adjacent lines. Scaling
            # each block independently makes strip boundaries look like lines
            # placed at the wrong height. A slow shared scale keeps brightness
            # coherent while still following gradual RF-level changes.
            scale_alpha = 0.08
            self.live_video_black += scale_alpha * (
                block_black - self.live_video_black)
            self.live_video_white += scale_alpha * (
                block_white - self.live_video_white)
        black = self.live_video_black
        white = self.live_video_white
        span = max(1e-6, white - black)
        physical_rate_hz = raster_msps * 1_000_000.0
        for (start_sample, line_number, row, row_chroma,
             burst_amplitude, burst_coherence) in active_lines:
            if (timestamp_us is not None and
                    self.live_video_vertical_anchor_us is not None):
                line_time_us = timestamp_us - (
                    len(fm) - start_sample) * 1_000_000.0 / physical_rate_hz
                field_period_us = self.live_video_field_period_us
                field_phase = (
                    line_time_us - self.live_video_vertical_anchor_us) % \
                    field_period_us
                # Conventional receivers hide the vertical blanking interval.
                # Mapping the entire field exposed about nine dark/noisy rows
                # above the camera's top OSD. Start at the first active-picture
                # line and expand the remaining field into the display raster.
                active_start_us = (
                    VIDEO_ACTIVE_START_LINES[video_standard] * 1_000_000.0 /
                    VIDEO_LINE_RATES_HZ[video_standard])
                if field_phase < active_start_us:
                    continue
                active_period_us = field_period_us - active_start_us
                target_row = int(
                    (field_phase - active_start_us) * self.live_video_height /
                    active_period_us)
            else:
                target_row = self.live_video_row
                self.live_video_row = (
                    self.live_video_row + 1) % self.live_video_height
            base = target_row * self.live_video_width
            rgb_base = base * 3
            burst_level = burst_amplitude / span
            color_locked = (
                burst_level >= VIDEO_COLOR_MIN_BURST_LEVEL and
                burst_coherence >= VIDEO_COLOR_MIN_COHERENCE)
            color_total = (self.live_video_color_locked_lines +
                           self.live_video_monochrome_lines)
            color_alpha = 1.0 if color_total == 0 else 0.10
            self.live_video_color_burst_level += color_alpha * (
                burst_level - self.live_video_color_burst_level)
            self.live_video_color_burst_coherence += color_alpha * (
                burst_coherence - self.live_video_color_burst_coherence)
            if color_locked:
                self.live_video_color_locked_lines += 1
                row_bias = sum(row_chroma, 0j) / max(1, len(row_chroma))
                bias_samples = self.live_video_color_bias_samples[video_standard]
                if bias_samples == 0:
                    self.live_video_color_bias[video_standard] = row_bias
                else:
                    bias_alpha = 0.025
                    self.live_video_color_bias[video_standard] += bias_alpha * (
                        row_bias - self.live_video_color_bias[video_standard])
                self.live_video_color_bias_samples[video_standard] = \
                    bias_samples + 1
                self.live_video_color_valid_rows.add(target_row)
            else:
                self.live_video_monochrome_lines += 1

            for x, value in enumerate(row):
                y = max(0.0, min(1.0, (value - black) / span))
                luma = max(0, min(255, int(round(y * 255.0))))
                self.live_video_pixels[base + x] = luma
                if color_locked:
                    red, green, blue = self._color_rgb(
                        video_standard, y,
                        row_chroma[x] -
                        self.live_video_color_bias[video_standard],
                        span, line_number)
                elif target_row in self.live_video_color_valid_rows:
                    # A noisy one-line burst must not turn a previously valid
                    # color row gray. Preserve its chroma differences and move
                    # all three channels together to follow the new luma.
                    pixel_base = rgb_base + x * 3
                    old_red = self.live_video_rgb_pixels[pixel_base]
                    old_green = self.live_video_rgb_pixels[pixel_base + 1]
                    old_blue = self.live_video_rgb_pixels[pixel_base + 2]
                    old_luma = (
                        0.299 * old_red + 0.587 * old_green +
                        0.114 * old_blue)
                    luma_delta = luma - old_luma
                    red = max(0, min(255, int(round(old_red + luma_delta))))
                    green = max(
                        0, min(255, int(round(old_green + luma_delta))))
                    blue = max(0, min(255, int(round(old_blue + luma_delta))))
                else:
                    red = green = blue = luma
                pixel_base = rgb_base + x * 3
                self.live_video_rgb_pixels[pixel_base] = red
                self.live_video_rgb_pixels[pixel_base + 1] = green
                self.live_video_rgb_pixels[pixel_base + 2] = blue
            self.live_video_rows_seen.add(target_row)

        elapsed = max(1e-6, time.monotonic() - self.live_iq_usb_started)
        usb_mbit = self.live_iq_usb_bytes * 8.0 / elapsed / 1_000_000.0
        block_rate = self.live_iq_blocks / elapsed
        source_text = (f"{self.live_iq_source_msps:.3f} MS/s finite-fill estimate"
                       if self.live_iq_source_msps else "source MS/s pending")
        coverage = 100.0 * len(self.live_video_rows_seen) / self.live_video_height
        # Field zero now comes from broad vertical-sync pulses. Never rotate
        # based on picture darkness; SMPTE PLUGE and black bars fooled the old
        # heuristic and moved large row groups into the middle of the image.
        self.live_video_display_offset = 0
        display_pixels = bytes(self.live_video_rgb_pixels)
        color_locked_now = (
            self.live_video_color_burst_level >= VIDEO_COLOR_MIN_BURST_LEVEL and
            self.live_video_color_burst_coherence >= VIDEO_COLOR_MIN_COHERENCE)
        color_text = (
            f"{video_standard} color burst "
            f"{self.live_video_color_burst_level * 100.0:.1f}% "
            f"coherence {self.live_video_color_burst_coherence:.2f}"
            if color_locked_now else
            f"monochrome fallback; burst "
            f"{self.live_video_color_burst_level * 100.0:.1f}% "
            f"coherence {self.live_video_color_burst_coherence:.2f}")
        lock_text = (
            f"{self.live_transport_name}; "
            f"{video_standard} H-sync score {sync_score:.2f}; "
            f"{color_text}; "
            f"raster {raster_msps:.0f} MS/s; {coverage:.0f}% rows; "
            f"V-lock {self.live_video_vertical_lock_events} "
            f"field {self.live_video_field_period_us:.2f} us; "
            f"clipped {clipped_text}; "
            f"rejected {self.live_video_rejected_blocks}")
        self._queue_live_iq_frame(
            display_pixels, source_text, usb_mbit, block_rate, lock_text)

    def _queue_live_iq_frame(self, pixels: bytes, source_text: str,
                             usb_mbit: float, block_rate: float,
                             lock_text: str) -> None:
        """Keep only the newest pending raster while Tk is drawing.

        A native PhotoImage update is deliberately much slower than the USB
        transport. Coalescing prevents fast proof mode from building an
        unbounded queue of stale frames while still drawing as fast as Tk can.
        """
        self.live_video_pending_frame = (
            pixels, source_text, usb_mbit, block_rate, lock_text)

    def _poll_live_iq_frame(self) -> None:
        """Render the newest worker result without cross-thread Tk calls."""
        pending = self.live_video_pending_frame
        self.live_video_pending_frame = None
        if pending is not None and self.live_iq_active:
            self._show_live_iq_frame(*pending)
        elif self.live_video_pending_status is not None and self.live_iq_active:
            self.preview_status_var.set(self.live_video_pending_status)
        self.live_video_pending_status = None
        self.after(33, self._poll_live_iq_frame)

    def _render_pending_live_iq_frame(self) -> None:
        pending = self.live_video_pending_frame
        self.live_video_pending_frame = None
        if pending is not None and self.live_iq_active:
            self._show_live_iq_frame(*pending)
        self.live_video_render_pending = False
        if self.live_video_pending_frame is not None and self.live_iq_active:
            self.live_video_render_pending = True
            self.after_idle(self._render_pending_live_iq_frame)

    def _update_vertical_display_offset(self) -> None:
        """Move the circular field-time seam into the vertical blanking gap."""
        height = self.live_video_height
        width = self.live_video_width
        if len(self.live_video_rows_seen) < int(height * 0.70):
            return
        if self.live_iq_blocks % 8:
            return

        row_means = [
            statistics.fmean(
                self.live_video_pixels[row * width:(row + 1) * width])
            for row in range(height)
        ]
        blank_rows = max(4, height // 16)
        candidates: list[tuple[float, int]] = []
        baseline = statistics.median(row_means)
        for start in range(height):
            window_rows = [(start + index) % height
                           for index in range(blank_rows)]
            if not all(row in self.live_video_rows_seen for row in window_rows):
                continue
            after_rows = [(start + blank_rows + index) % height
                          for index in range(3)]
            if not all(row in self.live_video_rows_seen for row in after_rows):
                continue
            blank_mean = statistics.fmean(row_means[row] for row in window_rows)
            after_mean = statistics.fmean(row_means[row] for row in after_rows)
            score = (baseline - blank_mean) + max(0.0, after_mean - blank_mean) * 0.35
            candidates.append((score, (start + blank_rows) % height))
        if not candidates:
            return
        score, candidate = max(candidates)
        if score < 12.0:
            return

        pending = self.live_video_vertical_pending
        if pending is None:
            self.live_video_vertical_pending = candidate
            self.live_video_vertical_pending_hits = 1
            return
        distance = abs(((candidate - pending + height // 2) % height) -
                       height // 2)
        if distance <= 3:
            self.live_video_vertical_pending_hits += 1
            if self.live_video_vertical_pending_hits >= 2:
                self.live_video_display_offset = candidate
        else:
            self.live_video_vertical_pending = candidate
            self.live_video_vertical_pending_hits = 1

    def _vertically_aligned_pixels(self) -> bytes:
        width = self.live_video_width
        offset_bytes = self.live_video_display_offset * width
        pixels = bytes(self.live_video_pixels)
        if not offset_bytes:
            return pixels
        return pixels[offset_bytes:] + pixels[:offset_bytes]

    def _vertically_aligned_rgb_pixels(self) -> bytes:
        row_bytes = self.live_video_width * 3
        offset_bytes = self.live_video_display_offset * row_bytes
        pixels = bytes(self.live_video_rgb_pixels)
        if not offset_bytes:
            return pixels
        return pixels[offset_bytes:] + pixels[:offset_bytes]

    @staticmethod
    def _vertical_median_rgb(pixels: bytes, width: int, height: int) -> bytes:
        """Remove isolated horizontal impulse rows without temporal smearing."""
        row_bytes = width * 3
        if width <= 0 or height < 3 or len(pixels) != row_bytes * height:
            return pixels
        source = np.frombuffer(pixels, dtype=np.uint8).reshape(
            height, width, 3)
        filtered = source.copy()
        above = source[:-2]
        center = source[1:-1]
        below = source[2:]
        filtered[1:-1] = np.maximum(
            np.minimum(above, center),
            np.minimum(np.maximum(above, center), below),
        )
        return filtered.tobytes()

    def _show_live_iq_frame(self, pixels: bytes, source_text: str,
                            usb_mbit: float, block_rate: float,
                            lock_text: str) -> None:
        pixels = self._vertical_median_rgb(
            pixels, self.live_video_width, self.live_video_height)
        if not self._show_rgb_frame(
                pixels, self.live_video_width, self.live_video_height):
            self.sink.write(
                f"C5VRX_HOST_VIDEO_RENDER_ERROR build={APP_BUILD}\n")
            return
        self.live_video_host_frames += 1
        if self.live_video_host_frames == 1 or self.live_video_host_frames % 10 == 0:
            self.sink.write(
                f"C5VRX_HOST_VIDEO_FRAME build={APP_BUILD} "
                f"frame={self.live_video_host_frames} {lock_text}\n")
        self.preview_status_var.set(
            f"{APP_BUILD} | A1 IQ->PC color raster | {source_text} | USB {usb_mbit:.2f} Mbit/s | "
            f"{block_rate:.2f} blocks/s | "
            f"{'FAST pipeline' if self.live_iq_fast_mode else 'single request'} "
            f"{self.live_iq_commands_outstanding}/{self.live_iq_pipeline_target} | "
            f"recoveries {self.live_iq_recoveries}; "
            f"CRC losses {self.live_iq_transport_losses}; "
            f"host drops {self.live_iq_processing_drops} | "
            f"{lock_text} | RETRIGGERED, NOT GAPLESS")

    def first_hardware_test(self) -> None:
        if not self.ser or not self.ser.is_open:
            messagebox.showwarning(APP_TITLE, "Connect to C5VRX first.")
            return
        center = FPV_BANDS[self.band_var.get()][int(self.channel_var.get()) - 1]
        tone = center + 2
        if not messagebox.askokcancel(
            APP_TITLE,
            f"Set a coherent RF generator to {tone} MHz at a safe, attenuated level. "
            f"The receiver baseline is {center} MHz. The suite takes about 90 seconds and never enables unbounded capture. Continue?",
        ):
            return
        self.first_test_active = True
        self.first_test_fine_sent = False
        self.first_test_center = center
        commands = [
            "STATUS",
            "WBFM HWTEST",
            "PRODUCER CADENCE PROBE ALL",
            "WRAP FLAG PROBE 0",
            "PHASE CONTINUITY PROBE 0",
            "PRODUCER SOAK 0 30000",
            "BENCH SPARSE 2",
            "BENCH SPARSE 4",
            "BENCH SPARSE 8",
            "BENCH BITSCRAMBLER",
            "BENCH PARLIO",
            "BENCH PIPELINE",
            "BENCH USB PREVIEW",
            "BENCH RING PIPELINE 0 1000 1024",
            "BENCH RING PIPELINE 0 1000 2048",
            "BENCH RING PIPELINE 0 1000 4096",
            "USB PREVIEW STOP",
            "CAPABILITIES",
            "STATUS",
        ]
        payload = "".join(command + "\n" for command in commands).encode("ascii")
        for command in commands:
            self.session.record_command(command)
        if self._write_serial_bytes(payload, retries=3):
            self.sink.write("\n=== FIRST HARDWARE TEST QUEUED ===\n")
            for command in commands:
                self.sink.write(f"> {command}\n")
        else:
            messagebox.showerror(
                APP_TITLE, "Could not queue diagnostics over native USB.")

    @staticmethod
    def _decode_iq(raw: int) -> tuple[int, int]:
        def s10(v: int) -> int:
            v &= 0x3FF
            return v - 0x400 if v & 0x200 else v
        return s10(raw >> 10), s10(raw)

    def render_iq_preview(self) -> None:
        canvas = getattr(self, "preview_canvas", None)
        if not canvas:
            return
        if self.preview_image is not None:
            self._redraw_preview()
            return
        canvas.delete("all")
        width = max(40, canvas.winfo_width())
        height = max(80, canvas.winfo_height())
        canvas.create_text(
            width / 2,
            height / 2,
            text=(f"{APP_BUILD}\n\nNo waveform display in this build.\n"
                  "Press ULTRA-SLOW VIDEO PROOF (A1)."),
            fill="white",
            justify="center",
        )

    def _redraw_preview(self, _event: object | None = None) -> None:
        """Keep decoded video visible when the preview canvas is resized."""
        if self.preview_image is None:
            if not self.live_iq_active:
                self.render_iq_preview()
            return
        canvas = self.preview_canvas
        canvas.delete("all")
        scale = max(1, min(
            max(1, canvas.winfo_width()) // max(1, self.preview_width),
            max(1, canvas.winfo_height()) // max(1, self.preview_height),
        ))
        self.preview_display_image = (
            self.preview_image.zoom(scale, scale)
            if scale > 1 else self.preview_image
        )
        canvas.create_image(
            max(0, canvas.winfo_width() // 2),
            max(0, canvas.winfo_height() // 2),
            image=self.preview_display_image,
            anchor="center",
        )

    def _show_gray_frame(self, payload: bytes, width: int, height: int) -> bool:
        if len(payload) != width * height:
            self.preview_status_var.set(
                f"GRAY8 render rejected: {len(payload)} bytes for {width}x{height}")
            return False
        self.preview_frame = payload
        self.preview_width = width
        self.preview_height = height
        try:
            # Tk's Windows build lacks PGM but does support binary PPM. Expand
            # gray bytes into RGB in C-backed slice operations; this avoids
            # creating 19,200 Tcl color strings for every displayed frame.
            rgb = bytearray(len(payload) * 3)
            rgb[0::3] = payload
            rgb[1::3] = payload
            rgb[2::3] = payload
            ppm = f"P6\n{width} {height}\n255\n".encode("ascii") + bytes(rgb)
            image = tk.PhotoImage(data=ppm, format="PPM")
            self.preview_image = image
        except tk.TclError:
            self.preview_image = None
            self.preview_display_image = None
            self.preview_status_var.set(
                "Valid GRAY8 frame received, but Tk pixel rendering failed")
            return False
        self._redraw_preview()
        self.preview_status_var.set(f"Live USB preview: {width}×{height} GRAY8, CRC valid")
        return True

    def _show_yuv_rgb_frame(self, payload: bytes, width: int, height: int,
                            burst_locked: bool) -> bool:
        if len(payload) != width * height * 3:
            self.preview_status_var.set(
                f"RGB render rejected: {len(payload)} bytes for {width}x{height}")
            return False
        self.preview_frame = payload
        self.preview_width = width
        self.preview_height = height
        try:
            ppm = f"P6\n{width} {height}\n255\n".encode("ascii") + payload
            self.preview_image = tk.PhotoImage(data=ppm, format="PPM")
        except tk.TclError:
            self.preview_image = None
            self.preview_display_image = None
            self.preview_status_var.set("Valid YUV411 frame, but rendering failed")
            return False
        self._redraw_preview()
        self.preview_status_var.set(
            f"Live USB preview: {width}×{height} YUV411, CRC valid, "
            f"{'burst locked' if burst_locked else 'chroma unlocked'}")
        return True

    def _show_rgb_frame(self, payload: bytes, width: int, height: int) -> bool:
        if len(payload) != width * height * 3:
            self.preview_status_var.set(
                f"RGB24 render rejected: {len(payload)} bytes for {width}x{height}")
            return False
        self.preview_frame = payload
        self.preview_width = width
        self.preview_height = height
        try:
            ppm = f"P6\n{width} {height}\n255\n".encode("ascii") + payload
            self.preview_image = tk.PhotoImage(data=ppm, format="PPM")
        except tk.TclError:
            self.preview_image = None
            self.preview_display_image = None
            self.preview_status_var.set(
                "Valid RGB24 frame received, but Tk pixel rendering failed")
            return False
        self._redraw_preview()
        self.preview_status_var.set(
            f"Live color preview: {width}×{height} RGB24, burst validated")
        return True

    def clear_preview(self) -> None:
        self.iq_words = []
        self.preview_frame = None
        self.preview_image = None
        self.preview_display_image = None
        self.preview_status_var.set(f"{APP_BUILD}: video raster cleared")
        self.render_iq_preview()

    def export_codex_bundle(self) -> None:
        self.session.update_test_config(
            port=self.selected_port(),
            band=self.band_var.get(),
            channel=int(self.channel_var.get()),
            bandwidth_mhz=int(self.bw_var.get()),
            finite_iq_samples=int(self.samples_var.get()),
        )
        suggested = self.session.path.name + "-codex-bundle.zip"
        selected = filedialog.asksaveasfilename(
            title="Export C5VRX Codex bundle",
            defaultextension=".zip",
            initialfile=suggested,
            filetypes=[("ZIP archive", "*.zip")],
        )
        if not selected:
            return
        try:
            bundle = self.session.create_bundle(Path(selected))
            self.sink.write(f"\n=== CODEX BUNDLE EXPORTED: {bundle} ===\n")
            messagebox.showinfo(APP_TITLE, f"Codex bundle exported:\n\n{bundle}")
        except Exception as exc:
            self.session.record_error("BUNDLE_EXPORT", str(exc))
            messagebox.showerror(APP_TITLE, f"Could not export Codex bundle:\n\n{exc}")

    def flash(self) -> None:
        if self._busy:
            return
        port = self.selected_port()
        if not port:
            messagebox.showerror(APP_TITLE, "No USB/COM port selected.")
            return
        if not messagebox.askokcancel(
            APP_TITLE,
            f"Flash {self.firmware_profile['display_name']} firmware to {port}?\n\n"
            f"AV mapping: {self.firmware_profile['av_pin_summary']}\n"
            f"Expected flash: {self.firmware_profile['flash_size_mb']} MB\n\n"
            "Use this image only on the named board/profile.",
        ):
            return

        self.disconnect_serial()
        self._busy = True
        self.flash_btn.configure(state="disabled")
        self.port_combo.configure(state="disabled")
        self.progress.start(10)
        self.sink.write(f"\n=== C5VRX FLASH START: {port} ===\n")
        threading.Thread(target=self._flash_worker, args=(port,), daemon=True).start()

    def _flash_worker(self, port: str) -> None:
        try:
            base_args, files = load_flash_plan()
            argv = base_args + ["--port", port, "write-flash"]
            for offset, path in files:
                argv += [str(offset), str(path)]

            self.sink.write("Firmware image set:\n")
            for offset, path in files:
                self.sink.write(f"  {offset}: {path.name}\n")
            self.sink.write("\nConnecting to ESP32-C5 bootloader...\n")

            with redirect_stdout(self.sink), redirect_stderr(self.sink):
                esptool.main(argv)

            self.after(0, self._done_ok)
        except SystemExit as exc:
            if exc.code in (0, None):
                self.after(0, self._done_ok)
            else:
                self.after(0, self._done_error, f"esptool exited with code {exc.code}")
        except Exception as exc:
            self.sink.write("\n" + traceback.format_exc() + "\n")
            self.after(0, self._done_error, str(exc))

    def _finish_flash(self) -> None:
        self._busy = False
        self.progress.stop()
        self.flash_btn.configure(state="normal")
        self.port_combo.configure(state="readonly")
        self.refresh_ports()

    def _done_ok(self) -> None:
        self._finish_flash()
        self.sink.write("\n=== FLASH COMPLETE ===\n")
        self.connection_var.set("Firmware flashed — waiting for USB console...")
        self.after(1800, self._auto_reconnect_after_flash)

    def _auto_reconnect_after_flash(self) -> None:
        self.refresh_ports()
        if self.connect_serial(show_errors=False):
            self.sink.write("C5VRX runtime control connected automatically.\n")
        else:
            self.connection_var.set("Flashed. Select the C5 USB console port and press Connect.")

    def _done_error(self, msg: str) -> None:
        self._finish_flash()
        self.sink.write(f"\n=== FLASH FAILED: {msg} ===\n")
        messagebox.showerror(
            APP_TITLE,
            f"Flash failed:\n\n{msg}\n\nTry another COM port or put the C5 into download mode (BOOT + RESET).",
        )

    def on_close(self) -> None:
        self.disconnect_serial()
        time.sleep(0.03)
        self.session.finalize({
            "status": "CONSOLE_SESSION_COMPLETE",
            "passed": None,
            "reason": "interactive Receiver Console session",
        })
        self.session.close()
        self.destroy()


if __name__ == "__main__":
    C5VRXApp().mainloop()
