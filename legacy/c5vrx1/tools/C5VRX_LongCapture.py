#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-only
"""C5VRX fast long IQ capture helper.

The ESP32-C5 vendor RF-test dump RAM is fixed at 0x10000 bytes, i.e. 16384
packed 32-bit IQ words. This host tool never asks firmware for more than that
in a single CAPTURE command. Larger captures are collected as multiple finite
blocks.

While a long capture is active the helper opens the existing CRC-framed USB
preview transport. Firmware then sends each finite IQ dump as one binary packet
instead of thousands of ``IQ:xxxxxxxx`` console lines. Blocks are separately
re-triggered and are NOT guaranteed gapless.
"""

from __future__ import annotations

import threading
import time
from pathlib import Path
import tkinter as tk
from tkinter import filedialog, messagebox, ttk

import serial
from serial.tools import list_ports

from c5vrx_usb_protocol import (
    PACKET_IQ_U32_BLOCK,
    Packet,
    StreamDecoder,
    decode_iq_block,
)

APP_TITLE = "C5VRX Long IQ Capture"
MAX_DEVICE_BLOCK = 16384
TOTAL_CHOICES = [16384, 65536, 262144, 524288, 1048576]


class LongCaptureApp(tk.Tk):
    def __init__(self) -> None:
        super().__init__()
        self.title(APP_TITLE)
        self.geometry("780x560")
        self.minsize(700, 500)

        self.ser: serial.Serial | None = None
        self.reader_thread: threading.Thread | None = None
        self.stop_event = threading.Event()

        self.capture_active = False
        self.transport_ready = False
        self.total_target = 0
        self.total_received = 0
        self.block_index = 0
        self.block_requested = 0
        self.block_packet_received = False
        self.capture_started_at = 0.0
        self.words: list[int] = []
        self.block_sizes: list[int] = []

        root = ttk.Frame(self, padding=14)
        root.pack(fill="both", expand=True)

        ttk.Label(
            root, text="C5VRX Fast Long IQ Capture",
            font=("Segoe UI", 20, "bold"),
        ).pack(anchor="w")
        ttk.Label(
            root,
            text=(
                "Fast binary USB transfer: each hardware-safe 16K RF dump is "
                "sent as one CRC-protected packet, not 16,384 console lines. "
                "Choose a total, capture, then copy or save all IQ in one go."
            ),
            wraplength=745,
        ).pack(anchor="w", pady=(4, 14))

        port_row = ttk.Frame(root)
        port_row.pack(fill="x")
        ttk.Label(port_row, text="USB / COM port:").pack(side="left")
        self.port_var = tk.StringVar()
        self.port_combo = ttk.Combobox(
            port_row, textvariable=self.port_var,
            state="readonly", width=45,
        )
        self.port_combo.pack(side="left", padx=8, fill="x", expand=True)
        ttk.Button(port_row, text="Refresh", command=self.refresh_ports).pack(
            side="left", padx=(0, 6)
        )
        self.connect_btn = ttk.Button(
            port_row, text="Connect", command=self.toggle_connect
        )
        self.connect_btn.pack(side="left")

        self.connection_var = tk.StringVar(value="Disconnected")
        ttk.Label(root, textvariable=self.connection_var).pack(
            anchor="w", pady=(6, 14)
        )

        capture_box = ttk.LabelFrame(root, text="Long finite capture", padding=12)
        capture_box.pack(fill="x")

        row = ttk.Frame(capture_box)
        row.pack(fill="x")
        ttk.Label(row, text="Total IQ words").pack(side="left")
        self.total_var = tk.StringVar(value="262144")
        ttk.Combobox(
            row,
            textvariable=self.total_var,
            state="readonly",
            values=[str(v) for v in TOTAL_CHOICES],
            width=12,
        ).pack(side="left", padx=8)

        self.capture_btn = ttk.Button(
            row, text="CAPTURE", command=self.start_capture
        )
        self.capture_btn.pack(side="left", padx=(8, 0), ipady=4)
        self.cancel_btn = ttk.Button(
            row, text="Cancel", command=self.cancel_capture, state="disabled"
        )
        self.cancel_btn.pack(side="left", padx=8)

        self.copy_btn = ttk.Button(
            row, text="COPY ALL IQ", command=self.copy_all_iq, state="disabled"
        )
        self.copy_btn.pack(side="right")
        self.save_btn = ttk.Button(
            row, text="SAVE TXT", command=self.save_txt, state="disabled"
        )
        self.save_btn.pack(side="right", padx=8)

        self.progress = ttk.Progressbar(
            capture_box, mode="determinate", maximum=100
        )
        self.progress.pack(fill="x", pady=(12, 4))
        self.progress_var = tk.StringVar(value="Idle")
        ttk.Label(capture_box, textvariable=self.progress_var).pack(anchor="w")

        ttk.Label(
            capture_box,
            text=(
                "Hardware remains capped at 16,384 words per RF arm. 64K/256K/"
                "512K/1M are multiple finite blocks and may contain gaps between blocks."
            ),
            wraplength=720,
        ).pack(anchor="w", pady=(8, 0))

        self.log = tk.Text(
            root, height=15, wrap="word", state="disabled",
            font=("Consolas", 9)
        )
        self.log.pack(fill="both", expand=True, pady=(12, 0))

        self.protocol("WM_DELETE_WINDOW", self.on_close)
        self.refresh_ports()

    def log_line(self, text: str) -> None:
        def append() -> None:
            self.log.configure(state="normal")
            self.log.insert("end", text + "\n")
            self.log.see("end")
            self.log.configure(state="disabled")
        self.after(0, append)

    def refresh_ports(self) -> None:
        selected = self.selected_port()
        ports = list(list_ports.comports())
        values = [f"{p.device}  —  {p.description}" for p in ports]
        self.port_combo["values"] = values
        if selected:
            for i, value in enumerate(values):
                if value.split()[0] == selected:
                    self.port_combo.current(i)
                    return
        if values:
            self.port_combo.current(0)
        else:
            self.port_var.set("")

    def selected_port(self) -> str | None:
        value = self.port_var.get().strip()
        return value.split()[0] if value else None

    def toggle_connect(self) -> None:
        if self.ser and self.ser.is_open:
            self.disconnect()
        else:
            self.connect()

    def connect(self) -> None:
        port = self.selected_port()
        if not port:
            messagebox.showerror(APP_TITLE, "No USB/COM port selected.")
            return
        self.disconnect()
        try:
            self.ser = serial.Serial(port, 115200, timeout=0.1, write_timeout=1)
            self.stop_event.clear()
            self.reader_thread = threading.Thread(
                target=self.reader_loop, daemon=True
            )
            self.reader_thread.start()
            self.connection_var.set(f"Connected: {port}")
            self.connect_btn.configure(text="Disconnect")
            self.send("PING")
            self.after(200, lambda: self.send("STATUS"))
        except Exception as exc:
            self.ser = None
            messagebox.showerror(APP_TITLE, f"Could not open {port}:\n\n{exc}")

    def disconnect(self) -> None:
        if self.capture_active:
            self.finish_capture(False, "serial disconnected")
        self.stop_event.set()
        ser = self.ser
        self.ser = None
        if ser:
            try:
                ser.close()
            except Exception:
                pass
        self.connection_var.set("Disconnected")
        self.connect_btn.configure(text="Connect")

    def send(self, command: str) -> None:
        ser = self.ser
        if not ser or not ser.is_open:
            return
        try:
            ser.write((command + "\n").encode("ascii"))
            ser.flush()
            self.log_line(f"> {command}")
        except Exception as exc:
            self.after(0, self.finish_capture, False, f"USB write error: {exc}")

    def start_capture(self) -> None:
        if self.capture_active:
            return
        if not self.ser or not self.ser.is_open:
            messagebox.showwarning(APP_TITLE, "Connect to C5VRX first.")
            return

        self.total_target = int(self.total_var.get())
        self.total_received = 0
        self.block_index = 0
        self.block_requested = 0
        self.block_packet_received = False
        self.words = []
        self.block_sizes = []
        self.capture_started_at = time.monotonic()
        self.capture_active = True
        self.transport_ready = False

        self.capture_btn.configure(state="disabled")
        self.cancel_btn.configure(state="normal")
        self.copy_btn.configure(state="disabled")
        self.save_btn.configure(state="disabled")
        self.progress["value"] = 0
        self.progress_var.set("Opening fast binary USB transport...")
        self.log_line(
            f"=== FAST LONG CAPTURE START total={self.total_target} ==="
        )

        # This starts only the framed USB transport. LIVE START is deliberately
        # not sent. Firmware uses this state as explicit negotiation for binary
        # finite-IQ packets while CAPTURE itself remains backward compatible.
        self.send("USB PREVIEW START")

    def start_next_block(self) -> None:
        if not self.capture_active:
            return
        remaining = self.total_target - self.total_received
        if remaining <= 0:
            self.finish_capture(True, "complete")
            return

        self.block_index += 1
        self.block_requested = min(MAX_DEVICE_BLOCK, remaining)
        self.block_packet_received = False
        self.send(f"CAPTURE {self.block_requested}")
        self.update_progress()

    def cancel_capture(self) -> None:
        if self.capture_active:
            self.finish_capture(False, "cancelled by user")

    def reader_loop(self) -> None:
        ser = self.ser
        if not ser:
            return
        decoder = StreamDecoder()
        try:
            while not self.stop_event.is_set() and ser.is_open:
                chunk = ser.read(65536)
                if not chunk:
                    continue
                for kind, value in decoder.feed(chunk):
                    if kind == "line":
                        self.handle_line(str(value))
                    elif kind == "packet":
                        assert isinstance(value, Packet)
                        self.handle_packet(value)
                    else:
                        self.log_line(f"C5VRX_BINARY_RESYNC reason={value}")
        except Exception as exc:
            if not self.stop_event.is_set():
                self.log_line(f"Serial reader stopped: {exc}")
                self.after(
                    0, self.finish_capture, False, f"serial error: {exc}"
                )

    @staticmethod
    def field(line: str, key: str) -> str | None:
        prefix = key + "="
        for part in line.split():
            if part.startswith(prefix):
                return part[len(prefix):]
        return None

    def handle_line(self, line: str) -> None:
        if line.startswith("C5VRX_USB_PREVIEW state=START"):
            code = self.field(line, "code")
            if self.capture_active and code == "0":
                self.transport_ready = True
                self.log_line("Binary IQ transport ready (CRC framed).")
                self.after(0, self.start_next_block)
            elif self.capture_active:
                self.after(
                    0, self.finish_capture, False,
                    f"could not open binary transport: {line}"
                )
            return

        if line.startswith("C5VRX_IQ_BINARY_BEGIN"):
            self.log_line(
                f"Receiving binary block {self.block_index} "
                f"({self.block_requested:,} words)..."
            )
            return

        if line.startswith("C5VRX_CAPTURE_DONE") and self.capture_active:
            try:
                code = int(self.field(line, "code") or "-1")
            except ValueError:
                code = -1
            if code != 0:
                self.after(
                    0, self.finish_capture, False,
                    f"device capture failed: {line}"
                )
                return
            if not self.block_packet_received:
                self.after(
                    0, self.finish_capture, False,
                    f"capture {self.block_index} completed without IQ packet"
                )
                return
            self.after(1, self.start_next_block)
            return

        # Keep the visible console compact. Never render per-sample IQ text.
        if line.startswith("C5VRX_") or line.startswith(("E (", "W (")):
            self.log_line(line)

    def handle_packet(self, packet: Packet) -> None:
        if packet.packet_type != PACKET_IQ_U32_BLOCK:
            return
        if not self.capture_active:
            return
        try:
            block = decode_iq_block(packet)
        except ValueError as exc:
            self.after(
                0, self.finish_capture, False, f"invalid IQ packet: {exc}"
            )
            return

        if len(block) != self.block_requested:
            self.after(
                0,
                self.finish_capture,
                False,
                f"short binary block {self.block_index}: "
                f"received {len(block)}/{self.block_requested}",
            )
            return
        if self.block_packet_received:
            self.after(
                0, self.finish_capture, False,
                f"duplicate IQ packet for block {self.block_index}"
            )
            return

        self.words.extend(block)
        self.block_sizes.append(len(block))
        self.total_received += len(block)
        self.block_packet_received = True
        self.log_line(
            f"Block {self.block_index}: {len(block):,} IQ words received "
            f"in one binary packet."
        )
        self.after(0, self.update_progress)

    def update_progress(self) -> None:
        if not self.total_target:
            return
        pct = min(100.0, self.total_received * 100.0 / self.total_target)
        self.progress["value"] = pct
        elapsed = max(0.001, time.monotonic() - self.capture_started_at)
        rate = self.total_received / elapsed if self.total_received else 0.0
        remaining = self.total_target - self.total_received
        eta = remaining / rate if rate else 0.0
        self.progress_var.set(
            f"{self.total_received:,}/{self.total_target:,} words  •  "
            f"block {self.block_index}  •  {pct:.1f}%"
            + (f"  •  {rate:,.0f} words/s  •  ETA ~{eta:.1f}s" if rate else "")
        )

    def finish_capture(self, success: bool, reason: str) -> None:
        if not self.capture_active:
            return
        self.capture_active = False
        self.capture_btn.configure(state="normal")
        self.cancel_btn.configure(state="disabled")

        if self.transport_ready and self.ser and self.ser.is_open:
            self.send("USB PREVIEW STOP")
        self.transport_ready = False

        self.update_progress()
        if self.words:
            self.copy_btn.configure(state="normal")
            self.save_btn.configure(state="normal")

        if success:
            self.progress["value"] = 100
            elapsed = max(0.001, time.monotonic() - self.capture_started_at)
            self.progress_var.set(
                f"Complete: {self.total_received:,} IQ words in "
                f"{self.block_index} binary blocks • {elapsed:.2f}s"
            )
            self.log_line(
                f"=== FAST LONG CAPTURE COMPLETE words={self.total_received} "
                f"blocks={self.block_index} ==="
            )
        else:
            self.progress_var.set(
                f"Stopped: {reason} • partial words={self.total_received:,}"
            )
            self.log_line(f"=== LONG CAPTURE STOPPED reason={reason} ===")

    def iq_text(self) -> str:
        return "".join(f"IQ:{word:08x}\n" for word in self.words)

    def copy_all_iq(self) -> None:
        if not self.words:
            return
        self.copy_btn.configure(state="disabled")
        self.progress_var.set(
            f"Preparing {len(self.words):,} IQ words for clipboard..."
        )

        def worker() -> None:
            text = self.iq_text()
            self.after(0, self._put_clipboard, text)

        threading.Thread(target=worker, daemon=True).start()

    def _put_clipboard(self, text: str) -> None:
        try:
            self.clipboard_clear()
            self.clipboard_append(text)
            self.update()
            self.progress_var.set(
                f"Copied {len(self.words):,} IQ words to clipboard in one go."
            )
        except Exception as exc:
            messagebox.showerror(APP_TITLE, f"Could not copy IQ:\n\n{exc}")
        finally:
            self.copy_btn.configure(state="normal")

    def save_txt(self) -> None:
        if not self.words:
            return
        path = filedialog.asksaveasfilename(
            title="Save C5VRX IQ capture",
            defaultextension=".txt",
            initialfile=f"c5vrx-{len(self.words)}-iq.txt",
            filetypes=[("Text capture", "*.txt"), ("All files", "*.*")],
        )
        if not path:
            return

        try:
            out = Path(path)
            with out.open("w", encoding="ascii", buffering=1024 * 1024) as f:
                f.write("# C5VRX fast binary long finite IQ capture\n")
                f.write(f"# total_words={len(self.words)}\n")
                f.write(f"# blocks={len(self.block_sizes)}\n")
                f.write(f"# block_sizes={','.join(map(str, self.block_sizes))}\n")
                f.write("# continuity=NOT_GUARANTEED blocks_are_separately_retriggered\n")
                offset = 0
                for index, size in enumerate(self.block_sizes, 1):
                    f.write(
                        f"C5VRX_HOST_BLOCK_BEGIN index={index} "
                        f"words={size} offset={offset}\n"
                    )
                    f.write("".join(
                        f"IQ:{word:08x}\n"
                        for word in self.words[offset:offset + size]
                    ))
                    f.write(f"C5VRX_HOST_BLOCK_END index={index}\n")
                    offset += size
            self.progress_var.set(f"Saved {len(self.words):,} IQ words to {out}")
            messagebox.showinfo(APP_TITLE, f"Saved:\n\n{out}")
        except Exception as exc:
            messagebox.showerror(APP_TITLE, f"Could not save IQ:\n\n{exc}")

    def on_close(self) -> None:
        if self.capture_active:
            if not messagebox.askyesno(
                APP_TITLE, "A capture is running. Stop it and close?"
            ):
                return
            self.finish_capture(False, "application closed")
        self.disconnect()
        self.destroy()


if __name__ == "__main__":
    LongCaptureApp().mainloop()
