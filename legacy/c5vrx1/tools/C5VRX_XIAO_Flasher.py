#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-only
"""XIAO ESP32-C5 Receiver Console with hardened native-USB flashing.

The XIAO uses the ESP32-C5 USB-Serial/JTAG peripheral directly. On Windows a
runtime reader which has only just been closed can briefly keep I/O pending on
the COM handle. This wrapper keeps the common Receiver Console UI while making
XIAO flash/reset transitions explicit, retryable, and runtime-readiness aware.
"""

from __future__ import annotations

import subprocess
import sys
import tempfile
import threading
import time
import traceback
from contextlib import redirect_stderr, redirect_stdout
from pathlib import Path
from tkinter import messagebox

import esptool
import serial

from C5VRX_Flasher import C5VRXApp, load_flash_plan


class C5VRXXIAOApp(C5VRXApp):
    """Receiver Console with XIAO/native-USB specific flash handling."""

    def __init__(self) -> None:
        super().__init__()
        self._runtime_seen = False
        self._runtime_probe_remaining = 0
        self._runtime_waiting = False
        self._runtime_deadline = 0.0
        self._runtime_reconnect_scheduled = False

    def disconnect_serial(self) -> None:
        """Close runtime USB and wait until the reader no longer owns the port."""
        self.serial_stop.set()
        self.preview_sequence = None

        ser = self.ser
        self.ser = None
        reader = self.serial_thread
        self.serial_thread = None

        if ser:
            # Cancel any overlapped Win32 I/O before close so esptool does not
            # race a still-pending ReadFile/ClearCommError on the same COM port.
            try:
                ser.cancel_read()
            except Exception:
                pass
            try:
                ser.cancel_write()
            except Exception:
                pass
            try:
                ser.close()
            except Exception:
                pass

        if (reader is not None and reader is not threading.current_thread()
                and reader.is_alive()):
            reader.join(timeout=1.5)

        self.connection_var.set("Disconnected")
        self.connect_btn.configure(text="Connect")

    @staticmethod
    def _replace_esptool_option(args: list[str], name: str, value: str) -> list[str]:
        out: list[str] = []
        i = 0
        while i < len(args):
            if args[i] == name and i + 1 < len(args):
                i += 2
                continue
            out.append(args[i])
            i += 1
        out += [name, value]
        return out

    @staticmethod
    def _retryable_process_output(output: str) -> bool:
        text = output.lower()
        return any(token in text for token in (
            "serialtimeoutexception",
            "write timeout",
            "port is busy",
            "doesn't exist",
            "access is denied",
            "toegang geweigerd",
            "permissionerror",
            "could not open",
            "semafoor",
        ))

    @staticmethod
    def _port_openable(port: str) -> bool:
        """Probe COM ownership without asserting reset-control lines."""
        probe = serial.Serial()
        probe.port = port
        probe.baudrate = 115200
        probe.timeout = 0.05
        probe.write_timeout = 0.05
        probe.dtr = False
        probe.rts = False
        try:
            probe.open()
            return True
        except (OSError, serial.SerialException):
            return False
        finally:
            try:
                probe.close()
            except Exception:
                pass

    def _wait_port_openable(self, port: str, timeout_s: float) -> bool:
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            if self._port_openable(port):
                return True
            time.sleep(0.25)
        return False

    @staticmethod
    def _worker_command(log_path: Path, argv: list[str]) -> list[str]:
        if getattr(sys, "frozen", False):
            return [
                sys.executable,
                "--c5vrx-esptool-worker",
                str(log_path),
                *argv,
            ]
        return [
            sys.executable,
            str(Path(__file__).resolve()),
            "--c5vrx-esptool-worker",
            str(log_path),
            *argv,
        ]

    def _run_esptool_process(self, argv: list[str]) -> tuple[int, str]:
        """Run one esptool connection in an isolated OS process."""
        with tempfile.NamedTemporaryFile(
                prefix="c5vrx-esptool-", suffix=".log", delete=False) as tmp:
            log_path = Path(tmp.name)
        creationflags = getattr(subprocess, "CREATE_NO_WINDOW", 0)
        try:
            completed = subprocess.run(
                self._worker_command(log_path, argv),
                stdin=subprocess.DEVNULL,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                check=False,
                timeout=120,
                creationflags=creationflags,
            )
            output = log_path.read_text(encoding="utf-8", errors="replace")
            return completed.returncode, output
        except subprocess.TimeoutExpired:
            return 124, "C5VRX_ESPTOOL_WORKER_TIMEOUT duration_s=120\n"
        finally:
            try:
                log_path.unlink()
            except OSError:
                pass

    def _flash_worker(self, port: str) -> None:
        try:
            # Let Windows finish retiring the runtime COM handle before the
            # ROM-loader connection opens the same device again.
            time.sleep(0.45)

            base_args, files = load_flash_plan()
            base_args = self._replace_esptool_option(
                base_args, "--before", "usb-reset")
            base_args = self._replace_esptool_option(
                base_args, "--after", "watchdog-reset")

            argv = base_args + ["--port", port, "write-flash"]
            for offset, path in files:
                argv += [str(offset), str(path)]

            self.sink.write("Firmware image set:\n")
            for offset, path in files:
                self.sink.write(f"  {offset}: {path.name}\n")
            self.sink.write(
                "\nConnecting to ESP32-C5 bootloader "
                "(XIAO USB reset mode)...\n")

            max_attempts = 4
            for attempt in range(1, max_attempts + 1):
                if attempt > 1:
                    delay = 0.75 + (attempt - 2) * 0.5
                    self.sink.write(
                        f"\nC5VRX_FLASH_RETRY attempt={attempt}/{max_attempts} "
                        f"delay_s={delay:.2f} reason=USB_COM_TRANSITION "
                        "isolation=NEW_PROCESS\n")
                    time.sleep(delay)
                if not self._wait_port_openable(port, 8.0):
                    self.sink.write(
                        f"C5VRX_FLASH_PORT_WAIT attempt={attempt}/{max_attempts} "
                        f"port={port} result=TIMEOUT\n")
                    if attempt < max_attempts:
                        continue
                    raise RuntimeError(f"{port} did not become exclusively openable")

                self.sink.write(
                    f"C5VRX_FLASH_ATTEMPT attempt={attempt}/{max_attempts} "
                    "isolation=NEW_PROCESS port_ready=1\n")
                returncode, output = self._run_esptool_process(argv)
                self.sink.write(output)
                self.sink.write(
                    f"C5VRX_FLASH_PROCESS_EXIT attempt={attempt}/{max_attempts} "
                    f"code={returncode}\n")
                if returncode == 0:
                    self.after(0, self._done_ok)
                    return
                if attempt < max_attempts and self._retryable_process_output(output):
                    self.sink.write(
                        "C5VRX_FLASH_TRANSIENT reason=ISOLATED_SERIAL_FAILURE "
                        "handle_state=PROCESS_DESTROYED\n")
                    continue
                raise RuntimeError(
                    f"isolated esptool process exited with code {returncode}")

        except Exception as exc:
            self.sink.write("\n" + traceback.format_exc() + "\n")
            self.after(0, self._done_error, str(exc))

    def _done_ok(self) -> None:
        self._finish_flash()
        self.sink.write("\n=== FLASH COMPLETE ===\n")
        self.sink.write(
            "C5VRX_FLASH_IMAGE_VERIFIED reset=WATCHDOG "
            "runtime=PHYSICAL_POWER_CYCLE_REQUIRED\n")
        self._runtime_seen = False
        self._runtime_waiting = False
        # Real XIAO ESP32-C5 hardware can retain a wedged native-USB runtime
        # state after esptool's watchdog reset. Do not issue misleading PING
        # probes before the board has been fully unpowered.
        self._runtime_recovery_required("XIAO_C5_POST_FLASH_USB_STATE")

    def _runtime_recovery_required(self, reason: str) -> None:
        self._runtime_waiting = False
        self.disconnect_serial()
        self.sink.write(
            "C5VRX_POST_FLASH_ACTION required=PHYSICAL_USB_POWER_CYCLE "
            "steps=UNPLUG_30S_REPLUG_THEN_CONNECT "
            f"reason={reason}\n")
        self.connection_var.set(
            "Image verified — unplug USB for 30 seconds, replug, then Connect")
        messagebox.showwarning(
            "C5VRX Receiver Console",
            "The firmware image was written and verified. A full power cycle is "
            "required before using the XIAO runtime.\n\n"
            "1. Unplug the XIAO USB cable for 30 seconds.\n"
            "2. Reconnect it without pressing BOOT or RESET.\n"
            "3. Press Connect.\n\n"
            "Do not start video proof until the terminal shows C5VRX_PONG or "
            "C5VRX_READY.",
        )

    def _schedule_runtime_reconnect(self, delay_ms: int) -> None:
        if self._runtime_reconnect_scheduled or not self._runtime_waiting:
            return
        self._runtime_reconnect_scheduled = True

        def run() -> None:
            self._runtime_reconnect_scheduled = False
            self._auto_reconnect_after_flash()

        self.after(delay_ms, run)

    def _auto_reconnect_after_flash(self) -> None:
        if self._runtime_seen or not self._runtime_waiting:
            return
        if time.monotonic() >= self._runtime_deadline:
            self.sink.write(
                "C5VRX_RUNTIME_TIMEOUT no=PONG/READY duration_s=30 "
                "classification=USB_REENUMERATION_OR_RUNTIME_NOT_READY\n")
            self._runtime_recovery_required("NO_PONG_OR_READY_AFTER_WATCHDOG_RESET")
            return
        self.refresh_ports()
        port = self.selected_port()
        if not port or not self._port_openable(port):
            self.connection_var.set("Waiting for exclusive runtime USB access...")
            self._schedule_runtime_reconnect(500)
            return
        if self.connect_serial(show_errors=False):
            self.sink.write(
                "C5VRX USB port reopened after readiness probe; "
                "probing runtime until PONG/READY...\n")
            self.connection_var.set("USB open — waiting for C5VRX_PONG...")
            self._runtime_probe_remaining = 12
            self.after(650, self._runtime_probe)
        else:
            self._schedule_runtime_reconnect(500)

    def _runtime_probe(self) -> None:
        if self._runtime_seen:
            return
        ser = self.ser
        if not ser or not ser.is_open:
            self.connection_var.set("Runtime USB re-enumerated; reopening...")
            self._schedule_runtime_reconnect(500)
            return
        if self._runtime_probe_remaining <= 0:
            self.sink.write(
                "C5VRX_RUNTIME_TIMEOUT no=PONG/READY duration_s~10 "
                "classification=FIRMWARE_BOOT_OR_CONTROL_NOT_READY\n")
            self._runtime_recovery_required("NO_PONG_OR_READY_AFTER_12_PROBES")
            return
        self._runtime_probe_remaining -= 1
        try:
            ser.write(b"PING\n")
            ser.flush()
            self.sink.write(
                f"> PING [runtime probe {12 - self._runtime_probe_remaining}/12]\n")
        except Exception as exc:
            self.sink.write(f"C5VRX_RUNTIME_PROBE_WRITE_FAILED error={exc}\n")
            self.disconnect_serial()
            self._schedule_runtime_reconnect(500)
            return
        self.after(800, self._runtime_probe)

    def _serial_lost(self) -> None:
        self.disconnect_serial()
        if self._runtime_waiting and not self._runtime_seen:
            self.connection_var.set("Runtime USB changed; waiting to reopen...")
            self._schedule_runtime_reconnect(500)

    def _parse_device_line(self, line: str) -> None:
        if (line.startswith("C5VRX_READY") or line.startswith("C5VRX_PONG")
                or line.startswith("C5VRX_STATUS")):
            first = not self._runtime_seen
            self._runtime_seen = True
            self._runtime_waiting = False
            if first:
                self.connection_var.set(
                    f"Connected: {self.selected_port()} — C5VRX runtime ready")
                self.sink.write("C5VRX_RUNTIME_READY handshake=PASS\n")
                if not line.startswith("C5VRX_STATUS"):
                    self.after(100, lambda: self.send_command("STATUS"))
        super()._parse_device_line(line)


def _esptool_worker(log_name: str, argv: list[str]) -> int:
    """Child-process entry point; exit guarantees Win32 handle teardown."""
    with Path(log_name).open("w", encoding="utf-8", buffering=1) as output:
        try:
            with redirect_stdout(output), redirect_stderr(output):
                esptool.main(argv)
            return 0
        except SystemExit as exc:
            return 0 if exc.code in (0, None) else int(exc.code)
        except BaseException:
            traceback.print_exc(file=output)
            return 1


if __name__ == "__main__":
    if len(sys.argv) >= 3 and sys.argv[1] == "--c5vrx-esptool-worker":
        raise SystemExit(_esptool_worker(sys.argv[2], sys.argv[3:]))
    C5VRXXIAOApp().mainloop()
