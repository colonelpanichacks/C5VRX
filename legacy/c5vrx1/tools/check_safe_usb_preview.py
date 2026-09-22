#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-only
"""Guard the XIAO USB-preview route against unsafe continuous RF ownership."""

from __future__ import annotations

import ast
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def function_source(path: Path, name: str) -> str:
    source = path.read_text(encoding="utf-8")
    tree = ast.parse(source, filename=str(path))
    for node in ast.walk(tree):
        if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef)) and node.name == name:
            return ast.get_source_segment(source, node) or ""
    raise AssertionError(f"missing Python function: {name}")


def c_function_source(path: Path, signature: str) -> str:
    source = path.read_text(encoding="utf-8")
    start = source.index(signature)
    opening = source.index("{", start)
    depth = 0
    for index in range(opening, len(source)):
        if source[index] == "{":
            depth += 1
        elif source[index] == "}":
            depth -= 1
            if depth == 0:
                return source[start:index + 1]
    raise AssertionError(f"unterminated C function: {signature}")


def main() -> None:
    flasher = ROOT / "tools" / "C5VRX_Flasher.py"
    control = ROOT / "main" / "c5vrx_control.c"

    button = function_source(flasher, "start_experimental_usb_preview")
    assert "_start_live_iq_video" in button
    assert "LIVE EXPERIMENTAL START" not in button
    assert "USB PREVIEW START" not in button

    transport = function_source(flasher, "_start_bounded_capture_transport")
    request = function_source(flasher, "_request_live_iq_capture")
    assert "_refill_live_iq_pipeline" in transport
    assert "CAPTURE PHASE8 16384" in request

    stalled = function_source(flasher, "_show_usb_transport_stall")
    assert "live_iq_active = False" in stalled
    assert "live_iq_commands_outstanding = 0" in stalled

    ring = c_function_source(
        control,
        "static esp_err_t start_ring_live(c5vrx_rf_dump_mode_t mode,")
    assert "return ESP_ERR_NOT_SUPPORTED;" in ring
    assert "c5vrx_live_ring_source_create" not in ring
    assert "c5vrx_live_pipeline_start" not in ring

    control_source = control.read_text(encoding="utf-8")
    assert "HP_SRAM_MAC_OWNERSHIP_WEDGES_FREERTOS_USB" in control_source
    print("C5VRX_SAFE_USB_PREVIEW result=PASS route=CAPTURE_PHASE8_BOUNDED")


if __name__ == "__main__":
    main()
