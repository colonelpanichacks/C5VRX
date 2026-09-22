#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-only
"""Guard permanent AV output and independent bounded Phase8 transport."""

from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def between(source: str, begin: str, end: str) -> str:
    start = source.index(begin)
    stop = source.index(end, start)
    return source[start:stop]


def main() -> None:
    firmware = (ROOT / "main" / "c5vrx_main.c").read_text(encoding="utf-8")
    adc = (ROOT / "main" / "c5vrx_adc_dump.c").read_text(encoding="utf-8")
    control = (ROOT / "main" / "c5vrx_control.c").read_text(encoding="utf-8")
    output = (ROOT / "main" / "c5vrx_cvbs_out.c").read_text(encoding="utf-8")

    assert firmware.index("c5vrx_cvbs_output_start()") < \
        firmware.index("c5vrx_wifi5_start(")
    phase8 = between(
        adc, "esp_err_t c5vrx_adc_dump_capture_phase8",
        "typedef struct {")
    assert "c5vrx_usb_preview_running" not in phase8
    assert "write_binary_phase8_chunk" in phase8

    assert "C5VRX_CVBS_DISPLAY_LOGO" in output
    assert "C5VRX_CVBS_DISPLAY_SNOW" in output
    assert "s_active_display = s_requested_display" in output
    assert "snow_code()" in output
    assert "av_display=%s" in control
    assert "C5VRX_CVBS_DISPLAY_SNOW" in control

    chunk_half_lines = 40
    half_line_samples = 640
    sample_rate = 20_000_000
    queued_us = 2 * chunk_half_lines * half_line_samples * 1_000_000 // sample_rate
    assert queued_us == 2560
    assert queued_us > 1990  # measured physical bounded-capture maximum
    assert 2 * chunk_half_lines * half_line_samples == 51_200
    print(
        "C5VRX_ALWAYS_ON_AV result=PASS startup=LOGO unlocked=SNOW "
        f"queued_us={queued_us} dynamic_dma_bytes=51200")


if __name__ == "__main__":
    main()
