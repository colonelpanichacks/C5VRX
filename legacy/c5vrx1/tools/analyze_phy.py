#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-only
"""Generate a focused reverse-engineering report for ESP32-C5 PHY/RF-test blobs.

The script is deliberately read-only: it never patches vendor libraries. It uses
Espressif's RISC-V binutils to extract symbol sizes, focused disassembly,
relocation-based call relationships, caller context and literal/MMIO context
around the receive/dump/frequency functions that matter to C5VRX.
"""

from __future__ import annotations

import argparse
import os
import pathlib
import re
import shutil
import subprocess
from collections import defaultdict

TARGETS = [
    # RF-test / factory dump path
    "adctrig",
    "sampledeal",
    "accumiq",
    "get_iq_est_pwr",
    "dc_iq_est_test",
    "get_rx_buffer",
    "get_rx_data_addr",
    "set_dump_mode",
    "print_dump_data",
    "loop_dump_test",
    "fedump_rd_rxmem",
    "fedump_rd_txmem",
    # PHY/front-end dump and IQ helpers (keep old + current name variants)
    "phy_chan_dump_cfg",
    "phy_chan_dump_cfg_752",
    "phy_adc_rate_set",
    "phy_fe_adc_on",
    "phy_iq_est_enable",
    "phy_iq_est_enable_new",
    "phy_iq_est_disable",
    "phy_dc_iq_est",
    "phy_dc_iq_est_new",
    "phy_rxiq_get_mis",
    "phy_get_iq_est_snr",
    "phy_fft_scale_force",
    "phy_csidump_force_lltf_cfg",
    # Frequency control candidates / conversion helpers
    "phy_set_freq",
    "phy_set_chanfreq",
    "phy_set_rfpll_freq",
    "phy_set_channel_rfpll_freq_new",
    "phy_write_chan_freq",
    "phy_chan_to_freq",
    "phy_freq_to_chan",
    "phy_set_step_01k",
    "phy_chip_set_chan",
    "phy_chip_set_chan_ana",
    "phy_chip_set_chan_offset",
    "phy_set_rx_pbus_freq",
    "phy_get_rx_pbus_freq",
]

# RISC-V often materializes a 32-bit address as LUI(upper20) + ADDI/load/store.
# Therefore match both complete addresses and the useful upper-immediate forms.
LITERAL_PATTERNS = {
    "finite dump RAM 0x40830000": re.compile(r"\b(?:0x)?40830000\b|\b40830\b", re.I),
    "FE/RX MMIO 0x600a04xx": re.compile(r"\b(?:0x)?600a04[0-9a-f]{2}\b|\b600a0\b", re.I),
}

FUNC_HEADER = re.compile(r"^\s*[0-9a-fA-F]+\s+<([^>]+)>:\s*$")
RELOC_CALL = re.compile(r"R_RISCV_(?:CALL|CALL_PLT|JAL)\s+([^\s+]+)")


def run(cmd: list[str]) -> str:
    p = subprocess.run(cmd, text=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, check=False)
    return p.stdout


def find_tool(name: str) -> str:
    for candidate in (f"riscv32-esp-elf-{name}", f"riscv32-unknown-elf-{name}", name):
        p = shutil.which(candidate)
        if p:
            return p
    raise SystemExit(f"Could not find {name}. Source/export your ESP-IDF environment first.")


def is_local_label(name: str) -> bool:
    """objdump renders .L* branch labels like function headers; they are not functions."""
    return name.startswith(".") or name.startswith("$")


def extract_functions(disassembly: str, symbols: list[str], tail_lines: int = 900) -> dict[str, str]:
    lines = disassembly.splitlines()
    wanted = set(symbols)
    hits: dict[str, str] = {}
    for i, line in enumerate(lines):
        m = FUNC_HEADER.match(line)
        if not m or m.group(1) not in wanted:
            continue
        name = m.group(1)
        chunk = [line]
        for nxt in lines[i + 1 : i + 1 + tail_lines]:
            hm = FUNC_HEADER.match(nxt)
            if hm and not is_local_label(hm.group(1)):
                break
            chunk.append(nxt)
        hits[name] = "\n".join(chunk)
    return hits


def symbol_sizes(nm_output: str) -> dict[str, tuple[int, str]]:
    """Return symbol -> (size, original nm line) for common GNU nm -S layouts."""
    out: dict[str, tuple[int, str]] = {}
    for line in nm_output.splitlines():
        parts = line.split()
        if len(parts) < 4:
            continue
        name = parts[-1]
        if name not in TARGETS:
            continue
        size = None
        for token in reversed(parts[:-1]):
            if re.fullmatch(r"[0-9a-fA-F]+", token):
                if size is None:
                    size = int(token, 16)
                else:
                    break
        if size is not None:
            out[name] = (size, line)
    return out


def call_graph(disassembly: str) -> tuple[dict[str, set[str]], dict[str, set[str]]]:
    callers: dict[str, set[str]] = defaultdict(set)
    callees: dict[str, set[str]] = defaultdict(set)
    current = None
    wanted = set(TARGETS)
    for line in disassembly.splitlines():
        m = FUNC_HEADER.match(line)
        if m:
            name = m.group(1)
            if not is_local_label(name):
                current = name
            continue
        if current is None:
            continue
        m = RELOC_CALL.search(line)
        if not m:
            continue
        target = m.group(1)
        if current in wanted or target in wanted:
            callees[current].add(target)
            callers[target].add(current)
    return callers, callees


def callsite_contexts(disassembly: str, targets: list[str], radius: int = 6) -> dict[str, list[str]]:
    """Return compact call-site snippets, retaining the owning non-local function."""
    lines = disassembly.splitlines()
    wanted = set(targets)
    current = None
    contexts: dict[str, list[str]] = defaultdict(list)
    for i, line in enumerate(lines):
        h = FUNC_HEADER.match(line)
        if h and not is_local_label(h.group(1)):
            current = h.group(1)
        m = RELOC_CALL.search(line)
        if not m or m.group(1) not in wanted:
            continue
        target = m.group(1)
        lo = max(0, i - radius)
        hi = min(len(lines), i + radius + 1)
        snippet = [f"# caller: {current or '?'}"] + lines[lo:hi]
        contexts[target].append("\n".join(snippet))
    return contexts


def literal_contexts(disassembly: str, radius: int = 8) -> dict[str, list[str]]:
    """Find code touching known dump-RAM/MMIO address materialization patterns."""
    lines = disassembly.splitlines()
    current = None
    contexts: dict[str, list[str]] = defaultdict(list)
    seen: dict[str, set[tuple[str | None, int]]] = defaultdict(set)

    for i, line in enumerate(lines):
        h = FUNC_HEADER.match(line)
        if h and not is_local_label(h.group(1)):
            current = h.group(1)

        for label, pattern in LITERAL_PATTERNS.items():
            if not pattern.search(line):
                continue
            # Adjacent instructions can contain the same LUI literal; collapse
            # hits within a small window in the same owner function.
            key = (current, i // 4)
            if key in seen[label]:
                continue
            seen[label].add(key)
            lo = max(0, i - radius)
            hi = min(len(lines), i + radius + 1)
            snippet = [f"# owner: {current or '?'}", f"# match: {line.strip()}"] + lines[lo:hi]
            contexts[label].append("\n".join(snippet))
    return contexts


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--idf", default=os.environ.get("IDF_PATH"), help="ESP-IDF root (defaults to IDF_PATH)")
    ap.add_argument("--out", default="research/generated/phy-analysis.md")
    args = ap.parse_args()

    if not args.idf:
        raise SystemExit("IDF_PATH is not set; pass --idf /path/to/esp-idf")

    root = pathlib.Path(args.idf)
    libdir = root / "components" / "esp_phy" / "lib" / "esp32c5"
    if not libdir.exists() and (root / "esp32c5").exists():
        libdir = root / "esp32c5"

    libs = [libdir / "librftest.a", libdir / "libphy.a"]
    missing = [str(p) for p in libs if not p.exists()]
    if missing:
        raise SystemExit("Missing libraries:\n  " + "\n  ".join(missing))

    nm = find_tool("nm")
    objdump = find_tool("objdump")

    report: list[str] = [
        "# ESP32-C5 PHY blob analysis",
        "",
        "Generated by `tools/analyze_phy.py`.",
        "",
        "Goal: identify a path from the 5 GHz front-end/ADC to phase-bearing receive samples before normal 802.11 decode.",
        "",
        "> Function names and call relationships are evidence; undocumented prototypes are not assumed from names alone.",
        "",
    ]

    for lib in libs:
        report += [f"## `{lib.name}`", ""]
        nm_out = run([nm, "-A", "-S", "-C", "--defined-only", str(lib)])
        sizes = symbol_sizes(nm_out)
        report += ["### Target symbol inventory", "", "| Symbol | Size |", "| --- | ---: |"]
        for target in TARGETS:
            if target in sizes:
                report.append(f"| `{target}` | {sizes[target][0]} bytes |")
        if not sizes:
            report.append("| _none found_ | — |")
        report.append("")

        dis = run([objdump, "-dr", "-C", str(lib)])
        callers, callees = call_graph(dis)
        contexts = callsite_contexts(dis, TARGETS)
        literals = literal_contexts(dis)
        funcs = extract_functions(dis, TARGETS)

        report += ["### Relocation-based call graph", ""]
        for target in TARGETS:
            inbound = sorted(callers.get(target, ()))
            outbound = sorted(callees.get(target, ()))
            if not inbound and not outbound:
                continue
            report += [f"#### `{target}`", ""]
            if inbound:
                report.append("Called by: " + ", ".join(f"`{x}`" for x in inbound))
            if outbound:
                report.append("Calls: " + ", ".join(f"`{x}`" for x in outbound))
            report.append("")

        report += ["### Dump-RAM / FE-MMIO literal contexts", ""]
        report += [
            "These snippets are especially useful for tracing the producer behind the finite IQ dump.",
            "RISC-V address materialization may show only the upper immediate, so both full and LUI forms are searched.",
            "",
        ]
        for label in LITERAL_PATTERNS:
            snippets = literals.get(label, [])
            report += [f"#### {label}", ""]
            if not snippets:
                report += ["_No literal/materialization match found in this archive._", ""]
                continue
            for snippet in snippets[:32]:
                report += ["```asm", snippet, "```", ""]

        report += ["### Call-site contexts", ""]
        for target in TARGETS:
            snippets = contexts.get(target, [])
            if not snippets:
                continue
            report += [f"#### `{target}`", ""]
            for snippet in snippets[:12]:
                report += ["```asm", snippet, "```", ""]

        report += ["### Focused disassembly", ""]
        for target in TARGETS:
            body = funcs.get(target)
            if body:
                report += [f"#### `{target}`", "", "```asm", body, "```", ""]

    out = pathlib.Path(args.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text("\n".join(report), encoding="utf-8")
    print(f"Wrote {out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
