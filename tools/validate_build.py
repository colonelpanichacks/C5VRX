#!/usr/bin/env python3
"""C5VRX-3 build validation script.

Checks that the production source tree meets all architectural constraints.
Run from the C5VRX-3 project root.

Exit 0: all checks pass.
Exit 1: one or more checks failed (details printed).
"""
import sys
import re
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
MAIN = ROOT / "main"

failures = []
passes = []


def check(name, condition, detail=""):
    if condition:
        passes.append(name)
    else:
        failures.append(f"FAIL: {name}" + (f" -- {detail}" if detail else ""))


def read(path):
    return path.read_text(encoding="utf-8", errors="replace")


# ---- BitScrambler program checks ----
# Two programs: the proven Phase5 default (fm.bsasm) and the opt-in
# Trajectory v2 weak-signal experiment (fm_traj.bsasm, runtime A/B 'T').
bsasm_files = list(MAIN.glob("*.bsasm"))
check("fm.bsasm + fm_traj.bsasm present",
      {f.name for f in bsasm_files} == {"fm.bsasm", "fm_traj.bsasm"},
      f"found {[f.name for f in bsasm_files]}")

if bsasm_files:
    bsasm = "\n".join(read(f) for f in bsasm_files)
    check("fm.bsasm: cfg eof_on downstream", "cfg eof_on downstream" in bsasm)
    check("fm.bsasm: cfg trailing_bytes 0", "cfg trailing_bytes 0" in bsasm)
    check("fm.bsasm: cfg prefetch true", "cfg prefetch true" in bsasm)
    check("fm.bsasm: cfg lut_width_bits 16", "cfg lut_width_bits 16" in bsasm)
    check("fm.bsasm: NO eof_on upstream", "cfg eof_on upstream" not in bsasm,
          "upstream EOF semantics must not appear")
    check("fm.bsasm: NO trailing_bytes 9", "trailing_bytes 9" not in bsasm,
          "9-byte tail causes 225 ns discard bug")

# ---- Production .c file checks ----
c_files = list(MAIN.glob("*.c"))
all_c = "\n".join(read(f) for f in c_files)
c_names = [f.name for f in c_files]

check("production receiver and dedicated menu raster modules", set(c_names) == {"main.c", "arc_phy.c", "rf.c", "video.c", "menu_raster.c", "buzzer.c", "status_led.c", "preview.c", "grab.c"},
      f"found: {c_names}")
check("main.c present", "main.c" in c_names)
check("rf.c present", "rf.c" in c_names)
check("video.c present", "video.c" in c_names)
check("menu bypasses the demodulator", "bitscrambler_disable(s_flight_bs)" in all_c)
check("no synthetic menu IQ", "s_black_iq" not in all_c and "get_white_word" not in all_c)
check("no live ring splice", not re.search(r"s_tx_dscr_nodes\[.*?->next\s*=", all_c))
check("serialized menu commands", "xQueueSend(s_menu_commands" in all_c and "xQueueReceive(s_menu_commands" in all_c)

check("no continuous_iq in production", "continuous_iq" not in all_c,
      "RF dump engine must not be present")
check("no telemetry_task in production", "telemetry_task" not in all_c,
      "periodic telemetry task must not be present")
check("no calibration subsystem in production",
      "calibration_get" not in all_c and "calibration.h" not in all_c,
      "runtime calibration must not be present")
check("no RF dump engine in production", "continuous_iq" not in all_c and "s_rf_dump" not in all_c,
      "RF dump subsystem must not be present")
check("no startup_trace in production", "startup_trace" not in all_c)
check("no snapshot infrastructure", "live_snapshot" not in all_c)
check("no trajectory in production", "trajectory" not in all_c)
check("no true40 in production", "true40" not in all_c)
check("no wbfm_q4.h in production", "wbfm_q4.h" not in all_c)

# Fixed constants
check("RAW_RING_BYTES == 16384",
      bool(re.search(r"RAW_RING_BYTES\s+16384", all_c)))
check("DAC_IDLE_CODE == 20",
      bool(re.search(r"DAC_IDLE_CODE\s+20", all_c)))
check("IQ_RATE_HZ == 40000000",
      bool(re.search(r"IQ_RATE_HZ\s+40000000", all_c)))

# RX POS edge (not NEG)
check("PARLIO_SAMPLE_EDGE_POS in video.c",
      "PARLIO_SAMPLE_EDGE_POS" in all_c)
check("no PARLIO_SAMPLE_EDGE_NEG for RX",
      "PARLIO_SAMPLE_EDGE_NEG" not in all_c,
      "RX must use POS edge; NEG is for TX shift edge only (PARLIO_SHIFT_EDGE_NEG)")

# TX NEG shift edge
check("PARLIO_SHIFT_EDGE_NEG in video.c",
      "PARLIO_SHIFT_EDGE_NEG" in all_c)

# loop_transmission
check("loop_transmission present", "loop_transmission" in all_c)

# Zero-EOF descriptor patch
check("Zero-EOF descriptor patch present", "patch_descriptors_clear_eof" in all_c,
      "Zero-EOF circular descriptor patch must be present to prevent wrap bubbles")

# No periodic tasks
check("no periodic telemetry or timer tasks in production",
      "telemetry_task" not in all_c and "hw_diag_task" not in all_c,
      "periodic tasks must not be present")

# BitScrambler programs (proven default + opt-in trajectory A/B)
cmake_main = read(MAIN / "CMakeLists.txt")
bs_srcs = re.findall(r'target_bitscrambler_add_src\("([^"]+)"\)', cmake_main)
check("two BitScrambler programs in CMakeLists",
      len(bs_srcs) == 2, f"found: {bs_srcs}")
if bs_srcs:
    check("BitScrambler programs are fm.bsasm + fm_traj.bsasm",
          set(bs_srcs) == {"fm.bsasm", "fm_traj.bsasm"})

# ---- Summary ----
print(f"\n{'='*50}")
print(f"C5VRX-3 build validation: {len(passes)} passed, {len(failures)} failed")
print(f"{'='*50}")
if failures:
    for f in failures:
        print(f)
    sys.exit(1)
else:
    print("All checks passed.")
    sys.exit(0)
