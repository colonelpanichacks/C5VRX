# C5VRX one-click flasher

The Windows console is built from `tools/C5VRX_Flasher.py`. CI produces separate
DevKitC/WROOM and Seeed Studio XIAO executables, each bundling its own A4 /
5805 MHz firmware and `profile.json`. The GUI displays the bundled board and AV
map before flashing and compares it with the firmware's `STATUS profile=...`
after reconnecting. Never substitute one executable's firmware for the other:
their six PARLIO AV GPIOs differ.

After flashing, run `tools/c5vrx_bench.py` as described in
`research/a4-bench-test.md`. It records the physical BitScrambler self-test,
VTX-off/on raw captures, finite RF-to-WBFM proof and chained-capture diagnostic
in one machine-readable report.
