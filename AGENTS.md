# C5VRX repository grounding

`Twotoz/C5VRX` is the canonical project repository.

- Current implementation: `/main`
- Current hardware-proven findings: `/docs`
- Historical experiments: `/legacy/c5vrx1`
- Preserved archive discussions: `/docs/legacy-issues`

`legacy/c5vrx1` is reference material, not current production code. Always
search it when investigating ESP32-C5 RF/PHY, MODEM_DIAG, IQ capture, analog
video, WBFM, PARLIO, DAC hardware, receiver-console behavior, or an architecture
that may already have been attempted.

When historical assumptions conflict with newer physical C5VRX evidence,
current hardware findings in `/docs` take precedence. Do not reintroduce a
rejected architecture before reading the corresponding current and legacy
findings that explain its failure.

## Current realtime invariants

- VTX presence and USB must never gate or pace IQ production.
- The normal live source is MODEM_DIAG Q4/I4 captured by PARLIO RX; active
  MAC-owned dump SRAM is a diagnostic writer, not a readable live source.
- Do not turn a physical SRAM or DMA block boundary into a DSP reset.
- Do not claim sample-gapless RF or AV transport without its physical proof.
- The normal live path recovers the transmitted composite waveform; it does not
  decode pixels or regenerate PAL/NTSC.
- Keep USB/debug outside realtime pacing.
- Do not silently change the tested XIAO D4..D9 DAC pin order or the physical
  8.2k/3.9k/2k/1k/470R/240R plus 200R network.
