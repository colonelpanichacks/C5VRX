# Archive issue #5 — Production AV path

| Field | Value |
|---|---|
| Repository | `Twotoz/C5VRX-archive` |
| Created | 2026-08-21 |
| State at export | Open |
| Original | [Issue #5](https://github.com/Twotoz/C5VRX-archive/issues/5) |

## Historical conclusion

This became the archive repository's main product-direction thread: receive A1
RF, demodulate WBFM, preserve the transmitter's composite waveform, and drive
the XIAO six-resistor DAC without a framebuffer. It also fixed the physical DAC
pin/resistor assumptions and kept USB outside realtime pacing.

The discussion recorded useful hardware lessons: correct H-sync polarity was
required for lock; a synthetic raster or plausible counters were not proof of
RF-derived video; and the receiver should remain a dedicated 5 GHz realtime
device rather than mixing networking into the datapath.

## Later status

- **Later confirmed:** direct recovered CVBS, the six-bit DAC, and a USB-free
  realtime path are the right architecture.
- **Superseded:** the archive's finite-capture/rearm implementation and several
  proposed state machines are not the modern production path.
- **Current evidence:** modern C5VRX obtained recognizable, locked live NTSC via
  MODEM_DIAG, PARLIO RX, TX BitScrambler WBFM, and the resistor DAC.

Use this issue for architectural lineage, not as current firmware authority.

