# Archive issue #27 — Adjacent-sample FM and recovered CVBS

| Field | Value |
|---|---|
| Repository | `Twotoz/C5VRX-archive` |
| Created | 2026-09-03 |
| State at export | Open |
| Original | [Issue #27](https://github.com/Twotoz/C5VRX-archive/issues/27) |

## Historical conclusion

This issue corrected the earlier DSP model:

```text
complex IQ
-> circular phase difference
-> real composite waveform
-> real-domain filtering/rate conversion
-> DAC
```

It rejected throwing away complex samples before the FM discriminator and
recognized that an analog VTX already transmits sync, blanking, luma, burst,
and chroma as one FM-modulated CVBS waveform. It also insisted that RF and AV
sample rates be measured separately.

## Later status

- **Confirmed:** preserve the VTX's recovered composite waveform; do not decode
  and regenerate PAL/NTSC in the live path.
- **Evolved:** hardware throughput led the compact production discriminator to
  operate on consecutive *retained* 40 MS/s MODEM samples and emit 20 MS/s.
  Read current DSP documentation for the exact implementation.
- **Still open:** long-duration sample continuity and two-sample phase-range
  behavior require physical proof; this issue did not prove them.

