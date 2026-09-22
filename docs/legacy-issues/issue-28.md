# Archive issue #28 — Native continuous IQ architecture

| Field | Value |
|---|---|
| Repository | `Twotoz/C5VRX-archive` |
| Created | 2026-09-04 |
| State at export | Open |
| Original | [Issue #28](https://github.com/Twotoz/C5VRX-archive/issues/28) |

## Why this issue matters

Issue #28 assembled the architecture that unlocked modern C5VRX. It identified
the fixed 64 KiB RF dump SRAM as a circular pre-trigger recorder rather than a
sequence of inherently finite software captures, and proposed using TX_START
as a deliberately unsatisfied trigger in dump-first mode.

It also established the intended signal path:

```text
continuous complex IQ
-> phase discriminator
-> recovered raw CVBS
-> real-domain filtering/rate conversion
-> continuous PARLIO/GDMA
-> six-bit resistor DAC
```

The issue correctly separated physical SRAM wrapping from proof of gapless RF
time and proposed a coherent-tone phase test across `16383 -> 0`.

## Later hardware findings

- **Confirmed:** one vendor-derived TX_START/dump-first start kept ENABLE set,
  the writer pointer moving, and the physical ring wrapping for at least 10,000
  observed wraps with zero software rearms or triggers.
- **Not confirmed:** coherent adjacent phase across physical SRAM wrap. The
  continuous writer must not be described as proven sample-gapless.
- **Rejected as production source:** while MAC ownership is active, CPU and
  AHB-GDMA see a stale/unusable dump-SRAM view; APM did not explain it.
- **Replacement confirmed:** MODEM_DIAG exposes the live Q4/I4 stream. PARLIO
  captures every second native MODEM sample at 40 MS/s and TX BitScrambler
  emits recovered CVBS at 20 MS/s.
- **End-to-end confirmed:** modern C5VRX produced a locked and clearly
  recognizable live NTSC picture. Picture quality and DMA-boundary continuity
  remain active work.

This issue is essential lineage, but the exact current topology is documented
in [continuous-iq-findings.md](../continuous-iq-findings.md) and current source.

