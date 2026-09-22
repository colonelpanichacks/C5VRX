# Archive issue #22 — The finite 16K restart boundary

| Field | Value |
|---|---|
| Repository | `Twotoz/C5VRX-archive` |
| Created | 2026-08-26 |
| State at export | Open |
| Original | [Issue #22](https://github.com/Twotoz/C5VRX-archive/issues/22) |

## Historical measurement

This issue isolated the restart boundary of the old 16,384-word finite dump.
REGDMA/LP experiments reached 22,496 rearms with zero reported failures; one
run measured a maximum restart sequence of 204 cycles (about 816 ns). The work
also separated START-only experiments from ENABLE-toggle experiments.

## Correct interpretation

Those results establish a fast, reliable control primitive. They do **not**
prove that no RF samples were lost between captures, nor that the resulting
CVBS waveform was continuous.

## Later status

The production path no longer rearms the dump at every 16K boundary. Preserve
this issue for register sequencing, REGDMA, and negative/diagnostic history;
do not use its successful rearm count as a claim of gapless IQ.

