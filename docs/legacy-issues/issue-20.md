# Archive issue #20 — VTX-on startup underrun and HP watchdog

| Field | Value |
|---|---|
| Repository | `Twotoz/C5VRX-archive` |
| Created | 2026-08-25 |
| Closed | 2026-08-25 |
| Original | [Issue #20](https://github.com/Twotoz/C5VRX-archive/issues/20) |

## Historical measurement

The direct AV experiment could start with an empty FIFO while the HP core was
parked. That produced startup underrun and a task-watchdog reset even though the
LP-side RF rearms were still operating.

## Historical conclusion

Prime the output before enabling transport, keep the HP scheduler alive, and
do not make USB preview part of realtime operation.

## Later status

The exact old implementation is superseded, but the failure remains a useful
warning: DMA startup order and producer/consumer ownership must be explicit.
Modern C5VRX uses an elastic raw ring and continuous PARLIO transport rather
than the old HP-parked design.

