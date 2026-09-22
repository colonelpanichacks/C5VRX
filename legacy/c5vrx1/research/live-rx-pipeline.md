# Live RF -> analog CVBS pipeline

## Current boundary

The source-level non-blocking RF producer and guarded ring source are now
**IMPLEMENTED / NOT PHYSICALLY TESTED**. They configure the hash-pinned vendor
sequence once, observe its live pointer, consume contiguous guarded windows and
stop/restore the saved register state on every failure path. The source remains
named `EXPERIMENTAL_RING_SOURCE_UNPROVEN` until cadence, wrap-flag and coherent
phase-boundary probes pass on real ESP32-C5 silicon.

This supersedes the older claim that the producer/ring mechanism had not been
reconstructed. What remains unknown is its physical cadence, bandwidth, signal
stage, sustained contention margin and continuity through wrap—not its software
control or reader architecture.

## Implemented streaming route

```text
5.8 GHz RF
  -> vendor-observed dump producer (mode 0/11; default off; mode 12 rejected)
  -> 0x40830000 16,384-word ring
  -> guarded short immutable copy
  -> persistent BitScrambler 4:1 phase discriminator
  -> DC/gain/polarity/filter conditioner
  -> fixed-size sample chunker
  -> two-buffer PARLIO/GDMA
  -> 6-bit passive DAC
  -> 75-ohm analog AV
```

There is no full framebuffer. `LIVE START` is fail-closed behind measured
cadence, coherent wrap, staged soak, alias-safe bandwidth, BitScrambler,
PARLIO and simultaneous real-ring pipeline gates. Retuning invalidates the
RF-dependent gates. `NEARLIVE START` remains a clearly labelled finite-chain
diagnostic and is never treated as continuity evidence.

The ring reader tracks writer/reader distance, service ambiguity, guard
collisions, wraps, missed words, dropped blocks, total and maximum copy cycles,
and producer stop. It splits at wrap and copies immediately into bounded
DMA-capable ownership buffers so the existing asynchronous source ABI continues
to guarantee immutable blocks.

## Reduction choice

The selected 4:1 BitScrambler program retains samples `x[0], x[4], x[8], ...`
and emits a coarse phase difference between successive retained samples. It
does not contain an anti-alias filter. Consequently, `/4` is allowed only when
hardware measurement establishes that the selected FE tap is sufficiently
band-limited. The physical RF rate is still unknown; 80 complex MS/s is a
design case, not a claimed measurement.

Direct ring-to-BitScrambler DMA is statically addressable but deliberately not
implemented yet. The required `BENCH RING PIPELINE 0 1000 1024`, `... 2048`
and `... 4096` matrix reports actual copy bytes/s,
total copy cycles and CPU percentage. It emits one of:

```text
zero_copy_action=KEEP_IMMUTABLE_COPY
zero_copy_action=IMPLEMENT_ZERO_COPY
zero_copy_action=NO_RESULT
```

`IMPLEMENT_ZERO_COPY` requires measured copy occupancy of at least 25% or an
actual `COPY_AMBIGUOUS` failure. Until then, the safer immutable-copy route is
kept. A future zero-copy implementation must synchronously check writer
distance, transform one contiguous segment, re-read the writer pointer and
discard ambiguous output; it must not expose overwriteable ring RAM through
the asynchronous queue.

## USB-C receiver preview

The optional preview observes conditioned CVBS samples without owning PARLIO
or stopping AV. Its adaptive sync tracker distinguishes 60–160-sample H-sync
pulses from 300–800-sample broad vertical sync, clusters the multiple broad
pulses in one vertical-sync interval into one field marker, requires three
plausible 1100–1450-sample line intervals for horizontal lock, expires stale
lock, follows PAL/NTSC-ish line-period drift, rejects short low glitches and
samples active video relative to the measured line period. `CVBS LOCK STATUS`
and bounded `CVBS LOCK PROBE 1000|5000` commands expose H/V rates, lock
transitions and frame counts.

Frames use the versioned CRC-protected binary protocol documented in
[`usb-preview-protocol.md`](usb-preview-protocol.md). Packet magic, header
length, payload length, header CRC and payload CRC let the Receiver Console
resynchronise after text interleaving, corruption or a partial packet. USB
disconnect/reconnect and preview stop do not own or stop analog AV.

## Physical gates still required

- actual producer cadence for modes 0 and 11 (mode 12 is BLE-specific);
- meaning of status bit `0x600a9004[18]`;
- coherent I/Q and phase continuity across ring wrap;
- pre/post-dump fine-tune phase slope;
- FE tap bandwidth and therefore safe sparse/4:1 reduction;
- real RF-writer + CPU-copy + BitScrambler + PARLIO contention;
- dirty-VTX H/V lock range and conditioner/DAC calibration.

The exact automated order is in
[`first-hardware-test.md`](first-hardware-test.md); the fuller implementation
and failure-state description is in
[`live-stream-architecture.md`](live-stream-architecture.md).
