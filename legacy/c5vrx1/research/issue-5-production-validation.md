# Issue 5 production validation

Software in this branch contains the candidate dataplane, but hardware claims
remain fail-closed. XIAO testing proved the ordinary FreeRTOS ring execution is
not viable while the MAC owns the dump HP-SRAM banks: native USB stops without
a normal command response. Never infer RF sample rate from finite USB transfer
time and never mark AV production-ready from a host-only test.

## Implemented dataplane

`RF ring → phase6/phase8 BitScrambler WBFM → canonical timing epoch →
structure-derived levels → AV mailbox/guardian → PARLIO → fixed 6-bit DAC`

The canonical observer owns polarity, PAL/NTSC choice, field identity/parity,
line phase and discontinuity epoch. AV receives no framebuffer or pixel
conversion. A dedicated priority-20 worker keeps legal PAL/NTSC black/sync on
PARLIO whenever real composite is not locked or misses its deadline. The old
sample-domain `C5` overlay is absent from production LIVE.

USB is a priority-4 side reader. Its buffers and task are created before LIVE;
without an explicit START plus a draining-session lease it performs no Y/C,
burst, YUV or packet work. The packed 160×120 YUV411 stream uses the canonical
timeline, drops stale side blocks, and parks on lease expiry or transport stall.

## Mandatory device sequence

1. Run `WBFM HWTEST`, `BENCH BITSCRAMBLER`, `BENCH PIPELINE` and
   `BENCH PARLIO`. Keep phase8 only if it passes exact hardware output checks,
   the physical burst-quality test and the selected block retains at least 2×
   RF service-deadline headroom.
2. Do not run the continuous ring matrix on the XIAO. Replace the exclusive
   HP-SRAM ownership design before any production `LIVE START` validation.
3. Use `PIPELINE STATS` during every run. Required healthy values are zero RF
   overrun, zero ambiguity, zero AV mailbox drop after lock, queue occupancy
   near empty, and no unexplained epoch change.
4. Start LIVE with USB stopped. Verify `usb_consumer_active=0` and
   `usb_worker_active=0`; compare CPU/deadline telemetry with USB active.
5. Start the Receiver Console preview. It sends quiet leases only until the
   first valid frame; successful binary writes then maintain the lease. Stop
   reading, unplug and reconnect repeatedly. AV/timing epoch may not change.
6. Only after the archived PAL and NTSC scope report passes the burst checks
   below, send `APPLY PHYSICAL BURST GATE CONFIRMED`. This volatile gate is
   cleared by reboot, RF-capability invalidation, or a failed configured-kernel
   self-test. Until it is recorded, production `LIVE START` remains disabled;
   use bounded Phase8 captures for measurement work.

## Scope and 75-ohm gates

Use exactly one 75 Ω termination. First test synthetic PAL and NTSC black/sync,
then real RF composite. Record actual volts and timing, not visual estimates:

- sync tip, blanking, black and white levels;
- H period/jitter, field period, equalising/serration structure and half-line
  parity for PAL and NTSC;
- burst frequency, amplitude and phase; active chroma amplitude and phase;
- luma bandwidth, clipping, hue/saturation stability, dot crawl/cross-colour;
- RF-to-DAC sample age and total added latency (target below 1 ms);
- output under VTX removal, RF reacquisition and USB hammer/stall;
- goggles, monitor and DVR lock plus a known colour-bars/scene comparison with
  a known-good RX5808 receiver;
- at least 60 minutes continuous soak.

Benchmark each achievable PARLIO clock candidate. Feed the measured
samples/line and standard line rate into the clock-choice telemetry. Production
remains fixed at the native 20 MS/s CVBS source rate. Prefer bypass when elastic
occupancy remains bounded; otherwise validate and wire the four-tap streaming
Farrow bridge against burst amplitude/phase before enabling an alternate
clock. A mathematically attractive bridge is not a winner without scope
evidence.

## Release rule

Do not close Issue 5, label the AV path production-ready, claim colour lock,
claim sub-millisecond latency, claim superiority to RX5808, or merge a release
configuration until the archived hardware report contains every measurement
above. Optional audio remains deliberately outside the AV deadline path and is
not a release blocker.
