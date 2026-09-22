# Prepared live RF streaming architecture

> Investigation update: the source-level non-blocking producer and guarded
> ring source are reconstructed and implemented. Physical continuous/wrap and
> phase-bearing behavior is not yet proven, so the source remains explicitly
> `EXPERIMENTAL_RING_SOURCE_UNPROVEN`.

## Status

**IMPLEMENTED / NOT PHYSICALLY TESTED**

- `c5vrx_rf_source_t` block ownership/acquire/release interface;
- bounded four-block producer/consumer queue and high-water accounting;
- separate source and processing tasks with drop/backpressure counters;
- persistent sample-domain route through the existing 4:1 C5 BitScrambler WBFM
  transform (no CPU `atan2` in the live path);
- configurable DC tracking, bias, polarity, Q8 gain, optional one-pole filter,
  and output clamp;
- two-buffer 20 MS/s PARLIO/GDMA live sample sink;
- finite vendor dump adapter and `NEARLIVE START` diagnostic;
- source/output underruns, dropped blocks, discontinuities, stage timing,
  achieved rate, WBFM/CVBS ranges, occupancy and high-water diagnostics.

**UNKNOWN UNTIL MEASURED ON HARDWARE**

- whether finite chained dumps contain useful analog-FPV I/Q;
- actual sample rate and gaps;
- RF-to-CVBS polarity, bias, gain, filtering and voltage calibration;
- sustained PARLIO timing and passive DAC levels into the real 75-ohm input.

**CONTINUOUS RF PRODUCER UNKNOWN**

An `EXPERIMENTAL_RING_SOURCE_UNPROVEN` module is now implemented. It arms the
non-blocking producer once, keeps a guarded reader pointer, exposes only
contiguous segments, immediately copies into owned DMA buffers to uphold the
immutable-block contract, and counts wraps, overruns, missed words, drops and
discontinuities. It cannot be relabeled `CONTINUOUS` until cadence, wrap and
coherent phase-boundary probes pass on hardware.

Promotion to `kind = C5VRX_RF_SOURCE_CONTINUOUS` is the remaining gate. The
experimental ring source already implements `acquire`/`release`; hardware must
show that its blocks are phase-bearing and gap-free enough for that stronger
classification. All downstream buffering, hardware WBFM, conditioning, output
and diagnostics remain unchanged.

Key unresolved question:

> Can the ESP32-C5 provide a sufficiently continuous, phase-bearing RF stream
> for real-time WBFM demodulation?

`RF DEEP PROBE` is now the first-board gate. The optional non-blocking producer
is hash-pinned, disabled by default and limited to vendor-derived modes 0/11.
Vendor mode 12 is deliberately rejected because its complete branch starts
BLE RX.
It is diagnostic infrastructure, not a promoted continuous source.

`NEARLIVE START` repeatedly uses the recovered finite vendor dump. Every block
is flagged discontinuous and logs identify the mode as
`FINITE_CHAINED_NOT_CONTINUOUS`. It is an experiment, never evidence of a true
continuous producer.

The route deliberately has no full-frame buffer:

```text
RF source -> 4-block queue -> persistent BitScrambler 4:1 -> conditioner
          -> fixed-size sample chunker -> two PARLIO blocks -> 6-bit resistor DAC
```

Contiguous ring segments may be shorter at wrap. The chunker carries only one
PARLIO block of samples and joins those variable segments without creating a
framebuffer or inserting a timing gap. One CPU phase-LUT calculation repairs
the BitScrambler program's first/priming delta at each continuous block
boundary; an actual source discontinuity deliberately resets it to bias.

Production `LIVE START` is bound to that route, but remains fail-closed until
the current boot has an unambiguous mode-0 cadence, coherent wrap validation,
30-second staged soak, externally measured/confirmed alias-safe bandwidth,
BitScrambler self-test, PARLIO benchmark and at least 20% synthetic pipeline
margin. A retune invalidates the RF-dependent gates. The current program has no
IQ anti-alias filter, so only a measured FE-bandlimited /4 input may pass.

Any ambiguous service interval, reader/guard collision, copy-time overwrite,
unexpected producer stop or PARLIO error stops the streaming tasks fail-closed.
RF/output ownership remains locked until `LIVE STOP` performs teardown and
prints the exact ring failure reason; probes cannot race that state.

Calibration defaults are placeholders. Change the Kconfig Q8 bias/gain,
polarity, filter shift and clamps only from measurements of a controlled VTX.
