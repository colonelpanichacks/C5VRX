# Issue #5 stage 1: continuous RF ring proof

This stage deliberately stops before selecting a WBFM kernel, timing core,
output clock or color implementation. Those choices require the continuous RF
source to pass this hardware gate first.

## What is now measurable

The fixed writer pointer is extended into monotonic `producer_absolute` and
`consumer_absolute` positions. A pointer interval is accepted only when the
conservative maximum producer rate proves that fewer than 16,384 words could
have elapsed. An interval that could contain one or more hidden wraps is a hard
failure; modulo arithmetic is never used to disguise it.

The reserved RF area now includes two 64-byte canary guards:

```text
0x4082ffc0..0x4082ffff  pre-ring canary
0x40830000..0x4083ffff  proven 16,384-word RF window
0x40840000..0x4084003f  post-ring canary
```

Every producer run seeds and checks both guards. Any adjacent write fails the
run and production LIVE remains disabled.

The live source configures the producer while allocating buffers but arms it
only inside the first source acquire. No allocation, PARLIO setup or task setup
can therefore happen while an unobserved producer is already overwriting the
ring.

## Required hardware sequence

Run with the production XIAO receiver-console profile, real 5 GHz RX active and
the intended VTX/source configuration:

```text
PRODUCER CADENCE PROBE 0
PHASE CONTINUITY PROBE 0
PRODUCER SOAK 0 30000
BENCH RING PIPELINE 0 1000 1024
BENCH RING PIPELINE 0 1000 2048
BENCH RING PIPELINE 0 1000 4096
CAPABILITIES
```

The 30-second soak is a tight writer-position proof. On the single-core C5 it
temporarily unsubscribes only the idle task from the task watchdog, then
restores it after stopping the producer. It never sleeps across an unknown
number of ring wraps. Any interrupt/scheduling gap large enough to make the
writer advance ambiguous fails the proof.

The block matrix is complete only after 1K, 2K and 4K have all run. A candidate
passes only with:

- at least one observed wrap;
- no RF overrun, fatal stop, dropped RF block or AV underrun;
- at least 90% of the measured producer rate;
- at least 2.000x measured service-deadline headroom.

After all three runs, firmware selects the smallest passing block to minimize
latency. If no candidate passes, `selected_ring_block_words=0` and LIVE stays
fail-closed.

## Production LIVE radio guard

Immediately before arming production LIVE, firmware reads back and requires:

- `WIFI_BAND_MODE_5G_ONLY`;
- Wi-Fi power save disabled;
- STA mode, disconnected, with promiscuous RX active;
- Bluetooth not compiled into the firmware;
- IEEE 802.15.4 not compiled into the firmware.

Failure is reported as `DEDICATED_5G_GUARD_FAILED`; experimental diagnostic
commands do not silently certify this production gate.

## Hardware evidence still required

Code and host tests cannot turn this stage green. The ordered XIAO/6-bit board
must produce a preserved log showing the 30-second ring proof and the complete
block matrix. Until then, the new capability fields remain zero and `LIVE
START` correctly fails closed.
