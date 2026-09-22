# FE dump investigation notes

## Why this path matters

The most promising route to analog FPV on ESP32-C5 is still the internal receive/debug dump machinery. A normal Wi-Fi RX API only exposes packet results; C5VRX needs phase-bearing receive data before packet decode.

## Historical clue from older Espressif PHY artifacts

A public historical ESP PHY symbol map contains the same family of factory/debug names now seen around C5 research:

```text
set_dump_mode
print_dump_data
loop_dump_test
fedump_rd_rxmem
fedump_rd_txmem
adctrig
sampledeal
accumiq
```

In that older artifact, the symbol sizes are especially interesting:

```text
set_dump_mode       0x24
print_dump_data     0x114
loop_dump_test      0x152
fedump_rd_rxmem     0x02
fedump_rd_txmem     0x02
```

This is **not evidence that the ESP32-C5 ABI or implementation is identical**. It is useful architectural evidence only.

The tiny historical `fedump_rd_*mem` bodies strongly suggest they were thin hardware/ROM primitives, while `loop_dump_test` and `print_dump_data` contained the real orchestration and formatting logic. That changes our static-analysis priority:

1. `loop_dump_test`
2. `set_dump_mode`
3. `print_dump_data`
4. callers/callees around `fedump_rd_rxmem`
5. ADC / IQ helper relationships

## What to extract from C5 `librftest.a`

For each function above, record:

- object/archive member that contains it,
- symbol size,
- incoming callers,
- outgoing calls,
- constants loaded immediately before calls,
- MMIO addresses touched,
- loops and buffer strides,
- whether `a0..a7` are consumed before being overwritten,
- whether a returned address is subsequently dereferenced.

`tools/analyze_phy.py` now emits symbol sizes plus a relocation-based caller/callee report to make this easier.

## Strongest success signal

A finite capture is enough for the next proof-of-concept. We do **not** need continuous live video yet.

The first useful result would be a buffer that changes predictably between:

```text
A. no transmitter
B. transmitter carrier / analog VTX on
C. static black image
D. black-white pattern
E. moving image
```

If a buffer changes with C/D/E despite zero valid 802.11 packets, it is probably below packet decode and worth classifying.

## Classification order

Try these interpretations before inventing a custom format:

1. interleaved signed 8-bit I/Q
2. interleaved signed 16-bit I/Q
3. separate I and Q banks
4. complex FFT bins
5. magnitude-only FFT bins
6. channel-estimator / calibration accumulators
7. real post-filter samples

For a candidate complex capture, run it through `tools/wbfm_demod.py`. A recognizable video-baseband spectrum or PAL/NTSC sync structure would be a major proof point.

## Safety rule for undocumented calls

Do not directly call `loop_dump_test`, `set_dump_mode` or `fedump_rd_rxmem` from firmware until their C5 calling conventions are recovered from actual call sites. A wrong prototype can corrupt registers/state while appearing to compile correctly.
