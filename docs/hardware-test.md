# Hardware proof status and remaining tests

## Wiring

| XIAO | GPIO | Series resistor to VIDEO |
|---|---:|---:|
| D4 | 23 | 8.2 kOhm |
| D5 | 24 | 3.9 kOhm |
| D6 | 11 | 2.0 kOhm |
| D7 | 12 | 1.0 kOhm |
| D8 | 8 | 470 Ohm |
| D9 | 9 | 240 Ohm |

Join the six resistors at VIDEO, fit 200 Ohm VIDEO-to-GND, share ground and use
the normal 75 Ohm receiver termination. Expected loaded levels are roughly
code 0 = 0 V, code 18 = 0.30 V and code 62 = 1.0 V.

## 1. Static AV levels

Build `C5VRX2_MODE_AV_STATIC` six times with codes 0, 18, 31, 32, 62 and 63.
Measure VIDEO into 75 Ohm. This isolates GPIO mapping and analog levels from RF.

## 2. PARLIO continuity and PAL diagnostics

Run `C5VRX2_MODE_AV_PAL_MONO`, then `C5VRX2_MODE_AV_PAL_COLOR`. Scope the
buffer-switch boundaries and verify no idle, duplicated, missing or stretched
DAC sample. Confirm stable sync and burst at the actual 20 MHz diagnostic rate.

## 3. Autonomous pre-trigger producer (passed)

The vendor `adctrig()` call was used to recover the C5 TX_START selector:

```c
adctrig(16383, 5, 0, 1, 1, 0, 0, 0, 0);
```

On C5 v6.0.1 the wrapper ignores `dump_trig`, so this call alone does not prove
dump-first. The separate direct bit-17 test passed 10,000 observed wraps with
one producer start and zero triggers/rearms. No normal wrap wrote `DUMP_CTRL`.

## 4. Live IQ mapping (passed)

`C5VRX2_MODE_MODEM_CAPTURE` passed with VTX OFF and ON. It proved
`DIAG[6:9] = Q[6:9]` and `DIAG[16:19] = I[6:9]`, with no swap, reversal or
inversion. The VTX-ON run matched 94.75% of individual bits and 718/900 full
bytes against the post-stop Q10/I10 ring. The VTX-OFF run matched 95.50% and
822/900 bytes. Both measured about 79.994 MS/s, one producer start, 19 wraps
and zero triggers. Use `tools/analyze_modem_capture.py` to reproduce this.

## 5. Long-duration 80-to-40 PARLIO acquisition

The bounded probe captured a bit-perfect sequence of every second MODEM sample
at about 40 MS/s. Soak the same mapping for minutes and compare periodic
snapshots against the dump timeline. Verify there is no occasional duplicate,
skip, or sampling-phase slip. An actual MODEM-derived 80-MHz receive clock is
an optional future upgrade, not a claim made by the current firmware.

## 6. Physical IQ-wrap continuity

Feed a coherent RF tone through the proven simultaneous live source and compare
adjacent phase across many physical `16383 -> 0` wraps. The old
`C5VRX2_MODE_RF_WRAP` CPU reader sees the stale MAC-owned SRAM view and therefore
cannot supply this proof; its output must not be accepted as gapless evidence.

## 7. Live recovered CVBS (functional picture passed)

Do not use the stale MAC-owned SRAM view for this test. The default LIVE mode
uses MODEM_DIAG Q4/I4 -> infinite PARLIO RX -> compact two-sample TX-BS WBFM
-> continuous PARLIO TX. On 2026-09-09 commit `69f52c3` produced a stable lock
and clearly recognizable NTSC camera video through the resistor DAC. Visible
static remains; receiver sync lock and picture recovery are therefore proven,
but blanking levels, burst/chroma accuracy and production signal quality are
not yet proven on measurement equipment.

Repeat VTX OFF -> ON -> OFF while capturing the raw Q4 and DAC waveform. Verify
the `n -> n+2` phase interval remains unambiguous, quantify the static source,
and separately prove that both cyclic DMA boundaries introduce no missing,
duplicated or stretched DAC sample.
