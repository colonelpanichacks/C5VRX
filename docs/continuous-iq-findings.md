# ESP32-C5 continuous IQ hardware findings

Hardware under test: ESP32-C5 revision v1.0, A1/5865 MHz receive path.
The observations below were made on 2026-09-05/06 with ESP-IDF v6.0.1.

## Proven autonomous circular writer

The C5 RF dump engine was armed once with:

- `DUMP_PTR_MODE[24:17] = 0x00060000` (TX_START selector)
- `DUMP_CTRL[17] = 1` (dump-first/pre-trigger)
- `DUMP_CTRL[31] = 1` (enable)
- no software START pulse
- MAC TX disabled, so TX_START cannot occur

The measured writer rate was approximately 79.97 MS/s. A producer-only
hardware soak then reported:

```text
observed physical wraps = 10000
observer-estimated wraps = 54240
producer starts = 1
rearms = 0
triggers = 0
DUMP_CTRL = 0x80024000
```

This proves that the 16,384-word/64-KiB region is a long-lived circular RF
writer rather than a mandatory sequence of finite 16K captures. Normal wraps
require no write to `DUMP_CTRL`.

## C5 vendor-wrapper finding

Disassembly of ESP-IDF v6.0.1 `librftest.a:adctrig` showed that C5 trigmode 5
selects `0x00060000`, but the historical `dump_trig` function argument is not
consumed by this implementation. Calls with `dump_trig=0` and `dump_trig=1`
therefore produce the same trigger-first state. Production code preserves the
vendor-derived TX_START selector and sets the historical dump-first bit
explicitly.

## SRAM ownership finding and current blocker

The vendor MAC dump allocation changes `0x60095004` to set bits `0x00010200`.
Both 64-KiB banks from `0x40830000` through `0x4084ffff` must be excluded from
the normal heap; reserving only the first bank caused a CPU lockup when the
ownership handoff overwrote allocations in the second bank.

With the full region reserved, the writer completes the soak without memory
corruption. However, direct HP-CPU reads of the first ring while the MAC owns
the banks returned zero-valued IQ windows. The guard check also appeared
unreadable during MAC ownership and became valid again after ownership was
returned. Feeding the same live ring directly to PARLIO/GDMA produced a FIFO
underrun followed by heap corruption.

Therefore the autonomous writer is proven, but the current SRAM ownership
configuration is not yet a usable simultaneous MAC-write/HP-read mapping.
`continuous_iq_acquire()` must not be considered production-ready until a
shared/arbitrated ownership configuration or another live-readable path is
physically demonstrated.

An AHB-GDMA visibility probe then copied the same 2-KiB physical RF-ring
window twice, 300 us apart, while the autonomous writer remained enabled. At
the measured 79.97 MS/s the writer traversed the 16K ring several times during
the probe. Both DMA operations completed successfully, but the snapshots were
byte-for-byte identical:

```text
VTX off: nonzero A=512, nonzero B=512, changed=0
VTX on:  nonzero A=256, nonzero B=256, changed=0
```

The different static occupancy with and without the VTX shows that the mapped
contents are not simply an untouched destination pattern. The absence of any
within-run changes nevertheless proves that ordinary AHB-GDMA sees the same
stale/non-live HP-domain view as direct CPU reads. Directly DMA-reading the
MAC-owned ring is therefore not the missing simultaneous-access mechanism.

A follow-up two-bank probe filled both candidate windows with distinct known
patterns before arming the same `SRAM_USAGE=2`, `MAC_DUMP_ALLOC=1` producer.
It copied both windows twice through AHB-GDMA while the writer ran, then
stopped the writer, restored CPU ownership and inspected the physical SRAM:

```text
RF rate                      = 79.959 MS/s
producer starts/rearms/trigs = 1 / 0 / 0
live GDMA changes, 0x40830000 = 0 / 256 bytes
live GDMA changes, 0x40840000 = 0 / 256 bytes
post-stop overwritten words, 0x40830000 = 16384 / 16384
post-stop overwritten words, 0x40840000 = 0 / 16384
guards intact                = yes
```

This physically locates the IQ dump at `0x40830000`; applying an additional
64-KiB address offset in the consumer would be wrong. `0x40840000` is not a
live alternate view. Both normal AHB views remain static while MAC ownership
is active, even though bank A contains the completed IQ ring immediately after
ownership is restored.

A second build retained documented HP-CPU ownership (`SRAM_USAGE=0`) while
leaving `MAC_DUMP_ALLOC=1`. The autonomous writer still advanced at about
79.96 MS/s, but neither CPU nor AHB-GDMA obtained a changing live view. Merely
leaving the ownership selector at zero is therefore not a shared-access mode.

Finally, the official ESP32-C5 HP_APM M1 exception registers were cleared and
sampled before and after four completed AHB-GDMA reads from both candidate
banks. The result was identical at every checkpoint:

```text
HP_APM_M1_STATUS          = 0x00000000
HP_APM_M1_EXCEPTION_INFO0 = 0x00000000
HP_APM_M1_EXCEPTION_INFO1 = 0x00000000
```

During that same run the RF writer measured 79.9705 MS/s and completed 28
physical wraps from one start with zero triggers. All DMA copies returned
success, their live snapshots did not change, and after stopping RF all 16,384
words of bank A were found overwritten while bank B remained untouched. Thus
HP_APM permission or bounds checking is not blocking the reads. The active
MAC dump bank exposes a stale/non-live normal AHB view by hardware design; an
APM permission change cannot make that view live.

## Claims deliberately not made yet

- No claim of sample-gapless RF time across `16383 -> 0` is made yet.
- The coherent-tone adjacent-phase test has not passed because HP could not
  observe the IQ words while the current MAC ownership was active.
- Live PARLIO output is not proven with the RF writer active.

The next bounded experiment is the C5 modem diagnostic path. Vendor
`coex_hw_debug_matrix_config()` writes `0x600a9408` and routes modem diagnostic
signals 106 through 109, while `bt_bb_ble_diag_all()` programs
`0x600a9404` and related selector fields. A diagnostic must first use only
vendor-observed selector states and determine whether a continuously changing,
RF-dependent IQ representation exists before attempting a production consumer.
Only after simultaneous live IQ access works should coherent-tone phase
continuity and the live WBFM-to-CVBS pipeline be tested.

The first GPIO-matrix loopback probe sampled all 32 `MODEM_DIAG` outputs in
six-line batches under three bounded states: reset/default widgets,
`DIAG_EXCHANGE=2` as written by C5 coexist code, and the final low-ten-bit
`0x14e` pattern produced by C5 `bt_bb_ble_diag_all()`. With the VTX off,
signals 0 through 19 toggled continuously while signals 20 through 31 were
static. With the VTX on, all 32 signals toggled. The strongest VTX-dependent
transition-count changes occurred on signals 6 through 9 and 16 through 19;
several increased by hundreds of percent. Both runs retained the autonomous
79.97-MS/s producer with one start and zero triggers/rearms.

This proves that the GPIO-exposed modem diagnostic bus is a simultaneous live,
RF-dependent observation path that does not depend on the inaccessible SRAM
read view. The XIAO has exactly eight free candidate data pads while keeping
the six-bit DAC and native USB intact.

## Proven XIAO 4+4-bit live-IQ mapping

The bounded `MODEM_CAPTURE` diagnostic routed the strongest eight signals to
the remaining XIAO pads, captured raw `GPIO_IN`, explicitly stopped the single
RF producer, and then persisted both the GPIO trace and completed Q10/I10 ring.
The active MAC-ownership window had to run from IRAM with interrupts masked;
otherwise a flash-backed interrupt handler caused `ESP_RST_WDT`. SRAM ownership
and interrupts are restored before any flash or USB operation.

VTX-OFF and VTX-ON captures independently converged on the same timing and
mapping:

```text
DIAG[6:9]   = dump Q[6:9]
DIAG[16:19] = dump I[6:9]
RF/GPIO timing ratio = 17.778
fixed alignment      = 148 RF samples before capture-end pointer

VTX off: RF 79.9935 MS/s, 95.50% bit match, 822/900 exact bytes
VTX on:  RF 79.9945 MS/s, 94.75% bit match, 718/900 exact bytes
```

The VTX-ON ring used the full signed range and all 16 values in both captured
nibbles, so the result proves individual bit order as well as component/sign
activity. No nibble swap, bit reversal or inversion improved the match. The
remaining mismatch is consistent with asynchronous 4.50-MS/s CPU GPIO reads
of an approximately 80-MS/s bus; it is why production must use the modem's
source-synchronous clock rather than CPU polling.

`tools/analyze_modem_capture.py` reproduces the circular-offset and timing-ratio
correlation from a saved `diagcap` partition image.

## Bounded PARLIO capture and first live path

Routing the proven Q4/I4 lanes into PARLIO RX produced a bit-perfect bounded
capture of every second native MODEM sample. The RF/dump cadence remained about
79.99 MS/s while PARLIO RX topped out at about 40 MS/s even when F80 or F160
was requested. The current LIVE path therefore treats 40 MS/s as its acquired
complex-IQ rate. The original adjacent-FM plus real-domain 2:1 program was
correct but exceeded the BitScrambler instruction-rate budget; the bounded
two-bundle replacement and its remaining caveat are documented below.

The receive transaction is genuinely cyclic: ESP-IDF 6.0.1 maps
`partial_rx_en=true` to an infinite transaction and links the final RX-GDMA
node back to the head. The output transaction independently uses PARLIO's
hardware `loop_transmission` mode. A short sequence match does not prove that
80-to-40 sampling remains phase-locked indefinitely, nor that either DMA ring
boundary is sample-perfect; those remain explicit hardware tests.

## Direct TX-BitScrambler WBFM proof

The RX-attached BitScrambler could not sustain the required input cadence, so
the realtime topology now stores raw Q4/I4 with PARLIO RX and decorates the
PARLIO TX transaction instead. A bounded full-duplex hardware test proved that
the direct TX decorator consumes two input bytes per 20-MHz output byte without
FIFO underrun while PARLIO RX captures the output concurrently.

The final realtime core contains two instruction bundles. It retains the
second Q4/I4 byte from each input pair as a compact Q3/I2 state and directly
addresses a 32-by-32 discriminator/output LUT with the preceding and current
states. Hardware address oracles established all relevant C5 behavior:

```text
constant embedded LUT[0]                  4000 / 4000 exact outputs
current selected byte (input 3,5,7,...)   4000 / 4000 exact addresses
previous selected byte (input 1,3,5,...)  4000 / 4000 exact addresses
16-bit LUT address bit 0                   O16
current IQ5 address bits                   O16..O20
previous IQ5 address bits                  O21..O25
```

This rejects the earlier high/reversed `O31..O22` address hypothesis for C5
16-bit LUT mode. The production 1024-entry embedded table then ran at the full
requested transport rate and matched the independent CPU reference for 3998
of 3999 aligned observable low-nibble outputs. The sole mismatch was the
bounded transaction's trailing pipeline boundary; setup, RX and TX all
returned `ESP_OK`.

The live DMA topology is consequently:

```text
MODEM_DIAG Q4/I4
 -> PARLIO RX 40 MHz
 -> 4 x 4096-byte raw cyclic ring
 -> one-block producer/consumer separation
 -> TX BitScrambler (two bundles, persistent history)
 -> PARLIO TX 20 MHz
 -> six-bit resistor DAC
```

RX writes 40 MB/s and TX-BS consumes the same raw 40 MB/s. Their clocks are
integer divisions of the same `PLL_F240M` root (`/6` and `/12`), so software
does not pace or copy the stream. The BitScrambler is started once as part of
one hardware-looping TX transaction; normal ring wraps do not reset its prior
IQ state. Normal live telemetry reads only control registers: it does not
copy, scan, persist or print samples from the DMA ring. USB/debug is therefore
outside both transport pacing and the hot SRAM data path.

The compact core evaluates circular phase change between consecutive retained
20-MS/s states, equivalent to an `n -> n+2` interval in the acquired 40-MS/s
Q4 stream. It is not yet proven that this wider interval never becomes
ambiguous for the actual VTX deviation. A raw-capture range test and a long AV
A/B-boundary continuity test remain required. The embedded LUT currently uses
pedestal 20, current-minus-previous polarity and the default 1.5x
post-difference gain; runtime calibration changes require regenerating or
selecting another embedded LUT.

## First end-to-end live NTSC lock

On 2026-09-09 the merged LIVE image from commit `69f52c3` was flashed to a
physical XIAO ESP32-C5 and tested with an NTSC FPV camera/VTX. The goggles
locked immediately and displayed a clearly recognizable moving camera image.
The image still contained visible static. This physically proves the complete
functional path:

```text
live MODEM_DIAG Q4/I4
 -> cyclic PARLIO RX at 40 MS/s
 -> raw 16-KiB ring
 -> direct two-bundle TX BitScrambler discriminator
 -> cyclic PARLIO TX at 20 MS/s
 -> six-bit resistor DAC
 -> locked NTSC receiver picture
```

It does not yet prove sample-perfect operation at every long-duration RX/TX
ring boundary, RF-time continuity across the private dump-SRAM wrap, or that
the compact `n -> n+2` discriminator never aliases at the VTX's full deviation.
Those remain separate scope/capture tests. The observed static is likewise a
remaining signal-quality problem, not a transport-startup failure.
