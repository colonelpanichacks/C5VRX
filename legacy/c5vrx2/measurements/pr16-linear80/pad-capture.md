# Six-pad RF-off repeating replay

After the existing oracle/sweep, TX repeats the deterministic 16384-byte IQ
pattern without DSP reset at the DMA wrap. After 3 ms, RX captures 4096
samples from the existing six DAC pads {23,24,11,12,8,9}. No extra clock,
VALID or signal wiring. No RF. RX input enable preserves TX pad outputs.

Two trials request TX40/TX80, both with internally clocked RX40. The latter
is explicitly undersampled: it cannot validate every DAC update. Neither
trial measures resistor-network analog settling or simultaneous RF reception.
Requested clocks alone are not measured rates; disagreement can be capture
timing as well as TX corruption. A single capture need not cross a DMA wrap.

Each L8DP record has 16 little-endian words then 4096 captured bytes:
magic 0x5044384c, version=1, header=64, samples=4096, requested TX/RX Hz,
TX error, RX error, overall error, IRQ before, RX elapsed us, IRQ after,
capture FNV, input FNV, two reserved words. Flash writes happen after cleanup.
Existing L80O data remains intact. Records occupy reserved diagcap offsets
0x12000 and 0x14000; trace stages 0x860/861 report persistence status.

Read 4160 bytes at absolute flash addresses 0x124000 and 0x126000, then:

```
python tools/analyze_linear80_pads.py <capture.bin>
```

The analyzer checks payload/input hashes and compares all captured six-bit
codes to the source-driven steady cyclic reference. It searches cyclic phase,
not arbitrary stretches of skipped samples. Nonmatching signatures are not
discarded or marked passing. LED still reflects the original stock-DMA-EOF
oracle gate, not these separate pad captures. Allow 30 seconds, VTX off.

## Initial physical result (2387411)

Both captures passed payload/input hashes, with TX/RX/setup ESP_OK. RX elapsed
160 us at requested TX40 and 133 us at TX80. IRQ before=0, after=2.
Neither captured six-bit sequence matched any exact 16-sample signature of
the nominal reference (stride 1 and 2 respectively). TX40 contained only 14
unique codes and a visibly repeating 16-sample pattern; TX80 only 8 codes
and a visibly repeating 8-sample pattern. This is a FAILED comparison,
not proof of its cause. Do not infer RF image quality from this test.

Next image adds same-pad direct-byte and Phase5 controls, each TX40/TX80.
Header word 14 now identifies mode: 0 linear80, 1 direct bytes, 2 Phase5.
Six records use offsets 0x12000+trial*0x2000 within diagcap, trials 0..5;
absolute flash addresses 0x124000, 0x126000, 0x128000, 0x12a000, 0x12c000,
0x12e000. Each record is 4160 bytes. Trace stages 0x860..865. Direct mode
does not allocate or enable BitScrambler. Reference analysis uses matching
source programs for each mode and preserves six-bit masking only.

## Control-run interruption (ab5ba15)

User observed a continuously lit LED. Recovered trace reaches 0x860/861
but has no completed direct-byte control (0x862) or later pad trial.
The two new linear80 records validate their hashes and again fail reference
alignment (12/8 distinct six-bit codes, RX elapsed 130/127 us). This is not
a completed successful suite. Exact stopping operation was not instrumented.

Next image runs pad trials in order 2,3,4,5,0,1 before any loopback or EOF
sweep, so direct bytes run before BitScrambler has ever been allocated that
boot. This tests a peripheral-state carryover hypothesis, not a proven fix.
Additional trace markers 0x870+trial mean entry and 0x880+trial mean setup
finished immediately before calling TX transmit. Both occur before TX starts.
The expected source pattern, capture format, pin order and comparison stay
unchanged. Only records with completion markers from this boot are current;
unreached flash slots can still contain earlier captures.

## Direct-first physical result (d77f905)

All six captures completed and the full sweep end marker was present.
Direct TX40 matches all 4096 six-bit pad samples (64 distinct codes);
Phase5 TX40 also matches all 4096 (31 distinct codes). Linear80 TX40
does not align (5 codes). All three TX80 captures fail alignment, including
the undecorated direct control (10 codes), Phase5 (5), linear80 (8).
Thus RX40/pad mapping works for the direct and Phase5 controls at TX40;
the TX80 measurement is not yet validated even without BitScrambler.

IDF 6.0.1 source inspection found two diagnostic defects: TX FIFO-empty ISR
logs and clears the event, invalidating a zero raw snapshot as evidence of
no underrun; interrupted/loop TX disable does not invoke BS disable, although
normal EOF ISR does. Next image masks the FIFO-empty interrupt while retaining
the raw sticky flag, and explicitly disables the decorator-owned BS after
stopping TX, before freeing it. No live sample processing changes. Pad header
word 15 records TX status after RX completion. These changes repair evidence
collection/teardown; they do not yet establish the cause of the bad patterns.

## Sticky-FIFO physical result (667d48f, CPU160)

All six pad trials and the full sweep completed. Direct TX40 and Phase5 TX40
again match all 4096 captured six-bit samples; FIFO-empty bit remains zero
before/after capture (raw IRQ 0/2). Linear80 at TX40 and all three TX80 trials
fail alignment and have FIFO-empty already latched before capture (IRQ 1/3).
Their distinct-code counts are respectively 3,8,9,6 for linear40, linear80,
direct80, Phase5-80. Thus the failed pattern observations coincide with
hardware TX FIFO-empty events; this is not merely a parser mismatch.

Every linear80 finite timing trial also reports FIFO-empty=1, including all
eight DATA_LEN trials which return ESP_OK. Their successful completion must
NOT be treated as passing throughput evidence. Earlier zero snapshots were
misleading because the driver ISR cleared the flag.

Next A/B uses identical source and capture clocks with CPU default 240 MHz
instead of 160 MHz. This does not by itself establish faster peripheral
clocks. Build uses sdkconfig.cpu240.defaults last and separate generated
SDKCONFIG=build-linear80-oracle/sdkconfig.cpu240. The existing build directory
is reused; its resulting binaries now belong to CPU240, not the earlier
CPU160 build. No per-sample CPU DSP or live firmware changes are introduced.

SDK clock check: IDF v6.0.1 `esp_hw_support/port/esp32c5/rtc_clk.c`,
`rtc_clk_cpu_freq_to_pll_160_mhz` and `_240_mhz`, both select AHB=40 MHz
(root dividers 4 and 6). Their source comments constrain AHB to <=48 MHz.
Therefore the CPU240 A/B does not automatically raise AHB bandwidth. No bus
overclock is applied by this experiment. This is configuration evidence,
not a measured hardware throughput ceiling.

## CPU240 physical A/B result

The saved L80O header confirms configured CPU=240000000, written=32768,
loopback mismatches=0. Every pad trial and final sweep marker completed.
Direct TX40 and Phase5 TX40 each match all 4096 six-bit samples without
FIFO-empty (raw IRQ 0/2). Linear80 TX40 and all TX80 variants fail reference
alignment with FIFO-empty latched (raw IRQ 1/3), exactly the CPU160 pass/fail
pattern. Their code counts are 5/9/11/5 for linear40/linear80/direct80/Phase5-80.
CPU240 does not fix the demonstrated transmission fault. Finite counted-EOF
transactions again finish with FIFO-empty, so completion remains insufficient.

Raw files: cpu240-trace.bin, cpu240-header.bin, cpu240-0.bin through -5.bin.
The tested firmware remains RF-off; no live 80 MS/s or analog improvement
has been demonstrated. Next investigation must target peripheral delivery
and instruction/output scheduling, using the byte-exact TX40 controls to
avoid treating FIFO replay as valid video. No additional flash was performed
when recovering this A/B result; C5 remains in download mode.

## Internal-SRAM INCR16 delivery candidate

Next pad test reconfigures the TX GDMA channel before enable to use 64-byte
bursts with access_ext_mem=false. IDF v6.0.1 C5 ahb_dma_ll.h explicitly maps
64 bytes to INCR16; the standard PARLIO driver requests PSRAM-capable DMA,
which limits its default bursts to 32 bytes. Raw input is now 64-byte aligned
internal DMA SRAM; the handle's alignment constraints are refreshed after
configuration. Descriptor allocation/max input length remain unchanged.
No priorities, bus clocks, pin order or video rates change. CPU remains 240
for a direct comparison against the preceding CPU240 run. This is a candidate
for reducing delivery overhead, not a guaranteed throughput fix. The finite
EOF sweep remains at its previous DMA setting; only pad replay is tuned.

Signal interpretation: 40 MS/s means 25 ns between available updates;
duplicating each 20-MS/s value holds it for 50 ns. Neither is by itself an
empty FIFO or proof of layer displacement. Sub-13-ns value changes require
more than 76.9 million real updates/s; interpolation cannot change that
arithmetic at a fixed 40-MS/s output clock. RF-off pad tests do not establish
the cause of the user's RF-dependent sawteeth/layers.

## INCR16 physical result (4cdd026)

All six captures and sweep completed. Increasing pad TX bursts to 64 bytes
did not change the pass/fail pattern at CPU240: direct TX40 and Phase5 TX40
match all 4096 captured codes with no FIFO-empty; linear80 TX40 and every
TX80 control fail alignment with FIFO-empty (raw IRQ before/after 1/3).
Raw captures are burst64-0.bin through burst64-5.bin and burst64-trace.bin.
The burst change has no demonstrated benefit and must not be promoted to
the live path as a fix. CPU/burst tuning has not validated the 80-MS/s path.

## Intermediate-rate test

Restore default DMA32, retain CPU240 and sticky fault recording. Pad rates
are now 40/60 MHz for linear80, direct bytes and Phase5 (slots 0..5), with
an additional direct-byte 48 MHz trial in slot 6 at flash 0x130000 (4160 bytes).
Order is 2,6,3,4,5,0,1; slot layout otherwise unchanged. Header word 4 records
the driver's selected output clock. PLL_F240M integer divisors 6,5,4 give
40,48,60 MHz; an available divider does not establish reliable throughput.
The six-bit direct input is already precomputed; no per-sample processing
occurs during replay. This is not a precomputed live CVBS implementation.

Host comparison models rational clock ratios 6/5 and 3/2 at RX40, searching
only possible fractional sampling phases plus cyclic source phase. All 4096
samples are compared after alignment. Tests cover both fractional phases and
corruption. Faster-than-RX output remains incompletely observed. No analog
or RF causation claim follows from this test. The finite EOF sweep remains
at its prior 40/80 MHz settings and should not be confused with pad rates.

## Intermediate-rate physical result (8156a1b)

All seven pad trials and final sweep completed. Driver-selected rates match
40/48/60 MHz requests. Direct TX40 again matches all 4096 samples with raw
IRQ 0/2. Direct TX48 and TX60 fail reference alignment with FIFO-empty
(IRQ 1/3), 14 and 11 distinct codes respectively. Phase5 TX60 and linear80
TX40/TX60 also fail with FIFO-empty (5,10,12 distinct codes respectively).

Important control regression: Phase5 TX40 fails the signature comparison in
this run despite no FIFO-empty (IRQ 0/2), with 45 distinct codes. Prior runs
matched exactly; do not omit this inconsistency or attribute it to a proven
speed ceiling. Sampling phase, test ordering/state and output integrity need
separate validation. No faster-than-40 MHz reliable path is demonstrated by
these measurements, but they do not prove that every possible implementation
on the chip is limited to 40 MHz. Files: intermediate-trace.bin and
intermediate-0.bin through intermediate-6.bin. No new firmware flashed during
result recovery; device remains in download mode, RF off.
