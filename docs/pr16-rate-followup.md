# PR16 rate and reconstruction follow-up

Baseline: `546672e`, 2026-09-13. User reports smaller but remaining sawteeth,
misaligned layers, gray/color static, worsening with VTX distance. Treat that
as the current hardware observation; earlier claims of complete resolution
in the PR body and diagnostic notes are superseded.

## Confirmed corrections

* Baseline emits `[D,D]` at a configured 40 MHz. This is still a 50 ns
  zero-order hold. It does not move the underlying 20 MS/s images to 40 MHz.
* C5 has **2048 bytes** of LUT: 2048x8, 1024x16, or **512x32**.
  An 11-bit programming register does not establish 2048x32 capacity.
  The proposed 32-bit table at addresses 0..1279 cannot fit.
* A lookup addressed only by previous/current IQ phase knows one FM output.
  Interpolation between consecutive FM outputs additionally needs the previous
  DAC value (or the preceding phase interval). Expanding the LUT word alone
  cannot supply that missing history.
* The old host check still modeled the reverted five-bundle program. It now
  executes the checked-in `set`, LUT, read/write and branch dataflow instead.
* The 80 MHz oracle transmits an alternating pattern without BitScrambler.
  API completion and elapsed wall time alone do not validate decorated throughput,
  every output sample, or resistor-ladder settling. Its 80 MHz RX request also
  does not override the historically measured 40 MS/s RX limit.
* NTSC's fractional line/sample ratio predicts a quantization pattern, but is
  not proof of the observed layer displacement. Shifting burst and active
  chroma together does not by itself imply a hue error relative to burst.
  Distance-dependent worsening also warrants RF-noise/burst-SNR measurements.
* Bounded frozen-IQ replay is not simultaneous live RX/TX evidence. The first
  687-sample discrepancy in old notes remains unexplained without an independent
  startup/boundary trace. DAC codes are not screen pixels or measured voltages.

References: [IDF v6.0.1 BitScrambler assembly and LUT capacity](https://docs.espressif.com/projects/esp-idf/en/v6.0.1/esp32c5/api-reference/peripherals/bitscrambler.html),
local `main/tx_80m_oracle.c`, `main/c5vrx2_wbfm_q4_phase5_2to1.bsasm`, and
`legacy/c5vrx1/research/dsp-pipeline.md` (single 2 KiB LUT constraint).

## Implementation direction

Keep the full Phase5 centroid LUT, raw 40 MB/s ring transport and fixed
gain=2/pedestal=20. Calculate interpolation inside BitScrambler with no CPU DSP.
Validate actual assembly arithmetic and then decorated FIFO/throughput on C5;
do not infer the bundle budget from the configured output rate alone.

## 80 MS/s candidate

`c5vrx2_phase5_linear80.bsasm` retains the original 1024x16 LUT and produces
`[A, floor((3A+B)/4), floor((A+B)/2), floor((A+3B)/4)]`. Counter A contains
two independent byte sums; their maxima are 252, so no inter-byte carry can
corrupt the weighted values. Counter B retains the phase across output and
ring boundaries. There is one prime plus six repeating bundles, not four.
No CPU processing, extra realtime SRAM ring, RX rearm, or pin-order change.

`validate_linear80.py` executes actual assembly and verifies all 4096 DAC
endpoint pairs plus 131080 outputs across four original 16 KiB boundaries.
All 64 DAC levels occur; all 34 original Phase5 target levels remain unchanged.
Flooring interpolation has less than one DAC code error, no overshoot, and
preserves constant levels. The causal reconstruction adds one 20 MS/s interval
of delay and attenuates some high-frequency content; it cannot recover lost RF
information or guarantee gray/color-static elimination.

The IDF v6.0.1 assembler accepts the program and the live candidate builds.
**Hardware throughput is not yet proven.** Enable with
`sdkconfig.linear80.defaults` only after the dedicated oracle passes. The oracle
uses full-byte I2S-triggered BitScrambler loopback, then decorated PARLIO timing
at 40/80 MHz with two input lengths and mid-transmission FIFO snapshots. RF
stays off. It stores all output/reference bytes and verifies flash readback.
Even a passing oracle does not establish loaded analog settling at 12.5 ns.

Synthetic coherent 3.5791015625 MHz, pedestal 20/amplitude 8-code test:
first image changes from -12.65 dBc (held 20 MS/s) to -25.22 dBc (linear 80 MS/s),
12.57 dB additional suppression. Fundamental changes by -0.433 dB, mean by
-0.223 code due to flooring. Reproduce with `tools/report_linear80_spectrum.py`.
This demonstrates the intended interpolation tradeoff, not actual RF quality.

An optional `sdkconfig.cpu240.defaults` supports a 160/240 MHz CPU A/B. There
is no per-sample CPU work in either profile. A higher CPU setting must not be
reported as a higher BitScrambler clock without measurement. CI builds live
and oracle variants at both settings. Keep the VTX off for these oracle runs.
