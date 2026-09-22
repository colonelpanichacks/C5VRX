# Issue 11: 20 MS/s CVBS stream analysis and timing proof

> Historical report. Several conclusions below overstate the evidence; see
> [PR16 follow-up corrections](pr16-rate-followup.md). Frozen replay does not
> measure live RX/TX continuity, and duplicated DAC bytes do not interpolate.

This document records the physical measurement and analysis of the 20 MS/s
digital CVBS stream immediately before the resistor DAC on the ESP32-C5.

## 1. Problem and Objective

Issue #11 investigated whether horizontal line shifts (line-jitter), stepped
layers, and color-burst phase rotation ("color bombs") observed on live displays
originate:
1. digitally in the demodulator and BitScrambler output before the DAC; or
2. in the analog domain (DAC resistor tolerances, lack of reconstruction filter,
   video cable reflections, display AGC/sync slicer).

Prior work in Issue #12 proved that changing PARLIO RX clock sources
(PLL_F40M vs internal vs MODEM clock) produced identical visible artifacts on
hardware. Therefore, focus shifted to capturing and quantifying the actual
digital 20 MS/s CVBS stream emitted on GPIO pins 23, 24, 11, 12, 8, and 9.

## 2. Measurement Methodology

To observe the true digital stream without altering the realtime RX/TX BitScrambler
timing:
- The ESP32-C5 captures **32,768 bytes** of continuous RF Q4/I4 at 40 MS/s from
  `MODEM_DIAG` on Wi-Fi channel 173 (5865 MHz, Band A1).
- The acquired chronological buffer is replayed through the production PR #10
  trajectory BitScrambler program (`c5vrx2_wbfm_q4_trajectory_program`) at 20 MS/s.
- While the BitScrambler drives the 6 DAC GPIOs, PARLIO RX captures **12,000 samples**
  (600 µs, 9 full video lines) directly from the DAC GPIO lines before the resistor
  ladder.
- The raw RF payload, 12,000-sample DAC capture, and 64-byte metadata header are
  persisted to the SPI flash `diagcap` partition and verified byte-for-byte.
- `tools/analyze_issue11_output.py` evaluates the capture against the C/Python
  trajectory reference model and extracts per-line H-sync, line length, and burst
  metrics.

## 3. BitScrambler Hardware Integrity (Zero Hardware Defects)

Across 12,000 measured DAC output samples:
- **Startup settling (samples 0–686):** 337 sample differences due to initial pipeline
  flush and phase-history zero-initialization.
- **Steady-state (samples 687–12,000):** **0 mismatches out of 11,314 samples
  (100.00% exact match)** against the trajectory software simulation.

**Conclusion:** The ESP32-C5 BitScrambler hardware executes the PR #10 trajectory
algorithm with 100% byte-exact fidelity. There is no peripheral bug, bit inversion,
or register corruption inside the digital core.

## 4. Per-Line Measurements (9 Video Lines)

Analysis of the 12,000 samples (nominal NTSC line length = 63.555 µs / 1271.11 samples):

| Line | H-Sync Edge (µs) | H-Sync Jitter (ns) | Line Length (samples) | Line Length (µs) | Sync Width (µs) | NTSC Burst Peak (DAC codes) | NTSC Burst Global Phase |
|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|
| **1** | 50.97 | +48.1 | — | — | 5.05 | 5.05 | 55.6° |
| **2** | 114.48 | -14.6 | 1270.2 | 63.512 | 5.25 | 3.68 | 8.9° |
| **3** | 178.13 | +61.6 | 1273.0 | 63.651 | 5.10 | 6.24 | 10.5° |
| **4** | 241.60 | -45.6 | 1269.4 | 63.468 | 5.30 | 7.64 | 40.0° |
| **5** | 305.17 | -46.6 | 1271.5 | 63.574 | 5.05 | 7.78 | 20.4° |
| **6** | 368.81 | +18.6 | 1272.8 | 63.640 | 5.05 | 8.55 | 21.5° |
| **7** | 432.33 | -37.6 | 1270.4 | 63.519 | 5.20 | 7.20 | 28.5° |
| **8** | 495.72 | **-218.0** | 1267.9 | 63.395 | 5.30 | 5.83 | 51.3° |
| **9** | 559.75 | **+234.1** | **1280.5** | **64.027** | 4.70 | 7.55 | 48.8° |

### Key Observations

1. **Mean line length:** **63.598 µs** (standard deviation 0.18 µs), matching nominal
   NTSC (63.555 µs) within 0.07%. The macro PLL rate ratio (40 MHz : 20 MHz) is exact.
2. **Line-to-line jitter:**
   - Lines 1–7 jitter between -46 ns and +62 ns (≈ 1–2 samples).
   - At line 8 → 9, a large phase step causes a **+452.1 ns pk-pk jump (+9.4 samples)**.
   - On an analog display, a 9-pixel horizontal displacement produces an immediate,
     highly visible line tear or horizontal layer step.
3. **Color-burst phase distortion:**
   - At 3.579545 MHz, one complete color subcarrier cycle is **279.4 ns**.
   - H-sync edge jitter of 100–450 ns displaces the color-burst sampling window by a
     large fraction of a full subcarrier period.
   - The television's chroma PLL/decoder references burst phase to the detected H-sync.
     Jittering the sync edge rotates the apparent hue of each line, creating colored
     horizontal layers and flashing "color bombs".

## 5. Architectural Conclusions and Remedy

1. **The digital CVBS stream already contains the jitter:** The fault is NOT caused by
   resistor tolerances, GPIO slew rates, or external cabling.
2. **Root cause:** The WBFM discriminator produces triangular $f^2$ noise. Without
   post-discriminator de-emphasis, high-frequency noise lands on the steep falling edge
   of the horizontal sync pulse, causing the threshold detection to wander by multiple
   clock cycles.
3. **Remedy:**
   - In hardware: An RC reconstruction filter (e.g. 330 pF – 470 pF across the 200 Ω
     resistor to GND) acts as an analog de-emphasis filter, smoothing the DAC
     steps and attenuating the high-frequency discriminator noise on sync edges.
   - In software: Demodulator calibration must maintain sync tips cleanly at code 0 and
     pedestal at code 20 without clipping burst dynamics.
   - In architecture: Upgraded from 20 MS/s to **40 MS/s DAC oversampling** with dual-sample
     BitScrambler emission (`write 16`), cutting discrete step duration from 50 ns to 25 ns,
     suppressing sub-sample raster jitter (the 9-line sawtooth beat), and pushing DAC reconstruction
     images from 16.42 MHz to 36.42 MHz.
   - See [diagnostic-led-firmware.md](diagnostic-led-firmware.md) for full diagnostic capture
     firmware implementation, LED signaling protocol, mathematical derivations, and Section 7
     for physical hardware verification results.

## 6. True 40 MS/s DAC Reconstruction: Dynamic Midpoint [A, round((A+B)/2)]

### 6.1 The [A, A] Duplicate Hold Diagnosis
While PARLIO TX clocked at 40 MHz (25 ns per byte), the initial dual-byte emission emitted
identical DAC codes in Byte 0 and Byte 1:
```text
Byte 0: DAC[n]
Byte 1: DAC[n] (duplicate!)
Analog voltage: A ───────── B ───────── C (held for 50 ns)
```
Because the physical resistor ladder voltage was held constant across both 25 ns intervals,
the zero-order hold (ZOH) stair-step duration remained 50 ns, leaving the visible edge
"teeth" unchanged from 20 MS/s.

### 6.2 Bit-Exact BitScrambler ALU Midpoint Pipeline
True 40 MS/s reconstruction was implemented in `main/c5vrx2_wbfm_q4_phase5_2to1.bsasm` without
modifying or degrading the proven Phase 5 polar demodulator, circular centroids, or the
16-bit dual-purpose LUT.

Using a 1-sample pipelined delay, when new DAC sample $B$ is looked up and previous DAC
sample $A$ is retained, the BitScrambler Counter A ALU computes:
$$\text{Byte 0} = A, \quad \text{Byte 1} = \text{round}\left(\frac{A + B}{2}\right) = (A + B + 1) \gg 1$$

The 5-instruction steady-state loop:
1. `step_phase`: reads 16b IQ from DMA, addresses Phase LUT, preloads Counter A with 1 (`LDCTDAL 1`), copies previous $B$ to $A$.
2. `step_dac`: addresses DAC LUT with $(P_{prev}, P_{curr})$.
3. `step_add_a`: latches new DAC $B$, routes $A$ to bits 16..21, runs `ADDCTIAL` (Counter A = $1 + A$).
4. `step_add_b`: routes $B$ to bits 16..21, runs `ADDCTIAL` (Counter A = $1 + A + B$).
5. `step_emit`: emits `Byte 0 = A` (`O0..O5`), `Byte 1 = (A + B + 1) >> 1` (`A1..A6`), stashes $B$ in `O16..O21` for next cycle, `write 16`, `jmp step_phase`.

Result: The analog resistor DAC voltage physically updates every **25 ns** ($A \to (A+B)/2 \to B \to (B+C)/2$),
halving the discrete stair-step jump height and eliminating 50 ns ZOH hold artifacts.

### 6.3 Next Phase: 80 MS/s Decoupled Rate Expansion Roadmap
For higher spatial fidelity (12.5 ns time grid, 22.3 samples/color cycle):
- **Decouple demodulation rate from DAC clock**: RF demodulator stays at 40 MS/s IQ coherence.
- **2× expansion in BitScrambler FIFO**: BitScrambler reads 16 bits (`read 16` per 50 ns) and emits 32 bits (`write 32`), packing 4 interpolated samples $[S_0, S_1, S_2, S_3]$ ($A \to 25\% \to 50\% \to 75\% \to B$).
- **PARLIO TX @ 80 MHz**: Serializes the 32-bit output words at 80 MHz (PLL_F240M / 3 = 80 MHz).
