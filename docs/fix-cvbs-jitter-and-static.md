# Fix for CVBS Horizontal Line Jitter and Video Static

This document details the root causes and digital firmware fixes for the
horizontal line jitter, line tearing, and colored static / "color bombs"
identified in Issue #11 and PR #10 on the Seeed Studio XIAO ESP32-C5 receiver.

---

## 1. Problem Summary & Prior Evidence

In Issue #11 and PR #16, physical measurements of the 20 MS/s digital CVBS stream
directly before the resistor DAC established:
1. **The BitScrambler hardware is 100% exact in steady-state** (0 mismatches across
   11,314 samples against the trajectory reference model). No peripheral corruption
   or clock slippage exists in the digital core.
2. **The digital CVBS stream already exhibits horizontal line jitter**:
   - RMS jitter: **112.9 ns** (2.3 samples).
   - Peak-to-peak jitter: **452.1 ns** (9.4 samples / 9 pixels).
   - Line length variations between 1267.9 and 1280.5 samples around nominal NTSC
     (1271.1 samples).
3. **Color-burst phase instability**: In NTSC, one complete 3.58 MHz color subcarrier
   period is only 279.4 ns. When H-sync edge detection wanders by 100–450 ns, the
   color-burst sampling window rotates phase line-to-line, causing TV chroma decoders
   to desync and produce colored horizontal layers and flashing "color bombs".

---

## 2. Root Causes Identified

### Root Cause A: Wi-Fi Analog Baseband Bandwidth (`WIFI_BW40` vs `WIFI_BW20`)
In `main/wifi5.c`, the 5 GHz Wi-Fi receiver frontend was configured with:
```c
wifi_bandwidths_t bandwidths = {
    .ghz_2g = WIFI_BW20,
    .ghz_5g = WIFI_BW40,
};
```
Analog composite video baseband is approximately 6 MHz (FM Carson bandwidth for
standard 5.8 GHz VTX is ~14–16 MHz). Setting `WIFI_BW40` opened the analog on-chip
channel filter to 40 MHz, capturing double the RF noise bandwidth and passing
**~3 dB of unnecessary out-of-band thermal noise** into the ADC.

### Root Cause B: Inverted Phase Wrap Bug in the Trajectory LUT (PR #10)
In `tools/train_trajectory_lut.py`, target calculation was defined as:
```python
adjacent = wrap_r(PHI[m] - PHI[p]) + wrap_r(PHI[c] - PHI[m])
target = scale(np.rint(adjacent * 256 / (2 * np.pi)).astype(int))
```
The sum `adjacent` was deliberately left unwrapped based on the assumption that
a 40 MS/s transition could legitimately exceed $\pm\pi$.

However, under the uniform white-noise prior, when intermediate sample $m$ falls in
the opposite quadrant due to noise near the origin, two successive $+135^\circ$
hops sum to $+270^\circ$. For a true bandlimited FM video signal, a $+270^\circ$
rotation in 50 ns would represent an impossible 15 MHz frequency deviation.
`scale()` clipped $+270^\circ$ to **code 60–63 (peak white)**.

Consequently:
- For transitions with $\Delta P_4 \le -4$ (sync tips / black levels), **24 addresses
  in the LUT output peak white (DAC 60–63)**!
- For transitions with $\Delta P_4 \ge +4$ (white level), **24 addresses in the LUT
  output sync tip / black (DAC 0–3)**!

Whenever low-amplitude RF carrier or noise caused a sample to hit these addresses
during a horizontal sync pulse, the DAC output produced a peak-white spike in the
middle of the sync tip. This corrupted the falling edge detected by the video
monitor's sync slicer, causing it to trigger 4–9 samples early or late and creating
the 452 ns pk-pk line jitter and horizontal layer steps.

### Root Cause C: Severe DAC Posterization in Uniform Prior
The uniform geometry prior in PR #10 left 380 out of 1024 addresses untrusted.
On real VTX video, this produced only **15 distinct DAC levels**, causing visible
quantization banding and coarse threshold crossings.

---

## 3. Firmware Fixes Applied

### 1. RF Baseband Bandwidth Requirement (`WIFI_BW40` is Mandatory)
Initially, testing hypothesized that setting `WIFI_BW20` would cut out-of-band RF noise.
However, physical hardware testing immediately disproved this hypothesis:
- Carson's bandwidth for 5.8 GHz analog video with 4.43 MHz / 3.58 MHz color subcarrier
  is $B = 2(\Delta f + f_m) \approx 16\text{--}18\text{ MHz}$.
- `WIFI_BW20` restricts the on-chip complex baseband filter to ~8–9 MHz, cutting off
  the FM color-burst sidebands and high-frequency luminance.
- On hardware, `WIFI_BW20` caused immediate loss of resolution and severe chroma PLL
  unlock (alternating red and green "Hanover bars" across lines).
- Therefore, **`WIFI_BW40` is strictly mandatory** to preserve the full video modulation
  spectrum. Pre-discriminator filtering cannot substitute for post-discriminator de-emphasis.

### 2. Correcting Trajectory Phase Wrapping in `tools/train_trajectory_lut.py`
In `tools/train_trajectory_lut.py`:
```python
target = scale(np.rint(wrap_r(adjacent) * 256 / (2 * np.pi)).astype(int))
```
Wrapping `adjacent` to $[-\pi, +\pi)$ enforces that instantaneous frequency deviation
remains within physical FM discriminator limits. This eliminates all 48 inverted
wrap entries (0 inverted white entries, 0 sync-tip white bursts).

### 3. Training Trajectory LUT on Real Hardware VTX RF
The LUT was trained using real RF captures:
- `measurements/issue-11-cvbs/vtx_real_capture_v3.bin` (32 KiB from user's hardware)
- Validated against held-out `measurements/issue-11-cvbs/raw-vtx-on.bin` (16 KiB)

Results:
- **DAC levels on live video increased from 15 to 34+ distinct levels** (smooth analog
  gradation matching Phase 5 quality).
- Inverted wrap spikes eliminated.
- Clean sync tips without bouncing to pedestal (code 20).
- Held-out validation MAE: **3.34 codes** (out of 64), hard errors $\ge 16$ codes: **5.06%**.

---

## 4. Hardware Recommendation (Analog De-emphasis & Reconstruction)

To complement the digital fixes:
- Standard analog video transmitters (VTX) apply 50 µs pre-emphasis (boosting high
  frequencies).
- The FM discriminator produces triangular $f^2$ noise (100x stronger power at 5 MHz
  than at 500 kHz).
- Fitting a **330 pF or 470 pF ceramic capacitor** directly across the 200 $\Omega$
  resistor (between XIAO `VIDEO` output and `GND`) creates a 1st-order analog
  reconstruction low-pass filter ($f_c \approx 6.2\text{--}8.9\text{ MHz}$).
- This attenuates the 20 MHz DAC switching staircase steps and suppresses the upper
  FM triangular noise without softening the video image.
