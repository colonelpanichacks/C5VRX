# Root causes of static and video filtering in C5VRX

This document details why video static (snow, grain, and salt-and-pepper noise)
occurs in the C5VRX analog 5.8 GHz FPV receiver pipeline and outlines both
software/digital and hardware/analog remedies.

---

## 1. What causes the static?

Testing on ESP32-C5 revision v1.0 and A1/5865 MHz proves that the autonomous
RF producer, PARLIO RX, and BitScrambler WBFM pipeline produce locked,
recognizable live NTSC/PAL video. However, visible static remains.
Analysis reveals five distinct physical, mathematical, and architectural root
causes:

### 1.1 The FM triangular noise spectrum (f^2) and lack of de-emphasis
In frequency modulation (FM), the discriminator differentiates phase:

$$\Delta f(t) = \frac{1}{2\pi} \frac{d\phi(t)}{dt}$$

In the frequency domain, differentiation scales amplitude linearly with
frequency ($j\omega$). As a result, white thermal RF noise from the antenna and
LNA is transformed after FM demodulation into **triangular noise**, with a
power spectral density proportional to frequency squared:

$$S_N(f) \propto f^2$$

Noise at 5 MHz is approximately **100 times stronger** in power than noise at
500 kHz.

- **How standard analog FPV solves this:**
  Analog video transmitters (VTX) apply pre-emphasis (boosting high video
  frequencies before transmission according to CCIR 405 / 525-line curves).
  Standard analog receivers (VRX) apply a matched **de-emphasis low-pass filter**
  immediately following the FM discriminator.
- **The C5VRX situation:**
  The baseline C5VRX BitScrambler outputs raw phase differences directly to the
  DAC without a post-demodulation de-emphasis filter. The high-frequency noise
  triangle is displayed at full amplitude, which appears on screen as fine,
  buzzing static (snow).

---

### 1.2 Sub-Nyquist decimation without anti-aliasing (Factor 4:1)
The dataflow from RF to video involves two successive decimation steps:
1. The ESP32-C5 RF ADC produces samples at **~80 MS/s**.
2. **PARLIO RX** has a hardware ceiling of **40 MS/s** and captures every
   second sample from `MODEM_DIAG`. There is no anti-aliasing filter before this
   step; energy between 20 MHz and 40 MHz aliases directly into the 0–20 MHz band.
3. The **BitScrambler** reads 16 bits (two 40 MS/s samples) per cycle, discards
   the first sample, and evaluates only the second sample (`read 16`, bits 8..15).
   This is another 2:1 decimation without a filter.

Consequently, only 1 out of every 4 raw RF samples is processed ($s_3, s_7, \dots$).
All out-of-band RF noise across the full 40–80 MHz band aliases into the 5 MHz
video baseband.

---

### 1.3 Near-origin phase singularity (Salt-and-Pepper noise)
The pipeline receives 4-bit signed I (-8..+7) and 4-bit signed Q (-8..+7).
When the received RF carrier has low instantaneous amplitude (e.g. during multipath
nulls or fading valleys):
- The vector $(I, Q)$ approaches the origin $(0, 0)$.
- Near $(0, 0)$, $\text{atan2}(Q, I)$ is mathematically ill-conditioned.
- A single 1-LSB fluctuation in noise (e.g. from $(0, 1)$ to $(1, 0)$) causes
  a sudden **90° or 180° phase jump**.
- Across $\Delta t = 50\text{ ns}$, a 180° jump corresponds to an instantaneous
  frequency spike of:

  $$\Delta f = \frac{180^\circ}{360^\circ \times 50\text{ ns}} = 10\text{ MHz}$$

  This spikes to extreme DAC codes (code 0 = sync tip / black or code 63 = peak
  white), creating sharp white and black specks.

---

### 1.4 The 50 ns (n -> n+2) discriminator span and phase wrapping
Because samples are skipped, the discriminator compares samples spaced 50 ns
apart ($f_s = 20\text{ MHz}$).
The unambiguous phase range $[-\pi, +\pi)$ across 50 ns corresponds to
$\pm 10\text{ MHz}$.
When full video modulation (carrier deviation up to $\pm 4\text{ MHz}$ plus
3.58/4.43 MHz color burst and 6.0/6.5 MHz audio subcarriers) coincides with an
RF noise peak, the total phase jump can exceed $\pi$. This triggers a
**cycle slip** (wrapping from $+180^\circ$ to $-180^\circ$), appearing as an
instantaneous horizontal tear or spike on that scanline.

---

### 1.5 20 MHz DAC switching harmonics and ground bounce
The passive resistor DAC is driven by 6 GPIOs switching at 20 MS/s with sharp
digital edges:
- Without an analog reconstruction filter, 20 MHz square-wave harmonics reach
  the monitor.
- GPIO driver currents on the 3.3V supply can couple into the adjacent 5.8 GHz
  on-chip RF receiver.

---

## 2. Digital mitigations in firmware

### 2.1 Narrowing Wi-Fi analog baseband bandwidth (`WIFI_BW20`)
In `main/wifi5.c`, setting `.ghz_5g = WIFI_BW20` forces the ESP32-C5 RF frontend
to engage its 20 MHz analog baseband channel filter instead of 40 MHz. This cuts
out approximately **3 dB of out-of-band thermal noise** before the signal ever
reaches the ADC.

### 2.2 Phase5 centroid mapping and near-origin suppression (PR #5)
In `main/wbfm_q4.c` and `main/c5vrx2_wbfm_q4_phase5_2to1.bsasm`:
- Vectors with $I^2 + Q^2 < 5$ are assigned to `INVALID_STATE` ($31$).
- When either sample is invalid, the LUT defaults to `pedestal_code` (blanking level 20)
  instead of clipping to extreme white/black levels.
- Precomputed circular centroids smooth the 5-bit quantization steps and ensure
  exact modulo wrapping across the $\pm\pi$ boundary.

### 2.3 Limits of digital filtering inside the BitScrambler
At 20 MS/s output rate, the BitScrambler has a strict instruction budget of
**2 instruction bundles per output byte**.
Multi-tap digital FIR/IIR filtering or moving-average accumulators require
multiple `LDCTDAL`/`ADDCTIAL` bundles, which causes FIFO underrun at 20 MS/s.
Therefore, high-order filtering cannot be performed purely in the BitScrambler
without reducing the output sample rate.

---

## 3. Hardware analog mitigation (Reconstruction & De-Emphasis Capacitor)

A simple, zero-latency, zero-CPU hardware addition effectively addresses the
$f^2$ noise triangle and 20 MHz DAC steps:

### The RC reconstruction filter
The XIAO DAC joins six resistors at the `VIDEO` node, with a 200 $\Omega$ shunt
resistor to GND and a 75 $\Omega$ terminated display:

$$R_{eff} = 200\ \Omega \parallel 75\ \Omega \approx 54.5\ \Omega$$

Placing a small ceramic capacitor $C$ in parallel with the 200 $\Omega$ resistor
(between `VIDEO` and `GND`) creates a 1st-order analog low-pass filter with
cutoff frequency:

$$f_c = \frac{1}{2\pi R_{eff} C}$$

| Capacitor ($C$) | Cutoff Frequency ($f_c$) | Effect on Video & Static |
|---|---|---|
| **None** | $> 100\text{ MHz}$ (trace parasitic) | Full 20 MHz DAC steps and full $f^2$ FM noise pass to display |
| **220 pF** | $\approx 13.3\text{ MHz}$ | Mild smoothing; attenuates 20 MHz DAC harmonics |
| **330 pF** | $\approx 8.9\text{ MHz}$ | Good compromise; cuts upper FM noise, preserves full PAL/NTSC chroma |
| **470 pF** | $\approx 6.2\text{ MHz}$ | Strong static suppression; acts as natural CCIR 405 de-emphasis filter |
| **680 pF** | $\approx 4.3\text{ MHz}$ | Very clean, soft picture; slight roll-off on PAL 4.43 MHz color burst |

### Recommendation
Fit a **330 pF or 470 pF ceramic capacitor** directly across the 200 $\Omega$
resistor (between XIAO `VIDEO` output and `GND`). This eliminates the DAC
staircase steps and suppresses the high-frequency FM noise triangle while
retaining full video sharpness and color lock.
