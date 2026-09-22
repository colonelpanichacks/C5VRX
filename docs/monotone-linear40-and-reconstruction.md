# Monotone Linear40 Reconstruction Oracle & Hardware Boundary Analysis

## 1. Architectural Principle: Golden Phase5 is Frozen

The core demodulation stage is strictly frozen bit-for-bit:
```text
Q4/I4 @ 40 MHz PARLIO RX
  → positive RX edge selection (odd byte indices 1, 3, 5, ...)
  → Phase5 LUT (uniform 5-bit atan2 quantizer)
  → 50 ns discriminator span
  → gain = 2, pedestal = 20
  → Golden CVBS code G[k] @ 20 MS/s
```
Live physical testing has confirmed this demodulator produces crisp edges, minimal static, and zero rail-clipping spikes. No IQ bit-reduction, alternative decimation, or demod changes are permitted.

---

## 2. Monotone Midpoint Reconstruction Specification

To eliminate the 50 ns DAC staircase without generating overshoot or ringing, the reconstruction stage is strictly defined as a monotone $2\times$ linear interpolator:

```text
Y[2k]   = Golden[k]                          // BIT-EXACT preservation
Y[2k+1] = round((Golden[k] + Golden[k+1]) / 2) // Strictly bounded midpoint
```

### Mathematical Invariants & Verification Oracle
Implemented in [`tools/test_linear40_monotone.py`](file:///C:/Users/leonb/Twotoz/C5VRX-issue11-output/tools/test_linear40_monotone.py):

1. **Bit-Exact Preservation**:
   $$\forall k, \quad Y[2k] = \text{Golden}[k]$$
   The original Golden samples are guaranteed untouched.
2. **Zero Overshoot / Monotonicity**:
   $$\min(A, B) \le M \le \max(A, B)$$
   The midpoint $M$ strictly lies between adjacent Golden codes $A$ and $B$. It is mathematically impossible for the filter to generate new spikes or ringing.
3. **Halved Step Bound**:
   $$\max(|A - M|, |M - B|) \le \left\lceil \frac{|A - B|}{2} \right\rceil$$
   Every transition step is strictly at most half the size of the original step.

### Physical Capture Verification Results (`vtx_real_capture_v3.bin`)
- **Mean Jump**: Halved from $7.05$ codes $\to$ $3.52$ codes.
- **Large Jumps ($\ge 16$ codes)**: Reduced from $7.24\%$ $\to$ $1.10\%$.
- **Extreme Jumps ($\ge 32$ codes)**: Virtually eliminated ($0.02\%$).

---

## 3. BitScrambler Hardware Budget & Rule 7 Evaluation

We evaluated implementing the monotone midpoint calculation directly in the ESP32-C5 BitScrambler engine according to the proposed pipeline:
- Hold `prev_dac` (6 bits).
- Delay output by 1 sample (50 ns latency).
- Compute midpoint via BitScrambler counter arithmetic (`LDCTI` / `ADDCTI`).
- Loop unrolling (4 or 8 samples) to amortize instruction overhead.

### Hardware Constraints Identified

1. **Instruction RAM Limit (ESP32-C5 Silicon Limit)**:
   - BitScrambler instruction RAM holds a maximum of **8 instructions total** (`bsasm.py:839`):
     ```python
     if len(insts) > 8:
         raise RuntimeError('Program has more than eight instructions.')
     ```
   - Unrolling 8 samples ($8 \times 2 = 16$ instructions) is physically impossible.

2. **Single Opcode Slot per Bundle**:
   - Each BitScrambler instruction bundle has exactly 1 opcode slot (`bsasm.py:574`):
     ```python
     if 'op' in inst:
         raise bsasm_syntax_error(..., 'Cannot have multiple opcodes in one instruction')
     ```
   - Branching (`jmp` / `OP_IF`) and ALU counter operations (`LDCTI` / `ADDCTI`) cannot coexist in the same bundle.

3. **Hardware Bus Coupling (LUT Address vs Counter ALU Operands)**:
   - According to the official ESP-IDF BitScrambler peripheral specification:
     - `LDCTI(A|B)` and `ADDCTI(A|B)` take their operand from output register bits `16..31` (`data_out[31:16]`).
     - The LUT address bus is hardwired to the exact same output register bits `16..31`.
   - Therefore, an instruction cannot simultaneously address the LUT with Phase/Delta addresses and feed an arbitrary DAC code into the ALU.

4. **100% Saturated Real-Time Bundle Budget**:
   - PARLIO TX DAC rate: 40 MS/s $\implies$ 25 ns per output byte.
   - For 1 Golden sample (2 output bytes at 40 MS/s = 50 ns), the time budget is strictly **2 BitScrambler instruction bundles (50 ns)**.
   - Golden Phase5 requires:
     - Bundle 1: IQ $\to$ Phase lookup (consumes bundle 1).
     - Bundle 2: Phase delta $\to$ DAC lookup and PARLIO TX emit (consumes bundle 2).
   - All 2 available bundles are 100% saturated.
   - Inserting Counter A operations (`LDCTIA`, `ADDCTIA`, and extraction) expands the loop to $\ge 3$ bundles per sample (75 ns).
   - $75\text{ ns} / 2\text{ bytes} = 37.5\text{ ns/byte} > 25.0\text{ ns/byte}$, which triggers immediate **PARLIO TX FIFO underrun (black screen)**.

### Conclusion under Rule 7
Per explicit project criteria:
> **"6. Gemiddeld moet hij op maximaal 1 BitScrambler-bundle per uitgegeven 40-MS/s byte blijven."**
> **"7. Als dat niet past: stoppen. Niet opnieuw bits weggooien of de demod veranderen om hem passend te maken."**

Because the firmware midpoint arithmetic cannot fit within $\le 1$ bundle/byte without starving the DAC or compromising Golden Phase5 demodulation, **we stop firmware modification here** in strict compliance with Rule 7.

---

## 4. Physical Solution: Analog Reconstruction Filter

With Golden Phase5 proven optimal and frozen, the remaining 25/50 ns staircase steps and GPIO switching transients are properly handled in the analog domain:

```text
Golden Phase5 (ESP32-C5)
       │ (20 MS/s CVBS codes via 40 MS/s PARLIO TX)
       ▼
   GPIO-DAC (R-2R / Weighted Resistor Network)
       │
       ├───||─── GND   (C_recon ≈ 470 pF – 680 pF)
       │
       ▼
  Analog Display / Goggles (75 Ω terminated CVBS input)
```

### Component Calculation
- **Source & Load Impedance**: For standard 75 $\Omega$ CVBS transmission with a matching resistor network, the effective equivalent Thevenin impedance is $R_{eff} \approx 37.5\text{--}75\,\Omega$.
- **Target Cutoff Frequency**:
  - Full PAL/NTSC CVBS video bandwidth: $0\text{--}4.2\text{ MHz}$ (must pass unattenuated).
  - First DAC alias / clock feedthrough: $20\text{ MHz}$ and $40\text{ MHz}$ (must be filtered).
  - Target $f_c \approx 5.0\text{--}7.5\text{ MHz}$.
- **Capacitor Sizing**:
  $$C = \frac{1}{2\pi \cdot R_{eff} \cdot f_c}$$
  - For $R_{eff} = 75\,\Omega$, $f_c = 5.5\text{ MHz}$:
    $$C = \frac{1}{2\pi \cdot 75 \cdot 5.5 \times 10^6} \approx 385\text{ pF}$$
  - For $R_{eff} = 37.5\,\Omega$, $f_c = 5.5\text{ MHz}$:
    $$C = \frac{1}{2\pi \cdot 37.5 \cdot 5.5 \times 10^6} \approx 770\text{ pF}$$
  - Recommended standard ceramic capacitor: **470 pF to 680 pF** (NP0/C0G preferred for linear phase).
