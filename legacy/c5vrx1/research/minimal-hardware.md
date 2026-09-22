# Minimal-hardware target

The end goal for C5VRX is not an SDR dev stack with extra video chips. It is a receiver that is physically closer to a single RF/MCU part plus a handful of passives.

## What "better than RX5808" can mean

For normal analog FPV, the transmitter is still PAL/NTSC-style composite video carried by wideband FM. C5VRX cannot invent source detail or real source frame rate that was never transmitted.

What it *can* potentially improve, if the C5 RF path is good enough, is:

- tuning flexibility and channel coverage;
- filtering/selectivity in software;
- AFC / carrier-offset correction;
- noise handling and clipping behavior;
- direct digital display output without re-encoding and re-decoding CVBS;
- line-based deinterlacing to a 50/60 Hz progressive display update;
- upscaling to the native LCD resolution without an extra analog video-decoder IC;
- lower end-to-end latency by avoiding frame buffers.

Sensitivity, multipath behavior and usable range versus RX5808 must be measured. The C5 front-end was built for Wi-Fi, not analog FPV, so "better RF" is a target rather than an assumption.

## The three end-state architectures

### 1. Absolute minimum integrated-display receiver

```text
antenna
  |
ESP32-C5
  |  RF -> I/Q -> WBFM -> sync/luma/chroma
  v
I80 / parallel / SPI LCD
```

No CVBS DAC and no external PAL/NTSC decoder. This is the smallest architecture when C5VRX is built into the same product as the screen.

The price is compute: after FM demodulation the C5 must recover video timing and pixels. This should be line-streamed, not frame-buffered. Grayscale is much cheaper than color; PAL/NTSC chroma can be added once the RF path is proven.

ESP32-C5 officially exposes GDMA, PARLIO, an I80 LCD path, I2S/PDM, Sigma-Delta Modulation and BitScrambler hardware, so several output/acceleration routes exist without another controller IC.

### 2. Minimum universal analog receiver

```text
antenna
  |
ESP32-C5
  |  RF -> I/Q -> hardware-assisted WBFM
  v
3-6 GPIO resistor DAC
  |
CVBS OUT
```

This skips PAL/NTSC decoding completely. WBFM already reconstructs the composite waveform; we only need to turn sampled CVBS back into a voltage.

For a short PCB trace into a known 75-ohm terminated video input, the first experiment can be only binary-weighted resistors. No THS7314 or external video DAC is required just to prove the architecture.

A long RCA/coax output is a different requirement: proper 75-ohm source matching and ESD/protection may justify one small buffer later. That is an interface-quality choice, not a receive-core requirement.

### 3. Development/debug receiver

```text
ESP32-C5 -> USB Serial/JTAG -> C5VRX Control
```

Useful for tuning, finite IQ/CVBS captures and low-resolution previews. The built-in USB full-speed interface is not the target for raw continuous wideband I/Q.

## How many DAC bits do we really need?

`tools/minimal_cvbs_dac.py` models a PAL-like composite line containing sync, luma and a 4.43361875 MHz chroma component, then quantizes it to tiny GPIO DACs.

The purpose is not to claim a broadcast-compliant output. It answers a simpler BOM question: how quickly does composite quality collapse as GPIO/resistor count falls?

Current synthetic result at 40 MS/s is approximately:

| Output | Synthetic result | Interpretation |
| --- | --- | --- |
| 1-bit PDM | ~12 dB SNR | wonderfully small, probably too noisy for a quality color target |
| 2-bit DAC | ~12 dB SNR | enough levels for sync/black/white-ish proof, chroma is very coarse |
| 3-bit DAC | ~20+ dB SNR | first plausible ultra-minimal color experiment |
| 4-bit DAC | low/mid-20 dB in current stress test | four resistors is still essentially free hardware |
| 5-bit DAC | ~30+ dB | attractive quality/BOM compromise |
| 6-bit DAC | ~40 dB | likely small enough that RF/video noise dominates first |

These figures depend on waveform and reconstruction filtering and must not be treated as measured C5 hardware performance.

## Interesting passive-only 3-bit experiment

For ideal 3.3 V GPIOs feeding a destination that really terminates at 75 ohm, a binary-weighted three-resistor network can be sized so code `111` lands near 1.0 V.

Ideal values from the simple loaded-node model are roughly:

```text
MSB:  ~302 ohm
bit1: ~604 ohm
LSB:  ~1.21 kohm
```

Convenient 1% starting values are around `301R / 604R / 1.21k` (or nearby stocked values).

That gives eight voltage codes from sync level toward white using only three GPIOs and three resistors. It does **not** provide a textbook 75-ohm source impedance, so it is for a short-trace / direct-input experiment first, not a claim of compliant long-cable CVBS output.

A 4-6 bit network costs only one to three extra resistors and may be the actual production sweet spot.

## Why direct digital display may ultimately look better

A normal cheap display chain can be:

```text
RX5808 -> analog CVBS -> video decoder -> frame processing -> LCD
```

C5VRX can potentially do:

```text
C5 RF -> WBFM -> line decode -> LCD pixels
```

That removes a DAC, an analog cable/trace, another ADC and a separate CVBS-decoder IC. The source is still analog PAL/NTSC, so native information does not magically become HD, but fewer conversions can preserve more of what was transmitted and reduce latency.

For interlaced sources, a line-based bob/deinterlace path can update a progressive LCD at the field rate (roughly 50 Hz PAL / 60 Hz NTSC) without waiting for a whole frame. Upscaling to 800x480 or another panel resolution is possible, but it is upscaling rather than new RF detail.

## Receiver-core BOM target

A credible ultimate target is therefore:

```text
ESP32-C5 module / SoC
+ RF matching / antenna parts required by the chosen package/module
+ normal power decoupling
+ optional 3-6 resistors for CVBS out
```

No RX5808. No external ADC. No external video decoder. No dedicated video DAC unless measurements show the passive output is the quality bottleneck.

If the product drives a compatible LCD directly, even the CVBS resistors disappear.

## The actual hard problem

Output hardware is no longer the scary part. The unresolved core is still:

1. obtain a continuous phase-bearing C5 receive stream rather than a finite 64 KiB dump;
2. perform WBFM at the required rate without consuming the whole CPU;
3. measure C5 RF sensitivity/noise/selectivity against an RX5808-class module;
4. choose either direct CVBS or line-decoded digital output based on the final product.

The BitScrambler LUT discriminator work exists specifically to make step 2 look like a streaming hardware transform rather than millions of CPU `atan2` operations.
