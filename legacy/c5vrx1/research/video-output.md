# Analog video output architecture

C5VRX's primary output is **analog composite video (CVBS)**.

The receiver should behave like a classic analog VRX: recover WBFM baseband and
reconstruct the composite waveform directly. Full PAL/NTSC-to-pixel decoding is
not part of the core receiver path.

## Permanent receiver output states

The receiver profile owns AV-out continuously after its control task starts.
Before any accepted RF samples it emits a standards-shaped PAL raster with the
C5VRX logo. Once varying samples are observed without a continuous H/V lock,
it emits moving monochrome snow. Missing later blocks retain snow, so ordinary
static never causes a misleading logo flash. A switch to live waveform data is
allowed only after a continuous source establishes valid timing.

The renderer is scanline-only. Two 25,600-byte PARLIO DMA chunks provide 2.56
ms already queued at 20 MS/s, exceeding the 1.93--1.99 ms bounded capture time
measured on the physical XIAO without allocating a video framebuffer.

## Primary path

```text
5.8 GHz analog FPV RF
        |
        v
ESP32-C5 RF / phase-bearing I,Q
        |
        v
hardware-assisted FM discriminator
        |
        v
filter / decimate / level normalize
        |
        v
~20 MS/s CVBS sample stream
        |
        v
PARLIO + GDMA
        |
        v
6-bit weighted resistor DAC
        |
        v
~1 Vpp into 75 ohm
        |
        v
analog monitor / goggles / DVR
```

The key property is that horizontal sync, vertical sync, luma and chroma remain
inside the reconstructed waveform. The display or DVR already knows how to
decode them.

## Why analog-only is the mainline target

A digital display path would add at least:

```text
CVBS
  -> sync detector
  -> active-line extraction
  -> luma/chroma decode
  -> YUV/RGB conversion
  -> frame/line buffering
  -> LCD or host transport
```

None of that is needed to produce normal AV output. It adds compute, memory,
latency and often extra hardware without helping the core goal of replacing an
RX5808-class analog receiver.

Digital preview may be revisited later as a diagnostic or companion-processor
feature, but it must not drive the core architecture.

## PARLIO as the video DAC engine

ESP32-C5 has PARLIO attached to GDMA. The peripheral can transmit an 8-bit
parallel stream at rates suitable for sampled composite video without
bit-banging GPIOs.

C5VRX transports one byte per video sample but does not need all eight physical
pins. The current reference output uses only six low bits:

```text
RAM byte
   |
   v
PARLIO 8-bit transport
   |
   +--> D0 GPIO -- R
   +--> D1 GPIO -- R
   +--> D2 GPIO -- R
   +--> D3 GPIO -- R
   +--> D4 GPIO -- R
   +--> D5 GPIO -- R
   |
   v
weighted analog node -> 75-ohm CVBS load
```

The unused D6/D7 outputs remain disconnected.

## Why six physical bits

A nominal 1 V full-scale composite output gives approximately:

```text
6-bit step = 1 V / 63 = 15.9 mV
```

That is a useful compromise between hardware count and analog level resolution.
It keeps blanking, black, white and sync levels much easier to represent than a
3- or 4-bit experiment while still requiring only six GPIOs.

Lower bit depths remain useful experiments, but six bits are the reference
architecture until hardware measurements show otherwise.

## Passive 75-ohm reference network

For a 3.3 V logic source and a normal 75-ohm terminated video input, a binary
weighted network can be chosen so that the source is approximately 75 ohms and
full-scale is approximately 1 V at the terminated load.

Nominal E96 starting values:

| Bit | Resistor to video node |
|---|---:|
| D0 / LSB | 7.87 kOhm |
| D1 | 3.92 kOhm |
| D2 | 1.96 kOhm |
| D3 | 976 Ohm |
| D4 | 487 Ohm |
| D5 / MSB | 243 Ohm |
| video node -> GND | 191 Ohm |

Ideal calculation gives roughly:

```text
Thevenin source impedance: ~75 ohm
open-circuit full-scale:   ~2.0 V
75-ohm loaded full-scale:  ~1.0 V
```

These are **starting values, not a production guarantee**. GPIO output
resistance, VOH droop, resistor tolerance, connector/cable capacitance and the
actual monitor termination alter the waveform. Validate on a scope and adjust
or calibrate the digital level mapping if needed.

A dedicated buffer may still be useful for long cables, ESD robustness or a
strict production video interface, but it is not assumed to be mandatory for
the minimum-hardware short-trace prototype.

Never connect raw 3.3 V GPIOs directly to a 75-ohm input.

## Existing independent output experiment

`main/c5vrx_cvbs_out.c` already exercises the output side independently of RF.
It generates a 20 MS/s PAL-line-like test waveform through PARLIO so the
following can be proven before continuous I/Q exists:

```text
C5 memory -> GDMA -> PARLIO -> resistor DAC -> scope / AV monitor
```

The next output-side milestone is a complete stable PAL/NTSC test frame rather
than only a repeated horizontal line.

## WBFM compute remains the difficult part

The expensive operation is earlier in the chain:

```text
y[n] = angle(x[n] * conj(x[n-1]))
```

Doing complex multiply plus an atan2-like operation at tens of millions of
samples per second on the CPU is a poor final architecture.

C5VRX therefore investigates the BitScrambler as a quantized phase
preprocessor/discriminator.

## Single-LUT BitScrambler direction

The recovered sample format contains signed 10-bit I and Q. Keep the five most
significant bits of each component:

```text
I10 -> I5 --+
            +--> 10-bit LUT address
Q10 -> Q5 --+
```

There are exactly 1024 coarse I/Q cells. A 1024 x 16-bit table occupies the C5
BitScrambler's complete 2048-byte LUT and can return:

```text
low byte:  phase8
high byte: -phase8 modulo 256
```

The preferred live experiment is then to use BitScrambler state/counters or
output history to form:

```text
phase[n] - phase[n-1]
```

without trigonometry.

### Important one-LUT constraint

`tools/bitlut_fm.py` also models a second 2 KiB phase-difference LUT. That is a
useful numerical comparison, but the C5 only has one 2 KiB BitScrambler LUT.
The I/Q-to-phase table and a second full-size delta table therefore cannot both
be resident simultaneously in the same live pipeline.

For the analog-first mainline, prefer:

```text
one resident I/Q -> phase LUT
        +
BitScrambler state/counter subtraction
```

over an architecture that assumes two simultaneous LUT memories.

## Target streaming pipeline

If a continuous RF sample producer can be recovered, the preferred end state is:

```text
~40 MS/s packed phase-bearing samples
        |
        v
single-LUT hardware-assisted discriminator
        |
        v
filter / decimate
        |
        v
~20 MS/s CVBS bytes
        |
        v
GDMA / PARLIO
        |
        v
6-bit passive DAC
```

Twenty MS/s is a development target, not a fixed final rate. Hardware testing
should determine whether a higher output sample rate materially improves color
or edge quality without harming the RF/DSP budget.

## OSD without an OSD chip

Analog-first does not require an AT7456E-class part. Once horizontal/vertical
sync timing is known, simple monochrome OSD can be inserted by replacing
selected CVBS samples during active lines:

```text
recovered CVBS
      |
      +--> sync/line timing
      +--> optional character mask
      |
      v
modified CVBS -> PARLIO
```

This can produce channel/RSSI/status text without converting the complete frame
to RGB.

## One-bit output research path

An even smaller experimental route is possible in principle:

```text
CVBS values
   -> software/noise-shaped 1-bit stream
   -> fast serial peripheral
   -> one GPIO
   -> passive reconstruction network
   -> video
```

This could reduce the output to one GPIO plus a few passives, but it trades a
handful of resistors for a much harder signal-integrity/noise-shaping problem.
It is therefore a research branch, not the reference architecture.

## Recommended development order

1. Generate a complete valid PAL/NTSC test frame from PARLIO.
2. Validate the six-bit passive DAC into a real 75-ohm load with a scope.
3. Compare 4-, 5- and 6-bit output quality on an analog monitor.
4. Prove real 5.8 GHz finite I/Q capture.
5. Verify the actual RF capture sample rate and usable bandwidth.
6. Trace the producer behind the finite dump RAM into a continuous/chained path.
7. Implement and benchmark the single-LUT BitScrambler discriminator.
8. Join RF -> WBFM -> CVBS -> PARLIO into the first live receiver.
9. Add RSSI/autoscan and simple sample-domain OSD only after live video works.

## Core rule

If a normal analog monitor can perform a task after the CVBS connector, C5VRX
should not perform that task before the connector unless a measurement proves
it is necessary.
