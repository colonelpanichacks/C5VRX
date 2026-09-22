# Analog FPV DSP pipeline

Once C5VRX obtains phase-bearing receive samples, the mainline pipeline stops at
**composite video**. It does not continue into RGB pixels.

```text
5.8 GHz analog FPV RF
        |
        v
C5 RF frontend / ADC
        |
        v
complex / phase-bearing I,Q
        |
        v
FM discriminator
        |
        v
filter / level normalize / decimate
        |
        v
composite video samples (CVBS)
        |
        v
PARLIO + 6-bit passive DAC
        |
        v
1 Vpp / 75-ohm analog video
```

That is the core receiver. Anything beyond the CVBS connector belongs to the
display/DVR, not to C5VRX.

See [`analog-first-architecture.md`](analog-first-architecture.md) and
[`video-output.md`](video-output.md).

## Reference FM discriminator

For complex samples `x[n] = I[n] + jQ[n]`:

```text
y[n] = angle(x[n] * conj(x[n-1]))
```

`y[n]` is phase increment in radians/sample. Instantaneous frequency is:

```text
f[n] = y[n] * sample_rate / (2*pi)
```

This is the mathematical reference and is ideal for host-side validation. It is
not automatically the correct real-time C5 implementation because complex
multiply plus atan2-like work at tens of millions of samples per second is
expensive on the application CPU.

## Hardware-assisted discriminator experiment

C5VRX investigates the ESP32-C5 BitScrambler as a quantized phase
preprocessor/discriminator.

The recovered RF-test format provides signed 10-bit I and Q. Quantize each to
its five most-significant bits:

```text
I10 -> I5 --+
            +--> 1024-entry LUT
Q10 -> Q5 --+
```

A 1024-entry table with a 16-bit result occupies exactly 2048 bytes. Each entry
can store:

```text
low byte:  phase8
high byte: -phase8 modulo 256
```

The preferred live experiment is then:

```text
phase8[n] + (-phase8[n-1]) -> delta phase8
```

using BitScrambler state, counters or output-history mechanisms rather than
complex multiplication/trigonometry.

`tools/bitlut_fm.py` verifies the numerical approximation on synthetic
video-like FM. It does **not** prove hardware throughput or continuous RF
capture.

## One-LUT hardware constraint

The host tool also models a lower-precision phase-difference LUT. That table is
another complete 2 KiB LUT.

The ESP32-C5 BitScrambler has only one 2 KiB LUT, therefore this conceptual pair:

```text
2 KiB I/Q -> phase LUT
+
2 KiB current/previous phase -> delta LUT
```

cannot be simultaneously resident in one live BitScrambler pipeline.

The mainline implementation should therefore target:

```text
one resident I/Q -> phase LUT
+
state/counter subtraction
```

unless later measurements prove that a time-multiplexed/reloaded alternative
can meet the required rate.

## CVBS reconstruction

FM discriminator output is not ready to drive the DAC directly. The streaming
path still needs measured signal conditioning such as:

```text
phase delta
  -> remove discriminator/DC bias as required
  -> reject out-of-band noise
  -> apply receive/baseband de-emphasis if required by measured VTX path
  -> scale/offset into composite-video voltage codes
  -> decimate to the chosen CVBS sample rate
```

Do not hard-code an elaborate television decoder into this stage. The goal is
to reproduce the analog waveform faithfully enough that a normal PAL/NTSC
monitor locks to it.

## Reference output representation

The internal stream can remain 8-bit even though the physical DAC is six bits:

```text
CVBS sample byte
      |
      v
scale / clamp
      |
      v
0..63 physical output code
      |
      v
PARLIO D0..D5
```

This keeps buffers simple and leaves room for calibration, filtering and OSD
operations before the final six-bit quantization.

## Buffering and latency

Avoid frame buffers.

At 20 MS/s a ~64 us PAL line is about:

```text
20,000,000 * 64e-6 = 1280 samples
```

A pair of byte-oriented line buffers is therefore only about 2.5 KiB. Streaming
or short ping-pong buffers should keep latency dominated by RF/DSP filtering
rather than an entire video frame.

The exact sample rate is a hardware decision. Twenty MS/s is a development
target, not a permanent requirement.

## OSD remains sample-domain

Simple status OSD can be inserted without decoding to pixels.

After horizontal/vertical timing is known:

```text
CVBS samples
  -> line/x timing
  -> tiny character mask
  -> replace selected samples with calibrated black/white levels
  -> DAC
```

This keeps an external OSD chip and full RGB path out of the core design.

## Host-side validation

`tools/wbfm_demod.py` implements the exact reference discriminator for
interleaved IQ captures. Use the host first to prove that a C5 dump contains
real analog-FPV phase information.

Synthetic self-test:

```bash
python tools/wbfm_demod.py --self-test
```

Example capture:

```bash
python tools/wbfm_demod.py capture.iq \
  --dtype '<i2' \
  --sample-rate 20000000 \
  --output demod.f32
```

The output is float32 instantaneous-frequency data.

## What to prove in the first real capture

Validate progressively:

1. A known CW signal should produce a nearly constant discriminator output.
2. A controlled FM source should reproduce its modulation waveform.
3. An analog FPV VTX with a static image should show repeatable line-sync structure.
4. Changing image brightness should change the recovered composite waveform.
5. A finite capture should reconstruct recognizable grayscale scanline content.
6. Measured chroma-region energy should be consistent with PAL/NTSC color content.
7. Only after continuous capture exists should the streaming discriminator be connected to PARLIO CVBS output.

## Mainline non-goals

Do not spend real-time C5 budget on these before live CVBS exists:

- complete PAL/NTSC color decoding to pixels;
- RGB/YUV conversion;
- direct LCD rendering;
- full-frame buffering;
- UVC video encoding/transport.

A normal analog display already solves those problems after the connector.
