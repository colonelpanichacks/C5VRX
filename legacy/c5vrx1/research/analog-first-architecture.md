# Analog-first C5VRX architecture

## Design rule

C5VRX should add external hardware only when the ESP32-C5 cannot perform the
required function itself or when measurements prove a passive implementation is
not good enough.

The reference product target is therefore **analog CVBS out**, not a direct
digital display.

```text
5.8 GHz antenna
      |
      v
ESP32-C5 RF chain
      |
      v
phase-bearing receive samples
      |
      v
WBFM discriminator
      |
      v
CVBS sample stream
      |
      v
PARLIO + GDMA
      |
      v
6-bit passive DAC
      |
      v
75-ohm AV output
```

## Why CVBS is the endpoint

Analog FPV transmitters already send PAL/NTSC composite video as wideband FM.
After FM demodulation the useful result is the original composite waveform.
Converting that waveform into RGB pixels inside C5VRX would only add work that
a normal analog monitor/goggle/DVR already performs.

The core receiver therefore does not need:

- a framebuffer;
- PAL/NTSC-to-RGB decoding;
- a digital LCD controller;
- UVC/high-speed USB video;
- a separate video decoder IC;
- a separate video DAC IC;
- an AT7456E-style OSD IC.

Digital features can be added later outside the critical path.

## Minimum active-hardware target

For a module-based implementation, the aspirational core BOM is:

```text
1x ESP32-C5 module
1x power stage if the host does not already provide a suitable rail
passive decoupling / boot support
6-bit video resistor network
antenna connection
AV connector or pad
```

A bare-chip design additionally needs the normal crystal/RF/flash requirements
for the selected C5 variant. A module is therefore the easier first custom PCB.

The goal is not to claim that every protection or production-interface part is
unnecessary. The goal is to keep those parts outside the functional minimum so
they are added for measured reliability requirements rather than habit.

## Reference six-bit video DAC

The reference passive output assumes:

```text
GPIO logic high: nominally 3.3 V
video load:       75 ohm
video full scale: approximately 1 V
```

Use six binary weighted GPIO contributions plus a shunt resistor at the video
node.

Nominal E96 starting values:

| Signal | Resistor to video node |
|---|---:|
| D0 / least significant | 7.87 kOhm |
| D1 | 3.92 kOhm |
| D2 | 1.96 kOhm |
| D3 | 976 Ohm |
| D4 | 487 Ohm |
| D5 / most significant | 243 Ohm |
| video node to GND | 191 Ohm |

Ideal resistor-network math gives approximately:

```text
source impedance:        75 ohm
open-circuit full scale: 2.0 V
75-ohm loaded full scale:1.0 V
```

This is intentionally a starting network for scope validation. Real C5 GPIOs
are not ideal voltage sources. Their output resistance and high-level droop,
plus PCB/cable/load parasitics, will change the final levels.

The prototype must measure at least:

```text
sync tip
blanking level
black level
white level
source amplitude into 75 ohm
edge/ringing behavior
```

Then either tune resistor values or use a software output mapping that does not
consume the full 0..63 range.

## Why six bits

At a 1 V full-scale range:

```text
4 bit: 1 / 15 = 66.7 mV per code
5 bit: 1 / 31 = 32.3 mV per code
6 bit: 1 / 63 = 15.9 mV per code
8 bit: 1 / 255 = 3.9 mV per code
```

Six bits are a useful first quality/BOM point. They preserve much more control
around sync/blank/black levels than very coarse ladders while still requiring
only six physical data pins.

The correct bit depth must ultimately be decided by real analog monitors, not
by the table alone. The firmware should retain 2-8-bit configurability for A/B
testing.

## PARLIO transport

The firmware can keep a byte-oriented CVBS stream even with a six-bit physical
DAC:

```text
sample byte 0..255
      |
      v
scale/quantize to 0..63
      |
      v
PARLIO byte transport
      |
      v
D0..D5 physically routed
D6..D7 disconnected
```

This avoids packed-bit complexity and keeps DMA buffers simple.

A development target around 20 MS/s makes one ~64 us PAL line approximately
1280 samples, so line-oriented buffers stay tiny.

## No mandatory output buffer

The minimum prototype should first test the passive network directly into a
known 75-ohm input over a short connection.

A buffer should be added only when measurements show it is needed for one of:

- cable drive or edge integrity;
- required ESD/protection behavior;
- isolation from variable loads;
- production amplitude tolerance;
- connector hot-plug robustness.

If a product requirement demands those properties, the buffer becomes a
reliability/interface component rather than part of the fundamental VRX signal
chain.

## DSP architecture

The reference FM discriminator should minimize CPU work.

The recovered vendor sample format contains signed 10-bit I and Q. A useful
BitScrambler experiment quantizes each to five bits:

```text
I10 -> I5 --+
            +--> 1024-entry LUT
Q10 -> Q5 --+
```

The complete 1024 x 16-bit table occupies exactly 2048 bytes and can store both:

```text
phase8
-phase8 modulo 256
```

The preferred hardware experiment is then:

```text
current phase + negative previous phase -> delta phase
```

using BitScrambler state/counter/output-history mechanisms rather than a second
full LUT.

### One-LUT constraint

The C5 BitScrambler has one 2 KiB LUT. A second 2 KiB phase-difference LUT can
be useful in host simulations but cannot be simultaneously resident beside the
I/Q-to-phase LUT.

The mainline architecture must therefore assume **one resident phase LUT** unless
a future implementation deliberately time-multiplexes/reloads hardware and can
prove the throughput.

## OSD without dedicated hardware

Simple analog OSD can be added after live video works.

Once sync timing is recovered, selected active-line CVBS samples can be
replaced with black/white levels according to a tiny character mask:

```text
CVBS stream
   |
   +--> horizontal/vertical timing
   +--> character mask
   |
   v
modified CVBS
```

This allows channel, RSSI or status text without decoding the whole frame and
without an external OSD chip.

## One-pin video is research only

A future experiment may serialize a noise-shaped one-bit video stream at a high
clock rate and reconstruct it with a passive low-pass network.

Potential benefit:

```text
one output GPIO + very few passives
```

But it introduces quantization-noise, filtering, EMI and chroma-quality risks.
Saving five GPIOs and a few resistors is not worth making this the default before
it has been measured against the six-bit reference output.

## Development split

The project has two independent halves that should be proven separately.

### Output half

```text
synthetic CVBS in RAM
 -> GDMA/PARLIO
 -> six-bit resistor DAC
 -> scope
 -> real monitor
```

### RF half

```text
5.8 GHz VTX
 -> C5 RF chain
 -> finite/live phase-bearing samples
 -> host or C5 WBFM
 -> recognizable CVBS structure
```

Only after both halves work independently should they be connected into a live
receiver.

## Immediate hardware test sequence

1. Generate a complete PAL test frame in firmware.
2. Build the six-bit resistor network on the DevKit header.
3. Terminate it into a real 75-ohm video input.
4. Scope sync, blank, black and white levels.
5. Verify a stable monochrome test pattern.
6. Verify color bars/subcarrier reproduction.
7. Repeat at 5 and 4 bits to quantify the actual minimum useful bit depth.
8. Decide whether a buffer adds visible or measurable value.

## Immediate RF test sequence

1. Use A4 / 5805 MHz so no arbitrary retune is required.
2. Capture VTX off, static VTX and changing-image VTX cases.
3. Prove the dump changes with the analog source.
4. Recover WBFM baseband on the host.
5. Detect valid PAL/NTSC line structure.
6. Characterize actual sample rate and bandwidth.
7. Reverse engineer the producer behind the finite dump RAM for continuous
   capture.
8. Move the discriminator into the C5 only after the signal path is proven.

## Final target

The ideal C5VRX core is conceptually:

```text
antenna -> ESP32-C5 -> passive resistor DAC -> AV out
```

Everything else must earn its place on the BOM through measurement.
