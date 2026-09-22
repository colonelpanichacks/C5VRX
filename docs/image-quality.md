# WBFM image-quality path

The physically proven baseline from PR2 is intentionally retained as a
fallback. It reduces the proven Q4/I4 byte to asymmetric Q3/I2 Cartesian
states, performs a direct 32 x 32 phase-difference lookup, and sustains the
required 20 MS/s output. On 2026-09-09 that path produced stable, recognizable
live NTSC video, but with visible static.

## Uniform phase5 core

The quality path uses the complete Q4/I4 input byte and a dual-purpose
1024 x 16-bit LUT:

```text
raw Q4/I4 byte
 -> atan2-derived uniform five-bit phase
 -> circular delta from the previous retained phase
 -> calibrated gain, pedestal and six-bit clamp
 -> PARLIO TX at 20 MS/s
```

The LUT stores raw-byte phase in bits 12..8 and the 32 x 32 discriminator
result in bits 5..0. This allows both lookups to coexist without another
buffer or another DMA stage. Persistent BitScrambler state carries the prior
phase across ordinary DMA and ring boundaries.

The host model exhaustively checks all 1024 LUT addresses and the circular
phase wrap. Across all useful Q4 amplitude states, uniform phase5 reduces the
absolute phase quantization error from 9.52 degrees RMS / 25.63 degrees max
for Q3/I2 to 3.27 degrees RMS / 5.62 degrees max.

## Reproducible builds

Live quality firmware:

```powershell
docker run --rm -v "${PWD}:/project" -w /project espressif/idf:v6.0.1 `
  idf.py -B build-quality-live `
  -D SDKCONFIG=/project/build-configs/sdkconfig.quality `
  -D SDKCONFIG_DEFAULTS=/project/sdkconfig.quality.defaults build
```

Bounded transport oracle (no RF):

```powershell
docker run --rm -v "${PWD}:/project" -w /project espressif/idf:v6.0.1 `
  idf.py -B build-quality-test `
  -D SDKCONFIG=/project/build-configs/sdkconfig.quality-test `
  -D SDKCONFIG_DEFAULTS=/project/sdkconfig.quality-test.defaults build
```

The oracle sends deterministic Q4/I4 input through the exact TX BitScrambler
at the production 40-to-20 MS/s ratio and captures the low four DAC bits
through PARLIO RX. The program primes once and then executes two bundles per
output. Its persisted result must match the C reference byte-for-byte before
the quality core is called physically proven.

## Proof status

Software and bounded-hardware proven:

- full-Q4/I4 to uniform phase5 mapping;
- all 1024 dual-purpose LUT entries;
- circular delta including the 31-to-0 wrap;
- both LIVE and bounded diagnostic ESP-IDF 6.0.1 builds;
- embedded 16-bit LUT high-lane oracle: 4000/4000 exact at 20 MS/s;
- complete two-bundle phase5 core: 4000/4000 exact, zero offset and no
  PARLIO RX/TX error at 20 MS/s;
- live NTSC remains locked and recognizable, shows substantial colour, and
  has visibly less static than the Q3/I2 PR2 baseline.

The hardware oracle also found a C5/IDF 6.0.1 lifecycle trap: loading the
dual-purpose LUT before the PARLIO TX transaction did not give the active run
the intended table. Embedding the LUT in the BitScrambler program makes the
instruction and LUT load atomic and produced byte-exact hardware output.
Direct register LUT “readback” returned only zeroes and is not treated as a
valid proof mechanism on this target.

Still requiring hardware proof or tuning:

- reduce the remaining static and grey cast without losing NTSC lock/colour;
- no hidden discontinuity at long-running RX/TX ring boundaries.

## Frozen raw-Q4 measurement

The raw snapshot diagnostic stops PARLIO RX before copying its 16 KiB DMA
ring. This is essential: copying while the 40 MB/s writer is active creates a
torn buffer and fabricates exactly the discontinuities being investigated.
The diagnostic intentionally bypasses WBFM and DAC, freezes after about one
second, copies the stable ring, and persists it in `diagcap`. The production
path is unchanged.

Analyze a readback with:

```powershell
python tools/analyze_live_q4.py build-raw-q4/raw-vtx-on.bin
```

The first VTX-on capture on 2026-09-09 established:

- carrier/DC offset was small: +0.071 MHz on even samples and +0.198 MHz on
  odd samples;
- about 10.4--11.0% of the production `n -> n+2` phase5 deltas were within
  two phase codes of +/-pi;
- reconstructing each `n -> n+2` step as the sum of its two underlying 40M
  adjacent phase5 steps found 8.78--9.41% outside the unambiguous signed
  five-bit interval;
- this ambiguity is amplitude-dependent: with all three Q4 samples at
  amplitude-squared >= 64 it fell to 0.12--0.50%;
- the current gain/clamp table emits only 13 distinct DAC codes:
  `0, 2, 8, 14, 20, 26, 32, 38, 44, 50, 56, 62, 63`;
- the three observed 4096-byte internal block crossings had deltas
  `[7, 0, -10]` for even parity and `[1, -10, 0]` for odd parity. This one
  bounded capture contains no uniquely saturated internal block crossing,
  but it does not yet prove long-running simultaneous RX/TX boundary safety.

Therefore the remaining static is not primarily explained by carrier/DC
offset. The strongest measured issue is ambiguity/noise in the two-interval
phase discriminator, amplified by coarse 13-level output quantization. Do
not add unconditional gap filling: first correlate any visible line with a
proven transport discontinuity. Filtering/de-emphasis remains a valid later
quality step after discriminator ambiguity is reduced.

### Centroid-output A/B

Keeping the same 32 phase5 states but evaluating their actual Q4 cluster
centroids at phase8 precision expanded the pair LUT from 13 to 34 DAC codes
without adding an instruction bundle. Host validation reduced phase error
slightly from 3.27 to 3.25 degrees RMS. The first live A/B remained locked
and no longer visibly rolled during the short observation, but the user saw
no material reduction in static or grey cast. More output levels alone are
therefore not the primary static fix; the measured unreliable low-amplitude
phase transitions must be addressed next.

Set `CONFIG_C5VRX2_WBFM_PHASE5_QUALITY=n` to restore the known-working Q3/I2
baseline while preserving the rest of the direct RX-ring-to-TX architecture.

### Gain calibration — live findings (2026-09-10)

First live test after the centroid/invalid-state commit confirmed:

- image locks and displays recognisable video with colour;
- gain=1 (effective 0.75×) is **too low**: sync tip only reaches ≈ code 1
  instead of code 0, and the colour burst amplitude falls below the TV
  chroma-detector threshold. The TV runs its colour decoder in free-running
  mode and interprets high-frequency FM noise as colour → **coloured static**.
- gain=2 (effective 1.5×) is the correct production default: sync tip
  reaches code 0, colour burst is above the detection threshold, and the
  image shows correct hues.
- The general static (snow) is not improved by changing gain alone. It is
  FM f²-noise — noise power proportional to frequency squared — concentrated
  in the 3–5 MHz band that overlaps the chroma subcarrier. This manifests as
  coloured snow independent of discriminator gain.

### Remaining static — root cause

FM f²-noise is inherent to every FM discriminator without de-emphasis. The
standard fix is a de-emphasis filter matched to the transmitter's pre-emphasis
curve (75 µs for broadcast, but most FPV VTXes transmit without pre-emphasis,
so a plain low-pass at ≈ 4.5 MHz is the correct target).

The current two-bundle BitScrambler pipeline has no spare instruction slot
for a filter tap. Implementing a 2-tap boxcar or first-order IIR requires a
**three-bundle redesign**:

1. Lookup phase for pair n → store in O-register.
2. Lookup phase for pair n+2 → compute delta and store raw code.
3. Average with previous raw code → clamp → emit.

This maintains 20 MS/s output and gives ≈ 6 dB high-frequency noise
reduction. That is the next planned quality step.
