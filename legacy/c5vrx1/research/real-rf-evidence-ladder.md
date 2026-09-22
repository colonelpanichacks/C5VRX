# Real-RF evidence ladder

Status: pre-silicon implementation complete; all six physical conclusions below
remain `UNKNOWN` until the commands are run on an ESP32-C5 with controlled RF.

This document separates useful static research from measurements that cannot be
manufactured from source code. A changing RF RAM hash is evidence that something
changed. It is not evidence of phase-bearing IQ, decoded CVBS, or a visible FPV
image.

## The six questions

| Question | What is known now | Hardware acceptance evidence |
|---|---|---|
| Analog VTX -> usable IQ | The vendor-observed mode-0 packing is signed 10-bit Q + signed 10-bit I. A representative RTC6705 is a 5.8 GHz FM video transmitter, so complex phase is the relevant observable. The C5 tap's bandwidth and placement remain unknown. | A VTX test pattern must survive RF -> IQ -> WBFM and produce `C5VRX_CVBS_LOCK_PROBE ... analog_vtx_usable_iq=1 classification=MEASURED_CVBS_LOCK`. VTX-on/off hashes alone never pass this gate. |
| Continuity across ring wraps | The probe compares local complex phase increments with the `16383 -> 0` increment. No physical result exists yet. | With a coherent offset tone, mode 0 must report coherence >= 0.90, magnitude >= 8, and a boundary residual within `min(0.25, max(0.05, 5 * local_rms))` rad: `classification=MEASURED_CONTINUOUS`. |
| Actual sample rate | The old `sample_80m` field is statically proven overwritten before enable. It is not a rate control. | `PRODUCER CADENCE PROBE ALL` must report an unambiguous mode-0 modulo-pointer cadence. Multi-wrap intervals are rejected rather than assigned the smallest delta. |
| Throughput at that rate | The C5 bus matrix, GDMA and BitScrambler make concurrent hardware movement architecturally plausible. Public documentation does not specify sustainable RF-ring throughput. | `BENCH RING PIPELINE 0 1000` must sustain >=90% of measured cadence with zero ring overruns, fatal stops, dropped RF blocks and PARLIO underruns. Copy CPU >=25% requests zero-copy work; it does not silently certify the copied path. |
| CVBS lock from real RF | The adaptive detector recognizes narrow line-sync pulses, clusters broad vertical-sync pulses, tracks line period and drops stale lock. PAL/NTSC candidates are timing labels, not assumptions about the source. | While live RF and USB preview are running, `CVBS LOCK PROBE 5000` requires final H+V lock, at least two complete frames, 15--16.5 kHz line events and 40--70 Hz field events. |
| Visible live FPV image | The 160x120 GRAY8 packet path, console renderer and continuous AV path are implemented but not physically exercised. | First obtain machine CVBS lock without ring/PARLIO faults. Then observe the known moving test pattern in the Receiver Console and on the analog display, save a screenshot/photo, and record the log. Firmware deliberately prints `visible_image_machine_proven=0`: human-visible content cannot be inferred from timing alone. |

## Why a 20 MS/s consumer may work, but is not yet certified

ITU-R BT.470 gives a 625-line PAL line period of 64 us (15,625 Hz), a nominal
video bandwidth commonly 5 MHz, and horizontal sync width 4.7 +/- 0.2 us. At
the fixed 20 MS/s CVBS output this corresponds to:

```
line period       64 us * 20 MS/s = 1280 samples
horizontal sync  4.7 us * 20 MS/s =   94 samples
field interval   20 ms * 20 MS/s = 400000 samples
```

The RTC6715 receiver datasheet's sensitivity test uses +/-2.5 MHz FM deviation.
As a representative design estimate, not a universal VTX specification,
Carson's rule gives about `2 * (5 + 2.5) = 15 MHz` for video only. Including a
6.5 MHz audio subcarrier can move that estimate to roughly 18 MHz. Consequently
a properly band-limited 20 MS/s complex stream is plausible for representative
analog FPV, but has little guard band. The actual transmitter, audio subcarrier,
C5 source tap and FE filter must be swept before enabling sparse decimation.

Direct stride-N discrimination,

```
d[k] = angle(x[Nk] * conj(x[N(k-1)]))
```

preserves accumulated phase change but does not provide an anti-alias filter.
It is permitted only after `APPLY MEASURED BANDWIDTH ... CONFIRMED` imports an
actual occupied-bandwidth result consistent with the measured cadence.

## Exact evidence run

### 1. Calibrated coherent tone

Use attenuation and a level known to be safe for the RF input. Tune a coherent
tone a known small offset from the receiver center, then run:

```
PRODUCER CADENCE PROBE ALL
PHASE CONTINUITY PROBE 0
TONE RESPONSE PROBE 0 <signed_offset_hz> <measured_rate_hz>
FINE TUNE VERIFY <center_mhz> <tone_mhz> <measured_rate_hz>
```

Save the raw records. Do not run the phase test on noise or on arbitrary VTX
video: low coherence makes the boundary comparison non-diagnostic.

### 2. Real simultaneous throughput

Run the staged soak and synthetic component checks, then:

```
PRODUCER SOAK 0 30000
BENCH SPARSE 2
BENCH SPARSE 4
BENCH SPARSE 8
BENCH BITSCRAMBLER
BENCH PARLIO
BENCH PIPELINE
BENCH RING PIPELINE 0 1000
```

The final command is the important one: RF writer, guarded ring reader,
BitScrambler discriminator/conditioning and PARLIO are active together. The C5
technical reference manual describes a multi-layer bus matrix and weighted GDMA
arbitration, but neither statement is a measured MB/s promise. Keep the current
immutable copy unless this benchmark prints `zero_copy_action=IMPLEMENT_ZERO_COPY`.

### 3. Analog VTX and CVBS

Use a PAL test card first: black/white bars plus a moving marker make stale or
misaddressed frames obvious. Use the same channel, start preview and live mode,
then measure rather than judge the first noisy picture by eye:

```
USB PREVIEW START
LIVE START
CVBS LOCK STATUS
CVBS LOCK PROBE 5000
PIPELINE STATS
LIVE STOP
USB PREVIEW STOP
```

If the production gates are not yet populated, `LIVE START` correctly fails
closed. `LIVE EXPERIMENTAL START 0` is the explicit, unproven laboratory route;
its log must be labelled experimental. A lock result proves a usable modulation
path and CVBS timing. It does not by itself prove correct polarity, grey mapping,
chroma, or subjective image quality.

Repeat with VTX off. It must not produce a stable lock. A detector that locks on
noise is rejected. Then test a brightness-step pattern, moving geometry, loss
and reacquisition, weak signal, frequency offset, and any enabled audio
subcarrier. Record lock losses, frame drops, ring errors and PARLIO underruns.

## Stop conditions

Stop and retain the complete log on any of these:

- cadence ambiguity or pointer outside the 16,384-word ring;
- `MEASURED_DISCONTINUITY` on a coherent tone;
- a producer restore failure;
- ring overrun, fatal stop, dropped RF block or PARLIO underrun;
- stable CVBS lock with VTX off;
- repeated H/V lock loss on a clean, strongly received test card;
- preview content that does not follow the transmitted moving marker.

None of these failures authorizes experimental RF register values. They narrow
the next diagnosis while preserving the vendor-observed, fail-closed producer.

## Primary references

- Espressif, [ESP32-C5 Technical Reference Manual v1.1](https://documentation.espressif.com/esp32-c5_technical_reference_manual_en.pdf): bus matrix, GDMA and arbitration architecture.
- Espressif, [ESP32-C5 BitScrambler API](https://docs.espressif.com/projects/esp-idf/en/latest/esp32c5/api-reference/peripherals/bitscrambler.html): DMA-stream transforms and memory-buffer loopback.
- Espressif, [ESP32-C5 datasheet](https://documentation.espressif.com/esp32-c5_datasheet_en.html): CPU, SRAM, GDMA, BitScrambler and specified 5 GHz receive range.
- ITU-R, [Recommendation BT.470-5](https://www.itu.int/dms_pubrec/itu-r/rec/bt/R-REC-BT.470-5-199802-S%21%21PDF-E.pdf): 625-line timing, sync duration and representative analogue-video bandwidth.
- RichWave, [5.8 GHz video transmitter/receiver product family](https://richwave.bike.idv.tw/en/page_prd_new.aspx?Id=8): RTC6705/RTC6715 device roles.
- RichWave, [RTC6705 datasheet mirror](https://www.lcsc.com/datasheet/C913074.pdf): FM video input and optional audio subcarriers.
- RichWave, [RTC6715 datasheet mirror](https://www.alldatasheet.com/html-pdf/1995824/RICHWAVE/RTC6715/2263/5/RTC6715.html): representative receiver sensitivity test deviation and video output conditions.
