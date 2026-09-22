# C5VRX knowledge index

Use this page to find evidence before changing the RF or video dataplane.
Current physical results override historical assumptions; negative experiments
remain valuable because they prevent repeated dead ends.

## Project lineage

```text
Original C5VRX
  -> RF/PHY reverse engineering
  -> receiver-console and finite IQ experiments
  -> AV/DAC, USB, LP-core, REGDMA, and restart diagnostics
  -> direct recovered-CVBS architecture
        |
        v
Modern C5VRX
  -> one-start pre-trigger writer proof
  -> live MODEM_DIAG Q4/I4 mapping
  -> PARLIO RX at 40 MS/s
  -> TX BitScrambler WBFM and 20 MS/s DAC
  -> first locked live NTSC
  -> phase5 color/static improvements
  -> current DMA-boundary and image-quality work
```

## Current authority

| Subject | Start here |
|---|---|
| Proven RF writer, SRAM visibility, MODEM_DIAG mapping, rates | [continuous-iq-findings.md](continuous-iq-findings.md) |
| Realtime contracts and source abstraction | [realtime-iq-plan.md](realtime-iq-plan.md) |
| Current image-quality path and proof gates | [image-quality.md](image-quality.md) |
| Corrected issue #6 winding/static measurements | [issue-6-static-analysis.md](issue-6-static-analysis.md) |
| Static root causes, digital filtering limits, analog capacitor de-emphasis | [static-reduction-and-filtering.md](static-reduction-and-filtering.md) |
| Issue #11 20 MS/s digital CVBS stream measurement and timing proof | [issue-11-cvbs-analysis.md](issue-11-cvbs-analysis.md) |
| Fix for CVBS horizontal line jitter, trajectory wrap, and static | [fix-cvbs-jitter-and-static.md](fix-cvbs-jitter-and-static.md) |
| Diagnostic LED firmware, empirical findings, 9-line raster beat, and 40 MS/s DAC | [diagnostic-led-firmware.md](diagnostic-led-firmware.md) |
| True 40 MS/s DAC reconstruction and 80 MS/s decoupled rate expansion roadmap | [issue-11-cvbs-analysis.md](issue-11-cvbs-analysis.md) |
| Issue #17 True 40 MS/s cadence, adjacent25 failure modes, and interleaved demodulation | [issue-17-true40-cadence-and-interleaved-phase5.md](issue-17-true40-cadence-and-interleaved-phase5.md) |
| Dual-loop self-calibrating AGC, FM phase coherence ($Q_{\text{phase}}$), and noise trap immunity | [dual-loop-adaptive-gain-optimizer.md](dual-loop-adaptive-gain-optimizer.md) |
| C5VRX-3 independent PAL/NTSC menu, waveform tests, decoder hypotheses and remaining hardware validation | [c5vrx3-menu-and-raster-architecture.md](c5vrx3-menu-and-raster-architecture.md) |
| Wiring and remaining physical tests | [hardware-test.md](hardware-test.md) |
| Accepted historical donor primitives | [proven-donors.md](proven-donors.md) |
| Licensing and contributor evidence | [licensing.md](licensing.md) |

For exact behavior, read `/main` together with these documents. A document may
describe a diagnostic mode or future proof gate rather than the default build.

## Historical evidence

- [Original repository README](../legacy/c5vrx1/README.md) — preserved visual
  and project history; its old status claims are not current.
- [Archive research](../legacy/c5vrx1/research/) — RF dump format, PHY reverse
  engineering, CVBS/DAC work, USB receiver-console sessions, and architecture
  experiments.
- [Archive diagnostics](../legacy/c5vrx1/diagnostics/) — bounded hardware probes
  and reproduction projects.
- [Archive firmware profiles](../legacy/c5vrx1/firmware_profiles/) — historical
  build configurations only.
- [Archive hardware](../legacy/c5vrx1/hardware/) — early BOM and board research;
  verify pin/resistor values against current docs.
- [Preserved GitHub issues](legacy-issues/README.md) — all archive issues and
  their later disposition.
- [High-signal archive PR discussions](legacy-issues/pull-requests.md).

## Search guide

- **ESP32-C5 RF / PHY / dump RAM:** current continuous-IQ findings, then archive
  `research/adc-dump-format.md` and `research/reverse-engineering.md`.
- **MODEM_DIAG / Q4-I4:** current continuous-IQ findings and `/main` mappings;
  archive material predates the final live-source proof.
- **WBFM / adjacent phase / filtering:** current image-quality and realtime
  docs, then legacy issues #27 and #28.
- **CVBS / PAL / NTSC / DAC:** current hardware test, then archive analog-first,
  CVBS proof, and resistor-model material.
- **PARLIO / GDMA / BitScrambler:** current source/findings, then legacy issues
  #20, #22, #24 and the preserved PR notes.
- **Continuous capture:** distinguish writer-pointer continuity, readable sample
  continuity, and coherent RF-time continuity. They are separate claims.
- **Old failed approaches:** start with legacy issue/PR dispositions before
  reviving finite rearm, active dump-SRAM reads, RX-attached BitScrambler, or a
  synthetic raster as the normal receiver.

