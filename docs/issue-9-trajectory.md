# Issue 9: two-bundle trajectory candidate

Based on main `8d0a8cb9d34642d4456c33eafd20d60e0586cbc1`.
This is an opt-in implementation, not a hardware-validated production fix.
No raw RF captures or attached ESP32-C5 were available in this environment.
The checked-in LUT is an exhaustive uniform Q4/I4 geometry prior. It is not
trained on the frozen capture mentioned in issue 9, which is absent from Git.

## Why this implementation

`docs/issue-6-static-analysis.md` reports 96.2% branch classification for the
phase4 + middle quadrant + phase4 representation and rejects the seven-level
analytic scaler. `main/c5vrx2_wbfm_q4_phase5_2to1.bsasm` supplies the proven
one-prime/two-bundle dual-purpose LUT pipeline. There is no new evidence for a
better architecture, so this change implements that representation directly.
It does not repeat PR5 invalid/pedestal substitution, PR8 node resizing, or the
older over-budget DSP cores described in `docs/continuous-iq-findings.md`.
Legacy RF/transport findings were checked against the current documentation.

The generator enumerates all 256^3 raw triples. It computes two separately
wrapped adjacent phase differences and DOES NOT wrap their sum. It applies
the production phase8 gain/round/pedestal/clamp before accumulating final
DAC targets by address. A branch with >=90% agreement and >=32 observations
uses the mean final target rounded to the nearest code (squared-error optimum).
Mixed addresses use the mean exact endpoint DAC result within that address;
this is a conservative compressed endpoint fallback, not the exact phase5
output for each individual sample. No phase code is reserved as invalid.
A compressed address cannot directly recover magnitude; confidence is an
address-level distribution estimate, not a per-sample low-IQ detector.

The uniform prior gives 644 trusted addresses and 28 distinct DAC codes.
That avoids the seven-level regression, but does not establish acceptable
video gradation or independence from a training distribution. The old phase5
LUT has 34 codes. Capture validation and live A/B must assess this tradeoff.

## Pipeline and host evidence

Each pair is `[middle,current]`. The phase-address bundle saves the middle
Q/I sign bits in O12/O13 BEFORE `read 16` advances the input window. The
trajectory-address bundle combines O8..O11 (previous), O12..O13 (middle), and
L8..L11 (current). The emit bundle writes only DAC bits 0..5, zeros bits 6..7,
and primes the next pair. State survives ring wraps; initialization happens
only at transaction startup. The first output uses reset phase4 zero, matching
the exact C reference. It is excluded from signal-quality statistics.

- Exactly three ROM instructions: one prime plus a two-instruction loop.
- Exactly one `read 16` and `write 8` per repeating loop: 40 MB/s to 20 MB/s.
- Embedded 1024-entry 16-bit LUT; no CPU DSP, M2M, new clock, RX-edge or ring change.
- ESP-IDF v6.0.1 C5 assembler accepted the program.
- Parsed assembly dataflow matches compiled C on 16,386 outputs.
- C reference state survives 16 KiB boundaries, tiny chunks, and limited output.
- All 16,777,216 raw triples match the generated address/DAC target in compiled C.
- Uniform-geometry quality: phase5 MAE 15.962, hard errors >=16 codes 25.033%;
  trajectory MAE 12.528, hard errors 21.800% (12.9% relative reduction).
- Existing phase5 and continuous-pipeline host checks pass.

These tests validate logic and instruction count, not peripheral timing,
FIFO behavior, bounded EOF, RF continuity, H-sync jitter, or live lock. The
uniform enumeration is the prior's construction distribution, NOT held-out
RF evidence. The host assembly model deliberately does not simulate EOF.

## Reproduce and train

```
python tools/train_trajectory_lut.py --write
python tools/validate_trajectory.py
python tools/validate_phase5_quality.py
python tools/validate_continuous_pipeline.py
```

Use independent whole v3 raw snapshots with declared 40/20 MHz rates:

```
python tools/train_trajectory_lut.py --train scene-a.bin scene-b.bin \
    --validate scene-c.bin scene-d.bin
```

Adding `--write` embeds the resulting table in both assembly and the C header.
Content-hash overlap is rejected, including renamed duplicate captures. Test
samples never enter training or fallback statistics. Unseen training states
retain the geometry prior. Review the reported held-out errors and DAC-level
coverage before adopting a trained table; the tool does not assert RF success.
Keep capture hashes and the report with the tested firmware revision.

## Hardware gates still required

CI builds the unchanged default and three candidate images: live, bounded
oracle low bank, and bounded oracle high bank. All use IDF v6.0.1.
For local builds use separate build directories and SDKCONFIG paths so a
previous config cannot silently override the defaults. Append
`sdkconfig.trajectory.defaults` to the usual live defaults. For oracle builds
also include `sdkconfig.tx-wbfm-test.defaults` before the trajectory defaults;
append `sdkconfig.trajectory-high.defaults` for the high bank.

The existing bounded direct-TX oracle only observes four data bits because
VALID occupies an RX lane. This change adds a high-bank diagnostic without
changing TX pins or its program. Record version 10 denotes trajectory low
bits 0..3, version 11 high bits 4..7. The full-byte loopback comparison runs
before either bank is masked. Two separately aligned deterministic captures
cover all DAC bits; they do not constitute a simultaneous eight-bit capture.
Compare all observable interior outputs, explicitly report startup/EOF and
alignment exclusions, and require both RX/TX ESP_OK and no FIFO errors.
Do not silently count the old oracle's single trailing mismatch as a pass.

After both banks and full-byte loopback pass, A/B phase5 vs trajectory at
40 MS/s IQ / 20 MS/s DAC using the same scene, edge, gain, and hardware.
Measure hard errors, H-sync leading-edge jitter, and video lock over time.
Use separate held-out captures to evaluate branch/error generalization.
Remaining fine snow alone is not a failure; unchanged layers despite better
demodulation should return to issue 6 diagnostics, not PR8 geometry.

The candidate's embedded LUT and C reference both use gain=2, pedestal=20,
current-minus-previous. Custom persisted calibration is not applied to this
fixed candidate table. Match those defaults during A/B.

Issue 9 must stay open until the independent-capture and physical gates pass.
