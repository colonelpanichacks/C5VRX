# First ESP32-C5 hardware procedure

This is the exact evidence-gathering sequence for the first board. It does not
enable production `LIVE START` unless all gates are measured.

## Connections

1. Flash the RF-deep-probe/Receiver Console firmware built from the pinned
   ESP-IDF v6.0.2 profile. For a Seeed Studio XIAO ESP32-C5, use only
   `c5vrx-xiao-receiver-console-firmware` or
   `C5VRX-XIAO-Receiver-Console-Windows`; verify that `STATUS` reports
   `profile=xiao-esp32c5` before connecting the AV resistor network.
2. Connect USB-C and the six-bit passive DAC to a high-impedance scope first;
   connect the HDZero 75-ohm AV input only after the CVBS test levels are safe.
3. Select a receiver center inside the C5 specified range. A4/5805 MHz is the
   simplest exact baseline.
4. Feed a coherent, attenuated tone at center +2 MHz (A4: 5807 MHz). Do not
   attach an unattenuated VTX or signal generator to the antenna input.

## One-button suite

For a command-line run with automatic USB-port selection, live output,
machine-readable PASS/FAIL and complete session artifacts, use:

```bash
python tools/c5vrx_lab.py auto-test
```

The legacy `c5vrx_bench.py` finite-capture runner remains available. The
session-oriented runner executes the current cadence/phase/soak/performance
suite described below, then appends the legacy finite IQ, WBFM and chain
diagnostics. See [`codex-hardware-lab.md`](codex-hardware-lab.md) for artifact
layout, exit codes and the separate bounded `vtx-proof` command.

Open `C5VRX Receiver Console`, connect, and press **FIRST HARDWARE TEST**. It
queues, in order:

```text
STATUS
WBFM HWTEST
PRODUCER CADENCE PROBE ALL
WRAP FLAG PROBE 0
PHASE CONTINUITY PROBE 0
PRODUCER SOAK 0 30000
BENCH SPARSE 2
BENCH SPARSE 4
BENCH SPARSE 8
BENCH BITSCRAMBLER
BENCH PARLIO
BENCH PIPELINE
BENCH USB PREVIEW
BENCH RING PIPELINE 0 1000 1024
BENCH RING PIPELINE 0 1000 2048
BENCH RING PIPELINE 0 1000 4096
USB PREVIEW STOP
CAPABILITIES
STATUS
```

See `research/issue5-stage1-ring-proof.md` for the monotonic pointer,
adjacent-memory canary, deadline-headroom and dedicated-radio acceptance rules.

When an unambiguous mode-0 cadence arrives, the console automatically appends
`FINE TUNE VERIFY <center> <center+2> <measured_rate>` so tone offsets are
reported before the reconstructed producer, during it, and after teardown.

The 30-second soak command internally runs 1 ms, 10 ms, 100 ms, 1 s, 5 s and
30 s stages, stopping at the first invariant failure. No unlimited run exists
in the normal parser.

## Pass gates

- mode 0 has `classification=MEASURED`, no ambiguous cadence intervals, a
  changing pointer/RAM, and exact register restoration;
- bit 18 correlation is reported as observation, not silently named wrap;
- phase test uses the coherent tone and reports
  `classification=MEASURED_CONTINUOUS`; its boundary tolerance is derived from
  local phase noise but hard-capped at 0.25 rad;
- fine-tune offsets follow the expected +2 MHz baseline and do not revert
  during producer setup;
- every soak stage preserves enable/progression/state, heap, Wi-Fi liveness and
  teardown restoration;
- synthetic BitScrambler, sparse, PARLIO, pipeline and preview benchmarks have
  enough margin for the measured source rate;
- the bounded one-second real ring -> BitScrambler -> conditioner -> PARLIO
  benchmark has no overrun/drop/underrun and consumes at least 90% of the
  measured producer rate;
- an RF tone sweep separately establishes the tap bandwidth before any sparse
  factor is marked alias-safe.

For that sweep, step a constant-amplitude generator through positive and
negative offsets and run, for each point:

```text
TONE RESPONSE PROBE 0 <signed_offset_hz> <measured_rate_hz>
```

The device fills at least one complete ring, measures phase slope, coherence
and IQ magnitude, and emits one machine-readable `C5VRX_TONE_RESPONSE` record.
Repeat the sweep for mode 11 only when comparing the secondary trigger path.
Do not use mode 12: the complete vendor branch starts BLE RX and is not a
5.8 GHz dump candidate. The host
cannot safely retune an arbitrary external generator, so changing the generator
is the only manual step; capture and classification on the C5 are automated.

Until the bandwidth sweep is complete, `CAPABILITIES` retains
`source_bandwidth_known=0` and `LIVE START` fails closed. XIAO hardware showed
that `LIVE EXPERIMENTAL START 0` wedges native USB when the MAC dump engine
takes HP-SRAM ownership while FreeRTOS remains active. The command now returns
`reason=HP_SRAM_MAC_OWNERSHIP_WEDGES_FREERTOS_USB`. Use the Receiver Console's
safe bounded Phase8 preview; do not retry the continuous ring path.

Normal receiver firmware now starts PAL AV-out independently of RF. Before an
accepted sample block it shows the built-in C5VRX logo. A successful varying
finite capture changes the next complete frame to moving snow, because finite
blocks cannot prove gapless H/V lock. Later sample gaps deliberately retain
snow instead of flashing back to the logo. Confirm `cvbs=1 av_display=LOGO` at
boot and `av_display=SNOW` after the first successful capture.

After preserving the sweep data, explicitly import its result for the current
boot with:

```text
APPLY MEASURED BANDWIDTH <occupied_hz> <alias_safe_factor> CONFIRMED
```

This command is intentionally not a measurement: it is a guarded operator
assertion that the external RF sweep established both occupied bandwidth and
anti-alias filtering. It rejects unknown rates, factors other than 1/2/4/8 and
results that violate complex Nyquist. Retuning clears the imported result and
all RF-dependent capability gates. The implemented BitScrambler route is fixed
at /4, so it is selectable only when the confirmed factor is at least four.

After all gates pass, `LIVE START` automatically selects and starts the
implemented ring/BitScrambler route. It additionally requires the full
mode-0 staged soak, WBFM self-test, PARLIO benchmark and at least 20% synthetic
pipeline throughput margin, plus the one-second real-ring benchmark. Lower-tap
and sparse-CPU choices remain fail-closed
until their corresponding consumers are implemented and physically validated.
## Real-VTX evidence stage

The coherent-tone suite cannot prove video. After all production gates pass,
transmit a known PAL test card with a moving marker and run:

```text
USB PREVIEW START
LIVE START
CVBS LOCK STATUS
CVBS LOCK PROBE 5000
PIPELINE STATS
LIVE STOP
USB PREVIEW STOP
```

The Receiver Console exposes the five-second command as **Measure CVBS lock**.
Only `classification=MEASURED_CVBS_LOCK analog_vtx_usable_iq=1` upgrades the
VTX path from RF response to usable demodulated video. Repeat with the VTX off;
noise must not lock. Finally verify that the displayed marker actually moves
and save the preview plus complete log. Timing lock alone cannot prove visible
content, polarity or grey mapping.

`python tools/c5vrx_lab.py vtx-proof` guides and archives this A4 off/on stage.
It does not assert the externally measured bandwidth gate. If guarded live
preview is not yet eligible, it records that limitation and still preserves
the bounded IQ and WBFM evidence for inspection.

Disconnecting or stopping preview does not stop PARLIO AV. Stop the receiver
with `LIVE STOP` and preserve the complete console log, especially ring
overrun/drop statistics. The acceptance rationale and representative FPV
bandwidth calculation are in
[`real-rf-evidence-ladder.md`](real-rf-evidence-ladder.md).
