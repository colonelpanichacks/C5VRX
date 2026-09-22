# A4 / 5805 MHz first-hardware test

This is the legacy finite-capture test gate. For the current producer/ring,
cadence, wrap, fine-tune, soak and performance suite, use
[`first-hardware-test.md`](first-hardware-test.md).

## Equipment

- ESP32-C5 board supported by the proof image;
- data-capable native USB cable;
- analog VTX set to A4 / 5805 MHz, antenna or dummy load attached;
- static video test image;
- for the separate CVBS test: 75-ohm terminated monitor or oscilloscope and the
  resistor DAC from `research/devkit-cvbs-proof.md`.

A4 is intentional: 5805 MHz is an exact supported Wi-Fi center, so this test
does not depend on the undocumented direct-frequency hook.

## Flash and run the RF proof

Download `c5vrx-proof-firmware` or `C5VRX-Flasher-Windows` from the latest PR
Actions run. Flash the proof firmware, reconnect native USB, and find the new
serial port.

On a machine with Python 3:

```bash
python -m pip install pyserial
python tools/c5vrx_bench.py --port /dev/ttyACM0
```

Use `COM5`-style names on Windows. The runner pauses before the VTX-off and
VTX-on captures and writes `c5vrx-bench-report.json`. Keep `--yes` for automated
fixtures only; it removes the two safety/experiment prompts.

The sequence is fixed:

1. `PING` and `STATUS`;
2. synthetic `WBFM HWTEST` on the physical BitScrambler;
3. raw 16,384-word capture with VTX off;
4. raw 16,384-word capture with VTX on at A4;
5. real finite RF dump through the hardware WBFM transform;
6. 32 chained finite blocks for continuity diagnostics.

## Automated pass gate

`bench_passed` is true only when:

- every command reaches its expected completion marker;
- every hardware return code is zero;
- both raw captures contain exactly 16,384 words;
- the VTX-off and VTX-on capture SHA-256 hashes differ.

This is a transport/execution gate. A changed capture is necessary but does not
by itself prove that the words contain valid 5.8 GHz phase-bearing I/Q.

## Engineering interpretation

Archive the JSON report even when the automated gate passes. Review:

- `WBFM HWTEST`: must return `code=0`;
- VTX-off/on capture uniqueness and hashes;
- WBFM output range, mean and mean absolute deviation in the captured log;
- `repeated_hashes`: zero is expected for a changing live source;
- `boundary_jump_power`: compare against ordinary adjacent-sample jump power;
  the absolute value alone has no honest universal threshold yet.

Change the VTX image brightness and repeat the on-capture/WBFM experiment. A
repeatable baseband response and line-period structure are required before the
finite dump can be accepted as useful analog-FPV input.

## Separate PAL output gate

Flash `c5vrx-pal-cvbs-proof-firmware` and follow
`research/devkit-cvbs-proof.md`. Required observations into 75 ohms:

- 625/50 raster and approximately 15.625 kHz line rate;
- approximately 1 Vpp composite level after selecting the final resistor set;
- stable sync and grayscale bars on the monitor;
- no unsafe GPIO voltage or excessive resistor/board temperature.

Do not connect the passive DAC to an unterminated input and interpret its open-
circuit voltage as the final CVBS level.

## What unlocks the next implementation

Return the JSON report plus scope screenshots/capture data. The guarded producer
and ring pipeline are already implemented; physical evidence determines which
capability gates can be enabled. Neither result should be hidden with guessed
video conditioning.
