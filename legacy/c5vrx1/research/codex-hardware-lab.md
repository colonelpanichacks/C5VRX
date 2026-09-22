# USB-C hardware lab and Codex bundles

`tools/c5vrx_lab.py` is the automation entry point for a connected ESP32-C5.
It uses only the documented Receiver Console commands and does not write RF
registers, import bandwidth measurements, or bypass `LIVE START` capability
gates.

Install the one host dependency and run the existing first-hardware suite:

```bash
python -m pip install pyserial
python tools/c5vrx_lab.py auto-test
```

The port is selected automatically when there is one Espressif/C5 USB serial
device. If several candidates exist, the tool probes them with the read-only
`PING` command. `--port COM5` or `--port /dev/ttyACM0` remains available.
Device lines stream live to stderr. Stdout contains exactly one JSON result,
and the process exits zero only for PASS, 1 for a completed FAIL, or 2 for a
setup/transport error. This lets Codex run a build/flash/test loop and parse the
outcome without suppressing live diagnostics.

After the cadence/phase/soak/performance gates, `auto-test` also appends the
legacy finite IQ capture, physical WBFM capture, and 32-block chain diagnostic.
This leaves a directly inspectable IQ artifact even when the separate VTX
off/on workflow is not requested.

## Session artifacts

Every `auto-test` and `vtx-proof` invocation creates a timestamped folder below
`c5vrx-sessions/` before port detection. It is retained on PASS, FAIL,
disconnect, cancellation, and setup errors. Use `--sessions-dir` to put it
elsewhere. Each session contains:

- `raw-serial.bin`: exact received USB bytes, including binary preview packets;
- `raw-serial.log`: decoded serial lines plus transmitted commands;
- `results.json`: final machine PASS/FAIL result and clear reasons;
- `measurements.json`: every parsed `C5VRX_*` record and command;
- `firmware-git-version.json`: device app/IDF version and host Git identity;
- `board-profile.json` and `test-configuration.json`;
- packed IQ words plus decoded little-endian I/Q under `iq/`;
- CRC-valid GRAY8 preview frames as PGM files under `preview/`;
- `errors-failures.json` and the final `session.json` outcome.

A sibling `*-codex-bundle.zip` is created automatically. An existing folder
can be exported again with:

```bash
python tools/c5vrx_lab.py export c5vrx-sessions/<session-folder>
```

The Receiver Console records the same artifacts below
`Documents/C5VRX Sessions`. **EXPORT CODEX BUNDLE** writes one ZIP with the
current raw log, structured results, firmware/Git identity, profile,
configuration, IQ, preview frames, and errors. Missing capture types still
have an explicit empty index so consumers do not have to guess whether the
bundle is incomplete or corrupt.

## A4 VTX proof

Run the bounded off/on comparison separately:

```bash
python tools/c5vrx_lab.py vtx-proof
```

The operator is prompted to set the VTX off and then on at A4 / 5805 MHz with
an antenna or safe load attached. `--yes` is only for a fixture that controls
and independently verifies those two states; it does not control a VTX.

For each state the tool saves a finite IQ capture, runs the physical
BitScrambler WBFM capture, and derives scale-independent IQ, phase-discriminator
and 55--72 us line-period candidate metrics. It also tries the existing guarded
preview path by default. If `LIVE START` rejects missing cadence, phase,
bandwidth, soak, or performance evidence, the proof records preview as
unavailable. It never calls `APPLY MEASURED BANDWIDTH` automatically.

The proof passes only when both finite captures and hardware WBFM commands
complete and bounded RF/IQ, WBFM, and video-sync evidence changes. A different
SHA-256 alone is explicitly not RF proof. A finite line-period correlation is
also only a candidate: the stronger existing machine gate remains an off-to-on
transition ending in
`classification=MEASURED_CVBS_LOCK analog_vtx_usable_iq=1`. Visible moving
content still requires human confirmation.

`--preview-mode experimental` can gather explicitly labelled frames through
`LIVE EXPERIMENTAL START 0`; it does not convert that path into production
evidence. `--preview-mode none` keeps the comparison finite-capture-only.
