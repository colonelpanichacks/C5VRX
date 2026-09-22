# DEVELOPMENT.md — maintainer notes

This file is the session-grounding note for AI-assisted work in this repo. It does not
replace [AGENTS.md](AGENTS.md) — AGENTS.md's realtime invariants are settled
and still apply; do not violate them.

## What this repo is

OUI-SPY // C5VRX: a 5.8 GHz analog FPV drone detector/foxhunter with live
224x168 video preview, on the Seeed XIAO ESP32-C5. Descends from
KonradIT/C5VRX, which descends from Twotoz/C5VRX (the original C5VRX).

## Repository layout

- `main/` — production firmware (ESP-IDF, target esp32c5):
  - `main.c` entry; `rf.c/rf.h` Wi-Fi PHY RX frontend + tuning;
  - `video.c/video.h` PARLIO RX/TX, Zero-EOF GDMA, classic AGC, scanner;
  - `grab.c/grab.h` composite frame grabber (Konrad's code, see Credits);
  - `preview.c/preview.h` USB row-streaming preview task (224x168 GRAY8_ROWS);
  - `buzzer.c/buzzer.h` foxhunt jingles + proximity cadence;
  - `status_led.c/.h` onboard LED; `fm.bsasm` BitScrambler demod;
  - `range_control.h` / `demod_quality.h` — ported range controller
    (currently BENCHED, observation-only — see decisions below).
- `tools/flask_app.py` — the Flask dashboard (MJPEG preview, meters,
  direct-tune grid, scan controls, CSV export). Imports
  `c5vrx_usb_protocol` from `legacy/c5vrx1/tools/` via `sys.path.insert` —
  that module has NOT been promoted into `tools/`; keep the path shim.
- `tools/validate_build.py` — architectural constraint validator; run after
  any build-touching change, all checks must pass.
- `tools/flash.py` — write-flash wrapper (auto port detect, offsets
  0x2000/0x8000/0x10000). See the erase-first rule below.
- `tools/auto_flash.py` — download-mode watcher; `tools/monitor.py` —
  interactive serial console.
- `tools/detections.csv` — runtime detection log, gitignored, never commit.
- `docs/` — hardware-proven findings; start at `docs/KNOWLEDGE_INDEX.md`.
- `legacy/` — historical C5VRX-1/2 trees; search before reinventing RF/PHY,
  PARLIO, or video architecture (per AGENTS.md), and host of the USB
  protocol module the dashboard imports.

## Git remotes

- `origin` = https://github.com/colonelpanichacks/C5VRX.git (this fork)
- `upstream` = https://github.com/Twotoz/C5VRX.git (original project)
- `konrad` = https://github.com/KonradIT/C5VRX.git (intermediate fork;
  `main/grab.c` came from its `tembed-link` branch)

## Hardware facts (OUI-SPY board, XIAO ESP32-C5)

- Buzzer on GPIO25 (XIAO D2 pad) — foxhunt proximity beeps.
- Onboard user LED on GPIO27 — heartbeat / scanning / locked.
- MODEM_DIAG Q[8] IQ lane moved from stock GPIO25 to GPIO2 (MTMS back pad)
  to free the buzzer pad; GPIO-matrix routed, any unconnected pad works.
- Optional upstream analog video out: DAC ladder D4..D9
  (8.2k/3.9k/2k/1k/470R/240R) + 200R shunt + 470pF de-emphasis.
- BOOT button (GPIO28): short click = channel cycle, long press = OSD menu.

## Hard-won rules

- **ALWAYS full-erase before flashing.** Stale PHY/calibration state from
  incremental flashes causes RF misbehavior that masquerades as firmware
  bugs. Sequence: `esptool erase_flash` → `write-flash` → `verify_flash`
  (see README for exact commands; `tools/flash.py` does NOT erase).
- Download mode: hold BOOT, tap RST (or plug USB), release BOOT.
- `python tools/validate_build.py` must pass before flashing.

## Settled behavioral decisions (do not re-litigate without new evidence)

- Scanner: flat 200 ms dwell on every channel; skip-dead parked channels
  get a 50 ms quick dwell, re-checked at full dwell every 4th sweep.
  A resumed scan always begins a fresh dwell.
- Classic AGC in `video.c` is the sole gain authority. The ported range
  controller (`range_control.h`) is BENCHED observation-only — it logs and
  scores but its gain writes are vetoed.
- While legacy-locked, RF writes freeze: no gain/BW/offset changes.
- Lock acquisition plays an instant lock jingle (no settle delay), followed
  by carrier-offset channel verification.
- Preview is always on: real video when locked, continuous channel static
  (snow rows) when unlocked — never a blank pane.
- USB preview pacing: 15 fps frame-level cap (row-streamed packets). 40 fps
  saturated USB-CDC and truncated frames mid-write; do not raise without
  new USB evidence. Frames drop whole, never mid-frame.
