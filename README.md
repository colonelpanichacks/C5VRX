# OUI-SPY // C5VRX

**A 5.8 GHz analog FPV drone detector and foxhunter with live video preview,
running on the Seeed Studio XIAO ESP32-C5.**

> **STATUS: very beta — proof of concept.** It works end-to-end (scan, lock,
> proximity beeps, live video, detection logging), but expect rough edges and
> breaking changes. Active development continues.

OUI-SPY sweeps all 48 standard FPV channels, locks onto active video
transmitters, beeps faster as you get closer, and streams a live 224x168
grayscale video preview over USB to a phone-friendly Flask dashboard.

---

## What it does

- **Scan** — hops all 48 FPV channels (RaceBand, Boscam A/B/E, FatShark,
  LowBand) with a 200 ms dwell, parking dead channels at a 50 ms quick dwell
  (re-checked at full dwell every 4th sweep so nothing is missed).
- **Lock & foxhunt** — on carrier acquisition the receiver locks, sounds an
  instant lock jingle, and drives the buzzer in a proximity cadence: beeps
  that speed up and rise in pitch as signal strength grows, a steady tone
  when the transmitter is right on top of you. The onboard LED shows
  heartbeat / scanning / locked state.
- **Live video preview** — a composite-video frame grabber demodulates the
  locked (or nearby) carrier; 224x168 grayscale frames stream over the USB
  Serial/JTAG console as row packets and are served by the Flask dashboard
  as MJPEG, viewable from any browser on the LAN — phone included. The feed
  is always on: real video when locked, live channel static when not.
- **CSV detection log** — every detection episode (channel, band, frequency,
  level stats, duration, video-sync flags) is appended to
  `tools/detections.csv`; the dashboard's EXPORT CSV button downloads it.

## Hardware

**Get the OUI-SPY PCB at [colonelpanic.tech](https://colonelpanic.tech)** —
it carries the XIAO ESP32-C5, routes the 5.8 GHz antenna, and breaks out the
buzzer pad. Bill of materials:

- **Seeed Studio XIAO ESP32-C5** — ESP32-C5, RISC-V, 5 GHz Wi-Fi PHY used as
  the RF front-end (the Wi-Fi radio is a 5.8 GHz receiver in disguise).
- **3V passive buzzer** — the foxhunt proximity beeper (driven by LEDC PWM).
- The OUI-SPY PCB (antenna matching + carrier for the XIAO + buzzer).
- Any 5.8 GHz FPV antenna (RP-SMA/SMA pigtail).
- **U.FL (IPEX-1) to SMA or RP-SMA female bulkhead pigtail** — joins the
  XIAO's U.FL RF pad to the antenna bulkhead on the OUI-SPY PCB.

- **OUI-SPY board add-ons:**
  | Function | GPIO | XIAO pad |
  |:---|:---:|:---:|
  | Buzzer (foxhunt proximity) | GPIO25 | D2 |
  | Status LED (onboard, yellow) | GPIO27 | onboard |
  | MODEM_DIAG Q[8] IQ lane | GPIO2 | MTMS (back pad) |

  Q[8] was moved from its stock GPIO25 to GPIO2 so the buzzer keeps the D2
  carrier-board pad. The lane is GPIO-matrix routed and never leaves the
  chip, so any unconnected pad works.
- **Analog video out (optional, from upstream C5VRX):** 6-bit resistor DAC
  ladder on D4..D9 (8.2k / 3.9k / 2k / 1k / 470R / 240R), 200R shunt to
  ground, 470 pF de-emphasis capacitor — feeds standard 75-ohm FPV goggles
  or a monitor with composite video. The USB dashboard preview works without
  this.
- **BOOT button:** short click cycles channels, long press (>= 600 ms) opens
  the on-screen menu.

## Build

Docker (no toolchain install needed), from the repo root:

```bash
docker run --rm -v "${PWD}:/workspace" -w /workspace espressif/idf:v6.0.2 idf.py build
```

or natively with ESP-IDF v6.0.x: `. $IDF_PATH/export.sh && idf.py build`.

Then validate the architectural constraints (all checks must pass):

```bash
python tools/validate_build.py
```

Build products: `build/bootloader/bootloader.bin` (offset `0x2000`),
`build/partition_table/partition-table.bin` (offset `0x8000`),
`build/c5vrx3.bin` (offset `0x10000`).

## Flash — READ THIS FIRST

> **ALWAYS full-erase before flashing.**
> Incremental flashes leave stale PHY/calibration state in flash and the RF
> front-end will misbehave in ways that look like firmware bugs. Every flash
> session starts with `erase_flash`, then `write-flash`, then `verify_flash`.

With the XIAO in download mode (see below):

```bash
python -m esptool --chip esp32c5 -p PORT -b 460800 erase_flash
python -m esptool --chip esp32c5 -p PORT -b 460800 \
    --before usb-reset --after watchdog-reset write-flash \
    --flash-mode dio --flash-size 8MB --flash-freq 80m \
    0x2000  build/bootloader/bootloader.bin \
    0x8000  build/partition_table/partition-table.bin \
    0x10000 build/c5vrx3.bin
python -m esptool --chip esp32c5 -p PORT verify_flash \
    0x10000 build/c5vrx3.bin
```

(`PORT` is `COM10` on Windows, `/dev/ttyACM0` on Linux,
`/dev/cu.usbmodem*` on macOS. `tools/flash.py [PORT]` wraps the
write-flash step with auto port detection — but it does **not** erase, so
run `erase_flash` yourself first.)

### The BOOT dance (download mode)

The XIAO has no onboard reset button, so entering download mode is a
two-handed move:

1. Hold the **BOOT** button.
2. Short the **RST** pads (or plug the USB cable in) — the jumper/reset tap.
3. Release **BOOT**.
4. Run the esptool commands; `--after watchdog-reset` reboots straight into
   the app when done.

`python tools/auto_flash.py` watches for a board in download mode and
write-flashes it automatically (again: erase first, by hand).

## Dashboard

```bash
pip install pyserial flask pillow
python3 tools/flask_app.py [serial_port] [--host 0.0.0.0] [--port 5000]
```

Then open `http://<machine-ip>:5000` from any browser on the LAN. The
dashboard shows the MJPEG preview, an auto-ranging 60 s level-history graph,
carrier/Q_phase/gain meters, a 48-channel direct-tune grid, scan/audio/skip
controls, and the CSV export button. Console hotkeys on the USB serial
port (`c` channel, `g` scan, `u` audio, `k` skip-dead, `+`/`-` gain,
`d` diagnostics, ...) work alongside it.

## Repository layout

```text
├── main/                  # Production firmware (ESP-IDF)
│   ├── main.c             #   Application entry
│   ├── rf.c / rf.h        #   Wi-Fi PHY RX frontend, channel/frequency tuning
│   ├── video.c / video.h  #   PARLIO RX/TX, Zero-EOF GDMA, AGC, scanner, OSD
│   ├── grab.c / grab.h    #   Composite video frame grabber (Konrad's code)
│   ├── preview.c/.h       #   224x168 USB row-streaming preview task
│   ├── buzzer.c/.h        #   Foxhunt jingles + proximity cadence
│   ├── status_led.c/.h    #   Onboard LED status
│   ├── fm.bsasm           #   BitScrambler WBFM demodulator program
│   └── range_control.h / demod_quality.h   # Range controller (benched, observation-only)
├── tools/
│   ├── flask_app.py       # The dashboard (MJPEG + HUD + CSV export)
│   ├── validate_build.py  # Architectural constraint validator
│   ├── flash.py           # write-flash wrapper (erase first — see above!)
│   ├── auto_flash.py      # Download-mode watcher / auto flasher
│   └── monitor.py         # Low-latency interactive serial console
├── docs/                  # Hardware-proven findings, knowledge index
├── legacy/                # Historical C5VRX-1/C5VRX-2 trees (reference only)
└── test/host/             # Host-side test harnesses
```

## Credits

- **C5VRX**, the original project, by **Twotoz**:
  <https://github.com/Twotoz/C5VRX> — the ESP32-C5 analog FPV receiver
  architecture (Zero-EOF circular GDMA, BitScrambler WBFM demodulation,
  PARLIO composite-video pipeline, AGC) and the range/demod-quality work
  ported into this firmware.
- This fork descends from **KonradIT's fork**:
  <https://github.com/KonradIT/C5VRX> — specifically the composite video
  frame grabber in `main/grab.c`, taken from Konrad's `tembed-link` branch.
- Foxhunt UX (buzzer, LED, scanner, dashboard) by Colonel Panic
  (<https://github.com/colonelpanichacks>).

## License

GPL-3.0-only, carried forward from the upstream C5VRX lineage — see
[LICENSE](LICENSE). Note the C5VRX name/logo branding exception in
[assets/BRANDING.md](assets/BRANDING.md).
