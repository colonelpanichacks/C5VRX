<div align="center">
  <img src="assets/c5vrx-logo.jpg" alt="C5VRX logo" width="760" />

  <p><strong>ESP32-C5 analog 5.8 GHz FPV receiver research</strong></p>
  <p>From live RF to recognizable composite video with one XIAO ESP32-C5 and a passive resistor DAC.</p>

  <p>
    <img src="https://img.shields.io/badge/status-live%20NTSC%20proven-success" alt="Live NTSC proven" />
    <img src="https://img.shields.io/badge/chip-ESP32--C5-111111" alt="ESP32-C5" />
    <img src="https://img.shields.io/badge/RF-5.8%20GHz-6f42c1" alt="5.8 GHz" />
    <img src="https://img.shields.io/badge/output-analog%20CVBS-orange" alt="Analog CVBS" />
    <img src="https://img.shields.io/badge/license-GPL--3.0--only-blue" alt="GPL-3.0-only" />
  </p>
</div>

---

## What is C5VRX?

C5VRX explores whether the ESP32-C5's 5 GHz RF chain and hardware dataplane can
act as a minimal analog FPV receiver. The live path recovers the composite
waveform already carried by the VTX; it does not decode frames and generate a
new PAL/NTSC signal.

```text
5.8 GHz analog FPV
        |
        v
ESP32-C5 RF / MODEM_DIAG
        |
        v
Q4/I4 -> PARLIO RX @ 40 MS/s
        |
        v
TX BitScrambler WBFM / 2:1 conversion
        |
        v
recovered CVBS @ 20 MS/s
        |
        v
PARLIO TX -> 6-bit resistor DAC -> 75-ohm goggles
```

The raw-Q4 elastic ring is the only realtime buffer. RX and TX use clocks
derived from the same 240 MHz PLL, USB is telemetry only, and software block
boundaries are not treated as RF signal boundaries.

## Hardware-proven status

On an ESP32-C5 revision v1.0 and A1/5865 MHz test setup:

- the dump-first/TX_START RF writer ran from one start through 10,000 observed
  physical wraps with zero software rearms or triggers;
- MODEM_DIAG mapping was measured as `DIAG[6:9] = Q[6:9]` and
  `DIAG[16:19] = I[6:9]`;
- PARLIO RX captured a bit-perfect sequence of every second approximately
  80 MS/s MODEM sample at 40 MS/s;
- continuous RX-ring -> TX-BitScrambler -> PARLIO-TX produced a stably locked,
  clearly recognizable live NTSC camera picture through the passive DAC;
- the newer full-Q4 phase5 path produced substantial color and less static
  than the first compact Q3/I2 proof.

C5VRX is still experimental. These results do **not** prove sample-gapless RF
time, indefinitely slip-free MODEM/PARLIO sampling, glitch-free cyclic DMA
boundaries, or production picture quality. Remaining static, grey cast, and
line displacement are active image-quality work.

Read [continuous IQ findings](docs/continuous-iq-findings.md),
[image-quality status](docs/image-quality.md), and the
[hardware test matrix](docs/hardware-test.md) before extending the dataplane.

## XIAO hardware

The tested output uses six XIAO pins, one resistor per branch, joined at the
`VIDEO` node:

| XIAO pin | GPIO | Series resistor |
|---|---:|---:|
| D4 | 23 | 8.2 kOhm |
| D5 | 24 | 3.9 kOhm |
| D6 | 11 | 2.0 kOhm |
| D7 | 12 | 1.0 kOhm |
| D8 | 8 | 470 Ohm |
| D9 | 9 | 240 Ohm |

Fit 200 Ohm from `VIDEO` to ground, share ground with the display, and use the
display's normal 75 Ohm termination. Do not connect a raw 3.3 V GPIO directly
to an AV input. See [hardware-test.md](docs/hardware-test.md) for expected
loaded levels and diagnostics.

## Build and flash

Release/test builds use ESP-IDF 6.0.1 and 40 MHz DIO flash. The tested 6.0.2
configuration caused an early MSPI/CPU lockup on C5 revision v1.0.

```bash
source /path/to/esp-idf-v6.0.1/export.sh
idf.py -D SDKCONFIG_DEFAULTS="sdkconfig.defaults;sdkconfig.flash40.defaults" build
idf.py -D SDKCONFIG_DEFAULTS="sdkconfig.defaults;sdkconfig.flash40.defaults" \
  merge-bin -o c5vrx-full.bin -f raw
esptool.py --chip esp32c5 write_flash 0x0 c5vrx-full.bin
```

The browser flasher is documented in [tools/flasher/README.md](tools/flasher/README.md).
Hardware modes are selected through the project Kconfig options; diagnostics
must not be confused with the normal live receiver.

## Knowledge base

[docs/KNOWLEDGE_INDEX.md](docs/KNOWLEDGE_INDEX.md) is the entry point for RF,
MODEM_DIAG, IQ, WBFM, CVBS, PARLIO, DAC, continuity, and rejected approaches.

The repository contains both eras of the project:

- `/main` — current firmware;
- `/docs` — current evidence and engineering contracts;
- `/legacy/c5vrx1` — verbatim snapshot of the original repository;
- `/docs/legacy-issues` — preserved issue and review conclusions.

The Git graph joins both original histories without rewriting their commits.
See [legacy/c5vrx1/ARCHIVE.md](legacy/c5vrx1/ARCHIVE.md) for provenance.

## Roadmap

- identify and eliminate cyclic-DMA boundary tearing;
- quantify the remaining FM/static source and add justified real-domain
  filtering/de-emphasis;
- calibrate pedestal, gain, polarity, blanking, and chroma response;
- complete long-duration source and output continuity proofs;
- keep the hardware path small enough for a practical receiver board.

## License

C5VRX is open-source software licensed under the GNU General Public License
v3.0 only (`GPL-3.0-only`).

See [LICENSE](LICENSE) and [docs/licensing.md](docs/licensing.md) for licensing,
historical attribution, contributor, and branding details.
