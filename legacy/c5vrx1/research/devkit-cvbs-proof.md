# ESP32-C5-DevKitC-1 v1.2 PAL CVBS output proof

This is the shortest hardware test for the analog-first C5VRX output path.
It does **not** test 5.8 GHz reception yet. The dedicated firmware skips RF and
starts a PAL 625/50 monochrome composite test immediately after boot.

## What you need

- ESP32-C5-DevKitC-1
- six series resistors
- one shunt resistor
- a short composite-video lead or video-input pads
- a display/goggle/DVR input that really terminates video at 75 ohm
- ideally an oscilloscope before connecting expensive equipment

## Supported board revision

Use an **ESP32-C5-DevKitC-1 v1.2**. The official v1.2 header table places all
six selected DAC GPIOs on J1 and confirms that none is a strapping pin. The
older DevKitC-1 v1.1 used different header positions and contains C5 silicon
revision v0.1; Espressif has discontinued v0.1 support, while the pinned C5VRX
ESP-IDF v6.0.2 build requires chip revision >= v1.0.

Always follow the GPIO label printed beside the header, not a header position
copied from an image of another board revision.

## DevKit pin assignment

The dedicated `sdkconfig.defaults.cvbs` uses GPIOs that are broken out on the
DevKit and avoids the documented USB Serial/JTAG pins and boot-strapping pins.

| DAC bit | DevKit GPIO | v1.2 header | Resistor to VIDEO node |
|---|---:|---:|---:|
| D0 / LSB | GPIO0 | J1 pin 5 | 7.87 kOhm |
| D1 | GPIO1 | J1 pin 6 | 3.92 kOhm |
| D2 | GPIO6 | J1 pin 7 | 1.96 kOhm |
| D3 | GPIO8 | J1 pin 9 | 976 Ohm |
| D4 | GPIO9 | J1 pin 10 | 487 Ohm |
| D5 / MSB | GPIO10 | J1 pin 11 | 243 Ohm |

Add **191 Ohm from the VIDEO node to GND**.

```text
GPIO0  --- 7.87k --+
GPIO1  --- 3.92k --+
GPIO6  --- 1.96k --+
GPIO8  ---  976R --+---- VIDEO ---- composite input
GPIO9  ---  487R --+
GPIO10 ---  243R --+
                    |
                   191R
                    |
                   GND ----------- video ground
```

The nominal ideal-network target is about 75 ohm Thevenin source impedance,
about 2.0 V open-circuit full scale, and about 1.0 V full scale when terminated
by 75 ohm. Real GPIO output resistance and VOH droop will move those numbers,
so the values are a **scope-validation starting point**, not a production
calibration.

Do not connect raw 3.3 V GPIOs directly to a video input.

Official sources:

- [ESP32-C5-DevKitC-1 v1.2 header block](https://docs.espressif.com/projects/esp-dev-kits/en/latest/esp32c5/esp32-c5-devkitc-1/user_guide.html#header-block)
- [ESP32-C5-DevKitC-1 v1.1 notice and old header block](https://docs.espressif.com/projects/esp-dev-kits/en/latest/esp32c5/esp32-c5-devkitc-1/user_guide_v1.1.html)
- [ESP32-C5-WROOM-1/1U pin definitions](https://documentation.espressif.com/esp32-c5-wroom-1_wroom-1u_datasheet_en.html)

## Firmware

The CI job `Build PAL CVBS output-proof firmware` builds an artifact named:

```text
c5vrx-pal-cvbs-proof-firmware
```

For a local build:

```bash
cp sdkconfig.defaults.cvbs sdkconfig.defaults
idf.py set-target esp32c5
idf.py build
idf.py flash monitor
```

The output-only build starts the CVBS generator automatically; no USB command is
required.

## What the firmware outputs

- 20 MS/s sample clock
- 64 us horizontal line period
- 625-line / 50-field interlaced timing
- half-line equalizing and broad vertical-sync pulse sequence
- 288 active grayscale lines per field
- eight grayscale bars from white to black
- optional 4.43361875 MHz alternating-phase burst during the back porch

The burst is deliberately only a bandwidth/lock stress. The active image is
monochrome, so this is **not** a claim of complete PAL color encoding.

A complete byte-coded frame would consume about 800 kB, so the C5 does not keep
one in RAM. Instead the firmware uses two 40,960-byte PARLIO/GDMA chunks. While
one chunk is transmitted, a high-priority task rebuilds the retired chunk and
queues it for the next buffer switch.

## Expected 6-bit levels before analog error

For the nominal 0..1 V loaded range:

| Video level | Nominal voltage | 6-bit code |
|---|---:|---:|
| sync tip | 0 mV | 0 |
| blank | 300 mV | 19 |
| black | 320 mV | 20 |
| white | 1000 mV | 63 |

The first scope test should verify the actual loaded values rather than assuming
these ideal numbers survive the GPIO and wiring.

## Scope checklist

1. Verify VIDEO is near 0 V during sync tip.
2. Verify blank is near 0.30 V into the actual 75-ohm input.
3. Verify black and blank remain distinguishable with the 6-bit DAC.
4. Verify white is near 1.0 V without obvious GPIO droop.
5. Measure a horizontal period near 64 us and H-sync near 4.7 us.
6. Check that the vertical sequence repeats at 50 fields/s.
7. Look for ringing/overshoot on the fastest transitions.
8. With burst enabled, verify energy around 4.43361875 MHz survives the passive network.
9. Only after the scope looks sane, connect a real analog display and verify stable grayscale lock.
10. Repeat at 5 and 4 bits if you want to find the true minimum useful ladder.

## What success means

A stable picture proves this complete half independently:

```text
C5 RAM -> GDMA/PARLIO -> GPIO resistor DAC -> 75-ohm CVBS -> display
```

It does **not** prove the RF half. The next independent proof is still:

```text
5.8 GHz VTX -> C5 receive path -> phase-bearing samples -> WBFM -> CVBS samples
```

Only after both halves work separately should they be joined into live C5VRX.
