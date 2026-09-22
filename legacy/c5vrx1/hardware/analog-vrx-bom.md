# Standalone analog C5VRX BOM

This is the recommended first **self-contained analog-output C5VRX PCB** once
the direct ESP32-C5 receive path is proven on hardware.

The design philosophy is deliberately conservative in only one place: use the
ESP32-C5 module rather than the bare chip for the first custom receiver PCB.
The video path itself remains minimal.

```text
5 V USB-C
   |
3.3 V LDO
   |
ESP32-C5-WROOM-1U-N4 <--- 5.8 GHz antenna
   |
PARLIO D0..D5
   |
6-bit passive 75-ohm DAC
   |
CVBS ESD
   |
RCA / AV OUT
```

## Recommended product BOM

| Ref | Qty | Part / value | Suggested MPN | Package | Purpose | Required? |
|---|---:|---|---|---|---|---|
| U1 | 1 | ESP32-C5-WROOM-1U, 4 MB flash | **ESP32-C5-WROOM-1U-N4** | module, 18.0 x 21.2 mm | RF + CPU + RAM + flash + crystal + RF matching | yes |
| U2 | 1 | 3.3 V / 1 A LDO | **TLV75733PDBVR** | SOT-23-5 | USB 5 V -> clean 3.3 V rail | yes for 5 V input |
| D1 | 1 | dual low-C USB ESD array | **TPD2EUSB30DRTR** | 1 x 1 mm DRT | protect USB D+/D- | recommended |
| D2 | 1 | single low-C ESD diode | **TPD1E05U06DYAR** | SOD-523 / DYA | protect CVBS connector node | recommended |
| J1 | 1 | USB-C USB 2.0 receptacle | 16-pin USB-C receptacle | SMT | power + native USB flash/debug | yes |
| J2 | 1 | composite video connector | RCA female or keyed 2-pin AV connector | TH/SMT | VIDEO + GND | yes |
| SW1 | 1 | momentary switch | generic | 3x2 mm class | RESET: EN -> GND | recommended |
| SW2 | 1 | momentary switch | generic | 3x2 mm class | BOOT: GPIO28 -> GND | recommended |
| R1 | 1 | 7.87 kOhm, 1% | generic thin-film | 0603 | CVBS DAC D0 / GPIO0 | yes |
| R2 | 1 | 3.92 kOhm, 1% | generic thin-film | 0603 | CVBS DAC D1 / GPIO1 | yes |
| R3 | 1 | 1.96 kOhm, 1% | generic thin-film | 0603 | CVBS DAC D2 / GPIO6 | yes |
| R4 | 1 | 976 Ohm, 1% | generic thin-film | 0603 | CVBS DAC D3 / GPIO8 | yes |
| R5 | 1 | 487 Ohm, 1% | generic thin-film | 0603 | CVBS DAC D4 / GPIO9 | yes |
| R6 | 1 | 243 Ohm, 1% | generic thin-film | 0603 | CVBS DAC D5 / GPIO10 | yes |
| R7 | 1 | 191 Ohm, 1% | generic thin-film | 0603 | VIDEO node -> GND; completes ~75-ohm Thevenin DAC | yes |
| R8,R9 | 2 | 5.1 kOhm, 1% | generic | 0603 | USB-C CC1/CC2 Rd | yes |
| R10,R11 | 2 | 22 Ohm | generic | 0402/0603 | USB D-/D+ series placeholders/populated start value | recommended |
| R12 | 1 | 10 kOhm | generic | 0603 | EN pull-up / reset RC | yes |
| C1 | 1 | 1 uF X7R | generic | 0603 | EN reset delay to GND | yes |
| C2 | 1 | 10 uF X5R/X7R, >=10 V | generic | 0805 | USB/VBUS bulk at board entrance | yes |
| C3 | 1 | 1 uF X7R, >=10 V | generic | 0603 | LDO input local bypass | recommended |
| C4 | 1 | 10 uF X5R/X7R, >=6.3 V | generic | 0805 | 3.3 V rail bulk close to module | yes |
| C5 | 1 | 0.1 uF X7R | generic | 0402/0603 | local 3.3 V high-frequency bypass | yes |
| TP1 | 1 | test pad | - | pad | 5 V | recommended |
| TP2 | 1 | test pad | - | pad | 3.3 V | recommended |
| TP3 | 1 | test pad | - | pad | CVBS video node | recommended |
| TP4 | 1 | test pad | - | pad | GND / scope spring | recommended |

External accessory, not normally assembled on the PCB:

- one 50-ohm 5.8 GHz antenna with U.FL / MHF-I compatible plug;
- USB-C cable;
- composite video cable if J2 is not a direct RCA connector.

## Why these active parts

### U1: ESP32-C5-WROOM-1U-N4

This is the preferred first product target because it already integrates the
48 MHz crystal, RF matching/diplexer and 4 MB flash. The 1U variant provides an
external first-generation miniature RF connector compatible with U.FL / MHF-I
style plugs. No PSRAM is required by the current streaming analog architecture.

The module needs a 3.0-3.6 V supply. The board should target 3.3 V.

### U2: TLV75733PDBVR

The regulator is deliberately oversized relative to expected receive current:
1 A capability leaves transient margin and its fixed 3.3 V version avoids a
feedback divider. The device accepts USB 5 V directly and is stable with small
ceramic output capacitance.

A switching regulator is not justified for the first receiver PCB unless power
or thermal measurements later show that the LDO loss matters.

### D1 / D2

D1 protects the native USB differential pair without adding a USB-UART bridge.
D2 is located at the AV connector and protects only the final composite node;
do not put large-capacitance protection directly on every DAC GPIO.

## USB wiring

```text
USB-C VBUS ---- U2 3.3 V ---- U1 3V3
USB-C GND ------------------- GND
USB-C D- -- D1 -- 22R ------- U1 GPIO13
USB-C D+ -- D1 -- 22R ------- U1 GPIO14
CC1 -------- 5.1k ----------- GND
CC2 -------- 5.1k ----------- GND
```

ESP32-C5 has native USB Serial/JTAG, so there is **no CH340 / CP2102 / CH343**
anywhere in the BOM.

Keep the two USB traces as a 90-ohm differential pair and reserve optional
capacitor footprints to ground near the C5 side if following the complete
Espressif USB reference layout.

## Boot / reset

The module EN pin must not float. Use:

```text
3V3 --- 10k ---+--- EN
               |
              1uF
               |
              GND

EN ---- RESET switch ---- GND
```

For manual USB download mode, GPIO28 can be pulled low while resetting the
chip. Therefore SW2 is:

```text
GPIO28 ---- BOOT switch ---- GND
```

Do not use GPIO26/27/28 for the video DAC. They participate in boot strapping.

## CVBS DAC wiring

Reference pin assignment:

```text
GPIO0  -- 7.87k --+
GPIO1  -- 3.92k --+
GPIO6  -- 1.96k --+
GPIO8  -- 976R  --+
GPIO9  -- 487R  --+---- VIDEO ---- D2 ---- J2 center/video
GPIO10 -- 243R  --+
                  |
                 191R
                  |
                 GND -------------------- J2 shield/GND
```

The receiving monitor provides its normal 75-ohm termination. Do **not** add a
second 75-ohm termination on the C5VRX board.

The nominal network is designed for approximately:

```text
Thevenin source impedance: ~75 ohm
open-circuit full scale:   ~2.0 V
75-ohm loaded full scale:  ~1.0 V
```

Real GPIO output resistance and VOH droop must be measured before freezing the
resistor values for production.

## Functional-minimum BOM

If C5VRX is embedded in another device that already provides a clean 3.3 V rail,
native USB is not required, and ESD/connector protection is handled elsewhere,
the actual VRX core becomes:

```text
1x ESP32-C5-WROOM-1U-N4
7x CVBS DAC resistors
1x EN 10k
1x EN 1uF
1x 10uF rail capacitor
1x 0.1uF rail capacitor
1x AV connector/pads
1x external 5.8 GHz antenna
```

That is **one active semiconductor module in the functional signal path**.

## Product-ready versus minimum

| Function | Functional minimum | Standalone recommended |
|---|---|---|
| RF/compute | C5-WROOM-1U-N4 | C5-WROOM-1U-N4 |
| 3.3 V supply | supplied by host | TLV75733PDBVR |
| Flash | inside module | inside module |
| Crystal | inside module | inside module |
| RF matching | inside module | inside module |
| Antenna connector | inside module | inside module |
| Video DAC | 7 resistors | 7 resistors |
| Video buffer | none | none initially; add only if scope/cable tests require it |
| USB-UART IC | none | none |
| USB ESD | host-dependent | TPD2EUSB30 |
| CVBS ESD | host-dependent | TPD1E05U06 |
| OSD IC | none | none |
| PSRAM | none | none |

## PCB recommendation

Use four layers for the first product PCB:

```text
L1 components + signals
L2 solid GND
L3 power + limited signals
L4 limited signals / GND fill
```

Place the module at a board edge, keep USB differential routing short, keep the
CVBS resistor summing node compact, and put D2 directly at the external video
connector. The module antenna connector means the base-board layout does not
need its own 5.8 GHz matching network for ANT1.

## Do not freeze this BOM yet

The BOM is electrically complete for the **standalone analog-output architecture**,
but production release depends on the remaining silicon proof:

1. live 5.8 GHz analog VTX appears in the recovered C5 I/Q path;
2. the finite producer can be converted to a continuous or sufficiently chained
   stream;
3. hardware WBFM sustains the required rate;
4. the passive DAC is measured into real 75-ohm equipment;
5. cable-drive/ESD tests determine whether an optional video buffer is useful.

Until items 1-4 are physically measured, this is a prototype/product-candidate
BOM, not a released manufacturing BOM.

## Primary design references

- Espressif ESP32-C5-WROOM-1 / WROOM-1U Datasheet v1.1.
- Espressif ESP32-C5 Hardware Design Guidelines / Schematic Checklist.
- Espressif ESP32-C5 USB Serial/JTAG documentation.
- Texas Instruments TLV757P datasheet.
- Texas Instruments TPD2EUSB30 datasheet.
- Texas Instruments TPD1E05U06 datasheet.
