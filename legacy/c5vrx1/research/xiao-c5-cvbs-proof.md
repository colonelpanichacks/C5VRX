# XIAO ESP32-C5 Receiver Console and PAL CVBS proof

Target: Seeed Studio XIAO ESP32-C5. This is a separate pin and flash profile;
do not use the DevKitC wiring or its ordinary Receiver Console artifact.

The mapping follows [Seeed's official XIAO ESP32-C5 pin table](https://wiki.seeedstudio.com/xiao_esp32c5_getting_started/)
and the [official ESP32-C5 datasheet](https://www.espressif.com/sites/default/files/documentation/esp32-c5_datasheet_en.pdf). It avoids native
USB GPIO13/14, battery measurement GPIO6/26, boot GPIO28, ADC header D0/GPIO1,
and strapping GPIO7/GPIO25. The selected header pins are ordinary digital GPIOs
and are routed through the C5 GPIO matrix to PARLIO.

| DAC bit | XIAO pin | GPIO | Series resistor |
|---|---:|---:|---:|
| D0 / LSB | D4 | 23 | 7.87 kΩ |
| D1 | D5 | 24 | 3.92 kΩ |
| D2 | D6 | 11 | 1.96 kΩ |
| D3 | D7 | 12 | 976 Ω |
| D4 | D8 | 8 | 487 Ω |
| D5 / MSB | D9 | 9 | 243 Ω |

This mapping has been checked against Seeed's current official pin table.
GPIO23/24/11/12/8/9 are header-accessible GPIO-matrix outputs and none is an
ESP32-C5 strapping pin. GPIO11 and GPIO12 are also the default UART0 TX/RX
functions; the proof image uses native USB Serial/JTAG and PARLIO takes over
those pins when the application starts. A short reset/ROM transient on
D6/GPIO11 is therefore possible and is not valid CVBS evidence.

Join the six resistor outputs at `VIDEO`:

```text
VIDEO node -> 191 ohm -> GND
VIDEO node -> composite video input
XIAO GND   -> video ground
```

Two XIAO artifacts are intentionally available:

- `c5vrx-xiao-pal-cvbs-proof-firmware` is output-only. No commands are required;
  it starts PAL timing at boot, shows the C5VRX splash for 12 seconds, then
  switches content (not timing) to the diagnostic screen.
- `c5vrx-xiao-receiver-console-firmware` is the full guarded RF/diagnostics,
  live AV and USB-preview image. Windows users can instead download
  `C5VRX-XIAO-Receiver-Console-Windows`, which bundles that exact firmware.

The full profile is [`sdkconfig.defaults.xiao_receiver_console`](../sdkconfig.defaults.xiao_receiver_console).
It configures the XIAO's documented 8 MB flash with a 6 MiB factory partition
and identifies itself as `profile=xiao-esp32c5` in `STATUS`. The XIAO Windows
console checks that identity after connecting. The onboard 8 MB PSRAM is not
enabled yet: the fixed RF dump RAM is internal and current PARLIO buffers stay
in DMA-capable internal RAM until measurement justifies another memory path.

The full profile does not start video unconditionally. First run the output
proof (`CVBS TEST`), then the automated hardware diagnostics. `LIVE START`
remains fail-closed until its RF cadence, phase continuity, bandwidth and
throughput gates pass.

The resistor values are calculated nominal targets. Measure sync, blank, black
and white levels into the exact 75-ohm AV input before treating them as a final
BOM. Parallel input termination and resistor tolerances change the result.
