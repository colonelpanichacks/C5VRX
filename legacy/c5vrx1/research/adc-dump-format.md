# ESP32-C5 finite ADC / I-Q dump path

This is currently the strongest C5VRX reverse-engineering result.

## What the C5 v6.0.2 blob proves statically

The ESP32-C5 `librftest.a` shipped with ESP-IDF v6.0.2 contains a complete finite ADC-dump path:

```text
set_dump_mode
adctrig
print_dump_data
sampledeal
accumiq
loop_dump_test
```

This is no longer only a naming hypothesis. Their disassembly shows an actual packed I/Q capture format and a fixed capture buffer.

## Raw sample format

Both `accumiq` and `print_dump_data` decode each 32-bit dump word in the same way:

```text
bits  0.. 9 : Q, signed 10-bit two's complement
bits 10..19 : I, signed 10-bit two's complement
```

`sampledeal()` is exactly the sign conversion:

```text
sample &= 0x3ff
if sample & 0x200:
    sample -= 0x400
```

`accumiq()` walks 32-bit words, applies that conversion independently to Q and I, then accumulates I, Q and `I² + Q²`.

`print_dump_data()` independently performs the same lower-20-bit decode. This is strong evidence that the buffer contains real complex receive samples rather than packet metadata or magnitude-only statistics.

**Still unproven:** that this dump can be triggered reliably on a physical C5 while tuned to an analog FPV VTX, and that the resulting finite samples retain enough RF bandwidth for WBFM video. That is now a hardware experiment, not a missing software architecture.

## C5 dump RAM

`loop_dump_test()` calls:

```text
print_dump_data(0x40830000, 2048, ...)
```

Inside `adctrig()`, the status print loads:

```text
buffer base = 0x40830000
buffer size = 0x00010000 bytes = 64 KiB
```

At one 32-bit word per complex sample, that buffer can hold up to **16,384 I/Q samples**.

The `sample_80m` write is overwritten before enable, so it does not establish a
physical C5 rate. At the still-useful **80 MS/s design case**, a full 64 KiB
ring spans **204.8 us**. The actual span is measured by `PRODUCER CADENCE PROBE`.

## `adctrig` calling convention

Historical Espressif RF-test tooling declares:

```c
void adctrig(
    int32_t smp_num_aft_trig,
    int32_t trigmode,
    int32_t trigcase,
    int32_t sample_80m,
    int32_t dump_trig,
    int32_t rx_gain_mode,
    int32_t rx_gain,
    int32_t rx_gain0,
    int32_t rx_gain0_wait_us
);
```

The C5 v6.0.2 disassembly matches a **nine-argument** function: it consumes the normal RISC-V argument registers and explicitly loads the ninth argument from the caller stack.

Historical Espressif Python tooling maps trigger modes as:

```text
0 = software
1 = BB
2 = CCA
3 = RX start
4 = RX end
5 = TX start
6 = TX end
7 = RX error
```

For receive-only C5VRX testing, software trigger (`0`) is the simplest first
experiment. C5 machine code confirms it by pulsing dump-controller bit 19 after
enable, without waiting for an 802.11 RX-start/end event.

## Vendor loop-dump example recovered from C5

The C5's own `loop_dump_test()` performs the following high-level sequence:

```text
phy_chip_set_chan(5500 MHz, mode)
set_dump_mode(...)
start a test transmission
...
adctrig(2048, 5, 0, 1, 0, 0, 0, 0, 0)
print_dump_data(0x40830000, 2048, ...)
```

Trigger mode `5` is consistent with the historical **TX-start** trigger mapping. That cross-check is another sign that the old RF-test API family and the current C5 implementation are closely related.

## C5VRX implementation

C5VRX now contains an opt-in finite capture helper around the recovered path:

```text
main/c5vrx_adc_dump.c
main/c5vrx_adc_dump.h
```

The experiment:

1. selects normal 10-bit dump mode,
2. uses software trigger,
3. passes the historical `sample_80m` argument but does not claim a physical
   C5 rate because the associated write is overwritten before enable,
4. captures into `0x40830000`,
5. decodes signed I/Q,
6. prints summary statistics,
7. optionally emits raw machine-readable `IQ:xxxxxxxx` lines.

The path remains **disabled by default** until physical C5 testing.

Host-side decoding is provided by:

```bash
python tools/decode_adc_dump.py serial.log --csv iq.csv
```

From there, the I/Q pairs can be fed into `tools/wbfm_demod.py`.

## First physical proof test

Use an exact center first to remove arbitrary tuning from the experiment:

```text
VTX: A4 / 5805 MHz
C5:  Wi-Fi ch161 / 5805 MHz
BW:  40 MHz bootstrap
ADC: software-triggered finite dump
```

Capture three conditions:

```text
1. VTX off
2. VTX on, static image
3. VTX on, strongly changing black/white image
```

Success criteria for the first test are deliberately modest:

- dump is non-constant and repeatable,
- I/Q statistics respond to the RF source,
- spectrum of the complex samples shows the analog carrier / modulation,
- WBFM discriminator output changes with the transmitted image.

Only after this works should the same test be repeated at R5 / 5806 MHz using the undocumented direct-frequency hook.

## Next bottleneck

The finite 64 KiB path is enough for proof-of-concept, but not live video. If the capture works, the next reverse-engineering target is the producer feeding the dump RAM so C5VRX can obtain continuous or rapidly chained sample blocks without gaps.
