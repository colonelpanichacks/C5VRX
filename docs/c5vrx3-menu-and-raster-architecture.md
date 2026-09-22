# C5VRX-3 Standalone Menu & Raster Architecture

## Implemented on PR #26: independent menu output

The dedicated generator is implemented and host-tested. Physical confirmation
on the XIAO ESP32-C5 and FatShark HD3 is still required. A build or sample test
does not establish decoder lock, analog levels, or uninterrupted DMA output.

The reported symptom was a readable menu for about one second, then inversion
and disappearance. Progressive timing, missing burst, synthetic IQ and live-ring
splicing were visible in the old source. Decoder standard detection and clamp
drift remain hypotheses, not measured root causes. The exact decoder model and
its internal behavior have not been verified. Missing burst alone does not
explain rejection of a monochrome signal.

### Output ownership

```text
FLIGHT: MODEM_DIAG -> PARLIO RX -> 16 KiB ring -> Phase5 -> PARLIO TX -> DAC
MENU:   independent SRAM raster -----------------------> PARLIO TX -> DAC
```

Flight clock, IQ ring, Phase5 program, sample edges and DAC GPIO order are
unchanged. Both sources use the 40 MHz DAC clock. Menu samples are direct bytes:
sync 0, blank/black 20, text 60, burst 20 +/- 8. Loaded voltages need measurement.

`video.c` now owns the BitScrambler handle explicitly. In ESP-IDF v6.0.2,
interrupting an infinite decorated TX transaction does not call its BitScrambler
disable hook, and a subsequent NULL program does not detach the previous one.
The new code explicitly disables/detaches Phase5 before menu output, then
enables, loads, resets and starts it before returning to flight.

Transitions stop TX through the driver. With the driver's transaction queue
empty, a C5-specific adapter starts the separate scatter chain using its allocated
channel's head register and PARLIO LL functions. The driver retains allocation
and stop/reset ownership. There is no private driver-struct access or live-ring
link replacement. FIFO-ready waiting has a one-millisecond timeout; unexpected
transition errors use `ESP_ERROR_CHECK` rather than continuing with partial state.

Returning to flight restarts RX at ring zero before the TX delay. Delaying an
already-running RX cannot establish separation. The delay calculation now uses
64-bit arithmetic: the old `8192u * 1000000u` overflowed. The requested delay is
204 us, truncated from 204.8 us, plus driver latency; exact physical separation
still needs measurement.

### Raster timing and memory

Timing reference: [ITU-R BT.470](https://www.itu.int/rec/R-REC-BT.470/en), including
the line/field diagrams and burst sequences in
[BT.470-6](https://www.itu.int/dms_pubrec/itu-r/rec/bt/R-REC-BT.470-6-199811-S!!PDF-E.pdf).
The hardware-independent `menu_raster.c` emits these DMA segments:

- PAL: 625 lines/frame, 312.5 lines/field, 64 us lines, 50 fields/s; five
  pre-equalizing, five broad and five post-equalizing half-line pulses. Line 1
  starts with broad sync. Eight fields occupy exactly 6,400,000 samples.
- NTSC: 525 lines/frame, 262.5 lines/field, approximately 59.94 fields/s; six
  pulses in each vertical group. Cumulative half-line times are rounded to DMA
  words rather than rounding every scanline independently. Eight fields occupy
  5,338,668 samples: period error +0.25 ppm. The carrier is adjusted about
  -0.895 Hz to close after 477,750 cycles, saving two frames of descriptors.
- Ordinary H sync stays on the full-line grid across both fields. H-sync width
  is 4.7 us; broad sync ends 4.7 us before the next half-line edge. Within the
  NTSC cycle, timing quantization is at most 44.45 ns.
- Burst is 4.43361875 MHz PAL or approximately 3.57954456 MHz NTSC. PAL uses
  alternating phase and nine-line burst-blanking windows; NTSC suppresses burst
  during its vertical pulse train. Absolute sample time determines phase,
  including loop closure. Shared 32-phase templates quantize starting phase
  by at most 5.625 degrees.

This is a quantized monochrome menu with a colour reference burst, not a claim
of laboratory broadcast compliance. Analog ratios, edge shaping, oscillator
tolerance and decoder compatibility require physical validation.

Shared porch, blank and text buffers avoid a 1.6 MB PAL framebuffer. Chains use
5,900 PAL or 5,100 NTSC nodes, with 6,000 slots reserved. Raster plus descriptor
capacity occupies 148,800 bytes, slightly less than the old menu allocation.
Text uses four samples/pixel. Only text pixels may change while scanning (one
refresh can tear); timing buffers and links change only with TX stopped.

### Controls, console and validation

The AGC/control task exclusively owns menu rendering, BOOT handling, transitions
and inactivity timeout. Console menu commands enter a bounded queue. Diagnostic
output no longer patches descriptors. The console directly drains the USB RX
FIFO in bounded batches: IDF 6.0.2's nonblocking VFS read checks driver-buffer
availability before calling its no-driver FIFO reader. No second FIFO reader or
USB ISR driver is installed. Output retains the unbuffered VFS; the menu-active
polling branch has no periodic output or `fflush`.

Build with ESP-IDF v6.0.2 for `esp32c5`; `python tools/validate_build.py` checks
35 production constraints. The host test runs the firmware generator and checks
every sample, DMA bounds/alignment, both fields' pulses, horizontal phase, DAC
range, burst windows/frequency, timing quantization and carrier closure:

```sh
cc -std=c11 -Wall -Wextra -Werror -fsanitize=address,undefined -I main \
  tools/test_menu_raster.c main/menu_raster.c -lm -o /tmp/test_menu_raster
/tmp/test_menu_raster
```

Remaining hardware tests: both standards beyond one second; repeated open/close;
standard changes while open; simultaneous console/BOOT; timeout without USB;
terminated DAC sync/burst/level measurements; underflows and restored flight
RX/TX distance. Runtime heap, USB host behavior and direct scatter startup cannot
be established by the host waveform test.

---

## Historical investigation and proposal (superseded)

The sections below preserve the earlier diagnosis and proposed architecture.
Their categorical decoder/clamp claims were not established measurements, and
their progressive-menu status and proposed timing are superseded by the
implementation above. In particular, PAL was 312 progressive lines (not 288
total), NTSC interlace uses 262.5 lines/field, and the current NTSC generator does
not use the proposed fixed 2542-sample line.

## 1. Executive Summary

This document captures the empirical findings, root-cause analyses, and architectural decisions regarding the **C5VRX-3 on-screen configuration menu** on the Seeed Studio XIAO ESP32-C5.

The previous "OSD-derived" menu implementation exhibited a critical failure mode on analog video goggles (notably the FatShark Dominator HD3):
* **The Symptom**: Upon holding the BOOT button, the menu displays clearly for approximately 1.0 second. Subsequently, the entire screen inverts (white background, black text), and within a few hundred milliseconds, fades to black or auto-mutes. Concurrently, serial communication frequently encounters `Write timeout` or hangs.
* **The Core Finding**: The menu was architected as an in-flight **GDMA descriptor splice** into the active WBFM RF pipeline, generating pseudo-progressive 262p/288p frames through fake synthetic IQ cycles fed into the Phase5 BitScrambler, with no color burst and no true interlace field structure. An overlay engine can "cheat" because the camera provides rock-solid sync and burst; a standalone generator cannot.

---

## 2. Root Cause Analysis

### 2.1 The 1-Second Freeze & Console Deadlock
* In `analog_agc_task`, tick 20 (exactly 1.0 s at 20 Hz) previously executed:
  ```c
  printf("[MENU ACTIVE] ...\n");
  fflush(stdout);
  ```
* When no serial monitor or terminal application is actively polling the USB-Serial-JTAG CDC endpoint on the host PC, the internal USB CDC TX buffer fills completely. Calling `fflush(stdout)` invokes `usb_serial_jtag_wait_tx_done()`, which blocked the priority-3 `analog_agc_task` indefinitely.
* Because the task was blocked:
  1. Button sampling stopped.
  2. The 12.0 s inactivity auto-exit timer stopped.
  3. The USB CDC OUT endpoint on the ESP32-C5 stopped pulling packets from the host, causing host-side `ser.write()` in pySerial to fail with `serial.serialutil.SerialTimeoutException: Write timeout`.

### 2.2 The 800 ns Sync Phase Jump & DC Clamp Inversion
* Mathematical inspection of `main/video.c` revealed that in the synthetic scanline generator:
  * Normal blank lines and OSD active lines positioned the falling H-sync edge at **Word 32** (1.6 µs front porch).
  * Equalizing pulse lines (`s_eq_line`) and vertical sync serration lines (`s_vsync_line`) positioned the falling edge at **Word 16** (0.8 µs front porch).
* Every vertical blanking interval (50 Hz or 60 Hz), the horizontal sync phase experienced a discontinuous **16-word (800 ns) phase shock**.
* The TP9950 / TVP5150 analog video digitizer inside the FatShark Dominator HD3 uses an internal horizontal PLL error accumulator and back-porch DC restoration clamp:
  1. Over ~30–60 frames (0.5–1.0 s, corresponding to the decoder's RC clamp filter time constant), the horizontal phase error drifted the clamp sampling window off the back porch and directly into the active video text area (1.0 V white text).
  2. Clamping 1.0 V active white to black inverted the entire video signal: the background became peak white and the font glyphs became pitch black.
  3. Shortly thereafter, the decoder's phase lock detector declared loss of horizontal lock and auto-muted the display.

### 2.3 The Non-Standard Raster & Missing Color Burst
* Even with the 800 ns sync jump corrected, the signal was generated as **262p progressive** (for NTSC) or **288p progressive** (for PAL):
  * **No 0.5 H half-line offset**: Broadcast PAL (625i) and NTSC (525i) strictly require an alternating half-line offset (Field 1: 312.5 lines, Field 2: 312.5 lines) so the CRT/digitizer interleaves fields.
  * **No color burst**: Broadcast decoders with automatic standard detection look for the 4.4336 MHz (PAL) or 3.5795 MHz (NTSC) subcarrier burst on the back porch (5.6 µs – 7.8 µs). Without a color burst or a valid field structure, the decoder's standard identification state machine times out and rejects the signal.

### 2.4 The BitScrambler Pipeline Mismatch
* The BitScrambler program `fm.bsasm` was designed exclusively for 50 ns WBFM phase-difference demodulation:
  ```text
  Packed Q4/I4 byte → LUT[cartesian] (Phase5 state) → LUT[delta] (DAC code) → [D, D]
  ```
* Forcing menu video through this pipeline required synthesising artificial IQ sequences to fool the demodulator into emitting specific DAC codes (0 for sync tip, 20 for pedestal, 60 for white).
* This architecture has severe drawbacks:
  1. The first menu frame inherits the lingering Phase5 state from the previous live RF sample.
  2. Splicing `s_tx_dscr_nodes[last].dscr->next = &s_osd_dma_nodes[0]` directly while the GDMA channel is running at 40 MB/s creates descriptor fetch race conditions.
  3. When GDMA completes transmission of descriptors, hardware updates descriptor status and ownership, which can lead to DMA descriptor errors and hardware lockups if the looped chain is not configured according to the driver's dual-link model.

---

## 3. Current Working State

The following components are fully operational, tested, and verified on live hardware:
1. **Live Flight Receiver Pipeline**:
   * RF reception at 5.8 GHz (BW40 and BW20) on channel 173 (and full A/B/R/F band tables).
   * PARLIO RX at 40 MS/s (POS edge) into 16 KiB cyclic GDMA ring.
   * Phase5 BitScrambler demodulator (1044-entry embedded LUT, 50 ns discriminator).
   * PARLIO TX at 40 MHz with `[D,D]` sample duplication driving 6-bit DAC ladder on GPIOs 23, 24, 11, 12, 8, 9.
   * Zero-EOF circular GDMA descriptor patch eliminating DMA boundary wrap bubbles.
2. **Adaptive Dual-Loop AGC**:
   * Realtime evaluation of $P_{\text{median}}$ and FM phase coherence ($Q_{\text{phase}}$).
   * Hysteresis deadband ($P \in [18, 32]$) preventing gain hunting in TRACK mode.
   * Hardware sticky status counters (`tx_fifo_rempty_int_raw`, `rx_fifo_wovf_int_raw`) for truthful hardware starvation monitoring.
3. **BOOT Button Debounce**:
   * Sampled reliably at 20 Hz in `analog_agc_task` on GPIO 28.
   * Short click (< 600 ms) cycles channels in flight mode; long press (≥ 600 ms) triggers menu mode.
4. **Build & Release Automation**:
   * `tools/validate_build.py` enforces 31 production architectural constraints.
   * `tools/package_release.py` automatically builds and mirrors standalone and merged binaries to `Desktop/` and `packages/`.

---

## 4. What Does NOT Work Yet

The following items are unresolved in the current `codex/c5vrx3-osd-freeze-and-inversion-fix` branch:
1. **Menu Display Stability**:
   * The menu does not maintain a continuous lock on auto-detect goggles (FatShark HD3) because it lacks genuine 625i/525i interlace timing and color burst.
2. **BitScrambler Decoupling**:
   * The menu still runs through `fm.bsasm` using synthetic IQ patterns instead of raw DAC CVBS.
3. **Descriptor Splicing**:
   * Splicing descriptors in-place during hardware streaming can cause GDMA ownership faults and lockups.
4. **USB Console Host Input**:
   * Calling `usb_serial_jtag_vfs_use_nonblocking()` in ESP-IDF v6.0 without the driver installed causes `getchar()` to never poll the hardware RX FIFO, preventing interactive console hotkeys and causing host pySerial `Write timeout` exceptions unless DTR/RTS is explicitly managed.

---

## 5. Approved Target Architecture

To resolve the remaining issues permanently, the menu will be restructured as an independent **Dedicated Broadcast Raster Generator**:

```text
========================================================================
FLIGHT MODE:
  RF (MODEM_DIAG) → PARLIO RX → 16 KiB Ring → Phase5 BS → TX → DAC
  (Autonomous zero-CPU hardware pipeline, identical to Seamless Golden 16K)

[Transition: parlio_tx_unit_disable(s_tx)]

MENU MODE:
  Dedicated Broadcast Standard PAL (625i) / NTSC (525i) Raster → DAC
  (Direct DAC bytes, NO BitScrambler, NO synthetic IQ, valid Color Burst)
========================================================================
```

### 5.1 Direct DAC Video Generation
* BitScrambler is detached or disabled during menu mode:
  ```c
  bitscrambler_disable(s_tx_bs_handle);
  ```
* PARLIO TX reads raw 8-bit DAC bytes directly from SRAM at 40 MS/s:
  * `0` = Sync tip (0.0 V)
  * `20` = Blanking / Black pedestal (0.33 V)
  * `60` = Active white menu text (1.0 V)
  * `20 ± 8` = Pre-calculated subcarrier sine wave on the back porch (4.4336 MHz for PAL, 3.5795 MHz for NTSC).

### 5.2 True Broadcast Raster Timing
* **PAL 625i (CCIR System B/G)**:
  * Line period: exactly 64.000 µs (2560 bytes @ 40 MHz).
  * Field 1: 312.5 lines (312 lines + 1 half-line of 1280 bytes).
  * Field 2: 312.5 lines (1 half-line of 1280 bytes + 312 lines).
  * Total frame: 625 lines = 1,600,000 bytes @ 40 MHz = exactly 25.0000 Hz frame rate (50.0000 Hz field rate).
* **NTSC 525i (EIA RS-170A)**:
  * Line period: 63.550 µs (2542 bytes @ 40 MHz).
  * Field 1: 262.5 lines (262 lines + 1 half-line of 1271 bytes).
  * Field 2: 262.5 lines (1 half-line of 1271 bytes + 262 lines).
  * Total frame: 525 lines = 1,334,550 bytes @ 40 MHz = 29.972 Hz frame rate (59.945 Hz field rate).

### 5.3 Clean Mode Switching Protocol
* **Entering Menu**:
  1. `parlio_tx_unit_disable(s_tx)`
  2. Detach / disable BitScrambler.
  3. Load menu descriptor chain and buffer pointers.
  4. `parlio_tx_unit_enable(s_tx)` and start loop transmission.
* **Exiting Menu**:
  1. `parlio_tx_unit_disable(s_tx)`
  2. Re-attach and enable Phase5 BitScrambler program.
  3. Load `s_raw_ring` transmit configuration.
  4. Resynchronize RX→TX separation: `esp_rom_delay_us(8192u * 1000000u / IQ_RATE_HZ)`.
  5. `parlio_tx_unit_enable(s_tx)` and start live receiver loop.
