# XIAO ESP32-C5 A1 IQ/USB hardware session

Date: 2026-08-20. Hardware: Seeed Studio XIAO ESP32-C5 rev v1.0,
8 MB flash, native USB Serial/JTAG. ESP-IDF: v6.0.2.

## Proven on the physical board

- A complete verified flash boots the normal receiver firmware and answers
  `PING` with `C5VRX_PONG` after the native USB endpoint has re-enumerated.
- `SET A 1` selects 5865 MHz through Wi-Fi channel 173. `STATUS` readback has
  reported band A, channel 1, 5865 MHz, channel 173 and 40 MHz bandwidth.
- The minimal direct USB diagnostic proved application USB RX and TX on the
  board. The earlier silent normal console was therefore a stdio/VFS software
  issue, not a different Windows driver requirement.
- Earlier finite A1 captures gave a repeatable VTX OFF/ON/OFF energy change and
  a ring-like IQ distribution with the VTX on. This is strong RF-dependent IQ
  evidence, but not proof of decoded analog video.

## Hard limits and honest classifications

- One recovered FE/ADC dump is at most 65536 bytes: 16384 packed `uint32_t`
  words. Firmware must never ask this producer for more than 16384 words.
- Multiple finite dumps are separately retriggered. They are not gapless and
  must not be presented as one continuous PAL frame.
- The producer sample rate is not yet calibrated. `finite_fill_msps` is logged
  only as a finite-fill timing estimate when the hardware done bit is seen.
- Native USB Serial/JTAG is Full Speed. Continuous raw 32-bit IQ at a possible
  tens-of-MS/s producer rate does not fit. A final continuous design needs
  on-device phase difference/WBFM, anti-alias filtering, decimation and compact
  packing before USB; PC-side sync and image rendering can then do the heavier
  video work.

## Capture safety changes

The recovered RF dump handoff can temporarily give the MAC ownership of HP
SRAM used by normal application code and stacks. The bounded capture kernel now
runs entirely from LP RAM, switches to an LP-RAM stack, pauses the hardware
stack guard around that deliberate switch, polls with a CPU-cycle deadline,
disables the producer and restores CPU SRAM ownership before returning.

This path remains experimental until `C5VRX_CAPTURE_KERNEL` plus a complete IQ
payload are observed repeatedly on hardware. RF capture is command-triggered;
it no longer starts automatically during boot, so USB control remains available
for diagnosis.

## USB transport and Receiver Console

Packet type 4 (legacy) contains one large IQ block. Packet type 5 fragments a
finite capture into independently CRC-protected chunks. Its little-endian
descriptor is five `uint32_t` values:

1. capture id;
2. total words;
3. word offset;
4. words in this chunk;
5. flags (bit 0 first, bit 1 last).

The current chunk size is 256 IQ words (1024 data bytes). The host validates
each chunk, rejects mixed/conflicting captures and reassembles only contiguous
offsets. CRC errors include packet type, sequence, payload size and both CRCs.

The combined XIAO Receiver Console exe includes an A1 IQ-to-PC-video mode. It
repeatedly requests safe 16K blocks, performs an FM discriminator on the PC,
finds candidate horizontal line spacing, resamples active line portions to 160
pixels and updates a 160x120 grayscale image row by row. It reports USB Mbit/s,
blocks/s and the finite-fill MS/s estimate. Until a continuous producer exists,
the UI explicitly labels this mode `RETRIGGERED, NOT GAPLESS`.

## Windows flashing/USB behavior

An esptool `SerialTimeoutException: Write timeout` occurs during bootloader sync,
before firmware is written. A fresh manual flash action often succeeds because
the failed in-process invocation has finally released COM10. The XIAO flasher
therefore runs every esptool attempt in a new OS process, waits for exclusive
port access and retries only after that process and its Win32 handle are gone.

After flashing or an abnormal capture/console close, Windows can show COM10 as
present while the endpoint returns write timeouts or no bytes. A physical USB
disconnect/reconnect has been the reliable recovery. Do not misclassify that
host endpoint state as an RF or firmware result.

The normal app was later observed by JTAG at `esp_cpu_wait_for_intr` while the
VFS console still returned zero bytes. This proved the silent state was not the
ROM downloader or an application crash. The control reader, control replies,
ESP logs and framed IQ/video packets were therefore moved onto one shared,
direct `usb_serial_jtag_*` transport. The new control reader assembles command
lines itself instead of using `fgets(stdin)`.

That direct transport was then proven on the physical XIAO. After a verified
flash and the required USB power-cycle, the normal receiver emitted
`C5VRX_READY`, the Windows console classified `C5VRX_RUNTIME_READY
handshake=PASS`, `PING` returned `C5VRX_PONG`, and Wi-Fi completed with
`C5VRX_BOOT stage=WIFI5_READY`. A1 boot tuning read back channel 173 at 40 MHz.
Early `STATUS` calls returned code 259 only while Wi-Fi initialization was still
in progress; this is the intended fail-open control-before-RF startup order.

The first repeated type-5 hardware run transferred 43 complete 16384-word
blocks with zero protocol errors and zero device write failures. Every capture
timed out at the former 10 ms LP-kernel deadline and every reconstructed block
had the identical CRC32 `023ecbba`. This proves the chunked USB transport while
also proving those words were stale SRAM, not 43 fresh RF captures. Timeout data
is now rejected instead of published, and the CPU-cycle deadline was increased
to 50 ms while remaining below the configured 300 ms interrupt watchdog.

With that deadline, the first physical capture attempt still timed out after
51.701 ms with `final_control=80004000` and no done bit. A following attempt
completed after 38.631 ms with `final_control=80044000`, `done=1`, and a
0.424115 MS/s finite-fill estimate. This is the first direct proof that this
bounded kernel reached a fresh hardware completion. The estimate includes
trigger/fill overhead and is not a calibrated producer sample rate.

That successful payload fell back to ASCII because the preview lifecycle had
already returned `STOP code=263`; a later `START` consequently returned code
257 while the old task still existed. The stop loop used
`pdMS_TO_TICKS(1)`, which can round to zero at the configured tick rate. The
preview lifecycle now uses real scheduler ticks, has idempotent START/STOP
commands, and avoids a potentially blocking STREAM_END write during teardown.
The Receiver Console waits for `START code=0` before its first capture and
retries a bounded RF timeout without tearing down the binary transport.

Follow-up hardware testing showed that 4096-word captures and repeated
16384-word captures all failed identically in the LP register clone: the
length field changed correctly, but the completion bit never asserted. This
rules out the 64 KiB block size as the cause. The capture path therefore uses
the complete, hash-pinned Espressif `adctrig()` as a fallback after the first
bounded LP miss. Before either attempt, CPU-owned dump RAM is filled with a
per-word sentinel. Vendor output is accepted only when it returns before the
recovered vendor timeout and at least 99 percent of the requested words have
actually replaced their sentinel. This retains fail-closed stale-RAM rejection
while returning to the vendor path that produced the earlier RF-dependent IQ.

The first hybrid hardware run proved binary transport and full sentinel
replacement, but also exposed two stricter requirements. Three consecutive
blocks were perfectly constant (`I=101`, `Q=40`) despite all 16384 sentinel
words being replaced, so replacement alone is not proof of usable IQ. On the
fourth call a tick interrupt entered `int_wdt.c:reconfigure_ticks` while
`adctrig()` owned SRAM and caused a store-access panic. The decoded backtrace
placed `adctrig` below the interrupt handler and watchdog tick hook.

The LP clone is therefore no longer run before the vendor capture. The complete
vendor path is called inside a short critical section (successful hardware
calls measured about 2 ms), preventing the tick hook from running during SRAM
ownership changes. Acceptance additionally requires at least one percent of
adjacent words to differ. A fully overwritten but constant block now fails
closed and is never transported as video IQ.

## Stable variance-validated hardware run

The next physical XIAO run validated that stricter path. One preview session
completed 34 consecutive 16384-word captures with all of the following true:

- every kernel returned `done=1`, `timeout=0` and `final_control=00004000`;
- all 16384 sentinel words changed in every capture;
- 16367 to 16378 adjacent words changed in each block, rejecting the earlier
  constant-buffer failure mode;
- every capture produced all 64 independently CRC-protected type-5 chunks with
  `write_failures=0` and ended with `C5VRX_CAPTURE_DONE code=0`;
- I/Q means, extrema and average power varied from block to block; and
- no Store access fault, watchdog panic or reboot occurred. Preview teardown
  completed with `C5VRX_USB_PREVIEW state=STOP code=0`.

The finite-fill time was 1.928 to 1.985 ms, corresponding to a repeatable
8.25 to 8.50 MS/s finite-fill estimate (normally about 8.45 MS/s). This remains
an estimate that includes trigger overhead, not a calibrated physical sample
clock. The durable Windows session recorded 34 capture kernels, 34 binary ends
and 34 successful completions, with no panic marker.

This result proves repeatable, fresh, varying finite IQ acquisition and robust
binary USB delivery. It does not prove live PAL video. A 16384-sample block at
the estimated cadence spans only about 1.94 ms, while PAL frames span 20 ms.
Observed command/capture/USB cycles delivered only about 3.4 to 3.6 blocks per
second at roughly 1.9 Mbit/s. More than 99 percent of the source timeline is
therefore absent, and every block starts at an unrelated video phase. Painting
candidate rows from successive retriggered blocks can provide a WBFM diagnostic
or line montage, but must not be described as a decoded live video frame.

True live output requires a continuous baseband producer plus on-device WBFM,
filtering and substantial reduction before native Full-Speed USB. If the C5
PHY cannot expose a safe continuous DMA path, an external analog receiver or
SDR front-end is required; the public Wi-Fi promiscuous API and the recovered
finite `adctrig()` diagnostic are not sufficient by themselves.

## Ultra-slow video-proof host mode

The first PC renderer incorrectly used `finite_fill_msps` as the PAL raster
sample clock. That number measures total vendor-call duration and is explicitly
uncalibrated; it cannot establish the physical sample cadence. A second GUI bug
redrew the diagnostic WBFM waveform whenever the preview canvas was configured,
which could overwrite a video raster that had just been assembled.

The Receiver Console now keeps the decoded raster visible and offers explicit
20, 40 and 80 MS/s raster-clock hypotheses, defaulting to 40 MS/s. It removes
the measured block I/Q DC offset before phase differencing, folds each finite
block at the selected 15625 Hz PAL line period, searches both FM polarities for
the repeated horizontal-sync plateau, reduces active portions to 160 pixels,
and uses device packet timestamps modulo the 20 ms PAL field period to fill a
160x120 grayscale raster slowly. Coverage and horizontal-sync fold score remain
visible so ordinary RF noise is not mislabeled as decoded video.

Offline replay of the 34-block hardware session exercised every new rendering
path and filled 67 of 120 rows at the 40 MS/s hypothesis without protocol
errors. The resulting raster was static-like and did not establish convincing
PAL lock or a recognizable scene. The new mode therefore fixes the display and
clock-assumption defects and provides a direct hardware proof experiment, but
the saved session itself remains IQ/RF evidence rather than video evidence.

A later physical run again delivered more than twenty consecutive, varying
16384-word blocks at roughly 8.42 to 8.49 MS/s finite-fill estimate, with 64/64
CRC chunks, zero USB write failures and clean preview teardown. The operator
still saw the blue WBFM diagnostic graph. Session errors contained no host
decoder exception, and the running process resolved to the newly built XIAO
console. The remaining ambiguity was therefore in presentation rather than RF
capture or transport: a retained diagnostic canvas could be mistaken for video
progress, and a failed grayscale-image render could leave it visible.

GUI build `video-proof-2` removes the WBFM canvas renderer entirely. It shows
the build identifier in the header, paints a black 160x120 raster immediately
when the proof starts, and emits `C5VRX_HOST_VIDEO_FRAME` after the first and
every tenth host raster update. This makes three outcomes unambiguous: an old
binary still shows a waveform, a host-render failure never reports a raster
frame, and a working renderer shows grayscale raster data. These changes do
not upgrade the RF evidence to PAL video proof; recognizable scene content and
credible horizontal synchronization are still required for that claim.

During the next reconnect attempt, the newest session recorded zero incoming
serial bytes even though COM10 remained present. An exclusive direct pyserial
probe reproduced the failure: the port opened but writes timed out, and after
a Windows PnP restart one write succeeded without any response. A subsequent
reset-line transition immediately made that handle stale again. This exposed a
host-side native-USB hazard in the ordinary connect path: constructing
`serial.Serial(port, ...)` opens the device with pyserial's default DTR/RTS
state before the application can release those lines. On ESP32-C5
USB-Serial/JTAG, that transition may reset the device during connect.

GUI build `video-proof-3` now constructs a closed serial object, sets DTR and
RTS false, and only then opens COM10. Failed candidates are explicitly closed.
This prevents the console's normal Connect action from generating an
unintended reset transition. It cannot revive a C5 that is already stuck; that
still requires a real board reset or removal of every power source once.

After the runtime recovered, host markers proved that raster processing was
active (`frame=1` and `frame=10`), but the placeholder remained visible. A
standalone test against the exact Windows Tcl/Tk runtime reproduced the cause:
both raw and base64 PGM inputs failed with `image format "PGM" is not
supported`, while direct `PhotoImage.put()` RGB pixels succeeded. The earlier
host-frame marker was emitted after `_show_gray_frame()` even when that method
had returned early, so it proved decoding but not successful presentation.

GUI build `video-proof-4` uses direct grayscale-to-RGB PhotoImage rows, keeps a
scaled display-image reference, and integer-scales the 160x120 raster to fit
the preview canvas. `_show_gray_frame()` now returns an explicit success flag;
`C5VRX_HOST_VIDEO_FRAME` is emitted only after successful rendering, while a
failure emits `C5VRX_HOST_VIDEO_RENDER_ERROR` and preserves its error status.

## VTX-off versus VTX-on hardware comparison

The operator performed two consecutive A1 / 5865 MHz, 40 MHz preview runs and
identified the first as VTX off and the second as VTX on. This produced the
first controlled evidence that the displayed raster responds to the intended
RF source rather than only to an internal or host-generated pattern.

With the VTX off, average block power was normally about 2300 to 4800 and the
40 MS/s PAL horizontal-sync fold score stayed near 0.21 to 0.34. I/Q extrema
were usually within roughly -180 to +190. With the VTX on, average power was
commonly about 20000 to 70000, with an observed peak above 108000, and the fold
score rose to 0.50 to 1.57. Many VTX-on blocks reached the signed 10-bit rails
at -512 and +511. Both runs continued to deliver complete finite captures and
successfully rendered host frames; each long run had one recoverable USB
payload-CRC resynchronization.

The large and repeatable off/on separation proves RF-source-dependent IQ and
raster data. The stronger 15625 Hz fold score is encouraging evidence of
line-periodic structure at the 40 MS/s clock hypothesis, but it is not yet a
stable PAL lock or recognizable video proof. Frequent rail hits also indicate
clipping or overload, which can destroy the FM phase information needed for a
clean image. The next experiment should reduce VTX coupling (lower power,
greater distance or attenuation) until most I/Q extrema remain inside roughly
+/-250 to +/-350, then compare the 20, 40 and 80 MS/s raster hypotheses while
looking for a stable image correlated with camera motion.

Offline decoding of the complete CRC-framed session later exposed a more
specific timing error. A broad horizontal-period search over VTX-on captures
peaked at about 5088 source samples per line. This is consistent with an
approximately 80 MS/s source and the 15.734 kHz NTSC line rate, whereas the
GUI had defaulted to a 40 MS/s PAL fold. That old setting effectively treated
each physical line as roughly two candidate lines and used a 20 ms PAL field
period for vertical placement, explaining why the raster responded strongly
to the VTX but did not resemble the camera scene.

GUI build `video-proof-5` defaults to the 80 MS/s hypothesis, evaluates PAL
and NTSC sync candidates, remembers the stronger standard across captures,
normalizes unknown discriminator polarity, and refuses to repaint pixels when
the horizontal-sync score is below 0.65. This prevents ordinary VTX-off noise
from being contrast-stretched into a fake full-frame image. The companion
`tools/analyze_receiver_session.py` utility reads the raw mixed ASCII/binary
session log, validates packet CRCs, reassembles finite IQ chunks, segments
capture runs, and reports power, clipping and PAL/NTSC fold scores.

For a future genuinely continuous path, the preferred USB unit is neither one
raw-IQ capture nor one USB transaction per video line. WBFM, synchronization
and pixel reduction must happen on the device; then groups of roughly 8 to 16
decoded lines should be sent as CRC-protected strips. This retains low latency
without paying framing and scheduling overhead for every line. Complete
160x120 GRAY8 frames are only 19200 bytes and remain a useful simple fallback.
The unsolved bottleneck is continuous phase-bearing RF acquisition, not the
bandwidth required for a reduced grayscale image.

## Phase8 finite-capture acceleration

GUI/firmware build `video-proof-6` adds a compact finite-capture path without
moving PAL/NTSC reconstruction onto the C5. `CAPTURE PHASE8 16384` still uses
the same bounded, variance-validated vendor capture. Firmware calculates the
block I/Q mean, removes that DC offset, quantizes every complex sample angle to
one unsigned byte through a small lookup table, and sends packet type 6 in 1024
sample CRC-protected chunks.

This changes a 16384-sample capture from 65536 raw data bytes in 64 type-5
chunks to 16384 phase bytes in 16 type-6 chunks. Sample count and nominal RF
detail are retained; only absolute amplitude and fine phase precision are
discarded. The PC unwraps adjacent phase bytes into the FM discriminator and
continues to perform binning, PAL/NTSC selection, horizontal-sync scoring,
polarity correction and 160x120 rendering. New firmware advertises
`phase8_capture=1`; the same exe automatically falls back to raw type-5 IQ for
older firmware.

The first physical `video-proof-6` run reached the new command and completed
the 16384-word RF capture, but returned code 257 (`ESP_ERR_NO_MEM`) before the
binary-begin marker. Wi-Fi initialization had left no contiguous 16 KiB
internal-heap allocation for the original whole-capture phase buffer. Build
`video-proof-7` removes that allocation: it encodes and immediately transmits
one 1024-byte chunk through fixed static scratch storage, so peak Phase8
working storage is bounded and independent of capture length.

The revised firmware, protocol decoder and Windows host path compile and pass
their offline framing checks. The expected transport reduction is fourfold,
but an actual block rate must be measured after flashing `video-proof-7`. This
remains a retriggered finite source and does not solve the gaps between
captures. `tools/analyze_receiver_session.py` understands both raw-IQ32 and
Phase8 session packets, although Phase8 deliberately cannot report absolute
power or clipping because those amplitude values are no longer transported.

## Circular vertical-field alignment

The finite blocks expose only a few consecutive NTSC lines and normally do not
contain a complete vertical-sync interval. Absolute device timestamps can place
those lines consistently within a 16.68 ms circular field, but the circle's
zero is unrelated to the camera's vertical blanking. The resulting picture can
therefore be geometrically correct yet wrap roughly one third of the scene from
the bottom to the top, or vice versa.

Host build `video-proof-8` keeps timestamp-based row placement, then searches
the sufficiently populated circular raster for a stable dark vertical-blanking
window followed by active video. It requires at least 70 percent row coverage,
a minimum blanking score and the same candidate twice before rotating only the
displayed rows. The capture-time raster remains untouched. It also replaces
per-block percentile stretching with a slowly adapting shared black/white
scale; otherwise each two-to-three-line block can acquire a different contrast
and create false horizontal strip boundaries. An offline synthetic seam test
validates the circular rotation, but real-camera convergence still requires a
physical `video-proof-8` run.
