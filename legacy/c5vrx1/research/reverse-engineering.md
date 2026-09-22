# Reverse-engineering notes

For the reproducible v6.0.2 machine-code audit, public API survey and final
continuous-stream verdict, see
[`continuous-rf-verdict.md`](continuous-rf-verdict.md). The key new result is
that `adctrig()` exposes a live pointer while armed but disables the engine
before returning; no public C5 GDMA/BitScrambler Wi-Fi-FE attachment exists.

## Mission

C5VRX investigates whether an ESP32-C5 can be repurposed as a low-cost analog 5.8 GHz FPV receiver, primarily as an RX5808/RTC6715-class alternative during the current receiver-module shortage.

The desired end state is:

```text
5.8 GHz analog FPV WBFM
        |
        v
ESP32-C5 RF front-end
        |
        v
raw/near-raw I/Q or equivalent receive samples
        |
        v
WBFM discriminator
        |
        v
PAL/NTSC composite baseband
        |
        +--> video decoder / display pipeline
```

## What is established

1. ESP32-C5 has a real 5 GHz receive chain and the normal ESP-IDF Wi-Fi API supports selecting 5 GHz-only operation and standard 5 GHz Wi-Fi channels.
2. Normal 5 GHz Wi-Fi channel centers overlap seven Band-A channels exactly; C5VRX nevertheless carries the classic A/B/E/F/R tables and plans all channels against the closest supported Wi-Fi center.
3. The current direct hardware target is **5645–5885 MHz**. Legacy FPV channels above 5885 MHz are intentionally marked out-of-window.
4. The generic `esp_phy_cert_test.h` RX API in ESP-IDF v6.0.2 documents channel values **1–14**. C5VRX therefore no longer assumes that `esp_phy_wifi_rx(161, ...)` is a valid 5 GHz API. The public Wi-Fi driver is the default 5 GHz bring-up backend.
5. Packet/promiscuous receive data is not analog video. We still need a lower-level front-end/baseband sample path.
6. ESP32-C5 `libphy.a`, `librftest.a`, and ROM symbols expose substantial ADC/IQ/dump/debug machinery worth reverse engineering.

## High-value finding: `phy_set_freq` ABI

The ESP32-C5 ESP-IDF v6.0.2 PHY blob contains `phy_set_freq`. Static disassembly shows it consumes **two arguments**, not one:

```asm
phy_set_freq:
    ...
    sh      a1, 30(phy_param)
    lbu     a1, 291(phy_param)
    tail    phy_chip_set_chan
```

An independent ESP32-C6 implementation in the Hubble Network device SDK uses:

```c
extern void phy_set_freq(uint16_t freq_mhz, int offset);
```

That two-argument shape is consistent with the C5 disassembly and replaces C5VRX's earlier one-argument guess. C5VRX keeps it weak and opt-in:

```c
phy_set_freq(5806, 0); // experimental integer-MHz R5 retune
```

This is still undocumented and must be validated on physical ESP32-C5 hardware before being treated as a working tuner API.

See [`frequency-tuning.md`](frequency-tuning.md).

## High-value finding: receive/sample dump path

The C5 RF-test binary exposes:

```text
adctrig
sampledeal
accumiq
get_rx_buffer
get_rx_data_addr
set_dump_mode
print_dump_data
loop_dump_test
fedump_rd_rxmem
fedump_rd_txmem
```

The C5 PHY binary/ROM surface separately exposes:

```text
phy_chan_dump_cfg
phy_chan_dump_cfg_752
phy_adc_rate_set
phy_fe_adc_on
phy_iq_est_enable_new
phy_iq_est_disable
phy_dc_iq_est_new
phy_rxiq_get_mis
phy_fft_scale_force
phy_csidump_force_lltf_cfg
phy_set_rx_pbus_freq
phy_get_rx_pbus_freq
```

A CI-generated v6.0.2 disassembly report has already confirmed these functions are real code paths, not just imagined names.

### Especially interesting: `sampledeal`

The current v6.0.2 RF-test blob implements `sampledeal` as a tiny signed conversion:

```asm
andi    tmp, sample, 0x200
andi    sample, sample, 0x3ff
beqz    tmp, positive
addi    sample, sample, -1024
```

That is a textbook conversion of a **10-bit two's-complement sample** to a signed integer. This does **not** yet prove that we can access continuous raw I/Q, but it is strong evidence that the RF-test code really processes low-level signed sample values.

### IQ estimator path

`get_iq_est_pwr` calls `phy_iq_est_disable`, then `phy_iq_est_enable_new`, and reads estimator state from the `0x600a04xx` MMIO region. `dc_iq_est_test` reads several adjacent registers in the same region. Again, these are estimator/measurement paths, not proof of streamable raw I/Q, but they confirm usable receive-side IQ-related hardware is exposed to the vendor test code.

### FE dump path

`fedump_rd_rxmem` itself is only a 2-byte `ret` stub in the linked static archive. That means it is not the useful implementation by itself. The more valuable targets are the orchestration/configuration around:

```text
loop_dump_test
set_dump_mode
print_dump_data
phy_chan_dump_cfg[_752]
```

The analysis tool is being improved to preserve local `.L*` labels correctly so it can recover complete function bodies and meaningful caller names instead of truncating at local branch labels.

See [`fe-dump-notes.md`](fe-dump-notes.md).

## Reverse-engineering order

### Phase 0 — supported 5 GHz baseline

Use the public ESP-IDF Wi-Fi backend to select 5 GHz-only operation, tune the nearest normal Wi-Fi channel and read the active channel back. Default target is R5 / **5806 MHz**, bootstrapped from ch161 / 5805 MHz HT40.

### Phase 1 — arbitrary-frequency validation

1. Verify the supported 5805 MHz bootstrap path.
2. Verify a 5806 MHz source is visible inside the 5805 MHz HT40 window.
3. Enable the two-argument `phy_set_freq(5806, 0)` experiment.
4. Confirm the receiver center actually changes with a controlled RF source.
5. Do not infer success merely from the function returning or the chip not crashing.

### Phase 2 — static binary analysis

Run:

```bash
python tools/analyze_phy.py
```

Focus on:

```text
loop_dump_test
  -> dump configuration / trigger / printer relationships

sampledeal
  -> signed sample representation

accumiq
  -> sample layout / I-Q accumulation

phy_set_freq
  -> frequency + offset semantics
```

For every target establish argument usage, return values, constants, MMIO accesses, callers/callees and buffer strides.

### Phase 3 — identify a receive dump buffer

Compare captures under:

1. no RF source,
2. carrier/VTX on,
3. static black image,
4. black/white pattern,
5. moving image.

A useful sample path should vary with the analog source even when no valid 802.11 packets are decoded.

### Phase 4 — classify sample format

Candidate formats include interleaved signed I/Q, separate I/Q banks, complex/magnitude FFT data, IQ-estimator accumulators or real post-filter samples.

For raw complex samples:

```text
z[n] = I[n] + jQ[n]
dphi[n] = arg(z[n] * conj(z[n-1]))
```

`tools/wbfm_demod.py` implements this offline and has a synthetic CI self-test.

### Phase 5 — continuous capture

A finite vendor dump is enough to prove the signal path. Live FPV then requires tracing the producer that feeds the finite debug memory and finding a practical ring-buffer/DMA/chunked path.

### Phase 6 — WBFM to composite

1. DC removal / IQ correction if needed
2. WBFM phase discriminator
3. analog-video baseband filtering
4. level normalization
5. PAL/NTSC sync handling
6. color recovery
7. low-latency display/output

Avoid full-frame buffering until the RF/sample path is proven.

## Safety rule

Undocumented PHY functions must not be called from guessed prototypes. The old one-argument `phy_set_freq(int)` experiment is a concrete example of why: real C5 v6.0.2 disassembly shows `a1` is consumed. Experimental calls remain compile-time/menuconfig opt-in until their C5 ABI and behavior are established.
