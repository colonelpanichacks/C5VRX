# ESP32-C5 RF/IQ dump path verdict

Scope: ESP-IDF v6.0.2, ESP32-C5 `librftest.a` SHA-256
`0f9680b41612762d854accf2334412e3e01b206a3ec290bbbb232842fdaec7ba`.
All register descriptions below are either public names or exact observed bit
operations. An inferred stage is never presented as a register definition.

## Answer

**The dump contains phase-bearing complex receive I/Q, but it is not yet proven
to retain the complete analog-FPV WBFM channel. Confidence: high for complex
I/Q; low for the exact transfer function; medium that mode 0 is the best
available 5.8 GHz candidate.**

Mode 0 is the first physical-test mode. It uses the ordinary C5 receive dump
source and, after enable, the vendor code pulses a software-trigger bit. It
therefore does not require a valid 802.11 packet, packet preamble, RX-start or
RX-end event. It does require the RF/PHY to be initialized and tuned, and its
samples can still have passed through RX filtering and automatic gain control.

Mode 11 selects a different internal trigger/debug-matrix encoding but has no
proven source, ADC-rate, clock-mux or bandwidth advantage over mode 0. Mode 12
is not a 5.8 GHz candidate: the vendor branch starts Bluetooth reception with
`ble_rx_start(0, 0)` and selects an alternate dump clock/path. The split C5VRX
producer now rejects mode 12 rather than executing an incomplete subset of
that vendor sequence.

## Evidence-ranked receive chain

```text
5 GHz antenna / RF mixer
  -> RF ADC
  -> PHY-selected receive filtering and channel-width configuration
  -> UNKNOWN exact placement of AGC, channel decimation and CFO processing
  -> ordinary receive/"BB dump" source mux (mode-zero set_dump_mode path)
  -> mode 0 or mode 11 trigger/debug selection
  -> modem data-dump writer
  -> fixed 64 KiB HP-SRAM window at 0x40830000
  -> signed Q10 + I10 words
```

The first two and last two transitions are strongly supported by C5 symbols,
machine code and public register names. The middle ordering is deliberately
marked unknown. No public C5 block diagram exposes the diagnostic tap, and no
xref from `adctrig` identifies a named pre-filter, post-filter, pre-AGC,
post-AGC, pre-FFT or post-CFO node.

The official C5 documentation establishes 5 GHz operation and 20/40 MHz Wi-Fi
channel support. It does **not** document an SDR port, the dump tap, dump sample
rate, or its passband:

- [ESP32-C5 datasheet](https://documentation.espressif.com/esp32-c5_datasheet_en.html)
- [ESP-IDF ESP32-C5 Wi-Fi overview](https://docs.espressif.com/projects/esp-idf/en/stable/esp32c5/api-guides/wifi-driver/overview.html)
- [Espressif ESP32-C5 RF test guide](https://docs.espressif.com/projects/esp-test-tools/en/latest/esp32c5/development_stage/rf_test_guide/rf_test_guide.html)

## What processing precedes the dump?

| Item | Finding | Confidence |
| --- | --- | --- |
| Complex sample format | `print_dump_data` and `accumiq` independently decode Q from bits 0..9 and I from bits 10..19 as signed 10-bit values. | **High / proven statically** |
| Packet demodulation | No call from `adctrig` enters the 802.11 packet demodulator, FFT or packet parser. Mode 0 produces its own trigger pulse. | **High negative static result**; exact hidden hardware routing remains unknown |
| Wi-Fi receive state | RF/PHY initialization and tuning are prerequisites. A decoded packet or packet-trigger event is not required by mode 0. | **High for mode 0** |
| AGC | `phy_chip_set_chan` disables AGC around retuning and re-enables it. C5VRX uses the automatic-gain `adctrig` branch and does not force gain, so ordinary AGC is expected to remain active during capture. | **Medium-high** |
| Channel filtering | `phy_chip_set_chan`, `phy_rfpll_set_adc_rate`, `phy_rx_filter_mode` and `phy_bb_cbw_chan_cfg` configure band/frequency/channel-width-dependent ADC/filter/baseband state before capture. | **High that filtering exists; low for exact dump response and tap ordering** |
| Decimation | No surviving dump-specific divider was recovered. Decimation may be inherited from the selected FE/BB node. | **Unknown** |
| CFO correction | No dump-path call or named register proves whether carrier-frequency correction precedes the tap. | **Unknown** |
| Wi-Fi packet gating | Mode 0 is software-triggered. Mode 11's internal trigger condition is unnamed and may depend on PHY activity. Mode 12 starts BLE RX. | **High / medium / high**, respectively |

The historical public RF-test decoder calls source-mux code 1 a “bb dump” and
shows gain/error/AGC metadata in the upper word bits on older chips. That is a
useful family clue, not proof of identical C5 upper-bit semantics or tap
placement. See the pinned historical
[`adc_dump.py`](https://github.com/qiuzhi12345/eagletest/blob/19c5ddacddeb53ff245f65f206a3bfd7106e03cf/eagletest/py_script/rftest/rflib/adc_dump.py).

## Modes 0, 11 and 12

The second `adctrig` argument is a **trigger/debug mode**, not a documented
sample-rate mode. Exact C5 behavior:

| Mode | Source/clock facts | Trigger fact | Likely bandwidth/rate | 5.8 GHz verdict |
| ---: | --- | --- | --- | --- |
| 0 | ordinary `set_dump_mode(0)` source; normal data-dump clock mux | pulses `0x600a9004[19]` high then low after enable | Same PHY-selected receive chain as mode 11. 40 or 80 MS/s are historical family design candidates, but exact C5 rate is unproven. Effective passband is likely bounded by the configured HT40 receive chain. | **Best/widest candidate; test first** |
| 11 | same source mux, FE path and dump-clock mux as mode 0; different four 6-bit selectors in `0x600a9018` and trigger encoding in `0x600a9008[24:17]` | internal condition unnamed | No static evidence of a different cadence or passband. Most likely same source cadence with a different trigger. | **Secondary comparison only** |
| 12 | alternate FE controls and clears public `MODEM_SYSCON_CLK_DATA_DUMP_MUX` | calls `ble_rx_start(0, 0)` | Numeric rate and format cannot be assigned. It belongs to a Bluetooth diagnostic route, not a proven lower-rate 5 GHz tap. | **Rejected for 5.8 GHz capture** |

The values written into the four 6-bit `0x600a9018` fields are consecutive
groups for modes 0 and 11 (`24..27` versus `20..23`). Their shape and placement
make trigger/debug-matrix selectors more likely than sample packing or timing,
but the public C5 headers do not name those fields. That interpretation remains
a hypothesis.

The old parameter named `sample_80m` does not establish 80 MS/s on C5. Its
write to `0x600a9008[23:21]` is overwritten by the later encompassing mode
write before enable. Historical older-chip tooling distinguishes 40 and
80 MHz views, so 40/80 MS/s are reasonable measurement hypotheses, not facts.
Mode 0 and mode 11 cadence must be read from the live pointer; mode 12 must not
be assigned a Wi-Fi-domain sample rate.

## Does it retain analog FPV WBFM?

Three facts make it plausible:

1. the words preserve complex phase, which is sufficient in principle for an
   FM phase discriminator;
2. mode 0 is not packet-triggered and can capture arbitrary energy in the
   tuned receive channel;
3. the C5 is configured for a 40 MHz Wi-Fi receive channel at A4/5805 MHz,
   wider than a representative analog-FPV occupied-bandwidth estimate.

None proves the passband seen at the dump. A channel filter could be narrower,
non-flat, decimated, gated, DC-notched or otherwise unsuitable, and normal AGC
can change amplitudes between captures. Therefore the current verdict is:

> **FULL ANALOG FPV WBFM: PLAUSIBLE, NOT PROVEN.**

Confidence that mode-0 data is usable complex receive I/Q: **high**. Confidence
that it contains the entire WBFM/video-sync information with adequate fidelity:
**low until physical tone-sweep and VTX evidence exist**.

## Decisive safe hardware test

Use only the existing bounded and hash-pinned PR #3 path:

1. Tune HT40 to A4 / 5805 MHz and measure mode-0 pointer cadence.
2. With a coherently generated, attenuated RF tone, sweep positive and negative
   offsets using `TONE RESPONSE PROBE 0`. Record amplitude, coherence and
   observed phase slope. This establishes usable passband and sample rate.
3. Capture identical bounded windows with the VTX off, then on with a known PAL
   test card and moving marker using `python tools/c5vrx_lab.py vtx-proof`.
4. Require changes in RF power/spectrum, I/Q distribution, WBFM discriminator
   output and video-sync evidence. A power-only change is insufficient.
5. Preserve the raw serial log, IQ words, parsed JSON and preview frames in the
   session bundle. Do not enable production live reception from this result
   alone.

Mode 11 may be compared after mode 0 if its trigger can be made repeatable.
Mode 12 is intentionally skipped. No RF register, filter, clock or gain value
outside the pinned vendor-observed mode-0/11 sequences is needed for this test.

## Static reproduction

```bash
. /root/esp-idf-v6.0.2/export.sh
python tools/audit_rf_dump_producer.py --idf "$IDF_PATH"
python tools/audit_rf_streamability.py --idf "$IDF_PATH" --out audit.md
```

The producer audit now verifies the mode-0 software-trigger pulse, verifies
that the pinned vendor mode-12 branch calls `ble_rx_start`, and verifies that
the split producer fails closed for mode 12.
