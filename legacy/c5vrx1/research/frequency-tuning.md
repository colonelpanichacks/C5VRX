# Frequency tuning research

C5VRX is not limited to Band A. The ESP32-C5 receive hardware covers the 5 GHz region used by most legacy analog FPV channels. The public ESP-IDF Wi-Fi driver gives us a supported way to bring up the 5 GHz RF chain on normal Wi-Fi channel centers; undocumented PHY functions are being investigated for exact arbitrary-frequency retuning.

## FPV frequency set

The firmware channel database mirrors the classic Betaflight factory table:

- **A:** 5865, 5845, 5825, 5805, 5785, 5765, 5745, 5725 MHz
- **B:** 5733, 5752, 5771, 5790, 5809, 5828, 5847, 5866 MHz
- **E:** 5705, 5685, 5665, 5645, 5885, 5905, 5925, 5945 MHz
- **F:** 5740, 5760, 5780, 5800, 5820, 5840, 5860, 5880 MHz
- **R:** 5658, 5695, 5732, 5769, 5806, 5843, 5880, 5917 MHz

The current C5 hardware target is **5645–5885 MHz**. Channels above 5885 MHz are deliberately marked out-of-window rather than pretending they are supported.

## Supported bootstrap path

For every requested FPV frequency, C5VRX computes the nearest normal 5 GHz Wi-Fi center. The default RF backend uses ESP-IDF's normal Wi-Fi driver in 5 GHz-only mode, sets 20/40 MHz bandwidth, tunes the standard Wi-Fi channel, enables receive/promiscuous mode, and reads the channel back.

That gives us two useful cases before any undocumented retune:

1. **Exact overlap** — no frequency hack required. Example: 5805 MHz = Wi-Fi channel 161.
2. **Near-center HT40 experiment** — start on the closest Wi-Fi center and observe a nearby analog carrier inside the wide receive passband. Example: RaceBand 5 at 5806 MHz is only +1 MHz from channel 161.

Promiscuous mode does **not** expose analog FPV samples. It is only a supported way to get the C5's 5 GHz receive chain onto a known RF center while we work on the front-end dump path.

## Direct frequency hook — corrected ABI finding

ESP32-C5 ESP-IDF v6.0.2 static disassembly gives us a much stronger result than the symbol name alone:

```asm
phy_set_freq:
    ...
    sh      a1, 30(phy_param)
    lbu     a1, 291(phy_param)
    tail    phy_chip_set_chan
```

So **`phy_set_freq` definitely consumes two arguments on this C5 blob**. The earlier one-argument guess was wrong and has been removed.

An independent ESP32-C6 implementation in the Hubble Network device SDK declares the same function family as:

```c
extern void phy_set_freq(uint16_t freq_mhz, int offset);
```

It then calls `phy_set_freq(base_frequency_mhz, offset)` for fine frequency stepping. This agrees with the C5 register usage far better than the old one-argument declaration.

C5VRX therefore currently uses the experimental C5 call shape:

```c
extern void phy_set_freq(uint16_t freq_mhz, int offset) __attribute__((weak));
```

For ordinary FPV channels, which are integer MHz values, the first experiment is:

```c
phy_set_freq(5806, 0);   // RaceBand 5
```

The second argument's fine-step scale is not needed for integer-MHz FPV centers and remains intentionally undocumented in C5VRX until verified.

### Important

This is still **not a vendor-supported API**. The two-argument calling convention is now supported by actual C5 v6.0.2 disassembly, but the semantic meaning and side effects must still be verified on real C5 hardware. Direct retuning remains opt-in in menuconfig.

## Why the C5 disassembly is encouraging

`phy_chip_set_chan`, which `phy_set_freq` reaches, performs the full receive-chain retune sequence: it calls frequency conversion/setup code, disables AGC during the change, toggles the RF/ADC path, updates RFPLL state, calls `phy_set_channel_rfpll_freq_new`, updates baseband channel-width configuration and later restores the receive chain.

That is much more promising than a function that merely changes a software channel variable.

The same blob also contains:

```text
phy_set_chanfreq
phy_set_rfpll_freq
phy_set_channel_rfpll_freq_new
phy_write_chan_freq
phy_chan_to_freq
phy_freq_to_chan
phy_chip_set_chan_offset
```

These are being included in the generated CI disassembly report so their call sites and argument patterns can be compared.

## Validation plan

1. Bring the C5 up on a normal 5 GHz center using the public ESP-IDF Wi-Fi backend.
2. Verify readback of the requested Wi-Fi channel.
3. Use a controlled RF source/VTX at the exact standard center as the baseline.
4. Move the source to R5 / **5806 MHz** while remaining on ch161 / 5805 MHz HT40.
5. Enable the experimental `phy_set_freq(5806, 0)` call.
6. Verify the actual receive center moved using a narrowband source or repeatable receive/dump metric; do not infer success from a lack of crashes.
7. Repeat at larger offsets only after the +1 MHz test is repeatable.
8. Only then use arbitrary tuning while capturing front-end/baseband dump data.

The best first target remains **R5 / 5806 MHz** because the expected shift from the supported 5805 MHz bootstrap center is only 1 MHz and can be tested without changing the rest of the RF setup.
