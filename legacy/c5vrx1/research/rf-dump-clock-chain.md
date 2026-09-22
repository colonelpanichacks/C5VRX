# ESP32-C5 RF dump clock and data chain

Scope: ESP-IDF v6.0.2, ESP32-C5 libraries and public headers, with C61/C6/C3
binary comparisons used only as semantic clues. No address or value is copied
between chips. Evidence words in this document are normative.

## Corrected top-level result

`0x60095004` is **not** an RX sample-clock register. The public C5 header names
it `HP_SYSTEM_SRAM_USAGE_CONF_REG`. Its fields are:

| Register / bits | C5 vendor behavior | Public or cross-chip analogue | Likely role | Confidence | Safe experiment |
|---|---|---|---|---|---|
| `0x60095004[11:8]` | `adctrig` writes 2 | `HP_SYSTEM_SRAM_USAGE`; header says CPU versus MAC-dump HP-memory access | SRAM owner/port selection | **PROVEN STATICALLY** | no arbitrary values; vendor-observed 2 only |
| `0x60095004[16]` | set before enable, cleared in teardown | `HP_SYSTEM_MAC_DUMP_ALLOC`; header says add 64 KiB MAC-dump offset | selects the observed dump bank/window | **PROVEN STATICALLY** | vendor-observed set only |
| C61 equivalent field | vendor binary writes field value `0x10` plus allocation bit | wider C61 field | chip-specific SRAM client code | **LIKELY** analogue | never copy to C5 |
| C6 equivalent field | vendor binary writes field value 4 | C6-specific layout | chip-specific SRAM client code | **LIKELY** analogue | never copy to C5 |

This also explains why the fixed ring must be reserved from both static DRAM
and the heap. C5VRX now uses `SOC_RESERVE_MEMORY_REGION(0x40830000,
0x40840000, ...)` and a link-time `_bss_end` assertion. Before that change the
RF-probe image placed roughly 20 KiB of application `.bss` inside the hardware
window; running the producer could have corrupted its own state.

The one named dump-clock control found is `0x600a9c04`, public
`MODEM_SYSCON_CLK_CONF_REG`:

| Register / bits | Write/read sites | C5 behavior | Likely role | Confidence | Safe experiment |
|---|---|---|---|---|---|
| `0x600a9c04[31]` | `adctrig`, `rftest_open_clk`; public header | initial all-ones write asserts it | data-dump module clock gate | **PROVEN STATICALLY** | vendor sequence only |
| `0x600a9c04[21]` | `adctrig`; public header names `CLK_DATA_DUMP_MUX` | modes 0/11 retain 1; mode 12 clears to 0 before enable | two-way dump-clock source mux | **PROVEN STATICALLY** field/action; source frequencies **UNKNOWN** | measure 0/11 only; mode 12 starts BLE RX and is rejected |
| `0x600a9c08[31]` | public header only | no producer write recovered | dump clock force-on | **CANDIDATE**, unused by audited producer | do not write |
| `0x600a9c10[31]` | public header only | no producer write recovered | dump module reset | **CANDIDATE**, unused by audited producer | do not write |

No named data-dump divider exists in the public `MODEM_SYSCON` block. The real
cadence can therefore be inherited from the selected FE/BB source rather than
being divided inside the writer. Modes 0 and 11 retain the same source and
clock mux, so their live pointer cadence is the safe comparison. Mode 12 also
calls `ble_rx_start(0, 0)` and is not eligible for the 5.8 GHz experiment.

## Upstream chain

The narrowest chain supported by static evidence is:

```text
5 GHz RF / mixer
  -> RF ADC (rate set indirectly by PHY band configuration)
  -> digital RX FE (filter mode configured by PHY)
  -> UNKNOWN placement of AGC / CFO correction / channel decimation
  -> mode/source selectors at 0x600a70b8 and 0x600a9008
  -> data-dump clock mux at MODEM_SYSCON[21]
  -> hardware dump writer
  -> HP SRAM ownership/64-KiB allocation
  -> 0x40830000..0x4083ffff
```

Relevant upstream controls found in C5 PHY code:

| Register / bits | Write sites and behavior | Cross-chip clue | Interpretation | Confidence | Experiment policy |
|---|---|---|---|---|---|
| `0x600a0448[1:0]` plus PHY I2C | `phy_adc_rate_set(bool)` is selected by band/frequency/bandwidth control flow | analogous band-dependent ADC-rate helpers exist on close chips | RF ADC-rate family selection | function/branch **PROVEN STATICALLY**; boolean meaning and physical Hz **UNKNOWN** | never force; log vendor state only |
| `0x600a0430[21:18]` | `phy_rx_filter_mode`; selection depends on band, frequency and channel-width state | close PHYs also select band/channel filters | RX digital filter mode | writes/branches **PROVEN STATICALLY**; exact 5805 response **UNKNOWN** | keep PHY-selected value; measure bandwidth |
| `0x600a790c[7:2]`, `0x600a7c00[30]` | `phy_chan_dump_cfg` helpers; no audited C5 caller in the dump path | C6 wrapper delegates to ROM channel-dump configuration | possible channel-dump/filter tap setup | **CANDIDATE** | do not call or write yet |
| `0x600a9c04[21]` | vendor mode 12 clears it and then starts BLE RX | named dump-clock mux | alternate Bluetooth diagnostic clock/domain | action/call **PROVEN STATICALLY**; rate **UNKNOWN** | do not use for 5.8 GHz capture |

The historical `sample_80m` write to `0x600a9008[23:21]` is overwritten by the
later mode write over `[24:17]` before capture enable. All eight historical
values remain `PROVEN_STATICALLY_OVERWRITTEN_BEFORE_ENABLE`; none is a valid
final-rate selector and firmware must not restore or force them.

## Tap-location verdict

The existence of band/frequency/channel-width-dependent receive filter
selection is **PROVEN STATICALLY**.
Its position relative to the dump tap is **UNKNOWN**. No xref proves whether
the 10+10 words are pre/post filter, decimator, CFO correction, or AGC. Mode 12
changes the named dump-clock mux plus additional FE-path controls and starts
BLE RX. It is not a lower-rate 5.8 GHz candidate.

No vendor-observed selector code beyond source-mux codes 0/1 and producer modes
0/11 was found that can safely be promoted for 5.8 GHz. The automated
mode-0/11 cadence,
`TONE RESPONSE PROBE`, phase-continuity and wrap probes are therefore the
terminal evidence path; another generic SDR/API search is not warranted.

The consolidated bandwidth/sample-rate and WBFM verdict is in
[`rf-iq-dump-verdict.md`](rf-iq-dump-verdict.md).
