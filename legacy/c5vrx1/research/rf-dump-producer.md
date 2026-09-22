# ESP32-C5 RF dump producer reverse engineering

This report is a static, read-only audit of the ESP-IDF v6.0.2 binary archives.
It deliberately does not promote guessed register values to executable code.
The audited C5 `librftest.a` SHA-256 is
`0f9680b41612762d854accf2334412e3e01b206a3ec290bbbb232842fdaec7ba`.

## Exhaustive C5 address cross-reference

Every archive under `components/**/esp32c5/*.a` was disassembled.  Exact
`0x600a90xx` effective addresses and every `lui 0x600a9` owner were inspected,
as were all `lui 0x40830` owners.  This wider LUI pass is necessary because a
RISC-V object normally materializes an MMIO address as an upper immediate plus
a signed 12-bit displacement.

| Archive / function | Addresses | Access and purpose |
| --- | --- | --- |
| `esp_phy/lib/esp32c5/librftest.a:adctrig` | `0x600a9004`, `0x600a9008`, `0x40830000`; also `0x600a9018` through the same base | The only C5 RF-dump controller. Configures length, trigger, source/mode and format-related fields, enables capture, polls done, reads pointer/flags, and tears down. |
| `esp_phy/lib/esp32c5/librftest.a:dactrig` | `0x40830000` | Initializes the same 64 KiB RAM for DAC playback; no dump-control-register access. |
| `esp_phy/lib/esp32c5/librftest.a:loop_dump_test` | `0x40830000` | Calls `adctrig`, then passes the RAM base to `print_dump_data`; consumer/orchestrator only. |
| `esp_phy/lib/esp32c5/librftest.a:run_rftest_case` | `0x40830000` | Uses the address for `phy_freq_mem_backup`; unrelated shared-RAM consumer. |
| `esp_coex/lib/esp32c5/libcoexist.a:coex_hw_debug_matrix_config` | `0x600a9408` | Coexistence debug-matrix register, outside the `0x600a9000..18` dump controller. |
| `esp_phy/lib/esp32c5/libbtbb.a:bt_bb_ble_diag_all` | `0x600a9404` | Bluetooth diagnostic mux, outside the dump controller. |

No other C5 binary/library in that ESP-IDF tree touches an exact address in
`0x600a9000..0x600a90ff`, and no other function materializes `0x40830000`.
In particular, `libphy.a`, Wi-Fi, GDMA and BitScrambler libraries contain no
second producer setup path.  `adctrig` is the producer owner.

## Recovered C5 ABI

The old public test script calls:

```text
adctrig(smp_num_aft_trig, trig_mode, trig_case, sample_80m,
        dump_trig, rx_gain_mode, rx_gain, rx_gain0, gain0_wait)
```

The machine code consumes the same nine arguments (`a0..a7`, then one stack
word).  The historic trigger enumeration is software=0, BB=1, CCA=2,
RX-start=3, RX-end=4, TX-start=5, TX-end=6 and RX-error=7.

## Register fields established by code

| Question | C5 machine-code result | Confidence |
| --- | --- | --- |
| Capture enable | `0x600a9004[31]`; set to start, cleared on teardown | Proven |
| Circular/wrap enable | No separately written circular-enable bit was found. The engine exposes wrap status and uses fixed ring RAM; circular behavior still requires a live test. | Negative static result |
| Historical rate write (`sample_80m`) | `0x600a9008[23:21] = (argument >> 1) & 7`, then overwritten by the later mode write to `[24:17]` before enable | Proven transient write; physical rate meaning rejected |
| Sample format selector | Trigger/mode setup writes `0x600a9008[24:17]`; `set_dump_mode` separately selects FE/BB data at `0x600a08cc` and `0x600a70b8`. Exact format names per selector remain unproven. | Field proven, semantics partial |
| Buffer length | `0x600a9004[16:0] = smp_num_aft_trig + 1` | Proven |
| Buffer start/end | Fixed start `0x40830000`, reported size `0x10000` bytes, hence exclusive end `0x40840000` | Proven |
| Trigger source | Trigger argument 0..12 dispatches through a vendor jump table and programs `0x600a9008[24:17]`, plus source-specific FE controls. Mode 0 pulses `0x600a9004[19]` after enable; mode 12 calls `ble_rx_start(0, 0)`. | Proven mapping below |
| Current pointer | `0x600a9008[15:0]`, read after stop and printed/returned as `curr_ptr` | Proven |
| Wrap counter/flag | `0x600a9004[18]` is polled as completion/wrap status and returned as `wrap_flag`; no counter is read | Proven flag; no counter found |
| Decimation/filter | The transient three-bit `sample_80m` write is not active at enable. A named 5 GHz RX filter-mode write exists upstream, but its relation to the dump tap is unresolved. No callable dump decimator API was found. | Partial |
| Interrupt/watermark | None. The wrapper busy-polls `0x600a9004[18]`; no ISR registration, interrupt status, threshold, descriptor, or GDMA relocation occurs. | Proven negative |
| Lower-rate I/Q/baseband | Eight transient historical values are overwritten. Mode 11 changes trigger/debug selectors but has no proven rate change. Mode 12 starts BLE RX and is not a 5.8 GHz lower-rate candidate. | No lower-rate tap proven |
| Indefinite capture | The hardware is enabled before the polling loop and only software later clears bit 31. No hardware auto-disable is visible. Skipping teardown can therefore leave it armed in principle, but indefinite wrap behavior and RF/clock ownership require a physical test. | Strong static evidence, not yet a safety guarantee |

### `sample_80m` exactly: transient, not an active rate selection

The implementation computes:

```c
rate_field = sample_80m > 1 ? (sample_80m >> 1) : 0;
REG(0x600a9008) = (REG(0x600a9008) & ~0x00e00000)
                    | ((rate_field << 21) & 0x00e00000);
```

Consequently the eight transient writes are:

| Argument values | `0x600a9008[23:21]` | Register contribution |
| ---: | ---: | ---: |
| 0, 1 | 0 | `0x00000000` |
| 2, 3 | 1 | `0x00200000` |
| 4, 5 | 2 | `0x00400000` |
| 6, 7 | 3 | `0x00600000` |
| 8, 9 | 4 | `0x00800000` |
| 10, 11 | 5 | `0x00a00000` |
| 12, 13 | 6 | `0x00c00000` |
| 14, 15 | 7 | `0x00e00000` |

Values above 15 alias modulo the retained three bits, but are not called
"valid": the vendor code does not range-check them and no vendor caller was
found using them. Most importantly, values 0 and 1 are identical at the
hardware register. The historic parameter name and Python default cannot be
used to label field 0 as 80 MS/s. Because the values do not survive until
enable, they are not useful rate-probe candidates; cadence must be measured on
the actual running producer in vendor-observed modes 0 and 11. Mode 12 belongs
to a BLE receive branch.

However, the next mode-dispatch branch clears or overwrites the encompassing
`0x600a9008[24:17]` field before `0x600a9004[31]` is set. Thus the eight values
do **not** survive to an enabled ordinary capture. Static evidence does not
support calling this a divider, source-clock selector, full-rate selector, or
powers-of-two progression. Legacy `RATE PROBE ALL` records the overwritten
paths but is superseded by `PRODUCER CADENCE PROBE ALL`. The machine-readable
table is [`rf-dump-rate-fields.json`](rf-dump-rate-fields.json).

### Trigger/mode jump table

The second argument indexes 13 vendor-observed branches:

| Mode | Changes derived from branch |
| ---: | --- |
| 0 | Clears `0x600a20b4[0]`; ORs selector `0x01e00000` into `0x600a9008`; after enable pulses `0x600a9004[19]` high then low (software trigger) |
| 1 | Clears `0x600a20b4[0]`; selector `0x00000000` |
| 2 | Clears `0x600a20b4[0]`; selector `0x00000000` |
| 3 | Clears `0x600a20b4[0]`; selector `0x00000000` |
| 4 | Clears `0x600a20b4[0]`; selector `0x00020000` |
| 5 | Clears `0x600a20b4[0]`; selector `0x00040000` |
| 6 | Sets `0x600a9004[17]`, clears `0x600a20b4[0]`; selector `0x00080000` |
| 7 | Programs `0x600a4e38[8:0]` from `trig_case`; selector `0x000a0000` |
| 8, 9, 10 | Selector is `(mode << 17) & 0x001e0000` |
| 11 | Selector `0x00160000`; alternate constants in `0x600a9018` |
| 12 | Enables alternate path at `0x600a20b4[0]`, changes `0x600a20ac[31:29]`, clears `0x600a9c04[21]`, selector `0x00120000`, uses alternate `0x600a9018` constants, and calls `ble_rx_start(0, 0)` |

The table records exact writes/calls, not guessed human-readable names beyond
the historical 0..7 enumeration. The mode-0 pulse means an 802.11 packet event
is not required to trigger an ordinary capture. It does not mean the RX chain
is free of PHY-selected filtering or AGC.

## Minimal split configure/start/observe/stop reconstruction

`adctrig` can be split at instruction offset `0x1c2`: everything before it is
configuration, the write setting `0x600a9004[31]` is start, the loop beginning
at `0x2d2` is observe/wait, and offset `0x334` begins stop/restore.

The source reconstruction in `main/c5vrx_rf_dump_producer.c` implements the
observed automatic-gain subset for modes 0 and 11. It preserves the vendor RMW
masks, mode-specific setup, mode-0 software-trigger pulse and teardown call.
It refuses every other mode. In particular it refuses mode 12 because a
register-only reconstruction would omit the guarded vendor
`ble_rx_start(0, 0)` call. It is disabled by default, requires ESP-IDF v6.0.2
and makes CMake fail if the archive SHA-256 differs. It does not call internal
instruction addresses.

```text
rf_dump_configure(args): adctrig 0x000..0x1c0, excluding start
rf_dump_start():         adctrig 0x1c2..0x212 (enable plus mode-0 pulse)
observe_pointer():       read 0x600a9008[15:0] and 0x600a9004[18]
rf_dump_stop():          adctrig 0x334..0x37a, including FE restore
```

Calling an internal instruction address is not ABI-safe. A source
reconstruction must reproduce the complete blocks and pin itself to the blob
hash. Before enabling writes, a hardware probe should snapshot every touched
register before/after the normal vendor call and verify the reconstruction's
RMW masks. This meets the requirement that no unobserved value be written.

## Cross-chip comparison

C6, C61 and C3 v6.0.2 `librftest.a` archives contain substantial `adctrig`
implementations. C61 is the closest control-flow match: it performs the same
`sample_80m > 1 ? sample_80m >> 1 : 0` transform and writes the same
`0x600a9008[23:21]` field. C6 retains the length/trigger/poll/teardown skeleton
but has a different ABI and field layout. C3 uses the older `0x60033dxx`
controller and directly combines trigger and sample arguments, which helps
explain the historical host names but not C5 bit positions. H2 is an important
negative comparison: Wi-Fi `adctrig` is a two-byte `ret` stub; only its separate
802.15.4/Bluetooth-oriented `bt_adctrig` is implemented. Register constants
from any comparison chip must not be copied to C5.

No compared binary exports `sample_80m` as a function: it is a host-side name
for an `adctrig` argument. Older public scripts document it as a boolean, while
C5 machine code consumes it as the three-bit field above. That mismatch is the
main reason rates must be measured rather than assigned from the old name.

## Next experiment before making RING PROBE primary

Run `RF DEEP PROBE` with the VTX off and again with A4/5805 MHz on. It executes
unchanged vendor arms for all eight historical arguments, compares modes 0/11
when the hash-pinned producer is enabled, records mode 12 as skipped, and logs
tuning proxies before the ring probe and finite IQ/WBFM sanity checks.
`PHASE PROBE <field>` is a
separate coherent-tone test. The next milestone is physical RF measurement,
not another broad static survey.

Build the opt-in image only against the audited tree:

```sh
idf.py -B build-rf-deep-probe \
  -DSDKCONFIG_DEFAULTS="sdkconfig.defaults;sdkconfig.defaults.rf_deep_probe" \
  set-target esp32c5 build
```

Tune A4 as usual and issue `RF DEEP PROBE` once with the VTX off, save the
complete log, then repeat after enabling the 5805 MHz VTX. Run
`PHASE PROBE 0` separately with a coherent unmodulated tone; the deep probe
does not infer phase continuity from random noise or video modulation.
