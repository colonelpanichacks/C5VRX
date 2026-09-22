# PROTOCOL.md — ELRS sniffer serial JSON contract

Version: `0.2.2` (banner reports `ELRS_SNIFFER_VERSION`). One JSON object
per line, LF-terminated, on the native USB serial port at **460800 8N1**.
All fields are additive-only within the 0.2 series — dashboard code must
ignore unknown keys and must skip any line that does not begin with `{`
(human-readable banner lines are printed at boot and on faults).

Device time base: `ms` is device `millis()` (uint32, wraps ~49.7 days) —
use host reception time for graphs, `ms` only for intra-device ordering.

## Cadence guarantees

| line | when |
|---|---|
| `stats` | exactly once per second — **always**, even with no radio (`radio:0`); a missing `stats` line for >2 s means the device is dead |
| `dwell` | on every sweep-step change while unlocked (≈ every 0.7–4 s) |
| `sync`, `lock`, `unlock`, `linkstats`, `gps`, `batt`, `atti`, `fm`, `tlm` | as decoded, asynchronous |
| `boot`, `ready`, `error`, `radio_up`, `probe` | boot / fault lifecycle |

## `stats` — 1 Hz summary (the primary dashboard feed)

```json
{"t":"stats","ms":12345,"rate":"LoRa 250Hz","iq":"i","rssi":-87,"snr10":85,"pps":243,"lq_permille":970,"lock":1,"ch":[1500,988,1501,1499],"arm":0,"uid":"a5b3c2"}
```

| field | type | unit | meaning |
|---|---|---|---|
| `rate` | string | — | dwell rate name: `LoRa 500Hz`, `LoRa 333Hz8`, `LoRa 250Hz`, `LoRa 150Hz`, `LoRa 100Hz8`, `LoRa 50Hz`; legacy 2.x sweeps append `*` |
| `iq` | char | — | `n` = standard IQ, `i` = inverted (ELRS `invertIQ = UID[5]&1`) |
| `rssi` | int | dBm | **live energy max** over the 1 s window (SX1280 GET_RSSIINST sampled at ~10 Hz) — includes noise floor; see below |
| `rssi_now` | int | dBm | most recent live sample |
| `snr10` | int | dB×10 | SNR of the last classified packet (sniffer receiver) |
| `pps` | uint | packets/s | **CRC-validated ELRS packets** demodulated in the window (true decode rate) |
| `rx_per_s` | uint | RxDone/s | raw demodulated packets in the window (anything the radio accepted) |
| `lq_permille` | uint | ‰ | while `lock=1`: `pps / expected_pps_for_rate × 1000` (sniffer-side estimate, quantized by the 1 s window; 1000 = ceiling). While unlocked: last link-reported LQ ×10 from a `linkstats` event, else 0 |
| `lock` | 0/1 | — | a CRC-validated ELRS sync has been captured and not timed out |
| `ch` | [4] uint | µs | sticks `[roll, pitch, throttle, yaw]`, 988–2012; `0` = not yet decoded |
| `arm` | 0/1 | — | AUX1 high (arm channel) |
| `radio` | 0/1 | — | 0 = radio faulted (no SX1280); device alive but deaf. `rssi` is then the last/initial -128 sentinel |
| `rx`, `crc_ok` | uint | count | cumulative since boot: RxDone demods / ELRS-CRC-validated packets |
| `types` | object | count | cumulative per-type parses: `rc`, `msp`, `sync`, `tlm`, and `unk` (demodded but failed classification = noise) |
| `uid` | string, optional | — | `UID[3..5]` as lowercase hex — **link fingerprint, not identity**; absent until a sync is captured |

## Events

`dwell` — emitted when a sweep dwell ENDS, carrying its energy + packet
fingerprint. Dwells start at 2 s and extend in 2 s chunks (cap 35 s) while
the dwell shows energy (`rssi_max` > −85 dBm) or classified packets — sync
packets can be tens of seconds apart on a connected link, so hot dwells
are patient:
```json
{"t":"dwell","step":0,"rate":"LoRa 250Hz","iq":"n","legacy":0,"rssi_max":-71,
 "rx":14,"crc_ok":11,"types":{"rc":11,"msp":0,"sync":0,"tlm":0,"unk":3}}
{"t":"dwell_ext","rate":"LoRa 250Hz","iq":"n","rssi_max":-69,"dwell_ms":4000}
{"t":"event","what":"awaiting_sync","rate":"LoRa 250Hz","iq":"n"}
```
`rx` = raw RxDone in the dwell; `crc_ok` = ELRS-CRC-validated; `types`
the per-type parses (`unk` = demodded noise that failed classification).
`awaiting_sync` repeats every 5 s while packets flow without a sync yet.

`pkt` — sync-first debug: every packet that passes the ELRS CRC (rate
limited to 2/s), so the actual bytes reaching the parser are visible:
```json
{"t":"pkt","type":"rc","len":8,"hex":"1a5802d42b88f3c2"}
```

`rawpkt` — FULL hex of every packet that classifies as sync (validated or
not), ≤2/s — ground truth for CRC forensics:
```json
{"t":"rawpkt","cls":0,"hex":"de3f0000000031de"}
```
`cls` is the parser classification (0 = CRC_OK, 1 = plausible, 2 = noise).
Re-run the host seed-search (`test/host/test_parser.cpp`) on any captured
hex to test whether ANY CRC init validates it.

`sync` — a sync packet decoded (`ok=1` means the ELRS software CRC validated):
```json
{"t":"sync","ok":1,"fhss":42,"nonce":240,"rateIdx":6,"swMode":0,"tlmRatio":2,"uid":"a5b3c2"}
```
`rateIdx` indexes the ELRS SX128X rate table (0–9: FLRC1000, FLRC500,
DVDA500, DVDA250, LoRa500, LoRa333-8, LoRa250, LoRa150, LoRa100-8,
LoRa50). `swMode` 0=wide/8ch, 1=hybrid16ch/12ch. `tlmRatio` is the ELRS
enum (0=STD, 1=off, 2=1:128 … 8=1:2). `uid` = UID[3..5] hex.

`lock` / `unlock` — link capture state:
```json
{"t":"lock"}
{"t":"unlock","why":"timeout"}
```

`linkstats` — the link's own report, relayed from telemetry (distinct from
`stats.rssi`!): `rssi1`, `rssi2`, `snr` are **already signed** (value = −dBm):
```json
{"t":"linkstats","lq":98,"rssi1":-87,"rssi2":-91,"snr":8}
```

Telemetry frames (reassembled CRSF):
```json
{"t":"gps","lat_e7":525123456,"lon_e7":13399123,"spd_kmh10":123,"sats":14}
{"t":"batt","v10":168,"a10":34,"mah":12345}
{"t":"atti","p":12,"r":-340,"y":1800}     // pitch/roll/yaw, rad × 10000
{"t":"fm","m":"ACRO"}                     // flight mode string
{"t":"tlm","ft":9}                          // CRSF frame type with no decoder yet
```

Boot/lifecycle:
```json
{"t":"boot","v":"0.2.2","board":"LilyGo T3-S3 SX1280-PA","flash_mb":4,"variant":"sx1280pa"}
{"t":"ready","steps":20,"sync_freq":2441,"radio":1,"oled":1}
{"t":"radio_up"}
{"t":"error","what":"radio_init","detail":"timeout 15s (no SX1280 ACK)","pins":"NSS=7 SCK=5 MISO=3 MOSI=6 RST=8 DIO1=9 BUSY=36"}
{"t":"error","what":"oled_init","detail":"no ACK at 0x3c SDA=18 SCL=17"}
```

- `boot` fires before any init, always — its absence means the board never
  ran (bad flash/hardware), not an app problem.
- `ready` reports init outcomes: `radio`/`oled` 0 = that subsystem faulted;
  stats still flow. `pins` (optional) names the pin set the radio probe
  settled on, `oled_drv` (optional) the active panel driver (`sh1106`/
  `ssd1306`, default sh1106 — see `event`).
- `error.what` values: `radio_init`, `oled_init`, `sx1280_init` (legacy
  string may still appear on very old builds).
- `radio_up` fires when a previously faulted radio recovers (retried every
  ~15 s); there is no `radio_down` — use `stats.radio:0` as the fault flag.
- `probe` — one line per pin-set the startup probe tries:
  ```json
  {"t":"probe","set":"v12-sx1280-pa","pins":"NSS=7 SCK=5 MISO=3 MOSI=6 RST=8 DIO1=9 BUSY=36","ok":1,"cached":1}
  ```
  `set` values: `v12-sx1280`, `v12-sx1280-pa`, `altB`, `altB-pa` (all share
  the SPI bus; RST/DIO1/BUSY and the PA switch pins differ). `cached:1`
  marks the NVS-cached set tried first; `{"t":"probe","info":"manual
  sweep start"}` precedes a console-forced re-probe (send `P`). The probe
  caches its winner in flash, so later boots try it first.
- `event` — runtime happenings (additive catch-all):
  ```json
  {"t":"event","what":"oled_driver","drv":"sh1106"}
  ```
  emitted when the OLED driver is toggled (`what:"oled_driver"`,
  `drv:"sh1106"|"ssd1306"`). Console commands: `P` = radio pin re-probe,
  `D` = toggle OLED driver. The T3-S3 BOOT button (GPIO0) held ≥1.5 s also
  toggles the OLED driver. Manual toggles pause `stats` for ~1.5 s (splash).

## Notes for the dashboard integration

- The sniffer listens on the **ELRS sync channel 2441.400 MHz** only (the
  over-air frequency is not in the JSON yet — `sync_freq` in `ready` is it).
- Sticks freeze at their last value when the link drops; use `lock`/`pps`
  to grey out stale `ch` data.
- `lq_permille` is the honest sniffer-side number; treat ≥980 as "healthy",
  and prefer `linkstats.lq` when present.
- Everything the sniffer reports about the *remote* link (rssi1/rssi2/snr/
  lq in `linkstats`) comes from the model's own receiver relayed over
  telemetry — it describes uplink quality at the aircraft, not at the
  sniffer.
- Coordinate-privacy note for the UI: `gps` lines are the pilot's own
  telemetry; display/store per your legal note in README.md.
