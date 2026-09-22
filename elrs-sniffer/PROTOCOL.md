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
| `boot`, `ready`, `error`, `radio_up` | boot / fault lifecycle |

## `stats` — 1 Hz summary (the primary dashboard feed)

```json
{"t":"stats","ms":12345,"rate":"LoRa 250Hz","iq":"i","rssi":-87,"snr10":85,"pps":243,"lq_permille":970,"lock":1,"ch":[1500,988,1501,1499],"arm":0,"uid":"a5b3c2"}
```

| field | type | unit | meaning |
|---|---|---|---|
| `rate` | string | — | dwell rate name: `LoRa 500Hz`, `LoRa 333Hz8`, `LoRa 250Hz`, `LoRa 150Hz`, `LoRa 100Hz8`, `LoRa 50Hz`; legacy 2.x sweeps append `*` |
| `iq` | char | — | `n` = standard IQ, `i` = inverted (ELRS `invertIQ = UID[5]&1`) |
| `rssi` | int | dBm | **sniffer's own** receiver, peak-hold over the 1 s window |
| `snr10` | int | dB×10 | SNR of the last classified packet (sniffer receiver) |
| `pps` | uint | packets/s | ELRS-classified packets demodulated in the window |
| `lq_permille` | uint | ‰ | while `lock=1`: `pps / expected_pps_for_rate × 1000` (sniffer-side estimate, quantized by the 1 s window; 1000 = ceiling). While unlocked: last link-reported LQ ×10 from a `linkstats` event, else 0 |
| `lock` | 0/1 | — | a CRC-validated ELRS sync has been captured and not timed out |
| `ch` | [4] uint | µs | sticks `[roll, pitch, throttle, yaw]`, 988–2012; `0` = not yet decoded |
| `arm` | 0/1 | — | AUX1 high (arm channel) |
| `radio` | 0/1 | — | 0 = radio faulted (no SX1280); device alive but deaf. `rssi` is then the last/initial -128 sentinel |
| `uid` | string, optional | — | `UID[3..5]` as lowercase hex — **link fingerprint, not identity**; absent until a sync is captured |

## Events

`dwell` — sweep moved to a new (rate, IQ) step:
```json
{"t":"dwell","step":7,"rate":"LoRa 250Hz","iq":"i","legacy":0}
```

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
  stats still flow.
- `error.what` values: `radio_init`, `oled_init`, `sx1280_init` (legacy
  string may still appear on very old builds).
- `radio_up` fires when a previously faulted radio recovers (retried every
  ~15 s); there is no `radio_down` — use `stats.radio:0` as the fault flag.

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
