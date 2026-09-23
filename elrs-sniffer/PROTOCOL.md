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
| `rx_dropped` | uint | count | cumulative rxpkt frames dropped by the 30/s export cap |
| `sig` | object | — | elrs_sig quality: `q` = `none`/`weak`/`firm`/`strong` (firm = >=3 `crc_pass` in the harvest pass OR >=10 `sync_struct` with >=1 repeated tail; strong = >=10 `crc_pass`; weak = >=3 `sync_struct`), plus `frames`, `sync_struct`, `crc_pass` totals and per-band `lora`/`flrc` `{f,s,c}` sub-objects |
| `sync_only` | 0/1 | — | locked but no validated RC data for >2 s — a sync-follow lock (TX beaconing, model disarmed/off). Sticks in `ch` are stale; expect no telemetry |
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

`interferer` — a discovery dwell bailed on junk (WiFi): >200 frames/s
sustained 500 ms with zero pair-gated candidates:
`{"t":"event","what":"interferer","rate":"DVDA 500Hz","fps":767,"action":"abort"}` —
the FIRST verdict aborts the dwell immediately (total dwell on junk <= ~2 s);
`action":"watch"` diagnostics are capped at 1/s. The rate
AND its twin are skipped for the current sweep pass only (marks clear when
the pass wraps). Dwell extensions count ONLY CRC-validated packets and
pair-gated candidates — raw type-classifier "sync" labels never extend.

Dwell lines carry `nf` (discovery noise floor dBm, running min) and
`nf_thr` (`nf` + 12 dB - the frame-acceptance threshold). `crc_ok`
stays 0 in discovery mode (radio CRC disabled there; the counter is
gated in software). `sync_seen` crack events coalesce per tail
(1 per 2 s) and crack events are globally capped at ~5/s.

A dwell line may carry `"jump":"iq-twin" — the dwell was hot-but-dead
(strong RSSI, zero validated/sync packets) and the sweep jumped straight
to the other IQ polarity of the same rate instead of advancing.
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

`dbg` — diagnostic lines (not emitted at a fixed rate):
```json
{"t":"dbg","what":"flrc_setup","fq":0,"mp":0,"pp":0,"sw":0,"crc":0,"status":135}
```
emitted per FLRC dwell setup with each config call's RadioLib return code
(`fq` setFrequency, `mp` SetModulationParams, `pp` SetPacketParams,
`sw` sync-word register write, `crc` CRC-seed register write; all 0 =
success) plus the chip STATUS byte. Any non-zero rc pinpoints the failing
call.

`sync` — a sync packet decoded. `ok=1` ONLY means the ELRS software CRC
validated (LoRa). FLRC syncs carry no software CRC: they appear with
`"ok":0,"band":"flrc"` (the radio's seeded 3-byte CRC did the filtering).
Additive fields `band` (`lora`|`flrc`), `len` (air bytes), `freq` (tuned Hz)
disambiguate multi-link captures:
```json
{"t":"sync","ok":1,"band":"lora","len":8,"freq":2441400000,"fhss":42,"nonce":240,"rateIdx":6,"swMode":0,"tlmRatio":2,"uid":"a5b3c2"}
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
  - `oled_driver` (`drv:"sh1106"|"ssd1306"`) — OLED driver toggle.
  - `verbose_on`/`verbose_off` (V command); `parked{step,rate}` /
    `sweep_resume` (R command).
  - `uid` — bind phrase applied: `{"t":"event","what":"uid","src":"stored|set|default","uid":"43 7f ..."}` (`src` says which phrase produced the UID).
  - `fp` — OSINT fingerprint, first sync of a link:
    `{"t":"event","what":"fp","band":"lora|flrc","uid_tail":"a1b2c3"}`.
  - `flrc_sync` — FLRC sync parsed. `tail_src`: `structural` = classified
    in discovery mode — emitted when a tail reaches ELRS_DISC_NEEDED (3)
    sightings inside a 2 s sliding window (RSSI-gated against the dwell's
    noise floor: +12 dB margin, -100 dBm absolute floor; windowed counting,
    not adjacency — a real 6-12/s link candidates in <1 s amid 800/s junk) —
    a sighting, not a confirmation; `syncword+crc24` = demodulated under the
    exact UID-derived 32-bit sync word with the seeded radio CRC
    (confirmed).
  - `uid2_crack` — per UID[2] candidate with any score:
    `{"t":"event","what":"uid2_crack","uid2":47,"valids":5}`.
  - `uid_cracked` — full UID recovered, hop-following starts:
    `{"t":"event","what":"uid_cracked","uid":".. .. .. .. .. .."}`.
  - `uid2_crack_failed` — all 256 candidates exhausted.
`crack` — the acquisition/crack state machine (dashboard contract).
Emitted on EVERY state transition and as progress every 16 candidates:
```json
{"t":"crack","state":"cracking","uid_tail":"61ace1","done":128,"total":256,"valids_best":2}
{"t":"crack","state":"cracked","uid_tail":"61ace1","done":47,"total":256,"valids_best":5,"uid_full":"5a9b4761ace1"}
```
states: `listening` (FLRC discovery parked / idle; `tail_src` says whose tail is being chased: `phrase` = configured-phrase guess, `sync` = tail learned from the air, `last-link` = previous link), `sync_seen` (first
structurally-valid sync; `uid_tail` = leaked UID[3..5]), `identity` (second
consistent sync adopted), `cracking` (`done`/`total`, `valids_best`),
`cracked` (+ `uid_full` = phrase-prefix guess + cracked tail — verify the
tail with phrase_crack), `failed` (256 exhausted → back to listening).
`stats.crack` mirrors the last state; `stats.mode` is one of
`sweep|park|follow|discovery`.

- `flrc_discovery` — FLRC unknown-UID discovery state machine:
  `listening` (dwell parked in discovery: no sync-word match, CRC off) →
  `sync_seen` (a sync-structured packet leaked a UID tail) → `cracking`
  (tail adopted after 2 consistent syncs; exact per-candidate sync words,
  radio-CRC scoring) → `cracked` (exact 32-bit sync + true seed, following).
  Phrase-derived UID stays the fast path: `flrc_sync match:1` on the first
  sync skips structural gating.

Lock semantics (sync-only streams): a lock drops only when BOTH no
validated data packet for 6 s AND no validated sync for 30 s — a TX
beaconing syncs (disconnected fast-sync) holds `lock:1` with
`sync_only:1` indefinitely instead of churning lock/unlock. FHSS
hop-following runs in both cases (syncs re-anchor the sequence).
  Console commands: `P` = radio pin re-probe, `D` = toggle OLED driver,
  `V` = print every demodded packet as `rawpkt` (10/s), `R [step]` = park
  the sweep on a step (`R` alone resumes; steps are numbered 0..N-1 as
  listed in the `dwell` lines), `U <phrase>` = set the ELRS bind phrase
  (rest of the line, spaces allowed; persisted). The T3-S3 BOOT button
  (GPIO0) held ≥1.5 s also toggles the OLED driver. Manual toggles pause
  `stats` for ~1.5 s (splash); while parked, dwell extensions still apply
  and the unlock path re-enters at the parked step.

Round-8 audit notes (values changed, fields unchanged): the tuned sync
frequency is now the exact register-unit value **2441399841 Hz** (all `freq`
fields); FLRC now includes the 500 Hz rate; `sync_frame` gating uses the
sequence-pointer semantics (fhssIndex is a pointer into the 240-entry hop
sequence, not a channel). `uid45.variant` now spans 3 values (init placement
/ byte order only — the FLRC CRC24 poly is the fixed 0x5D6DCB per the
SX1280 datasheet).

## ELRS 4.x air support + SF config (round 11)

- `sfconf` — boot probe: `{"t":"event","what":"sfconf","ok":1,"val":30}`
  proves REG_SF_ADDITIONAL_CONFIG (0x925, SF5/6=0x1E, SF7/8=0x37) writes
  and reads back — the register RadioLib omits and ELRS writes after every
  SetModulationParams (SX1280.cpp:283-299).
- `otaver` — emitted after each validated sync:
  `{"t":"event","what":"otaver","v":3|4,"layout":3|4}`. 4.x CRC init family:
  `(UID4<<8|UID5) ^ 0x0400` (master OTA.cpp OtaUpdateCrcInitFromUid), tried
  alongside the 3.x `^3` in every validation path. 4.x syncs carry
  `rfRateEnum` in byte 3 and NO UID3 (bytes: fhss, nonce, rfRateEnum,
  packed switch/tlm/gemini/proto, UID4, UID5) — `sync.uid` shows "00xxxx"
  for the absent UID3.
- Non-sync packets in exact mode additionally validate with the 4.x
  nonce-mixed init (`OtaCrcInitializer ^ OtaNonce`, master
  GeneratePacketCrcStd/Full) using the tracked expected nonce.
- `uid45` gains `uid4_4x` — the UID4 alternative under the 4.x high-byte
  XOR (UID4 ^ 4). FLRC sync-word identity remains 3.x-family (`UID5^3`).
- Harvest steps: 8 dwells x 2200 ms (syncs only when the TX sequence visits
  channel 41 — every 80 hops); list: LoRa 250/500 (n+i), DVDA 500/250,
  FLRC 500/1000. Bind parking (LoRa 50 i) remains in the SWEEP.

## Round 11/12 additions

- `lora_regs` — once per LoRa rate per boot, at dwell start:
  `{"t":"event","what":"lora_regs","rate":"LoRa 250Hz","sf":"60","bw":"18","cr":"07","reg925":"1e","pkt_type":"02","rx":"cont"}` —
  sf/bw/cr are the raw bytes written (no register readback exists for
  modulation params; they are now written via RAW SPI, bypassing RadioLib's
  setter chain entirely); reg925/pkt_type are read back from the chip; `rx`
  is always continuous raw SetRx (RadioLib's startReceive is NOT used — it
  re-sent stored packet params and clobbered per-dwell config, the prime
  LoRa-silence suspect).
- `Y` command — toggles the 0x925 SF-config write live (`y925_on`/`y925_off`
  events; default on) for A/B without reflashing.
- Boot selftest now covers BOTH modems: `rxdiag ... "selftest":true,
  "flrc_demod":N, "lora_demod":M` — a 200 ms FLRC-discovery window on
  2480.5 MHz plus a 300 ms LoRa 250 iq=n window on the sync frequency.
  `lora_demod:0` at boot = LoRa RX path fault.
- `rxdiag` is now emitted for EVERY dwell end (LoRa sweep included).
- Telemetry/linkstats events only fire from CRC-validated packets.

## Raw RX diagnostics (round 10)

`rxpkt` — EVERY demodulated packet, unfiltered (command `N` toggles, default
OFF; `V` implies rxpkt during harvest/fastlink only). Max 30/s; overflow is
counted in `stats.rx_dropped`:
```json
{"t":"rxpkt","band":"flrc","rate":"DVDA 500Hz","hex":"...","rssi":-42,"irq":2}
```
`rssi` is the per-frame packet-status RSSI read in the RX path; `irq` is the
IRQ status word captured at RxDone.

`rxdiag` — per-dwell counter line (each harvest/fastlink step end, and FLRC
sweep dwell ends):
```json
{"t":"event","what":"rxdiag","rate":"DVDA 500Hz","band":"flrc","demod":84,
 "exported":30,"dropped":54,"swerr":0,"rssi_max":-38,"rssi_min":-97}
```
`swerr` = SyncWordError IRQ count (sticky-register diff): in EXACT FLRC mode a
real link with a DIFFERENT UID sync word spikes swerr at its packet cadence —
an honest real-signal indicator junk cannot fake. At boot, an
`"selftest":true` rxdiag reports a 200 ms FLRC-discovery window on 2480.5 MHz
— nonzero `demod` proves the DIO1/IRQ/demod path works end-to-end.

Commands: `N` = rxpkt toggle (`rxpkt_on`/`rxpkt_off` events); `V` = verbose
(+ rxpkt during harvest only).

## Find mode (passive bind-phrase/UID discovery)

`sync_frame` — a sync-shaped frame captured while parked on the sync
channel during a harvest cycle (alternates with sweep passes when unlocked;
never starves detection). Throttled to ~4/s, deduped by (nonce, fhssIndex):
```json
{"t":"sync_frame","band":"flrc","rate":"FLRC 1000Hz","hex":"024129...","rssi":-55}
```
`band` `lora` frames also flow through the normal self-seeded sync path and
can lock directly. `band` `flrc` frames feed the on-device seed brute.

`uid45` — the FLRC HW-CRC seed (2^16 brute across the explicit CRC24
variants, 2-frame confirmation) leaked by a captured FLRC sync frame:
```json
{"t":"event","what":"uid45","uid4":"61","uid5":"ce","model_id":37,"variant":0,"uid3":"2f"}
```
`variant` identifies which SX1280 CRC24 model validated — until
tools/flrc_crc_probe.py confirms the variant against a known-phrase capture,
treat uid45 as strong-but-unconfirmed. Phrase check offline:
`python3 tools/phrase_crack.py <uid_tail> --frames capture.jsonl`.

`harvest` — `{"t":"event","what":"harvest","state":"start","freq":2441400000}`
marks each find-mode cycle (7 dwells x ~750 ms PARKED ON THE SYNC FREQUENCY
2441.4 MHz — the `freq` field is logged once per cycle so it can be verified
against the expected 2441.5 MHz class allocation).

`uid_found` — ELRS bind mode broadcasts the UID in plaintext
(`MSP_ELRS_BIND=0x09, UID[2..5]` on the sync channel, LoRa 50Hz, inverted IQ,
CRC init 0); the sniffer adopts it immediately:
```json
{"t":"event","what":"uid_found","uid":"?? ?? 5a 61 ac e1","src":"bind","freq":2441400000,"rssi":-42}
```
UID[0..1] are never broadcast ("??"). Adoption also emits the standard `uid`
event with `"src":"bind"`, persists the UID as `uidlast`, and starts
following. On boot with a stored `uidlast`, a 60 s `fastlink` window
(`{"t":"event","what":"fastlink","state":"start"|"end"}`) retries exact
follow on the four main LoRa rates before falling back to the full sweep.

`uid_cracked` may now carry `"via":"uid2-trackers"` and `uid2_alt`: the
FHSS sequence cannot distinguish UID[2] bit 7 (mod 2^31 drops it), so the
tracker survivor is reported with its `survivor|0x80` alternate.

Junk-hardening note: `lq` is clamped to 100 (values above are corruption)
and RC/arm stick values come ONLY from CRC-validated packets.

## FHSS hop-following

After a validated sync the sniffer brute-forces UID[2] (the byte ELRS
never broadcasts; needed to seed the hop sequence) and then follows the
ELRS 2.4G FHSS sequence (`elrs_fhss.h`, ported from 3.6.4 FHSS.cpp +
random.cpp): 80 × 1 MHz channels 2400.4–2479.4, sync channel 2441.4,
sequence length 160, TX position advancing once per `hopInterval` packets
(tx_main.cpp: `(OtaNonce+1) % hopInterval == 0`). While following, `stats`
gains `freq` (current tuned Hz) and `fhss` (sequence index, 255 = not
following); every received packet is captured — rc sticks, telemetry
frames, linkstats — not just sync-channel traffic. Sync packets re-anchor
the sequence position. Lock drops only after 6 s with zero validated
packets (deep fade), then the sweep resumes; a new link emits a new `fp`.

## FLRC rates

Dwell labels `FLRC 1000Hz`, `DVDA 500Hz`, `DVDA 250Hz` (single polarity —
ELRS ignores InvertIQ for FLRC) use the UID-derived 32-bit sync word and a
3-byte radio CRC seeded with `OtaCrcInitializer`, both from the bind
phrase. FLRC packets have NO ELRS software CRC: on FLRC dwells `crc_ok`
counts radio-validated packets, and a classified SYNC prints
`{"t":"event","what":"flrc_sync","uid_pkt":"...","uid_phrase":"...","match":0|1}`
— `match:1` when the packet's UID tail agrees with the phrase-derived UID.
A wrong phrase yields silence on FLRC dwells (radio CRC rejects everything).

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
