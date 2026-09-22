# ELRS-SNIFFER — passive ExpressLRS 2.4 GHz packet sniffer

OSINT companion to the **OUI-SPY** 5.8 GHz analog-FPV detector. Where OUI-SPY
finds *video* transmitters, this finds the *control link*: a LilyGo SX1280
(2.4 GHz) board passively receives ExpressLRS packets, decodes sticks and
telemetry, and shows them on the onboard display + USB serial (one JSON line
per second plus per-event lines, shaped for later fusion into the OUI-SPY
Flask dashboard).

> **STATUS: firmware-complete, hardware-pinned, bench-untested.** Board is the
> **LilyGo T3-S3 SX1280** (pins verified against LilyGo's factory sources —
> [BOARD.md](BOARD.md)); both PlatformIO envs build; the parser is
> host-tested. RF/OLED bring-up against a live ELRS TX is the remaining step.

## What it detects

| Source | Always? | Notes |
|---|---|---|
| **Stick channels** (roll/pitch/thr/yaw) | yes, from any decoded RC packet | 10-bit over-air values -> us |
| **AUX1 (arm)** | yes | 1-bit arm flag in every packet |
| **Other AUX switches** | yes but sampled | hybrid/wide round-robin: each AUX slot only rides every N packets |
| **Link stats** (LQ, RSSI@RX, SNR, TX power) | when the pilot enables telemetry | from TLM packets (both OTA4 LINK and OTA8 LinkStats prefix) |
| **GPS / battery / attitude / flight mode** | only if the pilot's model sends CRSF telemetry | fragmented CRSF frames, reassembled |
| **Link fingerprint** (UID[3..5] + rate + CRC init) | from SYNC packets | see the identity note below |
| RF beacon | n/a | **ELRS has none** — closest analogs are the periodic SYNC packets and telemetry bursts, both detected |

**Identity honesty:** ELRS packets carry **no per-packet TX ID**. A periodic
SYNC packet reveals UID[3..5], and the packet CRC initializer must equal
`((UID[4]<<8)|UID[5]) ^ 3` — together a *link fingerprint*. Two models bound
to the same bind phrase share it, so this fingerprints a link session, not a
unique aircraft. The code and UI say "link", never "pilot".

## How it works

- Parks on the **ELRS 2.4 GHz sync channel, 2441.4 MHz** (80 × 1 MHz FHSS
  grid, sync index 41), sweeping the ELRS 3.x rate table × both IQ
  polarizations (`invertIQ = UID[5]&1`, bind mode always inverted), plus
  legacy 2.x LoRa variants. On a CRC-valid SYNC it locks onto that rate/IQ.
- Decodes **OTA4 (8-byte)** and **OTA8 (13-byte)** packets, 3.x layout:
  RC data, SYNC, TLM, MSP. Validation tiers: `CRC_OK` (ELRS software CRC —
  the SX1280's radio CRC is switched OFF in the ELRS air config, and its
  LoRa modem has no sync word, so junk demods) / `PLAUSIBLE` (heuristics)
  / `NOISE`.
- The **CRC init is self-acquired**: a SYNC packet carries UID[4..5], from
  which the init is derived and the packet's own CRC re-validated. Bind-mode
  links use init 0 and are plaintext-equivalent.
- FLRC rates (1000/500 Hz + DVDA) are not swept by default: their 32-bit sync
  word is UID-derived. Once a LoRa SYNC reveals UID[3..5], only UID[2] is
  unknown — 256 candidate sync words make FLRC hunting a feasible TODO.

All constants live in `src/elrs_defs.h` with per-line citations to the
ExpressLRS source (3.6.4): `src/src/common.cpp` (rate table),
`src/lib/OTA/OTA.{h,cpp}` (layouts, CRC polys, channel packing),
`src/lib/SX1280Driver/SX1280.cpp` (air config, CRC-off, "LoRa has no sync
word"), `src/src/rx_main.cpp` (invertIQ rule, FLRC sync word = MAC seed),
`src/lib/FHSS/FHSS.cpp` (frequency plan), `src/lib/TelemetryProtocol/` +
CRSF spec (telemetry types).

## Hardware / wiring

**LilyGo T3-S3 (ESP32-S3) SX1280 2.4 GHz + 0.96" SSD1306 OLED** — no wiring,
everything onboard. Pinout is **[VERIFIED]** from LilyGo's factory sources:
[BOARD.md](BOARD.md). If the unit is the **PA** variant, add
`-D T3S3_SX1280_PA` to the build flags so the antenna-switch enables
(RX=21/TX=10) are driven — see BOARD.md.

## Build & flash

```bash
cd elrs-sniffer
pio run                                # generic env compile check
./flash.sh                             # build + FULL ERASE + flash + verify
pio device monitor -b 460800           # the JSON stream
```

`flash.sh` exists because `pio run -t upload` writes only the app image —
after the required full erase that leaves an unbootable board. The script
writes the full canonical image set at explicit offsets (documented, with
provenance, in [flash.md](flash.md)) and verifies. USB serial is the
product interface: JSON lines at 460800 baud over the T3-S3's native USB
port. If `pio` is unavailable: `pip install platformio` (or
`python3 -m pip install platformio`); flash.sh falls back to
`python3 -m platformio` / `python3 -m esptool` automatically.

## Serial protocol (dashboard-fusion ready)

Full contract — every field, units, cadence — is in
**[PROTOCOL.md](PROTOCOL.md)** (stable schema for the dashboard
integration; additive-only within 0.2.x). One `stats` line per second:

```json
{"t":"stats","ms":12345,"rate":"LoRa 250Hz","iq":"i","rssi":-87,"snr10":85,"pps":243,"lq_permille":970,"lock":1,"ch":[1500,988,1501,1499],"arm":0,"uid":"a5b3c2"}
```

Event lines: `dwell` (sweep step), `sync` (+uid, rate, nonce), `lock` /
`unlock`, `linkstats` (LQ/RSSI/SNR from the link), `gps`, `batt`, `atti`,
`fm` (flight mode), `tlm` (unknown CRSF type), `error`.

**Dashboard-fusion roadmap:** a small bridge (`tools/elrs_bridge.py`, TODO)
reads these lines over serial and feeds the OUI-SPY Flask app the same way
the 5.8 GHz side does — a second sensor tab with stick bars, an LQ/RSSI
history graph, and one fused "RF picture" (video + control links) per
detection. PROTOCOL.md is the contract.

## Legality

Passive receive only — this firmware never calls a transmit API. Receiving
in the license-free 2.4 GHz ISM band is broadly permitted; *using* decoded
third-party telemetry may be restricted where you operate (privacy,
direction-finding, interception laws). You are responsible for compliance.
The UI honesty note applies in the field too: "link fingerprint" is not
"identified pilot".

## OSINT

The sniffer's endgame is zero-prior-knowledge link capture: sync packets
leak `UID[3..5]` in the clear (fingerprint), the hop sequence is seeded by
the UID, and the full UID recovers the **binding phrase** that names the
pilot's setup:

```bash
python3 tools/phrase_crack.py 61ace1        # tail from a captured sync
python3 tools/phrase_crack.py --uids captures.txt --wordlist extra_words.txt
python3 tools/phrase_crack.py 61ace1 --wordnums --leet --case   # human patterns
python3 tools/phrase_crack.py 61ace1 --alnum 5                  # brute [a-z0-9]^5
```

`phrase_crack.py` (stdlib only) runs the exact ELRS derivation
`UID = md5("-DMY_BINDING_PHRASE=\"<phrase>\"")[:6]` over, in order: the
built-in dictionary (defaults, FPV vocabulary, ~300 first names × `["", "1",
"123", "2023", "2024", "2025", "!"]`); `--leet` (l33tspeak variants, single
+ double `a4 e3 i1 o0 s5`); `--case` (Capitalized + UPPER); `--wordnums`
(`word+d`, `d+word`, `word_d` for d 0..9999 — the classic "freestyle23",
"sarah2023" pattern, ~9M hashes); and `--alnum N` (full `[a-z0-9]^N` brute
across all cores — N=5 is 60M ≈ seconds-to-tens-of-seconds multicore,
N=6 is 36⁶ ≈ 2.2e9 ≈ a few minutes; an ETA prints before it starts).
Tail-only hits are reported "PROBABLE, verify with more captures".
~1.3M hashes/s per core single-threaded (~15M/s on a 10-core box).

What the tiers cover, in OSINT terms:

* **iFlight ecosystem, line-wide**: the official default phrase
  `iloveiflight` (UID `12ce9b6e75c4`, self-check verified) is the
  Commando8/radio default and ships on the iFlight BNF line — one dictionary
  hit labels the whole fleet.
* **HappyModel stock batches**: factory firmware with no binding phrase uses
  a MAC-derived UID, not a phrase — uncrackable, so it is *recognized*
  instead via the fixed-UID table (checked instantly, before any hashing):
  `0000609e0b58` (Mobula6/Crux3/Mobula7 batch), `000096af1f7c` (Mobula6
  batch 2). Add new dumps to `tools/factory_uids.json`.
* **Brand/model defaults**: per-brand official phrases and model/radio names
  (BetaFPV, RadioMaster Zorro/Boxer/Pocket, EMAX Tinyhawk, DJI, Walksnail,
  FrSky, …) sit at the top of the dictionary with labeled matches
  ("BRAND DEFAULT (RadioMaster): zorro2024").
* **Human patterns**: `--wordnums` (freestyle23, sarah2023), `--leet`,
  `--case`, and `--alnum 6` sweeps every short lowercase-alphanumeric
  phrase. `--rockyou PATH` adds a local rockyou-format list (never bundled;
  Kali `/usr/share/wordlists/rockyou.txt` or SecLists).

What is NOT covered: phrase-less stock firmware is MAC-derived (see fixed
UID table / OUI fingerprinting); 7+ char or mixed-case-symbol phrases blow
past brute force — use targeted wordlists from your own intel. A GPU MD5
attack is trivially possible; this tool stays stdlib-only because the tiers
above cover the realistic field cases.

**Legal posture:** the cracker is an *offline* attack on *your own passive
captures*. It exists to suggest a human-readable label for a link the
sniffer legitimately heard (dashboard alias candidate) — not to enable
impersonation: knowing a binding phrase lets you *receive* a link, never
control it (control requires the TX's model setup, and driving someone
else's craft is a crime regardless of protocol). Only run it against
captures you are legally allowed to possess, and follow your jurisdiction's
rules on intercepting and using third-party radio traffic.

OSINT pipeline in one line: `fp`/`sync` event → `uid_tail` →
`phrase_crack.py` → suggested alias (dashboard auto-labeling).

## Repository map

```text
├── platformio.ini        # esp32dev (CI) + lilygo-t3s3-sx1280 (target)
├── BOARD.md              # T3-S3 pinout, verified against LilyGo factory sources
├── flash.sh / flash.md   # full-erase flashing, exact offsets + provenance
├── PROTOCOL.md           # serial JSON contract for the dashboard bridge
├── src/
│   ├── elrs_defs.h       # researched constants, cited (rate table, layouts)
│   ├── elrs_crc.h        # CRC-14/16 (Koopman polys, ELRS semantics)
│   ├── elrs_parse.cpp/.h # classifier + stick/telemetry decoders
│   ├── sniffer_radio.cpp/.h # RadioLib glue + sweep table (+PA RF switch)
│   ├── ui.cpp/.h         # SSD1306 128x64 OLED, OUI-SPY dark theme
│   ├── board_pins.h      # T3-S3 SX1280 pins [VERIFIED]
│   └── main.cpp          # sweep/lock state machine, JSON out, lock LED
└── test/host/test_parser.cpp  # host parser tests (no hardware needed)
```

## Verification

- `pio run` — both envs (`esp32dev`, `lilygo-t3s3-sx1280`) build.
- Host tests (`test/host`): CRC cross-formulation, 4×10 bitpack round-trip,
  OTA8/OTA4 SYNC self-acquisition, RC decode after capture, noise gate —
  all pass (`c++ -std=c++11 -o test_parser test_parser.cpp
  ../../src/elrs_parse.cpp -I../../src && ./test_parser`).
- Not yet done (needs the bench): RF bring-up, OLED check, live-packet
  validation against a real ELRS TX.

## Known TODOs

- 4.x/master air-format drift (sync grew a byte; RC header bit re-map) —
  parser marks these, does not fully decode.
- FLRC listening after UID capture (256-candidate sync-word sweep).
- Wide-switch AUX slotting without nonce tracking (value shown unslotted).
- GPS altitude packing variant (CRSF spec vs ELRS history).
- 2.x "classic" 8-byte nonce-first packets: demod-length compatible, CRC
  claims unverified — marked legacy.
- `tools/elrs_bridge.py` dashboard bridge (against PROTOCOL.md).
