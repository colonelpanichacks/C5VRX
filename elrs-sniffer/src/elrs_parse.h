// =============================================================================
// elrs_parse.h — ELRS OTA packet classification + decoding (3.x format).
//
// Classifies demodulated payloads into RC data / SYNC / TLM / MSP, validates
// the ELRS software CRC when a link fingerprint (CRC init) is known, decodes
// channel data to microseconds and CRSF telemetry fragments into typed
// fields. Layouts and scaling per src/lib/OTA/OTA.{h,cpp} @ 3.6.4 — see
// elrs_defs.h for the full cited constant block.
//
// Identity honesty: ELRS packets carry NO full TX UID. A SYNC packet reveals
// UID[3..5]; together with the CRC initializer that fingerprints a link, it
// does not uniquely identify an aircraft (shared bind phrases share it).
// =============================================================================
#pragma once
#include <stdint.h>
#include <stddef.h>
#include "elrs_defs.h"

#define ELRS_MAX_CHANNELS 16
#define ELRS_TLM_BUF_LEN  64   // reassembly buffer for fragmented CRSF frames

typedef struct {
    uint32_t ch[ELRS_MAX_CHANNELS]; // CRSF 11-bit values (172..1811), 0 = unknown
    bool     has_ch[ELRS_MAX_CHANNELS];
    uint8_t  telemetry_status;      // OTA8 telemetryStatus bit (TLM expected)
    uint8_t  uplink_power;          // OTA8 uplinkPower field +1 (CRSF power level)
    bool     is_high_aux;
} elrs_rc_t;

typedef struct {
    bool    valid;
    uint8_t rssi1_db;      // |dBm|, antenna 0
    uint8_t rssi2_db;      // |dBm|, antenna 1
    uint8_t lq;            // 0..100
    int8_t  snr_db;        // as reported by the link
    uint8_t antenna : 1;
    uint8_t model_match : 1;
} elrs_linkstats_t;

typedef struct {
    bool    valid;
    uint8_t type;          // CRSF frame type (CRSF_FRAMETYPE_*)
    // GPS
    int32_t lat_e7, lon_e7;
    uint16_t gspeed_kmh10;
    uint16_t heading_deg100;
    int32_t alt_m10;       // TODO: CRSF alt offset/resolution variant
    uint8_t sats;
    // Battery
    uint16_t batt_mv10;    // 0.1 V
    uint16_t batt_ma10;    // 0.1 A
    uint32_t batt_mah;
    uint8_t  batt_fuel;
    // Attitude (rad*10000)
    int16_t pitch, roll, yaw;
    // Vario / baro
    int16_t vspeed_cms;
    int16_t baro_alt;      // 0.1 m, +1000 m offset per CRSF spec
    // Flight mode
    char    flight_mode[17];
} elrs_telemetry_t;

typedef struct {
    uint8_t        type;        // ELRS_PKT_* (low 2 bits of byte 0)
    elrs_pkt_class_t cls;       // CRC validated / plausible / noise
    size_t         len;         // 8 or 13
    elrs_rc_t      rc;
    elrs_sync_info_t sync;
    elrs_linkstats_t linkstats; // from TLM packets containing them
    elrs_telemetry_t tlm;       // last reassembled telemetry frame
    uint8_t        nonce;       // raw nonce (sync), for wide-mode CRC tracking
} elrs_packet_t;

// Decoding context: what the sniffer knows about the link so far.
typedef struct {
    bool     crc_init_known;
    uint16_t crc_init;
    // secondary validation seed: the configured phrase's UID tail, set by
    // the app after every bind-phrase change (multi-UID sync validation)
    uint8_t  cfg_uid4, cfg_uid5;
    bool     cfg_uid_valid;
    uint8_t  model_id;      // recovered ELRS modelId (0xFF = model match off/none)
    uint8_t  otaver;        // 0 unknown, 3, 4 (which CRC-init family validated)
    uint8_t  exp_nonce;     // expected slot nonce (4.x non-sync init mixing)
    bool     exp_valid;
    uint8_t  layout;        // sync layout: 0 unknown, 3, 4
    bool     uid_known;
    uint8_t  uid3, uid4, uid5;
    uint8_t  switch_mode;       // ELRS_SW_* from sync packet (best guess if none)
    uint8_t  last_nonce;        // last nonce seen in a sync packet
    bool     nonce_known;
    // telemetry fragment reassembly
    uint8_t  tlm_frag[ELRS_TLM_BUF_LEN];
    uint8_t  tlm_frag_len;
    uint8_t  tlm_frag_type;
    uint8_t  tlm_last_pkg;      // packageIndex continuity
    bool     tlm_frag_active;
} elrs_decode_ctx_t;

void elrs_decode_init(elrs_decode_ctx_t *ctx);

// Classify + decode one demodulated payload. RSSI/SNR are filled by caller.
// Returns true when the packet is ELRS-classified (cls != NOISE).
bool elrs_decode_packet(elrs_decode_ctx_t *ctx, const uint8_t *data, size_t len,
                        elrs_packet_t *out);

// Feed a TLM payload fragment (handles both OTA4 5-byte and OTA8 10-byte
// chunks, with the OTA8 LinkStats prefix when present).
void elrs_decode_tlm_fragment(elrs_decode_ctx_t *ctx, uint8_t package_index,
                              bool contains_linkstats, const uint8_t *payload,
                              size_t len, elrs_packet_t *out);

// Link fingerprint -> CRC init (OTA.cpp OtaUpdateCrcInitFromUid).
uint16_t elrs_crc_init_from_uid(uint8_t uid4, uint8_t uid5);
// FLRC sync word / FHSS MAC seed (common.cpp uidMacSeedGet). Needs UID[2].
uint32_t elrs_uid_mac_seed(uint8_t uid2, uint8_t uid3, uint8_t uid4, uint8_t uid5);

// ---------------------------------------------------------------------------
// Identity gating (field-proven against false syncs): a sync candidate only
// becomes the link IDENTITY after 2 consecutive sync-structured packets with
// the SAME UID tail, sane fields (rateIdx <= 9, tlmRatio enum <= 8,
// fhss < 160), and nonce/fhss advancing per cadence (equal allowed for
// disconnected beacons). Chance 2^-14 CRC hits and discovery-mode noise show
// rotating tails / random nonces and never reach 2 consistent hits.
typedef struct {
    uint8_t u3, u4, u5;     // candidate tail
    uint8_t count;          // consecutive consistent syncs for this tail
    uint8_t last_nonce;
    uint8_t last_fhss;
    bool known;             // identity accepted (count reached 2)
} elrs_identity_t;

// PRIMARY LoRa sync validator, zero prior knowledge (ELRS 3.6.4 facts):
// OTA4 sync [0]=type|crcHigh [1]=fhss [2]=nonce [3]=sw|tlm|rate [4..6]=UID3..5
// (UID5 XORed (~modelId & 0x3f) when model match is on) [7]=crcLow. CRC14
// poly 0x2E57, init = ((UID4<<8)|UID5) ^ 3 over bytes 0..6, inCRC =
// (byte0>>2)<<8 | byte7. The seed is self-derivable: read b5/b6 as candidate
// UID4/UID5, compute the seed FROM THE FRAME, validate; then sweep modelId
// 0..63 (UID5' = UID5 ^ (~m & 0x3f), recompute seed+CRC) to recover the true
// UID5 + modelId. Works for OTA8 (CRC16) identically (same init formula).
// Returns true on acceptance; fills init_out, uid5_true_out, and
// model_id_out (0xFF = no model-match XOR needed/disabled).
bool elrs_sync_crc_selfseed(const uint8_t *data, size_t len,
                            uint16_t *init_out, uint8_t *uid5_true_out,
                            uint8_t *model_id_out);

// FLRC-discovery classifier (round 5: WINDOWED TAIL COUNTING). The round-3
// consecutive-pair gate could never fire in the field: a real DVDA link
// yields only ~6-12 same-tail frames/s on a dwell channel among ~800
// WiFi-junk frames/s, so real frames are never adjacent. Instead: per-dwell
// sliding window (~2 s) of RSSI-gated, structurally-sane tails; a tail
// becomes a CANDIDATE at >= ELRS_DISC_NEEDED sightings in the window.
// Junk safety: 800 random 24-bit tails/s -> expected same-tail collisions
// per window ~= 0; a real link candidates in <1 s.
#define ELRS_DISC_NF_START -120.0f
#define ELRS_DISC_NF_MARGIN 12.0f
#define ELRS_DISC_ABS_FLOOR -100.0f
#define ELRS_DISC_WINDOW_MS 2000u
#define ELRS_DISC_NEEDED 3
#define ELRS_DISC_SLOTS 16
#define ELRS_DISC_EMIT_MIN_MS 2000u

typedef struct {
    float   nf;                          // running noise floor (min)
    uint8_t needed;                      // sightings required (configurable)
    struct {
        uint8_t  u3, u4, u5;
        uint8_t  count;
        uint32_t last_ms;                // 0 = free slot
    } slot[ELRS_DISC_SLOTS];
    uint32_t last_emit_ms;               // per-tail emission throttle
    uint8_t  le3, le4, le5;
} elrs_disc_gate_t;

void elrs_disc_reset(elrs_disc_gate_t *g);
// returns true exactly when a flrc_sync/adopt event should fire
bool elrs_disc_frame(elrs_disc_gate_t *g, const elrs_sync_info_t *s,
                     float rssi_dbm, uint32_t now_ms);

// Dwell-extension predicate (round-4): extend ONLY on CRC-validated packets
// and/or pair/window-gated discovery candidates — NEVER on raw
// type-classifier labels.
static inline bool elrs_dwell_extend(uint32_t crc_ok_count, uint32_t cand_count)
{
    return crc_ok_count > 0 || cand_count > 0;
}

// Interferer bail: >ELRS_INTERFERER_FPS frames/s sustained
// ELRS_INTERFERER_MS with zero accepted same-tail candidates = junk.
#define ELRS_INTERFERER_FPS 200u
#define ELRS_INTERFERER_MS 500u
static inline bool elrs_interferer_bail(uint32_t fps, uint32_t cand_count,
                                        uint32_t sustained_ms)
{
    return fps > ELRS_INTERFERER_FPS && cand_count == 0 &&
           sustained_ms >= ELRS_INTERFERER_MS;
}

// Last-link tail freshness: usable for 60 s after demote, then decays.
#define ELRS_LASTLINK_DECAY_MS 60000u
static inline bool elrs_lastlink_fresh(bool uid_known, uint32_t demote_ms,
                                       uint32_t now_ms)
{
    return uid_known && (now_ms - demote_ms <= ELRS_LASTLINK_DECAY_MS);
}

// ELRS bind-mode broadcast (tx_main.cpp 3.6.4 SendUIDOverMSP + EnterBinding-
// Mode): a TX in bind mode sends, on the sync channel, LoRa 50Hz, INVERTED
// IQ, CRC init 0, locked nonce: OTA4 MSP packets whose 5-byte payload is
// [MSP_ELRS_BIND=0x09, UID[2], UID[3], UID[4], UID[5]]. That is the one
// place ELRS ever broadcasts UID[2..5] in plaintext — the reliable passive
// UID source. Layout (8 bytes): [0]=type 0b01|crcHi6 [1]=pkgIdx:7|tlm:1
// [2]=0x09 [3..6]=UID2..5 [7]=crcLo; CRC14 poly 0x2E57 init 0 over 0..6.
#define ELRS_MSP_BIND 0x09
bool elrs_bind_parse(const uint8_t *data, size_t len, uint8_t uid2_5[4]);

// elrs_sig quality (round 9): honest ELRS-presence signal for the dashboard
// even before any fingerprint validates. firm = >=3 crc_pass in the pass OR
// >=10 sync_struct with >=1 repeated tail; strong = >=10 crc_pass;
// weak = >=3 sync_struct; else none.
typedef enum { ELRS_SIG_NONE = 0, ELRS_SIG_WEAK, ELRS_SIG_FIRM, ELRS_SIG_STRONG } elrs_sig_t;
static inline elrs_sig_t elrs_sig_quality(uint32_t crc_pass, uint32_t sync_struct,
                                          bool repeat_tail)
{
    if (crc_pass >= 10) return ELRS_SIG_STRONG;
    if (crc_pass >= 3 || (sync_struct >= 10 && repeat_tail)) return ELRS_SIG_FIRM;
    if (sync_struct >= 3) return ELRS_SIG_WEAK;
    return ELRS_SIG_NONE;
}
static inline const char *elrs_sig_name(elrs_sig_t q)
{
    return q == ELRS_SIG_STRONG ? "strong" : q == ELRS_SIG_FIRM ? "firm"
         : q == ELRS_SIG_WEAK ? "weak" : "none";
}

// rxpkt export rate cap (round 10): sliding 1 s window, max `max_per_s`
// exports; returns false when the frame must be dropped (caller counts it).
static inline bool elrs_rxcap_allow(uint32_t now_ms, uint32_t *window_ms,
                                    uint8_t *count, uint8_t max_per_s)
{
    if (now_ms - *window_ms >= 1000) {
        *window_ms = now_ms;
        *count = 0;
    }
    if (*count >= max_per_s) return false;
    (*count)++;
    return true;
}

// SX1280 REG_SF_ADDITIONAL_CONFIG (0x925) value per SF (SX1280.cpp:283-299):
// 0x1E for SF5/SF6, 0x37 for SF7/SF8, 0x32 for SF9+. RadioLib omits this
// register entirely — a likely deafness cause. sf is the register enum
// (0x50 = SF5 .. 0x90 = SF9).
static inline uint8_t elrs_sf_additional_config(uint8_t sf)
{
    if (sf <= 0x60) return 0x1E;
    if (sf <= 0x80) return 0x37;
    return 0x32;
}
#define ELRS_REG_SF_ADDITIONAL_CONFIG 0x925u

// ROUND 13: the exact per-dwell op sequence (ELRS-matched, src/sniffer_radio
// apply()/start_rx() emit these in order). The enum order IS the contract;
// the host test guards against reordering. Sequence: standby -> packettype
// -> modparams -> 0x925 -> freq -> packetparams -> rx_cont -> setrx.
typedef enum {
    DWELL_OP_STANDBY = 0,
    DWELL_OP_PACKET_TYPE,
    DWELL_OP_MOD_PARAMS,
    DWELL_OP_SF925,
    DWELL_OP_FREQUENCY,
    DWELL_OP_PACKET_PARAMS,
    DWELL_OP_RX_CONT,
    DWELL_OP_SET_RX,
    DWELL_OP_COUNT
} dwell_op_t;
static inline const char *dwell_op_name(dwell_op_t op)
{
    static const char *names[] = {
        "standby", "packettype", "modparams", "sf925",
        "frequency", "packetparams", "rx_cont", "setrx"
    };
    return (op >= 0 && op < DWELL_OP_COUNT) ? names[op] : "?";
}
// GET_STATUS chipmode (bits 7:5): 5 = RX
static inline bool elrs_chipmode_is_rx(uint8_t status_byte)
{
    return ((status_byte >> 5) & 0x07) == 5;
}
// FS-expiry safety net: re-arm only if no RxDone AND no re-arm for >2 s
static inline bool elrs_should_rearm(uint32_t now_ms, uint32_t last_pkt_ms,
                                     uint32_t last_arm_ms)
{
    return (now_ms - last_pkt_ms > 2000) && (now_ms - last_arm_ms > 2000);
}

// escan (round 14): measure the AIR directly when LoRa packet-complete
// gives no partial credit. Sweep math is host-tested; the firmware emits the
// measurement.
#define ELRS_ESCAN_START_HZ  2439400000u  // 2439.40 MHz
#define ELRS_ESCAN_STEP_HZ   100000u      // 100 kHz
#define ELRS_ESCAN_POINTS    41u          // 2439.40 .. 2443.40 MHz
#define ELRS_ESCAN_CENTER_IDX 20u         // 2439.40 + 20*0.1 = 2441.40 MHz
static inline uint32_t elrs_escan_freq_hz(uint32_t idx)
{
    return ELRS_ESCAN_START_HZ + idx * ELRS_ESCAN_STEP_HZ;
}
static inline bool elrs_escan_params_ok(void)
{
    return elrs_escan_freq_hz(ELRS_ESCAN_CENTER_IDX) == 2441400000u && // nominal center
           ELRS_ESCAN_POINTS == 41u &&
           elrs_escan_freq_hz(ELRS_ESCAN_POINTS - 1) == 2443400000u;
}

// round 15: live IRQ visibility predicates. RX_DONE is IRQ bit 1 (0x0002).
static inline bool elrs_irq_rxdone(uint16_t irq_word)
{
    return (irq_word & 0x0002) != 0;
}
// ISR-gap detector: the IRQ poll sees RX_DONE latched while DIO1 was never
// observed this dwell -> the chip completes packets but the ISR path drops them.
static inline bool elrs_dio_miss(uint16_t irq_word, bool dio_seen)
{
    return elrs_irq_rxdone(irq_word) && !dio_seen;
}

void elrs_identity_reset(elrs_identity_t *id);
bool elrs_identity_sane(const elrs_sync_info_t *s);  // structural checks only
// returns true the moment identity is ACCEPTED (2nd consistent sync)
bool elrs_identity_consider(elrs_identity_t *id, const elrs_sync_info_t *s, uint8_t hop);

// --- helpers shared with the host tests ---
uint32_t elrs_10bit_to_crsf_us(uint16_t v10, bool full_range);
uint16_t elrs_crsfval_to_us(uint16_t crsf);  // CRSF 11-bit value -> us
void     elrs_unpack_4x10(const uint8_t raw[5], uint16_t ch[4]);
