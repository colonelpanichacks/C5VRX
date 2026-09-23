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

void elrs_identity_reset(elrs_identity_t *id);
bool elrs_identity_sane(const elrs_sync_info_t *s);  // structural checks only
// returns true the moment identity is ACCEPTED (2nd consistent sync)
bool elrs_identity_consider(elrs_identity_t *id, const elrs_sync_info_t *s, uint8_t hop);

// --- helpers shared with the host tests ---
uint32_t elrs_10bit_to_crsf_us(uint16_t v10, bool full_range);
uint16_t elrs_crsfval_to_us(uint16_t crsf);  // CRSF 11-bit value -> us
void     elrs_unpack_4x10(const uint8_t raw[5], uint16_t ch[4]);
