// elrs_parse.cpp — see elrs_parse.h for the layout citations.
#include "elrs_parse.h"
#include "elrs_crc.h"
#include <string.h>
#include <stdlib.h>

void elrs_decode_init(elrs_decode_ctx_t *ctx)
{
    memset(ctx, 0, sizeof(*ctx));
    ctx->switch_mode = ELRS_SW_WIDE; // 3.x default until a sync packet says otherwise
}

uint16_t elrs_crc_init_from_uid(uint8_t uid4, uint8_t uid5)
{
    return (uint16_t)(((uint16_t)uid4 << 8) | uid5) ^ ELRS_OTA_VERSION_ID_3X;
}

uint32_t elrs_uid_mac_seed(uint8_t uid2, uint8_t uid3, uint8_t uid4, uint8_t uid5)
{
    return ((uint32_t)uid2 << 24) | ((uint32_t)uid3 << 16) |
           ((uint32_t)uid4 << 8) | ((uint32_t)uid5 ^ ELRS_OTA_VERSION_ID_3X);
}

// --- channel helpers (OTA.cpp PackUInt11ToChannels4x10 / UnpackChannels4x10) ---

void elrs_unpack_4x10(const uint8_t raw[5], uint16_t ch[4])
{
    // bit-unpack 4x 10-bit LSB-first across 5 bytes (inverse of ELRS packer)
    uint8_t bits_merged = 0;
    uint32_t read_value = 0;
    uint8_t idx = 0;
    for (uint8_t n = 0; n < 4; n++) {
        while (bits_merged < 10) {
            read_value |= ((uint32_t)raw[idx++]) << bits_merged;
            bits_merged += 8;
        }
        ch[n] = (uint16_t)(read_value & 0x3FF);
        read_value >>= 10;
        bits_merged -= 10;
    }
}

// 10-bit OTA value -> CRSF 11-bit channel value (172..1811).
// OTA4 (hybrid/wide): 10 bits cover only 988..2012us (Limit decimator).
// OTA8 (fullres): divide-by-2 packing of the full 11-bit CRSF range.
static uint16_t elrs_10bit_to_crsf(uint16_t v10, bool full_range)
{
    if (full_range) {
        uint16_t crsf = (uint16_t)(v10 * 2);
        return crsf > CRSF_VAL_MAX ? CRSF_VAL_MAX : crsf;
    }
    return (uint16_t)(CRSF_VAL_MIN + ((uint32_t)v10 * (CRSF_VAL_MAX - CRSF_VAL_MIN)) / 1023);
}

// CRSF 11-bit value -> servo pulse microseconds (172 -> 988us, 1811 -> 2012us)
uint16_t elrs_crsfval_to_us(uint16_t crsf)
{
    return (uint16_t)(988 + ((uint32_t)(crsf - CRSF_VAL_MIN) * 1024) / (CRSF_VAL_MAX - CRSF_VAL_MIN));
}

uint32_t elrs_10bit_to_crsf_us(uint16_t v10, bool full_range)
{
    return elrs_crsfval_to_us(elrs_10bit_to_crsf(v10, full_range));
}

// --- CRC validation helpers ---

// OTA4 CRC14: byte0's upper 6 bits are replaced before computing (0 for
// hybrid / non-RC, (nonce % hop)+1 for wide RC). inCRC = byte0[7:2]<<8 | byte7.
static bool ota4_crc_ok(const uint8_t *d, uint16_t init, uint8_t slot_inject)
{
    uint8_t tmp[7];
    memcpy(tmp, d, 7);
    tmp[0] = (uint8_t)((d[0] & 0x03) | (slot_inject << 2));
    uint16_t incrc = (uint16_t)((((uint16_t)d[0] >> 2) << 8) | d[7]);
    return elrs_crc14(tmp, 7, init) == incrc;
}

static bool ota8_crc_ok(const uint8_t *d, uint16_t init)
{
    uint16_t incrc = (uint16_t)(d[11] | ((uint16_t)d[12] << 8));
    return elrs_crc16(d, 11, init) == incrc;
}

// wide-mode slot byte uses (nonce % hop)+1 — nonce is tracked from sync
// packets; search a small window since sync packets are periodic, not per-packet
static bool ota4_rc_crc_ok_wide(elrs_decode_ctx_t *ctx, const uint8_t *d, uint8_t hop)
{
    if (!ctx->nonce_known) return false;
    for (uint8_t k = 0; k < 16; k++) {
        uint8_t nonce = (uint8_t)(ctx->last_nonce + k);
        if (ota4_crc_ok(d, ctx->crc_init, (nonce % hop) + 1)) {
            ctx->last_nonce = nonce;
            return true;
        }
    }
    return false;
}

// try both switch-mode CRC interpretations of an OTA4 RC packet
static bool ota4_rc_crc_ok(elrs_decode_ctx_t *ctx, const uint8_t *d, uint8_t hop)
{
    return ota4_crc_ok(d, ctx->crc_init, 0) || ota4_rc_crc_ok_wide(ctx, d, hop);
}

// --- plausibility heuristics for when the CRC init is not known yet ---

// Without the CRC init, an OTA4/OTA8 RC payload is genuinely hard to tell
// from demod junk (10-bit stick data spans the full field). This prefilter
// only drops the worst tail — packets pinned at the rails on most channels,
// which real stick data essentially never does (at most one axis is pinned,
// e.g. throttle at min). Everything else stays "PLAUSIBLE", promoted to a
// detection only by a CRC-valid sync or sustained dwell statistics.
static bool channels_look_sticky(const uint16_t *ch, uint8_t n)
{
    unsigned pinned = 0;
    for (uint8_t i = 0; i < n; i++) {
        if (ch[i] < 48 || ch[i] > 975) pinned++;
    }
    return pinned < (n >= 8 ? 3 : 2);
}

// --- telemetry fragment reassembly (CRSF frames over ELRS TLM packets) ---

static size_t crsf_frame_len(uint8_t type)
{
    switch (type) {
    case CRSF_FRAMETYPE_GPS: return 16;            // type + 15
    case CRSF_FRAMETYPE_VARIO: return 3;
    case CRSF_FRAMETYPE_BATTERY: return 9;
    case CRSF_FRAMETYPE_BARO_ALT: return 7;
    case CRSF_FRAMETYPE_LINK_STATISTICS: return 11;
    case CRSF_FRAMETYPE_ATTITUDE: return 7;
    case CRSF_FRAMETYPE_FLIGHT_MODE: return 17;
    default: return 0; // unknown -> keep accumulating until reset
    }
}

static void parse_crsf_frame(const uint8_t *f, size_t len, elrs_telemetry_t *tlm)
{
    tlm->valid = true;
    tlm->type = f[0];
    switch (f[0]) {
    case CRSF_FRAMETYPE_GPS:
        if (len < 16) break;
        tlm->lat_e7 = (int32_t)((uint32_t)f[1] | ((uint32_t)f[2] << 8) | ((uint32_t)f[3] << 16) | ((uint32_t)f[4] << 24));
        tlm->lon_e7 = (int32_t)((uint32_t)f[5] | ((uint32_t)f[6] << 8) | ((uint32_t)f[7] << 16) | ((uint32_t)f[8] << 24));
        tlm->gspeed_kmh10 = (uint16_t)(f[9] | ((uint16_t)f[10] << 8));
        tlm->heading_deg100 = (uint16_t)(f[11] | ((uint16_t)f[12] << 8));
        // TODO: altitude packing differs between the CRSF spec and ELRS
        // history ((m+1000) at 0.1m vs raw 0.1m) — verify against a real RX
        tlm->alt_m10 = (int32_t)(f[13] | ((uint16_t)f[14] << 8)) - 10000;
        tlm->sats = f[15];
        break;
    case CRSF_FRAMETYPE_BATTERY:
        if (len < 9) break;
        tlm->batt_mv10 = (uint16_t)(f[1] | ((uint16_t)f[2] << 8));
        tlm->batt_ma10 = (uint16_t)(f[3] | ((uint16_t)f[4] << 8));
        tlm->batt_mah = (uint32_t)f[5] | ((uint32_t)f[6] << 8) | ((uint32_t)f[7] << 16);
        tlm->batt_fuel = f[8];
        break;
    case CRSF_FRAMETYPE_ATTITUDE:
        if (len < 7) break;
        tlm->pitch = (int16_t)(f[1] | ((uint16_t)f[2] << 8));
        tlm->roll = (int16_t)(f[3] | ((uint16_t)f[4] << 8));
        tlm->yaw = (int16_t)(f[5] | ((uint16_t)f[6] << 8));
        break;
    case CRSF_FRAMETYPE_BARO_ALT:
        if (len < 7) break;
        tlm->baro_alt = (int16_t)(f[1] | ((uint16_t)f[2] << 8));
        tlm->vspeed_cms = (int16_t)(f[3] | ((uint16_t)f[4] << 8));
        break;
    case CRSF_FRAMETYPE_VARIO:
        if (len < 3) break;
        tlm->vspeed_cms = (int16_t)(f[1] | ((uint16_t)f[2] << 8));
        break;
    case CRSF_FRAMETYPE_FLIGHT_MODE:
        memcpy(tlm->flight_mode, &f[1], len - 2 > 16 ? 16 : len - 2);
        tlm->flight_mode[16] = 0;
        break;
    default:
        tlm->valid = false; // unknown type: left for raw logging
        break;
    }
}

void elrs_decode_tlm_fragment(elrs_decode_ctx_t *ctx, uint8_t package_index,
                              bool contains_linkstats, const uint8_t *payload,
                              size_t len, elrs_packet_t *out)
{
    const uint8_t *frag = payload;
    size_t frag_len = len;

    if (contains_linkstats && frag_len >= 4) {
        // OTA8: OTA_LinkStats_s rides in front of the telemetry fragment
        elrs_linkstats_t *ls = &out->linkstats;
        ls->rssi1_db = frag[0] & 0x7F;
        ls->antenna = frag[0] >> 7;
        ls->rssi2_db = frag[1] & 0x7F;
        ls->model_match = frag[1] >> 7;
        ls->lq = frag[2] & 0x7F;
        ls->snr_db = (int8_t)frag[3];
        ls->valid = true;
        frag += 4;
        frag_len -= 4;
        if (frag_len == 0) return;
    }

    // packageIndex restarts at 0 for a new frame; a discontinuity resets
    if (!ctx->tlm_frag_active || package_index == 0 ||
        package_index != (uint8_t)(ctx->tlm_last_pkg + 1)) {
        ctx->tlm_frag_active = true;
        ctx->tlm_frag_len = 0;
    }
    ctx->tlm_last_pkg = package_index;

    if (ctx->tlm_frag_len + frag_len > ELRS_TLM_BUF_LEN) {
        ctx->tlm_frag_len = 0; // overflow, start over
        return;
    }
    memcpy(&ctx->tlm_frag[ctx->tlm_frag_len], frag, frag_len);
    ctx->tlm_frag_len += frag_len;

    size_t want = crsf_frame_len(ctx->tlm_frag[0]);
    if (want && ctx->tlm_frag_len >= want) {
        parse_crsf_frame(ctx->tlm_frag, ctx->tlm_frag_len, &out->tlm);
        ctx->tlm_frag_len = 0;
        ctx->tlm_frag_active = false;
    }
}

// --- main entry ---

bool elrs_decode_packet(elrs_decode_ctx_t *ctx, const uint8_t *data, size_t len,
                        elrs_packet_t *out)
{
    memset(out, 0, sizeof(*out));
    out->len = len;
    if (len != ELRS_OTA4_LEN && len != ELRS_OTA8_LEN) return false;
    out->type = data[0] & 0x03;

    const bool is8 = (len == ELRS_OTA8_LEN);
    bool crc_ok = false;

    // ---- CRC validation. Sync packets are special: they carry UID[4..5],
    //      so the CRC init can be *derived from the packet itself*
    //      (init = (UID4<<8|UID5) ^ OTA_VERSION_ID) — that self-acquisition
    //      is how the sniffer captures a link fingerprint. Bind mode uses 0.
    if (out->type == ELRS_PKT_SYNC) {
        uint16_t derived = elrs_crc_init_from_uid(data[5], data[6]);
        if (is8 ? ota8_crc_ok(data, derived) : ota4_crc_ok(data, derived, 0)) {
            crc_ok = true;
            if (!ctx->crc_init_known) {
                ctx->crc_init = derived;
                ctx->crc_init_known = true;
            }
        } else if (is8 ? ota8_crc_ok(data, 0) : ota4_crc_ok(data, 0, 0)) {
            crc_ok = true; // bind mode (CRC init 0)
        }
        if (crc_ok && !ctx->uid_known) {
            ctx->uid3 = data[4];
            ctx->uid4 = data[5];
            ctx->uid5 = data[6];
            ctx->uid_known = true;
            if (!ctx->crc_init_known) {
                ctx->crc_init = elrs_crc_init_from_uid(ctx->uid4, ctx->uid5);
                ctx->crc_init_known = true;
            }
        }
    } else if (ctx->crc_init_known) {
        if (is8) {
            crc_ok = ota8_crc_ok(data, ctx->crc_init);
        } else if (out->type == ELRS_PKT_RCDATA) {
            crc_ok = ota4_rc_crc_ok(ctx, data, 4); // hop=4 (250Hz class; TODO per-rate)
        } else {
            crc_ok = ota4_crc_ok(data, ctx->crc_init, 0);
        }
    }
    out->cls = crc_ok ? ELRS_PKT_CLASS_CRC_OK : ELRS_PKT_CLASS_PLAUSIBLE;

    // ---- per-type decode ----
    switch (out->type) {
    case ELRS_PKT_RCDATA: {
        uint16_t ch[4];
        if (is8) {
            const uint8_t b0 = data[0];
            out->rc.telemetry_status = (b0 >> 2) & 1;
            out->rc.uplink_power = ((b0 >> 3) & 0x07) + 1;
            out->rc.is_high_aux = (b0 >> 6) & 1;
            uint8_t low_idx, high_idx;
            if (ctx->switch_mode == ELRS_SW_HYBRID8) {
                // 16ch fullres mapping (OTA.cpp UnpackChannelData8ch)
                low_idx = out->rc.is_high_aux ? 8 : 0;
                high_idx = out->rc.is_high_aux ? 12 : 4;
            } else {
                // 8ch/12ch mapping; AUX1 arm bit rides in byte0
                out->rc.has_ch[4] = true;
                out->rc.ch[4] = (b0 >> 7) & 1 ? CRSF_VAL_MAX : CRSF_VAL_MIN;
                low_idx = 0;
                high_idx = out->rc.is_high_aux ? 9 : 5;
            }
            elrs_unpack_4x10(&data[1], ch);
            for (int i = 0; i < 4; i++) {
                out->rc.ch[low_idx + i] = elrs_10bit_to_crsf(ch[i], true);
                out->rc.has_ch[low_idx + i] = true;
            }
            elrs_unpack_4x10(&data[6], ch);
            for (int i = 0; i < 4; i++) {
                out->rc.ch[high_idx + i] = elrs_10bit_to_crsf(ch[i], true);
                out->rc.has_ch[high_idx + i] = true;
            }
        } else {
            elrs_unpack_4x10(&data[1], ch);
            for (int i = 0; i < 4; i++) {
                out->rc.ch[i] = elrs_10bit_to_crsf(ch[i], false);
                out->rc.has_ch[i] = true;
            }
            out->rc.has_ch[4] = true;
            out->rc.ch[4] = (data[6] & 1) ? CRSF_VAL_MAX : CRSF_VAL_MIN;
            if (ctx->switch_mode == ELRS_SW_HYBRID8) {
                // hybrid8 round-robin switches (OTA.cpp UnpackChannelDataHybridSwitch8)
                uint8_t sw = data[6] >> 1; // byte is switches:7 | ch4:1
                out->rc.telemetry_status = (sw >> 6) & 1;
                uint8_t swidx = (sw >> 3) & 0x07;
                if (swidx >= 6) { // "index 6" encodes AUX8 hi-res, low bit is data
                    out->rc.has_ch[11] = true;
                    out->rc.ch[11] = (uint16_t)(CRSF_VAL_MIN + (uint32_t)(sw & 0x0F) * (CRSF_VAL_MAX - CRSF_VAL_MIN) / 15);
                } else {
                    out->rc.has_ch[5 + swidx] = true;
                    out->rc.ch[5 + swidx] = (uint16_t)(CRSF_VAL_MIN + (uint32_t)(sw & 0x07) * (CRSF_VAL_MAX - CRSF_VAL_MIN) / 7);
                }
            } else {
                // wide: 6/7-bit round-robin value; the AUX slot is implied by
                // the nonce (TODO without per-packet nonce tracking we park
                // the value on AUX2 and flag it as unslotted)
                out->rc.has_ch[5] = true;
                out->rc.ch[5] = (uint16_t)(CRSF_VAL_MIN + (uint32_t)((data[6] >> 1) & 0x7F) * (CRSF_VAL_MAX - CRSF_VAL_MIN) / 127);
            }
        }
        if (!crc_ok) {
            // weak prefilter on RAW 10-bit stick fields (see gate comment)
            uint16_t raw[8];
            if (is8) {
                elrs_unpack_4x10(&data[1], raw);
                elrs_unpack_4x10(&data[6], raw + 4);
            } else {
                elrs_unpack_4x10(&data[1], raw);
            }
            if (!channels_look_sticky(raw, is8 ? 8 : 4)) {
                out->cls = ELRS_PKT_CLASS_NOISE;
                return false;
            }
        }
        return true;
    }
    case ELRS_PKT_SYNC: {
        out->sync.fhss_index = data[1];
        out->sync.nonce = data[2];
        out->sync.switch_mode = data[3] & 1;
        out->sync.tlm_ratio = (data[3] >> 1) & 0x07;
        out->sync.rate_index = data[3] >> 4;
        out->sync.uid3 = data[4];
        out->sync.uid4 = data[5];
        out->sync.uid5 = data[6];
        out->nonce = data[2];
        if (!crc_ok) {
            // unknown link and not bind mode: only structurally sane 3.x
            // sync packets get through (rate_index in range)
            if (out->sync.rate_index > 9) {
                out->cls = ELRS_PKT_CLASS_NOISE;
                return false;
            }
        } else {
            ctx->last_nonce = data[2];
            ctx->nonce_known = true;
            // switchEncMode bit: 0 = wide/8ch, 1 = hybrid16ch/12ch
            ctx->switch_mode = out->sync.switch_mode ? ELRS_SW_HYBRID8 : ELRS_SW_WIDE;
        }
        return true;
    }
    case ELRS_PKT_TLM: {
        if (is8) {
            uint8_t b0 = data[0];
            bool has_ls = (b0 >> 2) & 1;
            uint8_t pkg = b0 >> 3;
            elrs_decode_tlm_fragment(ctx, pkg, has_ls, &data[1], 10, out);
        } else {
            uint8_t t = data[1] & 0x03;          // ELRS_TELEMETRY_TYPE_*
            uint8_t pkg = data[1] >> 2;          // ELRS4_TELEMETRY_SHIFT=2
            if (t == 0x01) {                     // LINK: 4-byte stats + free
                elrs_linkstats_t *ls = &out->linkstats;
                ls->rssi1_db = data[2] & 0x7F;
                ls->antenna = data[2] >> 7;
                ls->rssi2_db = data[3] & 0x7F;
                ls->model_match = data[3] >> 7;
                ls->lq = data[4] & 0x7F;
                ls->snr_db = (int8_t)data[5];
                ls->valid = true;
            } else if (t == 0x02) {              // DATA: CRSF fragment
                elrs_decode_tlm_fragment(ctx, pkg, false, &data[2], 5, out);
            }
        }
        return crc_ok || !ctx->crc_init_known;
    }
    case ELRS_PKT_MSP:
    default:
        // uplink model/MSP data — counted, not decoded (TODO)
        return crc_ok || !ctx->crc_init_known;
    }
}
