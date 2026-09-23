// =============================================================================
// elrs_fhss.h — ELRS 2.4 GHz FHSS sequence, ported character-for-character
// from ExpressLRS 3.6.4:
//   src/lib/FHSS/random.cpp   — LCG: seed=(214013*seed+2531011) % 2^31; rng=seed>>16
//   src/lib/FHSS/FHSS.cpp     — FHSSrandomiseFHSSsequenceBuild
//   src/lib/FHSS/FHSS.h       — 2.4G domain: 80 x 1 MHz from 2400.4 MHz,
//                               sync_channel = 80/2+1 = 41 (2441.4 MHz),
//                               sequence length = (256/80)*80 = 160
//   src/src/tx_main.cpp:858   — TX hops when (OtaNonce + 1) % hopInterval == 0,
//                               i.e. the sequence position advances once per
//                               hopInterval packets; sync packets are sent
//                               whenever the TX is on the sync channel
//                               (FHSSonSyncChannel), which by construction is
//                               exactly every 80th sequence entry (block starts).
// The sniffer seeds the sequence with uidMacSeedGet() (common.cpp) — which
// needs UID[2], the byte ELRS never broadcasts; main.cpp brute-forces it.
// =============================================================================
#pragma once
#include <stdint.h>
#include "elrs_defs.h"

#define FHSS_FREQ_COUNT 80u
#define FHSS_SYNC_INDEX 41u
#define FHSS_SEQ_COUNT 240u               // (256 / 80) * 80 (ELRS FHSS.cpp:86)
// Register-unit frequency plan (ELRS FREQ_HZ_TO_REG_VAL, FREQ_STEP = 52e6/2^18):
// spread = (stop_reg - start_reg) * 256 / 79; freq(idx)_reg = start + idx*spread/256
#define FHSS_START_REG 12100970u          // FREQ_HZ_TO_REG_VAL(2400400000)
#define FHSS_SPREAD_REG 1290554u          // (stop-start)*256/79

typedef struct { uint32_t s; } elrs_rng_t;

static inline uint16_t elrs_rng(elrs_rng_t *r)
{
    r->s = (214013u * r->s + 2531011u) % 2147483648u;
    return (uint16_t)(r->s >> 16);
}

static inline void elrs_rng_seed(elrs_rng_t *r, uint32_t seed) { r->s = seed; }
static inline uint8_t elrs_rng_n(elrs_rng_t *r, uint8_t max) { return (uint8_t)(elrs_rng(r) % max); }

// FHSSrandomiseFHSSsequenceBuild: block starts at the sync channel; slot
// (block*80 + 41) starts at 0; everything else i%80; then in-block swaps.
static inline void elrs_fhss_build(uint32_t mac_seed, uint8_t seq[FHSS_SEQ_COUNT])
{
    elrs_rng_t r;
    elrs_rng_seed(&r, mac_seed);
    for (uint16_t i = 0; i < FHSS_SEQ_COUNT; i++) {
        if (i % FHSS_FREQ_COUNT == 0) seq[i] = FHSS_SYNC_INDEX;
        else if (i % FHSS_FREQ_COUNT == FHSS_SYNC_INDEX) seq[i] = 0;
        else seq[i] = (uint8_t)(i % FHSS_FREQ_COUNT);
    }
    for (uint16_t i = 0; i < FHSS_SEQ_COUNT; i++) {
        if (i % FHSS_FREQ_COUNT != 0) {
            uint8_t offset = (uint8_t)((i / FHSS_FREQ_COUNT) * FHSS_FREQ_COUNT);
            uint8_t rand = (uint8_t)(elrs_rng_n(&r, FHSS_FREQ_COUNT - 1) + 1);
            uint8_t t = seq[i];
            seq[i] = seq[offset + rand];
            seq[offset + rand] = t;
        }
    }
}

static inline uint32_t elrs_fhss_channel_reg(uint8_t ch)
{
    return FHSS_START_REG + (uint32_t)ch * FHSS_SPREAD_REG / 256u;
}
static inline uint32_t elrs_fhss_channel_hz(uint8_t ch)
{
    return (uint32_t)(((uint64_t)elrs_fhss_channel_reg(ch) * 52000000u) >> 18);
}
#define ELRS_2G4_SYNC_FREQ_HZ_DRV elrs_fhss_channel_hz(FHSS_SYNC_INDEX) // 2441399841

// FIND-gate helper (round 8): fhssIndex is the SEQUENCE POINTER, not the
// channel — a sync frame was sent ON the sync channel iff seq[pointer]==41,
// which for any UID happens exactly at the block starts (p % 80 == 0).
static inline bool elrs_sync_on_sync_channel(uint8_t fhss_ptr,
                                             const uint8_t *seq, bool seq_known)
{
    if (fhss_ptr >= FHSS_SEQ_COUNT) return false;
    if (seq_known) return seq[fhss_ptr] == FHSS_SYNC_INDEX;
    return (fhss_ptr % FHSS_FREQ_COUNT) == 0;
}

// TX position advances one sequence entry per hopInterval packets
// (tx_main.cpp: (OtaNonce+1) % hopInterval == 0).
static inline uint16_t elrs_fhss_advance(uint16_t idx, uint32_t packets, uint8_t hop)
{
    return (uint16_t)((idx + packets / hop) % FHSS_SEQ_COUNT);
}

// --- reference-RX port (rx_main.cpp 3.6.4) ---------------------------------
// minLqForChaos (rx_main.cpp:273): most CRC-passing packets receivable on
// ONE channel per 100-slot LQ window by chance; tentative->connected needs
// LQ GREATER than this (rx_main.cpp:2227). hop=4, 80 ch -> 4; hop=2 -> 2.
static inline uint8_t elrs_min_lq_for_chaos(uint8_t hop)
{
    return (uint8_t)(hop * ((hop * FHSS_FREQ_COUNT + 99) / (hop * FHSS_FREQ_COUNT)));
}

// Nonce tracking per the RX. The RX free-runs a timer at the air rate and
// increments OtaNonce per slot (HWtimerCallbackTick, rx_main.cpp:678); a
// sync re-anchors only when sync.nonce == tracked OtaNonce, otherwise the
// mismatch IS a resync event (ProcessRfPacket_SYNC, rx_main.cpp:1092-1100:
// FHSSsetCurrIndex(sync.fhssIndex), OtaNonce = sync.nonce, TentativeConn).
// We have no hw timer: expected nonce is derived from WALL TIME since the
// anchor at the locked rate's packet interval (same discipline).
typedef struct {
    uint8_t  anchor_nonce;
    uint32_t anchor_ms;
    uint32_t interval_ms;   // locked rate packet period
} elrs_nonce_track_t;

static inline void elrs_nonce_anchor(elrs_nonce_track_t *t, uint8_t nonce,
                                     uint32_t now_ms, uint32_t interval_ms)
{
    t->anchor_nonce = nonce;
    t->anchor_ms = now_ms;
    t->interval_ms = interval_ms ? interval_ms : 1;
}

static inline uint8_t elrs_nonce_expected(const elrs_nonce_track_t *t, uint32_t now_ms)
{
    if (t->interval_ms == 0) return 0xFE; // never anchored — see on_track guard
    return (uint8_t)(t->anchor_nonce + (now_ms - t->anchor_ms) / t->interval_ms);
}

// RX semantics: a sync is on-track iff sync.nonce == expected slot nonce.
// Unanchored (pre-first-sync) tracks are NEVER on-track — without this guard
// the first sync of a session divides by interval_ms==0 and the CPU panics
// with IntegerDivideByZero (field-captured: Guru Meditation right after the
// first "crack identity").
static inline bool elrs_nonce_on_track(const elrs_nonce_track_t *t, uint32_t now_ms, uint8_t nonce)
{
    if (t->interval_ms == 0) return false;
    return nonce == elrs_nonce_expected(t, now_ms);
}

// --- UID[2] trackers (find mode) -------------------------------------------
// A validated LoRa sync gives UID[3..5] but the FHSS seed macSeed also needs
// UID[2] (never broadcast). 256 parallel trackers, one per candidate UID[2]:
// each predicts the sequence entry at the sync's position; a mismatch kills
// the tracker. Survivor after >=2 syncs = full UID. Host-tested with the
// real LCG/sequence code.
typedef struct {
    uint8_t alive[16];     // 128 bits — UID[2] bit7 is INVISIBLE to the FHSS
    uint8_t t0_nonce;      // sequence (mod 2^31 drops it): u2 and u2^0x80
    uint8_t t0_fhss;       // produce identical sequences
    bool armed;
    uint8_t syncs;         // validated syncs processed
} elrs_uid2_track_t;

static inline void elrs_uid2_track_init(elrs_uid2_track_t *t, uint8_t nonce, uint8_t fhss)
{
    memset(t->alive, 0xFF, sizeof(t->alive));
    t->t0_nonce = nonce;
    t->t0_fhss = fhss;
    t->armed = true;
    t->syncs = 1;
}

// returns the surviving UID[2] (0..255) once exactly one candidate remains
// and at least 2 syncs have been processed; -1 otherwise; -2 = all dead.
static inline int elrs_uid2_track_update(elrs_uid2_track_t *t, uint8_t uid2_skip,
                                         uint8_t uid3, uint8_t uid4, uint8_t uid5,
                                         uint8_t sync_nonce, uint8_t sync_fhss,
                                         uint8_t hop)
{
    if (!t->armed) return -1;
    t->syncs++;
    uint32_t slots = (uint8_t)(sync_nonce - t->t0_nonce); // per-packet nonce
    uint16_t idx = (uint16_t)((t->t0_fhss + slots / hop) % FHSS_SEQ_COUNT);
    int survivor = -1;
    unsigned alive_n = 0;
    for (uint16_t u2 = 0; u2 < 128; u2++) {
        if (!(t->alive[u2 / 8] & (uint8_t)(1u << (u2 % 8)))) continue;
        uint8_t seq[FHSS_SEQ_COUNT];
        elrs_fhss_build(((uint32_t)u2 << 24) | ((uint32_t)uid3 << 16) |
                        ((uint32_t)uid4 << 8) | ((uint32_t)uid5 ^ ELRS_OTA_VERSION_ID_3X), seq);
        if (seq[idx] != sync_fhss)
            t->alive[u2 / 8] &= (uint8_t)~(1u << (u2 % 8));
        else { survivor = (int)u2; alive_n++; }
    }
    (void)uid2_skip;
    if (alive_n == 0) return -2;
    if (alive_n == 1 && t->syncs >= 2) return survivor; // 7-bit survivor; the
    return -1;                                        // real UID[2] is this or
}                                                     // this | 0x80
