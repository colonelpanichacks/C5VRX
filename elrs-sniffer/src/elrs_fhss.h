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
#define FHSS_SEQ_COUNT 160u               // (256 / 80) * 80
#define FHSS_BASE_HZ    2400400000u
#define FHSS_STEP_HZ    1000000u

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

static inline uint32_t elrs_fhss_channel_hz(uint8_t ch)
{
    return FHSS_BASE_HZ + (uint32_t)ch * FHSS_STEP_HZ;
}

// TX position advances one sequence entry per hopInterval packets
// (tx_main.cpp: (OtaNonce+1) % hopInterval == 0).
static inline uint16_t elrs_fhss_advance(uint16_t idx, uint32_t packets, uint8_t hop)
{
    return (uint16_t)((idx + packets / hop) % FHSS_SEQ_COUNT);
}
