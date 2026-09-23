// =============================================================================
// elrs_crc24flrc.h — offline model of the SX1280 GFSK/FLRC radio CRC
// (DS_SX1280-1 Rev3.2 Table 14-12/14-13: 16-bit CRC Initial Value at
// 0x9C8/0x9C9 = "CRC Seed used for GFSK and FLRC"; 16-bit polynomial at
// 0x9C6/0x9C7, chip default 0xFFFF; ELRS seeds it with OtaCrcInitializer and
// leaves the polynomial at default). ELRS FLRC uses the 3-byte CRC.
//
// VERIFICATION STATUS (honest): the SEED story is datasheet-confirmed, but
// the silicon LFSR's bit-level details (init placement, CRC field byte
// order, exact coverage) are NOT documented in the register tables. This
// module therefore implements several explicit VARIANTS; the seed brute
// tries every variant, and tools/flrc_crc_probe.py identifies the live
// variant from a capture of a KNOWN-phrase FLRC link. Do not trust a single
// variant until the probe confirms it against silicon.
// =============================================================================
#pragma once
#include <stdint.h>
#include <stddef.h>

#define ELRS_CRC24FLRC_VARIANTS 4

// variant 0: MSB-first, 24-bit state, poly 0xFFFF00 (implicit top bit),
//            init = seed<<8, CRC field = last 3 bytes big-endian, payload only
// variant 1: same but init = seed (low 16 bits, top byte 0xFF)
// variant 2: same as 0 but CRC field little-endian
// variant 3: poly treated as full 0xFFFF (degree-16), init = seed<<8
static inline uint32_t elrs_crc24flrc_calc(uint8_t variant, uint16_t seed,
                                           const uint8_t *data, size_t len)
{
    uint32_t st;
    uint32_t poly;
    switch (variant) {
    default:
    case 0: st = ((uint32_t)seed << 8) & 0xFFFFFF; poly = 0xFFFF00; break;
    case 1: st = 0xFF0000 | seed;                  poly = 0xFFFF00; break;
    case 2: st = ((uint32_t)seed << 8) & 0xFFFFFF; poly = 0xFFFF00; break;
    case 3: st = ((uint32_t)seed << 8) & 0xFFFFFF; poly = 0x00FFFF; break;
    }
    for (size_t i = 0; i < len; i++) {
        st ^= (uint32_t)data[i] << 16;
        for (uint8_t b = 0; b < 8; b++)
            st = (st & 0x800000) ? ((st << 1) ^ poly) : (st << 1);
        st &= 0xFFFFFF;
    }
    return st;
}

// validate a captured fixed-length FLRC frame (payload incl. its 3 CRC bytes)
static inline bool elrs_crc24flrc_check(uint8_t variant, uint16_t seed,
                                        const uint8_t *frame, size_t len)
{
    if (len < 4) return false;
    uint32_t calc = elrs_crc24flrc_calc(variant, seed, frame, len - 3);
    uint32_t got;
    if (variant == 2) // little-endian CRC field
        got = (uint32_t)frame[len - 1] << 16 | (uint32_t)frame[len - 2] << 8 | frame[len - 3];
    else
        got = (uint32_t)frame[len - 3] << 16 | (uint32_t)frame[len - 2] << 8 | frame[len - 1];
    return calc == got;
}

// brute the 2^16 seed space across all variants; returns true on a hit,
// filling seed_out/variant_out. Two-frame confirmation is the caller's job.
static inline bool elrs_crc24flrc_brute(const uint8_t *frame, size_t len,
                                        uint16_t *seed_out, uint8_t *variant_out)
{
    for (uint32_t seed = 0; seed < 65536; seed++) {
        for (uint8_t v = 0; v < ELRS_CRC24FLRC_VARIANTS; v++) {
            if (elrs_crc24flrc_check(v, (uint16_t)seed, frame, len)) {
                *seed_out = (uint16_t)seed;
                *variant_out = v;
                return true;
            }
        }
    }
    return false;
}
