// =============================================================================
// elrs_crc24flrc.h — offline model of the SX1280 FLRC radio CRC.
// AUDIT FIX (round 8): the FLRC CRC polynomial is FIXED at 0x5D6DCB
// (DS_SX1280-1 Rev3.2 Table 14-40) — NOT the GFSK polynomial register.
// ELRS seeds the CRC Initial Value regs (0x9C8/0x9C9) with
// OtaCrcInitializer; the poly register is GFSK-only and irrelevant here.
// Variant dimension is now ONLY the undocumented init placement / CRC field
// byte order (3 variants). The 2^16 seed brute therefore CAN match silicon
// once the variant is right; tools/flrc_crc_probe.py identifies it from a
// known-phrase capture.
// =============================================================================
#pragma once
#include <stdint.h>
#include <stddef.h>

#define ELRS_CRC24FLRC_POLY 0x5D6DCBu
#define ELRS_CRC24FLRC_VARIANTS 3

// V0: init = seed<<8 (top), CRC field big-endian
// V1: init = 0xFF0000 | seed (bottom), CRC field big-endian
// V2: init = seed<<8, CRC field little-endian
// All: MSB-first, 24-bit state, fixed poly 0x5D6DCB, payload-only coverage.
static inline uint32_t elrs_crc24flrc_calc(uint8_t variant, uint16_t seed,
                                           const uint8_t *data, size_t len)
{
    uint32_t st = (variant == 1) ? (0xFF0000u | seed)
                                 : (((uint32_t)seed << 8) & 0xFFFFFF);
    for (size_t i = 0; i < len; i++) {
        st ^= (uint32_t)data[i] << 16;
        for (uint8_t b = 0; b < 8; b++)
            st = (st & 0x800000) ? ((st << 1) ^ ELRS_CRC24FLRC_POLY) : (st << 1);
        st &= 0xFFFFFF;
    }
    return st;
}

static inline bool elrs_crc24flrc_check(uint8_t variant, uint16_t seed,
                                        const uint8_t *frame, size_t len)
{
    if (len < 4) return false;
    uint32_t calc = elrs_crc24flrc_calc(variant, seed, frame, len - 3);
    uint32_t got = (variant == 2)
        ? ((uint32_t)frame[len - 1] << 16 | (uint32_t)frame[len - 2] << 8 | frame[len - 3])
        : ((uint32_t)frame[len - 3] << 16 | (uint32_t)frame[len - 2] << 8 | frame[len - 1]);
    return calc == got;
}

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
