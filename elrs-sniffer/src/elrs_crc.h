// =============================================================================
// elrs_crc.h — ELRS software CRC (the ONLY integrity check on ELRS LoRa links;
// the SX1280 radio CRC is switched off in the ELRS air config).
//
// Semantics (src/test/test_crc/test_crc.cpp + lib/CRC/crc.h):
//   MSB-first, no reflection, no final XOR, init passed in (not inverted).
//   CRC-14: poly 0x2E57, mask 0x3FFF (OTA4, 8-byte packets)
//   CRC-16: poly 0x3D65, mask 0xFFFF (OTA8, 13-byte packets)
// OTA.cpp: OtaCrcInitializer = ((UID[4]<<8)|UID[5]) ^ OTA_VERSION_ID, or 0 in
// bind mode.
// =============================================================================
#pragma once
#include <stdint.h>
#include <stddef.h>
#include "elrs_defs.h"

static inline uint16_t elrs_crc_update(uint16_t crc, uint8_t byte, uint16_t poly)
{
    crc ^= (uint16_t)byte << 8;
    for (uint8_t i = 0; i < 8; i++) {
        crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ poly) : (uint16_t)(crc << 1);
    }
    return crc;
}

// bits: 14 or 16. Returns the masked checksum.
static inline uint16_t elrs_crc_calc(const uint8_t *data, size_t len, uint16_t init,
                                     uint16_t poly, uint16_t mask)
{
    uint16_t crc = init & mask;
    for (size_t i = 0; i < len; i++) {
        crc = elrs_crc_update(crc, data[i], poly);
    }
    return (uint16_t)(crc & mask);
}

static inline uint16_t elrs_crc14(const uint8_t *data, size_t len, uint16_t init)
{
    return elrs_crc_calc(data, len, init, ELRS_CRC14_POLY, ELRS_CRC14_MASK);
}

static inline uint16_t elrs_crc16(const uint8_t *data, size_t len, uint16_t init)
{
    return elrs_crc_calc(data, len, init, ELRS_CRC16_POLY, ELRS_CRC16_MASK);
}
