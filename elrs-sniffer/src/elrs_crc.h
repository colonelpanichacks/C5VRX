// =============================================================================
// elrs_crc.h — ExpressLRS software CRC (the ONLY integrity check on ELRS LoRa
// links; the SX1280 radio CRC is switched OFF in the ELRS air config).
//
// EXACT behavioral port of ELRS 3.6.4 src/lib/CRC/crc.cpp (Crc2Byte), as used
// by src/lib/OTA/OTA.cpp (ValidatePacketCrcStd/Full):
//
//   void Crc2Byte::init(uint8_t bits, uint16_t poly) {
//       _bitmask = (1 << bits) - 1;
//       uint16_t highbit = 1 << (bits - 1);
//       for (i = 0; i < 256; i++) {
//           crc = i << (bits - 8);
//           for (j = 0; j < 8; j++)
//               crc = (crc << 1) ^ ((crc & highbit) ? poly : 0);
//           _crctab[i] = crc;                    // entries NOT masked
//       }
//   }
//   uint16_t Crc2Byte::calc(uint8_t *data, uint8_t len, uint16_t crc) {
//       while (len--)
//           crc = (crc << 8) ^ _crctab[((crc >> (bits-8)) ^ *data++) & 0x00FF];
//       return crc & _bitmask;                   // ONLY the result is masked
//   }
//
// Two properties are easy to get wrong and BOTH break real-world validation:
//  * the feedback bit is bit (bits-1) — for CRC-14 that is 0x2000, not 0x8000;
//  * the register is NOT masked to `bits` between bytes, and `init` is used
//    as-is (OtaCrcInitializer = (UID[4]<<8|UID[5]) ^ OTA_VERSION_ID is a full
//    16-bit value with high bits set for many real UIDs). The textbook
//    CRC-14 diverges from ELRS exactly in those cases — do not "fix" this.
//
//   OTA4: CRC-14, poly 0x2E57, over bytes 0..6, init = OtaCrcInitializer
//   OTA8: CRC-16, poly 0x3D65, over bytes 0..11, same init
// (OTA.cpp; OtaUpdateCrcInitFromUid; bind mode uses init 0.)
// =============================================================================
#pragma once
#include <stdint.h>
#include <stddef.h>
#include "elrs_defs.h"

// one table entry: 8 shift/XOR steps from idx << (bits-8), unmasked — as in
// Crc2Byte::init (table-free equivalent).
static inline uint16_t elrs_crc_tab(uint8_t idx, uint16_t poly, uint8_t bits)
{
    uint16_t crc = (uint16_t)((uint16_t)idx << (bits - 8));
    const uint16_t highbit = (uint16_t)(1u << (bits - 1));
    for (uint8_t j = 0; j < 8; j++) {
        crc = (uint16_t)((crc << 1) ^ ((crc & highbit) ? poly : 0));
    }
    return crc;
}

// Crc2Byte::calc — index ((crc >> (bits-8)) ^ byte) & 0xFF, register free to
// grow unmasked between bytes, result masked to `bits` at the end.
static inline uint16_t elrs_crc_calc(const uint8_t *data, size_t len, uint16_t crc,
                                     uint16_t poly, uint8_t bits)
{
    while (len--) {
        uint8_t idx = (uint8_t)(((crc >> (bits - 8)) ^ (uint16_t)*data++) & 0x00FF);
        crc = (uint16_t)((crc << 8) ^ elrs_crc_tab(idx, poly, bits));
    }
    return (uint16_t)(crc & (uint16_t)((1u << bits) - 1));
}

static inline uint16_t elrs_crc14(const uint8_t *data, size_t len, uint16_t init)
{
    return elrs_crc_calc(data, len, init, ELRS_CRC14_POLY, 14);
}

static inline uint16_t elrs_crc16(const uint8_t *data, size_t len, uint16_t init)
{
    return elrs_crc_calc(data, len, init, ELRS_CRC16_POLY, 16);
}
