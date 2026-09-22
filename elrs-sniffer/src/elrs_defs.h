// =============================================================================
// elrs_defs.h — ExpressLRS 2.4 GHz (SX1280) air-interface constants
//
// Everything in this file was extracted from the ExpressLRS source of record
// (tag 3.6.4, branch 3.x.x-maintenance — the currently deployed 3.x firmware;
// cross-checked against master for 4.x drift). Each block cites its source.
// Where the deployed fleet diverges (legacy 2.x, in-development 4.x) the
// difference is called out explicitly.
//
// Key source files (github.com/ExpressLRS/ExpressLRS @ 3.6.4):
//   src/src/common.cpp                       air-rate tables (SX128X section)
//   src/lib/SX1280Driver/SX1280.cpp          radio config, CRC-off, "LoRa has
//                                            no sync word", FLRC sync/CRC seed
//   src/lib/SX1280Driver/SX1280_Regs.h       register enum values
//   src/lib/OTA/OTA.h / OTA.cpp              OTA4/OTA8 layouts, CRC polys,
//                                            CRC init from UID, channel packing
//   src/lib/TelemetryProtocol/telemetry_protocol.h  tlm type constants
//   src/lib/FHSS/FHSS.cpp                    2.4 GHz frequency plan
//   src/src/rx_main.cpp                      SetRFLinkRate(): invertIQ rule,
//                                            FLRC sync word = uidMacSeedGet(),
//                                            CRC seed = OtaCrcInitializer
//   src/test/test_crc/test_crc.cpp           CRC semantics (MSB-first, no
//                                            reflection, init as given)
// =============================================================================
#pragma once
#include <stdint.h>

// ---------------------------------------------------------------------------
// OTA protocol version marker.
// OTA_VERSION_ID is XORed into OtaCrcInitializer and the FLRC MAC seed "to
// reduce compatibility with previous versions" (src/include/common.h).
// 3.x = 3. 4.x increments this — one more thing that breaks naive decoding.
// ---------------------------------------------------------------------------
#define ELRS_OTA_VERSION_ID_3X   3u

// ---------------------------------------------------------------------------
// UID
//   UID_LEN = 6. A "bind phrase" is hashed to these 6 bytes; unbound units
//   have UID == all zero (common.h UID_IS_BOUND macro).
//   IMPORTANT for a sniffer: NOTHING per-packet identifies the TX. The only
//   UID material on air is UID[3..5] inside periodic SYNC packets, plus a
//   16-bit CRC initializer that must equal ((UID[4]<<8)|UID[5]) ^ OTA_VERSION_ID
//   for packets to validate. Treat a captured (rate, CRC-init, UID[3..5])
//   triple as a *link fingerprint*, not a unique identity. Sync packets also
//   carry the FHSS hop table seed (see FHSS below).
// Source: OTA.h OTA_Sync_s; OTA.cpp OtaUpdateCrcInitFromUid().
// ---------------------------------------------------------------------------
#define ELRS_UID_LEN 6u

// ---------------------------------------------------------------------------
// Air-rate table, SX128X radio, ELRS 3.x (src/src/common.cpp,
// ExpressLRS_AirRateConfig[RATE_MAX], RATE_MAX = 10).
// Interval is the TX packet period in microseconds; payload is the fixed
// over-the-air size (see packet formats below). HopInterval = packets per
// FHSS hop. TLMinterval is the default telemetry ratio (enum: see
// expresslrs_tlm_ratio_e — value N means 1:N).
//
// SX1280 LoRa bandwidth enum values (SX1280_Regs.h): the register value is
// NOT the kHz figure — 812.5 kHz == 0x18. Coding rates: CR_LI_4_6 == 0x06,
// CR_LI_4_7 == 0x07, CR_LI_4_8 == 0x08 (long-interleaving variants, ELRS uses
// ONLY LI coding rates on 2.4 GHz LoRa in 3.x). Spreading factors SF5..SF8 ==
// 0x50..0x80 in steps of 0x10.
// ---------------------------------------------------------------------------
typedef struct {
    const char *name;      // human label
    uint8_t     bw;        // SX1280 LoRa BW register enum (0x18 = 812.5 kHz)
    uint8_t     sf;        // SX1280 SF register enum (0x50 = SF5 .. 0x80 = SF8)
    uint8_t     cr;        // SX1280 CR register enum (LI variants)
    uint8_t     preamble;  // symbols
    uint32_t    interval_us;
    uint8_t     payload;   // fixed OTA payload bytes: 8 (OTA4) or 13 (OTA8)
    uint8_t     hop_interval;
    uint8_t     rate_index;// ELRS index (sync packet rateIndex field)
} elrs_rate_t;

static const elrs_rate_t ELRS_RATES_3X[] = {
    // name           bw     sf     cr     pre  interval  len  hop  idx
    { "LoRa 500Hz",   0x18,  0x50,  0x06,  12,   2000,     8,   4,   4 },
    { "LoRa 333Hz8",  0x18,  0x50,  0x08,  12,   3003,    13,   4,   5 },
    { "LoRa 250Hz",   0x18,  0x60,  0x08,  14,   4000,     8,   4,   6 },
    { "LoRa 150Hz",   0x18,  0x70,  0x08,  12,   6666,     8,   4,   7 },
    { "LoRa 100Hz8",  0x18,  0x70,  0x08,  12,  10000,    13,   4,   8 },
    { "LoRa 50Hz",    0x18,  0x80,  0x08,  12,  20000,     8,   2,   9 },
};
#define ELRS_RATES_3X_COUNT (sizeof(ELRS_RATES_3X) / sizeof(ELRS_RATES_3X[0]))

// Legacy ELRS 2.x LoRa variants still seen in the field (2.5.2 common.cpp).
// Same 812.5 kHz BW everywhere; differences are SF/CR for 250/150/50 Hz.
// 2.x on-air packets were the classic 8-byte nonce-first format (see
// ELRS_PKT_LEGACY_V2) — payload length 8 covers them, the parser marks the
// CRC region unvalidated for these.
static const elrs_rate_t ELRS_RATES_2X[] = {
    { "LoRa 500Hz",   0x18,  0x50,  0x06,  12,   2000,     8,   4,  0xFF },
    { "LoRa 250Hz*",  0x18,  0x60,  0x07,  14,   4000,     8,   4,  0xFF }, // CR 4/7
    { "LoRa 150Hz*",  0x18,  0x70,  0x07,  12,   6666,     8,   4,  0xFF }, // CR 4/7
    { "LoRa 50Hz*",   0x18,  0x90,  0x06,  12,  20000,     8,   2,  0xFF }, // SF9
};
#define ELRS_RATES_2X_COUNT (sizeof(ELRS_RATES_2X) / sizeof(ELRS_RATES_2X[0]))

// FLRC rates (SX128X table rows 0..3) are NOT swept by default: the FLRC
// 32-bit sync word is UID-derived (uidMacSeedGet, below), unknown to a
// passive observer. After a LoRa SYNC packet reveals UID[3..5], only UID[2]
// is missing — 256 candidate sync words — so FLRC hunting is feasible TODO.

// ---------------------------------------------------------------------------
// FHSS (2.4 GHz): 80 channels, 1 MHz spacing, 2400.4 .. 2479.4 MHz.
// Sync channel index = freq_count/2 + 1 = 41 -> 2441.4 MHz. The TX returns to
// the sync channel for periodic SYNC packets, which is why a passive sniffer
// parked there harvests sync packets even while the RC stream hops.
// Hop sequence is seeded with uidMacSeedGet() (FHSSrandomiseFHSSsequence).
// Source: src/lib/FHSS/FHSS.cpp domains[] RADIO_SX128X section.
// ---------------------------------------------------------------------------
#define ELRS_2G4_FREQ_START_HZ   2400400000u
#define ELRS_2G4_FREQ_COUNT      80u
#define ELRS_2G4_FREQ_SPACING_HZ 1000000u
#define ELRS_2G4_SYNC_INDEX      41u
#define ELRS_2G4_SYNC_FREQ_HZ    (ELRS_2G4_FREQ_START_HZ + ELRS_2G4_SYNC_INDEX * ELRS_2G4_FREQ_SPACING_HZ)

// ---------------------------------------------------------------------------
// InvertIQ rule (src/src/rx_main.cpp SetRFLinkRate):
//     invertIQ = bindMode || (UID[5] & 0x01)
// So ~50% of bound links run INVERTED IQ. A sniffer MUST try both. Binding
// itself always uses inverted IQ, CRC init 0, 50 Hz (RATE_BINDING).
// ---------------------------------------------------------------------------

// ---------------------------------------------------------------------------
// Sync word situation, SX1280 — often misunderstood, so stated fully:
//  * LoRa on the SX1280 has NO sync word at all ("The SYNCWORD_VALID bit isn't
//    set on LoRa, it has no synch (sic) word" — SX1280.cpp RXnbISR). Packet
//    acceptance = preamble + implicit fixed-length header, and the RADIO CRC
//    is switched OFF (SetPacketParamsLoRa writes SX1280_LORA_CRC_OFF).
//    => A naive sniffer demods plenty of garbage; the ELRS software CRC
//    (below) is the ONLY integrity check on LoRa links.
//  * FLRC uses a 32-bit sync word = uidMacSeedGet() and a dedicated CRC seed:
//        flrcSyncWord = uidMacSeedGet()
//                   = (UID[2]<<24) | (UID[3]<<16) | (UID[4]<<8) | (UID[5] ^ OTA_VERSION_ID)
//        flrcCrcSeed  = OtaCrcInitializer = ((UID[4]<<8) | UID[5]) ^ OTA_VERSION_ID
//    written into the FLRC CRC seed register; radio CRC = 3 bytes.
//    Hardware erratum (DS_SX1280-1_V3.2 16.4): if the first two sync bytes are
//    0x8C 0x38 or 0x63 0x0E they must be swapped (driver does this).
// Source: SX1280.cpp SetPacketParamsFLRC / Config; rx_main.cpp SetRFLinkRate;
// common.cpp uidMacSeedGet(); OTA.cpp OtaUpdateCrcInitFromUid().
// ---------------------------------------------------------------------------

// ---------------------------------------------------------------------------
// OTA packet formats, ELRS 3.4+/3.5/3.6 (OTA.h). "OTA4" = 8 bytes, "OTA8" =
// 13 bytes — the names are ELRS's, not a channel count. 4.x (master) keeps the
// same sizes and CRC polys but reshuffles some bit fields (called out below).
//
// COMMON: the low 2 bits of byte 0 are the packet type for BOTH sizes:
#define ELRS_PKT_RCDATA 0x0   // 0b00
#define ELRS_PKT_MSP    0x1   // 0b01  (uplink MSP data, "model config")
#define ELRS_PKT_SYNC   0x2   // 0b10
#define ELRS_PKT_TLM    0x3   // 0b11  (downlink telemetry)
//
// ---- OTA4, 8 bytes (all 2.4G LoRa rates except the two 8ch ones) ----
// byte0   type:2 | crcHigh:6        (crcHigh byte abused, see CRC notes)
// byte1..5  ch: 4x 10-bit channels, packed LSB-first across 5 bytes
//           ("PackUInt11ToChannels4x10": bits A987654321 -> 87654321,000000A9)
// byte6   switches:7 | ch4:1         (AUX1 arm bit in LSB; encoding depends on
//                                     switch mode, below)
// byte7   CRC low byte (CRC14: this byte + the 6 bits in byte0)
//
// ---- OTA8, 13 bytes (LoRa 333Hz8 / 100Hz8 "full-res" rates) ----
// byte0   packetType:2 | telemetryStatus:1 | uplinkPower:3 | isHighAux:1 | ch4:1
// byte1..5   chLow:  4x 10-bit (CH0..3 or CH8..11 in 16ch mode)
// byte6..10  chHigh: 4x 10-bit (AUX2..5, AUX6..9, or CH12..15 by mode)
// byte11..12 CRC16 little-endian
// 4.x/master drift: byte0 is {packetType:2, stubbornAck:1, uplinkPower:3,
// isHighAux:1, isArmed:1} — telemetryStatus/stubbornAck and ch4/isArmed moved.
// Stick values for 3.x here; TODO 4.x header variant.
//
// ---- SYNC packet (both sizes; type = 0b10) ----
// byte0   0b10 in low bits (rest 0 / FHSS-slot bits)
// byte1   fhssIndex        (current hop index)
// byte2   nonce
// byte3   switchEncMode:1 | newTlmRatio:3 | rateIndex:4
// byte4   UID[3]
// byte5   UID[4]
// byte6   UID[5]
// (OTA8: + 4 free bytes, then CRC)
// 4.x/master drift: byte3 becomes rateIndex-independent {switchEncMode:1,
// newTlmRatio:3, geminiMode:1, otaProtocol:2, free:1} plus a separate
// rfRateEnum byte, and sync grows by one byte. Parsed as TODO.
//
// ---- TLM packet (downlink) ----
// OTA4: byte0 type=0b11; byte1 = tlmType:2 | packageIndex:6; payload[5];
//       tlmType: ELRS_TELEMETRY_TYPE_LINK (1) or _DATA (2).
// OTA8: byte0 = 0b11 | containsLinkStats:1 | packageIndex:5; then either
//       OTA_LinkStats_s (4 bytes) + payload[6], or payload[10].
// Source: telemetry_protocol.h (type constants), OTA.h (layouts).
//
// ---- Link statistics struct (inside TLM packets, 4 bytes) ----
// uplink_RSSI_1:7 | antenna:1 ; uplink_RSSI_2:7 | modelMatch:1 ;
// lq:7 | mspConfirm:1 ; SNR int8. RSSI fields are |dBm| (positive = -dBm).
// ---------------------------------------------------------------------------
#define ELRS_OTA4_LEN 8u
#define ELRS_OTA8_LEN 13u

typedef struct {
    uint8_t fhss_index;
    uint8_t nonce;
    uint8_t switch_mode;   // switchEncMode bit
    uint8_t tlm_ratio;     // newTlmRatio enum
    uint8_t rate_index;    // index into ELRS SX128X rate table
    uint8_t uid3, uid4, uid5;
} elrs_sync_info_t;

// ---------------------------------------------------------------------------
// Switch encodings for the OTA4 switches byte (OTA.cpp). Switch mode comes
// from the sync packet (switchEncMode bit) — Hybrid(0)=hybrid8 round-robin,
// Wide(1)=wide. Without a sync packet we try both and let the CRC arbitrate.
//  Hybrid8: switches = telem<<6 | switchIndex<<3 | value (index 6 = AUX8 hi-res)
//  Wide:    switches = 7-bit round-robin value (index from nonce), or
//           telem<<6 | uplinkPower when index==7
// Sniffer value: AUX channels only update when their slot comes around; the
// four sticks (ch0..3) + AUX1 are in every packet.
// ---------------------------------------------------------------------------
typedef enum { ELRS_SW_HYBRID8 = 0, ELRS_SW_WIDE = 1, ELRS_SW_12CH } elrs_switch_mode_t;

// ---------------------------------------------------------------------------
// ELRS software CRC — the only integrity check on LoRa links.
//   OTA4: CRC-14, poly 0x2E57 (Koopman 0x372B), over bytes 0..6
//   OTA8: CRC-16, poly 0x3D65 (Koopman 0x9EB2), over bytes 0..11
// MSB-first, no reflection, no final XOR; init = OtaCrcInitializer.
// OtaCrcInitializer = ((UID[4]<<8) | UID[5]) ^ OTA_VERSION_ID  (bound links)
//                   = 0                                            (bind mode)
// Gotcha (OTA.cpp ValidatePacketCrcStd): for RCDATA in Wide mode, byte0's
// crcHigh field is pre-loaded with (nonce % FHSShopInterval)+1 before the CRC
// is computed — validation needs the nonce (tracked from sync packets).
// For Hybrid mode and non-RCDATA, crcHigh is zeroed before the CRC.
// Source: OTA.h/OTA.cpp, test_crc.cpp.
// ---------------------------------------------------------------------------
#define ELRS_CRC14_POLY 0x2E57u
#define ELRS_CRC16_POLY 0x3D65u
#define ELRS_CRC14_MASK 0x3FFFu
#define ELRS_CRC16_MASK 0xFFFFu

// ---------------------------------------------------------------------------
// Channel value scaling (OTA.cpp + crsf_protocol.h).
// OTA4 10-bit values map the CRSF 11-bit range 988..2012us only:
//     CRSF_val = 172 + round(v10 * (1811-172)/1023)   (then us = f(CRSF_val))
// OTA8 10-bit values use divide-by-2 packing of the full 11-bit CRSF value:
//     CRSF_val = v10 * 2  (full 172..1811 "extended" range)
// CRSF value -> pulse width:  us = 988 + (CRSF_val - 172) * 1024 / 1639
// (CRSF units: 0.5us per LSB centered so MID 992 == 1500us).
// ---------------------------------------------------------------------------
#define CRSF_VAL_MIN 172u
#define CRSF_VAL_MAX 1811u
#define CRSF_VAL_MID 992u

// ---------------------------------------------------------------------------
// Telemetry payload types. Inside a TLM DATA packet the (reassembled)
// payload is a CRSF telemetry frame: byte0 = CRSF frame type, rest = payload.
// Fragment reassembly: packageIndex increments per fragment; frame length is
// implied by the CRSF type. Common CRSF frame types (CRSF spec / ELRS
// Telemetry.cpp):
//   0x02 GPS          15B: lat i32 LE (1e7 deg), lon i32, spd u16 (km/h*10),
//                     hdg u16 (deg*100), alt u16 ((m+1000)*10... see note), sat u8
//   0x07 VARIO         2B: vspeed i16 cm/s
//   0x08 BATTERY       8B: V u16 0.1V, A u16 0.1A, mAh u24, fuel u8 %
//   0x09 BARO_ALT      6B: alt i16 (0.1m, +1000m offset), vspeed i16 cm/s
//   0x14 LINK_STATISTICS 10B (CRSF wire format; OTA usually uses the 4-byte
//                     OTA_LinkStats_s above instead)
//   0x1E ATTITUDE      6B: pitch/roll/yaw i16, rad * 10000
//   0x21 FLIGHT_MODE   nB: zero-terminated string
// GPS alt encoding varies by source (0.1m with 1000m offset per CRSF spec;
// ELRS historically packs (m+1000) at 0.1m) — parser marks it TODO.
// ---------------------------------------------------------------------------
#define CRSF_FRAMETYPE_GPS 0x02
#define CRSF_FRAMETYPE_VARIO 0x07
#define CRSF_FRAMETYPE_BATTERY 0x08
#define CRSF_FRAMETYPE_BARO_ALT 0x09
#define CRSF_FRAMETYPE_LINK_STATISTICS 0x14
#define CRSF_FRAMETYPE_ATTITUDE 0x1E
#define CRSF_FRAMETYPE_FLIGHT_MODE 0x21
#define CRSF_FRAMETYPE_HEARTBEAT 0x0B

// ---------------------------------------------------------------------------
// Legacy 2.x 8-byte packet (ELRS 2.0–2.5, "classic"):
// byte0 nonce | byte1 {tlmeConfirmID:2, tlmLowFBits:2, switchEncMode:4} |
// bytes2..5 channel data | CRC16 (poly 0x3D65? 2.x used ELRS_CRC_POLY 0x07
// for 8-byte? — 2.x OTA8_CRC uses crc16 poly 0x8005? UNVERIFIED) | ...
// Marked ELRS_PKT_LEGACY_V2: parser reports "legacy/2.x" without CRC claims.
// TODO: pin down 2.x CRC init from 2.x-maintenance OTA.cpp before trusting.
// ---------------------------------------------------------------------------
typedef enum {
    ELRS_PKT_CLASS_CRC_OK = 0,   // validated by ELRS software CRC
    ELRS_PKT_CLASS_PLAUSIBLE,    // structure/entropy plausible, CRC not checkable
    ELRS_PKT_CLASS_NOISE         // demodulated junk (LoRa has no air CRC)
} elrs_pkt_class_t;

// Identity honesty note (also in README): ELRS carries no per-packet TX ID.
// What a sniffer can derive: UID[3..5] + CRC init + rate + hop timing from
// SYNC packets. That fingerprints a *link session* (and, since the bind
// phrase is the only UID entropy, two models bound to the same phrase share
// it), never a unique aircraft by itself.
