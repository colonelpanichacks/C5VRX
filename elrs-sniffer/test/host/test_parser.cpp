// test_parser.cpp — host-side sanity for the ELRS parser + CRC (no hardware).
// Build:  c++ -std=c++11 -o test_parser test_parser.cpp ../src/elrs_parse.cpp -I../src
// Run:    ./test_parser
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cassert>
#include "elrs_defs.h"
#include "elrs_crc.h"
#include "elrs_parse.h"

// ---- independent CRC formulation (bit-at-a-time, structured differently
// from the loop in elrs_crc.h as a cross-check) ----
static uint16_t crc_ref2(const uint8_t *d, size_t len, uint16_t init, uint16_t poly, uint16_t mask)
{
    uint16_t crc = init & mask;
    for (size_t i = 0; i < len; i++) {
        crc ^= (uint16_t)d[i] << 8;
        for (int b = 0; b < 8; b++) crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ poly) : (uint16_t)(crc << 1);
    }
    return (uint16_t)(crc & mask);
}

// ---- ELRS packer, ported from OTA.cpp PackUInt11ToChannels4x10 ----
static void pack_4x10(const uint16_t src[4], uint8_t dest[5])
{
    unsigned destShift = 0;
    uint8_t *d = dest;
    *d = 0;
    for (unsigned ch = 0; ch < 4; ++ch) {
        unsigned chVal = src[ch] & 0x3FF;
        *d++ |= chVal << destShift;
        unsigned srcBitsLeft = 10 - 8 + destShift;
        *d = chVal >> (10 - srcBitsLeft);
        destShift = srcBitsLeft;
    }
}

int main()
{
    srand(12345);

    // 1) CRC cross-check between the two formulations + masking + flip property
    for (int iter = 0; iter < 2000; iter++) {
        uint8_t buf[16];
        size_t len = 5 + rand() % 10;
        for (size_t i = 0; i < len; i++) buf[i] = rand() & 0xFF;
        uint16_t init = rand() & 0xFFFF;
        uint16_t a = elrs_crc_calc(buf, len, init, ELRS_CRC16_POLY, ELRS_CRC16_MASK);
        uint16_t b = crc_ref2(buf, len, init, ELRS_CRC16_POLY, ELRS_CRC16_MASK);
        assert(a == b);
        uint16_t c14 = elrs_crc14(buf, len, init);
        uint16_t d14 = crc_ref2(buf, len, init, ELRS_CRC14_POLY, ELRS_CRC14_MASK);
        assert(c14 == d14);
        assert((c14 & ~ELRS_CRC14_MASK) == 0);
        if (len > 0) {
            buf[rand() % len] ^= 1 << (rand() % 8);
            assert(elrs_crc_calc(buf, len, init, ELRS_CRC16_POLY, ELRS_CRC16_MASK) != a);
        }
    }
    printf("ok: crc14/16 cross-formulation + masking + flip\n");

    // 2) bitpack round trip incl. ELRS's documented bit order
    {
        uint16_t ch[4] = { 0x201, 0x155, 0x3FF, 0x000 };
        uint8_t raw[5] = { 0 };
        pack_4x10(ch, raw);
        uint16_t out[4];
        elrs_unpack_4x10(raw, out);
        for (int i = 0; i < 4; i++) assert(out[i] == ch[i]);
        // documented example: value bits A987654321 -> first byte 0x21 (low 8)
        uint16_t one[4] = { 0x321, 0, 0, 0 };
        uint8_t r2[5] = { 0 };
        pack_4x10(one, r2);
        assert(r2[0] == 0x21);
        assert((r2[1] & 0x03) == 0x03); // top bits 'A9' land in low bits of next byte
    }
    printf("ok: 4x10 bitpack round trip + ELRS bit order\n");

    // 3) OTA8 sync self-acquisition: CRC init derived from the packet itself
    {
        elrs_decode_ctx_t ctx;
        elrs_decode_init(&ctx);
        uint8_t pkt[ELRS_OTA8_LEN] = { 0 };
        pkt[0] = ELRS_PKT_SYNC;                 // type bits only
        pkt[1] = 0x2A;                          // fhss index
        pkt[2] = 0x77;                          // nonce
        pkt[3] = (6 << 4) | (2 << 1) | 0;       // rate 6 (250Hz), tlm 1:32, wide (enc bit 0)
        pkt[4] = 0xA5;                          // UID3
        pkt[5] = 0xB3;                          // UID4
        pkt[6] = 0xC2;                          // UID5
        uint16_t init = elrs_crc_init_from_uid(0xB3, 0xC2);
        uint16_t crc = elrs_crc16(pkt, 11, init);
        pkt[11] = crc & 0xFF;
        pkt[12] = crc >> 8;
        elrs_packet_t out;
        bool ok = elrs_decode_packet(&ctx, pkt, ELRS_OTA8_LEN, &out);
        assert(ok && out.type == ELRS_PKT_SYNC);
        assert(out.cls == ELRS_PKT_CLASS_CRC_OK);
        assert(ctx.uid_known && ctx.uid3 == 0xA5 && ctx.uid4 == 0xB3 && ctx.uid5 == 0xC2);
        assert(ctx.crc_init_known && ctx.crc_init == init);
        assert(ctx.switch_mode == ELRS_SW_WIDE); // sync bit 0 -> wide
        assert(out.sync.rate_index == 6 && out.sync.nonce == 0x77);
    }
    printf("ok: OTA8 sync self-acquisition (uid + crc init from packet)\n");

    // 4) OTA8 RC packet validates once the link is captured; ch = v10*2
    {
        elrs_decode_ctx_t ctx;
        elrs_decode_init(&ctx);
        // seed ctx from a crafted sync (as in test 3)
        uint8_t sync[ELRS_OTA8_LEN] = { ELRS_PKT_SYNC, 0, 1, (6 << 4), 0x11, 0x22, 0x33 };
        uint16_t init = elrs_crc_init_from_uid(0x22, 0x33);
        uint16_t sc = elrs_crc16(sync, 11, init);
        sync[11] = sc & 0xFF; sync[12] = sc >> 8;
        elrs_packet_t tmp;
        elrs_decode_packet(&ctx, sync, ELRS_OTA8_LEN, &tmp);

        uint8_t rc[ELRS_OTA8_LEN] = { 0 };
        uint16_t v[4] = { 500, 511, 200, 800 };  // stick 10-bit values
        uint8_t packed[5];
        pack_4x10(v, packed);
        rc[0] = ELRS_PKT_RCDATA | (1 << 2) | (3 << 3); // tlmStatus=1, power=3(+1)
        memcpy(&rc[1], packed, 5);
        pack_4x10(v, &rc[6]);
        uint16_t crc = elrs_crc16(rc, 11, init);
        rc[11] = crc & 0xFF; rc[12] = crc >> 8;

        elrs_packet_t out;
        bool ok = elrs_decode_packet(&ctx, rc, ELRS_OTA8_LEN, &out);
        assert(ok && out.type == ELRS_PKT_RCDATA);
        assert(out.cls == ELRS_PKT_CLASS_CRC_OK);
        for (int i = 0; i < 4; i++) {
            assert(out.rc.has_ch[i]);
            assert(out.rc.ch[i] == v[i] * 2); // full-range packing
        }
        assert(out.rc.uplink_power == 4);
    }
    printf("ok: OTA8 RC decode + CRC validation after capture\n");

    // 5) OTA4 sync (crc14 path) + bind-mode init 0
    {
        elrs_decode_ctx_t ctx;
        elrs_decode_init(&ctx);
        uint8_t s4[ELRS_OTA4_LEN] = { ELRS_PKT_SYNC, 9, 3, (4 << 4), 0xDE, 0xAD, 0xBE };
        uint16_t init = elrs_crc_init_from_uid(0xAD, 0xBE);
        uint16_t crc = elrs_crc14(s4, 7, init);
        s4[0] |= (crc >> 8) << 2;
        s4[7] = crc & 0xFF;
        elrs_packet_t out;
        bool ok = elrs_decode_packet(&ctx, s4, ELRS_OTA4_LEN, &out);
        assert(ok && out.cls == ELRS_PKT_CLASS_CRC_OK && out.type == ELRS_PKT_SYNC);
        assert(ctx.uid_known && ctx.uid5 == 0xBE);

        // bind mode: init 0
        elrs_decode_ctx_t ctx2;
        elrs_decode_init(&ctx2);
        uint8_t b4[ELRS_OTA4_LEN] = { ELRS_PKT_SYNC, 0, 0, (9 << 4), 0, 0, 0 };
        uint16_t bc = elrs_crc14(b4, 7, 0);
        b4[0] |= (bc >> 8) << 2;
        b4[7] = bc & 0xFF;
        elrs_packet_t out2;
        bool ok2 = elrs_decode_packet(&ctx2, b4, ELRS_OTA4_LEN, &out2);
        assert(ok2 && out2.cls == ELRS_PKT_CLASS_CRC_OK);
    }
    printf("ok: OTA4 sync crc14 + derived-init + bind-mode init 0\n");

    // 6) noise prefilter: rail-pinned junk must be dropped; random junk is
    //    merely "not promoted" (documented weakness — see gate comment)
    {
        elrs_decode_ctx_t ctx; // crc_init never known -> heuristic path
        elrs_decode_init(&ctx);
        uint8_t ff[ELRS_OTA4_LEN], zz[ELRS_OTA8_LEN];
        memset(ff, 0xFF, sizeof(ff)); ff[0] = (ff[0] & 0xFC) | ELRS_PKT_RCDATA;
        memset(zz, 0x00, sizeof(zz)); zz[0] = ELRS_PKT_RCDATA;
        elrs_packet_t out;
        assert(!elrs_decode_packet(&ctx, ff, ELRS_OTA4_LEN, &out));
        assert(!elrs_decode_packet(&ctx, zz, ELRS_OTA8_LEN, &out));
        // sync packets with an out-of-range rate index are rejected too
        uint8_t badsync[ELRS_OTA4_LEN] = { ELRS_PKT_SYNC, 0, 0, (15 << 4), 1, 2, 3 };
        assert(!elrs_decode_packet(&ctx, badsync, ELRS_OTA4_LEN, &out));
        unsigned rejected = 0, N = 4000;
        for (unsigned i = 0; i < N; i++) {
            uint8_t junk[ELRS_OTA8_LEN];
            for (int j = 0; j < ELRS_OTA8_LEN; j++) junk[j] = rand() & 0xFF;
            junk[0] = (junk[0] & 0xFC) | ELRS_PKT_RCDATA;
            if (!elrs_decode_packet(&ctx, junk, ELRS_OTA8_LEN, &out)) rejected++;
        }
        printf("random-junk hard rejects: %u/%u (weak gate by design)\n", rejected, N);
    }
    printf("ok: noise prefilter + sync structure gate\n");

    printf("ALL HOST TESTS PASSED\n");
    return 0;
}
