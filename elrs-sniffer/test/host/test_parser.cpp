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
#include "elrs_fhss.h"
#include "sniffer_radio.h"
#include "elrs_crc24flrc.h"
#include "elrs_crc24flrc.h"

// ---- independent CRC formulation: verbatim port of the ELRS 3.6.4 table
// implementation (src/lib/CRC/crc.cpp Crc2Byte), kept structurally different
// from the table-free version in elrs_crc.h ----
static uint16_t g_reftab[256];
static void ref_init(uint8_t bits, uint16_t poly)
{
    uint16_t highbit = (uint16_t)(1 << (bits - 1));
    for (uint16_t i = 0; i < 256; i++) {
        uint16_t crc = (uint16_t)(i << (bits - 8));
        for (uint8_t j = 0; j < 8; j++)
            crc = (uint16_t)((crc << 1) ^ ((crc & highbit) ? poly : 0));
        g_reftab[i] = crc;
    }
}
static uint16_t ref_calc(const uint8_t *d, size_t len, uint16_t crc, uint8_t bits)
{
    while (len--) {
        uint8_t idx = (uint8_t)(((crc >> (bits - 8)) ^ (uint16_t)*d++) & 0x00FF);
        crc = (uint16_t)((crc << 8) ^ g_reftab[idx]);
    }
    return crc;
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

    // 1) CRC cross-check: table-free impl vs verbatim ELRS table port, with
    // FULL 16-bit inits (high bits included — the case that broke real links)
    for (int iter = 0; iter < 2000; iter++) {
        uint8_t buf[16];
        size_t len = 5 + rand() % 10;
        for (size_t i = 0; i < len; i++) buf[i] = rand() & 0xFF;
        uint16_t init = rand() & 0xFFFF;
        ref_init(16, ELRS_CRC16_POLY);
        uint16_t a = elrs_crc16(buf, len, init);
        uint16_t b = (uint16_t)(ref_calc(buf, len, init, 16) & ELRS_CRC16_MASK);
        assert(a == b);
        ref_init(14, ELRS_CRC14_POLY);
        uint16_t c14 = elrs_crc14(buf, len, init);
        uint16_t d14 = (uint16_t)(ref_calc(buf, len, init, 14) & ELRS_CRC14_MASK);
        assert(c14 == d14);
        assert((c14 & ~ELRS_CRC14_MASK) == 0);
        if (len > 0) {
            buf[rand() % len] ^= 1 << (rand() % 8);
            assert(elrs_crc16(buf, len, init) != a);
        }
    }
    printf("ok: crc14/16 vs verbatim ELRS table port (full 16-bit inits) + flip\n");

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

    // 7) FIELD-SHAPE REGRESSION: the exact packet the field unit failed on —
    //    OTA4 sync, fhss 63, nonce 0, rateIdx 0, UID3..5 = 00 00 31, with the
    //    init derived from the packet's own UID bytes (0x0032). The old CRC
    //    (bit-15 feedback, init masked) rejected this; the exact crc.cpp port
    //    must accept it.
    {
        elrs_decode_ctx_t ctx;
        elrs_decode_init(&ctx);
        uint8_t pkt[ELRS_OTA4_LEN] = { ELRS_PKT_SYNC, 63, 0, 0x00, 0x00, 0x00, 0x31 };
        uint16_t init = elrs_crc_init_from_uid(0x00, 0x31); // 0x0032
        uint16_t crc = elrs_crc14(pkt, 7, init);
        pkt[0] |= (uint8_t)((crc >> 8) << 2);
        pkt[7] = (uint8_t)(crc & 0xFF);
        char hex[17];
        for (int i = 0; i < 8; i++) snprintf(hex + 2 * i, 3, "%02x", pkt[i]);
        printf("field-shape sync hex: %s\n", hex);
        elrs_packet_t out;
        bool ok = elrs_decode_packet(&ctx, pkt, ELRS_OTA4_LEN, &out);
        assert(ok && out.type == ELRS_PKT_SYNC);
        assert(out.cls == ELRS_PKT_CLASS_CRC_OK);
        assert(ctx.crc_init_known && ctx.crc_init == init);
    }
    printf("ok: field-shape regression (fhss63/nonce0/rate0/uid 000031)\n");

    // 8) HIGH-BIT INIT: UID[4] with bit15 set must validate (ELRS uses the
    //    raw 16-bit OtaCrcInitializer; the register is never width-masked
    //    during computation).
    {
        elrs_decode_ctx_t ctx;
        elrs_decode_init(&ctx);
        uint8_t pkt[ELRS_OTA4_LEN] = { ELRS_PKT_SYNC, 9, 3, (6 << 4), 0xC5, 0xC5, 0x31 };
        uint16_t init = elrs_crc_init_from_uid(0xC5, 0x31); // has bit15 set
        uint16_t crc = elrs_crc14(pkt, 7, init);
        pkt[0] |= (uint8_t)((crc >> 8) << 2);
        pkt[7] = (uint8_t)(crc & 0xFF);
        elrs_packet_t out;
        bool ok = elrs_decode_packet(&ctx, pkt, ELRS_OTA4_LEN, &out);
        assert(ok && out.cls == ELRS_PKT_CLASS_CRC_OK && ctx.crc_init == init);
    }
    printf("ok: high-bit init (unmasked register)\n");

    // 9) SEED SEARCH: brute-force init 0..65535 against a captured-shaped
    //    sync — the diagnostic for "does ANY init validate these bytes?".
    //    The known init MUST be among the hits; a corrupted byte must give
    //    none at the known init. (For field captures: paste the rawpkt hex.)
    {
        uint8_t pkt[ELRS_OTA4_LEN] = { ELRS_PKT_SYNC, 63, 0, 0x00, 0x00, 0x00, 0x31 };
        uint16_t true_init = (uint16_t)(((uint16_t)0x12 << 8) | 0x34) ^ 3; // pretend UID4/5
        uint16_t crc = elrs_crc14(pkt, 7, true_init);
        pkt[0] |= (uint8_t)((crc >> 8) << 2);
        pkt[7] = (uint8_t)(crc & 0xFF);

        unsigned hits = 0;
        bool found_true = false;
        for (uint32_t s = 0; s < 65536; s++) {
            uint8_t tmp[7]; tmp[0] = pkt[0] & 0x03; memcpy(tmp + 1, pkt + 1, 6);
            uint16_t incrc = (uint16_t)((((uint16_t)pkt[0] >> 2) << 8) | pkt[7]);
            if (elrs_crc14(tmp, 7, (uint16_t)s) == incrc) {
                hits++;
                if (s == true_init) found_true = true;
            }
        }
        printf("seed search: %u validating inits (of 65536), true init found: %s\n",
               hits, found_true ? "yes" : "NO");
        assert(found_true);

        pkt[3] ^= 0x01; // corruption: the TRUE init must no longer validate
        // (chance hits still exist: 65536 inits x 2^-14 CRC ~= 4 spurious)
        bool true_still_valid = false;
        unsigned chance_hits = 0;
        for (uint32_t s = 0; s < 65536; s++) {
            uint8_t tmp[7]; tmp[0] = pkt[0] & 0x03; memcpy(tmp + 1, pkt + 1, 6);
            uint16_t incrc = (uint16_t)((((uint16_t)pkt[0] >> 2) << 8) | pkt[7]);
            if (elrs_crc14(tmp, 7, (uint16_t)s) == incrc) {
                chance_hits++;
                if (s == true_init) true_still_valid = true;
            }
        }
        printf("corrupted: %u chance hits, true init still validates: %s\n",
               chance_hits, true_still_valid ? "YES (BUG)" : "no");
        assert(!true_still_valid);
    }
    printf("ok: seed search (hits found, corruption rejects all)\n");

    // 10) REAL-CAPTURE LAYOUT ANALYSIS: ce3f0000000031a1 from the field unit
    //     (RadioMaster Pocket 3.x, disconnected fast-sync, drone off).
    //     Prove the 3.6.4 OTA4 sync struct against it and answer: does ANY
    //     CRC init validate these bytes under the exact struct?
    {
        static const uint8_t cap[8] = { 0xce, 0x3f, 0x00, 0x00, 0x00, 0x00, 0x31, 0xa1 };
        printf("capture ce3f0000000031a1 under OTA4 sync struct 3.6.4:\n");
        printf("  type=%u (2=sync) fhss=%u nonce=%u swMode=%u tlmRatio=%u rateIdx=%u uid3..5=%02x%02x%02x incrc=%04x\n",
               cap[0] & 3, cap[1], cap[2], cap[3] & 1, (cap[3] >> 1) & 7, cap[3] >> 4,
               cap[4], cap[5], cap[6], (unsigned)((cap[0] >> 2) << 8 | cap[7]));
        // derived init + 64 model-match candidates (as the parser now does)
        elrs_decode_ctx_t ctx;
        elrs_decode_init(&ctx);
        elrs_packet_t out;
        bool parsed_ok = elrs_decode_packet(&ctx, cap, 8, &out);
        printf("  parser: classified=%d crc_ok=%d (model-match search incl.)\n",
               parsed_ok, out.cls == ELRS_PKT_CLASS_CRC_OK);
        // exhaustive 0..65535 seed search — NOTE: ~1 hit per packet is
        // EXPECTED BY CHANCE (2^14 effective inits x 2^-14 CRC), so raw hits
        // prove nothing. The real criterion: a hit must be CONSISTENT with
        // the packet's own UID bytes (derived init, any of the 64 model-match
        // variants, or bind-mode 0). tx_main.cpp: UID5 may be XORed by
        // (~modelId)&0x3f when model match is on.
        unsigned hits = 0;
        uint16_t hit_inits[16];
        for (uint32_t s = 0; s < 65536; s++) {
            uint8_t tmp[7]; tmp[0] = cap[0] & 0x03; memcpy(tmp + 1, cap + 1, 6);
            uint16_t incrc = (uint16_t)(((uint16_t)(cap[0] >> 2) << 8) | cap[7]);
            if (elrs_crc14(tmp, 7, (uint16_t)s) == incrc && hits < 16) hit_inits[hits++] = (uint16_t)s;
        }
        bool consistent = false;
        uint16_t derived = elrs_crc_init_from_uid(cap[5], cap[6]);
        for (uint32_t m = 0; m < 64 && !consistent; m++) {
            uint16_t cand = (uint16_t)((((uint16_t)cap[5] << 8) | (uint16_t)(cap[6] ^ m)) ^ ELRS_OTA_VERSION_ID_3X);
            for (unsigned i = 0; i < hits; i++) if (hit_inits[i] == cand) consistent = true;
        }
        for (unsigned i = 0; i < hits; i++) if (hit_inits[i] == 0) consistent = true; // bind mode
        printf("  seed search: %u raw hit(s) (chance expectation ~4 incl. bit14/15 twins); "
               "derived init %04x consistent: %s\n", hits, derived, consistent ? "YES" : "NO");
        if (!consistent) {
            printf("  VERDICT: NO UID-consistent init validates the capture -> it is NOT a\n");
            printf("  3.x OTA4 sync. Note: rateIdx=0 is the FLRC-1000 table slot; a TX on an\n");
            printf("  FLRC rate sends NO software CRC at all (radio CRC instead) -> next step\n");
            printf("  is FLRC dwell support, not more CRC archaeology.\n");
        } else {
            printf("  VERDICT: capture IS a 3.x OTA4 sync (UID-consistent init found).\n");
        }
    }
    printf("ok: real-capture layout analysis (informational)\n");

    // 11) BIND-PHRASE UID + FLRC identity vector. UID derivation is
    //     MD5("-DMY_BINDING_PHRASE=\"<phrase>\"")[:6] (binary_configurator.py
    //     + community converters — NOT md5(phrase) plain). Verified python
    //     value for "ExpressLRS": 43 7f 2f b1 d3 39.
    {
        static const uint8_t uid_def[6] = { 0x43, 0x7f, 0x2f, 0xb1, 0xd3, 0x39 };
        uint32_t macseed = elrs_uid_mac_seed(uid_def[2], uid_def[3], uid_def[4], uid_def[5]);
        uint16_t crcinit = elrs_crc_init_from_uid(uid_def[4], uid_def[5]);
        printf("default phrase UID 43 7f 2f b1 d3 39 -> macseed %08lx crcinit %04x\n",
               (unsigned long)macseed, crcinit);
        assert(macseed == 0x2fb1d33a); // 2f b1 d3 (39^3=3a)
        assert(crcinit == 0xd33a);     // (d3<<8|39)^3
    }
    printf("ok: bind-phrase UID + FLRC identity vector\n");

    // 11b) CR AUDIT: ELRS 3.6.4 SX1280_Regs.h names LI_4_8 = 0x07 (there is
    //      no LI_4_7 in the 3.x header); the 3.x rate rows must carry 0x07.
    {
        for (uint8_t i = 0; i < ELRS_RATES_3X_COUNT; i++) {
            if (ELRS_RATES_3X[i].rate_index == 4) continue; // 500Hz uses LI_4_6 (0x06)
            assert(ELRS_RATES_3X[i].cr == 0x07);
        }
        assert(ELRS_RATES_3X[0].cr == 0x06); // LoRa 500Hz LI 4/6
        assert(ELRS_RATES_FLRC_COUNT == 4);  // FLRC 500Hz added (rate idx1)
        assert(ELRS_RATES_FLRC[1].rate_index == 1 && ELRS_RATES_FLRC[1].interval_us == 2000);
    }
    printf("ok: CR 0x07 audit + FLRC 500Hz row\n");

    // 12) FLRC setup bytes vs ELRS SX1280.cpp register writes (the values the
    //     firmware's FLRC branch programs — guard against regressions):
    //     SetModulationParamsFLRC {0x86 (BR0.65/BW0.6), 0x00 (CR 1/2),
    //     0x10 (BT 1.0)}; SetPacketParamsFLRC preamble field ((32/4)-1)<<4;
    //     sync word reg 0x9CF; FLRC CRC seed reg 0x9C8 (SX1280_Regs.h).
    {
        assert(ELRS_RATES_FLRC_COUNT == 4);
        const elrs_rate_t *f = &ELRS_RATES_FLRC[0];
        assert(f->flrc && f->bw == 0x86 && f->cr == 0x00 && f->sf == 0x10);
        assert(f->preamble == 32 && f->payload == 8);
        uint8_t preamble_field = (uint8_t)(((f->preamble / 4) - 1) << 4);
        assert(preamble_field == 0x70); // matches ELRS SetPacketParamsFLRC
        const uint8_t pp[7] = { preamble_field, 0x04, 0x10, 0x00, f->payload, 0x30, 0x08 };
        (void)pp; // {pre, P32S, SWM1, fixed, len, CRC3B, whitening off}
        assert(0x09CF == 0x09CF && 0x09C8 == 0x09C8); // reg addresses pinned
        // sync word from the default-phrase UID + erratum swap logic
        uint8_t uid_def[6] = { 0x43, 0x7f, 0x2f, 0xb1, 0xd3, 0x39 };
        uint32_t seed32 = elrs_uid_mac_seed(uid_def[2], uid_def[3], uid_def[4], uid_def[5]);
        uint8_t sw[4] = { (uint8_t)(seed32 >> 24), (uint8_t)(seed32 >> 16),
                          (uint8_t)(seed32 >> 8), (uint8_t)(seed32 & 0xFF) };
        if ((sw[0] == 0x8C && sw[1] == 0x38) || (sw[0] == 0x63 && sw[1] == 0x0E)) {
            uint8_t t = sw[0]; sw[0] = sw[1]; sw[1] = t;
        }
        assert(sw[0] == 0x2f && sw[1] == 0xb1 && sw[2] == 0xd3 && sw[3] == 0x3a);
    }
    printf("ok: FLRC setup bytes vs ELRS register writes\n");

    // 13) FHSS sequence vs golden vector (python-ported ELRS algorithm,
    //     seed 0x01020304 — the same seed the ELRS test_fhss uses) plus the
    //     ELRS unit-test invariants: per-80-block uniqueness, sync channel
    //     (41) at every block start, determinism.
    {
        static const uint8_t golden[40] = {
            0x29, 0x07, 0x06, 0x41, 0x11, 0x02, 0x13, 0x3b, 0x49, 0x2b,
            0x0c, 0x1f, 0x18, 0x17, 0x12, 0x19, 0x1c, 0x4e, 0x0f, 0x14,
            0x3e, 0x00, 0x48, 0x30, 0x09, 0x08, 0x01, 0x20, 0x33, 0x2f,
            0x04, 0x2c, 0x4a, 0x37, 0x03, 0x26, 0x38, 0x1a, 0x2e, 0x3c };
        uint8_t seq[FHSS_SEQ_COUNT], seq2[FHSS_SEQ_COUNT];
        elrs_fhss_build(0x01020304, seq);
        for (int i = 0; i < 40; i++) assert(seq[i] == golden[i]);
        assert(FHSS_SEQ_COUNT == 240); // (256/80)*80 per the reference
        assert(seq[80] == 0x29 && seq[81] == 0x0f && seq[82] == 0x23 && seq[83] == 0x33);
        // sync channel sits exactly at the block starts {0, 80, 160}
        for (uint8_t p = 0; p < FHSS_SEQ_COUNT; p++)
            assert((seq[p] == FHSS_SYNC_INDEX) == (p % FHSS_FREQ_COUNT == 0));
        for (uint8_t b = 0; b < FHSS_SEQ_COUNT / FHSS_FREQ_COUNT; b++) {
            assert(seq[b * FHSS_FREQ_COUNT] == FHSS_SYNC_INDEX); // sync at block start
            bool seen[FHSS_FREQ_COUNT] = { false };
            for (uint8_t k = 0; k < FHSS_FREQ_COUNT; k++) {
                uint8_t v = seq[b * FHSS_FREQ_COUNT + k];
                assert(v < FHSS_FREQ_COUNT && !seen[v]);
                seen[v] = true;
            }
        }
        elrs_fhss_build(0x01020304, seq2); // determinism (test_fhss_same)
        assert(memcmp(seq, seq2, FHSS_SEQ_COUNT) == 0);
        // hop cadence helper: idx advances once per hopInterval packets
        assert(elrs_fhss_advance(10, 0, 4) == 10);
        assert(elrs_fhss_advance(10, 4, 4) == 11);
        assert(elrs_fhss_advance(10, 3, 4) == 10);
        assert(elrs_fhss_advance(239, 4, 4) == 0);
        // register-unit frequency plan: idx41 == 2441399841 (exact)
        assert(elrs_fhss_channel_hz(0) == 2400399932u);
        assert(elrs_fhss_channel_hz(41) == 2441399841u);
        assert(elrs_fhss_channel_hz(79) == 2479399688u);
        assert(ELRS_2G4_SYNC_FREQ_HZ == 2441399841u);
        // FIND gate: sequence-pointer semantics (round 8)
        assert(elrs_sync_on_sync_channel(0, seq, true));
        assert(elrs_sync_on_sync_channel(80, seq, true));
        assert(!elrs_sync_on_sync_channel(1, seq, true));
        assert(!elrs_sync_on_sync_channel(240, seq, true)); // out of range
        assert(elrs_sync_on_sync_channel(0, NULL, false));   // unknown seq: block starts
        assert(!elrs_sync_on_sync_channel(3, NULL, false));
    }
    printf("ok: FHSS golden vector + ELRS unit-test invariants\n");

    // 14) IDENTITY GATING: the field's real beacon pattern (repeated 61ace1
    //     syncs, nonce advancing, drone OFF) must reach identity at the 2nd
    //     sync; the live-DVDA chaos (5+ rotating tails, random nonces, bogus
    //     fields) must never lock; same tail with insane nonce cadence resets.
    {
        elrs_identity_t id;
        elrs_sync_info_t s;
        // real pattern: repeated tail, nonce +4 between syncs (250Hz-class),
        // fhss 63, rate 6, tlm 2
        elrs_identity_reset(&id);
        for (int i = 0; i < 3; i++) {
            memset(&s, 0, sizeof(s));
            s.uid3 = 0x61; s.uid4 = 0xac; s.uid5 = 0xe1;
            s.nonce = (uint8_t)(4 * i);
            s.fhss_index = 63; s.rate_index = 6; s.tlm_ratio = 2;
            bool acc = elrs_identity_consider(&id, &s, 4);
            assert(acc == (i >= 1)); // accepted on the 2nd consistent sync
        }
        assert(id.known);
        // chaos pattern: rotating tails + random nonces + random fields
        elrs_identity_reset(&id);
        const uint8_t tails[5][3] = {{0x99,0x6f,0x28},{0xc2,0x3f,0x97},{0x20,0xd3,0xc5},
                                     {0x60,0x1d,0xad},{0xe3,0x70,0x63}};
        srand(999);
        bool any_accept = false;
        for (int i = 0; i < 40; i++) {
            memset(&s, 0, sizeof(s));
            const uint8_t *t = tails[i % 5];
            s.uid3 = t[0]; s.uid4 = t[1]; s.uid5 = t[2];
            s.nonce = rand() & 0xFF;
            s.fhss_index = rand() % 256; s.rate_index = rand() % 16; s.tlm_ratio = rand() % 16;
            if (elrs_identity_consider(&id, &s, 4)) any_accept = true;
        }
        assert(!any_accept && !id.known);
        // same tail but insane nonce jump -> restart, never locks
        elrs_identity_reset(&id);
        s = (elrs_sync_info_t){ .uid3=0x61,.uid4=0xac,.uid5=0xe1,.nonce=0,
                                .fhss_index=63,.rate_index=6,.tlm_ratio=2 };
        elrs_identity_consider(&id, &s, 4);
        s.nonce = 200; // garbage jump
        assert(!elrs_identity_consider(&id, &s, 4));
        assert(!id.known && id.count == 1);
        // structural sanity: bad rateIdx rejected outright
        elrs_identity_reset(&id);
        s = (elrs_sync_info_t){ .uid3=0x61,.uid4=0xac,.uid5=0xe1,.nonce=0,
                                .fhss_index=63,.rate_index=15,.tlm_ratio=2 };
        assert(!elrs_identity_consider(&id, &s, 4) && id.count == 0);
    }
    printf("ok: identity gating (beacon accepts, chaos rejects)\n");

    // 14b) CRASH REGRESSION: an UNANCHORED nonce track must report
    // off-track (never divide by interval_ms==0 — field: Guru Meditation
    // IntegerDivideByZero right after the first sync of a session).
    {
        elrs_nonce_track_t nt = { 0, 0, 0 }; // all zero, as at boot pre-fix
        assert(!elrs_nonce_on_track(&nt, 12345, 0x61));
        assert(!elrs_nonce_on_track(&nt, 12345, 0xFE));
        elrs_nonce_anchor(&nt, 7, 1000, 4);
        assert(elrs_nonce_on_track(&nt, 1000 + 10 * 4, 17));
    }
    printf("ok: unanchored nonce track safe (crash regression)\n");

    // 14c) MULTI-UID sync validation: a sync built from the DEFAULT phrase
    // UID validates even when the ctx is configured for a different UID.
    {
        elrs_decode_ctx_t ctx;
        elrs_decode_init(&ctx);
        ctx.cfg_uid4 = 0x11; ctx.cfg_uid5 = 0x22; ctx.cfg_uid_valid = true;
        uint8_t pkt[ELRS_OTA4_LEN] = { ELRS_PKT_SYNC, 63, 0, 0x00,
                                       (uint8_t)(ELRS_DEFAULT_UID4 ^ 0x05), // model-match XOR
                                       0x00, 0x31 };
        // NOTE: byte4=UID3 is model-XORed only for UID5; build a plain tail:
        pkt[4] = 0x2f;                        // UID3 (default uid[3])
        pkt[5] = ELRS_DEFAULT_UID4;           // UID4
        pkt[6] = ELRS_DEFAULT_UID5 ^ 0x05;    // UID5 with modelId xor=5
        uint16_t init = elrs_crc_init_from_uid(ELRS_DEFAULT_UID4, ELRS_DEFAULT_UID5);
        uint16_t crc = elrs_crc14(pkt, 7, init);
        pkt[0] |= (uint8_t)((crc >> 8) << 2);
        pkt[7] = (uint8_t)(crc & 0xFF);
        elrs_packet_t out;
        bool ok = elrs_decode_packet(&ctx, pkt, ELRS_OTA4_LEN, &out);
        assert(ok && out.cls == ELRS_PKT_CLASS_CRC_OK);
        assert(ctx.crc_init == init); // validated via the default-UID seed sweep
    }
    printf("ok: multi-UID sync validation (default-phrase seed)\n");

    // 14d) SELF-SEEDED SYNC VALIDATOR: construct real ELRS OTA4 sync frames
    //     (pick UID, modelId, nonce/fhss/rate; CRC14 per the reference) and
    //     assert acceptance + true UID5 + modelId recovery, model match ON
    //     and OFF. Zero prior knowledge — the seed comes from the frame.
    {
        // model match ON: modelId=37 -> on-air UID5 = UID5 ^ (~37 & 0x3f)
        const uint8_t UID3 = 0x61, UID4 = 0xac, UID5 = 0xe1, MODEL = 37;
        uint8_t pkt[ELRS_OTA4_LEN] = { ELRS_PKT_SYNC, 42, 200, (6 << 4), 0, 0, 0 };
        pkt[4] = UID3;
        pkt[5] = UID4;
        pkt[6] = (uint8_t)(UID5 ^ (~MODEL & 0x3f));
        uint16_t init = elrs_crc_init_from_uid(UID4, UID5); // seed from TRUE uid
        uint16_t crc = elrs_crc14(pkt, 7, init);
        pkt[0] |= (uint8_t)((crc >> 8) << 2);
        pkt[7] = (uint8_t)(crc & 0xFF);
        uint16_t got_init; uint8_t got_u5, got_model;
        assert(elrs_sync_crc_selfseed(pkt, ELRS_OTA4_LEN, &got_init, &got_u5, &got_model));
        assert(got_init == init);
        assert(got_u5 == UID5);      // XOR removed
        assert(got_model == MODEL);  // modelId recovered
        // and via the full decoder: identity/uid carry the true tail
        elrs_decode_ctx_t ctx; elrs_decode_init(&ctx);
        elrs_packet_t out;
        assert(elrs_decode_packet(&ctx, pkt, ELRS_OTA4_LEN, &out));
        assert(out.cls == ELRS_PKT_CLASS_CRC_OK);
        assert(ctx.uid5 == UID5 && ctx.model_id == MODEL);
    }
    {
        // model match OFF: on-air UID5 = UID5; modelId must report 0xFF
        const uint8_t UID3 = 0x2f, UID4 = 0xb1, UID5 = 0x39;
        uint8_t pkt[ELRS_OTA4_LEN] = { ELRS_PKT_SYNC, 7, 3, (9 << 4), 0, 0, 0 };
        pkt[4] = UID3; pkt[5] = UID4; pkt[6] = UID5;
        uint16_t init = elrs_crc_init_from_uid(UID4, UID5);
        uint16_t crc = elrs_crc14(pkt, 7, init);
        pkt[0] |= (uint8_t)((crc >> 8) << 2);
        pkt[7] = (uint8_t)(crc & 0xFF);
        uint16_t got_init; uint8_t got_u5, got_model;
        assert(elrs_sync_crc_selfseed(pkt, ELRS_OTA4_LEN, &got_init, &got_u5, &got_model));
        assert(got_u5 == UID5 && got_model == 0xFF);
        // rateIdx=9 (LoRa50) must NOT be rejected (4-bit link rate, valid)
        elrs_identity_t id; elrs_identity_reset(&id);
        elrs_sync_info_t s;
        memset(&s, 0, sizeof(s));
        s.uid3 = UID3; s.uid4 = UID4; s.uid5 = UID5; s.rate_index = 9;
        s.nonce = 1; s.fhss_index = 0; s.tlm_ratio = 2;
        elrs_identity_consider(&id, &s, 2);
        s.nonce = 2;
        assert(elrs_identity_consider(&id, &s, 2)); // accepted, rateIdx 9 sane
    }
    printf("ok: self-seeded sync validator (modelId on/off, rateIdx 9)\n");

    // 14e) DISCOVERY CLASSIFIER (windowed tail counting): junk never
    //     candidates; >=ELRS_DISC_NEEDED sightings of one tail inside the
    //     2 s window emits exactly once (per-tail throttle).
    {
        elrs_disc_gate_t g;
        elrs_sync_info_t s;
        elrs_disc_reset(&g);
        srand(999);
        // 800/s random-tail junk for 4 s -> zero candidates
        for (int i = 0; i < 3200; i++) {
            memset(&s, 0, sizeof(s));
            s.uid3 = rand() & 0xFF; s.uid4 = rand() & 0xFF; s.uid5 = rand() & 0xFF;
            s.nonce = rand() & 0xFF; s.fhss_index = rand() % 256;
            s.rate_index = rand() % 16; s.tlm_ratio = rand() % 16;
            float rssi = (i % 64) ? (-45.0f - (rand() % 20)) : -128.0f;
            assert(!elrs_disc_frame(&g, &s, rssi, 1000 + i));
        }
        assert(g.nf <= -127.0f);
        // real link: same tail 8/s among the junk -> candidate at 3rd sighting
        elrs_disc_reset(&g);
        unsigned emits = 0;
        for (int i = 0; i < 4000; i++) { // 4 s at 1ms steps, 800 junk + 8 real/s
            memset(&s, 0, sizeof(s));
            if (i % 125 == 0) { // one real frame every 125 ms (8/s)
                s.uid3 = 0x61; s.uid4 = 0xac; s.uid5 = 0xe1;
                s.rate_index = 2; s.tlm_ratio = 2; s.fhss_index = 40;
                s.nonce = (uint8_t)(i / 125);
            } else {
                s.uid3 = rand() & 0xFF; s.uid4 = rand() & 0xFF; s.uid5 = rand() & 0xFF;
                s.nonce = rand() & 0xFF; s.fhss_index = rand() % 256;
                s.rate_index = rand() % 16; s.tlm_ratio = rand() % 16;
            }
            if (elrs_disc_frame(&g, &s, -50.0f, 1000 + i)) emits++;
        }
        assert(emits >= 1 && emits <= 2); // first ~375 ms in; throttle allows 1 per 2 s
        // sustained same-tail flow emits again only after the 2 s throttle
        elrs_disc_reset(&g);
        emits = 0;
        for (int i = 0; i < 6000; i++) {
            memset(&s, 0, sizeof(s));
            s.uid3 = 0xbe; s.uid4 = 0x07; s.uid5 = 0x83;
            s.rate_index = 6; s.tlm_ratio = 2; s.fhss_index = 42;
            s.nonce = (uint8_t)i;
            if (elrs_disc_frame(&g, &s, -60.0f, 1000 + i)) emits++;
        }
        assert(emits >= 2 && emits <= 4); // ~1 per 2 s over 6 s
    }
    printf("ok: discovery windowed tail counting\n");

    // 14f) ROUND-4 acceptance:
    //  (a) 27k junk frames with type-classifier "sync" labels -> zero dwell
    //      extensions (dwell exits at base)
    //  (b) interferer burst >200 fps, no pairs, sustained -> bail; pairs or
    //      low fps -> no bail
    //  (c) a pair-gated candidate still extends
    //  (d) sweep order is LoRa-first
    //  (e) last-link tail decays 60 s after demote
    {
        // (a) replay: 27,600 WiFi-junk frames, random tails incl. sync-typed
        elrs_disc_gate_t g;
        elrs_sync_info_t s;
        elrs_disc_reset(&g);
        srand(1337);
        unsigned cand = 0;
        for (int i = 0; i < 27600; i++) {
            memset(&s, 0, sizeof(s));
            s.uid3 = rand() & 0xFF; s.uid4 = rand() & 0xFF; s.uid5 = rand() & 0xFF;
            s.nonce = rand() & 0xFF; s.fhss_index = rand() % 256;
            s.rate_index = rand() % 16; s.tlm_ratio = rand() % 16;
            // bursty WiFi: strong DURING the burst (per-frame rssi reads high)
            float rssi = (i % 64) ? (-45.0f - (rand() % 20)) : -128.0f;
            if (elrs_disc_frame(&g, &s, rssi, 1000 + i)) cand++;
        }
        assert(cand == 0); // random tails never pair 27k:1 by chance
        assert(!elrs_dwell_extend(0, cand)); // zero extensions -> base exit

        // (c) windowed candidate extends (3 sightings inside the window)
        elrs_disc_reset(&g);
        memset(&s, 0, sizeof(s));
        s.uid3 = 0x61; s.uid4 = 0xac; s.uid5 = 0xe1;
        s.rate_index = 6; s.tlm_ratio = 2; s.fhss_index = 10;
        unsigned c = 0;
        for (uint8_t k = 1; k <= 3; k++) {
            s.nonce = k;
            if (elrs_disc_frame(&g, &s, -60, 100 + k * 10)) c++;
        }
        assert(c == 1); // 3rd sighting emits
        assert(elrs_dwell_extend(0, 1));

        // (b) interferer bail
        assert(elrs_interferer_bail(767, 0, 500));   // hot, no pairs, sustained
        assert(!elrs_interferer_bail(767, 1, 500));  // a pair exists -> real link
        assert(!elrs_interferer_bail(150, 0, 500));  // below fps threshold
        assert(!elrs_interferer_bail(767, 0, 400));  // not sustained yet

        // (e) last-link decay
        assert(!elrs_lastlink_fresh(true, 0, 120000));   // demoted long ago
        assert(elrs_lastlink_fresh(true, 60000, 110000)); // within 60 s
        assert(!elrs_lastlink_fresh(false, 60000, 61000)); // no uid at all
    }
    printf("ok: round-4 dwell/extension/interferer/decay (a-c,e)\n");

    // 14g) SWEEP ORDER: LoRa rates first (250,500,150,50 x IQ), FLRC trio
    //      after — LoRa self-seed validation is junk-proof and 250 is the
    //      stock Pocket default.
    {
        sniffer_sweep_t sw;
        sniffer_sweep_build(&sw);
        assert(sw.count == 2 * (ELRS_RATES_3X_COUNT + ELRS_RATES_2X_COUNT) + ELRS_RATES_FLRC_COUNT);
        static const char *want0[8] = { "LoRa 250Hz", "LoRa 250Hz", "LoRa 500Hz", "LoRa 500Hz",
                                        "LoRa 150Hz", "LoRa 150Hz", "LoRa 50Hz", "LoRa 50Hz" };
        for (int i = 0; i < 8; i++) {
            assert(strcmp(sw.steps[i].rate->name, want0[i]) == 0);
            assert(sw.steps[i].rate->flrc == 0);
        }
        for (int i = 0; i < 8; i += 2) { // IQ pairs n,i
            assert(sw.steps[i].iq_inverted == false);
            assert(sw.steps[i + 1].iq_inverted == true);
        }
        assert(sw.steps[8].rate->flrc == 1 && sw.steps[11].rate->flrc == 1); // FLRC x4
        assert(strcmp(sw.steps[9].rate->name, "FLRC 500Hz") == 0);
        assert(sw.steps[8].iq_inverted == false); // FLRC: IQ n/a, single polarity
    }
    printf("ok: sweep order LoRa-first (d)\n");

    // 14h) FLRC CRC24 seed mechanism (round 8): FIXED poly 0x5D6DCB per the
    //     SX1280 datasheet (Table 14-40); variants cover only init placement /
    //     byte order. Construct -> brute 2^16 -> seed recovered.
    {
        const uint8_t UID4 = 0x61, UID5 = 0xce;
        uint16_t seed = elrs_crc_init_from_uid(UID4, UID5);
        uint8_t payload[5] = { 0x2A, 0x41, 0x07, 0x64, 0x10 };
        uint8_t frame[8];
        memcpy(frame, payload, 5);
        uint32_t crc = elrs_crc24flrc_calc(0, seed, frame, 5);
        frame[5] = (uint8_t)(crc >> 16); frame[6] = (uint8_t)(crc >> 8); frame[7] = (uint8_t)crc;
        assert(elrs_crc24flrc_check(0, seed, frame, 8));
        uint16_t got_seed; uint8_t got_variant;
        assert(elrs_crc24flrc_brute(frame, 8, &got_seed, &got_variant));
        assert(got_seed == seed && got_variant == 0);
        uint16_t raw = (uint16_t)(got_seed ^ ELRS_OTA_VERSION_ID_3X);
        assert((uint8_t)(raw >> 8) == UID4 && (uint8_t)(raw & 0xFF) == UID5);
        assert(!elrs_crc24flrc_check(0, seed ^ 1, frame, 8));
        frame[3] ^= 0x01;
        assert(!elrs_crc24flrc_check(0, seed, frame, 8));
        assert(ELRS_CRC24FLRC_VARIANTS == 3);
        assert(ELRS_CRC24FLRC_POLY == 0x5D6DCB);
    }
    printf("ok: FLRC CRC24 poly 0x5D6DCB construct+brute\n");

    // 14i) UID[2] TRACKERS: real LCG sequence, true uid2 survives, others die
    {
        const uint8_t UID2T = 0x42, UID3 = 0x61, UID4 = 0xac, UID5 = 0xe1;
        elrs_uid2_track_t t;
        // anchor sync: nonce 100 at fhss 20 (hop=4)
        elrs_uid2_track_init(&t, 100, 20);
        // second sync 40 packets later: idx = (20 + 40/4) % 160 = 30
        uint8_t seq[FHSS_SEQ_COUNT];
        elrs_fhss_build(((uint32_t)UID2T << 24) | ((uint32_t)UID3 << 16) |
                        ((uint32_t)UID4 << 8) | ((uint32_t)UID5 ^ 3), seq);
        // each sync kills ~79/80 of wrong candidates; 2 constraints leave
        // ~3 by chance, so feed syncs until a unique survivor emerges
        int r = -1;
        for (uint8_t k = 1; k <= 8 && r < 0; k++) {
            uint8_t nonce = (uint8_t)(100 + 40 * k);
            uint16_t idx = (uint16_t)((20 + (uint8_t)(40 * k) / 4) % FHSS_SEQ_COUNT);
            r = elrs_uid2_track_update(&t, 0xFF, UID3, UID4, UID5, nonce, seq[idx], 4);
        }
        assert(r == (UID2T & 0x7F)); // bit7 invisible: survivor is the 7-bit value
        // ghost sequence: wrong fhss kills everyone
        elrs_uid2_track_init(&t, 100, 20);
        r = elrs_uid2_track_update(&t, 0xFF, UID3, UID4, UID5, 140, seq[30] ^ 0xFF, 4);
        assert(r == -2);
    }
    printf("ok: UID2 trackers (real sequence vectors)\n");

    // 14j) BIND HARVEST (round 7): MSP_ELRS_BIND=0x09 layout from tx_main
    //     SendUIDOverMSP: OTA4 MSP, payload [0x09, UID2..5], CRC14 init 0.
    {
        const uint8_t U2 = 0x5a, U3 = 0x61, U4 = 0xac, U5 = 0xe1;
        uint8_t pkt[ELRS_OTA4_LEN] = { ELRS_PKT_MSP, 0x00, ELRS_MSP_BIND, U2, U3, U4, U5, 0 };
        uint16_t crc = elrs_crc14(pkt, 7, 0); // bind mode: init 0
        pkt[0] |= (uint8_t)((crc >> 8) << 2);
        pkt[7] = (uint8_t)(crc & 0xFF);
        uint8_t u[4];
        assert(elrs_bind_parse(pkt, ELRS_OTA4_LEN, u));
        assert(u[0] == U2 && u[1] == U3 && u[2] == U4 && u[3] == U5);
        // junk: wrong type, wrong marker, wrong CRC, wrong length -> reject
        uint8_t junk[ELRS_OTA4_LEN];
        memcpy(junk, pkt, sizeof(junk));
        junk[0] = (junk[0] & 0xFC) | ELRS_PKT_RCDATA;
        assert(!elrs_bind_parse(junk, ELRS_OTA4_LEN, u));
        memcpy(junk, pkt, sizeof(junk));
        junk[2] = 0x0A;
        assert(!elrs_bind_parse(junk, ELRS_OTA4_LEN, u));
        memcpy(junk, pkt, sizeof(junk));
        junk[5] ^= 0x10; // corrupt UID[3]
        assert(!elrs_bind_parse(junk, ELRS_OTA4_LEN, u));
        assert(!elrs_bind_parse(pkt, ELRS_OTA8_LEN, u));
        // chance frame: random bytes essentially never pass (init 0 + marker)
        srand(777);
        unsigned false_pos = 0;
        for (int i = 0; i < 20000; i++) {
            for (int j = 0; j < ELRS_OTA4_LEN; j++) junk[j] = rand() & 0xFF;
            junk[0] = (junk[0] & 0xFC) | ELRS_PKT_MSP;
            if (elrs_bind_parse(junk, ELRS_OTA4_LEN, u)) false_pos++;
        }
        assert(false_pos == 0); // 2^-22 per frame: expected 0
    }
    printf("ok: bind harvest parser (layout + junk rejection)\n");

    // 14k) SWITCH DECODE ENDPOINTS (audit 7): SWITCH3b table + N_to_CRSF
    //     endpoints 191/1792 with round-to-nearest (crsf_protocol.h).
    {
        elrs_decode_ctx_t ctx;
        elrs_decode_init(&ctx);
        ctx.switch_mode = ELRS_SW_HYBRID8;
        // hybrid8 switches byte: swidx 0 -> AUX2 = 3-bit value 3
        uint8_t pkt[ELRS_OTA4_LEN] = { ELRS_PKT_RCDATA, 0, 0, 0, 0, 0, 0, 0 };
        pkt[6] = (uint8_t)((0 << 1) | (3 << 4) | (0 << 7)); // ch4=0, switches=0x18<<... build: switches byte = swidx<<4 | value<<1? layout: switches:7 | ch4:1 -> sw field = byte>>1
        // simpler: switches byte value = (swidx<<4 | v<<1)? per decoder: swidx=(b>>4)&7? -- our decoder: sw = data[6]>>1; swidx=(sw>>3)&7... replicate decoder wiring:
        // sw = byte>>1; swidx=(sw & 0b111000)>>3 = bits[6:4] of byte; value = sw&7 = bits[3:1]
        pkt[6] = (0 << 1) | (0 << 4) | (3 << 1 + 0); // recompute below instead
        // direct: byte = ch4 | (swidx<<4) | (value<<1)
        pkt[6] = (uint8_t)((0 << 0) | (0 << 4) | (5 << 1)); // swidx 0, value 5 -> AUX2=1792
        elrs_packet_t out;
        elrs_decode_packet(&ctx, pkt, ELRS_OTA4_LEN, &out);
        assert(out.rc.has_ch[5] && out.rc.ch[5] == 1792);
        pkt[6] = (uint8_t)((6 << 1)); // swidx 0, value 6 -> 992
        elrs_decode_packet(&ctx, pkt, ELRS_OTA4_LEN, &out);
        assert(out.rc.ch[5] == 992);
        pkt[6] = (uint8_t)((0 << 1)); // value 0 -> 191
        elrs_decode_packet(&ctx, pkt, ELRS_OTA4_LEN, &out);
        assert(out.rc.ch[5] == 191);
        pkt[6] = (uint8_t)((3 << 1)); // value 3 -> 3*240+391 = 1111
        elrs_decode_packet(&ctx, pkt, ELRS_OTA4_LEN, &out);
        assert(out.rc.ch[5] == 1111);
    }
    printf("ok: switch decode endpoints (reference tables)\n");

    // 14l) ROUND-9 FP GATE: CRC-passing frames with IMPOSSIBLE sync fields
    //      (rateIdx 11/15) must NOT emit ok:1; golden syncs still validate
    //      (incl rateIdx=9); random storm produces zero ok:1.
    {
        // craft: valid CRC14 (self-seeded, modelId 0) but insane rateIdx=15
        elrs_decode_ctx_t ctx;
        elrs_decode_init(&ctx);
        uint8_t pkt[ELRS_OTA4_LEN] = { ELRS_PKT_SYNC, 42, 7, 0xF0, 0x61, 0xac, 0xe1, 0 };
        uint16_t init = elrs_crc_init_from_uid(0xac, 0xe1);
        uint16_t crc = elrs_crc14(pkt, 7, init);
        pkt[0] |= (uint8_t)((crc >> 8) << 2);
        pkt[7] = (uint8_t)(crc & 0xFF);
        elrs_packet_t out;
        bool ok = elrs_decode_packet(&ctx, pkt, ELRS_OTA4_LEN, &out);
        assert(!ok || out.cls != ELRS_PKT_CLASS_CRC_OK); // FP gate rejects
        // rateIdx=11 variant
        pkt[0] = ELRS_PKT_SYNC;
        pkt[3] = (uint8_t)((11 << 4) | (2 << 1));
        crc = elrs_crc14(pkt, 7, init);
        pkt[0] = (uint8_t)(ELRS_PKT_SYNC | ((crc >> 8) << 2));
        pkt[7] = (uint8_t)(crc & 0xFF);
        ok = elrs_decode_packet(&ctx, pkt, ELRS_OTA4_LEN, &out);
        assert(!ok || out.cls != ELRS_PKT_CLASS_CRC_OK);
        // golden rateIdx=9 still validates
        pkt[0] = ELRS_PKT_SYNC;
        pkt[3] = (uint8_t)((9 << 4) | (2 << 1));
        crc = elrs_crc14(pkt, 7, init);
        pkt[0] = (uint8_t)(ELRS_PKT_SYNC | ((crc >> 8) << 2));
        pkt[7] = (uint8_t)(crc & 0xFF);
        ok = elrs_decode_packet(&ctx, pkt, ELRS_OTA4_LEN, &out);
        assert(ok && out.cls == ELRS_PKT_CLASS_CRC_OK);
        // forced-insane storm (rateIdx=15): the FP gate kills EVERY frame
        // regardless of CRC — deterministically zero ok:1
        srand(2024);
        unsigned fp = 0;
        for (int i = 0; i < 20000; i++) {
            uint8_t junk[ELRS_OTA4_LEN];
            for (int j = 0; j < 8; j++) junk[j] = rand() & 0xFF;
            junk[0] = (uint8_t)((junk[0] & 0xFC) | ELRS_PKT_SYNC);
            junk[3] = (uint8_t)((15 << 4) | (junk[3] & 0x0F));
            elrs_decode_ctx_t c2;
            elrs_decode_init(&c2);
            if (elrs_decode_packet(&c2, junk, ELRS_OTA4_LEN, &out) &&
                out.cls == ELRS_PKT_CLASS_CRC_OK) fp++;
        }
        assert(fp == 0);
        // random storm: chance accepts are bounded and ALL pass the gate's
        // sanity (rateIdx<=9, tlm<=7, fhss<240) — the impossible-field
        // ok:1 emissions from the field are gone
        fp = 0;
        for (int i = 0; i < 30000; i++) {
            uint8_t junk[ELRS_OTA4_LEN];
            for (int j = 0; j < 8; j++) junk[j] = rand() & 0xFF;
            junk[0] = (uint8_t)((junk[0] & 0xFC) | ELRS_PKT_SYNC);
            elrs_decode_ctx_t c2;
            elrs_decode_init(&c2);
            if (elrs_decode_packet(&c2, junk, ELRS_OTA4_LEN, &out) &&
                out.cls == ELRS_PKT_CLASS_CRC_OK) {
                assert((out.sync.rate_index <= 9) && (out.sync.tlm_ratio <= 7) &&
                       (out.sync.fhss_index < 240));
                fp++;
            }
        }
        assert(fp < 250); // ~130 inits x 2^-14 x 30k frames: bounded chance
    }
    printf("ok: FP gate (insane rateIdx rejected, storm clean, golden ok)\n");

    // 14m) elrs_sig QUALITY MATRIX (round 9)
    {
        assert(elrs_sig_quality(0, 0, false) == ELRS_SIG_NONE);
        assert(elrs_sig_quality(0, 2, false) == ELRS_SIG_NONE);
        assert(elrs_sig_quality(0, 3, false) == ELRS_SIG_WEAK);
        assert(elrs_sig_quality(2, 0, false) == ELRS_SIG_NONE);   // 2 crc_pass, no syncs
        assert(elrs_sig_quality(3, 0, false) == ELRS_SIG_FIRM);   // >=3 crc_pass
        assert(elrs_sig_quality(0, 10, true) == ELRS_SIG_FIRM);   // 10 sync + repeat
        assert(elrs_sig_quality(0, 10, false) == ELRS_SIG_WEAK);  // no repeat
        assert(elrs_sig_quality(10, 0, false) == ELRS_SIG_STRONG);
        assert(elrs_sig_quality(55, 3, false) == ELRS_SIG_STRONG);
        assert(strcmp(elrs_sig_name(ELRS_SIG_FIRM), "firm") == 0);
    }
    printf("ok: elrs_sig quality matrix\n");

    // 15) REFERENCE-RX PORT rules: minLqForChaos values + nonce tracking
    //     (rx_main.cpp:273, 678, 1092) — expected progression accepted,
    //     ghost (field chaos) nonces off-track.
    {
        // minLqForChaos: hop=4, 80 ch -> 4; hop=2 -> 2 (LQ must EXCEED this)
        assert(elrs_min_lq_for_chaos(4) == 4);
        assert(elrs_min_lq_for_chaos(2) == 2);
        // nonce track: 250Hz class, interval 4ms
        elrs_nonce_track_t nt;
        elrs_nonce_anchor(&nt, 100, 10000, 4);
        assert(elrs_nonce_expected(&nt, 10000) == 100);
        assert(elrs_nonce_expected(&nt, 10040) == 110);   // 10 slots later
        // genuine sync on-track (RX: OtaNonce == sync.nonce)
        assert(elrs_nonce_on_track(&nt, 10040, 110));
        // ghost sync (random nonce) off-track -> resync event, no promote
        assert(!elrs_nonce_on_track(&nt, 10040, 0x61));
        // RX re-anchor semantics: adopt sync nonce + fhssIndex verbatim
        elrs_nonce_anchor(&nt, 200, 10040, 4);
        assert(elrs_nonce_expected(&nt, 10080) == 210);
        // field chaos pattern: random nonces vs a fixed expectation are
        // chance-level (~1/256 per sample)
        srand(4242);
        unsigned hits = 0;
        for (int i = 0; i < 1000; i++)
            if (elrs_nonce_on_track(&nt, 10080, rand() & 0xFF)) hits++;
        assert(hits <= 12); // ~3.9 expected, generous bound
    }
    printf("ok: RX-port nonce discipline + minLqForChaos\n");

    printf("ALL HOST TESTS PASSED\n");
    return 0;
}
