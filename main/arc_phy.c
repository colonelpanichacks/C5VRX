#include "arc_phy.h"

#include <stddef.h>
#include <string.h>

/* Exact library contract:
 * esp-phy-lib 59c1234e929212aec0fdda75769b759951235536, ESP32-C5.
 * phy_set_rx_gain_table() copies the active 5 GHz stage spans from +0x422,
 * stores generated table maxima at +0x124..+0x126, and caps each table at 89.
 * phy_gen_rx_gain_table() packs RF[20:12], BB[10:4], fine[2:0]. */
#define PHY_PARAM_RX_MAX_A_OFFSET 0x124u
#define PHY_PARAM_RX_MAX_B_OFFSET 0x125u
#define PHY_PARAM_RX_MAX_C_OFFSET 0x126u
#define PHY_PARAM_RX_SPANS_OFFSET 0x422u

extern unsigned char phy_param[];

static const uint8_t s_default_spans[ARC_RX_STAGE_COUNT] = {
    15u, 13u, 5u, 8u, 6u, 4u, 4u, 6u, 0u
};

static const uint16_t s_rf_codes[ARC_RX_STAGE_COUNT] = {
    64u, 100u, 93u, 94u, 107u, 119u, 124u, 125u, 127u
};

static uint8_t conservative_table_max(const uint8_t maxima[3])
{
    uint8_t result = ARC_VENDOR_GAIN_MAX;
    bool found = false;
    for (unsigned i = 0; i < 3u; ++i) {
        if (maxima[i] <= ARC_VENDOR_GAIN_MAX && maxima[i] >= 2u) {
            if (!found || maxima[i] < result) result = maxima[i];
            found = true;
        }
    }
    return found ? result : ARC_VENDOR_GAIN_MAX;
}
void arc_gain_table_from_bytes(arc_gain_table_t *table,
                               const uint8_t spans[ARC_RX_STAGE_COUNT],
                               uint8_t max_index)
{
    if (!table) return;
    memset(table, 0, sizeof(*table));
    table->max_index = max_index <= ARC_VENDOR_GAIN_MAX && max_index >= 2u ?
                       max_index : ARC_VENDOR_GAIN_MAX;

    unsigned covered = 0u;
    bool valid = spans != NULL;
    if (valid) {
        for (unsigned i = 0; i + 1u < ARC_RX_STAGE_COUNT; ++i) {
            if (spans[i] == 0u) { valid = false; break; }
            covered += spans[i];
            if (covered > (unsigned)table->max_index) { valid = false; break; }
        }
    }

    memcpy(table->spans, valid ? spans : s_default_spans,
           sizeof(table->spans));
    table->runtime_spans_valid = valid;

    covered = 0u;
    for (unsigned i = 0; i + 1u < ARC_RX_STAGE_COUNT; ++i)
        covered += table->spans[i];
    table->spans[ARC_RX_STAGE_COUNT - 1u] =
        covered <= table->max_index ? (uint8_t)(table->max_index + 1u - covered) : 0u;
}

void arc_phy_capture_gain_table(arc_gain_table_t *table)
{
    uint8_t maxima[3] = {
        phy_param[PHY_PARAM_RX_MAX_A_OFFSET],
        phy_param[PHY_PARAM_RX_MAX_B_OFFSET],
        phy_param[PHY_PARAM_RX_MAX_C_OFFSET],
    };
    arc_gain_table_from_bytes(table, &phy_param[PHY_PARAM_RX_SPANS_OFFSET],
                              conservative_table_max(maxima));
}

bool arc_gain_tuple_decode(const arc_gain_table_t *table, uint8_t gain_index,
                           arc_gain_tuple_t *tuple)
{
    if (!table || !tuple || gain_index > table->max_index) return false;

    unsigned start = 0u;
    unsigned stage = ARC_RX_STAGE_COUNT - 1u;
    for (unsigned i = 0; i + 1u < ARC_RX_STAGE_COUNT; ++i) {
        unsigned end = start + table->spans[i];
        if (gain_index < end) { stage = i; break; }
        start = end;
    }
    unsigned within = (unsigned)gain_index - start;
    unsigned coarse = within / 6u;
    if (coarse > 6u) coarse = 6u;

    tuple->gain_index = gain_index;
    tuple->rf_stage = (uint8_t)stage;
    tuple->rf_code = s_rf_codes[stage];
    tuple->bb_code = (uint8_t)((1u << (coarse + 1u)) - 1u);
    tuple->fine_code = (uint8_t)(5u - (within % 6u));
    tuple->packed_state = ((uint32_t)tuple->rf_code << 12) |
                          ((uint32_t)tuple->bb_code << 4) |
                          tuple->fine_code;
    return true;
}

uint8_t arc_gain_highest_rf_stage_start(const arc_gain_table_t *table)
{
    if (!table) return 61u;
    unsigned start = 0u;
    for (unsigned i = 0; i + 1u < ARC_RX_STAGE_COUNT; ++i)
        start += table->spans[i];
    return start <= table->max_index ? (uint8_t)start : table->max_index;
}

static int8_t sign_extend(uint32_t value, unsigned bits)
{
    uint32_t sign = 1u << (bits - 1u);
    return (int8_t)((int32_t)((value ^ sign) - sign));
}

arc_iq_correction_t arc_iq_correction_decode(uint32_t reg)
{
    return (arc_iq_correction_t) {
        .enable = (uint8_t)((reg >> 29) & 0x7u),
        .coef0 = sign_extend((reg >> 22) & 0x7fu, 7u),
        .coef1 = sign_extend((reg >> 16) & 0x3fu, 6u),
    };
}
