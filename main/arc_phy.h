#pragma once

#include <stdbool.h>
#include <stdint.h>

#define ARC_RX_STAGE_COUNT 9u
#define ARC_VENDOR_GAIN_MAX 89u

typedef struct {
    uint8_t spans[ARC_RX_STAGE_COUNT];
    uint8_t max_index;
    bool runtime_spans_valid;
} arc_gain_table_t;

typedef struct {
    uint8_t gain_index;
    uint8_t rf_stage;
    uint16_t rf_code;
    uint8_t bb_code;
    uint8_t fine_code;
    uint32_t packed_state;
} arc_gain_tuple_t;

typedef struct {
    uint8_t enable;
    int8_t coef0;
    int8_t coef1;
} arc_iq_correction_t;

/* Capture the gain-table description produced by the pinned vendor PHY.
 * This function is read-only and must run after normal PHY initialization. */
void arc_phy_capture_gain_table(arc_gain_table_t *table);

/* Pure helpers are kept public so the closed-PHY reconstruction can be
 * regression-tested on the host. */
void arc_gain_table_from_bytes(arc_gain_table_t *table,
                               const uint8_t spans[ARC_RX_STAGE_COUNT],
                               uint8_t max_index);
bool arc_gain_tuple_decode(const arc_gain_table_t *table, uint8_t gain_index,
                           arc_gain_tuple_t *tuple);
uint8_t arc_gain_highest_rf_stage_start(const arc_gain_table_t *table);
arc_iq_correction_t arc_iq_correction_decode(uint32_t reg);
