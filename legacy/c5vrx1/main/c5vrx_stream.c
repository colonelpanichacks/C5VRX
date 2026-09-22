/* SPDX-License-Identifier: GPL-3.0-only */

#include "c5vrx_stream.h"

#include <string.h>

void c5vrx_rf_block_queue_init(c5vrx_rf_block_queue_t *queue)
{
    if (queue) {
        memset(queue, 0, sizeof(*queue));
        atomic_flag_clear(&queue->lock);
    }
}

bool c5vrx_rf_block_queue_push(c5vrx_rf_block_queue_t *queue,
                               const c5vrx_rf_block_t *block)
{
    if (!queue || !block) {
        return false;
    }
    while (atomic_flag_test_and_set(&queue->lock)) {}
    if (queue->occupancy == C5VRX_RF_BLOCK_QUEUE_CAPACITY) {
        atomic_flag_clear(&queue->lock);
        return false;
    }
    queue->slots[queue->write_index] = *block;
    queue->write_index = (queue->write_index + 1u) % C5VRX_RF_BLOCK_QUEUE_CAPACITY;
    ++queue->occupancy;
    if (queue->occupancy > queue->high_water_mark) {
        queue->high_water_mark = queue->occupancy;
    }
    atomic_flag_clear(&queue->lock);
    return true;
}

bool c5vrx_rf_block_queue_pop(c5vrx_rf_block_queue_t *queue,
                              c5vrx_rf_block_t *block)
{
    if (!queue || !block) {
        return false;
    }
    while (atomic_flag_test_and_set(&queue->lock)) {}
    if (queue->occupancy == 0u) {
        atomic_flag_clear(&queue->lock);
        return false;
    }
    *block = queue->slots[queue->read_index];
    queue->read_index = (queue->read_index + 1u) % C5VRX_RF_BLOCK_QUEUE_CAPACITY;
    --queue->occupancy;
    atomic_flag_clear(&queue->lock);
    return true;
}

void c5vrx_cvbs_conditioner_init(c5vrx_cvbs_conditioner_t *conditioner,
                                 const c5vrx_cvbs_conditioner_config_t *config)
{
    if (!conditioner || !config) {
        return;
    }
    memset(conditioner, 0, sizeof(*conditioner));
    conditioner->config = *config;
    conditioner->dc_q8 = config->bias_q8;
    conditioner->filter_q8 = ((int32_t)config->black_code) << 8;
    conditioner->primed = true;
}

static int clamp_int(int value, int low, int high)
{
    if (value < low) return low;
    if (value > high) return high;
    return value;
}

void c5vrx_cvbs_condition(c5vrx_cvbs_conditioner_t *conditioner,
                          const uint8_t *phase_delta,
                          uint8_t *cvbs,
                          size_t count,
                          uint8_t *minimum,
                          uint8_t *maximum)
{
    if (!conditioner || !phase_delta || !cvbs || count == 0u) {
        return;
    }
    const c5vrx_cvbs_conditioner_config_t *cfg = &conditioner->config;
    uint8_t lo = 63u;
    uint8_t hi = 0u;
    for (size_t i = 0; i < count; ++i) {
        const int input_q8 = ((int)(phase_delta[i] & 0x3fu)) << 8;
        /* Very slow DC tracker: roughly 3.3 ms at 20 MS/s for 2^16 samples. */
        conditioner->dc_q8 += (input_q8 - conditioner->dc_q8) >> 16;
        int centered_q8 = input_q8 - conditioner->dc_q8;
        if (cfg->invert) {
            centered_q8 = -centered_q8;
        }
        int target_q8 = (((int32_t)cfg->black_code) << 8) +
                        (centered_q8 * cfg->gain_q8 >> 8);
        if (cfg->filter_shift) {
            conditioner->filter_q8 +=
                (target_q8 - conditioner->filter_q8) >> cfg->filter_shift;
            target_q8 = conditioner->filter_q8;
        }
        int code = (target_q8 + 128) >> 8;
        code = clamp_int(code, cfg->clamp_min, cfg->clamp_max);
        cvbs[i] = (uint8_t)code;
        if (cvbs[i] < lo) lo = cvbs[i];
        if (cvbs[i] > hi) hi = cvbs[i];
    }
    if (minimum) *minimum = lo;
    if (maximum) *maximum = hi;
}
