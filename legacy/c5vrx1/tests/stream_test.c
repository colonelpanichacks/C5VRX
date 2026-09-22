/* SPDX-License-Identifier: GPL-3.0-only */

#include <assert.h>
#include <stdio.h>
#include <string.h>

#include "c5vrx_stream.h"

int main(void)
{
    c5vrx_rf_block_queue_t queue;
    c5vrx_rf_block_queue_init(&queue);
    for (unsigned i = 0; i < C5VRX_RF_BLOCK_QUEUE_CAPACITY; ++i) {
        c5vrx_rf_block_t block = {.sequence = i};
        assert(c5vrx_rf_block_queue_push(&queue, &block));
    }
    c5vrx_rf_block_t overflow = {0};
    assert(!c5vrx_rf_block_queue_push(&queue, &overflow));
    assert(queue.high_water_mark == C5VRX_RF_BLOCK_QUEUE_CAPACITY);
    for (unsigned i = 0; i < C5VRX_RF_BLOCK_QUEUE_CAPACITY; ++i) {
        c5vrx_rf_block_t block;
        assert(c5vrx_rf_block_queue_pop(&queue, &block));
        assert(block.sequence == i);
    }

    const c5vrx_cvbs_conditioner_config_t cfg = {
        .bias_q8 = 32 << 8, .gain_q8 = 256, .black_code = 20,
        .clamp_min = 4, .clamp_max = 55, .filter_shift = 0,
    };
    c5vrx_cvbs_conditioner_t conditioner;
    c5vrx_cvbs_conditioner_init(&conditioner, &cfg);
    const uint8_t fm[] = {32, 0, 32, 63};
    uint8_t cvbs[sizeof(fm)] = {0};
    uint8_t lo = 0, hi = 0;
    c5vrx_cvbs_condition(&conditioner, fm, cvbs, sizeof(fm), &lo, &hi);
    assert(cvbs[0] == 20);
    assert(cvbs[1] == 4);
    assert(cvbs[2] == 20);
    assert(cvbs[3] == 51);
    assert(lo == 4 && hi == 51);

    puts("stream_test: PASS");
    return 0;
}
