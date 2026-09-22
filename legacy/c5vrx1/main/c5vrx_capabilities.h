/* SPDX-License-Identifier: GPL-3.0-only */

#pragma once

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    uint32_t measured_source_rate;
    bool source_bandwidth_known;
    bool phase_continuity_valid;
    bool continuous_wrap_valid;
    bool writer_position_valid;
    bool adjacent_memory_valid;
    bool pipeline_service_headroom_valid;
    bool hardware_decimation_available;
    bool bitscrambler_path_available;
    uint8_t sparse_factor_allowed;
    int cpu_margin_percent;
} c5vrx_receiver_capabilities_t;

typedef enum {
    C5VRX_CONSUMER_NONE = 0,
    C5VRX_CONSUMER_LOWER_RATE_HARDWARE_TAP,
    C5VRX_CONSUMER_BITSCRAMBLER_RING,
    C5VRX_CONSUMER_SPARSE_CPU,
    C5VRX_CONSUMER_CPU_ALL_SAMPLES,
} c5vrx_consumer_strategy_t;

c5vrx_consumer_strategy_t c5vrx_select_consumer(
    const c5vrx_receiver_capabilities_t *capabilities,
    const char **fail_reason);
const char *c5vrx_consumer_strategy_name(c5vrx_consumer_strategy_t strategy);
