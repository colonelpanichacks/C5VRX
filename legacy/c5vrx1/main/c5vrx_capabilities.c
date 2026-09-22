/* SPDX-License-Identifier: GPL-3.0-only */

#include "c5vrx_capabilities.h"

#include <stddef.h>

const char *c5vrx_consumer_strategy_name(c5vrx_consumer_strategy_t strategy)
{
    switch (strategy) {
        case C5VRX_CONSUMER_LOWER_RATE_HARDWARE_TAP: return "LOWER_RATE_HARDWARE_TAP";
        case C5VRX_CONSUMER_BITSCRAMBLER_RING: return "BITSCRAMBLER_RING";
        case C5VRX_CONSUMER_SPARSE_CPU: return "SPARSE_CPU";
        case C5VRX_CONSUMER_CPU_ALL_SAMPLES: return "CPU_ALL_SAMPLES";
        default: return "NONE";
    }
}

c5vrx_consumer_strategy_t c5vrx_select_consumer(
    const c5vrx_receiver_capabilities_t *capabilities,
    const char **fail_reason)
{
    if (fail_reason) *fail_reason = NULL;
    if (!capabilities) {
        if (fail_reason) *fail_reason = "CAPABILITIES_MISSING";
        return C5VRX_CONSUMER_NONE;
    }
    if (!capabilities->measured_source_rate) {
        if (fail_reason) *fail_reason = "SOURCE_RATE_UNKNOWN";
        return C5VRX_CONSUMER_NONE;
    }
    if (!capabilities->phase_continuity_valid) {
        if (fail_reason) *fail_reason = "PHASE_CONTINUITY_UNPROVEN";
        return C5VRX_CONSUMER_NONE;
    }
    if (!capabilities->source_bandwidth_known) {
        if (fail_reason) *fail_reason = "ANTI_ALIAS_BANDWIDTH_UNKNOWN";
        return C5VRX_CONSUMER_NONE;
    }
    if (!capabilities->continuous_wrap_valid) {
        if (fail_reason) *fail_reason = "CONTINUOUS_WRAP_UNPROVEN";
        return C5VRX_CONSUMER_NONE;
    }
    if (!capabilities->writer_position_valid) {
        if (fail_reason) *fail_reason = "WRITER_POSITION_UNPROVEN";
        return C5VRX_CONSUMER_NONE;
    }
    if (!capabilities->adjacent_memory_valid) {
        if (fail_reason) *fail_reason = "ADJACENT_MEMORY_SAFETY_UNPROVEN";
        return C5VRX_CONSUMER_NONE;
    }
    if (!capabilities->pipeline_service_headroom_valid) {
        if (fail_reason) *fail_reason = "SERVICE_DEADLINE_HEADROOM_UNPROVEN";
        return C5VRX_CONSUMER_NONE;
    }
    if (capabilities->hardware_decimation_available)
        return C5VRX_CONSUMER_LOWER_RATE_HARDWARE_TAP;
    /* The implemented BitScrambler program is exactly /4. A successful
     * transform benchmark alone is not an anti-alias proof. */
    if (capabilities->bitscrambler_path_available &&
        capabilities->sparse_factor_allowed >= 4u)
        return C5VRX_CONSUMER_BITSCRAMBLER_RING;
    if (capabilities->sparse_factor_allowed >= 2u &&
        capabilities->cpu_margin_percent >= 20)
        return C5VRX_CONSUMER_SPARSE_CPU;
    if (capabilities->cpu_margin_percent >= 20)
        return C5VRX_CONSUMER_CPU_ALL_SAMPLES;
    if (fail_reason) *fail_reason = "NO_MEASURED_CONSUMER_WITH_CPU_MARGIN";
    return C5VRX_CONSUMER_NONE;
}
