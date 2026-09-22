/* SPDX-License-Identifier: GPL-3.0-only */

#include <assert.h>
#include <string.h>
#include "c5vrx_capabilities.h"

int main(void)
{
    const char *reason = NULL;
    c5vrx_receiver_capabilities_t c = {0};
    assert(c5vrx_select_consumer(&c, &reason) == C5VRX_CONSUMER_NONE);
    assert(strcmp(reason, "SOURCE_RATE_UNKNOWN") == 0);
    c.measured_source_rate = 80000000u;
    c.phase_continuity_valid = true;
    c.source_bandwidth_known = true;
    c.continuous_wrap_valid = true;
    c.writer_position_valid = true;
    c.adjacent_memory_valid = true;
    c.pipeline_service_headroom_valid = true;
    c.cpu_margin_percent = 30;
    c.sparse_factor_allowed = 4;
    assert(c5vrx_select_consumer(&c, &reason) == C5VRX_CONSUMER_SPARSE_CPU);
    c.bitscrambler_path_available = true;
    assert(c5vrx_select_consumer(&c, &reason) == C5VRX_CONSUMER_BITSCRAMBLER_RING);
    c.sparse_factor_allowed = 2;
    assert(c5vrx_select_consumer(&c, &reason) == C5VRX_CONSUMER_SPARSE_CPU);
    c.sparse_factor_allowed = 4;
    c.hardware_decimation_available = true;
    assert(c5vrx_select_consumer(&c, &reason) == C5VRX_CONSUMER_LOWER_RATE_HARDWARE_TAP);
    return 0;
}
