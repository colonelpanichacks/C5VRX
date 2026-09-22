/* SPDX-License-Identifier: GPL-3.0-only */
#include <assert.h>
#include <stdio.h>
#include "c5vrx_cvbs_levels.h"

int main(void)
{
    c5vrx_cvbs_levels_t levels;
    c5vrx_cvbs_levels_init(&levels);
    c5vrx_cvbs_sync_tracker_t timing = {0};
    timing.horizontal_locked = true;
    timing.line_period_samples = 1280u;
    timing.last_hsync_start = 1u;
    timing.last_hsync_width = 94u;
    timing.samples_seen = 1u;
    uint8_t line[1280];
    for (unsigned pass = 0; pass < 64u; ++pass) {
        for (unsigned i = 0; i < 1280u; ++i)
            line[i] = i < 94u ? 5u : i < 210u ? 22u :
                (uint8_t)(22u + (i % 37u));
        c5vrx_cvbs_levels_process(&levels, &timing, line, 1280u);
        timing.samples_seen += 1280u;
    }
    assert(levels.valid);
    assert((levels.sync_q16 >> 16) < (levels.blank_q16 >> 16));
    assert((levels.white_q16 >> 16) > (levels.blank_q16 >> 16));
    puts("cvbs_levels_test: PASS");
    return 0;
}

