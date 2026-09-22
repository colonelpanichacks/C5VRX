/* SPDX-License-Identifier: GPL-3.0-only */
#include <assert.h>
#include <stdio.h>
#include "c5vrx_av_clock.h"

int main(void)
{
    const uint32_t clocks[] = {16000000u, 20000000u, 24000000u, 26666667u};
    c5vrx_av_clock_choice_t c;
    assert(c5vrx_av_clock_choose(1280u << 16, 15625000u,
                                 clocks, 4u, &c));
    assert(c.requested_hz == 20000000u);
    assert(c.selected_hz == 20000000u);
    assert(c.bridge == C5VRX_CLOCK_BRIDGE_BYPASS);
    c5vrx_fractional_bridge_t b;
    c5vrx_fractional_bridge_init(&b, 20001000u, 20000000u, 8u);
    const uint8_t y = c5vrx_fractional_bridge_sample(&b, 10u, 20u, 30u, 40u);
    assert(y >= 19u && y <= 31u);
    puts("av_clock_test: PASS");
    return 0;
}

