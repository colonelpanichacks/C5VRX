/* SPDX-License-Identifier: GPL-3.0-only */
#include "c5vrx_av_clock.h"

#include <limits.h>

bool c5vrx_av_clock_choose(uint32_t samples_per_line_q16,
                           uint32_t line_rate_millihz,
                           const uint32_t *candidates, size_t count,
                           c5vrx_av_clock_choice_t *choice)
{
    if (!samples_per_line_q16 || !line_rate_millihz || !candidates ||
        !count || !choice) return false;
    const uint64_t requested =
        ((uint64_t)samples_per_line_q16 * line_rate_millihz + 32768000u) /
        65536000u;
    uint64_t best_error = UINT64_MAX;
    uint32_t selected = 0u;
    for (size_t i = 0; i < count; ++i) {
        if (!candidates[i]) continue;
        const uint64_t error = candidates[i] > requested ?
            candidates[i] - requested : requested - candidates[i];
        if (error < best_error) { best_error = error; selected = candidates[i]; }
    }
    if (!selected || requested > UINT32_MAX) return false;
    const int64_t delta = (int64_t)selected - (int64_t)requested;
    const int32_t ppm = (int32_t)(delta * 1000000ll / (int64_t)requested);
    *choice = (c5vrx_av_clock_choice_t) {
        .requested_hz = (uint32_t)requested,
        .selected_hz = selected,
        .residual_ppm = ppm,
        .bridge = (ppm >= -100 && ppm <= 100) ?
            C5VRX_CLOCK_BRIDGE_BYPASS : C5VRX_CLOCK_BRIDGE_FARROW_CUBIC,
    };
    return true;
}

void c5vrx_fractional_bridge_init(c5vrx_fractional_bridge_t *bridge,
                                  uint32_t source_hz, uint32_t output_hz,
                                  uint32_t initial_occupancy)
{
    if (!bridge) return;
    *bridge = (c5vrx_fractional_bridge_t) {0};
    if (source_hz && output_hz)
        bridge->step_q32 = ((uint64_t)source_hz << 32u) / output_hz;
    bridge->occupancy = initial_occupancy;
    bridge->occupancy_min = initial_occupancy;
    bridge->occupancy_max = initial_occupancy;
}

uint8_t c5vrx_fractional_bridge_sample(c5vrx_fractional_bridge_t *bridge,
                                       uint8_t xm1, uint8_t x0,
                                       uint8_t x1, uint8_t x2)
{
    if (!bridge) return x0;
    const int64_t t = (int64_t)(uint32_t)bridge->phase_q32;
    const int64_t a = -(int64_t)xm1 + 3ll * x0 - 3ll * x1 + x2;
    const int64_t b = 2ll * xm1 - 5ll * x0 + 4ll * x1 - x2;
    const int64_t c = -(int64_t)xm1 + x1;
    /* Catmull-Rom cubic in Q32, evaluated as a streaming four-tap Farrow
     * structure. It requires no line/frame buffer and is phase-continuous. */
    int64_t y = ((a * t >> 32) + b);
    y = ((y * t >> 32) + c);
    y = ((y * t >> 32) / 2ll) + x0;
    if (y < 0) y = 0;
    if (y > 63) y = 63;
    bridge->phase_q32 += bridge->step_q32;
    const uint32_t consumed = (uint32_t)(bridge->phase_q32 >> 32u);
    bridge->phase_q32 &= UINT32_MAX;
    if (consumed <= bridge->occupancy) bridge->occupancy -= consumed;
    else bridge->occupancy = 0u;
    if (bridge->occupancy < bridge->occupancy_min)
        bridge->occupancy_min = bridge->occupancy;
    if (bridge->occupancy > bridge->occupancy_max)
        bridge->occupancy_max = bridge->occupancy;
    ++bridge->samples_generated;
    return (uint8_t)y;
}

