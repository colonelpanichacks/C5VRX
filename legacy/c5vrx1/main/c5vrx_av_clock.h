/* SPDX-License-Identifier: GPL-3.0-only */
#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

typedef enum {
    C5VRX_CLOCK_BRIDGE_BYPASS = 0,
    C5VRX_CLOCK_BRIDGE_FARROW_CUBIC,
} c5vrx_clock_bridge_mode_t;

typedef struct {
    uint32_t requested_hz;
    uint32_t selected_hz;
    int32_t residual_ppm;
    c5vrx_clock_bridge_mode_t bridge;
} c5vrx_av_clock_choice_t;

typedef struct {
    uint64_t phase_q32;
    uint64_t step_q32;
    uint32_t occupancy;
    uint32_t occupancy_min;
    uint32_t occupancy_max;
    uint64_t samples_generated;
} c5vrx_fractional_bridge_t;

bool c5vrx_av_clock_choose(uint32_t source_samples_per_line_q16,
                           uint32_t line_rate_millihz,
                           const uint32_t *candidates, size_t candidate_count,
                           c5vrx_av_clock_choice_t *choice);
void c5vrx_fractional_bridge_init(c5vrx_fractional_bridge_t *bridge,
                                  uint32_t source_hz, uint32_t output_hz,
                                  uint32_t initial_occupancy);
uint8_t c5vrx_fractional_bridge_sample(c5vrx_fractional_bridge_t *bridge,
                                       uint8_t xm1, uint8_t x0,
                                       uint8_t x1, uint8_t x2);

