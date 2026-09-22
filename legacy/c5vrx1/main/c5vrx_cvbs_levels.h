/* SPDX-License-Identifier: GPL-3.0-only */
#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "c5vrx_cvbs_sync.h"

typedef struct {
    int32_t sync_q16;
    int32_t blank_q16;
    int32_t white_q16;
    uint64_t structured_samples;
    uint32_t clamp_events;
    uint8_t map[64];
    bool valid;
} c5vrx_cvbs_levels_t;

void c5vrx_cvbs_levels_init(c5vrx_cvbs_levels_t *levels);
void c5vrx_cvbs_levels_reset(c5vrx_cvbs_levels_t *levels);
void c5vrx_cvbs_levels_process(c5vrx_cvbs_levels_t *levels,
                               const c5vrx_cvbs_sync_tracker_t *timing,
                               uint8_t *samples, size_t count);
