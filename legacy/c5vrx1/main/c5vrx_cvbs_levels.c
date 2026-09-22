/* SPDX-License-Identifier: GPL-3.0-only */

#include "c5vrx_cvbs_levels.h"

#include <string.h>

#define TARGET_SYNC 0
#define TARGET_BLANK 19
#define TARGET_WHITE 63
#define ACTIVE_START_NUM 210u
#define NOMINAL_LINE_SAMPLES 1280u

static int clamp_code(int value, c5vrx_cvbs_levels_t *levels)
{
    if (value < 0) { ++levels->clamp_events; return 0; }
    if (value > 63) { ++levels->clamp_events; return 63; }
    return value;
}

static void slow_track(int32_t *estimate, int32_t sample_q16,
                       unsigned shift)
{
    *estimate += (sample_q16 - *estimate) >> shift;
}

void c5vrx_cvbs_levels_init(c5vrx_cvbs_levels_t *levels)
{
    if (!levels) return;
    memset(levels, 0, sizeof(*levels));
    levels->sync_q16 = 0 << 16;
    levels->blank_q16 = 19 << 16;
    levels->white_q16 = 63 << 16;
    for (unsigned i = 0; i < 64u; ++i) levels->map[i] = (uint8_t)i;
}

void c5vrx_cvbs_levels_reset(c5vrx_cvbs_levels_t *levels)
{
    c5vrx_cvbs_levels_init(levels);
}

void c5vrx_cvbs_levels_process(c5vrx_cvbs_levels_t *levels,
                               const c5vrx_cvbs_sync_tracker_t *timing,
                               uint8_t *samples, size_t count)
{
    if (!levels || !timing || !samples || !count ||
        !timing->horizontal_locked || !timing->line_period_samples ||
        !timing->last_hsync_start) return;

    const uint64_t start = timing->samples_seen;
    const uint32_t period = timing->line_period_samples;
    const uint32_t active_start =
        period * ACTIVE_START_NUM / NOMINAL_LINE_SAMPLES;
    const uint32_t sync_end = timing->last_hsync_width;
    const uint32_t porch_end = active_start > period / 32u ?
        active_start - period / 32u : active_start;
    const bool map_valid = levels->valid;
    const int32_t map_sync = levels->sync_q16;
    const int32_t map_blank = levels->blank_q16;
    const int32_t map_pedestal_span = levels->blank_q16 - levels->sync_q16;
    const int32_t map_active_span = levels->white_q16 - levels->blank_q16;
    if (map_valid) {
        for (unsigned code = 0; code < 64u; ++code) {
            const int32_t input_q16 = (int32_t)code << 16;
            int mapped;
            if (input_q16 <= map_blank) {
                mapped = TARGET_SYNC + (int)(((int64_t)(input_q16 -
                    map_sync) * (TARGET_BLANK - TARGET_SYNC)) /
                    map_pedestal_span);
            } else {
                mapped = TARGET_BLANK + (int)(((int64_t)(input_q16 -
                    map_blank) * (TARGET_WHITE - TARGET_BLANK)) /
                    map_active_span);
            }
            levels->map[code] = (uint8_t)clamp_code(mapped, levels);
        }
    }

    uint32_t phase = start >= timing->last_hsync_start ?
        (uint32_t)((start - timing->last_hsync_start) % period) : 0u;
    for (size_t i = 0; i < count; ++i) {
        const int32_t input_q16 = (int32_t)(samples[i] & 0x3fu) << 16;
        if (phase < sync_end) {
            slow_track(&levels->sync_q16, input_q16, 10u);
            ++levels->structured_samples;
        } else if (phase < porch_end) {
            slow_track(&levels->blank_q16, input_q16, 12u);
            ++levels->structured_samples;
        } else if (phase >= active_start) {
            /* Peak release is deliberately much slower than attack, so one
             * bright pixel cannot pump gain and a dark scene cannot collapse
             * the legal chroma/luma span. */
            const unsigned shift = input_q16 > levels->white_q16 ? 10u : 18u;
            slow_track(&levels->white_q16, input_q16, shift);
        }
        if (map_valid) samples[i] = levels->map[samples[i] & 0x3fu];
        if (++phase == period) phase = 0u;
    }

    const int32_t pedestal_span = levels->blank_q16 - levels->sync_q16;
    const int32_t active_span = levels->white_q16 - levels->blank_q16;
    levels->valid = levels->structured_samples >= 256u &&
        pedestal_span >= (3 << 16) && active_span >= (8 << 16);
}
