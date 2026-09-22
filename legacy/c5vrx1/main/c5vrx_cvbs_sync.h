/* SPDX-License-Identifier: GPL-3.0-only */

#pragma once

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define C5VRX_CVBS_SYNC_NO_LINE UINT16_MAX

typedef enum {
    C5VRX_VIDEO_STANDARD_UNKNOWN = 0,
    C5VRX_VIDEO_STANDARD_PAL,
    C5VRX_VIDEO_STANDARD_NTSC,
} c5vrx_video_standard_t;

typedef enum {
    C5VRX_SYNC_POLARITY_UNKNOWN = 0,
    C5VRX_SYNC_POLARITY_NEGATIVE,
    C5VRX_SYNC_POLARITY_POSITIVE,
} c5vrx_sync_polarity_t;

typedef struct {
    bool horizontal;
    bool vertical;
    bool locked;
    uint16_t field_line;
    uint32_t line_period_samples;
    uint64_t sample_index;
    uint64_t sync_start;
    uint64_t stream_epoch;
    uint64_t field_id;
    bool odd_field;
    c5vrx_video_standard_t standard;
    c5vrx_sync_polarity_t polarity;
} c5vrx_cvbs_sync_event_t;

typedef struct {
    uint64_t samples_seen;
    uint64_t pulse_start;
    uint64_t last_hsync_start;
    uint64_t last_vsync_start;
    uint64_t stream_epoch;
    uint64_t field_id;
    uint32_t line_period_samples;
    uint32_t line_period_q8;
    uint32_t pulse_samples;
    uint32_t horizontal_events;
    uint32_t vertical_events;
    uint32_t rejected_pulses;
    uint32_t lock_acquisitions;
    uint32_t lock_losses;
    uint32_t last_hsync_width;
    uint32_t last_vsync_width;
    uint16_t field_line;
    uint8_t sync_floor;
    uint8_t signal_peak;
    uint8_t horizontal_lock_score;
    uint8_t high_run;
    uint8_t polarity_votes;
    uint8_t pal_score;
    uint8_t ntsc_score;
    uint32_t sample_rate_hz;
    bool odd_field;
    bool in_sync;
    bool horizontal_locked;
    bool vertical_locked;
    c5vrx_video_standard_t standard;
    c5vrx_sync_polarity_t polarity;
} c5vrx_cvbs_sync_tracker_t;

void c5vrx_cvbs_sync_init(c5vrx_cvbs_sync_tracker_t *tracker);
void c5vrx_cvbs_sync_set_sample_rate(c5vrx_cvbs_sync_tracker_t *tracker,
                                     uint32_t sample_rate_hz);
void c5vrx_cvbs_sync_discontinuity(c5vrx_cvbs_sync_tracker_t *tracker);
bool c5vrx_cvbs_sync_consume(c5vrx_cvbs_sync_tracker_t *tracker,
                             uint8_t sample,
                             c5vrx_cvbs_sync_event_t *event);
bool c5vrx_cvbs_sync_consume_stride(c5vrx_cvbs_sync_tracker_t *tracker,
                                    uint8_t sample, uint8_t stride,
                                    c5vrx_cvbs_sync_event_t *event);
uint8_t c5vrx_cvbs_sync_threshold(const c5vrx_cvbs_sync_tracker_t *tracker);

#ifdef __cplusplus
}
#endif
