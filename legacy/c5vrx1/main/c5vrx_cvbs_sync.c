/* SPDX-License-Identifier: GPL-3.0-only */

#include "c5vrx_cvbs_sync.h"

#include <stddef.h>
#include <string.h>

#define NOMINAL_LINE_SAMPLES 1280u
#define HORIZONTAL_LOCK_EVENTS 3u
#define MAX_FIELD_LINES 340u
#define VERTICAL_CLUSTER_LINES 4u
#define HORIZONTAL_TIMEOUT_LINES 3u
#define DEFAULT_SAMPLE_RATE_HZ 20000000u
#define STANDARD_LOCK_SCORE 8u

static void classify_standard(c5vrx_cvbs_sync_tracker_t *tracker)
{
    if (!tracker->line_period_samples || !tracker->sample_rate_hz) return;
    const uint32_t rate = tracker->sample_rate_hz / tracker->line_period_samples;
    const bool pal = rate >= 15550u && rate <= 15700u;
    const bool ntsc = rate >= 15680u && rate <= 15800u;
    if (pal && !ntsc) {
        if (tracker->pal_score < STANDARD_LOCK_SCORE) ++tracker->pal_score;
        if (tracker->ntsc_score) --tracker->ntsc_score;
    } else if (ntsc && !pal) {
        if (tracker->ntsc_score < STANDARD_LOCK_SCORE) ++tracker->ntsc_score;
        if (tracker->pal_score) --tracker->pal_score;
    }
    if (tracker->pal_score >= STANDARD_LOCK_SCORE)
        tracker->standard = C5VRX_VIDEO_STANDARD_PAL;
    else if (tracker->ntsc_score >= STANDARD_LOCK_SCORE)
        tracker->standard = C5VRX_VIDEO_STANDARD_NTSC;
}

static void lose_horizontal_lock(c5vrx_cvbs_sync_tracker_t *tracker)
{
    if (tracker->horizontal_locked) ++tracker->lock_losses;
    tracker->horizontal_locked = false;
    tracker->horizontal_lock_score = 0u;
}

void c5vrx_cvbs_sync_init(c5vrx_cvbs_sync_tracker_t *tracker)
{
    if (!tracker) return;
    memset(tracker, 0, sizeof(*tracker));
    tracker->line_period_samples = NOMINAL_LINE_SAMPLES;
    tracker->line_period_q8 = NOMINAL_LINE_SAMPLES << 8u;
    tracker->sync_floor = 8u;
    tracker->signal_peak = 24u;
    tracker->field_line = C5VRX_CVBS_SYNC_NO_LINE;
    tracker->sample_rate_hz = DEFAULT_SAMPLE_RATE_HZ;
}

void c5vrx_cvbs_sync_set_sample_rate(c5vrx_cvbs_sync_tracker_t *tracker,
                                     uint32_t sample_rate_hz)
{
    if (!tracker || !sample_rate_hz) return;
    tracker->sample_rate_hz = sample_rate_hz;
    if (!tracker->horizontal_events) {
        tracker->line_period_samples =
            (sample_rate_hz + 7812u) / 15625u;
        tracker->line_period_q8 = tracker->line_period_samples << 8u;
    }
}

void c5vrx_cvbs_sync_discontinuity(c5vrx_cvbs_sync_tracker_t *tracker)
{
    if (!tracker) return;
    const uint64_t epoch = tracker->stream_epoch + 1u;
    const uint32_t rate = tracker->sample_rate_hz;
    c5vrx_cvbs_sync_init(tracker);
    tracker->stream_epoch = epoch;
    tracker->sample_rate_hz = rate ? rate : DEFAULT_SAMPLE_RATE_HZ;
}

uint8_t c5vrx_cvbs_sync_threshold(const c5vrx_cvbs_sync_tracker_t *tracker)
{
    if (!tracker) return 8u;
    const unsigned span = tracker->signal_peak > tracker->sync_floor ?
        (unsigned)tracker->signal_peak - (unsigned)tracker->sync_floor : 8u;
    unsigned threshold = tracker->sync_floor + span / 4u;
    if (threshold < 4u) threshold = 4u;
    if (threshold > 14u) threshold = 14u;
    return (uint8_t)threshold;
}

static void update_envelope(c5vrx_cvbs_sync_tracker_t *tracker, uint8_t sample)
{
    if (sample < tracker->sync_floor) tracker->sync_floor = sample;
    if (sample > tracker->signal_peak) tracker->signal_peak = sample;

    /* Slow release lets the threshold follow level drift without following
     * individual dirty-video samples or lifting into the black pedestal. */
    if ((tracker->samples_seen & 1023u) == 0u) {
        if (tracker->sync_floor < 8u) ++tracker->sync_floor;
        if (tracker->signal_peak > 20u) --tracker->signal_peak;
    }
}

static bool finish_pulse(c5vrx_cvbs_sync_tracker_t *tracker,
                         c5vrx_cvbs_sync_event_t *event)
{
    const uint32_t width = tracker->pulse_samples - tracker->high_run;
    const uint64_t pulse_start = tracker->pulse_start;
    tracker->in_sync = false;
    tracker->pulse_samples = 0;
    tracker->high_run = 0;

    const uint32_t rate = tracker->sample_rate_hz ?
        tracker->sample_rate_hz : DEFAULT_SAMPLE_RATE_HZ;
    const uint32_t min_hsync = rate * 30u / 10000000u;
    const uint32_t max_hsync = rate * 65u / 10000000u;
    const uint32_t min_broad = rate * 20u / 1000000u;
    const uint32_t max_broad = rate * 35u / 1000000u;
    const uint32_t nominal_line = rate / 15625u;
    const uint32_t min_line = nominal_line * 85u / 100u;
    const uint32_t max_line = nominal_line * 115u / 100u;

    if (width >= min_broad && width <= max_broad) {
        /* Equalising/broad-sync sequences contain several pulses per field.
         * Count and emit the cluster once, not once per constituent pulse. */
        const bool same_cluster = tracker->vertical_events != 0u &&
            pulse_start - tracker->last_vsync_start <
                (uint64_t)VERTICAL_CLUSTER_LINES * tracker->line_period_samples;
        if (same_cluster) return false;
        ++tracker->vertical_events;
        ++tracker->field_id;
        if (tracker->last_hsync_start && tracker->line_period_samples) {
            const uint32_t phase = (uint32_t)((pulse_start -
                tracker->last_hsync_start) % tracker->line_period_samples);
            tracker->odd_field = phase > tracker->line_period_samples / 4u &&
                phase < tracker->line_period_samples * 3u / 4u;
        } else {
            tracker->odd_field = (tracker->field_id & 1u) != 0u;
        }
        tracker->last_vsync_start = pulse_start;
        tracker->last_vsync_width = width;
        tracker->vertical_locked = true;
        tracker->field_line = 0u;
        tracker->last_hsync_start = 0u;
        lose_horizontal_lock(tracker);
        event->vertical = true;
        event->sync_start = pulse_start;
        event->field_line = 0u;
        event->stream_epoch = tracker->stream_epoch;
        event->field_id = tracker->field_id;
        event->odd_field = tracker->odd_field;
        event->standard = tracker->standard;
        event->polarity = tracker->polarity;
        return true;
    }
    if (width < min_hsync || width > max_hsync) {
        ++tracker->rejected_pulses;
        return false;
    }
    ++tracker->horizontal_events;
    if (tracker->polarity_votes < 8u) ++tracker->polarity_votes;
    if (tracker->polarity_votes >= 8u)
        tracker->polarity = C5VRX_SYNC_POLARITY_NEGATIVE;
    tracker->last_hsync_width = width;

    if (tracker->last_hsync_start) {
        const uint64_t interval64 = pulse_start - tracker->last_hsync_start;
        if (interval64 >= min_line && interval64 <= max_line) {
            const uint32_t interval = (uint32_t)interval64;
            tracker->line_period_q8 =
                (uint32_t)(((uint64_t)tracker->line_period_q8 * 7u +
                            ((uint64_t)interval << 8u) + 4u) / 8u);
            tracker->line_period_samples =
                (tracker->line_period_q8 + 128u) >> 8u;
            classify_standard(tracker);
            if (tracker->horizontal_lock_score < HORIZONTAL_LOCK_EVENTS)
                ++tracker->horizontal_lock_score;
        } else if (tracker->horizontal_lock_score) {
            --tracker->horizontal_lock_score;
        }
    }
    if (!tracker->horizontal_locked &&
        tracker->horizontal_lock_score >= HORIZONTAL_LOCK_EVENTS) {
        tracker->horizontal_locked = true;
        ++tracker->lock_acquisitions;
    } else if (tracker->horizontal_locked &&
               tracker->horizontal_lock_score < HORIZONTAL_LOCK_EVENTS) {
        lose_horizontal_lock(tracker);
    }
    tracker->last_hsync_start = pulse_start;

    event->horizontal = true;
    event->sync_start = pulse_start;
    event->line_period_samples = tracker->line_period_samples;
    event->field_line = tracker->vertical_locked ? tracker->field_line :
        C5VRX_CVBS_SYNC_NO_LINE;
    event->locked = tracker->vertical_locked && tracker->horizontal_locked;
    event->stream_epoch = tracker->stream_epoch;
    event->field_id = tracker->field_id;
    event->odd_field = tracker->odd_field;
    event->standard = tracker->standard;
    event->polarity = tracker->polarity;

    if (tracker->vertical_locked) {
        ++tracker->field_line;
        if (tracker->field_line > MAX_FIELD_LINES) {
            tracker->vertical_locked = false;
            tracker->field_line = C5VRX_CVBS_SYNC_NO_LINE;
            lose_horizontal_lock(tracker);
            event->locked = false;
        }
    }
    return true;
}

bool c5vrx_cvbs_sync_consume(c5vrx_cvbs_sync_tracker_t *tracker,
                             uint8_t sample,
                             c5vrx_cvbs_sync_event_t *event)
{
    return c5vrx_cvbs_sync_consume_stride(tracker, sample, 1u, event);
}

bool c5vrx_cvbs_sync_consume_stride(c5vrx_cvbs_sync_tracker_t *tracker,
                                    uint8_t sample, uint8_t stride,
                                    c5vrx_cvbs_sync_event_t *event)
{
    if (!tracker || !event || !stride) return false;
    memset(event, 0, sizeof(*event));
    event->field_line = C5VRX_CVBS_SYNC_NO_LINE;
    event->sample_index = tracker->samples_seen;
    sample &= 0x3fu;
    update_envelope(tracker, sample);
    const uint8_t threshold = c5vrx_cvbs_sync_threshold(tracker);

    if (tracker->horizontal_locked && tracker->last_hsync_start &&
        tracker->samples_seen - tracker->last_hsync_start >
            (uint64_t)HORIZONTAL_TIMEOUT_LINES * tracker->line_period_samples) {
        tracker->vertical_locked = false;
        tracker->field_line = C5VRX_CVBS_SYNC_NO_LINE;
        lose_horizontal_lock(tracker);
    }

    bool emitted = false;
    if (!tracker->in_sync) {
        if (sample <= threshold) {
            tracker->in_sync = true;
            tracker->pulse_start = tracker->samples_seen;
            tracker->pulse_samples = stride;
            tracker->high_run = 0u;
        }
    } else {
        tracker->pulse_samples += stride;
        if (sample > (uint8_t)(threshold + 2u)) {
            tracker->high_run = (uint8_t)(tracker->high_run + stride);
            if (tracker->high_run >= 3u * stride)
                emitted = finish_pulse(tracker, event);
        } else {
            tracker->high_run = 0u;
        }
    }
    tracker->samples_seen += stride;
    return emitted;
}
