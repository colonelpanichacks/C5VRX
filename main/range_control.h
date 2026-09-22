#pragma once
#include <stdbool.h>
#include <stdint.h>
#include "demod_quality.h"

/* 20 Hz supervisory controller; never paces or buffers live video.
 * Scores are relative heuristics, not calibrated SNR or measured dBm.
 *
 * OUI-SPY: optimize_allowed is the legacy-lock veto. The confirmed legacy
 * lock detector (envelope/modulation votes + 500 ms sustain, the authority
 * that drives [CARRIER] and the buzzer) passes false while it holds a lock;
 * the controller then keeps scoring/observing (locked state, counters,
 * quality sums) but every gain-changing path — clip emergency, clip cut,
 * trial start, trial rollback, no-video walk — is suppressed and the
 * current gain is returned unchanged. */
typedef struct {
    unsigned samples, syncs, hold, age, cooldown, failures, no_video;
    int quality_sum, power_sum, clip_sum, winding_sum, sync_quality_sum;
    int baseline;
    uint8_t gain, previous;
    bool trial, locked, reverse;
} range_control_t;

static inline void range_control_reset(range_control_t *c, uint8_t gain)
{
    *c = (range_control_t){.gain = gain, .hold = 10};
}

static inline uint8_t range_control_move(range_control_t *c, int gain)
{
    if (gain < 2) gain = 2;
    if (gain > 62) gain = 62;
    if (gain != c->gain) {
        c->gain = (uint8_t)gain;
        c->hold = 10;
        c->age = c->samples = c->syncs = 0;
        c->quality_sum = c->power_sum = c->clip_sum = 0;
        c->winding_sum = c->sync_quality_sum = 0;
        c->locked = false;
    }
    return c->gain;
}

static inline uint8_t range_control_tick(range_control_t *c, bool sync,
                                         int sync_quality, int power,
                                         int coherence, int clip, int origin,
                                         int winding_permille,
                                         bool optimize_allowed)
{
    ++c->age;
    if (c->cooldown) --c->cooldown;
    /* Strong evidence of overload takes priority over learned preferences.
     * Allow 100 ms after a write before another emergency reduction. */
    if (optimize_allowed && clip >= 80 && c->age >= 2 && c->gain > 2) {
        c->trial = false;
        c->no_video = 0;
        return range_control_move(c, c->gain - 4);
    }
    if (c->hold) { --c->hold; return c->gain; }
    ++c->samples;
    c->syncs += sync;
    if (sync) c->sync_quality_sum += sync_quality;
    c->power_sum += power;
    c->clip_sum += clip;
    c->winding_sum += winding_permille;
    /* Winding loss is a direct measure of how often the current 50 ns
     * endpoint discriminator disagrees with the two true adjacent 25 ns
     * phase steps.  It therefore penalizes "large but snowy" gain states. */
    c->quality_sum += coherence - clip / 2 - origin / 20 -
                      demod_winding_penalty(winding_permille) +
                      (sync ? sync_quality / 5 : 0);
    if (c->samples < 10) return c->gain;

    int p = c->power_sum / 10;
    int clipping = c->clip_sum / 10;
    int winding = c->winding_sum / 10;
    int sync_quality_avg = c->syncs ?
                           c->sync_quality_sum / (int)c->syncs : 0;
    int score = c->quality_sum / 10 + (int)c->syncs * 20;
    bool video = c->syncs >= 2 && sync_quality_avg >= 70;
    c->locked = video && clipping < 20 && !demod_static_heavy(winding);
    c->samples = c->syncs = 0;
    c->power_sum = c->clip_sum = c->quality_sum = 0;
    c->winding_sum = c->sync_quality_sum = 0;
    c->no_video = video ? 0 : c->no_video + 1;

    if (!optimize_allowed) {
        /* Legacy lock holds the veto: observation only. Cancel any trial
         * whose baseline was measured before the lock so no rollback to a
         * stale 'previous' can fire when the veto lifts. */
        c->trial = false;
        return c->gain;
    }
    if (clipping >= 20 && c->gain > 2) {
        c->trial = false;
        return range_control_move(c, c->gain - 2);
    }
    if (c->trial) {
        c->trial = false;
        /* Require a margin: random window variation is not an improvement. */
        if (score < c->baseline + 20) {
            if (c->failures < 4) ++c->failures;
            c->cooldown = 40u << c->failures; /* 4..32 seconds */
            c->reverse = !c->reverse;
            return range_control_move(c, c->previous);
        }
        c->failures = 0;
        c->cooldown = 40;
        return c->gain;
    }
    /* Stable, low-ambiguity video with sufficient headroom: absolutely no
     * optimization writes.  If sync remains valid but endpoint winding loss
     * is very high, allow one normal +/-2 trial and keep the existing
     * rollback/backoff machinery. */
    if (video && p >= 14 && p <= 36 && !demod_static_heavy(winding))
        return c->gain;
    /* Absence of video never parks permanently at one high-gain state.
     * Acquisition is separate from trials; no bad baseline to restore. */
    if (c->no_video >= 4) {
        c->no_video = 0;
        return range_control_move(c, c->gain <= 8 ? 62 : c->gain - 8);
    }
    if (!video || c->cooldown) return c->gain;
    int direction = p < 14 ? 2 : -2;
    if (c->reverse) direction = -direction;
    int candidate = c->gain + direction;
    if (candidate < 2 || candidate > 62) return c->gain;
    c->previous = c->gain;
    c->baseline = score;
    c->trial = true;
    return range_control_move(c, candidate);
}
