#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "arc_phy.h"

typedef enum {
    ARC_ACQUIRE = 0,
    ARC_LOCK,
} arc_state_t;

typedef struct {
    arc_gain_table_t table;
    uint8_t gain;
    uint8_t survival_gain;
    arc_state_t state;
    unsigned settle;
    unsigned weak_ticks;
    unsigned hot_ticks;
    unsigned lost_ticks;
    unsigned gain_up_hold; /* Do not reverse overload protection immediately. */
} arc_controller_t;

typedef struct {
    bool sync;
    int sync_quality;
    int p_median;
    int q_phase;
    int clip_permille;
    int origin_permille;
    int winding_permille;
} arc_observation_t;

static inline void arc_controller_reset(arc_controller_t *arc,
                                        const arc_gain_table_t *table,
                                        uint8_t gain)
{
    *arc = (arc_controller_t){0};
    arc->table = *table;
    arc->survival_gain = arc_gain_highest_rf_stage_start(table);
    arc->gain = gain <= table->max_index ? gain : arc->survival_gain;
    arc->state = ARC_ACQUIRE;
    arc->settle = 10u;
}
static inline uint8_t arc_step_gain(const arc_controller_t *arc, int delta)
{
    int gain = (int)arc->gain + delta;
    if (gain < 2) gain = 2;
    if (gain > arc->table.max_index) gain = arc->table.max_index;
    return (uint8_t)gain;
}

static inline uint8_t arc_controller_tick(arc_controller_t *arc,
                                          const arc_observation_t *o)
{
    if (arc->gain_up_hold) --arc->gain_up_hold;
    bool clean = o->sync && o->sync_quality >= 70 && o->q_phase >= 65 &&
                 o->p_median >= 14 && o->p_median <= 34 &&
                 o->clip_permille <= 16 && o->origin_permille < 500 &&
                 o->winding_permille < 260;
    bool hot = o->clip_permille >= 32 || o->p_median > 36;
    bool weak = o->clip_permille <= 8 &&
                (o->p_median < 14 || o->origin_permille >= 550 ||
                 o->q_phase < 45);

    if (o->clip_permille >= 80) {
        arc->gain = arc_step_gain(arc, -4);
        arc->state = ARC_ACQUIRE;
        arc->settle = 10u;
        arc->gain_up_hold = 40u; /* Two seconds at the 20 Hz supervisor rate. */
        arc->hot_ticks = arc->weak_ticks = arc->lost_ticks = 0u;
        return arc->gain;
    }

    /* Front-end protection above must remain live during settling. All
     * ordinary decisions wait for the new hardware state to stabilize. */
    if (arc->settle) { --arc->settle; return arc->gain; }

    /* Absence of convincing video semantics is not evidence that more
     * downstream gain will recover a carrier. After a persistent loss, pin
     * the first entry of the highest RF stage when Q4 has spare headroom.
     * This prevents noisy Q4 phase scores from walking G61 toward G89. */
    if (!o->sync) {
        if (arc->lost_ticks < 10u) ++arc->lost_ticks;
        arc->weak_ticks = arc->hot_ticks = 0u;
        if (arc->lost_ticks >= 10u) {
            uint8_t next = arc->gain;
            /* Missing sync can also mean overload or a sparse sync window.
             * Never undo a clipping cut just because video detection failed.
             * Ordinary overload must still be reduced without valid sync. */
            if (hot) {
                next = arc_step_gain(arc, -1);
            } else if (arc->gain > arc->survival_gain ||
                       (arc->gain_up_hold == 0u &&
                        o->clip_permille <= 8 && o->p_median < 14 &&
                        o->origin_permille >= 550)) {
                next = arc->survival_gain;
            }
            if (arc->gain != next) {
                if (next < arc->gain) arc->gain_up_hold = 40u;
                arc->gain = next;
                arc->settle = 10u;
            }
            arc->state = ARC_ACQUIRE;
        }
        return arc->gain;
    }
    if (arc->state == ARC_LOCK) {
        if (clean) {
            arc->weak_ticks = arc->hot_ticks = arc->lost_ticks = 0u;
            return arc->gain; /* LOCK invariant: clean IQ causes no PHY writes. */
        }
        if (o->sync_quality < 40) {
            arc->weak_ticks = arc->hot_ticks = 0u;
            if (++arc->lost_ticks < 10u) return arc->gain;
        } else if (weak) {
            arc->lost_ticks = arc->hot_ticks = 0u;
            if (++arc->weak_ticks < 15u) return arc->gain;
        } else if (hot) {
            arc->lost_ticks = arc->weak_ticks = 0u;
            if (++arc->hot_ticks < 6u) return arc->gain;
        } else {
            arc->weak_ticks = arc->hot_ticks = arc->lost_ticks = 0u;
            return arc->gain;
        }
        arc->state = ARC_ACQUIRE;
        arc->weak_ticks = arc->hot_ticks = arc->lost_ticks = 0u;
    }
    arc->lost_ticks = 0u;

    if (clean) {
        arc->state = ARC_LOCK;
        return arc->gain;
    }

    uint8_t next = arc->gain;
    if (hot) {
        if (++arc->hot_ticks >= 2u) next = arc_step_gain(arc, -1);
        arc->weak_ticks = 0u;
    } else if (weak && o->q_phase >= 18 && arc->gain_up_hold == 0u) {
        if (++arc->weak_ticks >= 4u) next = arc_step_gain(arc, +1);
        arc->hot_ticks = 0u;
    } else {
        arc->weak_ticks = arc->hot_ticks = arc->lost_ticks = 0u;
    }

    if (next != arc->gain) {
        if (next < arc->gain) arc->gain_up_hold = 40u;
        arc->gain = next;
        arc->settle = 10u;
        arc->weak_ticks = arc->hot_ticks = arc->lost_ticks = 0u;
    }
    return arc->gain;
}
