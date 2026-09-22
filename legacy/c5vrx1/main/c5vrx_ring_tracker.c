/* SPDX-License-Identifier: GPL-3.0-only */

#include "c5vrx_ring_tracker.h"

#include <limits.h>
#include <stddef.h>
#include <string.h>

static bool power_of_two(uint32_t value)
{
    return value && !(value & (value - 1u));
}

static uint32_t deadline_for_lag(const c5vrx_ring_tracker_t *t,
                                 uint64_t lag)
{
    const uint64_t safe_capacity = t->capacity_words - t->guard_words;
    if (lag >= safe_capacity) return 0u;
    const uint64_t free_words = safe_capacity - lag;
    const uint64_t cycles = free_words * t->counter_hz / t->maximum_rate_hz;
    return cycles > UINT32_MAX ? UINT32_MAX : (uint32_t)cycles;
}

int c5vrx_ring_tracker_init(c5vrx_ring_tracker_t *t,
                            uint32_t capacity_words,
                            uint32_t guard_words,
                            uint32_t counter_hz,
                            uint32_t maximum_rate_hz)
{
    if (!t || !power_of_two(capacity_words) || guard_words == 0u ||
        guard_words >= capacity_words / 2u || !counter_hz ||
        !maximum_rate_hz) {
        return -1;
    }
    memset(t, 0, sizeof(*t));
    t->capacity_words = capacity_words;
    t->guard_words = guard_words;
    t->counter_hz = counter_hz;
    t->maximum_rate_hz = maximum_rate_hz;
    t->minimum_deadline_cycles = UINT32_MAX;
    return 0;
}

c5vrx_ring_track_result_t c5vrx_ring_tracker_observe(
    c5vrx_ring_tracker_t *t, uint32_t pointer, uint32_t counter)
{
    if (!t || pointer >= t->capacity_words)
        return C5VRX_RING_TRACK_POINTER_OUT_OF_RANGE;
    if (!t->initialized) {
        /* One virtual wrap permits a half-ring initial reader lag without
         * unsigned underflow. Absolute zero is intentionally arbitrary. */
        t->producer_absolute = (uint64_t)t->capacity_words + pointer;
        t->consumer_absolute = t->producer_absolute;
        t->last_pointer = pointer;
        t->last_counter = counter;
        t->initialized = true;
        t->observations = 1u;
        return C5VRX_RING_TRACK_OK;
    }

    const uint32_t elapsed = counter - t->last_counter;
    if (elapsed > t->maximum_service_interval_cycles)
        t->maximum_service_interval_cycles = elapsed;
    /* Round the rate bound upward and allow a tiny MMIO observation skew.
     * The bound is used in both directions: a whole possible wrap makes the
     * interval ambiguous, while a modulo delta above the bound means the
     * pointer/rate observation itself is not credible. */
    const uint64_t possible_advance =
        ((uint64_t)elapsed * t->maximum_rate_hz + t->counter_hz - 1u) /
        t->counter_hz + 8u;
    if (possible_advance >= t->capacity_words) {
        ++t->ambiguous_intervals;
        return C5VRX_RING_TRACK_INTERVAL_AMBIGUOUS;
    }

    const uint32_t mask = t->capacity_words - 1u;
    const uint32_t advance = (pointer - t->last_pointer) & mask;
    if (advance > possible_advance) {
        ++t->ambiguous_intervals;
        return C5VRX_RING_TRACK_INTERVAL_AMBIGUOUS;
    }
    if (pointer < t->last_pointer) ++t->wraps;
    t->producer_absolute += advance;
    t->last_pointer = pointer;
    t->last_counter = counter;
    ++t->observations;

    const uint64_t lag = c5vrx_ring_tracker_lag(t);
    if (lag > t->maximum_lag_words) t->maximum_lag_words = lag;
    const uint32_t deadline = deadline_for_lag(t, lag);
    if (deadline < t->minimum_deadline_cycles)
        t->minimum_deadline_cycles = deadline;
    if (!deadline) return C5VRX_RING_TRACK_GUARD_VIOLATION;
    return C5VRX_RING_TRACK_OK;
}

c5vrx_ring_track_result_t c5vrx_ring_tracker_place_consumer(
    c5vrx_ring_tracker_t *t, uint32_t lag_words, uint32_t alignment)
{
    if (!t || !t->initialized || !alignment ||
        (alignment & (alignment - 1u)) || lag_words >= t->capacity_words ||
        lag_words >= t->capacity_words - t->guard_words)
        return C5VRX_RING_TRACK_GUARD_VIOLATION;
    uint64_t consumer = t->producer_absolute - lag_words;
    consumer &= ~((uint64_t)alignment - 1u);
    t->consumer_absolute = consumer;
    const uint64_t lag = c5vrx_ring_tracker_lag(t);
    t->maximum_lag_words = lag;
    t->minimum_deadline_cycles = deadline_for_lag(t, lag);
    return t->minimum_deadline_cycles ? C5VRX_RING_TRACK_OK :
                                        C5VRX_RING_TRACK_GUARD_VIOLATION;
}

c5vrx_ring_track_result_t c5vrx_ring_tracker_consume(
    c5vrx_ring_tracker_t *t, uint32_t words)
{
    if (!t || !t->initialized || words > c5vrx_ring_tracker_lag(t))
        return C5VRX_RING_TRACK_CONSUME_PAST_PRODUCER;
    t->consumer_absolute += words;
    return C5VRX_RING_TRACK_OK;
}

uint64_t c5vrx_ring_tracker_lag(const c5vrx_ring_tracker_t *t)
{
    if (!t || !t->initialized || t->producer_absolute < t->consumer_absolute)
        return 0u;
    return t->producer_absolute - t->consumer_absolute;
}

uint32_t c5vrx_ring_tracker_deadline_cycles(const c5vrx_ring_tracker_t *t)
{
    return t && t->initialized ? deadline_for_lag(t, c5vrx_ring_tracker_lag(t)) : 0u;
}

const char *c5vrx_ring_track_result_name(c5vrx_ring_track_result_t result)
{
    switch (result) {
        case C5VRX_RING_TRACK_OK: return "OK";
        case C5VRX_RING_TRACK_POINTER_OUT_OF_RANGE: return "POINTER_OUT_OF_RANGE";
        case C5VRX_RING_TRACK_INTERVAL_AMBIGUOUS: return "INTERVAL_AMBIGUOUS";
        case C5VRX_RING_TRACK_GUARD_VIOLATION: return "GUARD_VIOLATION";
        case C5VRX_RING_TRACK_CONSUME_PAST_PRODUCER:
            return "CONSUME_PAST_PRODUCER";
        default: return "UNKNOWN";
    }
}
