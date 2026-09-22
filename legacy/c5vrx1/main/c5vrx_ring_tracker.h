/* SPDX-License-Identifier: GPL-3.0-only */

#pragma once

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    C5VRX_RING_TRACK_OK = 0,
    C5VRX_RING_TRACK_POINTER_OUT_OF_RANGE,
    C5VRX_RING_TRACK_INTERVAL_AMBIGUOUS,
    C5VRX_RING_TRACK_GUARD_VIOLATION,
    C5VRX_RING_TRACK_CONSUME_PAST_PRODUCER,
} c5vrx_ring_track_result_t;

typedef struct {
    uint32_t capacity_words;
    uint32_t guard_words;
    uint32_t counter_hz;
    uint32_t maximum_rate_hz;
    uint32_t last_counter;
    uint32_t last_pointer;
    uint64_t producer_absolute;
    uint64_t consumer_absolute;
    uint64_t wraps;
    uint64_t observations;
    uint64_t ambiguous_intervals;
    uint64_t maximum_lag_words;
    uint32_t maximum_service_interval_cycles;
    uint32_t minimum_deadline_cycles;
    bool initialized;
} c5vrx_ring_tracker_t;

int c5vrx_ring_tracker_init(c5vrx_ring_tracker_t *tracker,
                            uint32_t capacity_words,
                            uint32_t guard_words,
                            uint32_t counter_hz,
                            uint32_t maximum_rate_hz);

/* Extend a wrapping hardware pointer into one monotonic producer timeline.
 * The maximum-rate bound makes an observation invalid whenever one complete
 * ring could have elapsed between reads; modulo arithmetic is never used to
 * hide an ambiguous interval. */
c5vrx_ring_track_result_t c5vrx_ring_tracker_observe(
    c5vrx_ring_tracker_t *tracker, uint32_t pointer, uint32_t counter);

/* Start the reader a fixed distance behind the current producer. */
c5vrx_ring_track_result_t c5vrx_ring_tracker_place_consumer(
    c5vrx_ring_tracker_t *tracker, uint32_t lag_words, uint32_t alignment);

c5vrx_ring_track_result_t c5vrx_ring_tracker_consume(
    c5vrx_ring_tracker_t *tracker, uint32_t words);

uint64_t c5vrx_ring_tracker_lag(const c5vrx_ring_tracker_t *tracker);
uint32_t c5vrx_ring_tracker_deadline_cycles(
    const c5vrx_ring_tracker_t *tracker);
const char *c5vrx_ring_track_result_name(c5vrx_ring_track_result_t result);

#ifdef __cplusplus
}
#endif
