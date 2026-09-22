/* SPDX-License-Identifier: GPL-3.0-only */

#pragma once

#include <stddef.h>
#include <stdint.h>
#include "esp_err.h"
#include "c5vrx_stream.h"
#include "c5vrx_rf_dump_producer.h"
#include "c5vrx_capabilities.h"
#include "c5vrx_cvbs_sync.h"
#include "c5vrx_wbfm_hw.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef esp_err_t (*c5vrx_cvbs_sink_fn)(const uint8_t *samples,
                                        size_t count,
                                        void *context);

typedef enum {
    C5VRX_RING_FAILURE_NONE = 0,
    C5VRX_RING_FAILURE_PRODUCER_STOPPED,
    C5VRX_RING_FAILURE_SERVICE_INTERVAL_AMBIGUOUS,
    C5VRX_RING_FAILURE_READER_INSIDE_GUARD,
    C5VRX_RING_FAILURE_COPY_AMBIGUOUS,
    C5VRX_RING_FAILURE_POINTER_OUT_OF_RANGE,
    C5VRX_RING_FAILURE_ADJACENT_MEMORY_CORRUPTED,
} c5vrx_live_ring_failure_t;

typedef struct {
    c5vrx_rf_source_t *source;
    c5vrx_cvbs_sink_fn sink;
    void *sink_context;
    c5vrx_cvbs_conditioner_config_t conditioner;
    c5vrx_wbfm_kernel_t wbfm_kernel;
    size_t maximum_input_words;
} c5vrx_live_pipeline_config_t;

esp_err_t c5vrx_live_pipeline_start(const c5vrx_live_pipeline_config_t *config);
esp_err_t c5vrx_live_pipeline_stop(void);
bool c5vrx_live_pipeline_running(void);
void c5vrx_live_pipeline_get_stats(c5vrx_stream_stats_t *stats);
void c5vrx_live_pipeline_get_timing(c5vrx_cvbs_sync_tracker_t *timing);
void c5vrx_live_pipeline_log_stats(void);

/* Explicitly experimental adapter over repeated finite vendor dump triggers. */
esp_err_t c5vrx_finite_chain_source_create(c5vrx_rf_source_t *source,
                                           size_t words_per_block);
void c5vrx_finite_chain_source_destroy(c5vrx_rf_source_t *source);

typedef struct {
    uint64_t blocks;
    uint64_t words;
    uint64_t dropped_blocks;
    uint64_t missed_words;
    uint64_t overruns;
    uint64_t discontinuities;
    uint64_t wraps_observed;
    uint64_t fatal_stops;
    uint64_t copy_cycles_total;
    uint64_t producer_absolute;
    uint64_t consumer_absolute;
    uint64_t lag_words;
    uint64_t maximum_lag_words;
    uint64_t stream_epoch;
    c5vrx_live_ring_failure_t fatal_reason;
    uint32_t maximum_copy_cycles;
    uint32_t maximum_service_interval_cycles;
    uint32_t minimum_service_deadline_cycles;
    uint32_t deadline_headroom_x1000;
    uint16_t reader_pointer;
    uint16_t writer_pointer;
    uint16_t guard_words;
} c5vrx_live_ring_stats_t;

/**
 * Configure the vendor-observed producer once and expose guarded, copied,
 * immutable contiguous windows. This remains explicitly UNPROVEN until real
 * hardware establishes continuous operation and phase continuity at wrap.
 * The producer is armed just-in-time by the first acquire, after all buffers
 * are allocated; setup work can therefore never consume an unobserved ring.
 */
esp_err_t c5vrx_live_ring_source_create(c5vrx_rf_source_t *source,
                                        c5vrx_rf_dump_mode_t mode,
                                        size_t maximum_words_per_block,
                                        size_t guard_words,
                                        uint32_t maximum_plausible_rate_hz);
void c5vrx_live_ring_source_get_stats(const c5vrx_rf_source_t *source,
                                      c5vrx_live_ring_stats_t *stats);
const char *c5vrx_live_ring_failure_name(c5vrx_live_ring_failure_t failure);
esp_err_t c5vrx_live_ring_source_destroy(c5vrx_rf_source_t *source);

#ifdef __cplusplus
}
#endif
