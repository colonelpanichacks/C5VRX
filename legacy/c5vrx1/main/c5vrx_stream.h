/* SPDX-License-Identifier: GPL-3.0-only */

#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdatomic.h>

#ifdef __cplusplus
extern "C" {
#endif

#define C5VRX_RF_BLOCK_QUEUE_CAPACITY 4u

typedef enum {
    C5VRX_RF_SOURCE_FINITE_CHAINED = 0,
    C5VRX_RF_SOURCE_EXPERIMENTAL_RING_UNPROVEN = 1,
    C5VRX_RF_SOURCE_CONTINUOUS = 2,
} c5vrx_rf_source_kind_t;

typedef struct {
    const uint32_t *words;
    size_t word_count;
    uint64_t sequence;
    uint64_t capture_time_us;
    bool discontinuity_before;
    void *owner;
} c5vrx_rf_block_t;

typedef struct c5vrx_rf_source c5vrx_rf_source_t;
typedef bool (*c5vrx_rf_acquire_fn)(c5vrx_rf_source_t *source,
                                    c5vrx_rf_block_t *block,
                                    uint32_t timeout_ms);
typedef void (*c5vrx_rf_release_fn)(c5vrx_rf_source_t *source,
                                    const c5vrx_rf_block_t *block);

struct c5vrx_rf_source {
    const char *name;
    c5vrx_rf_source_kind_t kind;
    uint32_t nominal_sample_rate_hz;
    c5vrx_rf_acquire_fn acquire;
    c5vrx_rf_release_fn release;
    void *context;
};

/* acquire() transfers one immutable block to the consumer and must honor its
 * finite timeout; release() returns that exact block to the producer. */

typedef struct {
    c5vrx_rf_block_t slots[C5VRX_RF_BLOCK_QUEUE_CAPACITY];
    unsigned read_index;
    unsigned write_index;
    unsigned occupancy;
    unsigned high_water_mark;
    atomic_flag lock;
} c5vrx_rf_block_queue_t;

typedef struct {
    int bias_q8;          /* Input code representing zero deviation, Q8. */
    int gain_q8;          /* Gain applied around bias, Q8; 256 = 1.0. */
    bool invert;
    uint8_t sync_code;
    uint8_t blank_code;
    uint8_t black_code;
    uint8_t white_code;
    uint8_t clamp_min;
    uint8_t clamp_max;
    unsigned filter_shift; /* 0 disables; otherwise y += (x-y)/2^N. */
} c5vrx_cvbs_conditioner_config_t;

typedef struct {
    c5vrx_cvbs_conditioner_config_t config;
    int32_t dc_q8;
    int32_t filter_q8;
    bool primed;
} c5vrx_cvbs_conditioner_t;

typedef struct {
    uint64_t source_underruns;
    uint64_t output_underruns;
    uint64_t dropped_rf_blocks;
    uint64_t blocks_processed;
    uint64_t input_samples;
    uint64_t output_samples;
    uint64_t discontinuities;
    uint64_t source_time_us;
    uint64_t wbfm_time_us;
    uint64_t conditioner_time_us;
    uint64_t output_time_us;
    uint64_t source_time_max_us;
    uint64_t wbfm_time_max_us;
    uint64_t conditioner_time_max_us;
    uint64_t output_time_max_us;
    uint64_t boundary_jump_sum;
    uint32_t boundary_jump_max;
    uint32_t achieved_input_rate_hz;
    uint32_t achieved_output_rate_hz;
    uint8_t wbfm_min;
    uint8_t wbfm_max;
    uint8_t cvbs_min;
    uint8_t cvbs_max;
    unsigned queue_occupancy;
    unsigned queue_high_water_mark;
} c5vrx_stream_stats_t;

void c5vrx_rf_block_queue_init(c5vrx_rf_block_queue_t *queue);
bool c5vrx_rf_block_queue_push(c5vrx_rf_block_queue_t *queue,
                               const c5vrx_rf_block_t *block);
bool c5vrx_rf_block_queue_pop(c5vrx_rf_block_queue_t *queue,
                              c5vrx_rf_block_t *block);

void c5vrx_cvbs_conditioner_init(c5vrx_cvbs_conditioner_t *conditioner,
                                 const c5vrx_cvbs_conditioner_config_t *config);
void c5vrx_cvbs_condition(c5vrx_cvbs_conditioner_t *conditioner,
                          const uint8_t *phase_delta,
                          uint8_t *cvbs,
                          size_t count,
                          uint8_t *minimum,
                          uint8_t *maximum);

#ifdef __cplusplus
}
#endif
