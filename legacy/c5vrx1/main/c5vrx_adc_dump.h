/* SPDX-License-Identifier: GPL-3.0-only */

#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * ESP32-C5 ESP-IDF v6.0.2 librftest.a uses this fixed FE/ADC dump RAM in
 * loop_dump_test(): base 0x40830000, size 0x10000 bytes.
 */
#define C5VRX_ADC_DUMP_BASE_ADDR 0x40830000u
#define C5VRX_ADC_DUMP_SIZE_BYTES 0x10000u
#define C5VRX_ADC_DUMP_MAX_SAMPLES (C5VRX_ADC_DUMP_SIZE_BYTES / sizeof(uint32_t))

typedef struct {
    int16_t i;
    int16_t q;
    uint32_t raw;
} c5vrx_iq10_sample_t;

typedef struct {
    size_t blocks_completed;
    size_t samples_per_block;
    uint64_t total_samples;
    uint32_t repeated_block_hashes;
    uint64_t boundary_jump_power_sum;
} c5vrx_adc_chain_stats_t;

typedef struct {
    uint32_t observations;
    uint32_t pointer_changes;
    uint32_t content_changes;
    uint32_t minimum_pointer;
    uint32_t maximum_pointer;
    uint32_t final_status;
    uint64_t active_time_us;
    bool completion_bit_seen;
    bool reached_vendor_timeout;
} c5vrx_adc_ring_probe_stats_t;

typedef struct {
    uint8_t field;
    uint8_t argument;
    uint32_t pointer_delta_lower_bound;
    uint32_t wraps_lower_bound;
    uint32_t observations;
    uint32_t content_changes;
    uint32_t final_pointer_mode;
    uint64_t duration_us;
    uint64_t estimated_words_per_sec_lower_bound;
    uint64_t iq_power;
    bool pointer_in_range;
    bool rate_field_survived_configuration;
} c5vrx_adc_rate_probe_result_t;

/** Decode the lower 20 bits of one C5 RF-test dump word as signed 10-bit I/Q. */
c5vrx_iq10_sample_t c5vrx_adc_decode_word(uint32_t raw);

/**
 * EXPERIMENTAL finite ADC/IQ capture using the vendor RF-test adctrig path.
 *
 * The ABI and dump layout are supported by C5 v6.0.2 disassembly, but actual
 * capture behavior still needs physical ESP32-C5 validation. The routine uses
 * software trigger, historical sample argument 1 (physical rate unknown), and
 * the packed 10-bit dump mode.
 */
esp_err_t c5vrx_adc_dump_capture(size_t sample_count, bool print_raw_words);

/**
 * Capture finite IQ, remove block DC, quantize complex angle to unsigned
 * phase8, and send CRC-protected USB chunks. This preserves one phase sample
 * per source sample while reducing transport from four bytes to one byte.
 */
esp_err_t c5vrx_adc_dump_capture_phase8(size_t sample_count);

/**
 * Re-trigger the finite vendor dump repeatedly and measure block hashes and
 * I/Q discontinuity at block boundaries.
 *
 * This is a diagnostic for determining whether finite captures can be chained
 * with sufficiently small gaps. It is deliberately NOT described as a
 * continuous stream: every block still comes from a separate adctrig() call.
 */
esp_err_t c5vrx_adc_dump_capture_chained(size_t block_count,
                                         size_t sample_count,
                                         c5vrx_adc_chain_stats_t *stats);

/**
 * Probe the vendor dump engine's single, hardware-driven pre-trigger interval.
 *
 * This invokes adctrig() exactly once with the documented RX-error trigger and
 * observes the C5 write-pointer register while that call is active. It neither
 * re-triggers blocks nor claims a continuous source. A long-running moving
 * pointer would justify a later guarded ring-reader experiment; a short run or
 * stationary pointer rejects that hypothesis on the tested silicon/blob.
 */
esp_err_t c5vrx_adc_dump_ring_probe(c5vrx_adc_ring_probe_stats_t *stats);

/** Safely invoke the unmodified vendor producer for all historical arguments. */
esp_err_t c5vrx_adc_rate_probe_all(void);

/** Compare modes 0/11 and record vendor BLE-specific mode 12 as skipped. */
esp_err_t c5vrx_adc_dump_mode_probe(void);

/** Optional coherent-tone phase continuity test for one historical field. */
esp_err_t c5vrx_adc_phase_probe(unsigned field);

#ifdef __cplusplus
}
#endif
