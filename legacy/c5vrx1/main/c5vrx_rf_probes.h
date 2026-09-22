/* SPDX-License-Identifier: GPL-3.0-only */

#pragma once

#include <stdint.h>
#include "esp_err.h"
#include "c5vrx_rf_dump_producer.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint32_t observations;
    uint32_t valid_intervals;
    uint32_t ambiguous_intervals;
    uint32_t wraps_exact;
    uint32_t wraps_lower_bound;
    uint64_t pointer_distance;
    uint64_t duration_cycles;
    uint64_t valid_cycles;
    uint64_t duration_us;
    uint64_t complex_samples_per_sec;
    uint32_t uncertainty_ppm;
    bool enabled_during;
    bool enabled_after;
    bool content_changed;
} c5vrx_producer_cadence_t;

typedef struct {
    float mean_phase_increment;
    float rms_deviation;
    float boundary_phase_increment;
    float boundary_residual;
    float boundary_tolerance;
    float mean_magnitude;
    float coherence;
    uint32_t pointer_wraps;
    bool boundary_continuous;
} c5vrx_phase_continuity_t;

esp_err_t c5vrx_producer_cadence_probe(c5vrx_rf_dump_mode_t mode,
                                       c5vrx_producer_cadence_t *result);
esp_err_t c5vrx_producer_wrap_flag_probe(c5vrx_rf_dump_mode_t mode);
esp_err_t c5vrx_producer_phase_continuity_probe(
    c5vrx_rf_dump_mode_t mode, c5vrx_phase_continuity_t *result);

typedef struct {
    uint64_t observations;
    uint64_t exact_wraps;
    uint64_t producer_absolute;
    uint64_t ambiguous_intervals;
    uint32_t pointer_out_of_range;
    uint32_t producer_stop_events;
    uint32_t adjacent_canary_failures;
    uint32_t register_invariant_failures;
    uint32_t maximum_observation_interval_cycles;
    uint64_t actual_duration_us;
    bool content_changed;
    bool restore_ok;
    bool wifi_restored;
    bool idle_watchdog_temporarily_unsubscribed;
    bool idle_watchdog_restored;
} c5vrx_producer_soak_result_t;

esp_err_t c5vrx_producer_soak(c5vrx_rf_dump_mode_t mode,
                              uint32_t maximum_duration_ms,
                              c5vrx_producer_soak_result_t *result);
/* Retry a watchdog subscription retained after a failed soak teardown. */
esp_err_t c5vrx_producer_soak_restore_watchdog(void);

typedef struct {
    double observed_offset_hz;
    double mean_phase_increment;
    double coherence;
    double mean_magnitude;
} c5vrx_tone_measurement_t;

esp_err_t c5vrx_iq_tone_measure(size_t sample_count,
                                uint32_t measured_sample_rate_hz,
                                c5vrx_tone_measurement_t *measurement);
esp_err_t c5vrx_producer_tone_response_probe(
    c5vrx_rf_dump_mode_t mode, int32_t expected_offset_hz,
    uint32_t measured_sample_rate_hz,
    c5vrx_tone_measurement_t *measurement);

#ifdef __cplusplus
}
#endif
