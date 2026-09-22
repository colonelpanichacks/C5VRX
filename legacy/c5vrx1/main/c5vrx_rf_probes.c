/* SPDX-License-Identifier: GPL-3.0-only */

#include "c5vrx_rf_probes.h"

#include <inttypes.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "sdkconfig.h"
#include "c5vrx_adc_dump.h"
#include "c5vrx_wifi5.h"
#include "c5vrx_ring_tracker.h"
#include "esp_cpu.h"
#include "esp_heap_caps.h"
#include "esp_timer.h"
#include "esp_task_wdt.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define PROBE_RING_WORDS C5VRX_ADC_DUMP_MAX_SAMPLES
#define PROBE_RING_MASK  (PROBE_RING_WORDS - 1u)
#define CADENCE_WINDOW_US 500u
#define CADENCE_PLAUSIBLE_MAX_HZ 320000000ull
#define PHASE_WINDOW 64u
#define C5VRX_PI_F 3.14159265358979323846f

static volatile const uint32_t *const s_dump =
    (volatile const uint32_t *)(uintptr_t)C5VRX_ADC_DUMP_BASE_ADDR;
static TaskHandle_t s_soak_idle_task;
static bool s_soak_idle_wdt_removed;

esp_err_t c5vrx_producer_soak_restore_watchdog(void)
{
    if (!s_soak_idle_wdt_removed) return ESP_OK;
    const esp_err_t err = esp_task_wdt_add(s_soak_idle_task);
    if (err == ESP_OK) {
        s_soak_idle_wdt_removed = false;
        s_soak_idle_task = NULL;
    }
    return err;
}

static bool mode_valid(c5vrx_rf_dump_mode_t mode)
{
    return mode == C5VRX_RF_DUMP_MODE_ORDINARY_RX ||
           mode == C5VRX_RF_DUMP_MODE_11;
}

static uint32_t cpu_hz(void)
{
    return (uint32_t)CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ * 1000000u;
}

static uint32_t dump_fingerprint(void)
{
    uint32_t h = 2166136261u;
    for (unsigned i = 0; i < 64u; ++i) {
        h ^= s_dump[(i * 251u) & PROBE_RING_MASK];
        h *= 16777619u;
    }
    return h;
}

esp_err_t c5vrx_producer_cadence_probe(c5vrx_rf_dump_mode_t mode,
                                       c5vrx_producer_cadence_t *result)
{
    if (!result || !mode_valid(mode)) return ESP_ERR_INVALID_ARG;
    memset(result, 0, sizeof(*result));
    if (!c5vrx_rf_dump_producer_available()) return ESP_ERR_NOT_SUPPORTED;

    const uint32_t content_before = dump_fingerprint();
    esp_err_t err = c5vrx_rf_dump_configure(PROBE_RING_WORDS, mode);
    if (err != ESP_OK) return err;
    err = c5vrx_rf_dump_start();
    if (err != ESP_OK) {
        (void)c5vrx_rf_dump_stop();
        return err;
    }

    c5vrx_rf_dump_status_t status = {0};
    (void)c5vrx_rf_dump_get_status(&status);
    result->enabled_during = status.enabled;
    uint32_t previous_pointer = status.pointer;
    uint32_t previous_cycle = (uint32_t)esp_cpu_get_cycle_count();
    const uint32_t first_cycle = previous_cycle;
    const int64_t first_us = esp_timer_get_time();
    uint32_t last_cycle = first_cycle;

    /* No scheduler delay belongs in this window. A tight MMIO/cycle-counter
     * loop keeps each interval far shorter than a candidate 10--80 MS/s ring
     * wrap. The 320 MS/s bound is deliberately conservative and is reported
     * as an assumption rather than a measured producer limit. */
    do {
        const uint32_t cycle = (uint32_t)esp_cpu_get_cycle_count();
        (void)c5vrx_rf_dump_get_status(&status);
        const uint32_t elapsed_cycles = cycle - previous_cycle;
        const uint32_t pointer = status.pointer;
        ++result->observations;

        const uint64_t possible_advance =
            (uint64_t)elapsed_cycles * CADENCE_PLAUSIBLE_MAX_HZ / cpu_hz();
        if (possible_advance >= PROBE_RING_WORDS) {
            ++result->ambiguous_intervals;
        } else {
            const uint32_t delta =
                (pointer - previous_pointer) & PROBE_RING_MASK;
            result->pointer_distance += delta;
            result->valid_cycles += elapsed_cycles;
            ++result->valid_intervals;
            if (pointer < previous_pointer) {
                ++result->wraps_exact;
                ++result->wraps_lower_bound;
            }
        }
        previous_pointer = pointer;
        previous_cycle = cycle;
        last_cycle = cycle;
    } while ((uint32_t)(last_cycle - first_cycle) <
             (uint64_t)cpu_hz() * CADENCE_WINDOW_US / 1000000u);

    const int64_t last_us = esp_timer_get_time();
    result->duration_cycles = (uint32_t)(last_cycle - first_cycle);
    result->duration_us = (uint64_t)(last_us - first_us);
    if (result->valid_cycles) {
        result->complex_samples_per_sec =
            result->pointer_distance * cpu_hz() / result->valid_cycles;
    }
    if (result->duration_us && result->pointer_distance) {
        const uint64_t timer_ppm = 1000000ull / result->duration_us;
        const uint64_t pointer_ppm = 1000000ull / result->pointer_distance;
        const uint64_t total = timer_ppm + pointer_ppm;
        result->uncertainty_ppm = total > UINT32_MAX ? UINT32_MAX : (uint32_t)total;
    } else {
        result->uncertainty_ppm = UINT32_MAX;
    }

    const esp_err_t stop_err = c5vrx_rf_dump_stop();
    c5vrx_rf_dump_status_t after = {0};
    (void)c5vrx_rf_dump_get_status(&after);
    result->enabled_after = after.enabled;
    result->content_changed = dump_fingerprint() != content_before;
    if (stop_err != ESP_OK) err = stop_err;

    const char *classification = err != ESP_OK ? "FAILED" :
        (result->ambiguous_intervals ? "AMBIGUOUS" :
         (result->valid_intervals && result->pointer_distance ? "MEASURED" :
                                                              "FAILED"));
    printf("C5VRX_PRODUCER_CADENCE mode=%u duration_cycles=%llu duration_us=%llu observations=%u valid_intervals=%u ambiguous_intervals=%u pointer_distance=%llu wraps_exact=%u wraps_lower_bound=%u complex_samples_per_sec=%llu measurement_uncertainty_ppm=%u ambiguity_bound_samples_per_sec=%llu enabled_during=%u enabled_after=%u content_changed=%u restore_ok=%u classification=%s code=%d\n",
           (unsigned)mode,
           (unsigned long long)result->duration_cycles,
           (unsigned long long)result->duration_us,
           (unsigned)result->observations, (unsigned)result->valid_intervals,
           (unsigned)result->ambiguous_intervals,
           (unsigned long long)result->pointer_distance,
           (unsigned)result->wraps_exact, (unsigned)result->wraps_lower_bound,
           (unsigned long long)result->complex_samples_per_sec,
           (unsigned)result->uncertainty_ppm,
           (unsigned long long)CADENCE_PLAUSIBLE_MAX_HZ,
           result->enabled_during ? 1u : 0u,
           result->enabled_after ? 1u : 0u,
           result->content_changed ? 1u : 0u,
           c5vrx_rf_dump_last_restore_ok() ? 1u : 0u,
           classification, (int)err);
    fflush(stdout);
    return err;
}

esp_err_t c5vrx_producer_wrap_flag_probe(c5vrx_rf_dump_mode_t mode)
{
    if (!mode_valid(mode)) return ESP_ERR_INVALID_ARG;
    if (!c5vrx_rf_dump_producer_available()) return ESP_ERR_NOT_SUPPORTED;
    esp_err_t err = c5vrx_rf_dump_configure(PROBE_RING_WORDS, mode);
    if (err != ESP_OK) return err;
    c5vrx_rf_dump_status_t status = {0};
    (void)c5vrx_rf_dump_get_status(&status);
    const bool initial_flag = status.wrap_or_done;
    err = c5vrx_rf_dump_start();
    if (err != ESP_OK) { (void)c5vrx_rf_dump_stop(); return err; }

    const uint32_t start_cycle = (uint32_t)esp_cpu_get_cycle_count();
    const int64_t start_us = esp_timer_get_time();
    uint32_t previous = status.pointer;
    uint32_t first_pointer_wrap_cycle = 0;
    uint32_t first_flag_change_cycle = 0;
    uint32_t observations = 0;
    uint32_t pointer_wraps = 0;
    while (esp_timer_get_time() - start_us < 5000) {
        (void)c5vrx_rf_dump_get_status(&status);
        const uint32_t now = (uint32_t)esp_cpu_get_cycle_count();
        ++observations;
        if (status.pointer < previous) {
            ++pointer_wraps;
            if (!first_pointer_wrap_cycle) first_pointer_wrap_cycle = now - start_cycle;
        }
        if (status.wrap_or_done != initial_flag && !first_flag_change_cycle)
            first_flag_change_cycle = now - start_cycle;
        previous = status.pointer;
        if (first_pointer_wrap_cycle && observations > 32u &&
            first_flag_change_cycle) break;
    }
    const bool flag_during = status.wrap_or_done;
    const bool enabled_during = status.enabled;
    const esp_err_t stop_err = c5vrx_rf_dump_stop();
    if (err == ESP_OK) err = stop_err;

    const char *classification = "NO_CORRELATION_OBSERVED";
    if (first_pointer_wrap_cycle && first_flag_change_cycle) {
        const uint32_t separation = first_pointer_wrap_cycle > first_flag_change_cycle
            ? first_pointer_wrap_cycle - first_flag_change_cycle
            : first_flag_change_cycle - first_pointer_wrap_cycle;
        classification = separation < cpu_hz() / 10000u
            ? "FLAG_TRANSITION_NEAR_POINTER_WRAP" :
              "FLAG_AND_POINTER_WRAP_BOTH_OBSERVED";
    } else if (first_flag_change_cycle) {
        classification = "FLAG_CHANGED_WITHOUT_OBSERVED_POINTER_WRAP";
    } else if (first_pointer_wrap_cycle) {
        classification = flag_during == initial_flag
            ? "POINTER_WRAP_WITHOUT_FLAG_TRANSITION" :
              "FLAG_ALREADY_ASSERTED_AT_POINTER_WRAP";
    }
    printf("C5VRX_WRAP_FLAG_PROBE mode=%u observations=%u pointer_wraps=%u initial_bit18=%u final_bit18=%u first_pointer_wrap_cycle=%u first_bit18_change_cycle=%u enabled_during=%u restore_ok=%u classification=%s code=%d\n",
           (unsigned)mode, (unsigned)observations, (unsigned)pointer_wraps,
           initial_flag ? 1u : 0u, flag_during ? 1u : 0u,
           (unsigned)first_pointer_wrap_cycle,
           (unsigned)first_flag_change_cycle,
           enabled_during ? 1u : 0u,
           c5vrx_rf_dump_last_restore_ok() ? 1u : 0u,
           classification, (int)err);
    fflush(stdout);
    return err;
}

static float wrap_phase(float value)
{
    while (value > C5VRX_PI_F) value -= 2.0f * C5VRX_PI_F;
    while (value < -C5VRX_PI_F) value += 2.0f * C5VRX_PI_F;
    return value;
}

static float phase_increment(unsigned previous, unsigned current,
                             float *magnitude)
{
    const c5vrx_iq10_sample_t a = c5vrx_adc_decode_word(s_dump[previous]);
    const c5vrx_iq10_sample_t b = c5vrx_adc_decode_word(s_dump[current]);
    const float cross = (float)a.i * b.q - (float)a.q * b.i;
    const float dot = (float)a.i * b.i + (float)a.q * b.q;
    if (magnitude) {
        *magnitude = 0.5f * (sqrtf((float)a.i * a.i + (float)a.q * a.q) +
                             sqrtf((float)b.i * b.i + (float)b.q * b.q));
    }
    return atan2f(cross, dot);
}

esp_err_t c5vrx_producer_phase_continuity_probe(
    c5vrx_rf_dump_mode_t mode, c5vrx_phase_continuity_t *result)
{
    if (!result || !mode_valid(mode)) return ESP_ERR_INVALID_ARG;
    memset(result, 0, sizeof(*result));
    if (!c5vrx_rf_dump_producer_available()) return ESP_ERR_NOT_SUPPORTED;
    esp_err_t err = c5vrx_rf_dump_configure(PROBE_RING_WORDS, mode);
    if (err != ESP_OK) return err;
    err = c5vrx_rf_dump_start();
    if (err != ESP_OK) { (void)c5vrx_rf_dump_stop(); return err; }

    c5vrx_rf_dump_status_t status = {0};
    (void)c5vrx_rf_dump_get_status(&status);
    uint32_t previous = status.pointer;
    const int64_t deadline = esp_timer_get_time() + 10000;
    while (esp_timer_get_time() < deadline && result->pointer_wraps == 0u) {
        (void)c5vrx_rf_dump_get_status(&status);
        if (status.pointer < previous) ++result->pointer_wraps;
        previous = status.pointer;
    }
    const esp_err_t stop_err = c5vrx_rf_dump_stop();
    if (err == ESP_OK) err = stop_err;
    if (!result->pointer_wraps && err == ESP_OK) err = ESP_ERR_TIMEOUT;

    float increments[PHASE_WINDOW * 2u];
    float sum_sin = 0.0f, sum_cos = 0.0f, magnitude_sum = 0.0f;
    unsigned count = 0;
    for (unsigned n = PROBE_RING_WORDS - PHASE_WINDOW;
         n < PROBE_RING_WORDS; ++n) {
        float magnitude = 0.0f;
        increments[count] = phase_increment(n - 1u, n, &magnitude);
        sum_sin += sinf(increments[count]);
        sum_cos += cosf(increments[count]);
        magnitude_sum += magnitude;
        ++count;
    }
    result->boundary_phase_increment =
        phase_increment(PROBE_RING_WORDS - 1u, 0u, NULL);
    for (unsigned n = 1; n <= PHASE_WINDOW; ++n) {
        float magnitude = 0.0f;
        increments[count] = phase_increment(n - 1u, n, &magnitude);
        sum_sin += sinf(increments[count]);
        sum_cos += cosf(increments[count]);
        magnitude_sum += magnitude;
        ++count;
    }
    result->mean_phase_increment = atan2f(sum_sin, sum_cos);
    float squared = 0.0f;
    for (unsigned n = 0; n < count; ++n) {
        const float residual = wrap_phase(increments[n] - result->mean_phase_increment);
        squared += residual * residual;
    }
    result->rms_deviation = sqrtf(squared / count);
    result->boundary_residual = wrap_phase(
        result->boundary_phase_increment - result->mean_phase_increment);
    result->mean_magnitude = magnitude_sum / count;
    result->coherence = sqrtf(sum_sin * sum_sin + sum_cos * sum_cos) / count;

    /* A fixed 0.25 radian ceiling prevents a noisy tone from making its own
     * acceptance window arbitrarily wide. The 5-sigma term keeps a clean tone
     * from being judged against an unrealistically exact floating-point zero. */
    result->boundary_tolerance = fminf(0.25f,
        fmaxf(0.05f, 5.0f * result->rms_deviation));
    result->boundary_continuous = err == ESP_OK &&
        result->coherence >= 0.90f && result->mean_magnitude >= 8.0f &&
        fabsf(result->boundary_residual) <= result->boundary_tolerance;

    const char *classification = err != ESP_OK ? "FAILED" :
        (result->coherence < 0.90f || result->mean_magnitude < 8.0f ?
            "INCOHERENT_SOURCE" :
            (result->boundary_continuous ? "MEASURED_CONTINUOUS" :
                                           "MEASURED_DISCONTINUITY"));
    printf("C5VRX_PHASE_CONTINUITY mode=%u mean_phase_increment=%g rms_deviation=%g boundary_phase_increment=%g boundary_residual=%g boundary_tolerance=%g boundary_continuous=%u signal_magnitude=%g coherence_metric=%g pointer_wraps=%u iq10_format=%s coherent_tone_required=1 restore_ok=%u classification=%s code=%d\n",
           (unsigned)mode, (double)result->mean_phase_increment,
           (double)result->rms_deviation,
           (double)result->boundary_phase_increment,
           (double)result->boundary_residual,
           (double)result->boundary_tolerance,
           result->boundary_continuous ? 1u : 0u,
           (double)result->mean_magnitude, (double)result->coherence,
           (unsigned)result->pointer_wraps,
           mode == C5VRX_RF_DUMP_MODE_ORDINARY_RX ? "VENDOR_DECODED" : "UNPROVEN",
           c5vrx_rf_dump_last_restore_ok() ? 1u : 0u,
           classification, (int)err);
    fflush(stdout);
    return err;
}

static bool duration_allowed(uint32_t duration_ms)
{
    static const uint32_t allowed[] = {1u, 10u, 100u, 1000u, 5000u, 30000u};
    for (unsigned i = 0; i < sizeof(allowed) / sizeof(allowed[0]); ++i)
        if (duration_ms == allowed[i]) return true;
    return false;
}

static esp_err_t soak_stage(c5vrx_rf_dump_mode_t mode, uint32_t duration_ms,
                            c5vrx_producer_soak_result_t *result)
{
    memset(result, 0, sizeof(*result));
    esp_err_t err = c5vrx_producer_soak_restore_watchdog();
    if (err != ESP_OK) return err;
    const size_t heap_before = heap_caps_get_free_size(MALLOC_CAP_INTERNAL);
    const uint32_t content_before = dump_fingerprint();
    c5vrx_wifi5_status_t wifi_before = {0};
    c5vrx_wifi5_status_t wifi_after = {0};
    const bool wifi_expected = c5vrx_wifi5_get_status(&wifi_before) == ESP_OK;
    err = c5vrx_rf_dump_configure(PROBE_RING_WORDS, mode);
    if (err != ESP_OK) return err;
    c5vrx_rf_dump_registers_t configured = {0};
    (void)c5vrx_rf_dump_read_registers(&configured);
    err = c5vrx_rf_dump_start();
    if (err != ESP_OK) { (void)c5vrx_rf_dump_stop(); return err; }

    /* Exact writer tracking cannot sleep across multiple potential wraps.
     * The single-core idle task is normally watched, so temporarily remove
     * only that idle task during the bounded tight-poll proof and restore it
     * immediately afterward. The producer/control task itself is unchanged. */
    TaskHandle_t idle_task = xTaskGetIdleTaskHandleForCore(xPortGetCoreID());
    const bool idle_wdt_removed = idle_task &&
        esp_task_wdt_status(idle_task) == ESP_OK &&
        esp_task_wdt_delete(idle_task) == ESP_OK;
    if (idle_wdt_removed) {
        s_soak_idle_task = idle_task;
        s_soak_idle_wdt_removed = true;
    }
    result->idle_watchdog_temporarily_unsubscribed = idle_wdt_removed;

    const int64_t start = esp_timer_get_time();
    const int64_t deadline = start + (int64_t)duration_ms * 1000;
    c5vrx_rf_dump_status_t status = {0};
    c5vrx_ring_tracker_t tracker;
    if (c5vrx_ring_tracker_init(&tracker, PROBE_RING_WORDS, 1u, cpu_hz(),
                                (uint32_t)CADENCE_PLAUSIBLE_MAX_HZ) != 0) {
        (void)c5vrx_rf_dump_stop();
        (void)c5vrx_producer_soak_restore_watchdog();
        return ESP_FAIL;
    }
    while (esp_timer_get_time() < deadline) {
        (void)c5vrx_rf_dump_get_status(&status);
        const uint32_t cycle = (uint32_t)esp_cpu_get_cycle_count();
        if (!status.enabled) {
            ++result->producer_stop_events;
            err = ESP_FAIL;
            break;
        }
        const c5vrx_ring_track_result_t tracked =
            c5vrx_ring_tracker_observe(&tracker, status.pointer, cycle);
        if (tracked == C5VRX_RING_TRACK_POINTER_OUT_OF_RANGE) {
            ++result->pointer_out_of_range;
            err = ESP_FAIL;
            break;
        }
        if (tracked == C5VRX_RING_TRACK_INTERVAL_AMBIGUOUS) {
            err = ESP_FAIL;
            break;
        }
        /* This proof observer is not a data consumer. Keep its synthetic
         * consumer at the writer so guard-distance policy does not obscure
         * the independent pointer/wrap proof. */
        tracker.consumer_absolute = tracker.producer_absolute;
        if ((tracker.observations & 1023u) == 0u &&
            !c5vrx_rf_dump_canaries_intact()) {
            ++result->adjacent_canary_failures;
            err = ESP_FAIL;
            break;
        }
    }
    c5vrx_rf_dump_registers_t during = {0};
    (void)c5vrx_rf_dump_read_registers(&during);
    if (during.dump_format != configured.dump_format ||
        during.source_mux != configured.source_mux ||
        during.fe_path != configured.fe_path || during.fe_aux != configured.fe_aux)
        ++result->register_invariant_failures;
    if (!c5vrx_rf_dump_canaries_intact())
        ++result->adjacent_canary_failures;
    const bool changed = dump_fingerprint() != content_before;
    const esp_err_t stop_err = c5vrx_rf_dump_stop();
    const esp_err_t idle_wdt_restore_err =
        c5vrx_producer_soak_restore_watchdog();
    result->idle_watchdog_restored = idle_wdt_restore_err == ESP_OK;
    if (err == ESP_OK) err = stop_err;
    if (err == ESP_OK) err = idle_wdt_restore_err;
    const size_t heap_after = heap_caps_get_free_size(MALLOC_CAP_INTERNAL);
    const intptr_t heap_delta = (intptr_t)heap_after - (intptr_t)heap_before;
    const bool wifi_after_valid =
        c5vrx_wifi5_get_status(&wifi_after) == ESP_OK;
    const bool wifi_restored = !wifi_expected ||
        (wifi_after_valid &&
         wifi_after.active_primary_channel == wifi_before.active_primary_channel &&
         wifi_after.ht40 == wifi_before.ht40 &&
         wifi_after.promiscuous_enabled == wifi_before.promiscuous_enabled);
    result->observations = tracker.observations;
    result->exact_wraps = tracker.wraps;
    result->producer_absolute = tracker.producer_absolute;
    result->ambiguous_intervals = tracker.ambiguous_intervals;
    result->maximum_observation_interval_cycles =
        tracker.maximum_service_interval_cycles;
    result->actual_duration_us = (uint64_t)(esp_timer_get_time() - start);
    result->content_changed = changed;
    result->restore_ok = c5vrx_rf_dump_last_restore_ok();
    result->wifi_restored = wifi_restored;
    if (!changed || !tracker.wraps || !result->restore_ok ||
        result->ambiguous_intervals || result->pointer_out_of_range ||
        result->producer_stop_events || result->adjacent_canary_failures ||
        result->register_invariant_failures ||
        heap_delta != 0 || !wifi_restored ||
        !result->idle_watchdog_restored) {
        if (err == ESP_OK) err = ESP_FAIL;
    }
    printf("C5VRX_PRODUCER_SOAK_STAGE mode=%u requested_ms=%u actual_us=%llu observations=%llu exact_wraps=%llu producer_absolute=%llu ambiguous_intervals=%llu pointer_out_of_range=%u producer_stop_events=%u adjacent_canary_failures=%u register_invariant_failures=%u maximum_observation_interval_cycles=%u content_changed=%u heap_before=%u heap_after=%u heap_delta=%ld wifi_expected=%u wifi_after_valid=%u wifi_channel_before=%u wifi_channel_after=%u wifi_restored=%u restore_ok=%u idle_wdt_temporarily_unsubscribed=%u idle_wdt_restored=%u classification=%s code=%d\n",
           (unsigned)mode, (unsigned)duration_ms,
           (unsigned long long)result->actual_duration_us,
           (unsigned long long)result->observations,
           (unsigned long long)result->exact_wraps,
           (unsigned long long)result->producer_absolute,
           (unsigned long long)result->ambiguous_intervals,
           (unsigned)result->pointer_out_of_range,
           (unsigned)result->producer_stop_events,
           (unsigned)result->adjacent_canary_failures,
           (unsigned)result->register_invariant_failures,
           (unsigned)result->maximum_observation_interval_cycles,
           changed ? 1u : 0u,
           (unsigned)heap_before, (unsigned)heap_after,
           (long)heap_delta, wifi_expected ? 1u : 0u,
           wifi_after_valid ? 1u : 0u,
           wifi_expected ? (unsigned)wifi_before.active_primary_channel : 0u,
           wifi_after_valid ? (unsigned)wifi_after.active_primary_channel : 0u,
           wifi_restored ? 1u : 0u,
           c5vrx_rf_dump_last_restore_ok() ? 1u : 0u,
           idle_wdt_removed ? 1u : 0u,
           result->idle_watchdog_restored ? 1u : 0u,
           err == ESP_OK ? "MEASURED_CONTINUOUS_RING_PASS" : "FAILED",
           (int)err);
    fflush(stdout);
    return err;
}

esp_err_t c5vrx_producer_soak(c5vrx_rf_dump_mode_t mode,
                              uint32_t maximum_duration_ms,
                              c5vrx_producer_soak_result_t *result)
{
    static const uint32_t stages[] = {1u, 10u, 100u, 1000u, 5000u, 30000u};
    if (!result || !mode_valid(mode) || !duration_allowed(maximum_duration_ms))
        return ESP_ERR_INVALID_ARG;
    memset(result, 0, sizeof(*result));
    for (unsigned i = 0; i < sizeof(stages) / sizeof(stages[0]); ++i) {
        c5vrx_producer_soak_result_t stage = {0};
        const esp_err_t err = soak_stage(mode, stages[i], &stage);
        *result = stage;
        if (err != ESP_OK) return err;
        if (stages[i] == maximum_duration_ms) return ESP_OK;
    }
    return ESP_ERR_INVALID_ARG;
}

esp_err_t c5vrx_iq_tone_measure(size_t sample_count,
                                uint32_t measured_sample_rate_hz,
                                c5vrx_tone_measurement_t *measurement)
{
    if (!measurement || sample_count < 16u ||
        sample_count > PROBE_RING_WORDS || !measured_sample_rate_hz)
        return ESP_ERR_INVALID_ARG;
    memset(measurement, 0, sizeof(*measurement));
    double unit_real = 0.0, unit_imag = 0.0, magnitude = 0.0;
    unsigned valid = 0;
    c5vrx_iq10_sample_t previous = c5vrx_adc_decode_word(s_dump[0]);
    for (size_t i = 1; i < sample_count; ++i) {
        const c5vrx_iq10_sample_t current = c5vrx_adc_decode_word(s_dump[i]);
        const double real = (double)current.i * previous.i +
                            (double)current.q * previous.q;
        const double imag = (double)current.q * previous.i -
                            (double)current.i * previous.q;
        const double power = sqrt(real * real + imag * imag);
        if (power > 1.0) {
            unit_real += real / power;
            unit_imag += imag / power;
            magnitude += sqrt((double)current.i * current.i +
                              (double)current.q * current.q);
            ++valid;
        }
        previous = current;
    }
    if (valid < sample_count / 2u) return ESP_ERR_INVALID_RESPONSE;
    measurement->mean_phase_increment = atan2(unit_imag, unit_real);
    measurement->observed_offset_hz =
        measurement->mean_phase_increment * measured_sample_rate_hz /
        (2.0 * 3.14159265358979323846);
    measurement->coherence =
        sqrt(unit_real * unit_real + unit_imag * unit_imag) / valid;
    measurement->mean_magnitude = magnitude / valid;
    return ESP_OK;
}

esp_err_t c5vrx_producer_tone_response_probe(
    c5vrx_rf_dump_mode_t mode, int32_t expected_offset_hz,
    uint32_t measured_sample_rate_hz,
    c5vrx_tone_measurement_t *measurement)
{
    if (!measurement || !mode_valid(mode) ||
        measured_sample_rate_hz < 1000000u ||
        measured_sample_rate_hz > 320000000u ||
        llabs((long long)expected_offset_hz) >=
            (long long)measured_sample_rate_hz / 2) {
        return ESP_ERR_INVALID_ARG;
    }
    memset(measurement, 0, sizeof(*measurement));
    esp_err_t err = c5vrx_rf_dump_configure(PROBE_RING_WORDS, mode);
    if (err != ESP_OK) return err;
    err = c5vrx_rf_dump_start();
    if (err != ESP_OK) {
        (void)c5vrx_rf_dump_stop();
        return err;
    }
    c5vrx_rf_dump_status_t status = {0};
    (void)c5vrx_rf_dump_get_status(&status);
    uint16_t previous = status.pointer;
    unsigned wraps = 0;
    const int64_t deadline = esp_timer_get_time() + 10000;
    while (esp_timer_get_time() < deadline && wraps == 0u) {
        (void)c5vrx_rf_dump_get_status(&status);
        if (status.pointer < previous) ++wraps;
        previous = status.pointer;
    }
    const esp_err_t stop_err = c5vrx_rf_dump_stop();
    if (err == ESP_OK) err = stop_err;
    if (!wraps && err == ESP_OK) err = ESP_ERR_TIMEOUT;
    if (err == ESP_OK)
        err = c5vrx_iq_tone_measure(PROBE_RING_WORDS,
                                    measured_sample_rate_hz, measurement);

    const double error_hz = measurement->observed_offset_hz -
                            expected_offset_hz;
    const char *classification = err != ESP_OK ? "FAILED" :
                                                   "MEASURED_ON_HARDWARE";
    const char *tone_state = measurement->coherence >= 0.90 &&
                             measurement->mean_magnitude >= 8.0
        ? "COHERENT" : "ATTENUATED_OR_INCOHERENT";
    printf("C5VRX_TONE_RESPONSE mode=%u expected_offset_hz=%ld observed_offset_hz=%.3f offset_error_hz=%.3f measured_sample_rate_hz=%u mean_phase_increment=%.9f coherence=%.6f signal_magnitude=%.3f pointer_wraps=%u restore_ok=%u tone_state=%s classification=%s code=%d\n",
           (unsigned)mode, (long)expected_offset_hz,
           measurement->observed_offset_hz, error_hz,
           (unsigned)measured_sample_rate_hz,
           measurement->mean_phase_increment, measurement->coherence,
           measurement->mean_magnitude, wraps,
           c5vrx_rf_dump_last_restore_ok() ? 1u : 0u,
           tone_state, classification, (int)err);
    fflush(stdout);
    return err;
}
