/* SPDX-License-Identifier: GPL-3.0-only */

#include "c5vrx_bench.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "c5vrx_cvbs_live_out.h"
#include "c5vrx_stream.h"
#include "c5vrx_usb_preview.h"
#include "c5vrx_wbfm_hw.h"
#include "esp_cpu.h"
#include "esp_heap_caps.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sdkconfig.h"

#define BENCH_WORDS 16384u
#define BENCH_RETAINED 1000000u

static uint32_t pack_iq(unsigned n)
{
    const int32_t i = (int32_t)((n * 29u) & 0x3ffu) - 512;
    const int32_t q = (int32_t)((n * 47u) & 0x3ffu) - 512;
    return (((uint32_t)i & 0x3ffu) << 10) | ((uint32_t)q & 0x3ffu);
}

esp_err_t c5vrx_bench_sparse(unsigned factor)
{
    if (factor != 2u && factor != 4u && factor != 8u)
        return ESP_ERR_INVALID_ARG;
    uint32_t *input = heap_caps_malloc(BENCH_WORDS * sizeof(uint32_t),
                                       MALLOC_CAP_INTERNAL);
    if (!input) return ESP_ERR_NO_MEM;
    for (unsigned i = 0; i < BENCH_WORDS; ++i) input[i] = pack_iq(i);
    volatile uint32_t checksum = 0;
    const uint32_t first_cycle = (uint32_t)esp_cpu_get_cycle_count();
    const int64_t first_us = esp_timer_get_time();
    unsigned index = 0;
    for (unsigned i = 0; i < BENCH_RETAINED; ++i) {
        checksum ^= input[index];
        index = (index + factor) & (BENCH_WORDS - 1u);
    }
    const uint32_t cycles = (uint32_t)esp_cpu_get_cycle_count() - first_cycle;
    const uint64_t us = (uint64_t)(esp_timer_get_time() - first_us);
    const uint64_t reads_s = us ? BENCH_RETAINED * 1000000ull / us : 0;
    const uint32_t cycles_per = cycles / BENCH_RETAINED;
    const uint32_t occupancy = us ?
        (uint32_t)((uint64_t)cycles * 100u /
                   ((uint64_t)CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ * us)) : 0;
    printf("C5VRX_BENCH_SPARSE factor=%u retained=%u duration_us=%llu reads_per_sec=%llu bytes_per_sec=%llu cycles_per_retained=%u cpu_occupancy_percent=%u checksum=%08lx classification=MEASURED_ON_HARDWARE_CPU_MEMORY_ONLY\n",
           factor, BENCH_RETAINED, (unsigned long long)us,
           (unsigned long long)reads_s,
           (unsigned long long)(reads_s * sizeof(uint32_t)),
           (unsigned)cycles_per, (unsigned)occupancy,
           (unsigned long)checksum);
    fflush(stdout);
    free(input);
    return ESP_OK;
}

static esp_err_t alloc_transform(uint32_t **input, uint8_t **output,
                                 c5vrx_wbfm_kernel_t kernel,
                                 c5vrx_wbfm_hw_context_t **context)
{
    *input = heap_caps_malloc(BENCH_WORDS * sizeof(uint32_t),
                              MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);
    *output = heap_caps_malloc(BENCH_WORDS / 4u,
                               MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);
    if (!*input || !*output) return ESP_ERR_NO_MEM;
    for (unsigned i = 0; i < BENCH_WORDS; ++i) (*input)[i] = pack_iq(i);
    return c5vrx_wbfm_hw_create_kernel(BENCH_WORDS, kernel, context);
}

esp_err_t c5vrx_bench_bitscrambler(uint64_t *input_bytes_per_second)
{
    if (input_bytes_per_second) *input_bytes_per_second = 0;
    const unsigned iterations = 16u;
    uint64_t rate[2] = {0};
    esp_err_t result = ESP_OK;
    for (unsigned kernel = 0; kernel < 2u; ++kernel) {
        uint32_t *input = NULL;
        uint8_t *output = NULL;
        c5vrx_wbfm_hw_context_t *context = NULL;
        esp_err_t err = alloc_transform(&input, &output,
            (c5vrx_wbfm_kernel_t)kernel, &context);
        size_t written = 0;
        const int64_t first_us = esp_timer_get_time();
        for (unsigned i = 0; i < iterations && err == ESP_OK; ++i)
            err = c5vrx_wbfm_hw_transform_context(
                context, input, BENCH_WORDS, output, BENCH_WORDS / 4u,
                &written);
        const uint64_t us = (uint64_t)(esp_timer_get_time() - first_us);
        const uint64_t input_bytes =
            (uint64_t)iterations * BENCH_WORDS * sizeof(uint32_t);
        rate[kernel] = us ? input_bytes * 1000000u / us : 0u;
        printf("C5VRX_BENCH_WBFM_CANDIDATE kernel=phase%u input_bytes=%llu output_bytes=%llu duration_us=%llu input_bytes_per_sec=%llu transform_factor=4 persistent_context=1 iq_dc_lut=continuous_sparse classification=%s code=%d\n",
               kernel ? 8u : 6u, (unsigned long long)input_bytes,
               (unsigned long long)iterations * written,
               (unsigned long long)us, (unsigned long long)rate[kernel],
               err == ESP_OK ? "MEASURED_ON_HARDWARE_SYNTHETIC" : "FAILED",
               (int)err);
        if (result == ESP_OK && err != ESP_OK) result = err;
        c5vrx_wbfm_hw_destroy(context);
        free(input); free(output);
    }
    const bool phase8_realtime = result == ESP_OK &&
        rate[1] * 100u >= rate[0] * 90u;
    const unsigned selected = phase8_realtime ? 8u : 6u;
    const uint64_t selected_rate = phase8_realtime ? rate[1] : rate[0];
    if (result == ESP_OK && input_bytes_per_second)
        *input_bytes_per_second = selected_rate;
    printf("C5VRX_BENCH_BITSCRAMBLER selected=phase%u selection_rule=HIGHEST_PHASE_RESOLUTION_WITHIN_10_PERCENT_THROUGHPUT input_bytes_per_sec=%llu physical_burst_quality_gate=PENDING code=%d\n",
           selected, (unsigned long long)selected_rate, (int)result);
    fflush(stdout);
    return result;
}

esp_err_t c5vrx_bench_parlio(void)
{
    return c5vrx_bench_parlio_clock(C5VRX_CVBS_SOURCE_SAMPLE_RATE_HZ);
}

esp_err_t c5vrx_bench_parlio_clock(uint32_t clock_hz)
{
    uint8_t *samples = heap_caps_malloc(4096u,
                                        MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);
    if (!samples) return ESP_ERR_NO_MEM;
    memset(samples, 19, 4096u);
    esp_err_t err = c5vrx_cvbs_live_out_start_at_rate(4096u, clock_hz);
    const unsigned blocks = 32u;
    c5vrx_cvbs_live_out_stats_t before = {0};
    c5vrx_cvbs_live_out_get_stats(&before);
    c5vrx_cvbs_live_out_qualification_begin(blocks);
    const int64_t first_us = esp_timer_get_time();
    for (unsigned i = 0; i < blocks && err == ESP_OK; ++i)
        err = c5vrx_cvbs_live_out_write_wait(samples, 4096u, 100u);
    const TickType_t wait_start = xTaskGetTickCount();
    c5vrx_cvbs_live_out_stats_t after = before;
    while (err == ESP_OK &&
           after.live_retirements_completed -
               before.live_retirements_completed < blocks &&
           xTaskGetTickCount() - wait_start < pdMS_TO_TICKS(1000u)) {
        taskYIELD();
        c5vrx_cvbs_live_out_get_stats(&after);
    }
    if (err == ESP_OK &&
        after.live_retirements_completed -
            before.live_retirements_completed < blocks)
        err = ESP_ERR_TIMEOUT;
    if (err == ESP_OK &&
        (after.mailbox_drops != before.mailbox_drops ||
         after.qualification_underruns != before.qualification_underruns ||
         after.phase_mismatch_drops != before.phase_mismatch_drops ||
         after.guardian_failures != before.guardian_failures))
        err = ESP_ERR_INVALID_STATE;
    c5vrx_cvbs_live_out_qualification_end();
    const uint64_t us = (uint64_t)(esp_timer_get_time() - first_us);
    const esp_err_t stop_err = c5vrx_cvbs_live_out_stop();
    if (err == ESP_OK) err = stop_err;
    printf("C5VRX_BENCH_PARLIO clock_hz=%u samples=%u duration_us=%llu samples_per_sec=%llu live_blocks=%llu live_blocks_retired=%llu live_retirements_completed=%llu filler_blocks=%llu mailbox_drops=%llu qualification_underruns=%llu phase_mismatch_drops=%llu guardian_failures=%llu underrun=%u production_clock=%u classification=%s code=%d\n",
           (unsigned)clock_hz, blocks * 4096u, (unsigned long long)us,
           (unsigned long long)(us ? (uint64_t)blocks * 4096u * 1000000u / us : 0u),
           (unsigned long long)(after.live_blocks - before.live_blocks),
           (unsigned long long)(after.live_blocks_retired - before.live_blocks_retired),
           (unsigned long long)(after.live_retirements_completed - before.live_retirements_completed),
           (unsigned long long)(after.filler_blocks - before.filler_blocks),
           (unsigned long long)(after.mailbox_drops - before.mailbox_drops),
           (unsigned long long)(after.qualification_underruns - before.qualification_underruns),
           (unsigned long long)(after.phase_mismatch_drops - before.phase_mismatch_drops),
           (unsigned long long)(after.guardian_failures - before.guardian_failures),
           err == ESP_OK ? 0u : 1u,
           clock_hz == C5VRX_CVBS_SOURCE_SAMPLE_RATE_HZ ? 1u : 0u,
           err == ESP_OK ? "MEASURED_ON_HARDWARE_SYNTHETIC" : "FAILED",
           (int)err);
    fflush(stdout);
    free(samples);
    return err;
}

esp_err_t c5vrx_bench_usb_preview(void)
{
    uint8_t *line = malloc(1280u);
    if (!line) return ESP_ERR_NO_MEM;
    memset(line, 19, 1280u);
    memset(line, 0, 94u);
    for (unsigned i = 210u; i < 1250u; ++i)
        line[i] = (uint8_t)(20u + (i - 210u) * 43u / 1040u);
    esp_err_t err = c5vrx_usb_preview_start();
    c5vrx_usb_preview_stats_t before = {0}, progress = {0};
    c5vrx_usb_preview_get_stats(&before);
    const int64_t first_us = esp_timer_get_time();
    c5vrx_cvbs_sync_tracker_t timing = {0};
    timing.horizontal_locked = true;
    timing.vertical_locked = true;
    timing.standard = C5VRX_VIDEO_STANDARD_PAL;
    timing.polarity = C5VRX_SYNC_POLARITY_NEGATIVE;
    timing.sample_rate_hz = 20000000u;
    timing.line_period_samples = 1280u;
    timing.field_id = 1u;
    for (unsigned line_n = 0; line_n < 265u && err == ESP_OK; ++line_n) {
        timing.last_hsync_start = (uint64_t)(line_n + 1u) * 1280u;
        timing.samples_seen = timing.last_hsync_start + 1280u;
        timing.field_line = (uint16_t)(line_n + 1u);
        c5vrx_usb_preview_ingest_timed(line, 1280u, &timing);
        const TickType_t wait_start = xTaskGetTickCount();
        do {
            vTaskDelay(pdMS_TO_TICKS(1u));
            c5vrx_usb_preview_get_stats(&progress);
        } while (progress.samples_ingested < timing.samples_seen &&
                 xTaskGetTickCount() - wait_start < pdMS_TO_TICKS(100u));
        if (progress.samples_ingested < timing.samples_seen)
            err = ESP_ERR_TIMEOUT;
    }
    if (err == ESP_OK && progress.frames_completed <= before.frames_completed)
        err = ESP_ERR_INVALID_STATE;
    if (err == ESP_OK && progress.frames_dropped != before.frames_dropped)
        err = ESP_ERR_INVALID_STATE;
    const uint64_t us = (uint64_t)(esp_timer_get_time() - first_us);
    printf("C5VRX_BENCH_USB_PREVIEW input_samples=%u output_bytes=%u duration_us=%llu frames_completed=%u frames_dropped=%u reducer_drained=1 hv_sync_tracker=1 binary_protocol_version=%u transport_throughput_requires_connected_host=1 classification=%s code=%d\n",
           265u * 1280u,
           C5VRX_USB_PREVIEW_WIDTH * C5VRX_USB_PREVIEW_HEIGHT * 3u / 2u,
           (unsigned long long)us,
           (unsigned)(progress.frames_completed - before.frames_completed),
           (unsigned)(progress.frames_dropped - before.frames_dropped),
           C5VRX_USB_PREVIEW_PROTOCOL_VERSION,
           err == ESP_OK ? "MEASURED_ON_HARDWARE_REDUCER_ONLY" : "FAILED",
           (int)err);
    fflush(stdout);
    /* Leave preview enabled long enough for its low-priority sender, then the
     * explicit STOP command can test disconnect/non-interference behavior. */
    free(line);
    return err;
}

esp_err_t c5vrx_bench_pipeline(uint64_t *input_samples_per_second)
{
    if (input_samples_per_second) *input_samples_per_second = 0;
    uint32_t *input = NULL;
    uint8_t *wbfm = NULL;
    c5vrx_wbfm_hw_context_t *context = NULL;
    esp_err_t err = alloc_transform(&input, &wbfm,
        C5VRX_WBFM_PHASE8_4TO1, &context);
    uint8_t *cvbs = heap_caps_malloc(BENCH_WORDS / 4u, MALLOC_CAP_INTERNAL);
    if (!cvbs && err == ESP_OK) err = ESP_ERR_NO_MEM;
    c5vrx_cvbs_conditioner_t conditioner;
    const c5vrx_cvbs_conditioner_config_t cfg = {
        .bias_q8 = 32 << 8, .gain_q8 = 256, .black_code = 20,
        .clamp_min = 4, .clamp_max = 63, .filter_shift = 2,
    };
    c5vrx_cvbs_conditioner_init(&conditioner, &cfg);
    const unsigned iterations = 16u;
    size_t written = 0;
    const int64_t first_us = esp_timer_get_time();
    for (unsigned i = 0; i < iterations && err == ESP_OK; ++i) {
        err = c5vrx_wbfm_hw_transform_context(
            context, input, BENCH_WORDS, wbfm, BENCH_WORDS / 4u, &written);
        if (err == ESP_OK) {
            for (size_t n = 0; n < written; ++n)
                wbfm[n] = (uint8_t)(((unsigned)wbfm[n] + 2u) >> 2u) & 0x3fu;
            c5vrx_cvbs_condition(&conditioner, wbfm, cvbs, written, NULL, NULL);
        }
    }
    const uint64_t us = (uint64_t)(esp_timer_get_time() - first_us);
    const uint64_t iq = (uint64_t)iterations * BENCH_WORDS;
    const uint64_t input_rate = us ? iq * 1000000u / us : 0u;
    if (err == ESP_OK && input_samples_per_second)
        *input_samples_per_second = input_rate;
    printf("C5VRX_BENCH_PIPELINE effective_input_samples=%llu wbfm_output_samples=%llu cvbs_output_samples=%llu duration_us=%llu effective_input_samples_per_sec=%llu latency_us_per_block=%llu underruns=0 includes_parlio=0 classification=%s code=%d\n",
           (unsigned long long)iq,
           (unsigned long long)iterations * written,
           (unsigned long long)iterations * written,
           (unsigned long long)us,
           (unsigned long long)input_rate,
           (unsigned long long)(us / iterations),
           err == ESP_OK ? "MEASURED_ON_HARDWARE_SYNTHETIC" : "FAILED",
           (int)err);
    fflush(stdout);
    c5vrx_wbfm_hw_destroy(context);
    free(input); free(wbfm); free(cvbs);
    return err;
}
