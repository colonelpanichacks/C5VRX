/* SPDX-License-Identifier: GPL-3.0-only */
#include "c5vrx_cvbs_live_out.h"

#include <stdlib.h>
#include <string.h>
#include "sdkconfig.h"
#include "driver/parlio_tx.h"
#include "esp_heap_caps.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#if CONFIG_C5VRX_EXPERIMENTAL_CVBS_PARLIO
#define PAL_FRAME_HALF_LINES 1250u
#define RETIRED_0 (1u << 0)
#define RETIRED_1 (1u << 1)
#define STOP_NOTIFY (1u << 31)

typedef struct {
    parlio_tx_unit_handle_t tx;
    uint8_t *dma[2];
    uint8_t *mailbox[2];
    uint64_t mailbox_filler_start_sample[2];
    uint64_t mailbox_filler_end_sample[2];
    bool mailbox_phase_valid[2];
    bool dma_live[2];
    bool mailbox_ready[2];
    bool mailbox_in_use[2];
    unsigned mailbox_write;
    size_t samples;
    uint64_t filler_sample;
    uint64_t filler_step_q32;
    uint64_t filler_frame_q32;
    uint32_t filler_eq_phase_q32;
    uint32_t filler_broad_phase_q32;
    uint32_t filler_hsync_phase_q32;
    uint64_t pending_live_filler_start_sample;
    uint64_t pending_live_filler_end_sample;
    bool pending_live_phase_valid;
    uint64_t timing_anchor_source_sample;
    uint64_t timing_anchor_phase_q32;
    uint64_t timing_source_step_q32;
    uint64_t timing_stream_epoch;
    bool timing_anchor_valid;
    uint64_t live_blocks;
    uint64_t live_blocks_retired;
    uint64_t live_retirements_completed;
    uint64_t filler_blocks;
    uint64_t mailbox_drops;
    uint64_t qualification_underruns;
    uint64_t guardian_failures;
    uint64_t phase_mismatch_drops;
    uint32_t qualification_unsubmitted;
    TaskHandle_t guardian;
    portMUX_TYPE lock;
    bool running;
    bool realign_required;
    c5vrx_video_standard_t filler_standard;
    uint32_t clock_hz;
} live_out_state_t;

static live_out_state_t s_out = {.lock = portMUX_INITIALIZER_UNLOCKED};

typedef struct {
    bool valid;
    c5vrx_video_standard_t standard;
    uint64_t initial_filler_sample;
    uint64_t first_live_start_sample;
    uint64_t first_live_end_sample;
    const uint8_t *first_live_samples;
    uint64_t timing_source_sample;
    uint64_t timing_source_step_q32;
    uint64_t timing_stream_epoch;
} aligned_start_t;

static aligned_start_t s_aligned_start;

static uint32_t pulse_phase_q32(uint32_t duration_numerator,
                                uint32_t duration_denominator,
                                uint32_t line_rate_numerator,
                                uint32_t line_rate_denominator)
{
    const uint64_t denominator =
        (uint64_t)duration_denominator * line_rate_denominator;
    uint64_t remainder =
        (uint64_t)duration_numerator * 2u * line_rate_numerator;
    uint32_t quotient = 0u;
    /* All supported pulse widths are shorter than a half-line, so remainder
     * starts below denominator. Binary long division produces floor(A*2^32/B)
     * exactly without a target-dependent 128-bit integer type. This runs only
     * three times while starting AV, never in the sample loop. */
    for (unsigned bit = 0u; bit < 32u; ++bit) {
        remainder <<= 1u;
        quotient <<= 1u;
        if (remainder >= denominator) {
            remainder -= denominator;
            quotient |= 1u;
        }
    }
    return quotient;
}

static bool filler_coordinate(const c5vrx_cvbs_sync_tracker_t *timing,
                              uint32_t output_clock_hz,
                              uint64_t *end_sample,
                              uint64_t *frame_samples,
                              uint64_t *step_q32)
{
    if (!timing || !output_clock_hz || !timing->horizontal_locked ||
        !timing->vertical_locked || !timing->last_vsync_start ||
        timing->standard == C5VRX_VIDEO_STANDARD_UNKNOWN) return false;
    const bool ntsc = timing->standard == C5VRX_VIDEO_STANDARD_NTSC;
    const uint32_t line_rate_num = ntsc ? 15750000u : 15625u;
    const uint32_t line_rate_den = ntsc ? 1001u : 1u;
    const uint32_t equalizing_halves = ntsc ? 6u : 5u;
    const uint32_t field_half_lines = ntsc ? 525u : 625u;
    const uint32_t frame_half_lines = field_half_lines * 2u;
    const uint64_t since_vsync = timing->samples_seen >=
        timing->last_vsync_start ?
        timing->samples_seen - timing->last_vsync_start : 0u;
    const uint32_t source_rate = timing->sample_rate_hz ?
        timing->sample_rate_hz : C5VRX_CVBS_SOURCE_SAMPLE_RATE_HZ;
    const uint64_t step =
        (((uint64_t)2u * line_rate_num) << 32u) /
        ((uint64_t)line_rate_den * output_clock_hz);
    const uint64_t field_phase = timing->odd_field ? 0u :
        (uint64_t)field_half_lines << 32u;
    const uint64_t broad_pulse_phase = field_phase +
        ((uint64_t)equalizing_halves << 32u);
    const uint64_t frame = (uint64_t)frame_half_lines << 32u;
    const uint64_t source_step =
        (((uint64_t)2u * line_rate_num) << 32u) /
        ((uint64_t)line_rate_den * source_rate);
    if (end_sample) *end_sample =
        (broad_pulse_phase + since_vsync * source_step) % frame;
    if (frame_samples) *frame_samples = frame;
    if (step_q32) *step_q32 = step;
    return true;
}

static uint8_t legal_filler_sample(uint64_t sample)
{
    const bool ntsc = s_out.filler_standard == C5VRX_VIDEO_STANDARD_NTSC;
    const uint32_t half = (uint32_t)(sample >> 32u);
    const uint32_t phase = (uint32_t)sample;
    const uint32_t local = half % (ntsc ? 525u : 625u);
    const uint32_t equalizing_halves = ntsc ? 6u : 5u;
    uint32_t pulse = 0u;
    if (local < equalizing_halves ||
        (local >= 2u * equalizing_halves &&
         local < 3u * equalizing_halves)) pulse = s_out.filler_eq_phase_q32;
    else if (local >= equalizing_halves &&
             local < 2u * equalizing_halves)
        pulse = s_out.filler_broad_phase_q32;
    else if ((half & 1u) == 0u) pulse = s_out.filler_hsync_phase_q32;
    return phase < pulse ? 0u : 19u;
}

static void fill_legal_filler(uint8_t *buffer)
{
    for (size_t i = 0; i < s_out.samples; ++i)
    {
        buffer[i] = legal_filler_sample(s_out.filler_sample);
        s_out.filler_sample += s_out.filler_step_q32;
        if (s_out.filler_sample >= s_out.filler_frame_q32)
            s_out.filler_sample -= s_out.filler_frame_q32;
    }
}

static esp_err_t queue_dma(unsigned index)
{
    const parlio_transmit_config_t cfg = {
        .idle_value = 19u,
        .flags = {.queue_nonblocking = 0, .loop_transmission = 0},
    };
    return parlio_tx_unit_transmit(
        s_out.tx, s_out.dma[index], s_out.samples * 8u, &cfg);
}

static bool on_switched(parlio_tx_unit_handle_t unit,
                        const parlio_tx_buffer_switched_event_data_t *event,
                        void *context)
{
    (void)unit; (void)context;
    if (!event || !s_out.guardian) return false;
    const uint32_t value = event->old_buffer_addr == s_out.dma[0] ? RETIRED_0 :
        event->old_buffer_addr == s_out.dma[1] ? RETIRED_1 : 0u;
    if (!value) return false;
    BaseType_t wake = pdFALSE;
    xTaskNotifyFromISR(s_out.guardian, value, eSetBits, &wake);
    return wake == pdTRUE;
}

static bool take_live_block(uint8_t *destination, uint64_t *filler_end_sample)
{
    int found = -1;
    const uint64_t frame_samples = s_out.filler_frame_q32;
    taskENTER_CRITICAL(&s_out.lock);
    for (unsigned n = 0; n < 2u; ++n) {
        const unsigned index = (s_out.mailbox_write + n) & 1u;
        if (!s_out.mailbox_ready[index] || s_out.mailbox_in_use[index]) continue;
        if (s_out.mailbox_phase_valid[index] && frame_samples &&
            s_out.mailbox_filler_start_sample[index] % frame_samples !=
                s_out.filler_sample % frame_samples) {
            /* Never splice a newly resumed live block onto a queued filler
             * tail at a different H/V coordinate. Drop this stale block so
             * the producer can publish a phase-current replacement; legal
             * filler remains continuous meanwhile. */
            s_out.mailbox_ready[index] = false;
            ++s_out.phase_mismatch_drops;
            s_out.realign_required = true;
            continue;
        }
        s_out.mailbox_ready[index] = false;
        s_out.mailbox_in_use[index] = true;
        found = (int)index;
        break;
    }
    taskEXIT_CRITICAL(&s_out.lock);
    if (found < 0) return false;
    memcpy(destination, s_out.mailbox[found], s_out.samples);
    taskENTER_CRITICAL(&s_out.lock);
    if (filler_end_sample)
        *filler_end_sample = s_out.mailbox_filler_end_sample[found];
    s_out.mailbox_in_use[found] = false;
    taskEXIT_CRITICAL(&s_out.lock);
    return true;
}

static void guardian_task(void *arg)
{
    (void)arg;
    while (s_out.running) {
        uint32_t retired = 0u;
        if (xTaskNotifyWait(0u, UINT32_MAX, &retired, portMAX_DELAY) != pdTRUE)
            continue;
        if (retired & STOP_NOTIFY) break;
        for (unsigned index = 0; index < 2u; ++index) {
            if (!(retired & (1u << index))) continue;
            const bool retired_live = s_out.dma_live[index];
            if (retired_live) ++s_out.live_blocks_retired;
            uint64_t live_filler_end = 0u;
            s_out.dma_live[index] =
                take_live_block(s_out.dma[index], &live_filler_end);
            if (s_out.dma_live[index]) {
                ++s_out.live_blocks;
                /* This is the tail of the buffer just appended to PARLIO's
                 * actual queue. Its coordinate is already phase-anchored to
                 * detected vertical timing, so fallback continues at the
                 * same H/V phase instead of treating RF capture start as a
                 * synthetic frame boundary. */
                s_out.filler_sample = live_filler_end ? live_filler_end :
                    (s_out.filler_sample +
                     s_out.samples * s_out.filler_step_q32) %
                        s_out.filler_frame_q32;
            }
            else {
                fill_legal_filler(s_out.dma[index]);
                ++s_out.filler_blocks;
                taskENTER_CRITICAL(&s_out.lock);
                if (s_out.qualification_unsubmitted)
                    ++s_out.qualification_underruns;
                taskEXIT_CRITICAL(&s_out.lock);
            }
            if (queue_dma(index) != ESP_OK) {
                ++s_out.guardian_failures;
                s_out.running = false;
            }
            /* Publish completion only after the replacement queue result and
             * its failure telemetry are visible. Qualification waiting on
             * this counter cannot race the post-retirement requeue. */
            if (retired_live) ++s_out.live_retirements_completed;
        }
    }
    s_out.guardian = NULL;
    vTaskDelete(NULL);
}

esp_err_t c5vrx_cvbs_live_out_start(size_t block_samples)
{
    return c5vrx_cvbs_live_out_start_at_rate(
        block_samples, C5VRX_CVBS_SOURCE_SAMPLE_RATE_HZ);
}

esp_err_t c5vrx_cvbs_live_out_start_aligned(
    const uint8_t *first_live_samples, size_t block_samples,
    const c5vrx_cvbs_sync_tracker_t *first_live_timing)
{
    if (s_out.tx || !first_live_samples || !block_samples)
        return ESP_ERR_INVALID_STATE;
    uint64_t live_end = 0u, frame = 0u, step = 0u;
    if (!filler_coordinate(first_live_timing,
            C5VRX_CVBS_SOURCE_SAMPLE_RATE_HZ, &live_end, &frame, &step))
        return ESP_ERR_INVALID_ARG;
    const uint64_t live_start =
        (live_end + frame - (block_samples * step) % frame) % frame;
    const uint64_t queued_filler =
        (2u * block_samples * step) % frame;
    s_aligned_start = (aligned_start_t) {
        .valid = true,
        .standard = first_live_timing->standard,
        .initial_filler_sample =
            (live_start + frame - queued_filler) % frame,
        .first_live_start_sample = live_start,
        .first_live_end_sample = live_end,
        .first_live_samples = first_live_samples,
        .timing_source_sample = first_live_timing->samples_seen,
        .timing_source_step_q32 =
            (((uint64_t)2u *
              (first_live_timing->standard == C5VRX_VIDEO_STANDARD_NTSC ?
                   15750000u : 15625u)) << 32u) /
            ((uint64_t)(first_live_timing->standard ==
                 C5VRX_VIDEO_STANDARD_NTSC ? 1001u : 1u) *
             (first_live_timing->sample_rate_hz ?
                  first_live_timing->sample_rate_hz :
                  C5VRX_CVBS_SOURCE_SAMPLE_RATE_HZ)),
        .timing_stream_epoch = first_live_timing->stream_epoch,
    };
    return c5vrx_cvbs_live_out_start(block_samples);
}

bool c5vrx_cvbs_live_out_running(void)
{
    return s_out.running && s_out.tx != NULL;
}

bool c5vrx_cvbs_live_out_take_realign_request(void)
{
    bool required;
    taskENTER_CRITICAL(&s_out.lock);
    required = s_out.realign_required;
    s_out.realign_required = false;
    taskEXIT_CRITICAL(&s_out.lock);
    return required;
}

esp_err_t c5vrx_cvbs_live_out_start_at_rate(size_t block_samples,
                                            uint32_t output_clock_hz)
{
    if (s_out.tx) return ESP_ERR_INVALID_STATE;
    if (!block_samples || output_clock_hz < 16000000u ||
        output_clock_hz > 30000000u) return ESP_ERR_INVALID_ARG;
    const aligned_start_t aligned = s_aligned_start;
    s_aligned_start.valid = false;
    s_out.samples = block_samples;
    s_out.clock_hz = output_clock_hz;
    s_out.live_blocks = s_out.live_blocks_retired = 0u;
    s_out.live_retirements_completed = 0u;
    s_out.filler_blocks = s_out.mailbox_drops = 0u;
    s_out.qualification_underruns = 0u;
    s_out.guardian_failures = 0u;
    s_out.qualification_unsubmitted = 0u;
    s_out.realign_required = false;
    s_out.filler_sample = aligned.valid ? aligned.initial_filler_sample : 0u;
    s_out.pending_live_filler_start_sample = aligned.valid ?
        aligned.first_live_start_sample : 0u;
    s_out.pending_live_filler_end_sample = aligned.valid ?
        aligned.first_live_end_sample : 0u;
    s_out.pending_live_phase_valid = aligned.valid;
    s_out.timing_anchor_valid = aligned.valid;
    s_out.timing_anchor_source_sample = aligned.timing_source_sample;
    s_out.timing_anchor_phase_q32 = aligned.first_live_end_sample;
    s_out.timing_source_step_q32 = aligned.timing_source_step_q32;
    s_out.timing_stream_epoch = aligned.timing_stream_epoch;
    s_out.mailbox_write = 0u;
    s_out.filler_standard = aligned.valid ?
        aligned.standard : C5VRX_VIDEO_STANDARD_PAL;
    const bool filler_ntsc =
        s_out.filler_standard == C5VRX_VIDEO_STANDARD_NTSC;
    const uint32_t filler_line_rate_num =
        filler_ntsc ? 15750000u : 15625u;
    const uint32_t filler_line_rate_den = filler_ntsc ? 1001u : 1u;
    s_out.filler_step_q32 =
        (((uint64_t)2u * filler_line_rate_num) << 32u) /
        ((uint64_t)filler_line_rate_den * output_clock_hz);
    s_out.filler_frame_q32 =
        (uint64_t)(filler_ntsc ? 1050u : PAL_FRAME_HALF_LINES) << 32u;
    s_out.filler_eq_phase_q32 = pulse_phase_q32(
        235u, 100000000u, filler_line_rate_num, filler_line_rate_den);
    s_out.filler_broad_phase_q32 = pulse_phase_q32(
        273u, 10000000u, filler_line_rate_num, filler_line_rate_den);
    s_out.filler_hsync_phase_q32 = pulse_phase_q32(
        47u, 10000000u, filler_line_rate_num, filler_line_rate_den);
    for (unsigned i = 0; i < 2u; ++i) {
        s_out.dma_live[i] = false;
        s_out.mailbox_ready[i] = false;
        s_out.mailbox_in_use[i] = false;
        s_out.mailbox_phase_valid[i] = false;
        s_out.dma[i] = heap_caps_malloc(
            block_samples, MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);
        s_out.mailbox[i] = heap_caps_malloc(block_samples, MALLOC_CAP_INTERNAL);
        if (!s_out.dma[i] || !s_out.mailbox[i]) {
            c5vrx_cvbs_live_out_stop();
            return ESP_ERR_NO_MEM;
        }
        fill_legal_filler(s_out.dma[i]);
    }
    if (aligned.valid) {
        memcpy(s_out.mailbox[0], aligned.first_live_samples, block_samples);
        s_out.mailbox_filler_start_sample[0] =
            aligned.first_live_start_sample;
        s_out.mailbox_filler_end_sample[0] = aligned.first_live_end_sample;
        s_out.mailbox_phase_valid[0] = true;
        s_out.mailbox_ready[0] = true;
        s_out.mailbox_write = 1u;
    }
    const parlio_tx_unit_config_t config = {
        .clk_src = PARLIO_CLK_SRC_DEFAULT, .clk_in_gpio_num = -1,
        .output_clk_freq_hz = output_clock_hz, .data_width = 8,
        .data_gpio_nums = {CONFIG_C5VRX_CVBS_D0_GPIO, CONFIG_C5VRX_CVBS_D1_GPIO,
            CONFIG_C5VRX_CVBS_D2_GPIO, CONFIG_C5VRX_CVBS_D3_GPIO,
            CONFIG_C5VRX_CVBS_D4_GPIO, CONFIG_C5VRX_CVBS_D5_GPIO,
            CONFIG_C5VRX_CVBS_D6_GPIO, CONFIG_C5VRX_CVBS_D7_GPIO},
        .clk_out_gpio_num = -1, .valid_gpio_num = -1,
        .trans_queue_depth = 2, .max_transfer_size = block_samples,
        .dma_burst_size = 32, .shift_edge = PARLIO_SHIFT_EDGE_NEG,
        .bit_pack_order = PARLIO_BIT_PACK_ORDER_LSB,
    };
    esp_err_t err = parlio_new_tx_unit(&config, &s_out.tx);
    const parlio_tx_event_callbacks_t callbacks = {.on_buffer_switched = on_switched};
    if (err == ESP_OK)
        err = parlio_tx_unit_register_event_callbacks(s_out.tx, &callbacks, NULL);
    if (err == ESP_OK) err = parlio_tx_unit_enable(s_out.tx);
    s_out.running = err == ESP_OK;
    if (err == ESP_OK && xTaskCreate(guardian_task, "c5vrx_av_guard", 3072,
            NULL, 20, &s_out.guardian) != pdPASS) err = ESP_ERR_NO_MEM;
    if (err == ESP_OK) err = queue_dma(0u);
    if (err == ESP_OK) err = queue_dma(1u);
    if (err != ESP_OK) c5vrx_cvbs_live_out_stop();
    return err;
}

esp_err_t c5vrx_cvbs_live_out_write(const uint8_t *samples, size_t count,
                                    void *context)
{
    (void)context;
    if (!s_out.running || !samples || count != s_out.samples)
        return ESP_ERR_INVALID_ARG;
    int index = -1;
    taskENTER_CRITICAL(&s_out.lock);
    for (unsigned n = 0; n < 2u; ++n) {
        const unsigned candidate = (s_out.mailbox_write + n) & 1u;
        if (!s_out.mailbox_ready[candidate] &&
            !s_out.mailbox_in_use[candidate]) {
            index = (int)candidate;
            s_out.mailbox_in_use[candidate] = true;
            break;
        }
    }
    if (index < 0) ++s_out.mailbox_drops;
    taskEXIT_CRITICAL(&s_out.lock);
    if (index < 0) return ESP_OK;
    memcpy(s_out.mailbox[index], samples, count);
    taskENTER_CRITICAL(&s_out.lock);
    s_out.mailbox_ready[index] = true;
    s_out.mailbox_filler_start_sample[index] =
        s_out.pending_live_filler_start_sample;
    s_out.mailbox_filler_end_sample[index] =
        s_out.pending_live_filler_end_sample;
    s_out.mailbox_phase_valid[index] = s_out.pending_live_phase_valid;
    s_out.mailbox_in_use[index] = false;
    s_out.mailbox_write = (unsigned)index ^ 1u;
    taskEXIT_CRITICAL(&s_out.lock);
    return ESP_OK;
}

esp_err_t c5vrx_cvbs_live_out_write_wait(const uint8_t *samples, size_t count,
                                         uint32_t timeout_ms)
{
    if (!s_out.running || !samples || count != s_out.samples)
        return ESP_ERR_INVALID_ARG;
    const TickType_t start = xTaskGetTickCount();
    const TickType_t timeout = pdMS_TO_TICKS(timeout_ms);
    for (;;) {
        int index = -1;
        taskENTER_CRITICAL(&s_out.lock);
        for (unsigned n = 0; n < 2u; ++n) {
            const unsigned candidate = (s_out.mailbox_write + n) & 1u;
            if (!s_out.mailbox_ready[candidate] &&
                !s_out.mailbox_in_use[candidate]) {
                index = (int)candidate;
                s_out.mailbox_in_use[candidate] = true;
                break;
            }
        }
        taskEXIT_CRITICAL(&s_out.lock);
        if (index >= 0) {
            memcpy(s_out.mailbox[index], samples, count);
            taskENTER_CRITICAL(&s_out.lock);
            s_out.mailbox_ready[index] = true;
            s_out.mailbox_filler_start_sample[index] =
                s_out.pending_live_filler_start_sample;
            s_out.mailbox_filler_end_sample[index] =
                s_out.pending_live_filler_end_sample;
            s_out.mailbox_phase_valid[index] = s_out.pending_live_phase_valid;
            s_out.mailbox_in_use[index] = false;
            s_out.mailbox_write = (unsigned)index ^ 1u;
            if (s_out.qualification_unsubmitted)
                --s_out.qualification_unsubmitted;
            taskEXIT_CRITICAL(&s_out.lock);
            return ESP_OK;
        }
        if ((xTaskGetTickCount() - start) >= timeout) return ESP_ERR_TIMEOUT;
        taskYIELD();
    }
}

void c5vrx_cvbs_live_out_qualification_begin(uint32_t blocks)
{
    taskENTER_CRITICAL(&s_out.lock);
    s_out.qualification_unsubmitted = blocks;
    taskEXIT_CRITICAL(&s_out.lock);
}

void c5vrx_cvbs_live_out_qualification_end(void)
{
    taskENTER_CRITICAL(&s_out.lock);
    s_out.qualification_unsubmitted = 0u;
    taskEXIT_CRITICAL(&s_out.lock);
}

esp_err_t c5vrx_cvbs_live_out_stop(void)
{
    s_out.running = false;
    if (s_out.guardian) xTaskNotify(s_out.guardian, STOP_NOTIFY, eSetBits);
    for (unsigned n = 0; n < 100u && s_out.guardian; ++n) vTaskDelay(1);
    if (s_out.guardian) return ESP_ERR_TIMEOUT;
    if (s_out.tx) {
        (void)parlio_tx_unit_disable(s_out.tx);
        (void)parlio_del_tx_unit(s_out.tx);
        s_out.tx = NULL;
    }
    for (unsigned i = 0; i < 2u; ++i) {
        free(s_out.dma[i]); free(s_out.mailbox[i]);
        s_out.dma[i] = s_out.mailbox[i] = NULL;
    }
    s_out.samples = 0u;
    s_out.clock_hz = 0u;
    s_out.filler_sample = 0u;
    s_out.filler_step_q32 = 0u;
    s_out.filler_frame_q32 = 0u;
    s_out.pending_live_filler_start_sample = 0u;
    s_out.pending_live_filler_end_sample = 0u;
    s_out.pending_live_phase_valid = false;
    s_out.timing_anchor_valid = false;
    s_out.mailbox_ready[0] = s_out.mailbox_ready[1] = false;
    s_out.mailbox_in_use[0] = s_out.mailbox_in_use[1] = false;
    return ESP_OK;
}

void c5vrx_cvbs_live_out_get_stats(c5vrx_cvbs_live_out_stats_t *stats)
{
    if (!stats) return;
    *stats = (c5vrx_cvbs_live_out_stats_t) {
        .live_blocks = s_out.live_blocks,
        .live_blocks_retired = s_out.live_blocks_retired,
        .live_retirements_completed = s_out.live_retirements_completed,
        .filler_blocks = s_out.filler_blocks,
        .mailbox_drops = s_out.mailbox_drops,
        .qualification_underruns = s_out.qualification_underruns,
        .guardian_failures = s_out.guardian_failures,
        .phase_mismatch_drops = s_out.phase_mismatch_drops,
        .guardian_running = s_out.guardian != NULL,
    };
}

void c5vrx_cvbs_live_out_update_timing(
    const c5vrx_cvbs_sync_tracker_t *timing)
{
    uint64_t filler_end = 0u, frame = 0u, step = 0u;
    if (!s_out.running || !filler_coordinate(
            timing, s_out.clock_hz, &filler_end, &frame, &step)) return;
    const bool ntsc = timing->standard == C5VRX_VIDEO_STANDARD_NTSC;
    const uint32_t line_rate_num = ntsc ? 15750000u : 15625u;
    const uint32_t line_rate_den = ntsc ? 1001u : 1u;
    const uint32_t source_rate = timing->sample_rate_hz ?
        timing->sample_rate_hz : C5VRX_CVBS_SOURCE_SAMPLE_RATE_HZ;
    const uint64_t source_step =
        (((uint64_t)2u * line_rate_num) << 32u) /
        ((uint64_t)line_rate_den * source_rate);
    taskENTER_CRITICAL(&s_out.lock);
    if (!s_out.timing_anchor_valid ||
        s_out.timing_stream_epoch != timing->stream_epoch ||
        s_out.filler_standard != timing->standard) {
        if (s_out.timing_anchor_valid) s_out.realign_required = true;
        s_out.timing_anchor_valid = true;
        s_out.timing_anchor_source_sample = timing->samples_seen;
        s_out.timing_anchor_phase_q32 = filler_end;
        s_out.timing_source_step_q32 = source_step;
        s_out.timing_stream_epoch = timing->stream_epoch;
    } else {
        const uint64_t elapsed = timing->samples_seen >=
            s_out.timing_anchor_source_sample ?
            timing->samples_seen - s_out.timing_anchor_source_sample : 0u;
        filler_end = (s_out.timing_anchor_phase_q32 +
            elapsed * s_out.timing_source_step_q32) % frame;
        /* Rebase every block so the multiply stays bounded indefinitely;
         * unlike a V-sync reanchor, this preserves the accumulated fraction. */
        s_out.timing_anchor_source_sample = timing->samples_seen;
        s_out.timing_anchor_phase_q32 = filler_end;
    }
    const uint64_t filler_start =
        (filler_end + frame - (s_out.samples * step) % frame) % frame;
    /* Attach phase-anchored canonical time to the next live mailbox block.
     * The guardian applies it only when that block enters actual PARLIO queue
     * order; producer-ahead timing can therefore never move the live cursor. */
    s_out.pending_live_filler_start_sample = filler_start;
    s_out.pending_live_filler_end_sample = filler_end;
    s_out.pending_live_phase_valid = true;
    taskEXIT_CRITICAL(&s_out.lock);
}
#else
esp_err_t c5vrx_cvbs_live_out_start(size_t n) { (void)n; return ESP_ERR_NOT_SUPPORTED; }
esp_err_t c5vrx_cvbs_live_out_start_at_rate(size_t n, uint32_t r)
{ (void)n; (void)r; return ESP_ERR_NOT_SUPPORTED; }
esp_err_t c5vrx_cvbs_live_out_start_aligned(
    const uint8_t *s, size_t n, const c5vrx_cvbs_sync_tracker_t *t)
{ (void)s; (void)n; (void)t; return ESP_ERR_NOT_SUPPORTED; }
bool c5vrx_cvbs_live_out_running(void) { return false; }
bool c5vrx_cvbs_live_out_take_realign_request(void) { return false; }
esp_err_t c5vrx_cvbs_live_out_write(const uint8_t *s, size_t n, void *c)
{ (void)s; (void)n; (void)c; return ESP_ERR_NOT_SUPPORTED; }
esp_err_t c5vrx_cvbs_live_out_write_wait(const uint8_t *s, size_t n, uint32_t t)
{ (void)s; (void)n; (void)t; return ESP_ERR_NOT_SUPPORTED; }
void c5vrx_cvbs_live_out_qualification_begin(uint32_t n) { (void)n; }
void c5vrx_cvbs_live_out_qualification_end(void) {}
esp_err_t c5vrx_cvbs_live_out_stop(void) { return ESP_OK; }
void c5vrx_cvbs_live_out_get_stats(c5vrx_cvbs_live_out_stats_t *stats)
{ if (stats) memset(stats, 0, sizeof(*stats)); }
void c5vrx_cvbs_live_out_update_timing(
    const c5vrx_cvbs_sync_tracker_t *timing) { (void)timing; }
#endif
