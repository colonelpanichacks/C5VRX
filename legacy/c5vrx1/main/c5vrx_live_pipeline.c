/* SPDX-License-Identifier: GPL-3.0-only */

#include "c5vrx_live_pipeline.h"

#include <stdlib.h>
#include <string.h>

#include "c5vrx_adc_dump.h"
#include "c5vrx_wbfm_hw.h"
#include "c5vrx_cvbs_levels.h"
#include "c5vrx_cvbs_live_out.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_task_wdt.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sdkconfig.h"

static const char *TAG = "c5vrx_live";
#define FINITE_SOURCE_BUFFER_COUNT (C5VRX_RF_BLOCK_QUEUE_CAPACITY + 1u)

typedef struct {
    uint32_t *words;
    atomic_bool in_use;
} finite_source_slot_t;

typedef struct {
    size_t words_per_block;
    uint64_t sequence;
    finite_source_slot_t slot[FINITE_SOURCE_BUFFER_COUNT];
} finite_source_context_t;

typedef struct {
    TaskHandle_t task;
    TaskHandle_t source_task;
    volatile bool stop;
    c5vrx_live_pipeline_config_t config;
    c5vrx_stream_stats_t stats;
    c5vrx_cvbs_conditioner_t conditioner;
    c5vrx_cvbs_sync_tracker_t timing;
    c5vrx_cvbs_levels_t levels;
    c5vrx_wbfm_hw_context_t *wbfm_hw;
    uint8_t *wbfm;
    uint8_t *cvbs;
    uint8_t *sink_pending;
    size_t sink_pending_count;
    size_t sink_block_samples;
    c5vrx_rf_block_queue_t queue;
    uint8_t previous_wbfm;
    bool have_previous_wbfm;
    uint8_t previous_retained_phase;
    bool have_previous_retained_phase;
} live_state_t;

static live_state_t s_live;
/* Kept outside s_live so a failed task creation may clear pipeline ownership
 * without losing the information required to retry watchdog restoration. */
static TaskHandle_t s_idle_task;
static bool s_idle_wdt_removed;

static void update_max_u64(uint64_t *maximum, uint64_t value)
{
    if (value > *maximum) *maximum = value;
}

static esp_err_t suspend_idle_watchdog_for_ring(void)
{
    if (s_live.config.source->kind !=
            C5VRX_RF_SOURCE_EXPERIMENTAL_RING_UNPROVEN &&
        s_live.config.source->kind != C5VRX_RF_SOURCE_CONTINUOUS) {
        return ESP_OK;
    }
    s_idle_task = xTaskGetIdleTaskHandleForCore(xPortGetCoreID());
    if (!s_idle_task || esp_task_wdt_status(s_idle_task) != ESP_OK) {
        return ESP_OK;
    }
    const esp_err_t err = esp_task_wdt_delete(s_idle_task);
    if (err == ESP_OK) s_idle_wdt_removed = true;
    return err;
}

static esp_err_t restore_idle_watchdog(void)
{
    if (!s_idle_wdt_removed) return ESP_OK;
    const esp_err_t err = esp_task_wdt_add(s_idle_task);
    if (err == ESP_OK) {
        s_idle_wdt_removed = false;
        s_idle_task = NULL;
    }
    return err;
}

static void restore_idle_watchdog_after_last_worker(void)
{
    if (!s_live.task && !s_live.source_task) {
        const esp_err_t err = restore_idle_watchdog();
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "idle watchdog restore failed: %s",
                     esp_err_to_name(err));
        }
    }
}

static bool finite_acquire(c5vrx_rf_source_t *source,
                           c5vrx_rf_block_t *block,
                           uint32_t timeout_ms)
{
    finite_source_context_t *ctx = source ? source->context : NULL;
    if (!ctx || !block) return false;
    finite_source_slot_t *slot = NULL;
    const int64_t deadline = esp_timer_get_time() + (int64_t)timeout_ms * 1000;
    do {
        for (unsigned i = 0; i < FINITE_SOURCE_BUFFER_COUNT; ++i) {
            bool expected = false;
            if (atomic_compare_exchange_strong(&ctx->slot[i].in_use,
                                               &expected, true)) {
                slot = &ctx->slot[i];
                break;
            }
        }
        if (slot || !timeout_ms) break;
        taskYIELD();
    } while (esp_timer_get_time() < deadline);
    if (!slot) return false;
    const int64_t begin = esp_timer_get_time();
    if (c5vrx_adc_dump_capture(ctx->words_per_block, false) != ESP_OK) {
        atomic_store(&slot->in_use, false);
        return false;
    }
    volatile const uint32_t *dump =
        (volatile const uint32_t *)(uintptr_t)C5VRX_ADC_DUMP_BASE_ADDR;
    for (size_t i = 0; i < ctx->words_per_block; ++i) {
        slot->words[i] = dump[i];
    }
    *block = (c5vrx_rf_block_t) {
        .words = slot->words,
        .word_count = ctx->words_per_block,
        .sequence = ctx->sequence++,
        .capture_time_us = (uint64_t)(esp_timer_get_time() - begin),
        .discontinuity_before = true,
        .owner = slot,
    };
    return true;
}

static void finite_release(c5vrx_rf_source_t *source,
                           const c5vrx_rf_block_t *block)
{
    (void)source;
    finite_source_slot_t *slot = block ? block->owner : NULL;
    if (slot) atomic_store(&slot->in_use, false);
}

esp_err_t c5vrx_finite_chain_source_create(c5vrx_rf_source_t *source,
                                           size_t words_per_block)
{
    if (!source || words_per_block < 8u ||
        words_per_block > C5VRX_ADC_DUMP_MAX_SAMPLES ||
        (words_per_block & 3u)) {
        return ESP_ERR_INVALID_ARG;
    }
    finite_source_context_t *ctx = calloc(1, sizeof(*ctx));
    if (!ctx) return ESP_ERR_NO_MEM;
    for (unsigned i = 0; i < FINITE_SOURCE_BUFFER_COUNT; ++i) {
        ctx->slot[i].words = heap_caps_malloc(
            words_per_block * sizeof(uint32_t),
            MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);
        if (!ctx->slot[i].words) {
            while (i) free(ctx->slot[--i].words);
            free(ctx);
            return ESP_ERR_NO_MEM;
        }
        atomic_init(&ctx->slot[i].in_use, false);
    }
    ctx->words_per_block = words_per_block;
    *source = (c5vrx_rf_source_t) {
        .name = "finite-vendor-dump-chain (NOT continuous RF)",
        .kind = C5VRX_RF_SOURCE_FINITE_CHAINED,
        .nominal_sample_rate_hz = 0,
        .acquire = finite_acquire,
        .release = finite_release,
        .context = ctx,
    };
    return ESP_OK;
}

void c5vrx_finite_chain_source_destroy(c5vrx_rf_source_t *source)
{
    if (!source) return;
    finite_source_context_t *ctx = source->context;
    if (ctx) {
        for (unsigned i = 0; i < FINITE_SOURCE_BUFFER_COUNT; ++i)
            free(ctx->slot[i].words);
        free(ctx);
    }
    memset(source, 0, sizeof(*source));
}

static void source_task(void *arg)
{
    (void)arg;
    while (!s_live.stop) {
        c5vrx_rf_block_t block = {0};
        const int64_t begin = esp_timer_get_time();
        if (!s_live.config.source->acquire(s_live.config.source, &block, 20u)) {
            ++s_live.stats.source_underruns;
            if (s_live.config.source->kind ==
                C5VRX_RF_SOURCE_EXPERIMENTAL_RING_UNPROVEN) {
                c5vrx_live_ring_stats_t ring = {0};
                c5vrx_live_ring_source_get_stats(s_live.config.source, &ring);
                if (ring.fatal_stops) {
                    ESP_LOGE(TAG, "ring stopped fail-closed: %s",
                             c5vrx_live_ring_failure_name(ring.fatal_reason));
                    s_live.stop = true;
                    break;
                }
            }
            vTaskDelay(pdMS_TO_TICKS(1));
            continue;
        }
        const uint64_t duration = (uint64_t)(esp_timer_get_time() - begin);
        s_live.stats.source_time_us += duration;
        update_max_u64(&s_live.stats.source_time_max_us, duration);
        if (!c5vrx_rf_block_queue_push(&s_live.queue, &block)) {
            ++s_live.stats.dropped_rf_blocks;
            s_live.config.source->release(s_live.config.source, &block);
        } else if (s_live.task) {
            /* The AV/DSP consumer has higher priority and blocks on this
             * notification. It therefore runs exactly when a block is ready,
             * then blocks again so the writer observer can resume before an
             * RF-ring wrap becomes ambiguous. */
            xTaskNotifyGive(s_live.task);
        }
        s_live.stats.queue_occupancy = s_live.queue.occupancy;
        s_live.stats.queue_high_water_mark = s_live.queue.high_water_mark;
    }
    s_live.source_task = NULL;
    restore_idle_watchdog_after_last_worker();
    vTaskDelete(NULL);
}

static void sink_samples(const uint8_t *samples, size_t count)
{
    size_t consumed = 0;
    while (consumed < count) {
        size_t copy = s_live.sink_block_samples - s_live.sink_pending_count;
        if (copy > count - consumed) copy = count - consumed;
        memcpy(s_live.sink_pending + s_live.sink_pending_count,
               samples + consumed, copy);
        s_live.sink_pending_count += copy;
        consumed += copy;
        if (s_live.sink_pending_count != s_live.sink_block_samples) continue;

        const int64_t output_begin = esp_timer_get_time();
        const esp_err_t output_err = s_live.config.sink(
            s_live.sink_pending, s_live.sink_block_samples,
            s_live.config.sink_context);
        const uint64_t output_duration =
            (uint64_t)(esp_timer_get_time() - output_begin);
        s_live.stats.output_time_us += output_duration;
        update_max_u64(&s_live.stats.output_time_max_us, output_duration);
        if (output_err != ESP_OK) {
            ++s_live.stats.output_underruns;
            ESP_LOGE(TAG, "output stopped fail-closed: %s",
                     esp_err_to_name(output_err));
            s_live.stop = true;
        }
        s_live.sink_pending_count = 0;
    }
}

static void pipeline_task(void *arg)
{
    (void)arg;
    const int64_t start_us = esp_timer_get_time();
    ESP_LOGW(TAG, "pipeline source=%s kind=%s",
             s_live.config.source->name,
             s_live.config.source->kind == C5VRX_RF_SOURCE_CONTINUOUS
                 ? "CONTINUOUS" :
             s_live.config.source->kind == C5VRX_RF_SOURCE_EXPERIMENTAL_RING_UNPROVEN
                 ? "EXPERIMENTAL_RING_SOURCE_UNPROVEN" :
                   "FINITE/CHAINED EXPERIMENT");

    while (!s_live.stop) {
        c5vrx_rf_block_t block = {0};
        if (!c5vrx_rf_block_queue_pop(&s_live.queue, &block)) {
            (void)ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
            continue;
        }
        s_live.stats.queue_occupancy = s_live.queue.occupancy;
        if (block.discontinuity_before) {
            ++s_live.stats.discontinuities;
            c5vrx_cvbs_sync_discontinuity(&s_live.timing);
            c5vrx_cvbs_levels_reset(&s_live.levels);
            s_live.have_previous_wbfm = false;
            s_live.have_previous_retained_phase = false;
        }
        if (!block.words || block.word_count > s_live.config.maximum_input_words ||
            (block.word_count & 3u)) {
            ++s_live.stats.dropped_rf_blocks;
            s_live.config.source->release(s_live.config.source, &block);
            continue;
        }

        size_t written = 0;
        const int64_t wbfm_begin = esp_timer_get_time();
        esp_err_t err = c5vrx_wbfm_hw_transform_context(
            s_live.wbfm_hw, block.words, block.word_count, s_live.wbfm,
            s_live.config.maximum_input_words / 4u, &written);
        const uint64_t wbfm_duration =
            (uint64_t)(esp_timer_get_time() - wbfm_begin);
        s_live.stats.wbfm_time_us += wbfm_duration;
        update_max_u64(&s_live.stats.wbfm_time_max_us, wbfm_duration);
        if (err == ESP_OK && written > 0u) {
            if (c5vrx_wbfm_hw_phase_bits(s_live.wbfm_hw) == 8u) {
                for (size_t i = 0; i < written; ++i) {
                    s_live.wbfm[i] = (uint8_t)(
                        ((unsigned)s_live.wbfm[i] + 2u) >> 2u) & 0x3fu;
                }
            }
            const uint8_t first_phase =
                c5vrx_wbfm_hw_context_phase6(s_live.wbfm_hw, block.words[0]);
            if (!block.discontinuity_before &&
                s_live.have_previous_retained_phase) {
                s_live.wbfm[0] = (uint8_t)(32u + first_phase -
                    s_live.previous_retained_phase) & 0x3fu;
            } else {
                s_live.wbfm[0] = 32u;
            }
            s_live.previous_retained_phase = c5vrx_wbfm_hw_context_phase6(
                s_live.wbfm_hw, block.words[(written - 1u) * 4u]);
            s_live.have_previous_retained_phase = true;
        }
        s_live.config.source->release(s_live.config.source, &block);
        if (err != ESP_OK || written == 0u) {
            ++s_live.stats.dropped_rf_blocks;
            continue;
        }

        uint8_t wmin = 63u, wmax = 0u;
        for (size_t i = 0; i < written; ++i) {
            const uint8_t v = s_live.wbfm[i] & 0x3fu;
            if (v < wmin) wmin = v;
            if (v > wmax) wmax = v;
        }
        if (s_live.have_previous_wbfm) {
            const uint8_t first = s_live.wbfm[0] & 0x3fu;
            const uint32_t jump = s_live.previous_wbfm > first
                ? s_live.previous_wbfm - first : first - s_live.previous_wbfm;
            s_live.stats.boundary_jump_sum += jump;
            if (jump > s_live.stats.boundary_jump_max)
                s_live.stats.boundary_jump_max = jump;
        }
        s_live.previous_wbfm = s_live.wbfm[written - 1u] & 0x3fu;
        s_live.have_previous_wbfm = true;
        uint8_t cmin = 63u, cmax = 0u;
        const int64_t condition_begin = esp_timer_get_time();
        c5vrx_cvbs_condition(&s_live.conditioner, s_live.wbfm, s_live.cvbs,
                             written, &cmin, &cmax);
        c5vrx_cvbs_levels_process(
            &s_live.levels, &s_live.timing, s_live.cvbs, written);
        /* Sync/timing needs edge accuracy, not every 20 MHz sample. A fixed
         * four-sample stride preserves sub-microsecond H timing while cutting
         * observer CPU by 75%; sample positions remain in source units. */
        for (size_t i = 0; i < written; i += 4u) {
            c5vrx_cvbs_sync_event_t timing_event;
            const uint8_t stride = (uint8_t)(written - i < 4u ?
                written - i : 4u);
            (void)c5vrx_cvbs_sync_consume_stride(
                &s_live.timing, s_live.cvbs[i], stride, &timing_event);
        }
        const uint64_t conditioner_duration =
            (uint64_t)(esp_timer_get_time() - condition_begin);
        s_live.stats.conditioner_time_us += conditioner_duration;
        update_max_u64(&s_live.stats.conditioner_time_max_us,
                       conditioner_duration);

        sink_samples(s_live.cvbs, written);
        ++s_live.stats.blocks_processed;
        s_live.stats.input_samples += block.word_count;
        s_live.stats.output_samples += written;
        s_live.stats.wbfm_min = wmin;
        s_live.stats.wbfm_max = wmax;
        s_live.stats.cvbs_min = cmin;
        s_live.stats.cvbs_max = cmax;
        const uint64_t elapsed = (uint64_t)(esp_timer_get_time() - start_us);
        if (elapsed) {
            s_live.stats.achieved_input_rate_hz =
                (uint32_t)(s_live.stats.input_samples * 1000000u / elapsed);
            s_live.stats.achieved_output_rate_hz =
                (uint32_t)(s_live.stats.output_samples * 1000000u / elapsed);
        }
    }
    s_live.task = NULL;
    restore_idle_watchdog_after_last_worker();
    vTaskDelete(NULL);
}

esp_err_t c5vrx_live_pipeline_start(const c5vrx_live_pipeline_config_t *config)
{
    if (!config || !config->source || !config->source->acquire ||
        !config->source->release || !config->sink ||
        config->maximum_input_words < 8u ||
        (config->maximum_input_words & 3u) || s_live.task) {
        return ESP_ERR_INVALID_ARG;
    }
    const esp_err_t pending_watchdog_err = restore_idle_watchdog();
    if (pending_watchdog_err != ESP_OK) return pending_watchdog_err;
    memset(&s_live, 0, sizeof(s_live));
    s_live.config = *config;
    const size_t output_capacity = config->maximum_input_words / 4u;
    s_live.sink_block_samples = output_capacity;
    s_live.wbfm = heap_caps_malloc(output_capacity,
                                   MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);
    s_live.cvbs = heap_caps_malloc(output_capacity,
                                   MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);
    s_live.sink_pending = heap_caps_malloc(output_capacity,
                                           MALLOC_CAP_INTERNAL);
    esp_err_t hardware_err = c5vrx_wbfm_hw_create_kernel(
        config->maximum_input_words, config->wbfm_kernel,
        &s_live.wbfm_hw);
    if (!s_live.wbfm || !s_live.cvbs || !s_live.sink_pending ||
        hardware_err != ESP_OK) {
        free(s_live.wbfm);
        free(s_live.cvbs);
        free(s_live.sink_pending);
        c5vrx_wbfm_hw_destroy(s_live.wbfm_hw);
        memset(&s_live, 0, sizeof(s_live));
        return hardware_err != ESP_OK ? hardware_err : ESP_ERR_NO_MEM;
    }
    c5vrx_cvbs_conditioner_init(&s_live.conditioner, &config->conditioner);
    c5vrx_cvbs_sync_init(&s_live.timing);
    c5vrx_cvbs_sync_set_sample_rate(
        &s_live.timing, C5VRX_CVBS_SOURCE_SAMPLE_RATE_HZ);
    c5vrx_cvbs_levels_init(&s_live.levels);
    c5vrx_rf_block_queue_init(&s_live.queue);
    esp_err_t watchdog_err = suspend_idle_watchdog_for_ring();
    if (watchdog_err != ESP_OK) {
        free(s_live.wbfm); free(s_live.cvbs); free(s_live.sink_pending);
        c5vrx_wbfm_hw_destroy(s_live.wbfm_hw);
        memset(&s_live, 0, sizeof(s_live));
        return watchdog_err;
    }
    /* Create the consumer first and give it deadline priority over the RF
     * staging task. It immediately blocks until source_task publishes a block;
     * after one block it blocks again, producing a deterministic single-core
     * handoff instead of a permanently-runnable producer starving AV. */
    if (xTaskCreate(pipeline_task, "c5vrx_live", 4096, NULL, 19,
                    &s_live.task) != pdPASS) {
        const esp_err_t restore_err = restore_idle_watchdog();
        free(s_live.wbfm); free(s_live.cvbs); free(s_live.sink_pending);
        c5vrx_wbfm_hw_destroy(s_live.wbfm_hw);
        memset(&s_live, 0, sizeof(s_live));
        return restore_err == ESP_OK ? ESP_ERR_NO_MEM : restore_err;
    }
    if (xTaskCreate(source_task, "c5vrx_source", 3072, NULL, 18,
                    &s_live.source_task) != pdPASS) {
        s_live.stop = true;
        xTaskNotifyGive(s_live.task);
        while (s_live.task) vTaskDelay(pdMS_TO_TICKS(1));
        const esp_err_t restore_err = restore_idle_watchdog();
        free(s_live.wbfm);
        free(s_live.cvbs);
        free(s_live.sink_pending);
        c5vrx_wbfm_hw_destroy(s_live.wbfm_hw);
        memset(&s_live, 0, sizeof(s_live));
        return restore_err == ESP_OK ? ESP_ERR_NO_MEM : restore_err;
    }
    return ESP_OK;
}

esp_err_t c5vrx_live_pipeline_stop(void)
{
    if (!s_live.task && !s_live.source_task && !s_live.wbfm_hw) {
        if (s_idle_wdt_removed) return restore_idle_watchdog();
        return ESP_ERR_INVALID_STATE;
    }
    s_live.stop = true;
    if (s_live.task) xTaskNotifyGive(s_live.task);
    for (unsigned i = 0; i < 1000u && (s_live.task || s_live.source_task); ++i) {
        vTaskDelay(pdMS_TO_TICKS(1));
    }
    if (s_live.task || s_live.source_task) return ESP_ERR_TIMEOUT;
    const esp_err_t watchdog_err = restore_idle_watchdog();
    free(s_live.wbfm);
    free(s_live.cvbs);
    free(s_live.sink_pending);
    c5vrx_wbfm_hw_destroy(s_live.wbfm_hw);
    s_live.wbfm = NULL;
    s_live.cvbs = NULL;
    s_live.wbfm_hw = NULL;
    return watchdog_err;
}

bool c5vrx_live_pipeline_running(void)
{
    /* Keep ownership asserted after a fail-closed task exit until LIVE STOP
     * releases the source, RF producer and output peripheral. */
    return s_live.task != NULL || s_live.source_task != NULL ||
           s_live.wbfm_hw != NULL;
}

void c5vrx_live_pipeline_get_stats(c5vrx_stream_stats_t *stats)
{
    if (stats) *stats = s_live.stats;
}

void c5vrx_live_pipeline_get_timing(c5vrx_cvbs_sync_tracker_t *timing)
{
    if (timing) *timing = s_live.timing;
}

void c5vrx_live_pipeline_log_stats(void)
{
    c5vrx_stream_stats_t *s = &s_live.stats;
    c5vrx_cvbs_live_out_stats_t av = {0};
    c5vrx_cvbs_live_out_get_stats(&av);
    ESP_LOGI(TAG,
             "blocks=%llu src_underrun=%llu out_underrun=%llu dropped=%llu discontinuities=%llu rate_iq=%u/s rate_cvbs=%u/s wbfm=%u..%u cvbs=%u..%u stage_us total/max source=%llu/%llu wbfm=%llu/%llu condition=%llu/%llu output=%llu/%llu boundary_jump avg/max=%llu/%u queue=%u high=%u timing_epoch=%llu field=%llu standard=%u polarity=%u h_lock=%u v_lock=%u line_q8=%u av_live=%llu av_filler=%llu av_mailbox_drop=%llu av_guardian=%u",
             (unsigned long long)s->blocks_processed,
             (unsigned long long)s->source_underruns,
             (unsigned long long)s->output_underruns,
             (unsigned long long)s->dropped_rf_blocks,
             (unsigned long long)s->discontinuities,
             (unsigned)s->achieved_input_rate_hz,
             (unsigned)s->achieved_output_rate_hz,
             s->wbfm_min, s->wbfm_max, s->cvbs_min, s->cvbs_max,
             (unsigned long long)s->source_time_us,
             (unsigned long long)s->source_time_max_us,
             (unsigned long long)s->wbfm_time_us,
             (unsigned long long)s->wbfm_time_max_us,
             (unsigned long long)s->conditioner_time_us,
             (unsigned long long)s->conditioner_time_max_us,
             (unsigned long long)s->output_time_us,
             (unsigned long long)s->output_time_max_us,
             (unsigned long long)(s->blocks_processed > 1u
                 ? s->boundary_jump_sum / (s->blocks_processed - 1u) : 0u),
             (unsigned)s->boundary_jump_max,
             s->queue_occupancy, s->queue_high_water_mark,
             (unsigned long long)s_live.timing.stream_epoch,
             (unsigned long long)s_live.timing.field_id,
             (unsigned)s_live.timing.standard,
             (unsigned)s_live.timing.polarity,
             s_live.timing.horizontal_locked ? 1u : 0u,
             s_live.timing.vertical_locked ? 1u : 0u,
             (unsigned)s_live.timing.line_period_q8,
             (unsigned long long)av.live_blocks,
             (unsigned long long)av.filler_blocks,
             (unsigned long long)av.mailbox_drops,
             av.guardian_running ? 1u : 0u);
}
