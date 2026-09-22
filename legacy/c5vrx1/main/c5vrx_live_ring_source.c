/* SPDX-License-Identifier: GPL-3.0-only */

#include "c5vrx_live_pipeline.h"

#include <stdatomic.h>
#include <stdlib.h>
#include <string.h>

#include "c5vrx_adc_dump.h"
#include "c5vrx_ring_tracker.h"
#include "esp_cpu.h"
#include "esp_heap_caps.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sdkconfig.h"

#define RING_WORDS C5VRX_ADC_DUMP_MAX_SAMPLES
#define RING_MASK  (RING_WORDS - 1u)
#define RING_BUFFER_COUNT (C5VRX_RF_BLOCK_QUEUE_CAPACITY + 1u)
#define RING_ABSOLUTE_MAX_HZ 320000000u

typedef struct {
    uint32_t *words;
    atomic_bool in_use;
} ring_slot_t;

typedef struct {
    size_t maximum_words;
    size_t guard_words;
    bool reader_valid;
    bool armed;
    bool discontinuity;
    bool fatal;
    bool stop_attempted;
    esp_err_t stop_result;
    uint32_t maximum_plausible_rate_hz;
    uint64_t sequence;
    c5vrx_ring_tracker_t tracker;
    ring_slot_t slots[RING_BUFFER_COUNT];
    c5vrx_live_ring_stats_t stats;
} ring_context_t;

static volatile const uint32_t *const s_ring =
    (volatile const uint32_t *)(uintptr_t)C5VRX_ADC_DUMP_BASE_ADDR;

static uint32_t cpu_hz(void)
{
    return (uint32_t)CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ * 1000000u;
}

static ring_slot_t *claim_slot(ring_context_t *ctx, uint32_t timeout_ms)
{
    const int64_t deadline = esp_timer_get_time() + (int64_t)timeout_ms * 1000;
    do {
        for (unsigned i = 0; i < RING_BUFFER_COUNT; ++i) {
            bool expected = false;
            if (atomic_compare_exchange_strong(&ctx->slots[i].in_use,
                                               &expected, true)) {
                return &ctx->slots[i];
            }
        }
        if (!timeout_ms) break;
        taskYIELD();
    } while (esp_timer_get_time() < deadline);
    return NULL;
}

static void sync_tracker_stats(ring_context_t *ctx)
{
    ctx->stats.producer_absolute = ctx->tracker.producer_absolute;
    ctx->stats.consumer_absolute = ctx->tracker.consumer_absolute;
    ctx->stats.lag_words = c5vrx_ring_tracker_lag(&ctx->tracker);
    ctx->stats.maximum_lag_words = ctx->tracker.maximum_lag_words;
    ctx->stats.wraps_observed = ctx->tracker.wraps;
    ctx->stats.maximum_service_interval_cycles =
        ctx->tracker.maximum_service_interval_cycles;
    ctx->stats.minimum_service_deadline_cycles =
        ctx->tracker.minimum_deadline_cycles == UINT32_MAX ? 0u :
        ctx->tracker.minimum_deadline_cycles;
    if (ctx->stats.maximum_service_interval_cycles) {
        ctx->stats.deadline_headroom_x1000 = (uint32_t)(
            (uint64_t)ctx->stats.minimum_service_deadline_cycles * 1000u /
            ctx->stats.maximum_service_interval_cycles);
    }
}

static void fail_ring(ring_context_t *ctx, ring_slot_t *slot,
                      c5vrx_live_ring_failure_t reason, uint64_t missed)
{
    if (slot) atomic_store(&slot->in_use, false);
    ctx->stats.missed_words += missed;
    ++ctx->stats.overruns;
    ++ctx->stats.fatal_stops;
    ++ctx->stats.stream_epoch;
    ctx->stats.fatal_reason = reason;
    ctx->fatal = true;
    sync_tracker_stats(ctx);
    if (ctx->armed) {
        ctx->stop_result = c5vrx_rf_dump_stop();
        ctx->stop_attempted = true;
    }
    ctx->armed = false;
}

static c5vrx_live_ring_failure_t tracker_failure(
    c5vrx_ring_track_result_t result, bool during_copy)
{
    if (result == C5VRX_RING_TRACK_POINTER_OUT_OF_RANGE)
        return C5VRX_RING_FAILURE_POINTER_OUT_OF_RANGE;
    if (result == C5VRX_RING_TRACK_INTERVAL_AMBIGUOUS)
        return during_copy ? C5VRX_RING_FAILURE_COPY_AMBIGUOUS :
                             C5VRX_RING_FAILURE_SERVICE_INTERVAL_AMBIGUOUS;
    return C5VRX_RING_FAILURE_READER_INSIDE_GUARD;
}

/* Keep observing while the reader waits for newly-written words. Returning to
 * source_task would introduce its 1 ms backoff, which is longer than a full
 * 16K-word wrap at the expected producer rates and would make the writer
 * position fundamentally ambiguous. This wait is bounded by the acquire
 * timeout and continuously refreshes the wrap tracker. */
static bool observe_until_lag(ring_context_t *ctx, ring_slot_t *slot,
                              uint64_t required_lag, uint32_t timeout_ms)
{
    const uint32_t start_cycle = (uint32_t)esp_cpu_get_cycle_count();
    uint64_t budget = (uint64_t)timeout_ms * cpu_hz() / 1000u;
    if (budget > UINT32_MAX) budget = UINT32_MAX;

    while (c5vrx_ring_tracker_lag(&ctx->tracker) < required_lag) {
        c5vrx_rf_dump_status_t status = {0};
        if (c5vrx_rf_dump_get_status(&status) != ESP_OK || !status.enabled) {
            ++ctx->stats.dropped_blocks;
            fail_ring(ctx, slot, C5VRX_RING_FAILURE_PRODUCER_STOPPED, 0u);
            return false;
        }
        const uint32_t cycle = (uint32_t)esp_cpu_get_cycle_count();
        const c5vrx_ring_track_result_t tracked =
            c5vrx_ring_tracker_observe(&ctx->tracker, status.pointer, cycle);
        if (tracked != C5VRX_RING_TRACK_OK) {
            fail_ring(ctx, slot, tracker_failure(tracked, false),
                      c5vrx_ring_tracker_lag(&ctx->tracker));
            return false;
        }
        if (c5vrx_ring_tracker_lag(&ctx->tracker) >= required_lag) return true;
        if (!timeout_ms || (uint32_t)(cycle - start_cycle) >= (uint32_t)budget) {
            atomic_store(&slot->in_use, false);
            return false;
        }
    }
    return true;
}

static bool ring_acquire(c5vrx_rf_source_t *source,
                         c5vrx_rf_block_t *block,
                         uint32_t timeout_ms)
{
    ring_context_t *ctx = source ? source->context : NULL;
    if (!ctx || !block || ctx->fatal) return false;
    ring_slot_t *slot = claim_slot(ctx, timeout_ms);
    if (!slot) {
        ++ctx->stats.dropped_blocks;
        return false;
    }

    if (!ctx->armed) {
        if (c5vrx_rf_dump_start() != ESP_OK) {
            fail_ring(ctx, slot, C5VRX_RING_FAILURE_PRODUCER_STOPPED, 0u);
            return false;
        }
        ctx->armed = true;
    }
    if (!c5vrx_rf_dump_canaries_intact()) {
        fail_ring(ctx, slot, C5VRX_RING_FAILURE_ADJACENT_MEMORY_CORRUPTED, 0u);
        return false;
    }

    c5vrx_rf_dump_status_t before = {0};
    if (c5vrx_rf_dump_get_status(&before) != ESP_OK || !before.enabled) {
        ++ctx->stats.dropped_blocks;
        fail_ring(ctx, slot, C5VRX_RING_FAILURE_PRODUCER_STOPPED, 0u);
        return false;
    }
    const uint32_t observation_cycle = (uint32_t)esp_cpu_get_cycle_count();
    const c5vrx_ring_track_result_t before_track =
        c5vrx_ring_tracker_observe(&ctx->tracker, before.pointer,
                                   observation_cycle);
    if (before_track != C5VRX_RING_TRACK_OK) {
        fail_ring(ctx, slot, tracker_failure(before_track, false),
                  c5vrx_ring_tracker_lag(&ctx->tracker));
        return false;
    }
    if (!ctx->reader_valid) {
        /* The first observed writer position is the only defensible boundary
         * between stale SRAM and words produced by this LIVE epoch. The
         * tracker initializes its consumer exactly there. Wait for a fresh
         * half-ring instead of backdating the reader into unknown memory. */
        if (!observe_until_lag(ctx, slot, RING_WORDS / 2u, timeout_ms)) {
            return false;
        }
        const uint32_t reader =
            (uint32_t)(ctx->tracker.consumer_absolute & RING_MASK);
        const uint32_t alignment_skip = (0u - reader) & 7u;
        if (alignment_skip &&
            c5vrx_ring_tracker_consume(&ctx->tracker, alignment_skip) !=
                C5VRX_RING_TRACK_OK) {
            fail_ring(ctx, slot, C5VRX_RING_FAILURE_READER_INSIDE_GUARD, 0u);
            return false;
        }
        ctx->reader_valid = true;
        ctx->discontinuity = true;
        ++ctx->stats.discontinuities;
        ++ctx->stats.stream_epoch;
    }
    sync_tracker_stats(ctx);

    size_t count = ctx->maximum_words;
    const uint16_t reader =
        (uint16_t)(ctx->tracker.consumer_absolute & RING_MASK);
    const size_t contiguous = RING_WORDS - reader;
    if (count > contiguous) count = contiguous;
    count &= ~(size_t)3u;
    if (count < 8u) {
        fail_ring(ctx, slot, C5VRX_RING_FAILURE_READER_INSIDE_GUARD, 0u);
        return false;
    }

    /* A fast consumer reaching the writer is normal backpressure, not an
     * overrun. Keep sampling the writer while waiting; sleeping even 1 ms can
     * hide multiple wraps at the expected input rate. */
    if (c5vrx_ring_tracker_lag(&ctx->tracker) < count) {
        if (!observe_until_lag(ctx, slot, count, timeout_ms)) return false;
    }

    const uint32_t first_cycle = (uint32_t)esp_cpu_get_cycle_count();
    for (size_t i = 0; i < count; ++i) slot->words[i] = s_ring[reader + i];
    __asm__ __volatile__("fence r, rw" ::: "memory");
    const uint32_t last_cycle = (uint32_t)esp_cpu_get_cycle_count();
    const uint32_t copy_cycles = last_cycle - first_cycle;
    ctx->stats.copy_cycles_total += copy_cycles;
    if (copy_cycles > ctx->stats.maximum_copy_cycles)
        ctx->stats.maximum_copy_cycles = copy_cycles;

    c5vrx_rf_dump_status_t after = {0};
    if (c5vrx_rf_dump_get_status(&after) != ESP_OK || !after.enabled) {
        ++ctx->stats.dropped_blocks;
        fail_ring(ctx, slot, C5VRX_RING_FAILURE_PRODUCER_STOPPED, 0u);
        return false;
    }
    if (!c5vrx_rf_dump_canaries_intact()) {
        ++ctx->stats.dropped_blocks;
        fail_ring(ctx, slot, C5VRX_RING_FAILURE_ADJACENT_MEMORY_CORRUPTED,
                  count);
        return false;
    }
    const c5vrx_ring_track_result_t after_track =
        c5vrx_ring_tracker_observe(&ctx->tracker, after.pointer, last_cycle);
    if (after_track != C5VRX_RING_TRACK_OK) {
        ++ctx->stats.dropped_blocks;
        fail_ring(ctx, slot, tracker_failure(after_track, true), count);
        return false;
    }
    if (c5vrx_ring_tracker_consume(&ctx->tracker, count) !=
        C5VRX_RING_TRACK_OK) {
        fail_ring(ctx, slot, C5VRX_RING_FAILURE_READER_INSIDE_GUARD, count);
        return false;
    }
    ctx->stats.reader_pointer =
        (uint16_t)(ctx->tracker.consumer_absolute & RING_MASK);
    ctx->stats.writer_pointer = after.pointer;
    ++ctx->stats.blocks;
    ctx->stats.words += count;
    *block = (c5vrx_rf_block_t) {
        .words = slot->words,
        .word_count = count,
        .sequence = ctx->sequence++,
        .capture_time_us = (uint64_t)copy_cycles * 1000000u / cpu_hz(),
        .discontinuity_before = ctx->discontinuity,
        .owner = slot,
    };
    sync_tracker_stats(ctx);
    ctx->discontinuity = false;
    return true;
}

static void ring_release(c5vrx_rf_source_t *source,
                         const c5vrx_rf_block_t *block)
{
    (void)source;
    ring_slot_t *slot = block ? block->owner : NULL;
    if (slot) atomic_store(&slot->in_use, false);
}

esp_err_t c5vrx_live_ring_source_create(c5vrx_rf_source_t *source,
                                        c5vrx_rf_dump_mode_t mode,
                                        size_t maximum_words_per_block,
                                        size_t guard_words,
                                        uint32_t maximum_plausible_rate_hz)
{
    if (!source || maximum_words_per_block < 8u ||
        maximum_words_per_block > RING_WORDS / 2u ||
        (maximum_words_per_block & 3u) || guard_words < 16u ||
        maximum_words_per_block + guard_words >= RING_WORDS / 2u ||
        !maximum_plausible_rate_hz ||
        maximum_plausible_rate_hz > RING_ABSOLUTE_MAX_HZ) {
        return ESP_ERR_INVALID_ARG;
    }
    ring_context_t *ctx = calloc(1, sizeof(*ctx));
    if (!ctx) return ESP_ERR_NO_MEM;
    ctx->maximum_words = maximum_words_per_block;
    ctx->guard_words = guard_words;
    ctx->maximum_plausible_rate_hz = maximum_plausible_rate_hz;
    ctx->stats.guard_words = (uint16_t)guard_words;
    if (c5vrx_ring_tracker_init(&ctx->tracker, RING_WORDS,
                                (uint32_t)guard_words, cpu_hz(),
                                maximum_plausible_rate_hz) != 0) {
        free(ctx);
        return ESP_ERR_INVALID_ARG;
    }
    for (unsigned i = 0; i < RING_BUFFER_COUNT; ++i) {
        ctx->slots[i].words = heap_caps_malloc(
            maximum_words_per_block * sizeof(uint32_t),
            MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);
        if (!ctx->slots[i].words) {
            while (i) free(ctx->slots[--i].words);
            free(ctx);
            return ESP_ERR_NO_MEM;
        }
        atomic_init(&ctx->slots[i].in_use, false);
    }
    esp_err_t err = c5vrx_rf_dump_configure(RING_WORDS, mode);
    if (err != ESP_OK) {
        for (unsigned i = 0; i < RING_BUFFER_COUNT; ++i)
            free(ctx->slots[i].words);
        free(ctx);
        return err;
    }
    *source = (c5vrx_rf_source_t) {
        .name = "EXPERIMENTAL_RING_SOURCE_UNPROVEN",
        .kind = C5VRX_RF_SOURCE_EXPERIMENTAL_RING_UNPROVEN,
        .nominal_sample_rate_hz = 0,
        .acquire = ring_acquire,
        .release = ring_release,
        .context = ctx,
    };
    return ESP_OK;
}

void c5vrx_live_ring_source_get_stats(const c5vrx_rf_source_t *source,
                                      c5vrx_live_ring_stats_t *stats)
{
    if (!stats) return;
    memset(stats, 0, sizeof(*stats));
    const ring_context_t *ctx = source ? source->context : NULL;
    if (ctx) *stats = ctx->stats;
}

const char *c5vrx_live_ring_failure_name(c5vrx_live_ring_failure_t failure)
{
    switch (failure) {
        case C5VRX_RING_FAILURE_PRODUCER_STOPPED: return "PRODUCER_STOPPED";
        case C5VRX_RING_FAILURE_SERVICE_INTERVAL_AMBIGUOUS:
            return "SERVICE_INTERVAL_AMBIGUOUS";
        case C5VRX_RING_FAILURE_READER_INSIDE_GUARD:
            return "READER_INSIDE_GUARD";
        case C5VRX_RING_FAILURE_COPY_AMBIGUOUS: return "COPY_AMBIGUOUS";
        case C5VRX_RING_FAILURE_POINTER_OUT_OF_RANGE:
            return "POINTER_OUT_OF_RANGE";
        case C5VRX_RING_FAILURE_ADJACENT_MEMORY_CORRUPTED:
            return "ADJACENT_MEMORY_CORRUPTED";
        default: return "NONE";
    }
}

esp_err_t c5vrx_live_ring_source_destroy(c5vrx_rf_source_t *source)
{
    if (!source) return ESP_ERR_INVALID_ARG;
    esp_err_t err = ESP_OK;
    ring_context_t *ctx = source->context;
    if (ctx) {
        err = ctx->stop_attempted ? ctx->stop_result : c5vrx_rf_dump_stop();
        for (unsigned i = 0; i < RING_BUFFER_COUNT; ++i)
            free(ctx->slots[i].words);
        free(ctx);
    }
    memset(source, 0, sizeof(*source));
    return err;
}
