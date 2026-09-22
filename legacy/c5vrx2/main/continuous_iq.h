#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"

#include "rf_dump.h"

typedef struct {
    const volatile uint32_t *data;
    size_t words;
    uint64_t absolute_start;
    uint32_t discontinuity_epoch;
    bool continuous_from_previous;
} iq_span_t;

typedef struct {
    uint32_t rf_sample_rate_hz;
    uint32_t writer_pointer;
    uint32_t dump_control;
    uint64_t producer_words;
    uint64_t consumer_words;
    uint32_t physical_wraps;
    uint32_t overruns;
    uint32_t ambiguous_wraps;
    uint32_t discontinuity_epoch;
    uint32_t producer_start_count;
    uint32_t rearm_count;
    uint32_t trigger_count;
} continuous_iq_stats_t;

/* Start the C5 dump engine with the vendor-derived C5 TX_START selector and
 * the bit-17 dump-first state subsequently proven by a one-start hardware
 * soak. There is deliberately no software trigger, DONE wait, timeout, or
 * periodic rearm. */
esp_err_t continuous_iq_start(void);

/* Single-consumer span interface.  The caller must poll fast enough that the
 * 16K ring cannot wrap ambiguously. Hardware has since shown that the live
 * MAC-owned bank is not readable through ordinary CPU/AHB-GDMA; this API is
 * therefore diagnostic-only until a live-readable mapping is proven. */
bool continuous_iq_acquire(iq_span_t *span);
void continuous_iq_release(const iq_span_t *span);

esp_err_t continuous_iq_stop(void);
bool continuous_iq_is_running(void);
void continuous_iq_get_stats(continuous_iq_stats_t *stats);

/* RTC-retained bring-up marker. This performs no flash or USB I/O and exists
 * solely to locate a CPU reset inside the ownership/enable sequence. */
uint32_t continuous_iq_debug_last_stage(void);
void continuous_iq_debug_mark(uint32_t stage);

uint32_t continuous_iq_sample_rate_hz(void);
const void *continuous_iq_ring_base(void);
size_t continuous_iq_ring_bytes(void);

/* Wait until base-address DMA will start approximately `lead_words` behind
 * the RF writer.  This does not stop or reset either stream. */
esp_err_t continuous_iq_wait_base_lead(uint32_t lead_words,
                                       uint32_t tolerance_words,
                                       uint32_t timeout_us);
