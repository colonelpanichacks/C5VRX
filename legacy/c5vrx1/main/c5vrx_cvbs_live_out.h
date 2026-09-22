/* SPDX-License-Identifier: GPL-3.0-only */

#pragma once
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "esp_err.h"
#include "c5vrx_cvbs_sync.h"

typedef struct {
    uint64_t live_blocks;
    uint64_t live_blocks_retired;
    uint64_t live_retirements_completed;
    uint64_t filler_blocks;
    uint64_t mailbox_drops;
    uint64_t qualification_underruns;
    uint64_t guardian_failures;
    uint64_t phase_mismatch_drops;
    bool guardian_running;
} c5vrx_cvbs_live_out_stats_t;

#define C5VRX_CVBS_SOURCE_SAMPLE_RATE_HZ 20000000u

esp_err_t c5vrx_cvbs_live_out_start(size_t block_samples);
esp_err_t c5vrx_cvbs_live_out_start_at_rate(size_t block_samples,
                                            uint32_t output_clock_hz);
esp_err_t c5vrx_cvbs_live_out_start_aligned(
    const uint8_t *first_live_samples, size_t block_samples,
    const c5vrx_cvbs_sync_tracker_t *first_live_timing);
bool c5vrx_cvbs_live_out_running(void);
bool c5vrx_cvbs_live_out_take_realign_request(void);
esp_err_t c5vrx_cvbs_live_out_write(const uint8_t *samples, size_t count,
                                    void *context);
/* Qualification-only blocking submit. The realtime dataplane must use the
 * nonblocking write above so AV can never backpressure RF processing. */
esp_err_t c5vrx_cvbs_live_out_write_wait(const uint8_t *samples, size_t count,
                                         uint32_t timeout_ms);
void c5vrx_cvbs_live_out_qualification_begin(uint32_t blocks);
void c5vrx_cvbs_live_out_qualification_end(void);
esp_err_t c5vrx_cvbs_live_out_stop(void);
void c5vrx_cvbs_live_out_get_stats(c5vrx_cvbs_live_out_stats_t *stats);
void c5vrx_cvbs_live_out_update_timing(
    const c5vrx_cvbs_sync_tracker_t *timing);
