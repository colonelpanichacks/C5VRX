/* SPDX-License-Identifier: GPL-3.0-only */

#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "esp_err.h"
#include "c5vrx_cvbs_sync.h"

#define C5VRX_USB_PREVIEW_WIDTH 160u
#define C5VRX_USB_PREVIEW_HEIGHT 120u
#define C5VRX_USB_PREVIEW_PROTOCOL_VERSION 1u

typedef struct {
    uint32_t samples_ingested;
    uint32_t horizontal_syncs;
    uint32_t vertical_syncs;
    uint32_t rejected_sync_pulses;
    uint32_t lock_acquisitions;
    uint32_t lock_losses;
    uint32_t frames_completed;
    uint32_t frames_sent;
    uint32_t frames_dropped;
    uint32_t line_period_samples;
    uint32_t last_hsync_width;
    uint32_t last_vsync_width;
    uint8_t sync_threshold;
    bool horizontal_locked;
    bool vertical_locked;
    bool consumer_active;
    bool worker_active;
    uint32_t session_epoch;
    uint32_t transport_stalls;
    uint32_t stale_session_drops;
    uint32_t worker_time_us;
    uint32_t burst_locks;
    bool burst_locked;
} c5vrx_usb_preview_stats_t;

esp_err_t c5vrx_usb_preview_start(void);
esp_err_t c5vrx_usb_preview_prepare(void);
esp_err_t c5vrx_usb_preview_stop(void);
bool c5vrx_usb_preview_running(void);
/* START allocates the pre-sized side-tap buffers and opens a short lease.
 * KEEPALIVE proves that a host is still actively draining the binary stream.
 * Merely enumerating or plugging in USB never activates expensive work. */
void c5vrx_usb_preview_keepalive(void);
bool c5vrx_usb_preview_consumer_active(void);
void c5vrx_usb_preview_ingest(const uint8_t *cvbs, size_t samples);
void c5vrx_usb_preview_ingest_timed(
    const uint8_t *cvbs, size_t samples,
    const c5vrx_cvbs_sync_tracker_t *canonical_timing);
void c5vrx_usb_preview_get_stats(c5vrx_usb_preview_stats_t *stats);
