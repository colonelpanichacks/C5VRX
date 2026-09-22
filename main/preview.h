#pragma once

#include <stdbool.h>
#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

/* USB video preview side-tap (always on; enabled once at boot).
 *
 * A low-priority task passively reads the raw Q4/I4 DMA ring through the
 * proven composite grabber (main/grab.c), streams the 224x168 GRAY8 rows, and
 * streams them over the USB Serial/JTAG console using the v1 packet wire
 * format (see legacy/c5vrx1/research/usb-preview-protocol.md). It never
 * writes the ring and never paces or gates the IQ pipeline. There is
 * deliberately no way to disable it: preview_set_enabled() honors only the
 * off -> on transition at boot. */

#define PREVIEW_WIDTH   224u
#define PREVIEW_HEIGHT  168u

/* Creates the preview task and enables the stream (always on, no disable
 * path). Call once from video_start(); preview_set_enabled(true) may be
 * re-asserted afterwards, it is idempotent. */
esp_err_t preview_init(void);

void preview_set_enabled(bool enabled);
bool preview_is_enabled(void);
void preview_get_stats(uint32_t *completed, uint32_t *sent,
                       uint32_t *dropped, uint32_t *lines,
                       bool *h_locked, uint32_t *vsyncs);
/* Grabber-tracked active-video line period in us. Returns false (blank
 * semantics) until the grabber has a lock with a measured period, so CSV
 * consumers can leave the cell empty instead of printing 0.00. */
bool preview_get_line_us(float *line_us);
