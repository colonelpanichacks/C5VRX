#pragma once
#include <stddef.h>
#include <stdint.h>
#include "esp_err.h"

/**
 * video_start() - Initialize and start the PARLIO RX+TX realtime pipeline.
 *
 * Sets up:
 *   - PARLIO RX @ 40 MHz, POS edge, 8-bit, 16 KiB cyclic DMA ring
 *   - PARLIO TX @ 40 MHz, [D,D] output, loop_transmission
 *   - TX BitScrambler with embedded Phase5 LUT (fm.bsasm)
 *   - One-time RX start, then TX starts after one-block delay
 *
 * After video_start() returns ESP_OK, the CPU is done.
 * The hardware pipeline runs forever without any software involvement.
 *
 * NO periodic tasks, NO telemetry, NO calibration loading.
 */
esp_err_t video_start(void);

/* Passive read-only tap of the raw Q4/I4 RX ring for diagnostics consumers
 * (preview.c snow fallback). Returns the ring base, its size, and the current
 * RX DMA descriptor offset (bytes into the ring) in one call so the consumer
 * never touches AHB_DMA registers itself. Callers must cache-sync before
 * reading and must never write the ring. */
const uint8_t *video_tap_iq_ring(size_t *size_bytes, uint32_t *rx_offset);

/* Raw ring size (bytes = 40 MS/s samples). */
#define VIDEO_RX_RING_BYTES 16384u

/** The raw RX ring (VIDEO_RX_RING_BYTES, one 40 MS/s sample per byte).
 * Read-only; written only by the RX GDMA. Owned by main/grab.c and the
 * diagnostics tap above. */
const uint8_t *video_rx_ring(void);

/** Ring offset of the descriptor the RX GDMA is writing now: every byte
 * before it (cyclically) is complete. False if the pointer is unknown.
 * Descriptor-granular (~102 us); consumers needing sample precision must
 * combine it with a timebase (see main/grab.c). */
bool video_rx_write_offset(uint32_t *offset);

/** Largest RX descriptor, bytes (how far the GDMA may be ahead of the
 * write offset). */
uint32_t video_rx_max_descriptor(void);
