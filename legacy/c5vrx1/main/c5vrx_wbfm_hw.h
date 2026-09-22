/* SPDX-License-Identifier: GPL-3.0-only */

#pragma once

#include <stddef.h>
#include <stdint.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct c5vrx_wbfm_hw_context c5vrx_wbfm_hw_context_t;

typedef enum {
    C5VRX_WBFM_PHASE6_4TO1 = 0,
    C5VRX_WBFM_PHASE8_4TO1 = 1,
} c5vrx_wbfm_kernel_t;

/** Exact 6-bit retained-sample phase quantizer used by the hardware LUT. */
uint8_t c5vrx_wbfm_coarse_phase6(uint32_t packed_iq);
uint8_t c5vrx_wbfm_hw_context_phase6(
    const c5vrx_wbfm_hw_context_t *context, uint32_t packed_iq);

/** Create one reusable BitScrambler/GDMA loopback transform. */
esp_err_t c5vrx_wbfm_hw_create(size_t maximum_input_words,
                               c5vrx_wbfm_hw_context_t **context);
esp_err_t c5vrx_wbfm_hw_create_kernel(
    size_t maximum_input_words, c5vrx_wbfm_kernel_t kernel,
    c5vrx_wbfm_hw_context_t **context);
unsigned c5vrx_wbfm_hw_phase_bits(const c5vrx_wbfm_hw_context_t *context);

void c5vrx_wbfm_hw_destroy(c5vrx_wbfm_hw_context_t *context);

/** Run a transform using an already configured hardware/DMA context. */
esp_err_t c5vrx_wbfm_hw_transform_context(
    c5vrx_wbfm_hw_context_t *context,
    const uint32_t *packed_iq,
    size_t input_words,
    uint8_t *phase_delta,
    size_t output_capacity,
    size_t *output_written);

/**
 * Transform packed C5 RF-test I/Q words to biased 6-bit phase deltas.
 *
 * Input words use the recovered format (Q10 in bits 0..9, I10 in bits 10..19).
 * Four input words are consumed for each output byte. Output low six bits use
 * bias code 32: approximately zero instantaneous frequency is code 32.
 *
 * Compatibility wrapper which creates and destroys a context for one call.
 * Real-time code should use the persistent context API above.
 */
esp_err_t c5vrx_wbfm_hw_transform(const uint32_t *packed_iq,
                                   size_t input_words,
                                   uint8_t *phase_delta,
                                   size_t output_capacity,
                                   size_t *output_written);

/**
 * Capture one finite vendor I/Q block, copy it to DMA-capable SRAM and run the
 * hardware 4:1 WBFM transform. Intended for first physical RF validation.
 */
esp_err_t c5vrx_wbfm_hw_probe_dump(size_t sample_count);

/**
 * Run the C5 BitScrambler 4:1 WBFM discriminator against synthetic packed
 * 10-bit I/Q in RAM and compare the hardware result with a CPU reference.
 */
esp_err_t c5vrx_wbfm_hw_self_test(void);
esp_err_t c5vrx_wbfm_hw_self_test_kernel(c5vrx_wbfm_kernel_t kernel);

#ifdef __cplusplus
}
#endif
