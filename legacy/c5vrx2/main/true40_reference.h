#pragma once
#include <stddef.h>
#include <stdint.h>

/* Exact table reference for the embedded True 40 MS/s adjacent discriminator program.
 * Emits two adjacent demodulated DAC codes per input pair (40 MS/s rate).
 * State starts at reset zero, preserved across chunks/ring wraps.
 * Fixed embedded gain=2 / pedestal=20. */
size_t c5vrx2_true40_reference(const uint8_t *input, size_t input_bytes,
                               uint8_t *output, size_t output_bytes,
                               uint8_t *previous_iq5);
