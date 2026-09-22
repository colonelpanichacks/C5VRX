#pragma once
#include <stddef.h>
#include <stdint.h>
/* State starts at phase zero, as in the BitScrambler prime. Preserve across
 * chunks/ring wraps. Calls consume complete pairs only; caller carries any
 * unpaired byte to the next call. Fixed embedded gain=2 / pedestal=20. */
size_t c5vrx2_trajectory_reference(const uint8_t *input, size_t input_bytes,
                                  uint8_t *output, size_t output_bytes,
                                  uint8_t *previous_phase);
