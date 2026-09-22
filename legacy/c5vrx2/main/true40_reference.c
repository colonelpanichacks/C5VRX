/* Exact table reference for the embedded True 40 MS/s adjacent discriminator program.
 * Emits two adjacent demodulated DAC codes per input pair (40 MS/s rate).
 * This is a diagnostic oracle, never CPU DSP in the live path. */
#include "true40_reference.h"
#include "true40_lut.h"

static inline unsigned raw_to_iq5(uint8_t raw)
{
    /* Q[3:1] (bits 1,2,3) -> bits 0..2 of iq5
     * I[3:2] (bits 6,7)   -> bits 3..4 of iq5 */
    return ((raw >> 1) & 7u) | (((raw >> 6) & 3u) << 3);
}

size_t c5vrx2_true40_reference(const uint8_t *input, size_t input_bytes,
                               uint8_t *output, size_t output_bytes,
                               uint8_t *previous_iq5)
{
    if (!input || !output || !previous_iq5) return 0;
    size_t pairs = input_bytes / 2u;
    if (pairs * 2u > output_bytes) pairs = output_bytes / 2u;
    unsigned previous = *previous_iq5 & 31u;
    for (size_t n = 0; n < pairs; ++n) {
        unsigned iq0 = raw_to_iq5(input[2u * n]);
        unsigned iq1 = raw_to_iq5(input[2u * n + 1u]);
        output[2u * n] = (uint8_t)(c5vrx2_true40_lut[(previous << 5) | iq0] & 63u);
        output[2u * n + 1u] = (uint8_t)(c5vrx2_true40_lut[(iq0 << 5) | iq1] & 63u);
        previous = iq1;
    }
    *previous_iq5 = (uint8_t)previous;
    return pairs * 2u;
}
