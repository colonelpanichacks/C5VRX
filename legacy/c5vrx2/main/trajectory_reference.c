/* Exact table reference for the embedded trajectory program (fixed calibration).
 * This is a diagnostic oracle, never CPU DSP in the live path. */
#include "trajectory_reference.h"
#include "trajectory_lut.h"

size_t c5vrx2_trajectory_reference(const uint8_t *input, size_t input_bytes,
                                  uint8_t *output, size_t output_bytes,
                                  uint8_t *previous_phase)
{
    if (!input || !output || !previous_phase) return 0;
    size_t pairs = input_bytes / 2;
    if (pairs > output_bytes) pairs = output_bytes;
    unsigned previous = *previous_phase & 15u;
    for (size_t n = 0; n < pairs; ++n) {
        unsigned middle = input[2*n];
        unsigned current = (c5vrx2_trajectory_lut[input[2*n+1]] >> 8) & 15u;
        unsigned quadrant = ((middle >> 3) & 1u) | ((middle >> 6) & 2u);
        output[n] = c5vrx2_trajectory_lut[previous | (quadrant << 4) |
                                                    (current << 6)] & 63u;
        previous = current;
    }
    *previous_phase = previous;
    return pairs;
}
