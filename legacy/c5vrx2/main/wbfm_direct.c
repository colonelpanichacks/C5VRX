#include "wbfm_direct.h"

#include <math.h>
#include <stdint.h>
#include <stdlib.h>

#include "driver/bitscrambler.h"
#include "esp_heap_caps.h"
#include "soc/soc_caps.h"

#include "calibration.h"

BITSCRAMBLER_PROGRAM(c5vrx2_wbfm_direct6_4to1_program,
                    "c5vrx2_wbfm_direct6_4to1");

#define LUT_BYTES 2048u
#define PI_F 3.14159265358979323846f

/* An 8-bit LUT has an 11-bit address on C5.  Use five bits for each IQ
 * point and the last bit to alternate Q3/I2 and Q2/I3 quantizers.  That
 * doubles the Cartesian phase states from the old 2+2 implementation while
 * keeping both the current and previous sample in every adjacent lookup.
 * Alternating the finer axis avoids permanently favouring I or Q. */
static float signed_bucket_center(unsigned code, unsigned bits)
{
    const unsigned width = 1u << (10u - bits);
    float center = (float)(code * width) + ((float)width - 1.0f) * 0.5f;
    if (center >= 512.0f) center -= 1024.0f;
    return center;
}

static float compact_phase(unsigned compact, bool q_fine)
{
    const unsigned q_bits = q_fine ? 3u : 2u;
    const unsigned i_bits = q_fine ? 2u : 3u;
    const unsigned q_mask = (1u << q_bits) - 1u;
    const float q = signed_bucket_center(compact & q_mask, q_bits);
    const float i = signed_bucket_center(compact >> q_bits, i_bits);
    return atan2f(q, i);
}

static int adjacent_delta(unsigned previous, unsigned current,
                          bool current_q_fine)
{
    /* Consecutive samples use opposite mappings. */
    float delta = compact_phase(current, current_q_fine) -
                  compact_phase(previous, !current_q_fine);
    while (delta >= PI_F) delta -= 2.0f * PI_F;
    while (delta < -PI_F) delta += 2.0f * PI_F;
    int phase8 = (int)lrintf(delta * (256.0f / (2.0f * PI_F)));
    if (phase8 < -128) phase8 = -128;
    if (phase8 > 127) phase8 = 127;
    return phase8;
}

static void build_adjacent_lut(uint8_t lut[LUT_BYTES])
{
    const c5vrx2_calibration_t *cal = c5vrx2_calibration_get();
    const int pedestal_correction = (int)cal->pedestal_code - 20;

    for (unsigned current_q_fine = 0; current_q_fine < 2u;
         ++current_q_fine) {
        for (unsigned current = 0; current < 32u; ++current) {
            for (unsigned previous = 0; previous < 32u; ++previous) {
                int delta = adjacent_delta(previous, current,
                                           current_q_fine != 0u);
                if (cal->polarity == C5VRX2_POLARITY_PREVIOUS_MINUS_CURRENT)
                    delta = -delta;

                /* FM first: every adjacent phase step contributes before the
                 * real four-sample boxcar/decimator. */
                int contribution = delta * (int)cal->discriminator_gain;
                contribution += pedestal_correction;
                /* Counter A starts at 4*20. Keep the four-term accumulator
                 * within the legal unsigned six-bit output excursion. */
                if (contribution < -20) contribution = -20;
                if (contribution > 43) contribution = 43;

                const unsigned index =
                    (current_q_fine << 10u) | (previous << 5u) | current;
                lut[index] = (uint8_t)(int8_t)contribution;
            }
        }
    }
}

esp_err_t c5vrx2_wbfm_direct_configure(bitscrambler_handle_t handle)
{
    if (!handle) return ESP_ERR_INVALID_ARG;

    uint8_t *lut = heap_caps_malloc(LUT_BYTES, MALLOC_CAP_INTERNAL);
    if (!lut) return ESP_ERR_NO_MEM;
    build_adjacent_lut(lut);

    /* Loading the program here establishes its 8-bit LUT width. PARLIO's
     * official decoration loads the same program again immediately before
     * starting GDMA; because the program has no embedded LUT entries, that
     * does not overwrite these runtime-calibrated 2048 bytes. */
    esp_err_t err = bitscrambler_load_program(
        handle, c5vrx2_wbfm_direct6_4to1_program);
    if (err == ESP_OK) err = bitscrambler_load_lut(handle, lut, LUT_BYTES);
    free(lut);
    return err;
}

const void *c5vrx2_wbfm_direct_program(void)
{
    return c5vrx2_wbfm_direct6_4to1_program;
}
