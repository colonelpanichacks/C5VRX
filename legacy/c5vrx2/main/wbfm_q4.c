#include "wbfm_q4.h"

#include <math.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "esp_heap_caps.h"
#include "hal/bitscrambler_ll.h"
#include "soc/bitscrambler_struct.h"
#if CONFIG_C5VRX2_WBFM_SELFTEST_ONCE
#include "driver/bitscrambler_loopback.h"
#include "esp_cpu.h"
#include "esp_partition.h"
#include "soc/soc_caps.h"
#endif

#include "calibration.h"

BITSCRAMBLER_PROGRAM(c5vrx2_wbfm_q4_2to1_program,
                    "c5vrx2_wbfm_q4_2to1");
BITSCRAMBLER_PROGRAM(c5vrx2_wbfm_q4_fast_2to1_program,
                    "c5vrx2_wbfm_q4_fast_2to1");
BITSCRAMBLER_PROGRAM(c5vrx2_wbfm_q4_lut3_2to1_program,
                    "c5vrx2_wbfm_q4_lut3_2to1");
BITSCRAMBLER_PROGRAM(c5vrx2_wbfm_q4_iq5_2to1_program,
                    "c5vrx2_wbfm_q4_iq5_2to1");
BITSCRAMBLER_PROGRAM(c5vrx2_wbfm_q4_phase5_2to1_program,
                    "c5vrx2_wbfm_q4_phase5_2to1");
BITSCRAMBLER_PROGRAM(c5vrx2_wbfm_q4_phase5_100ns_2to1_program,
                    "c5vrx2_wbfm_q4_phase5_100ns_2to1");
BITSCRAMBLER_PROGRAM(c5vrx2_q4_delta_program, "c5vrx2_q4_delta");
BITSCRAMBLER_PROGRAM(c5vrx2_linear80_program, "c5vrx2_phase5_linear80");

const void *c5vrx2_wbfm_linear80_program(void)
{
    return c5vrx2_linear80_program;
}
#if CONFIG_C5VRX2_WBFM_SELFTEST_ONCE
BITSCRAMBLER_PROGRAM(c5vrx2_q4_phase_program, "c5vrx2_q4_phase");
BITSCRAMBLER_PROGRAM(c5vrx2_q4_negative_program, "c5vrx2_q4_negative");
BITSCRAMBLER_PROGRAM(c5vrx2_q4_state_program, "c5vrx2_q4_state");
BITSCRAMBLER_PROGRAM(c5vrx2_q4_pairsum_program, "c5vrx2_q4_pairsum");
BITSCRAMBLER_PROGRAM(c5vrx2_counter_lut_program, "c5vrx2_counter_lut");
BITSCRAMBLER_PROGRAM(c5vrx2_input_passthrough_program,
                    "c5vrx2_input_passthrough");
BITSCRAMBLER_PROGRAM(c5vrx2_q4_address_conventional_program,
                    "c5vrx2_q4_address_conventional");
BITSCRAMBLER_PROGRAM(c5vrx2_counter_lut_conventional_program,
                    "c5vrx2_counter_lut_conventional");
#endif

#define LUT_ITEMS 1024u
#define LUT_BYTES (LUT_ITEMS * sizeof(uint16_t))
#define PI_F 3.14159265358979323846f

static float signed_bucket_center(unsigned code, unsigned bits)
{
    const unsigned width = 1u << (10u - bits);
    float center = (float)(code * width) + ((float)width - 1.0f) * 0.5f;
    if (center >= 512.0f) center -= 1024.0f;
    return center;
}

static int q4_phase8(unsigned packed)
{
    const float q = signed_bucket_center(packed & 0x0fu, 4u);
    const float i = signed_bucket_center(packed >> 4u, 4u);
    int phase8 = (int)lrintf(atan2f(q, i) *
                             (256.0f / (2.0f * PI_F)));
    if (phase8 < -128) phase8 = -128;
    if (phase8 > 127) phase8 = 127;
    return phase8;
}

static uint8_t q4_phase4(unsigned packed)
{
    const float q = signed_bucket_center(packed & 0x0fu, 4u);
    const float i = signed_bucket_center(packed >> 4u, 4u);
    int phase4 = (int)lrintf(atan2f(q, i) *
                             (16.0f / (2.0f * PI_F)));
    return (uint8_t)phase4 & 0x0fu;
}

static uint8_t q4_phase5(unsigned packed)
{
    const float q = signed_bucket_center(packed & 0x0fu, 4u);
    const float i = signed_bucket_center(packed >> 4u, 4u);
    const int phase5 = (int)lrintf(atan2f(q, i) *
                                    (32.0f / (2.0f * PI_F)));
    return (uint8_t)phase5 & 0x1fu;
}

static uint8_t q4_phase5_state(unsigned packed)
{
    /* Keep all 32 circular phase states. Reusing state 31 as a confidence
     * flag aliases a valid phase sector and turns legitimate samples into
     * pedestal specks. Confidence repair needs separate temporal state. */
    return q4_phase5(packed);
}

/* Circular means of the Q4/I4 vectors assigned to all 32 phase states,
 * expressed on the signed phase8 circle. */
static const int8_t s_phase5_centroid_phase8[32] = {
       0,    8,   15,   24,   32,   40,   49,   56,
      64,   72,   79,   87,   96,  104,  113,  120,
    -128, -120, -113, -104,  -96,  -88,  -79,  -72,
     -64,  -56,  -49,  -40,  -32,  -23,  -15,   -8,
};

static int phase5_delta_phase8(unsigned previous, unsigned current)
{
    int delta = (int)s_phase5_centroid_phase8[current] -
                (int)s_phase5_centroid_phase8[previous];
    if (delta >= 128) delta -= 256;
    if (delta < -128) delta += 256;
    return delta;
}

uint8_t c5vrx2_wbfm_q4_phase5_value(uint8_t packed)
{
    return q4_phase5(packed);
}

static uint8_t q4_compact_iq5(unsigned packed)
{
    /* Keep Q[9:7] and I[9:8] from the physically proven Q4/I4 byte. */
    return (uint8_t)(((packed >> 1u) & 0x07u) |
                     (((packed >> 6u) & 0x03u) << 3u));
}

static float compact_iq5_phase(unsigned compact)
{
    const float q = signed_bucket_center(compact & 0x07u, 3u);
    const float i = signed_bucket_center(compact >> 3u, 2u);
    return atan2f(q, i);
}

static int scale_real_sum(int sum, unsigned calibration_gain)
{
    /* sum is signed (delta1 + delta2). Gain belongs here, after circular
     * differencing and signed unwrap. Settings 1..4 represent 1.0x, 1.5x,
     * 2.0x and 2.5x respectively; division by two is the real 40->20 MS/s
     * boxcar. Round symmetrically before the final DAC clamp. */
    const int numerator = sum * ((int)calibration_gain + 1);
    return numerator < 0 ? -((-numerator + 2) / 4) :
                           (numerator + 2) / 4;
}

static void build_lut(uint16_t lut[LUT_ITEMS])
{
    const c5vrx2_calibration_t *cal = c5vrx2_calibration_get();
    for (unsigned index = 0; index < LUT_ITEMS; ++index) {
        if (index < 0x100u) {
            int phase = q4_phase8(index);
            if (cal->polarity == C5VRX2_POLARITY_PREVIOUS_MINUS_CURRENT)
                phase = -phase;
            const uint8_t phase_mod = (uint8_t)phase;
            const uint8_t negative_phase = (uint8_t)(0u - phase_mod);
            lut[index] = (uint16_t)phase_mod |
                         ((uint16_t)negative_phase << 8u);
        } else if (index < 0x200u) {
            int sum = (int)(index & 0xffu);
            if (sum >= 128) sum -= 256;
            int code = (int)cal->pedestal_code +
                       scale_real_sum(sum, cal->discriminator_gain);
            if (code < 0) code = 0;
            if (code > 63) code = 63;
            lut[index] = (uint16_t)(uint8_t)code;
        } else {
            lut[index] = 0u;
        }
    }
}

static void build_lut3(uint16_t lut[LUT_ITEMS])
{
    const c5vrx2_calibration_t *cal = c5vrx2_calibration_get();
    memset(lut, 0, LUT_BYTES);
    for (unsigned packed = 0u; packed < 256u; ++packed) {
        uint8_t phase = q4_phase4(packed);
        if (cal->polarity == C5VRX2_POLARITY_PREVIOUS_MINUS_CURRENT)
            phase = (uint8_t)(0u - phase) & 0x0fu;
        lut[packed] = phase;
    }
    for (unsigned previous = 0u; previous < 16u; ++previous) {
        for (unsigned current = 0u; current < 16u; ++current) {
            int delta = (int)((current - previous) & 0x0fu);
            if (delta >= 8) delta -= 16;
            /* One phase4 step equals 16 phase8 steps.  scale_real_sum()
             * applies the real-domain 2:1 average and calibrated gain. */
            int code = (int)cal->pedestal_code +
                       scale_real_sum(delta * 16, cal->discriminator_gain);
            if (code < 0) code = 0;
            if (code > 63) code = 63;
            lut[0x100u | (previous << 4u) | current] = (uint16_t)code;
        }
    }
}

static int iq5_delta_phase8(unsigned previous, unsigned current)
{
    float delta = compact_iq5_phase(current) - compact_iq5_phase(previous);
    while (delta >= PI_F) delta -= 2.0f * PI_F;
    while (delta < -PI_F) delta += 2.0f * PI_F;
    int phase8 = (int)lrintf(delta * (256.0f / (2.0f * PI_F)));
    if (phase8 < -128) phase8 = -128;
    if (phase8 > 127) phase8 = 127;
    return phase8;
}

static void build_iq5_lut(uint16_t lut[LUT_ITEMS])
{
    const c5vrx2_calibration_t *cal = c5vrx2_calibration_get();
    for (unsigned previous = 0u; previous < 32u; ++previous) {
        for (unsigned current = 0u; current < 32u; ++current) {
            int delta = iq5_delta_phase8(previous, current);
            if (cal->polarity == C5VRX2_POLARITY_PREVIOUS_MINUS_CURRENT)
                delta = -delta;
            int code = (int)cal->pedestal_code +
                       scale_real_sum(delta, cal->discriminator_gain);
            if (code < 0) code = 0;
            if (code > 63) code = 63;
            const unsigned index = (previous << 5u) | current;
            lut[index] = (uint16_t)code;
        }
    }
}

esp_err_t c5vrx2_wbfm_q4_configure_phase5(bitscrambler_handle_t handle)
{
    if (!handle) return ESP_ERR_INVALID_ARG;
    /* The TX transaction reloads this program. Keep its LUT embedded so the
     * instructions and table become active atomically in that same load. A
     * bounded C5 oracle proved that a separately preloaded LUT is not retained
     * by the active PARLIO TX run, whereas embedded LUT data is byte-exact. */
    return bitscrambler_load_program(
        handle, c5vrx2_wbfm_q4_phase5_2to1_program);
}

esp_err_t c5vrx2_wbfm_q4_configure_iq5(bitscrambler_handle_t handle)
{
    if (!handle) return ESP_ERR_INVALID_ARG;
    uint16_t *lut = heap_caps_malloc(LUT_BYTES, MALLOC_CAP_INTERNAL);
    if (!lut) return ESP_ERR_NO_MEM;
    build_iq5_lut(lut);
    esp_err_t err = bitscrambler_load_program(
        handle, c5vrx2_wbfm_q4_iq5_2to1_program);
    if (err == ESP_OK) err = bitscrambler_load_lut(handle, lut, LUT_BYTES);
    free(lut);
    return err;
}

esp_err_t c5vrx2_wbfm_q4_configure_lut3(bitscrambler_handle_t handle)
{
    if (!handle) return ESP_ERR_INVALID_ARG;
    uint16_t *lut = heap_caps_malloc(LUT_BYTES, MALLOC_CAP_INTERNAL);
    if (!lut) return ESP_ERR_NO_MEM;
    build_lut3(lut);
    esp_err_t err = bitscrambler_load_program(
        handle, c5vrx2_wbfm_q4_lut3_2to1_program);
    if (err == ESP_OK) err = bitscrambler_load_lut(handle, lut, LUT_BYTES);
    free(lut);
    return err;
}

esp_err_t c5vrx2_wbfm_q4_configure(bitscrambler_handle_t handle)
{
    if (!handle) return ESP_ERR_INVALID_ARG;
    uint16_t *lut = heap_caps_malloc(LUT_BYTES, MALLOC_CAP_INTERNAL);
    if (!lut) return ESP_ERR_NO_MEM;
    build_lut(lut);
    esp_err_t err = bitscrambler_load_program(
        handle, c5vrx2_wbfm_q4_2to1_program);
    if (err == ESP_OK) err = bitscrambler_load_lut(handle, lut, LUT_BYTES);
    free(lut);
    return err;
}

esp_err_t c5vrx2_wbfm_q4_configure_delta(bitscrambler_handle_t handle)
{
    if (!handle) return ESP_ERR_INVALID_ARG;
    uint16_t *lut = heap_caps_malloc(LUT_BYTES, MALLOC_CAP_INTERNAL);
    if (!lut) return ESP_ERR_NO_MEM;
    build_lut(lut);
    esp_err_t err = bitscrambler_load_program(handle,
                                               c5vrx2_q4_delta_program);
    if (err == ESP_OK) err = bitscrambler_load_lut(handle, lut, LUT_BYTES);
    free(lut);
    return err;
}

esp_err_t c5vrx2_wbfm_q4_load_tx_lut(void)
{
    uint16_t *lut = heap_caps_malloc(LUT_BYTES, MALLOC_CAP_INTERNAL);
    if (!lut) return ESP_ERR_NO_MEM;
    build_lut(lut);

    /* PARLIO owns the opaque TX BitScrambler handle. The decorator loads the
     * program at transaction start but deliberately leaves LUT RAM untouched
     * when the program has no embedded LUT items. Load the shared C5 TX LUT
     * port directly, using the same packing as bitscrambler_load_lut(). */
    bitscrambler_ll_set_lut_width(&BITSCRAMBLER, BITSCRAMBLER_DIR_TX,
                                  BITSCRAMBLER_LUT_WIDTH_32BIT);
    const uint32_t *words = (const uint32_t *)lut;
    for (unsigned word = 0u; word < LUT_BYTES / sizeof(uint32_t); ++word) {
        bitscrambler_ll_lutmem_write(&BITSCRAMBLER, BITSCRAMBLER_DIR_TX,
                                     (int)word, words[word]);
    }
    bitscrambler_ll_set_lut_width(&BITSCRAMBLER, BITSCRAMBLER_DIR_TX,
                                  BITSCRAMBLER_LUT_WIDTH_16BIT);
    free(lut);
    return ESP_OK;
}

esp_err_t c5vrx2_wbfm_q4_load_tx_lut3(void)
{
    uint16_t *lut = heap_caps_malloc(LUT_BYTES, MALLOC_CAP_INTERNAL);
    if (!lut) return ESP_ERR_NO_MEM;
    build_lut3(lut);
    bitscrambler_ll_set_lut_width(&BITSCRAMBLER, BITSCRAMBLER_DIR_TX,
                                  BITSCRAMBLER_LUT_WIDTH_32BIT);
    const uint32_t *words = (const uint32_t *)lut;
    for (unsigned word = 0u; word < LUT_BYTES / sizeof(uint32_t); ++word)
        bitscrambler_ll_lutmem_write(&BITSCRAMBLER, BITSCRAMBLER_DIR_TX,
                                     (int)word, words[word]);
    bitscrambler_ll_set_lut_width(&BITSCRAMBLER, BITSCRAMBLER_DIR_TX,
                                  BITSCRAMBLER_LUT_WIDTH_16BIT);
    free(lut);
    return ESP_OK;
}

esp_err_t c5vrx2_wbfm_q4_load_tx_iq5(void)
{
    uint16_t *lut = heap_caps_malloc(LUT_BYTES, MALLOC_CAP_INTERNAL);
    if (!lut) return ESP_ERR_NO_MEM;
    build_iq5_lut(lut);
    bitscrambler_ll_set_lut_width(&BITSCRAMBLER, BITSCRAMBLER_DIR_TX,
                                  BITSCRAMBLER_LUT_WIDTH_32BIT);
    const uint32_t *words = (const uint32_t *)lut;
    for (unsigned word = 0u; word < LUT_BYTES / sizeof(uint32_t); ++word)
        bitscrambler_ll_lutmem_write(&BITSCRAMBLER, BITSCRAMBLER_DIR_TX,
                                     (int)word, words[word]);
    bitscrambler_ll_set_lut_width(&BITSCRAMBLER, BITSCRAMBLER_DIR_TX,
                                  BITSCRAMBLER_LUT_WIDTH_16BIT);
    free(lut);
    return ESP_OK;
}

static uint32_t hash_words(const uint32_t *words, size_t count)
{
    uint32_t hash = 2166136261u;
    for (size_t i = 0u; i < count; ++i) {
        uint32_t value = words[i];
        for (unsigned byte = 0u; byte < 4u; ++byte) {
            hash ^= value & 0xffu;
            hash *= 16777619u;
            value >>= 8u;
        }
    }
    return hash;
}

uint32_t c5vrx2_wbfm_q4_verify_tx_iq5_lut(uint32_t *actual_hash,
                                          uint32_t *expected_hash)
{
    uint16_t *expected = heap_caps_malloc(LUT_BYTES, MALLOC_CAP_INTERNAL);
    uint32_t *actual = heap_caps_malloc(LUT_BYTES, MALLOC_CAP_INTERNAL);
    if (!expected || !actual) {
        free(expected);
        free(actual);
        if (actual_hash) *actual_hash = 0u;
        if (expected_hash) *expected_hash = 0u;
        return UINT32_MAX;
    }
    build_iq5_lut(expected);
    bitscrambler_ll_set_lut_width(&BITSCRAMBLER, BITSCRAMBLER_DIR_TX,
                                  BITSCRAMBLER_LUT_WIDTH_32BIT);
    for (unsigned word = 0u; word < LUT_BYTES / sizeof(uint32_t); ++word) {
        BITSCRAMBLER.lut_cfg[BITSCRAMBLER_DIR_TX].cfg0.lut_idx = word;
        actual[word] = BITSCRAMBLER.lut_cfg[BITSCRAMBLER_DIR_TX].cfg1.lut;
    }
    bitscrambler_ll_set_lut_width(&BITSCRAMBLER, BITSCRAMBLER_DIR_TX,
                                  BITSCRAMBLER_LUT_WIDTH_16BIT);
    uint32_t mismatches = 0u;
    const uint32_t *expected_words = (const uint32_t *)expected;
    for (unsigned word = 0u; word < LUT_BYTES / sizeof(uint32_t); ++word)
        mismatches += actual[word] != expected_words[word];
    if (actual_hash)
        *actual_hash = hash_words(actual, LUT_BYTES / sizeof(uint32_t));
    if (expected_hash)
        *expected_hash = hash_words(expected_words,
                                    LUT_BYTES / sizeof(uint32_t));
    free(expected);
    free(actual);
    return mismatches;
}

size_t c5vrx2_wbfm_q4_reference(const uint8_t *input, size_t input_bytes,
                                uint8_t *output, size_t output_bytes)
{
    if (!input || !output) return 0u;
    const size_t pairs = input_bytes / 2u < output_bytes ?
                         input_bytes / 2u : output_bytes;
    uint8_t previous = 0u;
    for (size_t pair = 0u; pair < pairs; ++pair) {
        int phase1_i = q4_phase8(input[pair * 2u]);
        int phase2_i = q4_phase8(input[pair * 2u + 1u]);
        if (c5vrx2_calibration_get()->polarity ==
            C5VRX2_POLARITY_PREVIOUS_MINUS_CURRENT) {
            phase1_i = -phase1_i;
            phase2_i = -phase2_i;
        }
        const uint8_t phase1 = (uint8_t)phase1_i;
        const uint8_t phase2 = (uint8_t)phase2_i;
        const uint8_t sum = (uint8_t)((uint8_t)(phase1 - previous) +
                                      (uint8_t)(phase2 - phase1));
        int signed_sum = sum;
        if (signed_sum >= 128) signed_sum -= 256;
        const c5vrx2_calibration_t *cal = c5vrx2_calibration_get();
        int code = (int)cal->pedestal_code +
                   scale_real_sum(signed_sum, cal->discriminator_gain);
        if (code < 0) code = 0;
        if (code > 63) code = 63;
        output[pair] = (uint8_t)code;
        previous = phase2;
    }
    return pairs;
}

size_t c5vrx2_wbfm_q4_fast_reference(const uint8_t *input,
                                     size_t input_bytes, uint8_t *output,
                                     size_t output_bytes)
{
    if (!input || !output) return 0u;
    const size_t pairs = input_bytes / 2u < output_bytes ?
                         input_bytes / 2u : output_bytes;
    const c5vrx2_calibration_t *cal = c5vrx2_calibration_get();
    uint8_t previous = 0u;
    for (size_t pair = 0u; pair < pairs; ++pair) {
        int phase_i = q4_phase8(input[pair * 2u + 1u]);
        if (cal->polarity == C5VRX2_POLARITY_PREVIOUS_MINUS_CURRENT)
            phase_i = -phase_i;
        const uint8_t phase = (uint8_t)phase_i;
        const uint8_t biased_sum =
            (uint8_t)(2u * cal->pedestal_code + phase - previous);
        output[pair] = (biased_sum >> 1u) & 0x3fu;
        previous = phase;
    }
    return pairs;
}

size_t c5vrx2_wbfm_q4_lut3_reference(const uint8_t *input,
                                     size_t input_bytes, uint8_t *output,
                                     size_t output_bytes)
{
    if (!input || !output) return 0u;
    const size_t pairs = input_bytes / 2u < output_bytes ?
                         input_bytes / 2u : output_bytes;
    const c5vrx2_calibration_t *cal = c5vrx2_calibration_get();
    uint8_t previous = 0u;
    for (size_t pair = 0u; pair < pairs; ++pair) {
        uint8_t current = q4_phase4(input[pair * 2u + 1u]);
        if (cal->polarity == C5VRX2_POLARITY_PREVIOUS_MINUS_CURRENT)
            current = (uint8_t)(0u - current) & 0x0fu;
        int delta = (int)((current - previous) & 0x0fu);
        if (delta >= 8) delta -= 16;
        int code = (int)cal->pedestal_code +
                   scale_real_sum(delta * 16, cal->discriminator_gain);
        if (code < 0) code = 0;
        if (code > 63) code = 63;
        output[pair] = (uint8_t)code;
        previous = current;
    }
    return pairs;
}

size_t c5vrx2_wbfm_q4_iq5_reference(const uint8_t *input,
                                    size_t input_bytes, uint8_t *output,
                                    size_t output_bytes)
{
    if (!input || !output) return 0u;
    const size_t pairs = input_bytes / 2u < output_bytes ?
                         input_bytes / 2u : output_bytes;
    const c5vrx2_calibration_t *cal = c5vrx2_calibration_get();
    uint8_t previous = 0u;
    for (size_t pair = 0u; pair < pairs; ++pair) {
        const uint8_t current = q4_compact_iq5(input[pair * 2u + 1u]);
        int delta = iq5_delta_phase8(previous, current);
        if (cal->polarity == C5VRX2_POLARITY_PREVIOUS_MINUS_CURRENT)
            delta = -delta;
        int code = (int)cal->pedestal_code +
                   scale_real_sum(delta, cal->discriminator_gain);
        if (code < 0) code = 0;
        if (code > 63) code = 63;
        output[pair] = (uint8_t)code;
        previous = current;
    }
    return pairs;
}

size_t c5vrx2_wbfm_q4_phase5_reference(const uint8_t *input,
                                       size_t input_bytes, uint8_t *output,
                                       size_t output_bytes)
{
    if (!input || !output) return 0u;
    const size_t pairs = input_bytes / 2u < output_bytes ?
                         input_bytes / 2u : output_bytes;
    const c5vrx2_calibration_t *cal = c5vrx2_calibration_get();
    uint8_t previous = 0u;
    for (size_t pair = 0u; pair < pairs; ++pair) {
        const uint8_t current = q4_phase5_state(input[pair * 2u + 1u]);
        int delta = phase5_delta_phase8(previous, current);
        if (cal->polarity == C5VRX2_POLARITY_PREVIOUS_MINUS_CURRENT)
            delta = -delta;
        int code = (int)cal->pedestal_code +
                   scale_real_sum(delta, cal->discriminator_gain);
        if (code < 0) code = 0;
        if (code > 63) code = 63;
        output[pair] = (uint8_t)code;
        previous = current;
    }
    return pairs;
}

const void *c5vrx2_wbfm_q4_program(void)
{
    return c5vrx2_wbfm_q4_2to1_program;
}

const void *c5vrx2_wbfm_q4_fast_program(void)
{
    return c5vrx2_wbfm_q4_fast_2to1_program;
}

const void *c5vrx2_wbfm_q4_lut3_program(void)
{
    return c5vrx2_wbfm_q4_lut3_2to1_program;
}

const void *c5vrx2_wbfm_q4_iq5_program(void)
{
    return c5vrx2_wbfm_q4_iq5_2to1_program;
}

const void *c5vrx2_wbfm_q4_phase5_program(void)
{
    return c5vrx2_wbfm_q4_phase5_2to1_program;
}

const void *c5vrx2_wbfm_q4_phase5_100ns_program(void)
{
    return c5vrx2_wbfm_q4_phase5_100ns_2to1_program;
}

#if CONFIG_C5VRX2_WBFM_SELFTEST_ONCE
#define SELFTEST_INPUT_BYTES 1024u
#define SELFTEST_STAGE_COUNT 8u
#define SELFTEST_RESULT_BYTES SELFTEST_INPUT_BYTES
#define SELFTEST_MAGIC 0x31544257u /* little-endian "WBT1" */
#define SELFTEST_SUBTYPE ((esp_partition_subtype_t)0x42)
#define SELFTEST_ALIGNMENT_LIMIT 16u

typedef struct __attribute__((packed)) {
    int32_t run_error;
    uint32_t bytes_written;
    uint32_t bytes_compared;
    uint32_t mismatches;
    uint32_t first_mismatch;
    uint32_t expected_offset;
} selftest_stage_t;

typedef struct __attribute__((packed)) {
    uint32_t magic;
    uint16_t version;
    uint16_t header_bytes;
    uint32_t result_code;
    uint32_t input_bytes;
    selftest_stage_t stage[SELFTEST_STAGE_COUNT];
    uint32_t reserved[12];
} selftest_header_t;

_Static_assert(sizeof(selftest_header_t) == 256u, "selftest header size");

static DMA_ATTR __attribute__((aligned(64))) uint8_t
    s_selftest_input[SELFTEST_INPUT_BYTES];
static DMA_ATTR __attribute__((aligned(64))) uint8_t
    s_selftest_actual[SELFTEST_RESULT_BYTES];
static DRAM_ATTR uint8_t s_selftest_expected[SELFTEST_RESULT_BYTES];

static uint8_t calibrated_phase(uint8_t sample)
{
    int phase = q4_phase8(sample);
    if (c5vrx2_calibration_get()->polarity ==
        C5VRX2_POLARITY_PREVIOUS_MINUS_CURRENT)
        phase = -phase;
    return (uint8_t)phase;
}

static uint8_t map_real_sum(uint8_t sum_mod)
{
    int sum = (int)sum_mod;
    if (sum >= 128) sum -= 256;
    const c5vrx2_calibration_t *cal = c5vrx2_calibration_get();
    int code = (int)cal->pedestal_code +
               scale_real_sum(sum, cal->discriminator_gain);
    if (code < 0) code = 0;
    if (code > 63) code = 63;
    return (uint8_t)code;
}

static void build_expected(unsigned stage, size_t *expected_bytes)
{
    *expected_bytes = (stage == 5u || stage == 8u) ?
                      SELFTEST_INPUT_BYTES / 2u : SELFTEST_INPUT_BYTES;
    if (stage == 1u) {
        for (size_t i = 0; i < *expected_bytes; ++i)
            s_selftest_expected[i] = calibrated_phase(s_selftest_input[i]);
    } else if (stage == 2u || stage == 3u) {
        for (size_t i = 0; i < *expected_bytes; ++i)
            s_selftest_expected[i] =
                (uint8_t)(0u - calibrated_phase(s_selftest_input[i]));
    } else if (stage == 4u) {
        uint8_t previous = 0u;
        for (size_t i = 0; i < *expected_bytes; ++i) {
            const uint8_t phase = calibrated_phase(s_selftest_input[i]);
            s_selftest_expected[i] = (uint8_t)(phase - previous);
            previous = phase;
        }
    } else if (stage == 5u || stage == 8u) {
        uint8_t previous = 0u;
        for (size_t pair = 0; pair < *expected_bytes; ++pair) {
            const uint8_t phase1 = calibrated_phase(s_selftest_input[2u * pair]);
            const uint8_t phase2 = calibrated_phase(s_selftest_input[2u * pair + 1u]);
            const uint8_t sum = (uint8_t)((uint8_t)(phase1 - previous) +
                                          (uint8_t)(phase2 - phase1));
            s_selftest_expected[pair] = stage == 8u ? map_real_sum(sum) : sum;
            previous = phase2;
        }
    } else if (stage == 6u) {
        memcpy(s_selftest_expected, s_selftest_input, *expected_bytes);
    } else if (stage == 7u) {
        for (size_t i = 0; i < *expected_bytes; ++i)
            s_selftest_expected[i] = map_real_sum(s_selftest_input[i]);
    }
    memset(s_selftest_expected + *expected_bytes, 0xa5,
           sizeof(s_selftest_expected) - *expected_bytes);
}

static const void *stage_program(unsigned stage)
{
    switch (stage) {
    case 1: return c5vrx2_q4_phase_program;
    case 2: return c5vrx2_q4_negative_program;
    case 3: return c5vrx2_q4_state_program;
    case 4: return c5vrx2_q4_delta_program;
    case 5: return c5vrx2_q4_pairsum_program;
    case 6:
    case 7: return c5vrx2_counter_lut_program;
    default: return c5vrx2_wbfm_q4_2to1_program;
    }
}

static void build_identity_lut(uint16_t lut[LUT_ITEMS])
{
    for (unsigned index = 0; index < LUT_ITEMS; ++index)
        lut[index] = (uint16_t)(index & 0xffu);
}

static void compare_stage(selftest_stage_t *result, size_t expected_bytes)
{
    const size_t written = result->bytes_written;
    uint32_t best_mismatches = UINT32_MAX;
    uint32_t best_first = UINT32_MAX;
    uint32_t best_offset = 0u;
    uint32_t best_compared = 0u;

    for (size_t offset = 0; offset <= SELFTEST_ALIGNMENT_LIMIT; ++offset) {
        if (offset >= expected_bytes) break;
        const size_t available = expected_bytes - offset;
        const size_t compared = written < available ? written : available;
        uint32_t mismatches = (uint32_t)(written - compared);
        uint32_t first = UINT32_MAX;
        for (size_t i = 0; i < compared; ++i) {
            if (s_selftest_actual[i] != s_selftest_expected[offset + i]) {
                if (first == UINT32_MAX) first = (uint32_t)i;
                mismatches++;
            }
        }
        if (mismatches < best_mismatches) {
            best_mismatches = mismatches;
            best_first = first;
            best_offset = (uint32_t)offset;
            best_compared = (uint32_t)compared;
        }
    }
    /* A healthy prefetching program omits at most the final eight input bytes
     * (four outputs in the 2:1 stages). Do not call that deterministic tail a
     * mismatch, but reject a short/empty transfer. */
    const size_t maximum_tail = expected_bytes == SELFTEST_INPUT_BYTES ? 8u : 4u;
    if (written + maximum_tail < expected_bytes) {
        if (best_first == UINT32_MAX) best_first = (uint32_t)written;
        best_mismatches += (uint32_t)(expected_bytes - maximum_tail - written);
    }
    result->bytes_compared = best_compared;
    result->mismatches = best_mismatches;
    result->first_mismatch = best_first;
    result->expected_offset = best_offset;
}

unsigned c5vrx2_wbfm_q4_selftest_once(void)
{
    /* Traverse all packed Q4/I4 values in a non-periodic order. An incorrect
     * LUT address, state lane, accumulator or bank selection cannot pass by
     * producing a constant or a short repeating pattern. */
    for (size_t i = 0u; i < sizeof(s_selftest_input); ++i)
        s_selftest_input[i] = (uint8_t)(i * 73u + (i >> 3u) * 29u + 17u);

    selftest_header_t header = {
        .magic = SELFTEST_MAGIC,
        .version = 3u,
        .header_bytes = sizeof(selftest_header_t),
        .result_code = 0u,
        .input_bytes = sizeof(s_selftest_input),
    };
    for (unsigned stage = 0; stage < SELFTEST_STAGE_COUNT; ++stage) {
        header.stage[stage].run_error = ESP_ERR_INVALID_STATE;
        header.stage[stage].first_mismatch = UINT32_MAX;
    }

    const esp_partition_t *partition = esp_partition_find_first(
        ESP_PARTITION_TYPE_DATA, SELFTEST_SUBTYPE, "diagcap");
    esp_err_t persist_error = partition ? ESP_OK : ESP_ERR_NOT_FOUND;
    const size_t persisted_bytes = sizeof(header) + SELFTEST_INPUT_BYTES +
        SELFTEST_STAGE_COUNT * 2u * SELFTEST_RESULT_BYTES;
    const size_t erase_bytes = (persisted_bytes + 4095u) & ~4095u;
    if (persist_error == ESP_OK)
        persist_error = esp_partition_erase_range(partition, 0u, erase_bytes);
    if (persist_error == ESP_OK)
        persist_error = esp_partition_write(partition, sizeof(header),
                                             s_selftest_input,
                                             sizeof(s_selftest_input));

    bitscrambler_handle_t handle = NULL;
    /* The generic loopback helper directly connects a pair of GDMA channels
     * to the selected trigger; it does not instantiate/configure that
     * peripheral.  Espressif's C5 loopback tests use the I2S0 trigger.  The
     * PARL_IO trigger only becomes meaningful with the real PARLIO RX unit
     * active, so using it here corrupts even a trivial byte passthrough and
     * says nothing about the program under test. */
    esp_err_t create_error = bitscrambler_loopback_create(
        &handle, SOC_BITSCRAMBLER_ATTACH_I2S0, SELFTEST_INPUT_BYTES);
    uint16_t *lut = heap_caps_malloc(LUT_BYTES, MALLOC_CAP_INTERNAL);
    for (unsigned stage = 1u; stage <= SELFTEST_STAGE_COUNT; ++stage) {
        selftest_stage_t *result = &header.stage[stage - 1u];
        size_t expected_bytes = 0u;
        build_expected(stage, &expected_bytes);
        memset(s_selftest_actual, 0xa5, sizeof(s_selftest_actual));

        esp_err_t err = create_error;
        if (err == ESP_OK && !lut) err = ESP_ERR_NO_MEM;
        if (err == ESP_OK) {
            if (stage == 6u) build_identity_lut(lut);
            else build_lut(lut);
            err = bitscrambler_load_program(handle, stage_program(stage));
        }
        if (err == ESP_OK) err = bitscrambler_load_lut(handle, lut, LUT_BYTES);
        size_t written = 0u;
        uint32_t run_cycles = 0u;
        if (err == ESP_OK) {
            const uint32_t begin = esp_cpu_get_cycle_count();
            err = bitscrambler_loopback_run(
                handle, s_selftest_input, sizeof(s_selftest_input),
                s_selftest_actual, expected_bytes, &written);
            run_cycles = esp_cpu_get_cycle_count() - begin;
        }
        header.reserved[stage - 1u] = run_cycles;
        result->run_error = err;
        result->bytes_written = (uint32_t)written;
        if (err == ESP_OK) compare_stage(result, expected_bytes);
        else result->mismatches = UINT32_MAX;

        if (header.result_code == 0u &&
            (err != ESP_OK || result->mismatches != 0u))
            header.result_code = stage == 8u ? 9u : stage;

        const size_t stage_offset = sizeof(header) + SELFTEST_INPUT_BYTES +
            (stage - 1u) * 2u * SELFTEST_RESULT_BYTES;
        if (persist_error == ESP_OK)
            persist_error = esp_partition_write(partition, stage_offset,
                                                 s_selftest_actual,
                                                 sizeof(s_selftest_actual));
        if (persist_error == ESP_OK)
            persist_error = esp_partition_write(partition,
                stage_offset + SELFTEST_RESULT_BYTES, s_selftest_expected,
                sizeof(s_selftest_expected));
    }
    free(lut);
    header.reserved[8] = CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ * 1000000u;
    if (handle) bitscrambler_free(handle);
    if (header.result_code == 0u) header.result_code = 8u;
    if (persist_error != ESP_OK && header.result_code == 8u)
        header.result_code = 9u;
    if (partition && esp_partition_write(partition, 0u, &header,
                                         sizeof(header)) != ESP_OK)
        header.result_code = 9u;
    return header.result_code;
}
#endif

BITSCRAMBLER_PROGRAM(c5vrx2_wbfm_q4_trajectory_2to1_program,
                    "c5vrx2_wbfm_q4_trajectory_2to1");

esp_err_t c5vrx2_wbfm_q4_configure_trajectory(bitscrambler_handle_t handle)
{
    if (!handle) return ESP_ERR_INVALID_ARG;
    return bitscrambler_load_program(handle, c5vrx2_wbfm_q4_trajectory_2to1_program);
}

const void *c5vrx2_wbfm_q4_trajectory_program(void)
{
    return c5vrx2_wbfm_q4_trajectory_2to1_program;
}

BITSCRAMBLER_PROGRAM(c5vrx2_wbfm_q4_true40_2to1_program,
                    "c5vrx2_wbfm_q4_true40_2to1");

esp_err_t c5vrx2_wbfm_q4_configure_true40(bitscrambler_handle_t handle)
{
    if (!handle) return ESP_ERR_INVALID_ARG;
    return bitscrambler_load_program(handle, c5vrx2_wbfm_q4_true40_2to1_program);
}

const void *c5vrx2_wbfm_q4_true40_program(void)
{
    return c5vrx2_wbfm_q4_true40_2to1_program;
}
