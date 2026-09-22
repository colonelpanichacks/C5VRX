/* SPDX-License-Identifier: GPL-3.0-only */

#include "c5vrx_wbfm_hw.h"

#include <math.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

#include "c5vrx_adc_dump.h"
#include "driver/bitscrambler.h"
#include "driver/bitscrambler_loopback.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "soc/soc_caps.h"

static const char *TAG = "c5vrx_wbfm_hw";

BITSCRAMBLER_PROGRAM(c5vrx_wbfm_4to1_program, "c5vrx_wbfm_4to1");
BITSCRAMBLER_PROGRAM(c5vrx_wbfm_phase8_4to1_program,
                    "c5vrx_wbfm_phase8_4to1");

#define C5VRX_WBFM_TEST_OUTPUT_SAMPLES 256u
#define C5VRX_WBFM_TEST_DECIMATION     4u
#define C5VRX_WBFM_TEST_INPUT_WORDS    (C5VRX_WBFM_TEST_OUTPUT_SAMPLES * C5VRX_WBFM_TEST_DECIMATION)
#define C5VRX_WBFM_PHASE_BIAS          32u
#define C5VRX_WBFM_LUT_WORDS           1024u
#define C5VRX_PI_F                     3.14159265358979323846f

struct c5vrx_wbfm_hw_context {
    bitscrambler_handle_t bs;
    uint16_t *lut;
    size_t maximum_input_words;
    int32_t i_dc_q16;
    int32_t q_dc_q16;
    int8_t applied_i_dc5;
    int8_t applied_q_dc5;
    c5vrx_wbfm_kernel_t kernel;
};

static uint32_t pack_iq10(int16_t i, int16_t q)
{
    return (((uint32_t)i & 0x3ffu) << 10) | ((uint32_t)q & 0x3ffu);
}

static float coarse_signed_center(unsigned code5)
{
    const int signed5 = (code5 & 0x10u) ? (int)code5 - 32 : (int)code5;
    return (float)signed5 * 32.0f + 15.5f;
}

static uint8_t coarse_phase6_from_iq_dc(int16_t i, int16_t q,
                                        int dc_i, int dc_q)
{
    const unsigned i5 = (((uint16_t)i & 0x3ffu) >> 5) & 0x1fu;
    const unsigned q5 = (((uint16_t)q & 0x3ffu) >> 5) & 0x1fu;
    float phase = atan2f(coarse_signed_center(q5) - (float)dc_q,
                         coarse_signed_center(i5) - (float)dc_i);
    if (phase < 0.0f) {
        phase += 2.0f * C5VRX_PI_F;
    }
    return (uint8_t)lrintf(phase * (64.0f / (2.0f * C5VRX_PI_F))) & 0x3fu;
}

static uint8_t coarse_phase6_from_iq(int16_t i, int16_t q)
{
    return coarse_phase6_from_iq_dc(i, q, 0, 0);
}

static uint8_t coarse_phase_from_iq_bits(int16_t i, int16_t q,
                                         unsigned phase_bits)
{
    const unsigned i5 = (((uint16_t)i & 0x3ffu) >> 5) & 0x1fu;
    const unsigned q5 = (((uint16_t)q & 0x3ffu) >> 5) & 0x1fu;
    float phase = atan2f(coarse_signed_center(q5), coarse_signed_center(i5));
    if (phase < 0.0f) phase += 2.0f * C5VRX_PI_F;
    const unsigned modulus = 1u << phase_bits;
    return (uint8_t)lrintf(
        phase * ((float)modulus / (2.0f * C5VRX_PI_F)));
}

uint8_t c5vrx_wbfm_coarse_phase6(uint32_t packed_iq)
{
    const c5vrx_iq10_sample_t sample = c5vrx_adc_decode_word(packed_iq);
    return coarse_phase6_from_iq(sample.i, sample.q);
}

static void build_phase_lut(uint16_t lut[C5VRX_WBFM_LUT_WORDS],
                            int dc_i, int dc_q, unsigned phase_bits)
{
    for (unsigned i5 = 0; i5 < 32u; ++i5) {
        for (unsigned q5 = 0; q5 < 32u; ++q5) {
            float phase = atan2f(coarse_signed_center(q5) - (float)dc_q,
                                 coarse_signed_center(i5) - (float)dc_i);
            if (phase < 0.0f) {
                phase += 2.0f * C5VRX_PI_F;
            }
            const unsigned modulus = 1u << phase_bits;
            const uint8_t p = (uint8_t)lrintf(
                phase * ((float)modulus / (2.0f * C5VRX_PI_F)));
            const uint8_t bias_minus =
                (uint8_t)((modulus / 2u) - p);
            lut[(i5 << 5) | q5] =
                (uint16_t)p | ((uint16_t)bias_minus << 8);
        }
    }
}

uint8_t c5vrx_wbfm_hw_context_phase6(
    const c5vrx_wbfm_hw_context_t *context, uint32_t packed_iq)
{
    if (!context) return c5vrx_wbfm_coarse_phase6(packed_iq);
    const c5vrx_iq10_sample_t sample = c5vrx_adc_decode_word(packed_iq);
    return coarse_phase6_from_iq_dc(
        sample.i, sample.q,
        (int)context->applied_i_dc5 * 32,
        (int)context->applied_q_dc5 * 32);
}

static esp_err_t update_dc_lut(c5vrx_wbfm_hw_context_t *ctx,
                               const uint32_t *packed_iq,
                               size_t input_words)
{
    /* 1/64 sparse observations keep this control loop cheap. The 2^12 IIR
     * time constant spans many blocks and cannot follow FM modulation. */
    for (size_t i = 0; i < input_words; i += 64u) {
        const c5vrx_iq10_sample_t sample = c5vrx_adc_decode_word(packed_iq[i]);
        ctx->i_dc_q16 += (((int32_t)sample.i << 16) - ctx->i_dc_q16) >> 12;
        ctx->q_dc_q16 += (((int32_t)sample.q << 16) - ctx->q_dc_q16) >> 12;
    }
    int i5 = ctx->i_dc_q16 / (32 << 16);
    int q5 = ctx->q_dc_q16 / (32 << 16);
    if (i5 < -16) i5 = -16;
    if (i5 > 15) i5 = 15;
    if (q5 < -16) q5 = -16;
    if (q5 > 15) q5 = 15;
    if (i5 == ctx->applied_i_dc5 && q5 == ctx->applied_q_dc5) return ESP_OK;
    build_phase_lut(ctx->lut, i5 * 32, q5 * 32,
                    ctx->kernel == C5VRX_WBFM_PHASE8_4TO1 ? 8u : 6u);
    const esp_err_t err = bitscrambler_load_lut(
        ctx->bs, ctx->lut, C5VRX_WBFM_LUT_WORDS * sizeof(uint16_t));
    if (err == ESP_OK) {
        ctx->applied_i_dc5 = (int8_t)i5;
        ctx->applied_q_dc5 = (int8_t)q5;
    }
    return err;
}

esp_err_t c5vrx_wbfm_hw_create(size_t maximum_input_words,
                               c5vrx_wbfm_hw_context_t **context)
{
    return c5vrx_wbfm_hw_create_kernel(
        maximum_input_words, C5VRX_WBFM_PHASE6_4TO1, context);
}

esp_err_t c5vrx_wbfm_hw_create_kernel(
    size_t maximum_input_words, c5vrx_wbfm_kernel_t kernel,
    c5vrx_wbfm_hw_context_t **context)
{
    if (!context || maximum_input_words < 8u ||
        (maximum_input_words & 3u) || kernel > C5VRX_WBFM_PHASE8_4TO1) {
        return ESP_ERR_INVALID_ARG;
    }
    *context = NULL;
    c5vrx_wbfm_hw_context_t *ctx = calloc(1, sizeof(*ctx));
    if (!ctx) return ESP_ERR_NO_MEM;

    ctx->lut = heap_caps_malloc(C5VRX_WBFM_LUT_WORDS * sizeof(uint16_t),
                                MALLOC_CAP_INTERNAL);
    if (!ctx->lut) {
        free(ctx);
        return ESP_ERR_NO_MEM;
    }
    ctx->kernel = kernel;
    build_phase_lut(ctx->lut, 0, 0,
                    kernel == C5VRX_WBFM_PHASE8_4TO1 ? 8u : 6u);

    const size_t max_transfer = maximum_input_words * sizeof(uint32_t);
    esp_err_t err = bitscrambler_loopback_create(
        &ctx->bs,
        SOC_BITSCRAMBLER_ATTACH_I2S0,
        max_transfer);
    if (err == ESP_OK) {
        err = bitscrambler_load_program(ctx->bs,
            kernel == C5VRX_WBFM_PHASE8_4TO1 ?
                c5vrx_wbfm_phase8_4to1_program : c5vrx_wbfm_4to1_program);
    }
    if (err == ESP_OK) {
        err = bitscrambler_load_lut(ctx->bs, ctx->lut,
                                    C5VRX_WBFM_LUT_WORDS * sizeof(uint16_t));
    }
    if (err != ESP_OK) {
        if (ctx->bs) bitscrambler_free(ctx->bs);
        free(ctx->lut);
        free(ctx);
        return err;
    }
    ctx->maximum_input_words = maximum_input_words;
    *context = ctx;
    return ESP_OK;
}

unsigned c5vrx_wbfm_hw_phase_bits(const c5vrx_wbfm_hw_context_t *context)
{
    return context && context->kernel == C5VRX_WBFM_PHASE8_4TO1 ? 8u : 6u;
}

void c5vrx_wbfm_hw_destroy(c5vrx_wbfm_hw_context_t *context)
{
    if (!context) return;
    if (context->bs) bitscrambler_free(context->bs);
    free(context->lut);
    free(context);
}

esp_err_t c5vrx_wbfm_hw_transform_context(
    c5vrx_wbfm_hw_context_t *context,
    const uint32_t *packed_iq,
    size_t input_words,
    uint8_t *phase_delta,
    size_t output_capacity,
    size_t *output_written)
{
    if (output_written) *output_written = 0;
    if (!context || !packed_iq || !phase_delta || input_words < 8u ||
        (input_words & 3u) || input_words > context->maximum_input_words) {
        return ESP_ERR_INVALID_ARG;
    }
    const size_t expected_output = input_words / C5VRX_WBFM_TEST_DECIMATION;
    if (output_capacity < expected_output) return ESP_ERR_INVALID_SIZE;

    esp_err_t err = update_dc_lut(context, packed_iq, input_words);
    if (err != ESP_OK) return err;

    size_t written = 0;
    err = bitscrambler_loopback_run(
        context->bs, (void *)packed_iq, input_words * sizeof(uint32_t),
        phase_delta, output_capacity, &written);

    if (err == ESP_OK && output_written) *output_written = written;
    return err;
}

esp_err_t c5vrx_wbfm_hw_transform(const uint32_t *packed_iq,
                                   size_t input_words,
                                   uint8_t *phase_delta,
                                   size_t output_capacity,
                                   size_t *output_written)
{
    c5vrx_wbfm_hw_context_t *context = NULL;
    esp_err_t err = c5vrx_wbfm_hw_create(input_words, &context);
    if (err == ESP_OK) {
        err = c5vrx_wbfm_hw_transform_context(
            context, packed_iq, input_words, phase_delta,
            output_capacity, output_written);
    }
    c5vrx_wbfm_hw_destroy(context);
    return err;
}

esp_err_t c5vrx_wbfm_hw_probe_dump(size_t sample_count)
{
    if (sample_count < 8u || sample_count > C5VRX_ADC_DUMP_MAX_SAMPLES ||
        (sample_count & 3u) != 0u) {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t err = c5vrx_adc_dump_capture(sample_count, false);
    if (err != ESP_OK) {
        return err;
    }

    const size_t output_capacity = sample_count / C5VRX_WBFM_TEST_DECIMATION;
    uint32_t *iq_copy = heap_caps_malloc(sample_count * sizeof(uint32_t),
                                         MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);
    uint8_t *fm = heap_caps_calloc(output_capacity, 1,
                                   MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);
    if (!iq_copy || !fm) {
        free(iq_copy);
        free(fm);
        return ESP_ERR_NO_MEM;
    }

    volatile const uint32_t *const dump =
        (volatile const uint32_t *)(uintptr_t)C5VRX_ADC_DUMP_BASE_ADDR;
    for (size_t i = 0; i < sample_count; ++i) {
        iq_copy[i] = dump[i];
    }

    size_t written = 0;
    err = c5vrx_wbfm_hw_transform(
        iq_copy,
        sample_count,
        fm,
        output_capacity,
        &written);

    if (err == ESP_OK && written > 1u) {
        unsigned min_code = 63u;
        unsigned max_code = 0u;
        uint64_t sum = 0;
        uint64_t abs_dev_sum = 0;
        for (size_t i = 1; i < written; ++i) {
            const unsigned code = fm[i] & 0x3fu;
            if (code < min_code) min_code = code;
            if (code > max_code) max_code = code;
            sum += code;
            abs_dev_sum += code >= C5VRX_WBFM_PHASE_BIAS
                ? code - C5VRX_WBFM_PHASE_BIAS
                : C5VRX_WBFM_PHASE_BIAS - code;
        }
        const size_t valid = written - 1u;
        ESP_LOGW(TAG,
                 "finite RF -> BitScrambler WBFM proof: iq=%u, fm=%u (4:1 retained-sample transform; physical rates UNKNOWN), code min/max=%u/%u mean=%u mean_abs_dev_from_bias=%u",
                 (unsigned)sample_count,
                 (unsigned)written,
                 min_code,
                 max_code,
                 (unsigned)(sum / valid),
                 (unsigned)(abs_dev_sum / valid));
        printf("C5VRX_WBFM_METRICS iq_samples=%u fm_samples=%u decimation=4 min=%u max=%u mean=%u mean_abs_dev_from_bias=%u physical_rates=UNKNOWN code=0\n",
               (unsigned)sample_count, (unsigned)written,
               min_code, max_code, (unsigned)(sum / valid),
               (unsigned)(abs_dev_sum / valid));
        fflush(stdout);
    }

    free(iq_copy);
    free(fm);
    return err;
}

esp_err_t c5vrx_wbfm_hw_self_test(void)
{
    return c5vrx_wbfm_hw_self_test_kernel(C5VRX_WBFM_PHASE6_4TO1);
}

esp_err_t c5vrx_wbfm_hw_self_test_kernel(c5vrx_wbfm_kernel_t kernel)
{
    if (kernel > C5VRX_WBFM_PHASE8_4TO1) return ESP_ERR_INVALID_ARG;
    const unsigned phase_bits =
        kernel == C5VRX_WBFM_PHASE8_4TO1 ? 8u : 6u;
    const unsigned modulus = 1u << phase_bits;
    const unsigned bias = modulus / 2u;
    const size_t input_bytes = C5VRX_WBFM_TEST_INPUT_WORDS * sizeof(uint32_t);
    const size_t output_bytes = C5VRX_WBFM_TEST_OUTPUT_SAMPLES;

    uint32_t *input = heap_caps_malloc(input_bytes,
                                       MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);
    uint8_t *output = heap_caps_calloc(output_bytes, 1,
                                       MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);
    if (!input || !output) {
        free(input);
        free(output);
        return ESP_ERR_NO_MEM;
    }

    const float kept_step = 5.0f * (2.0f * C5VRX_PI_F / 64.0f);
    const float input_step = kept_step / (float)C5VRX_WBFM_TEST_DECIMATION;

    uint8_t kept_phase[C5VRX_WBFM_TEST_OUTPUT_SAMPLES];
    for (unsigned n = 0; n < C5VRX_WBFM_TEST_INPUT_WORDS; ++n) {
        const float phase = input_step * (float)n;
        const int16_t i = (int16_t)lrintf(400.0f * cosf(phase));
        const int16_t q = (int16_t)lrintf(400.0f * sinf(phase));
        input[n] = pack_iq10(i, q);
        if ((n % C5VRX_WBFM_TEST_DECIMATION) == 0) {
            kept_phase[n / C5VRX_WBFM_TEST_DECIMATION] =
                coarse_phase_from_iq_bits(i, q, phase_bits);
        }
    }

    size_t written = 0;
    c5vrx_wbfm_hw_context_t *context = NULL;
    esp_err_t err = c5vrx_wbfm_hw_create_kernel(
        C5VRX_WBFM_TEST_INPUT_WORDS, kernel, &context);
    if (err == ESP_OK)
        err = c5vrx_wbfm_hw_transform_context(
            context, input, C5VRX_WBFM_TEST_INPUT_WORDS,
            output, output_bytes, &written);
    c5vrx_wbfm_hw_destroy(context);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "BitScrambler self-test transform failed: %s",
                 esp_err_to_name(err));
        free(input);
        free(output);
        return err;
    }

    if (written < C5VRX_WBFM_TEST_OUTPUT_SAMPLES - 1u) {
        ESP_LOGE(TAG, "short BitScrambler output: %u bytes", (unsigned)written);
        free(input);
        free(output);
        return ESP_FAIL;
    }

    unsigned mismatches = 0;
    const unsigned compare_n =
        written < C5VRX_WBFM_TEST_OUTPUT_SAMPLES ? (unsigned)written
                                                 : C5VRX_WBFM_TEST_OUTPUT_SAMPLES;

    /* output[0] is the priming sample after counter reset. */
    for (unsigned k = 1; k < compare_n; ++k) {
        const uint8_t expected = (uint8_t)(
            bias + kept_phase[k] - kept_phase[k - 1]) & (modulus - 1u);
        const uint8_t actual = output[k];
        if (actual != expected) {
            if (mismatches < 8u) {
                ESP_LOGE(TAG,
                         "mismatch k=%u actual=%u expected=%u phase=%u prev=%u",
                         k, actual, expected,
                         (unsigned)kept_phase[k],
                         (unsigned)kept_phase[k - 1]);
            }
            ++mismatches;
        }
    }

    ESP_LOGI(TAG,
             "BitScrambler phase%u 4:1 WBFM proof: input=%u bytes output=%u bytes mismatches=%u (first byte ignored as priming)",
             phase_bits,
             (unsigned)input_bytes,
             (unsigned)written,
             mismatches);

    free(input);
    free(output);
    return mismatches == 0u ? ESP_OK : ESP_FAIL;
}
