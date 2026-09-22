/* SPDX-License-Identifier: GPL-3.0-only */

#include "c5vrx_adc_dump.h"

#include <inttypes.h>
#include <limits.h>
#include <stdio.h>
#include <string.h>
#include <math.h>

#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "c5vrx_rf_dump_producer.h"
#include "c5vrx_usb_preview.h"
#include "c5vrx_usb_transport.h"

#define printf c5vrx_usb_printf

static const char *TAG = "c5vrx_adc_dump";
static portMUX_TYPE s_capture_lock = portMUX_INITIALIZER_UNLOCKED;

/* Recovered from C5 v6.0.2 adctrig() disassembly. These are intentionally
 * private diagnostic constants, not presented as documented SoC registers. */
#define C5VRX_FE_DUMP_CONTROL_REG 0x600a9004u
#define C5VRX_FE_DUMP_POINTER_REG 0x600a9008u
#define C5VRX_FE_DUMP_DONE_BIT    (1u << 18)

#define C5VRX_USB_HEADER_BYTES 32u
#define C5VRX_USB_IQ_CHUNK_DESCRIPTOR_BYTES 20u
#define C5VRX_USB_PACKET_IQ_U32_CHUNK 5u
#define C5VRX_USB_PACKET_PHASE8_CHUNK 6u
#define C5VRX_USB_IQ_CHUNK_WORDS 256u
#define C5VRX_USB_PHASE8_CHUNK_SAMPLES 1024u
#define C5VRX_USB_IQ_CHUNK_FLAG_FIRST (1u << 0)
#define C5VRX_USB_IQ_CHUNK_FLAG_LAST  (1u << 1)

static const uint8_t s_iq_usb_magic[8] = {
    0x00, 'C', '5', 'V', 'R', 'X', 0xa5, 0x5a
};
static uint32_t s_iq_usb_sequence;
static uint32_t s_iq_capture_id;
static uint64_t s_last_capture_completed_us;
static uint8_t s_phase8_lut[1024];
static uint8_t s_phase8_chunk[C5VRX_USB_PHASE8_CHUNK_SAMPLES];
static bool s_phase8_lut_ready;

/*
 * Prototype is independently documented by historical Espressif RF-test
 * tooling and matches the C5 v6.0.2 RISC-V argument usage, including the ninth
 * argument being loaded from the caller's stack.
 */
extern void adctrig(int32_t smp_num_aft_trig,
                    int32_t trigmode,
                    int32_t trigcase,
                    int32_t sample_80m,
                    int32_t dump_trig,
                    int32_t rx_gain_mode,
                    int32_t rx_gain,
                    int32_t rx_gain0,
                    int32_t rx_gain0_wait_us);
extern void set_dump_mode(int mode);

static int16_t sign_extend_10(uint32_t value)
{
    value &= 0x3ffu;
    return (value & 0x200u) ? (int16_t)(value - 0x400u) : (int16_t)value;
}

c5vrx_iq10_sample_t c5vrx_adc_decode_word(uint32_t raw)
{
    return (c5vrx_iq10_sample_t) {
        .i = sign_extend_10(raw >> 10),
        .q = sign_extend_10(raw),
        .raw = raw,
    };
}

typedef struct {
    uint32_t final_control;
    size_t changed_words;
    size_t transition_words;
    int64_t elapsed_us;
    uint64_t completed_us;
    bool complete;
    const char *engine;
} capture_result_t;

static uint32_t capture_sentinel(size_t index)
{
    return 0xa5c30000u ^ ((uint32_t)index * 0x9e3779b9u);
}

static void prepare_capture_sentinel(size_t sample_count)
{
    volatile uint32_t *const words =
        (volatile uint32_t *)(uintptr_t)C5VRX_ADC_DUMP_BASE_ADDR;
    for (size_t i = 0; i < sample_count; ++i) words[i] = capture_sentinel(i);
    __asm__ __volatile__("fence iorw, iorw" ::: "memory");
}

static size_t count_changed_capture_words(size_t sample_count)
{
    volatile const uint32_t *const words =
        (volatile const uint32_t *)(uintptr_t)C5VRX_ADC_DUMP_BASE_ADDR;
    size_t changed = 0;
    for (size_t i = 0; i < sample_count; ++i)
        if (words[i] != capture_sentinel(i)) ++changed;
    return changed;
}

static size_t count_capture_transitions(size_t sample_count)
{
    if (sample_count < 2u) return 0u;
    volatile const uint32_t *const words =
        (volatile const uint32_t *)(uintptr_t)C5VRX_ADC_DUMP_BASE_ADDR;
    size_t transitions = 0;
    uint32_t previous = words[0];
    for (size_t i = 1; i < sample_count; ++i) {
        const uint32_t current = words[i];
        if (current != previous) ++transitions;
        previous = current;
    }
    return transitions;
}

static capture_result_t trigger_dump(size_t sample_count)
{
    capture_result_t result = {0};
    prepare_capture_sentinel(sample_count);

    /* The complete vendor path produced the original RF-dependent IQ. The
     * LP register clone did not reliably start the producer and could leave
     * the subsequent vendor call returning a constant block, so it is not
     * placed in front of this path. The dump-owned RAM is heap-reserved.
     * Keep the tick/interrupt watchdog hook from running while the vendor has
     * changed SRAM ownership; hardware measures the successful call at ~2 ms. */
    set_dump_mode(0);
    const int64_t vendor_begin_us = esp_timer_get_time();
    portENTER_CRITICAL(&s_capture_lock);
    adctrig((int32_t)sample_count - 1,
            0,  /* software trigger */
            0,  /* trigger case */
            1,  /* historical field; physical rate remains unproven */
            0,  /* trigger then dump */
            0,  /* automatic RX gain */
            0, 0, 0);
    portEXIT_CRITICAL(&s_capture_lock);
    const int64_t vendor_end_us = esp_timer_get_time();
    result.elapsed_us = vendor_end_us - vendor_begin_us;
    result.completed_us = (uint64_t)vendor_end_us;
    __asm__ __volatile__("fence iorw, iorw" ::: "memory");
    result.final_control =
        *(volatile const uint32_t *)(uintptr_t)C5VRX_FE_DUMP_CONTROL_REG;
    result.changed_words = count_changed_capture_words(sample_count);
    result.transition_words = count_capture_transitions(sample_count);
    const size_t tolerance = sample_count / 100u + 1u;
    const size_t minimum_transitions = sample_count / 100u + 1u;
    result.complete = result.elapsed_us < 900000 &&
                      result.changed_words + tolerance >= sample_count &&
                      result.transition_words >= minimum_transitions;
    result.engine = "VENDOR_ADCTRIG_SENTINEL_VARIANCE_VALIDATED";
    return result;
}

static uint32_t fnv1a_dump_hash(volatile const uint32_t *words, size_t sample_count)
{
    uint32_t hash = 2166136261u;
    for (size_t i = 0; i < sample_count; ++i) {
        uint32_t v = words[i];
        for (unsigned b = 0; b < 4; ++b) {
            hash ^= v & 0xffu;
            hash *= 16777619u;
            v >>= 8;
        }
    }
    return hash;
}

static void put_le16(uint8_t *out, uint16_t value)
{
    out[0] = (uint8_t)value;
    out[1] = (uint8_t)(value >> 8u);
}

static void put_le32(uint8_t *out, uint32_t value)
{
    for (unsigned i = 0; i < 4u; ++i) out[i] = (uint8_t)(value >> (8u * i));
}

static void put_le64(uint8_t *out, uint64_t value)
{
    for (unsigned i = 0; i < 8u; ++i) out[i] = (uint8_t)(value >> (8u * i));
}

static uint32_t crc32_update(uint32_t crc, const uint8_t *data, size_t count)
{
    for (size_t i = 0; i < count; ++i) {
        crc ^= data[i];
        for (unsigned bit = 0; bit < 8u; ++bit)
            crc = (crc >> 1u) ^ (0xedb88320u & (0u - (crc & 1u)));
    }
    return crc;
}

static uint32_t crc32_bytes(const uint8_t *data, size_t count)
{
    return ~crc32_update(UINT32_MAX, data, count);
}

static uint32_t iq_payload_crc(const uint8_t descriptor[C5VRX_USB_IQ_CHUNK_DESCRIPTOR_BYTES],
                               volatile const uint32_t *words,
                               size_t sample_count)
{
    uint32_t crc = crc32_update(UINT32_MAX, descriptor,
                                C5VRX_USB_IQ_CHUNK_DESCRIPTOR_BYTES);
    uint8_t raw[4];
    for (size_t i = 0; i < sample_count; ++i) {
        put_le32(raw, words[i]);
        crc = crc32_update(crc, raw, sizeof(raw));
    }
    return ~crc;
}

static uint32_t byte_payload_crc(
    const uint8_t descriptor[C5VRX_USB_IQ_CHUNK_DESCRIPTOR_BYTES],
    const uint8_t *samples, size_t sample_count)
{
    uint32_t crc = crc32_update(UINT32_MAX, descriptor,
                                C5VRX_USB_IQ_CHUNK_DESCRIPTOR_BYTES);
    crc = crc32_update(crc, samples, sample_count);
    return ~crc;
}

static unsigned write_binary_iq_chunks(volatile const uint32_t *words,
                                       size_t sample_count,
                                       uint64_t capture_timestamp_us)
{
    const uint32_t capture_id = s_iq_capture_id++;
    unsigned failed_chunks = 0;

    for (size_t offset = 0; offset < sample_count;
         offset += C5VRX_USB_IQ_CHUNK_WORDS) {
        size_t chunk_words = sample_count - offset;
        if (chunk_words > C5VRX_USB_IQ_CHUNK_WORDS)
            chunk_words = C5VRX_USB_IQ_CHUNK_WORDS;

        uint32_t flags = 0u;
        if (offset == 0u) flags |= C5VRX_USB_IQ_CHUNK_FLAG_FIRST;
        if (offset + chunk_words == sample_count)
            flags |= C5VRX_USB_IQ_CHUNK_FLAG_LAST;

        uint8_t descriptor[C5VRX_USB_IQ_CHUNK_DESCRIPTOR_BYTES] = {0};
        put_le32(descriptor, capture_id);
        put_le32(descriptor + 4u, (uint32_t)sample_count);
        put_le32(descriptor + 8u, (uint32_t)offset);
        put_le32(descriptor + 12u, (uint32_t)chunk_words);
        put_le32(descriptor + 16u, flags);

        uint8_t header[C5VRX_USB_HEADER_BYTES] = {0};
        memcpy(header, s_iq_usb_magic, sizeof(s_iq_usb_magic));
        header[8] = C5VRX_USB_PREVIEW_PROTOCOL_VERSION;
        header[9] = C5VRX_USB_PACKET_IQ_U32_CHUNK;
        put_le16(header + 10u, C5VRX_USB_HEADER_BYTES);
        put_le32(header + 12u, s_iq_usb_sequence++);
        put_le32(header + 16u,
                 C5VRX_USB_IQ_CHUNK_DESCRIPTOR_BYTES +
                     (uint32_t)chunk_words * sizeof(uint32_t));
        /* Timestamp the sampled RF block, not this later USB write. All
         * fragments from one capture deliberately carry the same acquisition
         * timestamp so the host raster PLL is independent of encode/USB
         * latency and cannot roll vertically under transport load. */
        put_le64(header + 20u, capture_timestamp_us);
        put_le32(header + 28u, crc32_bytes(header, 28u));

        const uint32_t payload_crc =
            iq_payload_crc(descriptor, words + offset, chunk_words);
        uint8_t trailer[4];
        put_le32(trailer, payload_crc);

        /* A 1 KiB fragment fits comfortably inside the configured 4 KiB USB
         * TX buffer. Each fragment has its own header and payload CRC, so a
         * short write loses at most one fragment and the next magic marker
         * restores framing. */
        const c5vrx_usb_iovec_t packet[] = {
            {.data = header, .size = sizeof(header)},
            {.data = descriptor, .size = sizeof(descriptor)},
            {.data = (const void *)(words + offset),
             .size = chunk_words * sizeof(uint32_t)},
            {.data = trailer, .size = sizeof(trailer)},
        };
        if (c5vrx_usb_writev(packet, sizeof(packet) / sizeof(packet[0])) !=
            ESP_OK) ++failed_chunks;
    }
    return failed_chunks;
}

static esp_err_t write_binary_phase8_chunk(const uint8_t *samples,
                                           size_t total_samples,
                                           size_t offset,
                                           size_t chunk_samples,
                                           uint32_t capture_id,
                                           uint64_t capture_timestamp_us)
{
    uint32_t flags = 0u;
    if (offset == 0u) flags |= C5VRX_USB_IQ_CHUNK_FLAG_FIRST;
    if (offset + chunk_samples == total_samples)
        flags |= C5VRX_USB_IQ_CHUNK_FLAG_LAST;

    uint8_t descriptor[C5VRX_USB_IQ_CHUNK_DESCRIPTOR_BYTES] = {0};
    put_le32(descriptor, capture_id);
    put_le32(descriptor + 4u, (uint32_t)total_samples);
    put_le32(descriptor + 8u, (uint32_t)offset);
    put_le32(descriptor + 12u, (uint32_t)chunk_samples);
    put_le32(descriptor + 16u, flags);

    uint8_t header[C5VRX_USB_HEADER_BYTES] = {0};
    memcpy(header, s_iq_usb_magic, sizeof(s_iq_usb_magic));
    header[8] = C5VRX_USB_PREVIEW_PROTOCOL_VERSION;
    header[9] = C5VRX_USB_PACKET_PHASE8_CHUNK;
    put_le16(header + 10u, C5VRX_USB_HEADER_BYTES);
    put_le32(header + 12u, s_iq_usb_sequence++);
    put_le32(header + 16u,
             C5VRX_USB_IQ_CHUNK_DESCRIPTOR_BYTES +
                 (uint32_t)chunk_samples);
    put_le64(header + 20u, capture_timestamp_us);
    put_le32(header + 28u, crc32_bytes(header, 28u));

    const uint32_t payload_crc =
        byte_payload_crc(descriptor, samples, chunk_samples);
    uint8_t trailer[4];
    put_le32(trailer, payload_crc);
    const c5vrx_usb_iovec_t packet[] = {
        {.data = header, .size = sizeof(header)},
        {.data = descriptor, .size = sizeof(descriptor)},
        {.data = samples, .size = chunk_samples},
        {.data = trailer, .size = sizeof(trailer)},
    };
    return c5vrx_usb_writev(packet, sizeof(packet) / sizeof(packet[0]));
}

static float phase8_coarse_center(unsigned code5)
{
    const int signed5 = (code5 & 0x10u) ? (int)code5 - 32 : (int)code5;
    return (float)signed5 * 32.0f + 15.5f;
}

static void initialize_phase8_lut(void)
{
    const float pi = 3.14159265358979323846f;
    if (s_phase8_lut_ready) return;
    for (unsigned i5 = 0; i5 < 32u; ++i5) {
        for (unsigned q5 = 0; q5 < 32u; ++q5) {
            float phase = atan2f(phase8_coarse_center(q5),
                                 phase8_coarse_center(i5));
            if (phase < 0.0f) phase += 2.0f * pi;
            s_phase8_lut[(i5 << 5) | q5] =
                (uint8_t)lrintf(phase * (256.0f / (2.0f * pi)));
        }
    }
    s_phase8_lut_ready = true;
}

static int16_t clamp_iq10(int32_t value)
{
    if (value < -512) return -512;
    if (value > 511) return 511;
    return (int16_t)value;
}

static esp_err_t capture_internal(size_t sample_count, bool print_raw_words,
                                  bool summarize)
{
    if (sample_count == 0 || sample_count > C5VRX_ADC_DUMP_MAX_SAMPLES) {
        ESP_LOGE(TAG, "sample_count must be 1..%u", (unsigned)C5VRX_ADC_DUMP_MAX_SAMPLES);
        return ESP_ERR_INVALID_ARG;
    }
    if (!c5vrx_rf_dump_memory_reserved()) {
        ESP_LOGE(TAG, "RF dump RAM is not excluded from the system heap");
        return ESP_ERR_INVALID_STATE;
    }

    ESP_LOGW(TAG,
             "EXPERIMENTAL finite IQ capture: %u samples, buffer=0x%08" PRIx32 ", historical rate argument=1 (physical rate UNKNOWN)",
             (unsigned)sample_count, (uint32_t)C5VRX_ADC_DUMP_BASE_ADDR);

    /* Mode 0 is the recovered normal packed 10-bit I/Q dump path. */
    set_dump_mode(0);
    const capture_result_t capture = trigger_dump(sample_count);
    s_last_capture_completed_us = capture.completed_us;
    const int64_t capture_elapsed_us = capture.elapsed_us;
    const bool capture_done = capture.complete;
    const double finite_fill_msps =
        capture_done && capture_elapsed_us > 0
            ? (double)sample_count / (double)capture_elapsed_us
            : 0.0;
    printf("C5VRX_CAPTURE_KERNEL engine=%s final_control=%08" PRIx32
           " done=%u timeout=%u elapsed_us=%" PRId64
           " changed_words=%u transition_words=%u requested_words=%u"
           " finite_fill_msps=%.6f rate_classification=%s\n",
           capture.engine,
           capture.final_control,
           capture_done ? 1u : 0u,
           capture_done ? 0u : 1u,
           capture_elapsed_us,
           (unsigned)capture.changed_words,
           (unsigned)capture.transition_words,
           (unsigned)sample_count,
           finite_fill_msps,
           capture_done
               ? "FINITE_FILL_ESTIMATE_NOT_CALIBRATED"
               : "INVALID_TIMEOUT");
    fflush(stdout);

    /* Never publish stale/uninitialized HP SRAM as IQ. A timeout is a valid
     * hardware diagnostic result, not a successful capture. */
    if (!capture_done) return ESP_ERR_TIMEOUT;

    volatile const uint32_t *const words =
        (volatile const uint32_t *)(uintptr_t)C5VRX_ADC_DUMP_BASE_ADDR;

    /* Phase8 performs its own DC-mean pass immediately afterward. Avoid a
     * redundant full-buffer statistics pass on that latency-sensitive path. */
    if (!print_raw_words && !summarize) return ESP_OK;

    int64_t sum_i = 0;
    int64_t sum_q = 0;
    uint64_t sum_power = 0;
    int16_t min_i = INT16_MAX;
    int16_t max_i = INT16_MIN;
    int16_t min_q = INT16_MAX;
    int16_t max_q = INT16_MIN;
    const bool binary_transport =
        print_raw_words && c5vrx_usb_preview_running();

    if (print_raw_words) {
        if (binary_transport) {
            printf("C5VRX_IQ_BINARY_BEGIN samples=%u chunk_words=%u packet_type=%u\n",
                   (unsigned)sample_count,
                   C5VRX_USB_IQ_CHUNK_WORDS,
                   C5VRX_USB_PACKET_IQ_U32_CHUNK);
        } else {
            printf("C5VRX_IQ_BEGIN samples=%u base=0x%08" PRIx32 "\n",
                   (unsigned)sample_count, (uint32_t)C5VRX_ADC_DUMP_BASE_ADDR);
        }
        fflush(stdout);
    }

    for (size_t i = 0; i < sample_count; ++i) {
        const uint32_t raw = words[i];
        const c5vrx_iq10_sample_t s = c5vrx_adc_decode_word(raw);

        sum_i += s.i;
        sum_q += s.q;
        sum_power += (uint64_t)((int32_t)s.i * s.i) + (uint64_t)((int32_t)s.q * s.q);
        if (s.i < min_i) min_i = s.i;
        if (s.i > max_i) max_i = s.i;
        if (s.q < min_q) min_q = s.q;
        if (s.q > max_q) max_q = s.q;

        if (print_raw_words && !binary_transport) {
            printf("IQ:%08" PRIx32 "\n", raw);
        }
    }

    if (binary_transport) {
        const unsigned failed_chunks =
            write_binary_iq_chunks(
                words, sample_count, s_last_capture_completed_us);
        printf("C5VRX_IQ_BINARY_END samples=%u chunks=%u write_failures=%u\n",
               (unsigned)sample_count,
               (unsigned)((sample_count + C5VRX_USB_IQ_CHUNK_WORDS - 1u) /
                          C5VRX_USB_IQ_CHUNK_WORDS),
               failed_chunks);
        fflush(stdout);
    } else if (print_raw_words) {
        printf("C5VRX_IQ_END\n");
    }

    ESP_LOGI(TAG,
             "IQ summary: n=%u mean_i=%ld mean_q=%ld min/max_i=%d/%d min/max_q=%d/%d avg_power=%" PRIu64,
             (unsigned)sample_count,
             (long)(sum_i / (int64_t)sample_count),
             (long)(sum_q / (int64_t)sample_count),
             (int)min_i, (int)max_i, (int)min_q, (int)max_q,
             sum_power / sample_count);

    return ESP_OK;
}

esp_err_t c5vrx_adc_dump_capture(size_t sample_count, bool print_raw_words)
{
    return capture_internal(sample_count, print_raw_words, true);
}

esp_err_t c5vrx_adc_dump_capture_phase8(size_t sample_count)
{
    if (sample_count < 256u || sample_count > C5VRX_ADC_DUMP_MAX_SAMPLES)
        return ESP_ERR_INVALID_ARG;

    esp_err_t err = capture_internal(sample_count, false, false);
    if (err != ESP_OK) return err;

    initialize_phase8_lut();

    volatile const uint32_t *const words =
        (volatile const uint32_t *)(uintptr_t)C5VRX_ADC_DUMP_BASE_ADDR;
    int64_t sum_i = 0;
    int64_t sum_q = 0;
    for (size_t i = 0; i < sample_count; ++i) {
        const c5vrx_iq10_sample_t sample = c5vrx_adc_decode_word(words[i]);
        sum_i += sample.i;
        sum_q += sample.q;
    }
    const int32_t mean_i = (int32_t)(sum_i / (int64_t)sample_count);
    const int32_t mean_q = (int32_t)(sum_q / (int64_t)sample_count);

    printf("C5VRX_PHASE8_BINARY_BEGIN samples=%u chunk_samples=%u packet_type=%u dc_i=%ld dc_q=%ld capture_timestamp_us=%llu\n",
           (unsigned)sample_count, C5VRX_USB_PHASE8_CHUNK_SAMPLES,
           C5VRX_USB_PACKET_PHASE8_CHUNK, (long)mean_i, (long)mean_q,
           (unsigned long long)s_last_capture_completed_us);
    fflush(stdout);
    const uint32_t capture_id = s_iq_capture_id++;
    int64_t encode_us = 0;
    int64_t transport_us = 0;
    unsigned failed_chunks = 0;
    for (size_t offset = 0; offset < sample_count;
         offset += C5VRX_USB_PHASE8_CHUNK_SAMPLES) {
        size_t chunk_samples = sample_count - offset;
        if (chunk_samples > C5VRX_USB_PHASE8_CHUNK_SAMPLES)
            chunk_samples = C5VRX_USB_PHASE8_CHUNK_SAMPLES;

        const int64_t encode_begin_us = esp_timer_get_time();
        for (size_t i = 0; i < chunk_samples; ++i) {
            const c5vrx_iq10_sample_t sample =
                c5vrx_adc_decode_word(words[offset + i]);
            const int16_t centered_i =
                clamp_iq10((int32_t)sample.i - mean_i);
            const int16_t centered_q =
                clamp_iq10((int32_t)sample.q - mean_q);
            const unsigned i5 =
                (((uint16_t)centered_i & 0x3ffu) >> 5) & 0x1fu;
            const unsigned q5 =
                (((uint16_t)centered_q & 0x3ffu) >> 5) & 0x1fu;
            s_phase8_chunk[i] = s_phase8_lut[(i5 << 5) | q5];
        }
        encode_us += esp_timer_get_time() - encode_begin_us;

        const int64_t transport_begin_us = esp_timer_get_time();
        if (write_binary_phase8_chunk(
                s_phase8_chunk, sample_count, offset, chunk_samples,
                capture_id, s_last_capture_completed_us) != ESP_OK)
            ++failed_chunks;
        transport_us += esp_timer_get_time() - transport_begin_us;
    }
    const unsigned chunks = (unsigned)(
        (sample_count + C5VRX_USB_PHASE8_CHUNK_SAMPLES - 1u) /
        C5VRX_USB_PHASE8_CHUNK_SAMPLES);
    printf("C5VRX_PHASE8_BINARY_END samples=%u chunks=%u write_failures=%u encode_us=%lld transport_us=%lld bytes_per_sample=1\n",
           (unsigned)sample_count, chunks, failed_chunks,
           (long long)encode_us, (long long)transport_us);
    fflush(stdout);
    return failed_chunks ? ESP_FAIL : ESP_OK;
}

typedef struct {
    volatile bool started;
    volatile bool finished;
    unsigned argument;
    unsigned mode;
    int64_t begin_us;
    int64_t end_us;
} vendor_probe_worker_t;

static vendor_probe_worker_t s_vendor_probe;

static void vendor_probe_task(void *arg)
{
    vendor_probe_worker_t *worker = arg;
    worker->begin_us = esp_timer_get_time();
    worker->started = true;
    adctrig((int32_t)C5VRX_ADC_DUMP_MAX_SAMPLES - 1,
            (int32_t)worker->mode, 1, (int32_t)worker->argument,
            0, 0, 0, 0, 0);
    worker->end_us = esp_timer_get_time();
    worker->finished = true;
    vTaskDelete(NULL);
}

static esp_err_t run_vendor_probe(unsigned argument, unsigned mode,
                                  c5vrx_adc_rate_probe_result_t *out)
{
    memset(out, 0, sizeof(*out));
    out->argument = argument;
    out->field = argument >> 1;
    out->pointer_in_range = true;
    memset(&s_vendor_probe, 0, sizeof(s_vendor_probe));
    s_vendor_probe.argument = argument;
    s_vendor_probe.mode = mode;
    set_dump_mode(0);
    if (xTaskCreate(vendor_probe_task, "c5vrx_rate_probe", 3072,
                    &s_vendor_probe, 4, NULL) != pdPASS)
        return ESP_ERR_NO_MEM;
    const int64_t deadline = esp_timer_get_time() + 1500000;
    while (!s_vendor_probe.started && esp_timer_get_time() < deadline) vTaskDelay(1);
    uint32_t previous = UINT32_MAX;
    uint32_t prior_word = 0;
    while (!s_vendor_probe.finished && esp_timer_get_time() < deadline) {
        const uint32_t value = *(volatile const uint32_t *)(uintptr_t)C5VRX_FE_DUMP_POINTER_REG;
        const uint32_t pointer = value & 0xffffu;
        if (pointer >= C5VRX_ADC_DUMP_MAX_SAMPLES) out->pointer_in_range = false;
        if (previous != UINT32_MAX && pointer != previous) {
            if (pointer < previous) ++out->wraps_lower_bound;
            out->pointer_delta_lower_bound +=
                (pointer - previous) & (C5VRX_ADC_DUMP_MAX_SAMPLES - 1u);
        }
        const uint32_t safe = (pointer - 128u) & (C5VRX_ADC_DUMP_MAX_SAMPLES - 1u);
        const uint32_t word = *(volatile const uint32_t *)(uintptr_t)
            (C5VRX_ADC_DUMP_BASE_ADDR + safe * 4u);
        if (out->observations && word != prior_word) ++out->content_changes;
        prior_word = word;
        previous = pointer;
        ++out->observations;
        vTaskDelay(1);
    }
    while (!s_vendor_probe.finished && esp_timer_get_time() < deadline + 500000) vTaskDelay(1);
    if (!s_vendor_probe.finished) return ESP_ERR_TIMEOUT;
    out->duration_us = (uint64_t)(s_vendor_probe.end_us - s_vendor_probe.begin_us);
    out->final_pointer_mode =
        *(volatile const uint32_t *)(uintptr_t)C5VRX_FE_DUMP_POINTER_REG;
    out->rate_field_survived_configuration =
        ((out->final_pointer_mode >> 21) & 7u) == out->field;
    if (out->duration_us)
        out->estimated_words_per_sec_lower_bound =
            out->pointer_delta_lower_bound * 1000000ull / out->duration_us;
    volatile const uint32_t *words =
        (volatile const uint32_t *)(uintptr_t)C5VRX_ADC_DUMP_BASE_ADDR;
    uint64_t power = 0;
    for (unsigned i = 0; i < 256; ++i) {
        c5vrx_iq10_sample_t s = c5vrx_adc_decode_word(words[i]);
        power += (uint64_t)((int32_t)s.i * s.i) + (uint64_t)((int32_t)s.q * s.q);
    }
    out->iq_power = power / 256u;
    return ESP_OK;
}

esp_err_t c5vrx_adc_rate_probe_all(void)
{
    for (unsigned field = 0; field < 8; ++field) {
        c5vrx_adc_rate_probe_result_t r;
        const esp_err_t err = run_vendor_probe(field * 2u, 7u, &r);
        printf("C5VRX_RATE_PROBE field=%u arg=%u duration_us=%llu pointer_delta_lower_bound=%u wraps_lower_bound=%u wrap_exact=0 estimated_words_per_sec_lower_bound=%llu estimated_complex_samples_per_sec_lower_bound=%llu content_changes=%u iq_power=%llu pointer_in_range=%u final_9008=%08x requested_field_survived=%u register_delta_ok=UNKNOWN classification=%s code=%d\n",
               field, field * 2u, (unsigned long long)r.duration_us,
               (unsigned)r.pointer_delta_lower_bound,
               (unsigned)r.wraps_lower_bound,
               (unsigned long long)r.estimated_words_per_sec_lower_bound,
               (unsigned long long)r.estimated_words_per_sec_lower_bound,
               (unsigned)r.content_changes, (unsigned long long)r.iq_power,
               r.pointer_in_range ? 1u : 0u, (unsigned)r.final_pointer_mode,
               r.rate_field_survived_configuration ? 1u : 0u,
               r.rate_field_survived_configuration
                   ? "MEASUREMENT_REQUIRED" : "OVERWRITTEN_BEFORE_ENABLE",
               (int)err);
        fflush(stdout);
        if (err != ESP_OK) return err;
    }
    return ESP_OK;
}

static uint32_t entropy_proxy(volatile const uint32_t *words, unsigned count)
{
    uint32_t changes = 0;
    for (unsigned i = 1; i < count; ++i) changes += words[i] != words[i - 1];
    return changes;
}

esp_err_t c5vrx_adc_dump_mode_probe(void)
{
    static const c5vrx_rf_dump_mode_t modes[] = {
        C5VRX_RF_DUMP_MODE_ORDINARY_RX, C5VRX_RF_DUMP_MODE_11,
    };
    if (!c5vrx_rf_dump_producer_available()) return ESP_ERR_NOT_SUPPORTED;
    volatile const uint32_t *words =
        (volatile const uint32_t *)(uintptr_t)C5VRX_ADC_DUMP_BASE_ADDR;
    for (unsigned n = 0; n < sizeof(modes) / sizeof(modes[0]); ++n) {
        esp_err_t err = c5vrx_rf_dump_configure(16384u, modes[n]);
        c5vrx_rf_dump_status_t before = {0}, during = {0}, after = {0};
        if (err == ESP_OK) err = c5vrx_rf_dump_get_status(&before);
        if (err == ESP_OK) err = c5vrx_rf_dump_start();
        const int64_t start = esp_timer_get_time();
        if (err == ESP_OK) {
            while (esp_timer_get_time() - start < 5000) {
                (void)c5vrx_rf_dump_get_status(&during);
            }
            err = c5vrx_rf_dump_stop();
        }
        if (err == ESP_OK) err = c5vrx_rf_dump_get_status(&after);
        uint64_t power = 0;
        for (unsigned i = 0; i < 256; ++i) {
            c5vrx_iq10_sample_t s = c5vrx_adc_decode_word(words[i]);
            power += (uint64_t)((int32_t)s.i * s.i) + (uint64_t)((int32_t)s.q * s.q);
        }
        printf("C5VRX_DUMP_MODE_PROBE mode=%u before_9008=%08x during_9008=%08x format=%08x fe_path=%08x fe_aux=%08x pointer=%u entropy_proxy=%u iq10_power=%llu iq10_format=UNPROVEN_AFTER_MODE_CHANGE stopped=%u code=%d\n",
               (unsigned)modes[n], (unsigned)before.pointer_mode,
               (unsigned)during.pointer_mode, (unsigned)during.format,
               (unsigned)during.fe_path, (unsigned)during.fe_aux,
               (unsigned)during.pointer,
               (unsigned)entropy_proxy(words, 256),
               (unsigned long long)(power / 256u),
               after.enabled ? 0u : 1u, (int)err);
        fflush(stdout);
        if (err != ESP_OK) return err;
    }
    printf("C5VRX_DUMP_MODE_PROBE mode=12 classification=SKIPPED_VENDOR_BLE_RX_START_REQUIRED code=%d\n",
           (int)ESP_ERR_NOT_SUPPORTED);
    fflush(stdout);
    return ESP_OK;
}

esp_err_t c5vrx_adc_phase_probe(unsigned field)
{
    if (field > 7u) return ESP_ERR_INVALID_ARG;
    c5vrx_adc_rate_probe_result_t r;
    esp_err_t err = run_vendor_probe(field * 2u, 7u, &r);
    if (err != ESP_OK) return err;
    volatile const uint32_t *words =
        (volatile const uint32_t *)(uintptr_t)C5VRX_ADC_DUMP_BASE_ADDR;
    /* Ring wrap is the fixed 16383 -> 0 boundary, not the final write pointer.
     * The vendor arm has completed before these reads, so the pair is stable. */
    const unsigned boundary = 0;
    float increments[9];
    for (int k = -4; k < 5; ++k) {
        const unsigned a = (boundary + k - 1) & 0x3fffu;
        const unsigned b = (boundary + k) & 0x3fffu;
        const c5vrx_iq10_sample_t x = c5vrx_adc_decode_word(words[a]);
        const c5vrx_iq10_sample_t y = c5vrx_adc_decode_word(words[b]);
        const float cross = (float)x.i * y.q - (float)x.q * y.i;
        const float dot = (float)x.i * y.i + (float)x.q * y.q;
        increments[k + 4] = atan2f(cross, dot);
    }
    float local = 0.0f;
    for (unsigned i = 0; i < 9; ++i) if (i != 4) local += increments[i];
    local /= 8.0f;
    printf("C5VRX_PHASE_PROBE field=%u arg=%u boundary=%u boundary_phase_jump=%g local_average_phase_increment=%g residual=%g wraps_lower_bound=%u wrap_observed=%u coherent_source_required=1 rate_field_survived=%u classification=%s\n",
           field, field * 2u, boundary, (double)increments[4], (double)local,
           (double)(increments[4] - local),
           (unsigned)r.wraps_lower_bound,
           r.wraps_lower_bound ? 1u : 0u,
           r.rate_field_survived_configuration ? 1u : 0u,
           r.rate_field_survived_configuration ? "MEASURED_NOT_INTERPRETED" :
                                                 "RATE_FIELD_OVERWRITTEN");
    fflush(stdout);
    return ESP_OK;
}

esp_err_t c5vrx_adc_dump_capture_chained(size_t block_count,
                                         size_t sample_count,
                                         c5vrx_adc_chain_stats_t *stats)
{
    if (block_count < 2 || block_count > 1024 ||
        sample_count == 0 || sample_count > C5VRX_ADC_DUMP_MAX_SAMPLES) {
        return ESP_ERR_INVALID_ARG;
    }

    c5vrx_adc_chain_stats_t local = {
        .blocks_completed = 0,
        .samples_per_block = sample_count,
        .total_samples = 0,
        .repeated_block_hashes = 0,
        .boundary_jump_power_sum = 0,
    };

    volatile const uint32_t *const words =
        (volatile const uint32_t *)(uintptr_t)C5VRX_ADC_DUMP_BASE_ADDR;

    uint32_t previous_hash = 0;
    c5vrx_iq10_sample_t previous_last = {0};
    bool have_previous = false;

    set_dump_mode(0);
    ESP_LOGW(TAG,
             "EXPERIMENTAL chained finite capture: blocks=%u samples/block=%u; this measures re-trigger continuity, not true streaming",
             (unsigned)block_count,
             (unsigned)sample_count);

    for (size_t block = 0; block < block_count; ++block) {
        const capture_result_t capture = trigger_dump(sample_count);
        if (!capture.complete) return ESP_ERR_TIMEOUT;

        const uint32_t hash = fnv1a_dump_hash(words, sample_count);
        const c5vrx_iq10_sample_t first = c5vrx_adc_decode_word(words[0]);
        const c5vrx_iq10_sample_t last = c5vrx_adc_decode_word(words[sample_count - 1]);

        if (have_previous) {
            if (hash == previous_hash) {
                ++local.repeated_block_hashes;
            }
            const int32_t di = (int32_t)first.i - previous_last.i;
            const int32_t dq = (int32_t)first.q - previous_last.q;
            local.boundary_jump_power_sum +=
                (uint64_t)(di * di) + (uint64_t)(dq * dq);
        }

        ESP_LOGI(TAG,
                 "chain block=%u hash=%08" PRIx32 " first=%d,%d last=%d,%d",
                 (unsigned)block,
                 hash,
                 (int)first.i,
                 (int)first.q,
                 (int)last.i,
                 (int)last.q);

        previous_hash = hash;
        previous_last = last;
        have_previous = true;
        ++local.blocks_completed;
        local.total_samples += sample_count;
    }

    const uint64_t boundaries = local.blocks_completed > 1
        ? (uint64_t)local.blocks_completed - 1u
        : 0u;
    ESP_LOGW(TAG,
             "chain summary: blocks=%u total=%" PRIu64 " repeated_hashes=%" PRIu32 " avg_boundary_jump_power=%" PRIu64,
             (unsigned)local.blocks_completed,
             local.total_samples,
             local.repeated_block_hashes,
             boundaries ? local.boundary_jump_power_sum / boundaries : 0u);

    if (stats) {
        *stats = local;
    }
    return ESP_OK;
}

typedef struct {
    volatile bool started;
    volatile bool finished;
    int64_t begin_us;
    int64_t end_us;
} ring_probe_worker_t;

static ring_probe_worker_t s_ring_probe_worker;
static volatile bool s_ring_probe_busy;

static void ring_probe_adctrig_task(void *arg)
{
    ring_probe_worker_t *worker = arg;
    worker->begin_us = esp_timer_get_time();
    worker->started = true;
    /* RX-error-1 is a known historical trigger. The vendor function owns all
     * setup, its one-second timeout, shutdown and RF-state restoration. */
    adctrig((int32_t)C5VRX_ADC_DUMP_MAX_SAMPLES - 1,
            7, 1, 1, 0, 0, 0, 0, 0);
    worker->end_us = esp_timer_get_time();
    worker->finished = true;
    vTaskDelete(NULL);
}

esp_err_t c5vrx_adc_dump_ring_probe(c5vrx_adc_ring_probe_stats_t *stats)
{
    if (!stats) return ESP_ERR_INVALID_ARG;
    if (s_ring_probe_busy) return ESP_ERR_INVALID_STATE;
    s_ring_probe_busy = true;
    memset(stats, 0, sizeof(*stats));
    stats->minimum_pointer = UINT32_MAX;

    set_dump_mode(0);
    ring_probe_worker_t *const worker = &s_ring_probe_worker;
    memset(worker, 0, sizeof(*worker));
    if (xTaskCreate(ring_probe_adctrig_task, "c5vrx_ring_probe", 3072,
                    worker, 4, NULL) != pdPASS) {
        s_ring_probe_busy = false;
        return ESP_ERR_NO_MEM;
    }

    const int64_t wait_deadline = esp_timer_get_time() + 1500000;
    while (!worker->started && esp_timer_get_time() < wait_deadline)
        vTaskDelay(1);
    if (!worker->started) {
        s_ring_probe_busy = false;
        return ESP_ERR_TIMEOUT;
    }

    uint32_t previous_pointer = UINT32_MAX;
    uint32_t previous_word = 0;
    bool have_word = false;
    while (!worker->finished && esp_timer_get_time() < wait_deadline) {
        const uint32_t pointer =
            (*(volatile const uint32_t *)(uintptr_t)C5VRX_FE_DUMP_POINTER_REG) & 0xffffu;
        ++stats->observations;
        if (previous_pointer != UINT32_MAX && pointer != previous_pointer)
            ++stats->pointer_changes;
        if (pointer < stats->minimum_pointer) stats->minimum_pointer = pointer;
        if (pointer > stats->maximum_pointer) stats->maximum_pointer = pointer;

        /* Historical tools define this pointer in words. Stay 256 words behind
         * the producer so the sampled location is not the current write word. */
        const uint32_t safe_index = (pointer - 256u) &
            (C5VRX_ADC_DUMP_MAX_SAMPLES - 1u);
        const uint32_t word = *(volatile const uint32_t *)(uintptr_t)
            (C5VRX_ADC_DUMP_BASE_ADDR + safe_index * sizeof(uint32_t));
        if (have_word && word != previous_word) ++stats->content_changes;
        previous_word = word;
        have_word = true;
        previous_pointer = pointer;
        vTaskDelay(1);
    }
    if (!worker->finished) {
        /* Keep the static state reserved until the vendor-owned call exits.
         * Its recovered timeout is one second, so this is only a safety net. */
        while (!worker->finished) vTaskDelay(1);
        s_ring_probe_busy = false;
        return ESP_ERR_TIMEOUT;
    }

    stats->active_time_us = (uint64_t)(worker->end_us - worker->begin_us);
    stats->final_status =
        *(volatile const uint32_t *)(uintptr_t)C5VRX_FE_DUMP_CONTROL_REG;
    stats->completion_bit_seen =
        (stats->final_status & C5VRX_FE_DUMP_DONE_BIT) != 0;
    stats->reached_vendor_timeout = stats->active_time_us >= 900000u;
    if (stats->minimum_pointer == UINT32_MAX) stats->minimum_pointer = 0;

    ESP_LOGW(TAG,
             "single-arm ring probe (NOT a stream): active_us=%llu observations=%u pointer_changes=%u range=%u..%u content_changes=%u done=%u vendor_timeout=%u status=%08" PRIx32,
             (unsigned long long)stats->active_time_us,
             (unsigned)stats->observations,
             (unsigned)stats->pointer_changes, (unsigned)stats->minimum_pointer,
             (unsigned)stats->maximum_pointer, (unsigned)stats->content_changes,
             stats->completion_bit_seen ? 1u : 0u,
             stats->reached_vendor_timeout ? 1u : 0u,
             (uint32_t)stats->final_status);
    s_ring_probe_busy = false;
    return ESP_OK;
}
