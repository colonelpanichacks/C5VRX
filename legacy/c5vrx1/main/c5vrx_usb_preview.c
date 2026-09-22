/* SPDX-License-Identifier: GPL-3.0-only */

#include "c5vrx_usb_preview.h"

#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "c5vrx_cvbs_sync.h"
#include "c5vrx_usb_transport.h"
#include "esp_heap_caps.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#define YUV411_STRIDE (C5VRX_USB_PREVIEW_WIDTH * 3u / 2u)
#define FRAME_BYTES (YUV411_STRIDE * C5VRX_USB_PREVIEW_HEIGHT)
#define USB_HEADER_BYTES 32u
#define FRAME_DESCRIPTOR_BYTES 8u
#define PACKET_STREAM_INFO 1u
#define PACKET_GRAY8_FRAME 2u
#define PACKET_YUV411_FRAME 7u
#define PIXEL_FORMAT_GRAY8 1u
#define PIXEL_FORMAT_YUV411 2u
#define FIRST_ACTIVE_FIELD_LINE 20u
#define ACTIVE_FIELD_LINES 240u
#define NOMINAL_LINE_SAMPLES 1280u
#define NOMINAL_ACTIVE_START 210u
#define NOMINAL_ACTIVE_SAMPLES 1040u
#define CONSUMER_LEASE_US 750000u
#define COMPOSITE_BLOCK_MAX 4096u

static const uint8_t s_usb_magic[8] = {0x00, 'C', '5', 'V', 'R', 'X', 0xa5, 0x5a};

typedef struct {
    uint8_t *frame[2];
    uint8_t *composite[2];
    size_t composite_count[2];
    c5vrx_cvbs_sync_tracker_t composite_timing[2];
    uint64_t composite_sequence[2];
    uint64_t composite_next_sequence;
    bool composite_ready[2];
    bool composite_in_use[2];
    unsigned fill_index;
    unsigned ready_index;
    unsigned x;
    uint16_t current_row;
    uint32_t line_phase;
    uint32_t line_period;
    uint32_t sequence;
    uint64_t dropped;
    uint64_t consumer_deadline_us;
    uint64_t worker_time_us;
    uint32_t session_epoch;
    uint64_t last_field_id;
    uint64_t expected_next_sample;
    uint32_t completed_rows[4];
    uint32_t transport_stalls;
    uint32_t stale_session_drops;
    int32_t luma_q8;
    int32_t chroma_u_q8;
    int32_t chroma_v_q8;
    int64_t burst_i;
    int64_t burst_q;
    uint32_t burst_count;
    int32_t burst_ref_i;
    int32_t burst_ref_q;
    uint32_t burst_locks;
    uint64_t burst_line_key;
    bool burst_locked;
    uint8_t y_group[4];
    int32_t u_group;
    int32_t v_group;
    unsigned group_count;
    int8_t sine[256];
    uint32_t frames_completed;
    uint32_t frames_sent;
    c5vrx_cvbs_sync_tracker_t sync;
    c5vrx_usb_preview_stats_t telemetry;
    bool current_line;
    bool current_row_valid;
    bool sending[2];
    volatile bool ready;
    volatile bool running;
    volatile bool consumer_active;
    volatile bool worker_active;
    volatile bool stream_info_pending;
    bool prepared;
    TaskHandle_t task;
    SemaphoreHandle_t session_mutex;
    portMUX_TYPE lock;
} preview_state_t;

static preview_state_t s_preview = {.lock = portMUX_INITIALIZER_UNLOCKED};

static void decode_timed_block(
    const uint8_t *cvbs, size_t samples,
    const c5vrx_cvbs_sync_tracker_t *timing);

static void put_le16(uint8_t *out, uint16_t value)
{
    out[0] = (uint8_t)value;
    out[1] = (uint8_t)(value >> 8u);
}

static uint8_t clamp_u8(int value)
{
    if (value < 0) return 0u;
    if (value > 255) return 255u;
    return (uint8_t)value;
}

static void put_le32(uint8_t *out, uint32_t value)
{
    for (unsigned i = 0; i < 4u; ++i) out[i] = (uint8_t)(value >> (8u * i));
}

static void put_le64(uint8_t *out, uint64_t value)
{
    for (unsigned i = 0; i < 8u; ++i) out[i] = (uint8_t)(value >> (8u * i));
}

/* Standard reflected CRC-32/ISO-HDLC, matching Python zlib.crc32 exactly. */
static uint32_t crc32_update(uint32_t crc, const uint8_t *data, size_t count)
{
    for (size_t i = 0; i < count; ++i) {
        crc ^= data[i];
        for (unsigned bit = 0; bit < 8u; ++bit)
            crc = (crc >> 1u) ^ (0xedb88320u & (0u - (crc & 1u)));
    }
    return crc;
}

static uint32_t crc32_parts(const uint8_t *first, size_t first_count,
                            const uint8_t *second, size_t second_count)
{
    uint32_t crc = crc32_update(UINT32_MAX, first, first_count);
    crc = crc32_update(crc, second, second_count);
    return ~crc;
}

static esp_err_t write_packet(unsigned type,
                         const uint8_t *first, size_t first_count,
                         const uint8_t *second, size_t second_count)
{
    uint8_t header[USB_HEADER_BYTES] = {0};
    memcpy(header, s_usb_magic, sizeof(s_usb_magic));
    header[8] = C5VRX_USB_PREVIEW_PROTOCOL_VERSION;
    header[9] = (uint8_t)type;
    put_le16(header + 10, USB_HEADER_BYTES);
    put_le32(header + 12, s_preview.sequence++);
    put_le32(header + 16, (uint32_t)(first_count + second_count));
    put_le64(header + 20, (uint64_t)esp_timer_get_time());
    put_le32(header + 28, crc32_parts(header, 28u, NULL, 0u));
    const uint32_t payload_crc =
        crc32_parts(first, first_count, second, second_count);
    uint8_t trailer[4];
    put_le32(trailer, payload_crc);

    const c5vrx_usb_iovec_t packet[] = {
        {.data = header, .size = sizeof(header)},
        {.data = first, .size = first_count},
        {.data = second, .size = second_count},
        {.data = trailer, .size = sizeof(trailer)},
    };
    return c5vrx_usb_writev(packet, sizeof(packet) / sizeof(packet[0]));
}

static void park_consumer(bool transport_stall)
{
    s_preview.worker_active = false;
    taskENTER_CRITICAL(&s_preview.lock);
    if (transport_stall) ++s_preview.transport_stalls;
    if (s_preview.consumer_active || s_preview.ready) {
        ++s_preview.stale_session_drops;
    }
    s_preview.consumer_active = false;
    s_preview.worker_active = false;
    s_preview.ready = false;
    s_preview.composite_ready[0] = s_preview.composite_ready[1] = false;
    s_preview.current_line = false;
    s_preview.x = 0u;
    s_preview.group_count = 0u;
    s_preview.burst_locked = false;
    taskEXIT_CRITICAL(&s_preview.lock);
}

static void make_descriptor(uint8_t descriptor[FRAME_DESCRIPTOR_BYTES],
                            uint8_t flags)
{
    put_le16(descriptor, C5VRX_USB_PREVIEW_WIDTH);
    put_le16(descriptor + 2, C5VRX_USB_PREVIEW_HEIGHT);
    put_le16(descriptor + 4, YUV411_STRIDE);
    descriptor[6] = PIXEL_FORMAT_YUV411;
    descriptor[7] = flags;
}

static void preview_task(void *arg)
{
    (void)arg;
    uint8_t descriptor[FRAME_DESCRIPTOR_BYTES];
    for (;;) {
        (void)ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        /* Serialize the whole bounded USB iteration with session reset. This
         * task is low priority and never owns RF/AV state, so waiting here
         * cannot delay the production AV path. */
        xSemaphoreTake(s_preview.session_mutex, portMAX_DELAY);
        s_preview.worker_active = true;
        if (!s_preview.running) goto iteration_done;
        if (s_preview.stream_info_pending) {
            make_descriptor(descriptor, 0u);
            s_preview.stream_info_pending = false;
            if (write_packet(PACKET_STREAM_INFO, descriptor,
                             sizeof(descriptor), NULL, 0u) != ESP_OK) {
                park_consumer(true);
                goto iteration_done;
            }
            c5vrx_usb_preview_keepalive();
        }
        if (!c5vrx_usb_preview_consumer_active()) {
            park_consumer(false);
            goto iteration_done;
        }
        unsigned composite_index = 0u;
        bool have_composite = false;
        taskENTER_CRITICAL(&s_preview.lock);
        for (unsigned i = 0; i < 2u; ++i) {
            if (!s_preview.composite_ready[i]) continue;
            if (have_composite &&
                s_preview.composite_sequence[i] >=
                    s_preview.composite_sequence[composite_index]) continue;
            composite_index = i;
            have_composite = true;
        }
        if (have_composite) {
            const unsigned i = composite_index;
            s_preview.composite_ready[i] = false;
            s_preview.composite_in_use[i] = true;
        }
        taskEXIT_CRITICAL(&s_preview.lock);
        if (have_composite) {
            const int64_t decode_begin = esp_timer_get_time();
            decode_timed_block(
                s_preview.composite[composite_index],
                s_preview.composite_count[composite_index],
                &s_preview.composite_timing[composite_index]);
            taskENTER_CRITICAL(&s_preview.lock);
            s_preview.worker_time_us +=
                (uint64_t)(esp_timer_get_time() - decode_begin);
            s_preview.composite_in_use[composite_index] = false;
            taskEXIT_CRITICAL(&s_preview.lock);
        }
        if (!s_preview.running || !c5vrx_usb_preview_consumer_active()) {
            park_consumer(false);
            goto iteration_done;
        }
        unsigned index = 0;
        bool have_frame = false;
        taskENTER_CRITICAL(&s_preview.lock);
        if (s_preview.ready) {
            index = s_preview.ready_index;
            s_preview.ready = false;
            s_preview.sending[index] = true;
            have_frame = true;
        }
        taskEXIT_CRITICAL(&s_preview.lock);
        if (!have_frame) goto iteration_done;

        make_descriptor(descriptor, (uint8_t)(1u |
            (s_preview.burst_locked ? 2u : 0u)));
        const int64_t work_begin = esp_timer_get_time();
        const esp_err_t write_err = write_packet(PACKET_YUV411_FRAME,
            descriptor, sizeof(descriptor), s_preview.frame[index], FRAME_BYTES);
        taskENTER_CRITICAL(&s_preview.lock);
        s_preview.worker_time_us +=
            (uint64_t)(esp_timer_get_time() - work_begin);
        if (write_err == ESP_OK) ++s_preview.frames_sent;
        s_preview.telemetry.frames_sent = s_preview.frames_sent;
        s_preview.sending[index] = false;
        taskEXIT_CRITICAL(&s_preview.lock);
        if (write_err != ESP_OK) park_consumer(true);
        else c5vrx_usb_preview_keepalive();
iteration_done:
        s_preview.worker_active = false;
        xSemaphoreGive(s_preview.session_mutex);
    }

    /*
     * Do not send STREAM_END here.  A stopped host may no longer drain USB,
     * making the task block in the direct writer while stop() waits for it.
     * The ASCII STOP acknowledgement is the authoritative lifecycle marker.
     */
}

static void publish_frame(void)
{
    taskENTER_CRITICAL(&s_preview.lock);
    ++s_preview.frames_completed;
    s_preview.telemetry.frames_completed = s_preview.frames_completed;
    const unsigned next = s_preview.fill_index ^ 1u;
    if (s_preview.ready || s_preview.sending[next]) {
        ++s_preview.dropped;
        s_preview.telemetry.frames_dropped =
            s_preview.dropped > UINT32_MAX ? UINT32_MAX : (uint32_t)s_preview.dropped;
    } else {
        s_preview.ready_index = s_preview.fill_index;
        s_preview.ready = true;
        s_preview.fill_index = next;
        if (s_preview.task) xTaskNotifyGive(s_preview.task);
    }
    taskEXIT_CRITICAL(&s_preview.lock);
}

esp_err_t c5vrx_usb_preview_prepare(void)
{
    if (s_preview.prepared) return ESP_OK;
    s_preview.session_mutex = xSemaphoreCreateMutex();
    if (!s_preview.session_mutex) return ESP_ERR_NO_MEM;
    for (unsigned i = 0; i < 2u; ++i) {
        s_preview.frame[i] = heap_caps_malloc(FRAME_BYTES, MALLOC_CAP_INTERNAL);
        s_preview.composite[i] = heap_caps_malloc(
            COMPOSITE_BLOCK_MAX, MALLOC_CAP_INTERNAL);
        if (!s_preview.frame[i] || !s_preview.composite[i]) {
            for (unsigned n = 0; n <= i; ++n) {
                free(s_preview.frame[n]);
                free(s_preview.composite[n]);
                s_preview.frame[n] = s_preview.composite[n] = NULL;
            }
            vSemaphoreDelete(s_preview.session_mutex);
            s_preview.session_mutex = NULL;
            return ESP_ERR_NO_MEM;
        }
        memset(s_preview.frame[i], 0, FRAME_BYTES);
    }
    for (unsigned i = 0; i < 256u; ++i) {
        s_preview.sine[i] = (int8_t)lrintf(
            127.0f * sinf((float)i * 6.2831853071795864769f / 256.0f));
    }
    if (xTaskCreate(preview_task, "c5vrx_preview", 3072, NULL, 4,
                    &s_preview.task) != pdPASS) {
        free(s_preview.frame[0]);
        free(s_preview.frame[1]);
        free(s_preview.composite[0]);
        free(s_preview.composite[1]);
        s_preview.frame[0] = s_preview.frame[1] = NULL;
        s_preview.composite[0] = s_preview.composite[1] = NULL;
        vSemaphoreDelete(s_preview.session_mutex);
        s_preview.session_mutex = NULL;
        return ESP_ERR_NO_MEM;
    }
    s_preview.prepared = true;
    return ESP_OK;
}

esp_err_t c5vrx_usb_preview_start(void)
{
    /* START is deliberately idempotent for reconnecting Windows hosts. */
    if (s_preview.running && c5vrx_usb_preview_consumer_active()) {
        c5vrx_usb_preview_keepalive();
        return ESP_OK;
    }
    esp_err_t err = c5vrx_usb_preview_prepare();
    if (err != ESP_OK) return err;
    /* A reconnect cannot reset decoder or frame state until any old decode or
     * bounded USB write has fully quiesced. This is a hard epoch boundary. */
    xSemaphoreTake(s_preview.session_mutex, portMAX_DELAY);
    c5vrx_cvbs_sync_init(&s_preview.sync);
    s_preview.current_row = C5VRX_CVBS_SYNC_NO_LINE;
    s_preview.line_period = NOMINAL_LINE_SAMPLES;
    s_preview.ready = false;
    s_preview.composite_ready[0] = s_preview.composite_ready[1] = false;
    s_preview.current_line = false;
    s_preview.group_count = 0u;
    s_preview.burst_locked = false;
    s_preview.burst_line_key = UINT64_MAX;
    s_preview.last_field_id = UINT64_MAX;
    s_preview.expected_next_sample = 0u;
    memset(s_preview.completed_rows, 0, sizeof(s_preview.completed_rows));
    s_preview.current_row_valid = false;
    s_preview.worker_active = false;
    s_preview.stream_info_pending = true;
    s_preview.running = true;
    s_preview.consumer_active = true;
    s_preview.consumer_deadline_us =
        (uint64_t)esp_timer_get_time() + CONSUMER_LEASE_US;
    ++s_preview.session_epoch;
    xSemaphoreGive(s_preview.session_mutex);
    xTaskNotifyGive(s_preview.task);
    return ESP_OK;
}

esp_err_t c5vrx_usb_preview_stop(void)
{
    /* STOP is deliberately idempotent as well. */
    if (!s_preview.running) return ESP_OK;
    taskENTER_CRITICAL(&s_preview.lock);
    s_preview.running = false;
    s_preview.consumer_active = false;
    taskEXIT_CRITICAL(&s_preview.lock);
    xSemaphoreTake(s_preview.session_mutex, portMAX_DELAY);
    park_consumer(false);
    xSemaphoreGive(s_preview.session_mutex);
    return ESP_OK;
}

bool c5vrx_usb_preview_running(void) { return s_preview.running; }

void c5vrx_usb_preview_keepalive(void)
{
    if (!s_preview.running) return;
    taskENTER_CRITICAL(&s_preview.lock);
    s_preview.consumer_deadline_us =
        (uint64_t)esp_timer_get_time() + CONSUMER_LEASE_US;
    s_preview.consumer_active = true;
    taskEXIT_CRITICAL(&s_preview.lock);
}

bool c5vrx_usb_preview_consumer_active(void)
{
    const uint64_t now = (uint64_t)esp_timer_get_time();
    bool active = false;
    taskENTER_CRITICAL(&s_preview.lock);
    if (s_preview.running && s_preview.consumer_active &&
        now <= s_preview.consumer_deadline_us) {
        active = true;
    } else if (s_preview.consumer_active) {
        /* Expiry must be O(1) on the AV caller: revoke the side cursor but do
         * not touch decoder state that the USB worker may currently own. */
        ++s_preview.stale_session_drops;
        s_preview.consumer_active = false;
        s_preview.ready = false;
        s_preview.composite_ready[0] = s_preview.composite_ready[1] = false;
    }
    taskEXIT_CRITICAL(&s_preview.lock);
    return active;
}

void c5vrx_usb_preview_get_stats(c5vrx_usb_preview_stats_t *stats)
{
    if (!stats) return;
    taskENTER_CRITICAL(&s_preview.lock);
    *stats = s_preview.telemetry;
    taskEXIT_CRITICAL(&s_preview.lock);
}

static void update_telemetry(void)
{
    c5vrx_usb_preview_stats_t next = {
        .samples_ingested = s_preview.sync.samples_seen > UINT32_MAX ?
            UINT32_MAX : (uint32_t)s_preview.sync.samples_seen,
        .horizontal_syncs = s_preview.sync.horizontal_events,
        .vertical_syncs = s_preview.sync.vertical_events,
        .rejected_sync_pulses = s_preview.sync.rejected_pulses,
        .lock_acquisitions = s_preview.sync.lock_acquisitions,
        .lock_losses = s_preview.sync.lock_losses,
        .frames_completed = s_preview.frames_completed,
        .frames_sent = s_preview.frames_sent,
        .frames_dropped = s_preview.dropped > UINT32_MAX ?
            UINT32_MAX : (uint32_t)s_preview.dropped,
        .line_period_samples = s_preview.sync.line_period_samples,
        .last_hsync_width = s_preview.sync.last_hsync_width,
        .last_vsync_width = s_preview.sync.last_vsync_width,
        .sync_threshold = c5vrx_cvbs_sync_threshold(&s_preview.sync),
        .horizontal_locked = s_preview.sync.horizontal_locked,
        .vertical_locked = s_preview.sync.vertical_locked,
        .consumer_active = s_preview.consumer_active,
        .worker_active = s_preview.worker_active,
        .session_epoch = s_preview.session_epoch,
        .transport_stalls = s_preview.transport_stalls,
        .stale_session_drops = s_preview.stale_session_drops,
        .worker_time_us = s_preview.worker_time_us > UINT32_MAX ?
            UINT32_MAX : (uint32_t)s_preview.worker_time_us,
    };
    taskENTER_CRITICAL(&s_preview.lock);
    s_preview.telemetry = next;
    taskEXIT_CRITICAL(&s_preview.lock);
}

void c5vrx_usb_preview_ingest(const uint8_t *cvbs, size_t samples)
{
    /* Diagnostics-only compatibility path. Production LIVE calls
     * ingest_timed() with the upstream canonical timing authority. */
    if (!cvbs || !c5vrx_usb_preview_consumer_active()) return;
    s_preview.worker_active = true;
    for (size_t n = 0; n < samples; ++n) {
        const uint8_t value = cvbs[n] & 0x3fu;
        c5vrx_cvbs_sync_event_t event;
        if (c5vrx_cvbs_sync_consume(&s_preview.sync, value, &event)) {
            if (event.vertical) {
                s_preview.current_line = false;
                s_preview.current_row = C5VRX_CVBS_SYNC_NO_LINE;
                s_preview.x = 0u;
            } else if (event.horizontal) {
                s_preview.current_line = false;
                s_preview.x = 0u;
                s_preview.line_phase =
                    (uint32_t)(event.sample_index - event.sync_start);
                s_preview.line_period = event.line_period_samples;
                if (event.locked &&
                    event.field_line >= FIRST_ACTIVE_FIELD_LINE &&
                    event.field_line < FIRST_ACTIVE_FIELD_LINE + ACTIVE_FIELD_LINES) {
                    const unsigned active_line =
                        event.field_line - FIRST_ACTIVE_FIELD_LINE;
                    if ((active_line & 1u) == 0u) {
                        s_preview.current_row = (uint16_t)(active_line / 2u);
                        s_preview.current_line = true;
                    }
                }
            }
        }

        if (s_preview.current_line &&
            s_preview.current_row < C5VRX_USB_PREVIEW_HEIGHT &&
            s_preview.x < C5VRX_USB_PREVIEW_WIDTH) {
            const uint32_t active_start =
                s_preview.line_period * NOMINAL_ACTIVE_START /
                NOMINAL_LINE_SAMPLES;
            const uint32_t active_samples =
                s_preview.line_period * NOMINAL_ACTIVE_SAMPLES /
                NOMINAL_LINE_SAMPLES;
            const uint32_t target = active_start +
                s_preview.x * active_samples / C5VRX_USB_PREVIEW_WIDTH;
            if (s_preview.line_phase >= target) {
                s_preview.frame[s_preview.fill_index]
                    [s_preview.current_row * C5VRX_USB_PREVIEW_WIDTH +
                     s_preview.x] = (uint8_t)((value * 255u + 31u) / 63u);
                ++s_preview.x;
                if (s_preview.x == C5VRX_USB_PREVIEW_WIDTH) {
                    s_preview.current_line = false;
                    if (s_preview.current_row == C5VRX_USB_PREVIEW_HEIGHT - 1u)
                        publish_frame();
                }
            }
        }
        ++s_preview.line_phase;
    }
    update_telemetry();
    s_preview.worker_active = false;
}

static void decode_timed_block(
    const uint8_t *cvbs, size_t samples,
    const c5vrx_cvbs_sync_tracker_t *timing)
{
    s_preview.worker_active = true;
    if (!timing->horizontal_locked || !timing->vertical_locked ||
        !timing->line_period_samples || !timing->last_hsync_start) {
        s_preview.worker_active = false;
        return;
    }
    const uint64_t block_start = timing->samples_seen >= samples ?
        timing->samples_seen - samples : 0u;
    const bool gap = s_preview.expected_next_sample &&
        block_start != s_preview.expected_next_sample;
    s_preview.expected_next_sample = timing->samples_seen;
    if (timing->field_id != s_preview.last_field_id || gap) {
        s_preview.last_field_id = timing->field_id;
        s_preview.current_row = C5VRX_CVBS_SYNC_NO_LINE;
        s_preview.current_line = false;
        s_preview.current_row_valid = false;
        s_preview.x = 0u;
        s_preview.group_count = 0u;
        memset(s_preview.completed_rows, 0, sizeof(s_preview.completed_rows));
    }

    const uint32_t period = timing->line_period_samples;
    uint16_t current_line = timing->field_line ?
        (uint16_t)(timing->field_line - 1u) : 0u;
    uint64_t line_start = timing->last_hsync_start;
    /* The canonical snapshot names the newest H-sync in this block. Walk its
     * stable period backwards so samples preceding that sync remain attached
     * to the prior canonical line instead of being discarded. A block spans
     * only a few lines; field-boundary samples are vertical blanking. */
    while (block_start < line_start && current_line > 0u) {
        line_start = line_start >= period ? line_start - period : 0u;
        --current_line;
    }
    const uint32_t active_start =
        period * NOMINAL_ACTIVE_START / NOMINAL_LINE_SAMPLES;
    const uint32_t active_samples =
        period * NOMINAL_ACTIVE_SAMPLES / NOMINAL_LINE_SAMPLES;
    const uint32_t burst_start = period * 105u / NOMINAL_LINE_SAMPLES;
    const uint32_t burst_end = period * 170u / NOMINAL_LINE_SAMPLES;
    const uint32_t subcarrier_hz =
        timing->standard == C5VRX_VIDEO_STANDARD_NTSC ? 3579545u : 4433619u;
    const uint64_t nco_step = timing->sample_rate_hz ?
        ((uint64_t)subcarrier_hz << 32u) / timing->sample_rate_hz : 0u;
    for (size_t n = 0; n < samples; ++n) {
        const uint64_t absolute = block_start + n;
        if (absolute < line_start) continue;
        while (absolute - line_start >= period) {
            line_start += period;
            ++current_line;
        }
        const uint32_t phase = (uint32_t)(absolute - line_start);
        const bool selected_line =
            current_line >= FIRST_ACTIVE_FIELD_LINE &&
            current_line < FIRST_ACTIVE_FIELD_LINE + ACTIVE_FIELD_LINES &&
            !((current_line - FIRST_ACTIVE_FIELD_LINE) & 1u);
        if (!selected_line) {
            s_preview.current_line = false;
            continue;
        }
        const uint16_t row =
            (uint16_t)((current_line - FIRST_ACTIVE_FIELD_LINE) / 2u);
        if (s_preview.current_row != row) {
            s_preview.current_row = row;
            s_preview.x = 0u;
            s_preview.group_count = 0u;
            s_preview.u_group = s_preview.v_group = 0;
            /* A row is publishable only if this reducer observed it from
             * before burst/active video, not after a mid-line start or gap. */
            s_preview.current_row_valid = phase <= burst_start;
        }
        s_preview.current_line = s_preview.x < C5VRX_USB_PREVIEW_WIDTH;
        const uint64_t line_key = (timing->field_id << 16u) | current_line;
        if (s_preview.burst_line_key != line_key) {
            s_preview.burst_line_key = line_key;
            s_preview.burst_i = s_preview.burst_q = 0;
            s_preview.burst_count = 0u;
            s_preview.burst_locked = false;
        }
        const int32_t composite_q8 = (int32_t)(cvbs[n] & 0x3fu) << 8u;
        s_preview.luma_q8 += (composite_q8 - s_preview.luma_q8) >> 1u;
        const int32_t chroma = composite_q8 - s_preview.luma_q8;
        const uint8_t nco = (uint8_t)((absolute * nco_step) >> 24u);
        const int32_t cosine = s_preview.sine[(uint8_t)(nco + 64u)];
        const int32_t sine = s_preview.sine[nco];
        if (phase >= burst_start && phase < burst_end) {
            s_preview.burst_i += (int64_t)chroma * cosine;
            s_preview.burst_q += (int64_t)chroma * sine;
            ++s_preview.burst_count;
            continue;
        }
        if (!s_preview.burst_locked && phase >= burst_end &&
            s_preview.burst_count >= 8u) {
            s_preview.burst_ref_i =
                (int32_t)(s_preview.burst_i / (int32_t)s_preview.burst_count);
            s_preview.burst_ref_q =
                (int32_t)(s_preview.burst_q / (int32_t)s_preview.burst_count);
            const int32_t magnitude = abs(s_preview.burst_ref_i) +
                abs(s_preview.burst_ref_q) / 2;
            s_preview.burst_locked = magnitude > 256;
            if (s_preview.burst_locked) ++s_preview.burst_locks;
        }
        if (s_preview.burst_locked && phase >= active_start) {
            const int32_t norm = abs(s_preview.burst_ref_i) +
                abs(s_preview.burst_ref_q) / 2;
            int32_t u = (int32_t)(((int64_t)chroma *
                (cosine * s_preview.burst_ref_i +
                 sine * s_preview.burst_ref_q)) / (norm * 127ll));
            int32_t v = (int32_t)(((int64_t)chroma *
                (-cosine * s_preview.burst_ref_q +
                  sine * s_preview.burst_ref_i)) / (norm * 127ll));
            if (timing->standard == C5VRX_VIDEO_STANDARD_PAL &&
                (current_line & 1u)) v = -v;
            s_preview.chroma_u_q8 += (u - s_preview.chroma_u_q8) >> 3u;
            s_preview.chroma_v_q8 += (v - s_preview.chroma_v_q8) >> 3u;
        }
        if (!s_preview.current_line) continue;
        const uint32_t target = active_start +
            s_preview.x * active_samples / C5VRX_USB_PREVIEW_WIDTH;
        if (phase >= target) {
            const int y = 16 + ((s_preview.luma_q8 >> 8u) - 19) * 219 / 44;
            s_preview.y_group[s_preview.group_count] = clamp_u8(y);
            s_preview.u_group += s_preview.chroma_u_q8;
            s_preview.v_group += s_preview.chroma_v_q8;
            ++s_preview.group_count;
            ++s_preview.x;
            if (s_preview.group_count == 4u) {
                const size_t group = (s_preview.x - 4u) / 4u;
                uint8_t *out = s_preview.frame[s_preview.fill_index] +
                    row * YUV411_STRIDE + group * 6u;
                out[0] = clamp_u8(128 + (s_preview.u_group / 1024) * 5);
                out[1] = s_preview.y_group[0];
                out[2] = s_preview.y_group[1];
                out[3] = clamp_u8(128 + (s_preview.v_group / 1024) * 5);
                out[4] = s_preview.y_group[2];
                out[5] = s_preview.y_group[3];
                s_preview.group_count = 0u;
                s_preview.u_group = s_preview.v_group = 0;
            }
            if (s_preview.x == C5VRX_USB_PREVIEW_WIDTH) {
                s_preview.current_line = false;
                if (s_preview.current_row_valid)
                    s_preview.completed_rows[row >> 5u] |=
                        1u << (row & 31u);
                const bool complete =
                    s_preview.completed_rows[0] == UINT32_MAX &&
                    s_preview.completed_rows[1] == UINT32_MAX &&
                    s_preview.completed_rows[2] == UINT32_MAX &&
                    s_preview.completed_rows[3] == 0x00ffffffu;
                if (row == C5VRX_USB_PREVIEW_HEIGHT - 1u && complete) {
                    publish_frame();
                    memset(s_preview.completed_rows, 0,
                           sizeof(s_preview.completed_rows));
                }
            }
        }
    }

    s_preview.worker_active = false;
    taskENTER_CRITICAL(&s_preview.lock);
    s_preview.telemetry.samples_ingested = timing->samples_seen > UINT32_MAX ?
        UINT32_MAX : (uint32_t)timing->samples_seen;
    s_preview.telemetry.horizontal_syncs = timing->horizontal_events;
    s_preview.telemetry.vertical_syncs = timing->vertical_events;
    s_preview.telemetry.rejected_sync_pulses = timing->rejected_pulses;
    s_preview.telemetry.lock_acquisitions = timing->lock_acquisitions;
    s_preview.telemetry.lock_losses = timing->lock_losses;
    s_preview.telemetry.line_period_samples = timing->line_period_samples;
    s_preview.telemetry.last_hsync_width = timing->last_hsync_width;
    s_preview.telemetry.last_vsync_width = timing->last_vsync_width;
    s_preview.telemetry.sync_threshold = c5vrx_cvbs_sync_threshold(timing);
    s_preview.telemetry.horizontal_locked = timing->horizontal_locked;
    s_preview.telemetry.vertical_locked = timing->vertical_locked;
    s_preview.telemetry.consumer_active = s_preview.consumer_active;
    s_preview.telemetry.worker_active = s_preview.worker_active;
    s_preview.telemetry.burst_locks = s_preview.burst_locks;
    s_preview.telemetry.burst_locked = s_preview.burst_locked;
    taskEXIT_CRITICAL(&s_preview.lock);
}

void c5vrx_usb_preview_ingest_timed(
    const uint8_t *cvbs, size_t samples,
    const c5vrx_cvbs_sync_tracker_t *timing)
{
    if (!cvbs || !timing || !samples || samples > COMPOSITE_BLOCK_MAX ||
        !c5vrx_usb_preview_consumer_active()) return;
    int slot = -1;
    uint32_t epoch = 0u;
    taskENTER_CRITICAL(&s_preview.lock);
    epoch = s_preview.session_epoch;
    for (unsigned i = 0; i < 2u; ++i) {
        if (!s_preview.composite_ready[i] && !s_preview.composite_in_use[i]) {
            slot = (int)i;
            s_preview.composite_in_use[i] = true;
            break;
        }
    }
    if (slot < 0) {
        /* The USB cursor is best-effort and latency-bounded: replace the
         * oldest queued (never the in-use) side block with the newest legal
         * canonical block. RF/DSP/AV never wait for this cursor. */
        for (unsigned i = 0; i < 2u; ++i) {
            if (s_preview.composite_ready[i] &&
                !s_preview.composite_in_use[i] &&
                (slot < 0 || s_preview.composite_sequence[i] <
                    s_preview.composite_sequence[slot])) slot = (int)i;
        }
        if (slot >= 0) {
            s_preview.composite_ready[slot] = false;
            s_preview.composite_in_use[slot] = true;
            ++s_preview.dropped;
            s_preview.telemetry.frames_dropped =
                s_preview.dropped > UINT32_MAX ? UINT32_MAX :
                (uint32_t)s_preview.dropped;
        }
    }
    taskEXIT_CRITICAL(&s_preview.lock);
    if (slot >= 0) {
        memcpy(s_preview.composite[slot], cvbs, samples);
        taskENTER_CRITICAL(&s_preview.lock);
        if (epoch == s_preview.session_epoch && s_preview.running &&
            s_preview.consumer_active) {
            s_preview.composite_count[slot] = samples;
            s_preview.composite_timing[slot] = *timing;
            s_preview.composite_sequence[slot] =
                ++s_preview.composite_next_sequence;
            s_preview.composite_ready[slot] = true;
        } else {
            ++s_preview.stale_session_drops;
        }
        s_preview.composite_in_use[slot] = false;
        taskEXIT_CRITICAL(&s_preview.lock);
    } else {
        taskENTER_CRITICAL(&s_preview.lock);
        ++s_preview.dropped;
        s_preview.telemetry.frames_dropped = s_preview.dropped > UINT32_MAX ?
            UINT32_MAX : (uint32_t)s_preview.dropped;
        taskEXIT_CRITICAL(&s_preview.lock);
    }
    if (slot >= 0 && s_preview.task) xTaskNotifyGive(s_preview.task);
}
