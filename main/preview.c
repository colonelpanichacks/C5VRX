/**
 * preview.c - Passive USB video preview side-tap for C5VRX-3.
 *
 * The production datapath demodulates FM in the BitScrambler and the CPU
 * never sees CVBS. This module is a strictly passive observer: it hands the
 * raw Q4/I4 ring to the proven composite-video grabber (main/grab.c, ported
 * from the Konrad tembed-link branch), which FM-demodulates at the full
 * 40 MS/s sample rate, locks PAL/NTSC horizontal and vertical sync with
 * flywheels, and delivers interlaced-correct rows one at a time at the
 * native 224x168. Rows are streamed on the USB Serial/JTAG console as
 * PACKET_GRAY8_ROWS inside the v1 envelope (same magic, 32-byte header,
 * payload CRC32 + trailer; see legacy/c5vrx1/tools/c5vrx_usb_protocol.py).
 *
 * Realtime contract (AGENTS.md invariants):
 *   - The ring is never written, never gated, and USB never paces IQ.
 *   - All work happens in one low-priority task (prio 1) that sleeps when
 *     disabled and yields 1 ms per pass; the AGC task (prio 3) preempts
 *     freely at any point. Each grab_frame() call is bounded by a timeout so
 *     lower priorities keep breathing.
 *   - The grabber reads the ring through cache-synced copies and its own
 *     overwrite checks; a DMA boundary is never treated as a DSP reset of
 *     the production path.
 *
 * Row streaming: the row callback stages up to ROW_BATCH row records
 * (row_index, flags, 224 GRAY8 bytes) and flushes a packet when the batch
 * is full or the frame's last row (GRAB_H-1) arrives. Two pacing layers,
 * both frame-level, never mid-frame: a 15 fps cap drops whole frames whose
 * predecessor finished < 66 ms ago (full-rate row bursts overwhelmed the
 * USB-Serial/JTAG link), and if a frame's final packet does not get out
 * within USB_WRITE_BUDGET_US it is kept and retried while rows of the next
 * frame are dropped (s_draining) until the last row has been sent. The
 * grabber always runs at full rate; only the wire is paced.
 *
 * Snow: whenever no sync frame has completed on the wire for
 * SYNC_FEED_STALE_US (1.5 s), raw ring bytes (I^2+Q^2 as luma) are streamed
 * as rows too, so the preview always shows what is on the channel: live
 * noise while unlocked, real video while sync frames flow, and snow again
 * within 1.5 s if a locked feed stalls, so the video pane can never freeze.
 */

#include "preview.h"
#include "grab.h"
#include "video.h"

#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "esp_cache.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "c5vrx3_preview";

/* ----- v1 USB packet wire format (CRC-32/ISO-HDLC, little-endian). The host
 * parser (tools/flask_app.py) depends on this exact layout: do not change. --- */
#define USB_HEADER_BYTES    32u
#define PACKET_STREAM_INFO  1u
#define PACKET_GRAY8_FRAME  2u
#define PACKET_STREAM_END   3u
/* Ids 4..7 are the legacy IQ/YUV packet types (see
 * legacy/c5vrx1/tools/c5vrx_usb_protocol.py); 8 is the next free id. */
#define PACKET_GRAY8_ROWS   8u
#define PIXEL_FORMAT_GRAY8  1u
#define PROTOCOL_VERSION    1u
#define FRAME_DESCRIPTOR_BYTES 8u

#define SYNC_FEED_STALE_US  1500000   /* no sync frame complete for 1.5 s -> snow */
#define USB_WRITE_BUDGET_US   60000   /* drop/skip instead of stalling longer */
#define FRAME_MIN_INTERVAL_US 66000   /* 15 fps frame-level cap: full-rate row
                                         * bursts (~1.1 MB/s at 30-50 fields/s)
                                         * correlate with USB-Serial/JTAG
                                         * drop-offs; whole frames are dropped
                                         * at this boundary, never mid-frame. */

/* Bounds for one grab_frame() call: locked, one frame is two fields (33 ms
 * NTSC / 40 ms PAL), so 60 ms completes a frame with slack; unlocked, the
 * acquire search gets enough window cadences (102 us each) to lock. Both
 * bounds keep the task from monopolizing the CPU between its 1 ms yields. */
#define GRAB_TIMEOUT_LOCKED_MS   60
#define GRAB_TIMEOUT_ACQUIRE_MS  40
/* After a grab that produced no rows (no signal), re-acquire in short
 * bursts so the snow path gets the wire several times per canvas cycle
 * instead of once per 40 ms acquire block. Acquisition only needs a few
 * window cadences (102 us each), so 12 ms still gives real video ample
 * chance to lock on the next pass. */
#define GRAB_TIMEOUT_SEARCH_MS   12

/* Row batching: up to ROW_BATCH row records per PACKET_GRAY8_ROWS.
 * record = u8 row_index (0..GRAB_H-1), u8 flags (bit0 = last row of frame),
 * GRAB_W bytes GRAY8. Payload = u8 row_count followed by the records. */
#define ROW_BATCH        4u
#define ROW_RECORD_BYTES (2u + GRAB_W)
_Static_assert(GRAB_W == (int)PREVIEW_WIDTH && GRAB_H == (int)PREVIEW_HEIGHT,
               "wire canvas must match the grabber geometry");

static const uint8_t s_usb_magic[8] = {0x00, 'C', '5', 'V', 'R', 'X', 0xa5, 0x5a};

static volatile bool s_enabled;
static TaskHandle_t s_task;
static uint32_t s_sequence;
static uint32_t s_frames_completed;     /* grabs completing all GRAB_H rows */
static uint32_t s_lines_captured;       /* grabber rows delivered, lifetime */
static uint32_t s_field_count;          /* fields visited, lifetime */
static uint32_t s_errs;                 /* grabs ending in error, lifetime */
static int64_t s_last_sync_publish_us;  /* last completed sync frame on the wire */
static bool s_h_locked;
/* Last grab, reduced to what the heartbeat prints (grab_info_t is ~150 B;
 * HP SRAM is the binding constraint). */
static struct {
    float line_us;
    int   rows;
    int   nosync;
    int   late;
    int   vmisses;
    bool  pal;
} s_last;

/* Row staging and field-level pacing. */
static uint8_t s_rows[ROW_BATCH][ROW_RECORD_BYTES];
static unsigned s_nrows;        /* staged row records (<= ROW_BATCH) */
static bool s_draining;         /* previous frame's final packet not yet sent:
                                   rows of the current frame are dropped and
                                   the buffered final packet is retried first */
static uint32_t s_rows_sent;
static uint32_t s_rows_dropped;
static int64_t  s_last_frame_done_us;   /* final row of the last published frame out */
static bool     s_pace_drop;            /* this frame's rows dropped by the 15 fps cap */

/* Snow mode state: rows of raw-I/Q luma at circular row indices. */
static unsigned s_snow_row;         /* circular row index 0..PREVIEW_HEIGHT-1 */
static unsigned s_snow_chunks;      /* ticks since the last snow flush */
#define SNOW_MAX_CHUNKS 4u          /* a complete snow frame every 4th snow tick */
#define SNOW_ROWS_PER_TICK 4u       /* == ROW_BATCH: one row packet per snow tick */

/* ~1 Hz heartbeat cadence (printed only while enabled). */
static int64_t s_dbg_last_us;
static uint32_t s_passes;           /* preview_tick passes since boot */
static uint32_t s_grab_us;          /* last grab_frame duration */
static uint32_t s_grab_max_us;      /* worst grab_frame duration since last print */

/* Cache synchronization helper for CPU reads of the DMA ring.
 * Same align-down/size-up pattern as video.c's sync_dma_m2c(). */
static inline void sync_ring_m2c(const void *addr, size_t size)
{
    if (!addr || size == 0) return;
    uint32_t start = (uint32_t)addr & ~(64u - 1u);
    uint32_t end = ((uint32_t)addr + size + 63u) & ~(64u - 1u);
    (void)esp_cache_msync((void *)start, end - start,
                          ESP_CACHE_MSYNC_FLAG_DIR_M2C | ESP_CACHE_MSYNC_FLAG_UNALIGNED);
}

/* Standard reflected CRC-32/ISO-HDLC, matching Python zlib.crc32 exactly.
 * Table-less port of the v1 firmware implementation. */
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

/* Bounded best-effort write. stdout is O_NONBLOCK, so a stalled or absent
 * host yields short writes; we retry until the deadline and then give up.
 * Callers MUST hold flockfile(stdout) for the entire packet: header +
 * payload + trailer must stay contiguous or the host parser desyncs.
 * (Only send_packet calls this, and it does.) */
static bool write_all(const uint8_t *data, size_t len, int64_t deadline_us)
{
    while (len) {
        size_t n = fwrite(data, 1, len, stdout);
        data += n;
        len -= n;
        if (!len) return true;
        if (esp_timer_get_time() > deadline_us) return false;
        if (n == 0) vTaskDelay(pdMS_TO_TICKS(1));
    }
    return true;
}

static bool send_packet(unsigned type,
                        const uint8_t *first, size_t first_count,
                        const uint8_t *second, size_t second_count,
                        int64_t deadline_us)
{
    uint8_t header[USB_HEADER_BYTES] = {0};
    memcpy(header, s_usb_magic, sizeof(s_usb_magic));
    header[8] = PROTOCOL_VERSION;
    header[9] = (uint8_t)type;
    put_le16(header + 10, USB_HEADER_BYTES);
    put_le32(header + 12, s_sequence++);
    put_le32(header + 16, (uint32_t)(first_count + second_count));
    put_le64(header + 20, (uint64_t)esp_timer_get_time());
    put_le32(header + 28, crc32_parts(header, 28u, NULL, 0u));
    uint8_t trailer[4];
    put_le32(trailer, crc32_parts(first, first_count, second, second_count));

    flockfile(stdout);
    bool sent = write_all(header, sizeof(header), deadline_us) &&
                write_all(first, first_count, deadline_us) &&
                write_all(second, second_count, deadline_us) &&
                write_all(trailer, sizeof(trailer), deadline_us);
    funlockfile(stdout);
    return sent;
}

static void make_descriptor(uint8_t descriptor[FRAME_DESCRIPTOR_BYTES],
                            uint8_t flags)
{
    put_le16(descriptor, PREVIEW_WIDTH);
    put_le16(descriptor + 2, PREVIEW_HEIGHT);
    put_le16(descriptor + 4, PREVIEW_WIDTH); /* stride == width for GRAY8 */
    descriptor[6] = PIXEL_FORMAT_GRAY8;
    descriptor[7] = flags;
}

/* Send the staged rows as one PACKET_GRAY8_ROWS (payload: u8 row_count,
 * then row_count records of u8 row_index, u8 flags, GRAB_W bytes).
 * keep_on_fail retains the staging (and the rows are not counted) so the
 * final packet of a frame can be retried; otherwise a budget failure drops
 * the staged rows. Never blocks past USB_WRITE_BUDGET_US. */
static bool rows_send_staged(bool keep_on_fail)
{
    if (s_nrows == 0u) return true;
    uint8_t payload[1u + ROW_BATCH * ROW_RECORD_BYTES];
    payload[0] = (uint8_t)s_nrows;
    memcpy(payload + 1u, s_rows, s_nrows * ROW_RECORD_BYTES);
    bool ok = send_packet(PACKET_GRAY8_ROWS, payload,
                          1u + s_nrows * ROW_RECORD_BYTES,
                          NULL, 0u, esp_timer_get_time() + USB_WRITE_BUDGET_US);
    if (ok) {
        s_rows_sent += s_nrows;
        s_nrows = 0u;
        return true;
    }
    if (!keep_on_fail) {
        s_rows_dropped += s_nrows;
        s_nrows = 0u;
    }
    return false;
}

/* Stage one row record (grabber row or snow row). last = frame-final row:
 * flags bit0 set and the packet flushed at once; if it does not get out,
 * s_draining is armed so the next frame's rows are dropped and this final
 * packet is retried from the task loop. flags bit1 = grabber-locked, valid
 * on frame-final rows (the host's lock badge); snow passes locked=false. */
static void row_stage(uint8_t row_index, const uint8_t *row, bool last, bool locked)
{
    if (s_draining) { ++s_rows_dropped; return; }
    uint8_t *rec = s_rows[s_nrows];
    rec[0] = row_index;
    rec[1] = (uint8_t)((last ? 1u : 0u) | ((last && locked) ? 2u : 0u));
    memcpy(rec + 2u, row, GRAB_W);
    ++s_nrows;
    if (s_nrows >= ROW_BATCH || last) {
        if (!rows_send_staged(last) && last) s_draining = true;
        if (last && !s_draining) {
            s_last_sync_publish_us = esp_timer_get_time();
            s_last_frame_done_us = s_last_sync_publish_us;
        }
    }
}

/* ----- Snow path ----- */

/* Snow mode: whenever no sync frame has completed on the wire recently,
 * show the channel as-is -- rows of raw ring bytes mapped to luma
 * (I^2+Q^2), straight from a lagging slice of the ring, at circular row
 * indices so the whole canvas refreshes round-robin. One packet per tick
 * (SNOW_ROWS_PER_TICK == ROW_BATCH); every SNOW_MAX_CHUNKS-th tick the last
 * row carries bit0 (a complete snow frame) -- passed through row_stage so
 * the batch-full flush carries it, instead of flushing behind its back
 * (which starved the cadence: the batch flush emptied the staging before
 * the bit0 check ever saw it). The gate is a stale-sync timeout, NOT
 * h_locked: a locked grab whose frames stop completing falls back to snow
 * within SYNC_FEED_STALE_US so the video pane can never freeze. */
static void snow_tick(void)
{
    size_t ring_size = 0;
    uint32_t rx_off = 0;
    const uint8_t *ring = video_tap_iq_ring(&ring_size, &rx_off);
    if (!ring || ring_size < SNOW_ROWS_PER_TICK * PREVIEW_WIDTH + 4096u) return;

    uint8_t luma[SNOW_ROWS_PER_TICK * PREVIEW_WIDTH];
    uint32_t start = (rx_off + ring_size - 4096u - (uint32_t)sizeof(luma)) % ring_size;
    uint32_t first = (uint32_t)sizeof(luma);
    if (start + first > ring_size) first = ring_size - start;
    sync_ring_m2c(ring + start, first);
    if (first < (uint32_t)sizeof(luma))
        sync_ring_m2c(ring, (uint32_t)sizeof(luma) - first);
    for (uint32_t k = 0; k < (uint32_t)sizeof(luma); ++k) {
        uint8_t b = ring[(start + k) % ring_size];
        int i4 = (int)(int8_t)(b & 0xf0u) >> 4;
        int q4 = (int)(int8_t)((b & 0x0fu) << 4) >> 4;
        unsigned v = (unsigned)(i4 * i4 + q4 * q4);     /* 0..128 */
        luma[k] = (uint8_t)(v > 127u ? 255u : v * 2u);
    }

    ++s_snow_chunks;
    const bool frame_end = (s_snow_chunks >= SNOW_MAX_CHUNKS);
    for (unsigned r = 0; r < SNOW_ROWS_PER_TICK; ++r) {
        /* Snow rows are a fallback display: never let them pile up behind a
         * stuck link -- drop on drain instead of buffering. */
        if (s_draining) { s_rows_dropped += SNOW_ROWS_PER_TICK - r; break; }
        const bool last = frame_end && (r == SNOW_ROWS_PER_TICK - 1u);
        row_stage((uint8_t)s_snow_row, luma + r * PREVIEW_WIDTH, last, false);
        if (++s_snow_row >= PREVIEW_HEIGHT) s_snow_row = 0;
    }
    if (frame_end) s_snow_chunks = 0u;
}

/* ----- Grabber wiring ----- */

/* Row callback (runs inside the grab loop; must be quick). Native
 * resolution: the row is copied verbatim, no decimation. */
static void preview_on_row(void *ctx, int y, const uint8_t *row)
{
    (void)ctx;
    if (y < 0 || y >= GRAB_H) return;
    s_lines_captured++;
    /* Frame-level rate cap (set once per frame in preview_tick): the frame
     * is dropped whole, never mid-frame, and the grabber still ran -- only
     * the wire is paced. Snow rows do not pass through here. */
    if (s_pace_drop) { ++s_rows_dropped; return; }
    /* Row 167 arriving means every row of this frame was captured, so the
     * grab is complete and locked regardless of the flag's update timing. */
    row_stage((uint8_t)y, row, y == GRAB_H - 1,
              s_h_locked || y == GRAB_H - 1);
}

static void preview_reset_counters(void)
{
    s_frames_completed = 0;
    s_lines_captured = 0;
    s_field_count = 0;
    s_errs = 0;
    s_last_sync_publish_us = 0;
    s_h_locked = false;
    memset(&s_last, 0, sizeof(s_last));
    s_nrows = 0u;
    s_draining = false;
    s_rows_sent = 0;
    s_rows_dropped = 0;
    s_snow_row = 0;
    s_snow_chunks = 0;
    s_passes = 0;
    s_grab_us = 0;
    s_grab_max_us = 0;
    s_last_frame_done_us = 0;
    s_pace_drop = false;
}

/* One task pass: retry a stuck final packet, then a bounded grab attempt,
 * then finalize an incomplete frame's staged tail, then the snow/stale
 * gate. The 1 ms yield keeps lower priorities fed. */
static void preview_tick(void)
{
    ++s_passes;
    /* ~1 Hz heartbeat FIRST, before any gate, and via esp_log: printf output
     * from this task demonstrably never reached the USB host while esp_log
     * output did. A silent [TAP] now unambiguously means the task is dead. */
    int64_t now0 = esp_timer_get_time();
    if (now0 - s_dbg_last_us >= 1000000) {
        s_dbg_last_us = now0;
        unsigned lwhole = (unsigned)s_last.line_us;
        unsigned lfrac = (unsigned)(s_last.line_us * 100.0f + 0.5f) % 100u;
        ESP_LOGW(TAG, "[TAP] hlock=%u pal=%u line=%u.%02u rows=%d fields=%lu nosync=%d late=%d vmiss=%d err=%lu wire=%lu drop=%lu pass=%lu gus=%lu gmax=%lu",
                 s_h_locked ? 1u : 0u, s_last.pal ? 1u : 0u, lwhole, lfrac, s_last.rows,
                 (unsigned long)s_field_count, s_last.nosync, s_last.late, s_last.vmisses,
                 (unsigned long)s_errs, (unsigned long)s_rows_sent, (unsigned long)s_rows_dropped,
                 (unsigned long)s_passes, (unsigned long)s_grab_us, (unsigned long)s_grab_max_us);
        s_grab_max_us = 0u;     /* worst-since-print, like the demod min..max pair */
    }

    /* The previous frame never finished on the wire: retry its final packet
     * first; rows of this frame are dropped until it gets out. */
    if (s_draining && rows_send_staged(true)) {
        s_draining = false;
        s_last_sync_publish_us = esp_timer_get_time();
        s_last_frame_done_us = s_last_sync_publish_us;
    }

    /* 15 fps frame-level cap: drop this frame's rows if the previous
     * frame's final row went out < 66 ms ago. Decided once per grab, so a
     * frame is never cut mid-way; partial frames that DO pass still
     * finalize via the post-grab tail flush. */
    s_pace_drop = (s_last_frame_done_us != 0) &&
                  (esp_timer_get_time() - s_last_frame_done_us <
                   (int64_t)FRAME_MIN_INTERVAL_US);

    /* Row staging buffer: one grabber row at a time, on the stack (lifetime
     * is a single grab_frame call; .bss is the binding constraint). */
    uint8_t row_stage[GRAB_W];
    grab_info_t info;
    const int timeout_ms = s_h_locked ? GRAB_TIMEOUT_LOCKED_MS
                          : (s_last.rows > 0 ? GRAB_TIMEOUT_ACQUIRE_MS
                                             : GRAB_TIMEOUT_SEARCH_MS);
    int64_t t_grab = esp_timer_get_time();
    int rows = grab_frame(row_stage, &info, timeout_ms,
                          preview_on_row, NULL, NULL);
    s_grab_us = (uint32_t)(esp_timer_get_time() - t_grab);
    if (s_grab_us > s_grab_max_us) s_grab_max_us = s_grab_us;
    s_last.line_us = info.line_us;
    s_last.rows = info.rows;
    s_last.nosync = info.nosync;
    s_last.late = info.late;
    s_last.vmisses = info.vmisses;
    s_last.pal = info.pal;
    if (info.fields > 0) s_field_count += (uint32_t)info.fields;
    if (info.error) ++s_errs;
    s_h_locked = (rows == GRAB_H && info.error == NULL);
    if (s_h_locked) ++s_frames_completed;

    /* Incomplete grab (a row was given up or the timeout hit): the rows
     * still staged are the frame's tail -- mark the last one frame-final
     * and send it. Rows the grabber missed keep the host's previous frame. */
    if (!s_draining && s_nrows > 0u) {
        /* Incomplete grab: the staged tail ends the frame here. Its lock
         * badge reflects the last grab state (transitions may lag a frame). */
        s_rows[s_nrows - 1u][1] |= (uint8_t)(1u | (s_h_locked ? 2u : 0u));
        if (!rows_send_staged(true)) s_draining = true;
        else {
            s_last_sync_publish_us = esp_timer_get_time();
            s_last_frame_done_us = s_last_sync_publish_us;
        }
    }

    if (esp_timer_get_time() - s_last_sync_publish_us > SYNC_FEED_STALE_US) {
        /* No sync frame completed on the wire recently (unlocked, or a
         * locked feed that stalled): keep the pane alive with snow. */
        snow_tick();
    } else {
        /* Live sync feed owns the wire; park snow state so a later stall
         * restarts from a fresh canvas. */
        s_snow_row = 0;
        s_snow_chunks = 0;
    }
    vTaskDelay(pdMS_TO_TICKS(1));
}

static void preview_task(void *arg)
{
    (void)arg;
    bool stream_open = false;
    for (;;) {
        if (!s_enabled) {
            if (stream_open) {
                /* Best-effort STREAM_END (64-bit device-dropped row count). */
                uint8_t payload[8];
                put_le64(payload, s_rows_dropped);
                (void)send_packet(PACKET_STREAM_END, payload, sizeof(payload),
                                  NULL, 0u, esp_timer_get_time() + 20000);
                stream_open = false;
            }
            /* Bounded wait: the notification is a latency hint, never a
             * handshake -- a missed give can never park the task. */
            ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(200));
            continue;
        }
        if (!stream_open) {
            uint8_t descriptor[FRAME_DESCRIPTOR_BYTES];
            make_descriptor(descriptor, 0u);
            (void)send_packet(PACKET_STREAM_INFO, descriptor, sizeof(descriptor),
                              NULL, 0u, esp_timer_get_time() + 20000);
            stream_open = true;
        }
        preview_tick();
    }
}

/* ----- Public API ----- */

esp_err_t preview_init(void)
{
    if (s_task) return ESP_OK;
    grab_init();
    preview_reset_counters();

    /* Buffers are static (see declarations); nothing can fail here. Stack
     * is 4 KB (IDF xTaskCreate measures in bytes): the deepest path is the
     * row flush (~1.9 KB peak with the grabber arrays also static). */
    if (xTaskCreate(preview_task, "usb_preview", 4096, NULL, 1,
                    &s_task) != pdPASS) {
        s_task = NULL;
        return ESP_ERR_NO_MEM;
    }
    /* Always on from boot: no external trigger, no enable handshake. */
    s_enabled = true;
    return ESP_OK;
}

void preview_set_enabled(bool enabled)
{
    if (!s_task) {
        if (enabled) printf("[PREVIEW] UNAVAILABLE (task creation failed at boot)\n");
        return;
    }
    if (s_enabled == enabled) return;
    if (enabled) {
        /* Re-enable: drop whatever lock the grabber holds and restart the
         * counters, exactly like a fresh boot of the tap. */
        grab_unlock();
        preview_reset_counters();
        s_enabled = true;
        if (s_task) xTaskNotifyGive(s_task);
    } else {
        s_enabled = false;
    }
}

bool preview_is_enabled(void)
{
    return s_enabled;
}

void preview_get_stats(uint32_t *completed, uint32_t *sent,
                       uint32_t *dropped, uint32_t *lines,
                       bool *h_locked, uint32_t *vsyncs)
{
    if (completed) *completed = s_frames_completed;
    if (sent) *sent = s_rows_sent;
    if (dropped) *dropped = s_rows_dropped;
    if (lines) *lines = s_lines_captured;
    if (h_locked) *h_locked = s_h_locked;
    if (vsyncs) *vsyncs = s_field_count;
}
