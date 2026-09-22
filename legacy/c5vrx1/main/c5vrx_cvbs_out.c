/* SPDX-License-Identifier: GPL-3.0-only */

#include "c5vrx_cvbs_out.h"

#include <math.h>
#include <stddef.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "sdkconfig.h"
#include "driver/gpio.h"
#include "driver/parlio_tx.h"
#include "esp_attr.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

/*
 * Analog-first output proof.
 *
 * The C5 cannot afford a full 625-line byte framebuffer at 20 MS/s, so the
 * test generator streams a PAL 625/50 interlaced raster through two small DMA
 * chunks. PARLIO's loop-buffer switching keeps one chunk on the wire while a
 * high-priority task rebuilds the retired chunk.
 *
 * The active picture is monochrome grayscale. Optional PAL-frequency swinging
 * burst is present only as an output-bandwidth/PLL stress signal; there is no
 * active chroma yet and this is not claimed as broadcast-compliance equipment.
 */
#define C5VRX_CVBS_SAMPLE_RATE_HZ       20000000u
#define C5VRX_CVBS_LINE_SAMPLES         1280u
#define C5VRX_CVBS_HALF_LINE_SAMPLES    640u
#define C5VRX_CVBS_FIELD_HALF_LINES     625u
#define C5VRX_CVBS_FRAME_HALF_LINES     1250u

#define C5VRX_CVBS_HSYNC_SAMPLES        94u   /* 4.70 us */
#define C5VRX_CVBS_EQ_SAMPLES           47u   /* 2.35 us */
#define C5VRX_CVBS_BROAD_SYNC_SAMPLES   546u  /* 27.30 us */
#define C5VRX_CVBS_ACTIVE_START         210u  /* 10.50 us after line datum */
#define C5VRX_CVBS_ACTIVE_END           1250u /* leaves 1.50 us front porch */
#define C5VRX_CVBS_ACTIVE_SAMPLES       (C5VRX_CVBS_ACTIVE_END - C5VRX_CVBS_ACTIVE_START)

#define C5VRX_CVBS_EQ_HALF_LINES        5u
#define C5VRX_CVBS_BROAD_HALF_LINES     5u
#define C5VRX_CVBS_NORMAL_START_HALF    15u
/*
 * Starting active video at local half-line 49 gives 576 active half-lines
 * (288 full lines) per field while keeping the first active half-line aligned
 * to the normal H-sync cadence that resumes after the post-equalizing pulses.
 */
#define C5VRX_CVBS_ACTIVE_START_HALF    49u

/* Two 40-half-line DMA chunks keep 2.56 ms already queued on the wire. That
 * exceeds the 1.93..1.99 ms bounded vendor-capture interval measured on the
 * physical XIAO while using 30,720 fewer heap bytes than the proof build. */
#define C5VRX_CVBS_CHUNK_HALF_LINES     40u
#define C5VRX_CVBS_CHUNK_SAMPLES        (C5VRX_CVBS_CHUNK_HALF_LINES * C5VRX_CVBS_HALF_LINE_SAMPLES)

#define C5VRX_CVBS_STREAM_STACK         4096u
#define C5VRX_CVBS_STREAM_PRIORITY      18u

#define C5VRX_CVBS_RETIRED_BUF0         (1u << 0)
#define C5VRX_CVBS_RETIRED_BUF1         (1u << 1)
#define C5VRX_CVBS_STOP_NOTIFY          (1u << 31)

#define C5VRX_PAL_BURST_START_SAMPLES   112u  /* 5.60 us after line datum */
#define C5VRX_PAL_BURST_SAMPLES         45u   /* 2.25 us @ 20 MS/s */
#define C5VRX_PAL_SUBCARRIER_HZ          4433618.75
#define C5VRX_PAL_BURST_AMPLITUDE_MV    150.0
#define C5VRX_PI                        3.14159265358979323846

#if CONFIG_C5VRX_EXPERIMENTAL_CVBS_PARLIO

static const char *TAG = "c5vrx_cvbs";

typedef struct {
    TaskHandle_t task;
    volatile bool stop_requested;
} c5vrx_cvbs_stream_state_t;

static uint8_t s_eq_half[C5VRX_CVBS_HALF_LINE_SAMPLES] __attribute__((aligned(64)));
static uint8_t s_broad_half[C5VRX_CVBS_HALF_LINE_SAMPLES] __attribute__((aligned(64)));
static uint8_t s_blank_first[C5VRX_CVBS_HALF_LINE_SAMPLES] __attribute__((aligned(64)));
static uint8_t s_blank_second[C5VRX_CVBS_HALF_LINE_SAMPLES] __attribute__((aligned(64)));
static uint8_t s_active_first[2][C5VRX_CVBS_HALF_LINE_SAMPLES] __attribute__((aligned(64)));
static uint8_t s_active_second[C5VRX_CVBS_HALF_LINE_SAMPLES] __attribute__((aligned(64)));

/* Keep the 51,200-byte always-on stream buffer out of static DRAM. The RF
 * dump writer owns the fixed 0x40830000..0x4083ffff window, so a static array
 * this large could make the linker place ordinary application state inside
 * hardware-owned RAM. DMA-capable buffers are allocated when AV starts; the
 * reserved dump window is excluded from the heap. */
static uint8_t *s_chunk[2];

static parlio_tx_unit_handle_t s_tx;
static c5vrx_cvbs_stream_state_t s_stream;
static uint16_t s_next_half_line;
static uint32_t s_frame_counter;
static bool s_running;
static volatile c5vrx_cvbs_display_t s_requested_display =
    C5VRX_CVBS_DISPLAY_LOGO;
static volatile c5vrx_cvbs_display_t s_active_display =
    C5VRX_CVBS_DISPLAY_LOGO;
static uint32_t s_snow_lfsr = 0xc5f0a17du;

static uint8_t cvbs_code_from_mv(unsigned mv);
static uint8_t grayscale_code(unsigned active_x);

/* Compact 5x7 uppercase font, indexed as ASCII 32..90. Only branding glyphs
 * are populated; missing glyphs render as spacing. This remains scanline-only. */
typedef struct { char c; uint8_t row[7]; } glyph_t;
static const glyph_t s_font[] = {
    {'0',{14,17,19,21,25,17,14}}, {'2',{14,17,1,2,4,8,31}},
    {'3',{30,1,1,14,1,1,30}},
    {'5',{31,16,30,1,1,17,14}}, {'6',{6,8,16,30,17,17,14}},
    {'-',{0,0,0,31,0,0,0}}, {'/',{1,2,4,8,16,0,0}},
    {'A',{14,17,17,31,17,17,17}}, {'B',{30,17,17,30,17,17,30}},
    {'C',{14,17,16,16,16,17,14}}, {'D',{30,17,17,17,17,17,30}},
    {'E',{31,16,16,30,16,16,31}}, {'F',{31,16,16,30,16,16,16}},
    {'G',{14,17,16,23,17,17,15}},
    {'I',{31,4,4,4,4,4,31}}, {'L',{16,16,16,16,16,16,31}},
    {'M',{17,27,21,21,17,17,17}}, {'N',{17,25,21,19,17,17,17}},
    {'O',{14,17,17,17,17,17,14}}, {'P',{30,17,17,30,16,16,16}},
    {'R',{30,17,17,30,20,18,17}}, {'S',{15,16,16,14,1,1,30}},
    {'T',{31,4,4,4,4,4,4}}, {'U',{17,17,17,17,17,17,14}},
    {'V',{17,17,17,17,17,10,4}}, {'X',{17,17,10,4,10,17,17}},
};

static uint8_t glyph_row(char c, unsigned row)
{
    for (unsigned i = 0; i < sizeof(s_font) / sizeof(s_font[0]); ++i) {
        if (s_font[i].c == c) return s_font[i].row[row];
    }
    return 0;
}

static bool text_pixel(const char *text, int x, int y, int origin_x,
                       int origin_y, unsigned scale)
{
    if (x < origin_x || y < origin_y) return false;
    const unsigned px = (unsigned)(x - origin_x) / scale;
    const unsigned py = (unsigned)(y - origin_y) / scale;
    if (py >= 7u) return false;
    const unsigned char_index = px / 6u;
    const unsigned column = px % 6u;
    const size_t length = strlen(text);
    if (char_index >= length || column >= 5u) return false;
    return (glyph_row(text[char_index], py) & (1u << (4u - column))) != 0u;
}

static bool branded_pixel(unsigned x, unsigned y)
{
    const bool splash = s_frame_counter < 300u; /* 12 seconds at 25 fps. */
    if (splash) {
        return text_pixel("C5VRX", x, y, 320, 70, 12) ||
               text_pixel("ESP32-C5 ANALOG FPV RECEIVER", x, y, 250, 190, 3) ||
               text_pixel("PAL CVBS OUTPUT", x, y, 380, 230, 3);
    }
    if (x < 8u || x >= C5VRX_CVBS_ACTIVE_SAMPLES - 8u || y < 5u || y >= 283u)
        return true;
    if ((x % 130u == 0u) || (y % 48u == 0u)) return true;
    return text_pixel("C5VRX", x, y, 24, 15, 5) ||
           text_pixel("PAL CVBS PROOF", x, y, 650, 18, 3) ||
           text_pixel("20 MS/S - 6-BIT DAC", x, y, 590, 252, 3);
}

static uint8_t diagnostic_code(unsigned x, unsigned y)
{
    if (branded_pixel(x, y)) return cvbs_code_from_mv(1000);
    if (s_frame_counter < 300u) return cvbs_code_from_mv(320);
    if (y >= 210u && y < 238u) {
        if (x < C5VRX_CVBS_ACTIVE_SAMPLES / 3u) return cvbs_code_from_mv(320);
        if (x > 2u * C5VRX_CVBS_ACTIVE_SAMPLES / 3u) return cvbs_code_from_mv(1000);
    }
    return grayscale_code(x);
}

static uint8_t logo_code(unsigned x, unsigned y)
{
    const bool mark =
        text_pixel("C5VRX", x, y, 320, 70, 12) ||
        text_pixel("ESP32-C5 ANALOG FPV RECEIVER", x, y, 250, 190, 3) ||
        text_pixel("WAITING FOR VIDEO", x, y, 370, 230, 3);
    return cvbs_code_from_mv(mark ? 1000u : 320u);
}

static uint8_t snow_code(void)
{
    /* A Galois LFSR gives moving full-range monochrome snow without a
     * framebuffer. H/V sync and blanking remain standards-shaped, so an
     * attached display stays locked instead of confusing noise with no AV. */
    const uint32_t lsb = s_snow_lfsr & 1u;
    s_snow_lfsr = (s_snow_lfsr >> 1u) ^ (0xd0000001u & (0u - lsb));
    const unsigned mv = 320u + ((s_snow_lfsr >> 16u) & 0xffu) * 680u / 255u;
    return cvbs_code_from_mv(mv);
}

static uint8_t cvbs_code_from_mv(unsigned mv)
{
    const unsigned bits = CONFIG_C5VRX_CVBS_DAC_BITS;
    const unsigned max_code = (1u << bits) - 1u;
    if (mv >= 1000u) {
        return (uint8_t)max_code;
    }
    return (uint8_t)((mv * max_code + 500u) / 1000u);
}

static uint8_t cvbs_code_from_mv_signed(double mv)
{
    if (mv <= 0.0) {
        return 0;
    }
    if (mv >= 1000.0) {
        return cvbs_code_from_mv(1000);
    }
    return cvbs_code_from_mv((unsigned)(mv + 0.5));
}

static uint8_t grayscale_code(unsigned active_x)
{
    const unsigned black_mv = 320u;
    const unsigned white_mv = 1000u;
    unsigned bar = active_x / (C5VRX_CVBS_ACTIVE_SAMPLES / 8u);
    if (bar > 7u) {
        bar = 7u;
    }

    /* White -> black makes clipping and pedestal errors easy to spot. */
    const unsigned level = white_mv -
        ((white_mv - black_mv) * bar + 3u) / 7u;
    return cvbs_code_from_mv(level);
}

static void add_pal_burst(uint8_t *line_first, unsigned phase_index)
{
#if CONFIG_C5VRX_CVBS_TEST_COLOR_BURST
    const double phase_step =
        2.0 * C5VRX_PI * C5VRX_PAL_SUBCARRIER_HZ /
        (double)C5VRX_CVBS_SAMPLE_RATE_HZ;

    /*
     * PAL-frequency swinging burst stress: alternate +135/-135 degrees. The
     * active image remains monochrome, so this tests output bandwidth/locking
     * rather than claiming complete PAL chroma generation.
     */
    const double phase0 = phase_index ? (-3.0 * C5VRX_PI / 4.0)
                                      : ( 3.0 * C5VRX_PI / 4.0);

    for (unsigned i = 0; i < C5VRX_PAL_BURST_SAMPLES; ++i) {
        const unsigned p = C5VRX_PAL_BURST_START_SAMPLES + i;
        const double mv = 300.0 +
            C5VRX_PAL_BURST_AMPLITUDE_MV * sin(phase0 + phase_step * i);
        line_first[p] = cvbs_code_from_mv_signed(mv);
    }
#else
    (void)line_first;
    (void)phase_index;
#endif
}

static void build_templates(void)
{
    const uint8_t sync = cvbs_code_from_mv(0);
    const uint8_t blank = cvbs_code_from_mv(300);

    memset(s_eq_half, blank, sizeof(s_eq_half));
    memset(s_eq_half, sync, C5VRX_CVBS_EQ_SAMPLES);

    memset(s_broad_half, blank, sizeof(s_broad_half));
    memset(s_broad_half, sync, C5VRX_CVBS_BROAD_SYNC_SAMPLES);

    memset(s_blank_first, blank, sizeof(s_blank_first));
    memset(s_blank_first, sync, C5VRX_CVBS_HSYNC_SAMPLES);
    memset(s_blank_second, blank, sizeof(s_blank_second));

    for (unsigned phase = 0; phase < 2; ++phase) {
        memset(s_active_first[phase], blank, sizeof(s_active_first[phase]));
        memset(s_active_first[phase], sync, C5VRX_CVBS_HSYNC_SAMPLES);
        add_pal_burst(s_active_first[phase], phase);

        for (unsigned p = C5VRX_CVBS_ACTIVE_START;
             p < C5VRX_CVBS_HALF_LINE_SAMPLES; ++p) {
            s_active_first[phase][p] =
                grayscale_code(p - C5VRX_CVBS_ACTIVE_START);
        }
    }

    memset(s_active_second, blank, sizeof(s_active_second));
    const unsigned second_active_samples =
        C5VRX_CVBS_ACTIVE_END - C5VRX_CVBS_HALF_LINE_SAMPLES;
    for (unsigned p = 0; p < second_active_samples; ++p) {
        const unsigned x =
            (C5VRX_CVBS_HALF_LINE_SAMPLES - C5VRX_CVBS_ACTIVE_START) + p;
        s_active_second[p] = grayscale_code(x);
    }
}

static const uint8_t *template_for_half_line(uint16_t frame_half_line)
{
    const unsigned field_pos =
        (unsigned)(frame_half_line % C5VRX_CVBS_FIELD_HALF_LINES);

    if (field_pos < C5VRX_CVBS_EQ_HALF_LINES) {
        return s_eq_half;
    }
    if (field_pos < C5VRX_CVBS_EQ_HALF_LINES + C5VRX_CVBS_BROAD_HALF_LINES) {
        return s_broad_half;
    }
    if (field_pos < C5VRX_CVBS_NORMAL_START_HALF) {
        return s_eq_half;
    }

    const unsigned normal_offset = field_pos - C5VRX_CVBS_NORMAL_START_HALF;
    const bool first_half = ((normal_offset & 1u) == 0u);
    const bool active = (field_pos >= C5VRX_CVBS_ACTIVE_START_HALF);

    if (!active) {
        return first_half ? s_blank_first : s_blank_second;
    }

    if (!first_half) {
        return s_active_second;
    }

    const unsigned line_index = normal_offset / 2u;
    return s_active_first[line_index & 1u];
}

static void overlay_active_half(uint8_t *half, uint16_t frame_half_line)
{
    const unsigned field_pos = frame_half_line % C5VRX_CVBS_FIELD_HALF_LINES;
    if (field_pos < C5VRX_CVBS_ACTIVE_START_HALF) return;
    const unsigned normal_offset = field_pos - C5VRX_CVBS_NORMAL_START_HALF;
    const bool first_half = (normal_offset & 1u) == 0u;
    const unsigned y = (field_pos - C5VRX_CVBS_ACTIVE_START_HALF) / 2u;
    const unsigned start = first_half ? C5VRX_CVBS_ACTIVE_START : 0u;
    const unsigned end = first_half ? C5VRX_CVBS_HALF_LINE_SAMPLES
                                    : C5VRX_CVBS_ACTIVE_END - C5VRX_CVBS_HALF_LINE_SAMPLES;
    for (unsigned p = start; p < end; ++p) {
        const unsigned x = first_half ? p - C5VRX_CVBS_ACTIVE_START
                                      : (C5VRX_CVBS_HALF_LINE_SAMPLES - C5VRX_CVBS_ACTIVE_START) + p;
        if (s_active_display == C5VRX_CVBS_DISPLAY_SNOW)
            half[p] = snow_code();
        else if (s_active_display == C5VRX_CVBS_DISPLAY_TEST)
            half[p] = diagnostic_code(x, y);
        else
            half[p] = logo_code(x, y);
    }
}

static void build_next_chunk(uint8_t *dst)
{
    for (unsigned i = 0; i < C5VRX_CVBS_CHUNK_HALF_LINES; ++i) {
        /* Apply state only where a complete PAL frame begins. A request made
         * while a chunk is being built can never split logo/snow/test content
         * across an arbitrary scanline. */
        if (s_next_half_line == 0u)
            s_active_display = s_requested_display;
        const uint8_t *src = template_for_half_line(s_next_half_line);
        uint8_t *half = dst + i * C5VRX_CVBS_HALF_LINE_SAMPLES;
        memcpy(half,
               src,
               C5VRX_CVBS_HALF_LINE_SAMPLES);
        overlay_active_half(half, s_next_half_line);
        ++s_next_half_line;
        if (s_next_half_line == C5VRX_CVBS_FRAME_HALF_LINES) {
            s_next_half_line = 0;
            ++s_frame_counter;
        }
    }
}

static unsigned configured_pins(int pins[8])
{
    const int all[8] = {
        CONFIG_C5VRX_CVBS_D0_GPIO,
        CONFIG_C5VRX_CVBS_D1_GPIO,
        CONFIG_C5VRX_CVBS_D2_GPIO,
        CONFIG_C5VRX_CVBS_D3_GPIO,
        CONFIG_C5VRX_CVBS_D4_GPIO,
        CONFIG_C5VRX_CVBS_D5_GPIO,
        CONFIG_C5VRX_CVBS_D6_GPIO,
        CONFIG_C5VRX_CVBS_D7_GPIO,
    };
    for (unsigned i = 0; i < 8; ++i) {
        pins[i] = all[i];
    }
    return (unsigned)CONFIG_C5VRX_CVBS_DAC_BITS;
}

static bool pins_valid(void)
{
    int pins[8];
    const unsigned required = configured_pins(pins);

    for (unsigned i = 0; i < required; ++i) {
        if (!GPIO_IS_VALID_OUTPUT_GPIO(pins[i])) {
            return false;
        }
#if CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG
        /* ESP32-C5 native USB is fixed to GPIO13 D- and GPIO14 D+. */
        if (pins[i] == 13 || pins[i] == 14) {
            return false;
        }
#endif
        /* GPIO28 is the C5 boot strap and must stay high/floating at reset. */
        if (pins[i] == 28) {
            return false;
        }
        for (unsigned j = i + 1; j < required; ++j) {
            if (pins[i] == pins[j]) {
                return false;
            }
        }
    }
    return true;
}

static esp_err_t queue_loop_buffer(uint8_t *buffer)
{
    const parlio_transmit_config_t tx_cfg = {
        .idle_value = cvbs_code_from_mv(300),
        .bitscrambler_program = NULL,
        .flags = {
            .queue_nonblocking = 0,
            .loop_transmission = 1,
        },
    };

    return parlio_tx_unit_transmit(
        s_tx,
        buffer,
        C5VRX_CVBS_CHUNK_SAMPLES * 8u,
        &tx_cfg);
}

static bool IRAM_ATTR on_buffer_switched(
    parlio_tx_unit_handle_t tx_unit,
    const parlio_tx_buffer_switched_event_data_t *edata,
    void *user_ctx)
{
    (void)tx_unit;
    c5vrx_cvbs_stream_state_t *state =
        (c5vrx_cvbs_stream_state_t *)user_ctx;

    if (!state || !state->task || !edata) {
        return false;
    }

    uint32_t bit = 0;
    if (edata->old_buffer_addr == s_chunk[0]) {
        bit = C5VRX_CVBS_RETIRED_BUF0;
    } else if (edata->old_buffer_addr == s_chunk[1]) {
        bit = C5VRX_CVBS_RETIRED_BUF1;
    } else {
        return false;
    }

    BaseType_t high_task_wakeup = pdFALSE;
    xTaskNotifyFromISR(
        state->task,
        bit,
        eSetBits,
        &high_task_wakeup);
    return high_task_wakeup == pdTRUE;
}

static void stream_task(void *arg)
{
    (void)arg;

    for (;;) {
        uint32_t notification = 0;
        (void)xTaskNotifyWait(
            0,
            UINT32_MAX,
            &notification,
            portMAX_DELAY);

        if (s_stream.stop_requested ||
            (notification & C5VRX_CVBS_STOP_NOTIFY)) {
            break;
        }

        for (unsigned index = 0; index < 2; ++index) {
            const uint32_t bit =
                index == 0 ? C5VRX_CVBS_RETIRED_BUF0
                           : C5VRX_CVBS_RETIRED_BUF1;
            if (!(notification & bit)) {
                continue;
            }

            build_next_chunk(s_chunk[index]);

            if (s_stream.stop_requested) {
                break;
            }

            const esp_err_t err = queue_loop_buffer(s_chunk[index]);
            if (err != ESP_OK) {
                if (!s_stream.stop_requested) {
                    ESP_LOGE(TAG,
                             "PAL stream buffer queue failed: %s",
                             esp_err_to_name(err));
                }
                s_stream.stop_requested = true;
                break;
            }
        }

        if (s_stream.stop_requested) {
            break;
        }
    }

    s_stream.task = NULL;
    vTaskDelete(NULL);
}

static void stop_stream_task(void)
{
    TaskHandle_t task = s_stream.task;
    if (!task) {
        return;
    }

    s_stream.stop_requested = true;
    (void)xTaskNotify(task, C5VRX_CVBS_STOP_NOTIFY, eSetBits);

    for (unsigned i = 0; i < 100 && s_stream.task; ++i) {
        vTaskDelay(pdMS_TO_TICKS(1));
    }

    if (s_stream.task) {
        ESP_LOGW(TAG, "PAL stream task did not exit cleanly; deleting it");
        vTaskDelete(s_stream.task);
        s_stream.task = NULL;
    }
}

static esp_err_t start_output(c5vrx_cvbs_display_t initial_display)
{
    if (s_running) {
        s_requested_display = initial_display;
        return ESP_OK;
    }
    if (!pins_valid()) {
        ESP_LOGE(TAG,
                 "CVBS PARLIO requires unique valid output GPIOs D0..D%u; GPIO13/14 (USB) and GPIO28 (BOOT) are reserved",
                 (unsigned)CONFIG_C5VRX_CVBS_DAC_BITS - 1u);
        return ESP_ERR_INVALID_ARG;
    }

    for (unsigned i = 0; i < 2; ++i) {
        s_chunk[i] = heap_caps_malloc(C5VRX_CVBS_CHUNK_SAMPLES,
                                      MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);
        if (!s_chunk[i]) {
            while (i) free(s_chunk[--i]);
            return ESP_ERR_NO_MEM;
        }
    }

    build_templates();
    s_next_half_line = 0;
    s_frame_counter = 0;
    s_requested_display = initial_display;
    s_active_display = initial_display;
    s_snow_lfsr = 0xc5f0a17du;
    build_next_chunk(s_chunk[0]);
    build_next_chunk(s_chunk[1]);

    const parlio_tx_unit_config_t cfg = {
        .clk_src = PARLIO_CLK_SRC_DEFAULT,
        .clk_in_gpio_num = -1,
        .input_clk_src_freq_hz = 0,
        .output_clk_freq_hz = C5VRX_CVBS_SAMPLE_RATE_HZ,
        .data_width = 8,
        .data_gpio_nums = {
            CONFIG_C5VRX_CVBS_D0_GPIO,
            CONFIG_C5VRX_CVBS_D1_GPIO,
            CONFIG_C5VRX_CVBS_D2_GPIO,
            CONFIG_C5VRX_CVBS_D3_GPIO,
            CONFIG_C5VRX_CVBS_D4_GPIO,
            CONFIG_C5VRX_CVBS_D5_GPIO,
            CONFIG_C5VRX_CVBS_D6_GPIO,
            CONFIG_C5VRX_CVBS_D7_GPIO,
        },
        .clk_out_gpio_num = -1,
        .valid_gpio_num = -1,
        .valid_start_delay = 0,
        .valid_stop_delay = 0,
        .trans_queue_depth = 2,
        .max_transfer_size = C5VRX_CVBS_CHUNK_SAMPLES,
        .dma_burst_size = 32,
        .shift_edge = PARLIO_SHIFT_EDGE_NEG,
        .bit_pack_order = PARLIO_BIT_PACK_ORDER_LSB,
    };

    esp_err_t err = parlio_new_tx_unit(&cfg, &s_tx);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "parlio_new_tx_unit failed: %s", esp_err_to_name(err));
        s_tx = NULL;
        free(s_chunk[0]);
        free(s_chunk[1]);
        s_chunk[0] = s_chunk[1] = NULL;
        return err;
    }

    s_stream.stop_requested = false;
    s_stream.task = NULL;
    TaskHandle_t stream_handle = NULL;
    if (xTaskCreate(
            stream_task,
            "c5vrx_pal",
            C5VRX_CVBS_STREAM_STACK,
            NULL,
            C5VRX_CVBS_STREAM_PRIORITY,
            &stream_handle) != pdPASS) {
        parlio_del_tx_unit(s_tx);
        s_tx = NULL;
        free(s_chunk[0]);
        free(s_chunk[1]);
        s_chunk[0] = s_chunk[1] = NULL;
        return ESP_ERR_NO_MEM;
    }
    s_stream.task = stream_handle;

    const parlio_tx_event_callbacks_t callbacks = {
        .on_trans_done = NULL,
        .on_buffer_switched = on_buffer_switched,
    };
    err = parlio_tx_unit_register_event_callbacks(
        s_tx,
        &callbacks,
        &s_stream);
    if (err != ESP_OK) {
        ESP_LOGE(TAG,
                 "parlio callback registration failed: %s",
                 esp_err_to_name(err));
        stop_stream_task();
        parlio_del_tx_unit(s_tx);
        s_tx = NULL;
        free(s_chunk[0]);
        free(s_chunk[1]);
        s_chunk[0] = s_chunk[1] = NULL;
        return err;
    }

    err = parlio_tx_unit_enable(s_tx);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "parlio_tx_unit_enable failed: %s", esp_err_to_name(err));
        stop_stream_task();
        parlio_del_tx_unit(s_tx);
        s_tx = NULL;
        free(s_chunk[0]);
        free(s_chunk[1]);
        s_chunk[0] = s_chunk[1] = NULL;
        return err;
    }

    err = queue_loop_buffer(s_chunk[0]);
    if (err == ESP_OK) {
        err = queue_loop_buffer(s_chunk[1]);
    }
    if (err != ESP_OK) {
        ESP_LOGE(TAG,
                 "initial PAL stream queue failed: %s",
                 esp_err_to_name(err));
        (void)parlio_tx_unit_disable(s_tx);
        stop_stream_task();
        parlio_del_tx_unit(s_tx);
        s_tx = NULL;
        free(s_chunk[0]);
        free(s_chunk[1]);
        s_chunk[0] = s_chunk[1] = NULL;
        return err;
    }

    s_running = true;
    ESP_LOGW(TAG,
             "PAL 625/50 always-on output active: display=%s %u-bit DAC, 20 MS/s, 2x%u-byte DMA chunks",
             c5vrx_cvbs_display_name(initial_display),
             (unsigned)CONFIG_C5VRX_CVBS_DAC_BITS,
             (unsigned)C5VRX_CVBS_CHUNK_SAMPLES);
#if CONFIG_C5VRX_CVBS_TEST_COLOR_BURST
    ESP_LOGW(TAG,
             "Output is monochrome; 4.43361875 MHz swinging burst is enabled for display lock and analog bandwidth validation");
#else
    ESP_LOGW(TAG, "Output is monochrome with color burst disabled");
#endif
    ESP_LOGW(TAG,
             "Use a correctly scaled resistor network into a known 75-ohm input; never connect raw 3.3 V GPIOs directly");
    return ESP_OK;
}

esp_err_t c5vrx_cvbs_output_start(void)
{
    return start_output(C5VRX_CVBS_DISPLAY_LOGO);
}

esp_err_t c5vrx_cvbs_output_set_display(c5vrx_cvbs_display_t display)
{
    if (display > C5VRX_CVBS_DISPLAY_TEST) return ESP_ERR_INVALID_ARG;
    if (!s_running) return ESP_ERR_INVALID_STATE;
    s_requested_display = display;
    return ESP_OK;
}

c5vrx_cvbs_display_t c5vrx_cvbs_output_display(void)
{
    return s_active_display;
}

const char *c5vrx_cvbs_display_name(c5vrx_cvbs_display_t display)
{
    switch (display) {
        case C5VRX_CVBS_DISPLAY_LOGO: return "LOGO";
        case C5VRX_CVBS_DISPLAY_SNOW: return "SNOW";
        case C5VRX_CVBS_DISPLAY_TEST: return "TEST";
        default: return "UNKNOWN";
    }
}

esp_err_t c5vrx_cvbs_test_start(void)
{
    return start_output(C5VRX_CVBS_DISPLAY_TEST);
}

esp_err_t c5vrx_cvbs_test_stop(void)
{
    if (!s_running || !s_tx) {
        return ESP_ERR_INVALID_STATE;
    }
    s_requested_display = C5VRX_CVBS_DISPLAY_LOGO;
    ESP_LOGI(TAG, "PAL diagnostics stopped; permanent logo output retained");
    return ESP_OK;
}

bool c5vrx_cvbs_test_running(void)
{
    return s_running;
}

#else

esp_err_t c5vrx_cvbs_test_start(void)
{
    return ESP_ERR_NOT_SUPPORTED;
}

esp_err_t c5vrx_cvbs_output_start(void) { return ESP_ERR_NOT_SUPPORTED; }
esp_err_t c5vrx_cvbs_output_set_display(c5vrx_cvbs_display_t display)
{ (void)display; return ESP_ERR_NOT_SUPPORTED; }
c5vrx_cvbs_display_t c5vrx_cvbs_output_display(void)
{ return C5VRX_CVBS_DISPLAY_LOGO; }
const char *c5vrx_cvbs_display_name(c5vrx_cvbs_display_t display)
{ (void)display; return "UNAVAILABLE"; }

esp_err_t c5vrx_cvbs_test_stop(void)
{
    return ESP_ERR_NOT_SUPPORTED;
}

bool c5vrx_cvbs_test_running(void)
{
    return false;
}

#endif
