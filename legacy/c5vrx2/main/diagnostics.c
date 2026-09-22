#include "diagnostics.h"
#include "trajectory_reference.h"
#include "true40_reference.h"

#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "driver/gpio.h"
#include "driver/bitscrambler.h"
#include "driver/bitscrambler_loopback.h"
#include "driver/parlio_bitscrambler.h"
#include "driver/parlio_rx.h"
#include "driver/parlio_tx.h"
#include "parlio_priv.h"
#undef TAG
#include "esp_async_memcpy.h"
#include "esp_attr.h"
#include "esp_cpu.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_partition.h"
#include "esp_rom_gpio.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "soc/gpio_reg.h"
#include "soc/gpio_sig_map.h"
#include "soc/gpio_ext_struct.h"
#include "soc/clk_tree_defs.h"
#include "soc/hp_apm_reg.h"
#include "soc/parl_io_struct.h"
#include "soc/pcr_struct.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "continuous_iq.h"
#include "calibration.h"
#include "rf_dump.h"
#include "startup_trace.h"
#include "wifi5.h"
#include "wbfm_q4.h"

#define REG32(a) (*(volatile uint32_t *)(uintptr_t)(a))
#define DUMP_CTRL     0x600a9004u
#define DUMP_PTR_MODE 0x600a9008u
#define HP_SRAM_USAGE 0x60095004u
#define PTR_MASK      0x00003fffu

#define RF_WRAP_SOAK_TARGET 10000u
#define RF_PHASE_WINDOWS      256u
#define RF_DMA_PROBE_BYTES    256u
#define RF_DMA_TIMEOUT_US    2000u
#define RF_BANK_A      0x40830000u
#define RF_BANK_B      0x40840000u
#define RF_BANK_WORDS       16384u
#define RF_BANK_A_SEED 0xa53c0000u
#define RF_BANK_B_SEED 0x5ac30000u

#define MODEM_WIDGET_DIAG_FIX      0x600a9404u
#define MODEM_WIDGET_DIAG_EXCHANGE 0x600a9408u
#define MODEM_SYSCON_TEST_CONF      0x600a9c00u
#define MODEM_FPGA_DEBUG_CLK80      (1u << 4)
#define MODEM_FPGA_DEBUG_CLKSWITCH  (1u << 3)
#define MODEM_DIAG_SAMPLES            8192u
#define MODEM_DIAG_LANES                 6u
#define MODEM_DIAG_CONFIGS               4u
#define MODEM_CAPTURE_WORDS            4096u
#define MODEM_CAPTURE_LANES                8u
#define MODEM_PARLIO_BYTES             8192u
#define MODEM_PARLIO_SAMPLE_RATE_HZ   40000000u
#define MODEM_PARLIO_CAPTURE_US          350u
#define MODEM_PARLIO_CLOCK_GPIO       GPIO_NUM_2
#define MODEM_CAPTURE_MAGIC       0x5043444du
#define MODEM_CAPTURE_SUBTYPE ((esp_partition_subtype_t)0x42)
#if CONFIG_C5VRX2_MODE_MODEM_WBFM
#define MODEM_PARLIO_WBFM_ENABLED 1u
#else
#define MODEM_PARLIO_WBFM_ENABLED 0u
#endif

#define PAL_RATE_HZ          20000000u
#define PAL_HALF_SAMPLES     640u
#define PAL_FRAME_HALVES     1250u
#define PAL_FIELD_HALVES     625u
#define PAL_CHUNK_HALVES     40u
#define PAL_CHUNK_SAMPLES    (PAL_HALF_SAMPLES * PAL_CHUNK_HALVES)
#define PAL_HSYNC            94u
#define PAL_EQ               47u
#define PAL_BROAD            546u
#define PAL_ACTIVE_START     210u
#define PAL_ACTIVE_END       1250u
#define PAL_SYNC_CODE        0u
#define PAL_BLACK_CODE       18u
#define PAL_WHITE_CODE       62u

extern void adctrig(int32_t smp_num_aft_trig, int32_t trigmode,
                    int32_t trigcase, int32_t sample_80m,
                    int32_t dump_trig, int32_t rx_gain_mode,
                    int32_t rx_gain, int32_t rx_gain0,
                    int32_t rx_gain0_wait_us);
extern void set_dump_mode(int mode);

static const char *TAG = "c5vrx2_diag";
static parlio_tx_unit_handle_t s_diag_tx;
static uint8_t *s_pal_buffers[2];
static TaskHandle_t s_pal_task;
static uint16_t s_pal_next_half;
static bool s_pal_colour;
static int8_t s_sine[256];

static const gpio_num_t s_modem_diag_pins[MODEM_DIAG_LANES] = {
    GPIO_NUM_23, GPIO_NUM_24, GPIO_NUM_11,
    GPIO_NUM_12, GPIO_NUM_8, GPIO_NUM_9,
};

/* Candidate Q4/I4 bus on the nine XIAO pads left beside the six DAC pins.
 * GPIO2 is deliberately kept free for the later source-synchronous clock
 * probe. No DAC pin, USB pin, flash/PSRAM pin, LED, or battery pin is used. */
static const gpio_num_t s_modem_capture_pins[MODEM_CAPTURE_LANES] = {
    GPIO_NUM_1, GPIO_NUM_0, GPIO_NUM_25, GPIO_NUM_7,
    GPIO_NUM_10, GPIO_NUM_5, GPIO_NUM_3, GPIO_NUM_4,
};
static const uint8_t s_modem_capture_signals[MODEM_CAPTURE_LANES] = {
    6u, 7u, 8u, 9u, 16u, 17u, 18u, 19u,
};
static DRAM_ATTR uint32_t s_modem_capture[MODEM_CAPTURE_WORDS];
static DRAM_ATTR uint8_t s_modem_parlio_capture[MODEM_PARLIO_BYTES]
    __attribute__((aligned(64)));

#define TX_WBFM_INPUT_BYTES  8192u
#define TX_WBFM_OUTPUT_BYTES 4000u
#define TX_WBFM_PACKED_BYTES (TX_WBFM_OUTPUT_BYTES / 2u)
#define TX_WBFM_MAGIC        0x31584254u /* little-endian "TBX1" */
#define TX_WBFM_VALID_GPIO   GPIO_NUM_23
#if CONFIG_C5VRX2_WBFM_TRUE40
#define TX_WBFM_TEST_VARIANT 17 /* True 40 MS/s adjacent candidate (Issue 17) */
#elif CONFIG_C5VRX2_WBFM_TRAJECTORY
#define TX_WBFM_TEST_VARIANT 8 /* middle-sample trajectory candidate */
#elif CONFIG_C5VRX2_WBFM_PHASE5_QUALITY
#define TX_WBFM_TEST_VARIANT 5 /* embedded uniform phase5 production core */
#else
#define TX_WBFM_TEST_VARIANT 4 /* physically proven Q3/I2 fallback */
#endif
#define TX_WBFM_CONSTANT_LUT_ORACLE 0
BITSCRAMBLER_PROGRAM(c5vrx2_tx_2to1_probe_program,
                    "c5vrx2_tx_2to1_probe");
BITSCRAMBLER_PROGRAM(c5vrx2_phase5_lookup_probe_program,
                    "c5vrx2_phase5_lookup_probe");
static DMA_ATTR __attribute__((aligned(64))) uint8_t
    s_tx_wbfm_input[TX_WBFM_INPUT_BYTES];
static DMA_ATTR __attribute__((aligned(64))) uint8_t
    s_tx_wbfm_actual[TX_WBFM_OUTPUT_BYTES];
static DMA_ATTR __attribute__((aligned(64))) uint8_t
    s_tx_wbfm_packed[TX_WBFM_PACKED_BYTES];
static DRAM_ATTR uint8_t s_tx_wbfm_expected[TX_WBFM_OUTPUT_BYTES];

static uint32_t fnv1a_hash(const void *data, size_t bytes);

typedef struct __attribute__((packed)) {
    uint32_t magic;
    uint32_t version;
    uint32_t header_bytes;
    uint32_t input_bytes;
    uint32_t output_bytes;
    int32_t setup_error;
    int32_t rx_error;
    int32_t tx_error;
    uint32_t elapsed_cycles;
    uint32_t cpu_hz;
    uint32_t parlio_int_raw;
    uint32_t parlio_tx_status;
    uint32_t parlio_rx_status;
    uint32_t best_offset;
    uint32_t compared;
    uint32_t mismatches;
    uint32_t first_mismatch;
    uint32_t actual_hash;
    uint32_t expected_hash;
    uint32_t loop_error;
    uint32_t loop_written;
    uint32_t loop_mismatches;
    uint32_t loop_first_mismatch;
    uint32_t reserved[9];
} tx_wbfm_header_t;

_Static_assert(sizeof(tx_wbfm_header_t) == 128u,
               "TX WBFM diagnostic header size");

/* Granting the live dump bank to the modem makes flash-backed interrupt
 * handlers unsafe on this C5 revision. The capture proof keeps that window
 * below 5 ms and masks interrupts only from just before arm through stop.
 * This is diagnostic containment, not the eventual realtime architecture. */
static inline IRAM_ATTR uint32_t modem_capture_irq_save_disable(void)
{
    uint32_t previous;
    const uint32_t machine_interrupt_enable = 0x8u;
    __asm__ __volatile__("csrrc %0, mstatus, %1"
                         : "=r"(previous)
                         : "r"(machine_interrupt_enable)
                         : "memory");
    return previous;
}

static inline IRAM_ATTR void modem_capture_irq_restore(uint32_t previous)
{
    if ((previous & 0x8u) != 0u) {
        const uint32_t machine_interrupt_enable = 0x8u;
        __asm__ __volatile__("csrs mstatus, %0"
                             :
                             : "r"(machine_interrupt_enable)
                             : "memory");
    }
}

#define MODEM_CAPTURE_PIN_MASK \
    ((1u << 1) | (1u << 0) | (1u << 25) | (1u << 7) | \
     (1u << 10) | (1u << 5) | (1u << 3) | (1u << 4))
#define DAC_PIN_MASK \
    ((1u << 23) | (1u << 24) | (1u << 11) | (1u << 12) | \
     (1u << 8) | (1u << 9))
#define USB_PIN_MASK ((1u << 13) | (1u << 14))
_Static_assert((MODEM_CAPTURE_PIN_MASK & DAC_PIN_MASK) == 0u,
               "modem capture overlaps six-bit DAC");
_Static_assert((MODEM_CAPTURE_PIN_MASK & USB_PIN_MASK) == 0u,
               "modem capture overlaps native USB");
_Static_assert((MODEM_CAPTURE_PIN_MASK & (1u << 2)) == 0u,
               "GPIO2 must remain free for the source clock");

typedef struct __attribute__((packed)) {
    uint32_t magic;
    uint32_t version;
    uint32_t header_bytes;
    uint32_t raw_words;
    uint32_t ring_words;
    uint32_t diag_fix;
    uint32_t diag_exchange;
    uint32_t gpio_mask;
    uint32_t sample_us;
    uint32_t rf_rate_hz;
    uint32_t writer_pointer;
    uint32_t dump_control;
    uint32_t dump_ptr_mode;
    uint32_t producer_starts;
    uint32_t physical_wraps;
    uint32_t trigger_count;
    uint32_t raw_hash;
    uint32_t ring_hash;
    uint32_t capture_end_pointer;
    uint32_t capture_end_ptr_mode;
    uint32_t stop_us;
    uint8_t gpio_for_diag[20];
    uint8_t reserved[24];
} modem_capture_header_t;

_Static_assert(sizeof(modem_capture_header_t) == 128u,
               "modem capture header size");

static esp_err_t configure_dac_tx(uint32_t rate, size_t max_transfer,
                                  unsigned queue_depth)
{
    const int pins[6] = {23, 24, 11, 12, 8, 9};
    for (unsigned i = 0; i < 6u; ++i) {
        esp_err_t err = gpio_set_drive_capability((gpio_num_t)pins[i],
                                                  GPIO_DRIVE_CAP_3);
        if (err != ESP_OK) return err;
    }
    const parlio_tx_unit_config_t cfg = {
        .clk_src = PARLIO_CLK_SRC_DEFAULT,
        .clk_in_gpio_num = -1,
        .input_clk_src_freq_hz = 0,
        .output_clk_freq_hz = rate,
        .data_width = 8,
        .data_gpio_nums = {23, 24, 11, 12, 8, 9, -1, -1},
        .clk_out_gpio_num = -1,
        .valid_gpio_num = -1,
        .trans_queue_depth = queue_depth,
        .max_transfer_size = max_transfer,
        .dma_burst_size = 32,
        .shift_edge = PARLIO_SHIFT_EDGE_NEG,
        .bit_pack_order = PARLIO_BIT_PACK_ORDER_LSB,
    };
    esp_err_t err = parlio_new_tx_unit(&cfg, &s_diag_tx);
    if (err == ESP_OK) err = parlio_tx_unit_enable(s_diag_tx);
    return err;
}

static esp_err_t transmit_loop(const void *buffer, size_t bytes,
                               unsigned idle)
{
    const parlio_transmit_config_t cfg = {
        .idle_value = idle,
        .bitscrambler_program = NULL,
        .flags.loop_transmission = true,
    };
    return parlio_tx_unit_transmit(s_diag_tx, buffer, bytes * 8u, &cfg);
}

esp_err_t c5vrx2_av_static_diagnostic_start(unsigned code)
{
    if (code > 63u) return ESP_ERR_INVALID_ARG;
    static uint8_t codes[256] __attribute__((aligned(64)));
    memset(codes, (int)code, sizeof(codes));
    esp_err_t err = configure_dac_tx(1000000u, sizeof(codes), 1u);
    if (err == ESP_OK) err = transmit_loop(codes, sizeof(codes), code);
    if (err == ESP_OK)
        ESP_LOGW(TAG, "AV STATIC code=%u (test 0,18,31,32,62,63)", code);
    return err;
}

static uint8_t clamp_code(int value)
{
    if (value < 0) return 0u;
    if (value > 63) return 63u;
    return (uint8_t)value;
}

static void render_pal_half(uint8_t *dst, uint16_t half)
{
    const unsigned field_half = half % PAL_FIELD_HALVES;
    const bool first_half = field_half >= 15u && ((field_half - 15u) & 1u) == 0u;
    const unsigned line_number = field_half >= 15u ? (field_half - 15u) / 2u : 0u;
    memset(dst, PAL_BLACK_CODE, PAL_HALF_SAMPLES);

    unsigned pulse = 0u;
    if (field_half < 5u || (field_half >= 10u && field_half < 15u))
        pulse = PAL_EQ;
    else if (field_half < 10u)
        pulse = PAL_BROAD;
    else if (first_half)
        pulse = PAL_HSYNC;
    if (pulse != 0u) memset(dst, PAL_SYNC_CODE, pulse);
    if (field_half < 15u) return;

    const unsigned base = first_half ? 0u : PAL_HALF_SAMPLES;
    if (s_pal_colour && first_half) {
        /* NCO uses the actual 20 MHz output rate; no integer samples/cycle
         * assumption is made for the 4.43361875 MHz PAL burst. */
        uint32_t phase = (line_number & 1u) ? 0x40000000u : 0xc0000000u;
        const uint32_t step = (uint32_t)(4433618.75 * 4294967296.0 /
                                         (double)PAL_RATE_HZ);
        for (unsigned x = 115u; x < 165u; ++x) {
            dst[x] = clamp_code(PAL_BLACK_CODE +
                                (s_sine[phase >> 24u] * 5) / 127);
            phase += step;
        }
    }

    if (line_number < 24u || line_number >= 312u) return;
    for (unsigned p = 0; p < PAL_HALF_SAMPLES; ++p) {
        const unsigned line_pos = base + p;
        if (line_pos < PAL_ACTIVE_START || line_pos >= PAL_ACTIVE_END) continue;
        const unsigned x = line_pos - PAL_ACTIVE_START;
        const unsigned bar = x * 8u / (PAL_ACTIVE_END - PAL_ACTIVE_START);
        static const uint8_t luma[8] = {55, 50, 44, 39, 34, 29, 23, 18};
        int code = luma[bar];
        if (s_pal_colour) {
            static const int8_t chroma[8] = {0, 7, -7, 9, -9, 6, -6, 0};
            const uint64_t phase = (uint64_t)line_pos * 443361875ull *
                                   4294967296ull / (PAL_RATE_HZ * 100ull);
            code += (chroma[bar] * s_sine[(uint32_t)phase >> 24u]) / 127;
        }
        dst[p] = clamp_code(code);
    }
}

static void render_pal_chunk(uint8_t *buffer)
{
    for (unsigned i = 0; i < PAL_CHUNK_HALVES; ++i) {
        render_pal_half(buffer + i * PAL_HALF_SAMPLES, s_pal_next_half);
        s_pal_next_half++;
        if (s_pal_next_half == PAL_FRAME_HALVES) s_pal_next_half = 0u;
    }
}

static bool pal_buffer_switched(parlio_tx_unit_handle_t tx,
    const parlio_tx_buffer_switched_event_data_t *event, void *context)
{
    (void)tx;
    (void)context;
    BaseType_t wake = pdFALSE;
    unsigned index = event && event->old_buffer_addr == s_pal_buffers[1] ? 1u : 0u;
    if (s_pal_task)
        xTaskNotifyFromISR(s_pal_task, 1u << index, eSetBits, &wake);
    return wake == pdTRUE;
}

static void pal_refill_task(void *argument)
{
    (void)argument;
    for (;;) {
        uint32_t bits = 0u;
        (void)xTaskNotifyWait(0u, UINT32_MAX, &bits, portMAX_DELAY);
        for (unsigned i = 0; i < 2u; ++i) {
            if ((bits & (1u << i)) == 0u) continue;
            render_pal_chunk(s_pal_buffers[i]);
            if (transmit_loop(s_pal_buffers[i], PAL_CHUNK_SAMPLES,
                              PAL_BLACK_CODE) != ESP_OK) {
                ESP_LOGE(TAG, "PAL diagnostic DMA refill failed");
                vTaskDelete(NULL);
            }
        }
    }
}

esp_err_t c5vrx2_av_pal_diagnostic_start(bool colour)
{
    s_pal_colour = colour;
    for (unsigned i = 0; i < 256u; ++i)
        s_sine[i] = (int8_t)lrintf(127.0f * sinf((float)i *
                                                2.0f * 3.14159265f / 256.0f));
    for (unsigned i = 0; i < 2u; ++i) {
        s_pal_buffers[i] = heap_caps_malloc(PAL_CHUNK_SAMPLES,
            MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);
        if (!s_pal_buffers[i]) return ESP_ERR_NO_MEM;
        render_pal_chunk(s_pal_buffers[i]);
    }
    esp_err_t err = configure_dac_tx(PAL_RATE_HZ, PAL_CHUNK_SAMPLES, 2u);
    if (err != ESP_OK) return err;
    if (xTaskCreate(pal_refill_task, "pal_diag", 3072u, NULL, 20u,
                    &s_pal_task) != pdPASS) return ESP_ERR_NO_MEM;
    const parlio_tx_event_callbacks_t callbacks = {
        .on_buffer_switched = pal_buffer_switched,
    };
    if ((err = parlio_tx_unit_register_event_callbacks(
             s_diag_tx, &callbacks, NULL)) != ESP_OK) return err;
    if ((err = transmit_loop(s_pal_buffers[0], PAL_CHUNK_SAMPLES,
                             PAL_BLACK_CODE)) != ESP_OK) return err;
    if ((err = transmit_loop(s_pal_buffers[1], PAL_CHUNK_SAMPLES,
                             PAL_BLACK_CODE)) != ESP_OK) return err;
    ESP_LOGW(TAG, "AV synthetic PAL %s: 20 MHz, continuous double-buffer DMA",
             colour ? "colour bars + burst" : "monochrome bars");
    return ESP_OK;
}

esp_err_t c5vrx2_tx_wbfm_diagnostic_run(void)
{
    for (size_t i = 0u; i < sizeof(s_tx_wbfm_input); ++i) {
        s_tx_wbfm_input[i] =
            (uint8_t)(i * 73u + (i >> 3u) * 29u + (i >> 7u) * 11u + 17u);
    }
    size_t expected_bytes = sizeof(s_tx_wbfm_expected);
#if TX_WBFM_TEST_VARIANT == 1
    for (size_t i = 0u; i < expected_bytes; ++i)
        s_tx_wbfm_expected[i] = s_tx_wbfm_input[i * 2u + 1u];
#elif TX_WBFM_TEST_VARIANT == 2
    expected_bytes = c5vrx2_wbfm_q4_fast_reference(
        s_tx_wbfm_input, sizeof(s_tx_wbfm_input), s_tx_wbfm_expected,
        sizeof(s_tx_wbfm_expected));
#elif TX_WBFM_TEST_VARIANT == 3
    expected_bytes = c5vrx2_wbfm_q4_lut3_reference(
        s_tx_wbfm_input, sizeof(s_tx_wbfm_input), s_tx_wbfm_expected,
        sizeof(s_tx_wbfm_expected));
#elif TX_WBFM_TEST_VARIANT == 4
#if TX_WBFM_CONSTANT_LUT_ORACLE
    memset(s_tx_wbfm_expected, 21, expected_bytes);
#else
    expected_bytes = c5vrx2_wbfm_q4_iq5_reference(
        s_tx_wbfm_input, sizeof(s_tx_wbfm_input), s_tx_wbfm_expected,
        sizeof(s_tx_wbfm_expected));
#endif
#elif TX_WBFM_TEST_VARIANT == 17
    uint8_t previous_iq5 = 0;
    expected_bytes = c5vrx2_true40_reference(
        s_tx_wbfm_input, sizeof(s_tx_wbfm_input), s_tx_wbfm_expected,
        sizeof(s_tx_wbfm_expected), &previous_iq5);
#elif TX_WBFM_TEST_VARIANT == 8
    uint8_t previous_phase = 0;
    expected_bytes = c5vrx2_trajectory_reference(
        s_tx_wbfm_input, sizeof(s_tx_wbfm_input), s_tx_wbfm_expected,
        sizeof(s_tx_wbfm_expected), &previous_phase);
#elif TX_WBFM_TEST_VARIANT == 5
    expected_bytes = c5vrx2_wbfm_q4_phase5_reference(
        s_tx_wbfm_input, sizeof(s_tx_wbfm_input), s_tx_wbfm_expected,
        sizeof(s_tx_wbfm_expected));
#elif TX_WBFM_TEST_VARIANT == 6
    for (size_t pair = 0u; pair < expected_bytes; ++pair)
        s_tx_wbfm_expected[pair] = c5vrx2_wbfm_q4_phase5_value(
            s_tx_wbfm_input[pair * 2u + 1u]);
#elif TX_WBFM_TEST_VARIANT == 7
    for (size_t pair = 0u; pair < expected_bytes; ++pair)
        s_tx_wbfm_expected[pair] = s_tx_wbfm_input[pair * 2u + 1u] & 0x1fu;
#else
    expected_bytes = c5vrx2_wbfm_q4_reference(
        s_tx_wbfm_input, sizeof(s_tx_wbfm_input), s_tx_wbfm_expected,
        sizeof(s_tx_wbfm_expected));
#endif
    esp_err_t loop_error = ESP_ERR_NOT_SUPPORTED;
    size_t loop_written = 0u;
    uint32_t loop_mismatches = UINT32_MAX;
    uint32_t loop_first_mismatch = UINT32_MAX;
#if TX_WBFM_TEST_VARIANT == 3 || \
    (TX_WBFM_TEST_VARIANT == 4 && !TX_WBFM_CONSTANT_LUT_ORACLE) || \
    TX_WBFM_TEST_VARIANT == 5 || TX_WBFM_TEST_VARIANT == 8 || TX_WBFM_TEST_VARIANT == 17
    bitscrambler_handle_t loop_bs = NULL;
    loop_error = bitscrambler_loopback_create(
        &loop_bs, SOC_BITSCRAMBLER_ATTACH_I2S0, TX_WBFM_INPUT_BYTES);
    if (loop_error == ESP_OK)
        loop_error = TX_WBFM_TEST_VARIANT == 17 ?
                     c5vrx2_wbfm_q4_configure_true40(loop_bs) :
                     TX_WBFM_TEST_VARIANT == 8 ?
                     c5vrx2_wbfm_q4_configure_trajectory(loop_bs) :
                     TX_WBFM_TEST_VARIANT == 3 ?
                     c5vrx2_wbfm_q4_configure_lut3(loop_bs) :
                     (TX_WBFM_TEST_VARIANT == 5 ||
                      TX_WBFM_TEST_VARIANT == 6 ?
                      c5vrx2_wbfm_q4_configure_phase5(loop_bs) :
                      c5vrx2_wbfm_q4_configure_iq5(loop_bs));
    if (loop_error == ESP_OK)
        loop_error = bitscrambler_loopback_run(
            loop_bs, s_tx_wbfm_input, sizeof(s_tx_wbfm_input),
            s_tx_wbfm_actual, expected_bytes, &loop_written);
    if (loop_error == ESP_OK) {
        loop_mismatches = 0u;
        for (size_t i = 0u; i < expected_bytes; ++i) {
            if (s_tx_wbfm_actual[i] != s_tx_wbfm_expected[i]) {
                if (loop_first_mismatch == UINT32_MAX)
                    loop_first_mismatch = (uint32_t)i;
                loop_mismatches++;
            }
        }
    }
    if (loop_bs) bitscrambler_free(loop_bs);
#endif
    /* A level delimiter consumes one of PARLIO RX's eight physical input
     * lanes, so a full-width 8-bit receive cannot also use VALID.  This
     * bounded test captures bits 0..3 (or 4..7 in the high-bank build) on
     * every output clock; run both banks on the identical deterministic input;
     * the production TX remains eight-bit and drives all six DAC bits. */
    for (size_t i = 0u; i < expected_bytes; ++i)
        s_tx_wbfm_expected[i] =
#if CONFIG_C5VRX2_TX_ORACLE_HIGH_NIBBLE
            (s_tx_wbfm_expected[i] >> 4u) & 0x0fu;
#else
            s_tx_wbfm_expected[i] & 0x0fu;
#endif
    memset(s_tx_wbfm_actual, 0xa5, sizeof(s_tx_wbfm_actual));
    memset(s_tx_wbfm_packed, 0xa5, sizeof(s_tx_wbfm_packed));

    parlio_tx_unit_handle_t tx = NULL;
    parlio_rx_unit_handle_t rx = NULL;
    parlio_rx_delimiter_handle_t delimiter = NULL;
    bool decorated = false;
    bool tx_enabled = false;
    bool rx_enabled = false;
    esp_err_t setup_err = ESP_OK;
    esp_err_t rx_err = ESP_ERR_INVALID_STATE;
    esp_err_t tx_err = ESP_ERR_INVALID_STATE;
    uint32_t elapsed_cycles = 0u;
    uint32_t lut_pre_actual_hash = 0u;
    uint32_t lut_pre_expected_hash = 0u;
    uint32_t lut_pre_mismatches = UINT32_MAX;
    uint32_t lut_post_actual_hash = 0u;
    uint32_t lut_post_expected_hash = 0u;
    uint32_t lut_post_mismatches = UINT32_MAX;

    const parlio_tx_unit_config_t tx_cfg = {
        .clk_src = PARLIO_CLK_SRC_DEFAULT,
        .clk_in_gpio_num = -1,
        .input_clk_src_freq_hz = 0u,
        .output_clk_freq_hz = 20000000u,
        .data_width = 8u,
        .data_gpio_nums = {
            GPIO_NUM_1, GPIO_NUM_0, GPIO_NUM_25, GPIO_NUM_7,
            GPIO_NUM_10, GPIO_NUM_5, GPIO_NUM_3, GPIO_NUM_4,
        },
        .clk_out_gpio_num = MODEM_PARLIO_CLOCK_GPIO,
        .valid_gpio_num = TX_WBFM_VALID_GPIO,
        .valid_start_delay = 0,
        .valid_stop_delay = 0,
        .trans_queue_depth = 1u,
        .max_transfer_size = TX_WBFM_INPUT_BYTES,
        .dma_burst_size = 32u,
        .shift_edge = PARLIO_SHIFT_EDGE_NEG,
        .bit_pack_order = PARLIO_BIT_PACK_ORDER_LSB,
    };
    setup_err = parlio_new_tx_unit(&tx_cfg, &tx);
    if (setup_err != ESP_OK) goto persist;
    setup_err = parlio_tx_unit_decorate_bitscrambler(tx);
    if (setup_err != ESP_OK) goto cleanup;
    decorated = true;
    /* Configure through the decorator-owned TX handle. The phase5 and IQ5
     * production programs carry their LUT in the program image so the table
     * is installed by the same load that starts the bounded transaction. */
    setup_err = TX_WBFM_TEST_VARIANT == 17 ?
                c5vrx2_wbfm_q4_configure_true40(tx->bs_handle) :
                TX_WBFM_TEST_VARIANT == 8 ?
                c5vrx2_wbfm_q4_configure_trajectory(tx->bs_handle) :
                TX_WBFM_TEST_VARIANT == 3 ?
                c5vrx2_wbfm_q4_configure_lut3(tx->bs_handle) :
                (TX_WBFM_TEST_VARIANT == 5 ||
                 TX_WBFM_TEST_VARIANT == 6 ?
                 c5vrx2_wbfm_q4_configure_phase5(tx->bs_handle) :
                (TX_WBFM_TEST_VARIANT == 7 ?
                 bitscrambler_load_program(
                     tx->bs_handle, c5vrx2_phase5_lookup_probe_program) :
                (TX_WBFM_TEST_VARIANT == 4 ?
                 c5vrx2_wbfm_q4_configure_iq5(tx->bs_handle) :
                 c5vrx2_wbfm_q4_load_tx_lut())));
    if (setup_err != ESP_OK) goto cleanup;
    if (TX_WBFM_TEST_VARIANT == 4)
        lut_pre_mismatches = c5vrx2_wbfm_q4_verify_tx_iq5_lut(
            &lut_pre_actual_hash, &lut_pre_expected_hash);
#if TX_WBFM_TEST_VARIANT == 4 && TX_WBFM_CONSTANT_LUT_ORACLE
    /* Configure the program first, then overwrite the complete physical LUT.
     * configure_iq5() itself loads the production IQ table, so doing this in
     * the opposite order silently invalidates the constant-LUT oracle. */
    uint16_t *constant_lut = heap_caps_malloc(8192u, MALLOC_CAP_INTERNAL);
    if (!constant_lut) {
        setup_err = ESP_ERR_NO_MEM;
        goto cleanup;
    }
    for (size_t i = 0u; i < 8192u / sizeof(uint16_t); ++i)
        constant_lut[i] = 21u;
    setup_err = bitscrambler_load_lut(tx->bs_handle, constant_lut, 8192u);
    free(constant_lut);
    if (setup_err != ESP_OK) goto cleanup;
#endif

    const parlio_rx_unit_config_t rx_cfg = {
        .trans_queue_depth = 1u,
        .max_recv_size = TX_WBFM_PACKED_BYTES,
        .dma_burst_size = 32u,
        .data_width = 4u,
        .clk_src = PARLIO_CLK_SRC_DEFAULT,
        .ext_clk_freq_hz = 20000000u,
        .exp_clk_freq_hz = 20000000u,
        .clk_in_gpio_num = MODEM_PARLIO_CLOCK_GPIO,
        .clk_out_gpio_num = -1,
        .valid_gpio_num = TX_WBFM_VALID_GPIO,
        .data_gpio_nums = {
            #if CONFIG_C5VRX2_TX_ORACLE_HIGH_NIBBLE
            GPIO_NUM_10, GPIO_NUM_5, GPIO_NUM_3, GPIO_NUM_4,
#else
            GPIO_NUM_1, GPIO_NUM_0, GPIO_NUM_25, GPIO_NUM_7,
#endif
            -1, -1, -1, -1,
        },
        .flags = {
            .free_clk = false,
            .clk_gate_en = false,
            .allow_pd = false,
        },
    };
    setup_err = parlio_new_rx_unit(&rx_cfg, &rx);
    if (setup_err != ESP_OK) goto cleanup;
    const parlio_rx_level_delimiter_config_t delimiter_cfg = {
        .valid_sig_line_id = PARLIO_RX_UNIT_MAX_DATA_WIDTH - 1,
        .sample_edge = PARLIO_SAMPLE_EDGE_POS,
        .bit_pack_order = PARLIO_BIT_PACK_ORDER_LSB,
        .eof_data_len = TX_WBFM_PACKED_BYTES,
        .timeout_ticks = 0u,
        .flags.active_low_en = false,
    };
    setup_err = parlio_new_rx_level_delimiter(&delimiter_cfg, &delimiter);
    if (setup_err != ESP_OK) goto cleanup;
    setup_err = parlio_rx_unit_enable(rx, true);
    if (setup_err != ESP_OK) goto cleanup;
    rx_enabled = true;
    setup_err = parlio_tx_unit_enable(tx);
    if (setup_err != ESP_OK) goto cleanup;
    tx_enabled = true;

    const parlio_receive_config_t receive_cfg = {
        .delimiter = delimiter,
        .flags = {
            .partial_rx_en = false,
            .indirect_mount = false,
        },
    };
    setup_err = parlio_rx_unit_receive(rx, s_tx_wbfm_packed,
                                       sizeof(s_tx_wbfm_packed),
                                       &receive_cfg);
    if (setup_err != ESP_OK) goto cleanup;
    const parlio_transmit_config_t transmit_cfg = {
        .idle_value = c5vrx2_calibration_get()->pedestal_code,
        .bitscrambler_program =
#if TX_WBFM_TEST_VARIANT == 1
            c5vrx2_tx_2to1_probe_program,
#elif TX_WBFM_TEST_VARIANT == 2
            c5vrx2_wbfm_q4_fast_program(),
#elif TX_WBFM_TEST_VARIANT == 3
            c5vrx2_wbfm_q4_lut3_program(),
#elif TX_WBFM_TEST_VARIANT == 4
            c5vrx2_wbfm_q4_iq5_program(),
#elif TX_WBFM_TEST_VARIANT == 17
            c5vrx2_wbfm_q4_true40_program(),
#elif TX_WBFM_TEST_VARIANT == 8
            c5vrx2_wbfm_q4_trajectory_program(),
#elif TX_WBFM_TEST_VARIANT == 5
            c5vrx2_wbfm_q4_phase5_program(),
#elif TX_WBFM_TEST_VARIANT == 6 || TX_WBFM_TEST_VARIANT == 7
            c5vrx2_phase5_lookup_probe_program,
#else
            c5vrx2_wbfm_q4_program(),
#endif
        .flags.loop_transmission = false,
    };
    /* The shared VALID line starts RX on the first real output word.  Do not
     * force rx_sw_en: doing so captures the queue/start latency as zero-valued
     * samples and can finish before the transformed payload is observable. */
    const uint32_t begin = esp_cpu_get_cycle_count();
    setup_err = parlio_tx_unit_transmit(tx, s_tx_wbfm_input,
                                        sizeof(s_tx_wbfm_input) * 8u,
                                        &transmit_cfg);
    if (setup_err == ESP_OK) rx_err = parlio_rx_unit_wait_all_done(rx, 1000);
    if (setup_err == ESP_OK) tx_err = parlio_tx_unit_wait_all_done(tx, 1000);
    elapsed_cycles = esp_cpu_get_cycle_count() - begin;
    if (TX_WBFM_TEST_VARIANT == 4)
        lut_post_mismatches = c5vrx2_wbfm_q4_verify_tx_iq5_lut(
            &lut_post_actual_hash, &lut_post_expected_hash);

    if (rx_err == ESP_OK) {
        for (size_t i = 0u; i < sizeof(s_tx_wbfm_packed); ++i) {
            s_tx_wbfm_actual[i * 2u] = s_tx_wbfm_packed[i] & 0x0fu;
            s_tx_wbfm_actual[i * 2u + 1u] = s_tx_wbfm_packed[i] >> 4u;
        }
    }

cleanup:
    if (tx_enabled) (void)parlio_tx_unit_disable(tx);
    if (rx_enabled) (void)parlio_rx_unit_disable(rx);
    if (delimiter) (void)parlio_del_rx_delimiter(delimiter);
    if (rx) (void)parlio_del_rx_unit(rx);
    if (decorated) (void)parlio_tx_unit_undecorate_bitscrambler(tx);
    if (tx) (void)parlio_del_tx_unit(tx);

persist:
    uint32_t best_mismatches = UINT32_MAX;
    uint32_t best_offset = 0u;
    uint32_t best_compared = 0u;
    uint32_t best_first = UINT32_MAX;
    for (size_t offset = 0u; offset <= 16u && offset < expected_bytes;
         ++offset) {
        const size_t compared = expected_bytes - offset;
        uint32_t mismatches = 0u;
        uint32_t first = UINT32_MAX;
        for (size_t i = 0u; i < compared; ++i) {
            if (s_tx_wbfm_actual[i] != s_tx_wbfm_expected[offset + i]) {
                if (first == UINT32_MAX) first = (uint32_t)i;
                mismatches++;
            }
        }
        mismatches += (uint32_t)(expected_bytes - compared);
        if (mismatches < best_mismatches) {
            best_mismatches = mismatches;
            best_offset = (uint32_t)offset;
            best_compared = (uint32_t)compared;
            best_first = first;
        }
    }

    const tx_wbfm_header_t header = {
        .magic = TX_WBFM_MAGIC,
        .version = TX_WBFM_TEST_VARIANT == 8 ?
#if CONFIG_C5VRX2_TX_ORACLE_HIGH_NIBBLE
                   11u :
#else
                   10u :
#endif
                   TX_WBFM_TEST_VARIANT == 1 ? 3u :
                   (TX_WBFM_TEST_VARIANT == 2 ? 4u :
                   (TX_WBFM_TEST_VARIANT == 3 ? 5u :
                   (TX_WBFM_TEST_VARIANT == 4 ? 6u :
                    (TX_WBFM_TEST_VARIANT == 5 ? 7u :
                    (TX_WBFM_TEST_VARIANT == 6 ? 8u :
                     (TX_WBFM_TEST_VARIANT == 7 ? 9u : 2u)))))),
        .header_bytes = sizeof(tx_wbfm_header_t),
        .input_bytes = sizeof(s_tx_wbfm_input),
        .output_bytes = expected_bytes,
        .setup_error = setup_err,
        .rx_error = rx_err,
        .tx_error = tx_err,
        .elapsed_cycles = elapsed_cycles,
        .cpu_hz = CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ * 1000000u,
        .parlio_int_raw = PARL_IO.int_raw.val,
        .parlio_tx_status = PARL_IO.tx_st0.val,
        .parlio_rx_status = PARL_IO.rx_st0.val,
        .best_offset = best_offset,
        .compared = best_compared,
        .mismatches = best_mismatches,
        .first_mismatch = best_first,
        .actual_hash = fnv1a_hash(s_tx_wbfm_actual,
                                  sizeof(s_tx_wbfm_actual)),
        .expected_hash = fnv1a_hash(s_tx_wbfm_expected,
                                    sizeof(s_tx_wbfm_expected)),
        .loop_error = loop_error,
        .loop_written = (uint32_t)loop_written,
        .loop_mismatches = loop_mismatches,
        .loop_first_mismatch = loop_first_mismatch,
        .reserved = {
            lut_pre_mismatches, lut_pre_actual_hash, lut_pre_expected_hash,
            lut_post_mismatches, lut_post_actual_hash,
            lut_post_expected_hash,
        },
    };
    const esp_partition_t *partition = esp_partition_find_first(
        ESP_PARTITION_TYPE_DATA, MODEM_CAPTURE_SUBTYPE, "diagcap");
    esp_err_t persist_err = partition ? ESP_OK : ESP_ERR_NOT_FOUND;
    if (persist_err == ESP_OK)
        persist_err = esp_partition_erase_range(partition, 0u,
                                                partition->erase_size * 3u);
    if (persist_err == ESP_OK)
        persist_err = esp_partition_write(partition, sizeof(header),
                                          s_tx_wbfm_actual,
                                          sizeof(s_tx_wbfm_actual));
    if (persist_err == ESP_OK)
        persist_err = esp_partition_write(
            partition, sizeof(header) + sizeof(s_tx_wbfm_actual),
            s_tx_wbfm_expected, sizeof(s_tx_wbfm_expected));
    if (persist_err == ESP_OK)
        persist_err = esp_partition_write(partition, 0u, &header,
                                          sizeof(header));
    ESP_LOGW(TAG,
             "TX WBFM setup=%s rx=%s tx=%s mismatch=%u offset=%u "
             "cycles=%u int=%08x",
             esp_err_to_name(setup_err), esp_err_to_name(rx_err),
             esp_err_to_name(tx_err), (unsigned)best_mismatches,
             (unsigned)best_offset, (unsigned)elapsed_cycles,
             (unsigned)PARL_IO.int_raw.val);
    if (persist_err != ESP_OK) return persist_err;
    if (setup_err != ESP_OK) return setup_err;
    if (rx_err != ESP_OK) return rx_err;
    if (tx_err != ESP_OK) return tx_err;
    return best_mismatches == 0u ? ESP_OK : ESP_FAIL;
}

static void oracle_task(void *argument)
{
    (void)argument;
    set_dump_mode(0);
    adctrig(16383, 5, 0, 1, 1, 0, 0, 0, 0);
    vTaskDelete(NULL);
}

esp_err_t c5vrx2_rf_oracle_diagnostic_start(void)
{
    if (!c5vrx2_rf_dump_memory_reserved()) return ESP_ERR_INVALID_STATE;
    if (xTaskCreate(oracle_task, "rf_oracle", 4096u, NULL, 8u, NULL) != pdPASS)
        return ESP_ERR_NO_MEM;
    vTaskDelay(pdMS_TO_TICKS(2u));
    const int64_t begin = esp_timer_get_time();
    uint32_t last = REG32(DUMP_PTR_MODE) & PTR_MASK;
    uint32_t changes = 0u;
    uint32_t wraps = 0u;
    uint32_t ram_changes = 0u;
    uint32_t sample = REG32(C5VRX2_RF_DUMP_BASE);
    while (esp_timer_get_time() - begin < 250000) {
        const uint32_t pointer = REG32(DUMP_PTR_MODE) & PTR_MASK;
        if (pointer != last) changes++;
        if (pointer < last) wraps++;
        last = pointer;
        const uint32_t current = REG32(C5VRX2_RF_DUMP_BASE);
        if (current != sample) {
            ram_changes++;
            sample = current;
        }
    }
    ESP_LOGW(TAG,
             "RF ORACLE dump_first=1 tx_start=5 ctrl=0x%08x selector=0x%08x "
             "ptr=%u changes=%u wraps=%u ram_changes=%u",
             (unsigned)REG32(DUMP_CTRL), (unsigned)REG32(DUMP_PTR_MODE),
             (unsigned)last, (unsigned)changes, (unsigned)wraps,
             (unsigned)ram_changes);
    return changes && ram_changes ? ESP_OK : ESP_ERR_INVALID_RESPONSE;
}

static int16_t sign10(uint32_t value)
{
    value &= 0x3ffu;
    return (value & 0x200u) ? (int16_t)(value - 0x400u) : (int16_t)value;
}

static float phase_step(uint32_t previous, uint32_t current)
{
    const int pi = sign10(previous >> 10u);
    const int pq = sign10(previous);
    const int ci = sign10(current >> 10u);
    const int cq = sign10(current);
    const int32_t dot = ci * pi + cq * pq;
    const int32_t cross = cq * pi - ci * pq;
    return atan2f((float)cross, (float)dot);
}

static DMA_ATTR uint8_t s_rf_dma_a0[RF_DMA_PROBE_BYTES];
static DMA_ATTR uint8_t s_rf_dma_a1[RF_DMA_PROBE_BYTES];
static DMA_ATTR uint8_t s_rf_dma_b0[RF_DMA_PROBE_BYTES];
static DMA_ATTR uint8_t s_rf_dma_b1[RF_DMA_PROBE_BYTES];
static volatile bool s_rf_dma_done_a0;
static volatile bool s_rf_dma_done_a1;
static volatile bool s_rf_dma_done_b0;
static volatile bool s_rf_dma_done_b1;

static bool rf_dma_done(async_memcpy_handle_t handle,
                        async_memcpy_event_t *event, void *argument)
{
    (void)handle;
    (void)event;
    *(volatile bool *)argument = true;
    return false;
}

static esp_err_t rf_dma_copy_wait(async_memcpy_handle_t dma, void *destination,
                                  const void *source, volatile bool *done,
                                  uint32_t *elapsed_us)
{
    *done = false;
    const int64_t begin = esp_timer_get_time();
    esp_err_t err = esp_async_memcpy(dma, destination, (void *)source,
                                     RF_DMA_PROBE_BYTES, rf_dma_done,
                                     (void *)done);
    if (err != ESP_OK) return err;
    while (!*done &&
           (uint32_t)(esp_timer_get_time() - begin) < RF_DMA_TIMEOUT_US) {
        __asm__ __volatile__("nop");
    }
    *elapsed_us = (uint32_t)(esp_timer_get_time() - begin);
    return *done ? ESP_OK : ESP_ERR_TIMEOUT;
}

static uint32_t rf_bank_seed(uint32_t seed, unsigned word)
{
    return seed ^ (0x9e3779b9u * (word + 1u));
}

static unsigned changed_bytes(const uint8_t *a, const uint8_t *b)
{
    unsigned changed = 0u;
    for (unsigned i = 0; i < RF_DMA_PROBE_BYTES; ++i)
        changed += a[i] != b[i];
    return changed;
}

static uint32_t probe_hash(const uint8_t *data)
{
    uint32_t hash = 2166136261u;
    for (unsigned i = 0; i < RF_DMA_PROBE_BYTES; ++i) {
        hash ^= data[i];
        hash *= 16777619u;
    }
    return hash;
}

static uint32_t fnv1a_hash(const void *data, size_t bytes)
{
    const uint8_t *input = data;
    uint32_t hash = 2166136261u;
    for (size_t i = 0; i < bytes; ++i) {
        hash ^= input[i];
        hash *= 16777619u;
    }
    return hash;
}

typedef struct {
    uint32_t status;
    uint32_t info0;
    uint32_t info1;
} hp_apm_m1_snapshot_t;

static hp_apm_m1_snapshot_t hp_apm_m1_snapshot(void)
{
    const hp_apm_m1_snapshot_t snapshot = {
        .status = REG32(HP_APM_M1_STATUS_REG),
        .info0 = REG32(HP_APM_M1_EXCEPTION_INFO0_REG),
        .info1 = REG32(HP_APM_M1_EXCEPTION_INFO1_REG),
    };
    return snapshot;
}

static void hp_apm_m1_clear(void)
{
    REG32(HP_APM_M1_STATUS_CLR_REG) = HP_APM_M1_EXCEPTION_STATUS_CLR;
    __asm__ __volatile__("fence iorw, iorw" ::: "memory");
}

esp_err_t c5vrx2_rf_dma_diagnostic_run(void)
{
    for (unsigned second = 10u; second != 0u; --second) {
        ESP_LOGW(TAG, "RF DMA diagnostic starts in %u seconds (USB settle)",
                 second);
        vTaskDelay(pdMS_TO_TICKS(1000u));
    }

    async_memcpy_handle_t dma = NULL;
    async_memcpy_config_t dma_config = ASYNC_MEMCPY_DEFAULT_CONFIG();
    dma_config.backlog = 2u;
    dma_config.weight = 1u;
    dma_config.dma_burst_size = 32u;
    esp_err_t err = esp_async_memcpy_install_gdma_ahb(&dma_config, &dma);
    if (err != ESP_OK) return err;

    volatile uint32_t *bank_a = (volatile uint32_t *)(uintptr_t)RF_BANK_A;
    volatile uint32_t *bank_b = (volatile uint32_t *)(uintptr_t)RF_BANK_B;
    for (unsigned i = 0; i < RF_BANK_WORDS; ++i) {
        bank_a[i] = rf_bank_seed(RF_BANK_A_SEED, i);
        bank_b[i] = rf_bank_seed(RF_BANK_B_SEED, i);
    }
    __asm__ __volatile__("fence iorw, iorw" ::: "memory");

    memset(s_rf_dma_a0, 0xa5, sizeof(s_rf_dma_a0));
    memset(s_rf_dma_a1, 0xa5, sizeof(s_rf_dma_a1));
    memset(s_rf_dma_b0, 0x5a, sizeof(s_rf_dma_b0));
    memset(s_rf_dma_b1, 0x5a, sizeof(s_rf_dma_b1));
    err = continuous_iq_start();
    if (err != ESP_OK) {
        (void)esp_async_memcpy_uninstall(dma);
        return err;
    }

    const uint32_t pointer = REG32(DUMP_PTR_MODE) & PTR_MASK;
    uint32_t source_word = (pointer + C5VRX2_RF_WORDS / 2u) & PTR_MASK;
    source_word &= ~((RF_DMA_PROBE_BYTES / sizeof(uint32_t)) - 1u);
    const void *source_a = (const void *)(uintptr_t)
        (RF_BANK_A + source_word * sizeof(uint32_t));
    const void *source_b = (const void *)(uintptr_t)
        (RF_BANK_B + source_word * sizeof(uint32_t));

    uint32_t a0_us = 0u, a1_us = 0u, b0_us = 0u, b1_us = 0u;
    hp_apm_m1_clear();
    const hp_apm_m1_snapshot_t apm_before = hp_apm_m1_snapshot();
    c5vrx2_trace_stage_detail(181u, ESP_OK, apm_before.status,
                              apm_before.info0, apm_before.info1);
    const esp_err_t a0_err =
        rf_dma_copy_wait(dma, s_rf_dma_a0, source_a, &s_rf_dma_done_a0,
                         &a0_us);
    const hp_apm_m1_snapshot_t apm_after_a0 = hp_apm_m1_snapshot();
    c5vrx2_trace_stage_detail(182u, a0_err, apm_after_a0.status,
                              apm_after_a0.info0, apm_after_a0.info1);
    const esp_err_t b0_err =
        rf_dma_copy_wait(dma, s_rf_dma_b0, source_b, &s_rf_dma_done_b0,
                         &b0_us);
    const hp_apm_m1_snapshot_t apm_after_b0 = hp_apm_m1_snapshot();
    c5vrx2_trace_stage_detail(183u, b0_err, apm_after_b0.status,
                              apm_after_b0.info0, apm_after_b0.info1);
    /* At ~80 MS/s this exceeds one 16K traversal, so the same physical
     * address should contain a later RF generation. */
    const int64_t wait_begin = esp_timer_get_time();
    while ((uint32_t)(esp_timer_get_time() - wait_begin) < 300u) {
        __asm__ __volatile__("nop");
    }
    const esp_err_t a1_err = a0_err == ESP_OK
        ? rf_dma_copy_wait(dma, s_rf_dma_a1, source_a, &s_rf_dma_done_a1,
                           &a1_us)
        : ESP_ERR_INVALID_STATE;
    const hp_apm_m1_snapshot_t apm_after_a1 = hp_apm_m1_snapshot();
    c5vrx2_trace_stage_detail(184u, a1_err, apm_after_a1.status,
                              apm_after_a1.info0, apm_after_a1.info1);
    const esp_err_t b1_err = b0_err == ESP_OK
        ? rf_dma_copy_wait(dma, s_rf_dma_b1, source_b, &s_rf_dma_done_b1,
                           &b1_us)
        : ESP_ERR_INVALID_STATE;
    const hp_apm_m1_snapshot_t apm_after_b1 = hp_apm_m1_snapshot();
    c5vrx2_trace_stage_detail(185u, b1_err, apm_after_b1.status,
                              apm_after_b1.info0, apm_after_b1.info1);

    continuous_iq_stats_t stats;
    continuous_iq_get_stats(&stats);
    const uint32_t ownership_active = REG32(HP_SRAM_USAGE);
    const esp_err_t stop_err = continuous_iq_stop();
    const uint32_t ownership_restored = REG32(HP_SRAM_USAGE);

    unsigned overwritten_a = 0u;
    unsigned overwritten_b = 0u;
    for (unsigned i = 0; i < RF_BANK_WORDS; ++i) {
        overwritten_a += bank_a[i] != rf_bank_seed(RF_BANK_A_SEED, i);
        overwritten_b += bank_b[i] != rf_bank_seed(RF_BANK_B_SEED, i);
    }
    const unsigned live_changed_a = changed_bytes(s_rf_dma_a0, s_rf_dma_a1);
    const unsigned live_changed_b = changed_bytes(s_rf_dma_b0, s_rf_dma_b1);
    const bool visible_a = a0_err == ESP_OK && a1_err == ESP_OK &&
                           live_changed_a != 0u;
    const bool visible_b = b0_err == ESP_OK && b1_err == ESP_OK &&
                           live_changed_b != 0u;
    const bool guards_after_stop = c5vrx2_rf_dump_guards_valid();
    c5vrx2_trace_stage_detail(201u, a0_err, a0_us, 0u, 0u);
    c5vrx2_trace_stage_detail(202u, b0_err, b0_us, 0u, 0u);
    c5vrx2_trace_stage_detail(203u, a1_err, a1_us, 0u, 0u);
    c5vrx2_trace_stage_detail(204u, b1_err, b1_us, 0u, 0u);
    c5vrx2_trace_stage_detail(205u,
                              visible_a ? ESP_OK : ESP_ERR_NOT_SUPPORTED,
                              live_changed_a, probe_hash(s_rf_dma_a0),
                              probe_hash(s_rf_dma_a1));
    c5vrx2_trace_stage_detail(206u,
                              visible_b ? ESP_OK : ESP_ERR_NOT_SUPPORTED,
                              live_changed_b, probe_hash(s_rf_dma_b0),
                              probe_hash(s_rf_dma_b1));
    c5vrx2_trace_stage_detail(207u, stop_err, overwritten_a, overwritten_b,
                              guards_after_stop);
    c5vrx2_trace_stage_detail(208u, ESP_OK, ownership_active,
                              ownership_restored, 0u);
    c5vrx2_trace_stage_detail(209u, ESP_OK, apm_before.status,
                              apm_before.info0, apm_before.info1);
    c5vrx2_trace_stage_detail(210u, ESP_OK, apm_after_a0.status,
                              apm_after_a0.info0, apm_after_a0.info1);
    c5vrx2_trace_stage_detail(211u, ESP_OK, apm_after_b0.status,
                              apm_after_b0.info0, apm_after_b0.info1);
    c5vrx2_trace_stage_detail(212u, ESP_OK, apm_after_a1.status,
                              apm_after_a1.info0, apm_after_a1.info1);
    c5vrx2_trace_stage_detail(213u, ESP_OK, apm_after_b1.status,
                              apm_after_b1.info0, apm_after_b1.info1);
    for (unsigned report = 1u; report <= 30u; ++report) {
        ESP_LOGW(TAG,
                 "RF BANK VISIBILITY report=%u/30 src_word=%u "
                 "A=%s/%uus,%s/%uus live=%u overwritten=%u "
                 "B=%s/%uus,%s/%uus live=%u overwritten=%u "
                 "ptr=%u ctrl=0x%08x starts=%u rearms=%u triggers=%u "
                 "owner=0x%08x->0x%08x apm=%x/%08x/%08x "
                 "stop=%s guards=%u verdict=%s",
                 report, (unsigned)source_word,
                 esp_err_to_name(a0_err), (unsigned)a0_us,
                 esp_err_to_name(a1_err), (unsigned)a1_us,
                 live_changed_a, overwritten_a,
                 esp_err_to_name(b0_err), (unsigned)b0_us,
                 esp_err_to_name(b1_err), (unsigned)b1_us,
                 live_changed_b, overwritten_b,
                 (unsigned)stats.writer_pointer, (unsigned)stats.dump_control,
                 (unsigned)stats.producer_start_count,
                 (unsigned)stats.rearm_count, (unsigned)stats.trigger_count,
                 (unsigned)ownership_active, (unsigned)ownership_restored,
                 (unsigned)apm_after_b1.status,
                 (unsigned)apm_after_b1.info0,
                 (unsigned)apm_after_b1.info1,
                 esp_err_to_name(stop_err), guards_after_stop,
                 visible_a ? "BANK_A_LIVE" :
                 visible_b ? "BANK_B_LIVE" : "BOTH_LIVE_VIEWS_BLOCKED");
        vTaskDelay(pdMS_TO_TICKS(1000u));
    }

    const esp_err_t uninstall_err = esp_async_memcpy_uninstall(dma);
    if (!visible_a && !visible_b) return ESP_ERR_NOT_SUPPORTED;
    if (stop_err != ESP_OK) return stop_err;
    return uninstall_err;
}

typedef struct {
    uint32_t transitions[MODEM_DIAG_LANES];
    uint32_t ever_high;
    uint32_t ever_low;
    uint32_t elapsed_us;
} modem_diag_result_t;

static uint32_t IRAM_ATTR modem_diag_read_lanes(void)
{
    const uint32_t gpio = REG32(GPIO_IN_REG);
    uint32_t lanes = 0u;
    for (unsigned lane = 0u; lane < MODEM_DIAG_LANES; ++lane) {
        lanes |= ((gpio >> s_modem_diag_pins[lane]) & 1u) << lane;
    }
    return lanes;
}

static void IRAM_ATTR modem_diag_route_batch(unsigned first_signal)
{
    for (unsigned lane = 0u; lane < MODEM_DIAG_LANES; ++lane) {
        const unsigned signal = first_signal + lane;
        if (signal < 32u) {
            esp_rom_gpio_connect_out_signal(s_modem_diag_pins[lane],
                                             MODEM_DIAG0_IDX + signal,
                                             false, false);
        } else {
            esp_rom_gpio_connect_out_signal(s_modem_diag_pins[lane],
                                             SIG_GPIO_OUT_IDX,
                                             false, false);
            (void)gpio_set_level(s_modem_diag_pins[lane], 0);
        }
    }
    __asm__ __volatile__("fence iorw, iorw" ::: "memory");
}

static modem_diag_result_t IRAM_ATTR modem_diag_sample(void)
{
    modem_diag_result_t result = {0};
    uint32_t previous = modem_diag_read_lanes();
    result.ever_high = previous;
    result.ever_low = (~previous) & ((1u << MODEM_DIAG_LANES) - 1u);
    const int64_t begin = esp_timer_get_time();
    for (unsigned sample = 0u; sample < MODEM_DIAG_SAMPLES; ++sample) {
        const uint32_t current = modem_diag_read_lanes();
        const uint32_t changed = current ^ previous;
        result.ever_high |= current;
        result.ever_low |= ~current;
        for (unsigned lane = 0u; lane < MODEM_DIAG_LANES; ++lane) {
            result.transitions[lane] += (changed >> lane) & 1u;
        }
        previous = current;
    }
    result.ever_low &= (1u << MODEM_DIAG_LANES) - 1u;
    result.elapsed_us = (uint32_t)(esp_timer_get_time() - begin);
    return result;
}

esp_err_t c5vrx2_modem_diag_diagnostic_run(void)
{
    const uint64_t pin_mask =
        (1ULL << GPIO_NUM_23) | (1ULL << GPIO_NUM_24) |
        (1ULL << GPIO_NUM_11) | (1ULL << GPIO_NUM_12) |
        (1ULL << GPIO_NUM_8) | (1ULL << GPIO_NUM_9);
    const gpio_config_t gpio_cfg = {
        .pin_bit_mask = pin_mask,
        .mode = GPIO_MODE_INPUT_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    esp_err_t err = gpio_config(&gpio_cfg);
    if (err != ESP_OK) return err;

    /* Leave time for a monitor to display startup before MAC SRAM ownership
     * makes USB/JTAG observability unreliable on the current C5 setup. */
    vTaskDelay(pdMS_TO_TICKS(3000u));

    const uint32_t saved_fix = REG32(MODEM_WIDGET_DIAG_FIX);
    const uint32_t saved_exchange = REG32(MODEM_WIDGET_DIAG_EXCHANGE);
    const uint32_t saved_test_conf = REG32(MODEM_SYSCON_TEST_CONF);
    modem_diag_result_t results[MODEM_DIAG_CONFIGS][6] = {0};
    uint32_t test_conf_by_config[MODEM_DIAG_CONFIGS] = {0};

    const uint32_t saved_mstatus = modem_capture_irq_save_disable();
    err = continuous_iq_start();
    if (err != ESP_OK) {
        modem_capture_irq_restore(saved_mstatus);
        return err;
    }

    for (unsigned config = 0u; config < MODEM_DIAG_CONFIGS; ++config) {
        uint32_t fix = saved_fix;
        uint32_t exchange = saved_exchange;
        uint32_t test_conf = saved_test_conf;
        if (config == 1u) exchange = 2u; /* coex vendor state */
        if (config == 2u) {
            exchange = 2u;
            /* Final low ten bits produced by C5 bt_bb_ble_diag_all(). */
            fix = (saved_fix & ~0x3ffu) | 0x14eu;
        }
        if (config == 3u) {
            /* The output lane is undocumented, so measure all 32 signals. */
            test_conf |= MODEM_FPGA_DEBUG_CLKSWITCH |
                         MODEM_FPGA_DEBUG_CLK80;
        }
        REG32(MODEM_WIDGET_DIAG_FIX) = fix;
        REG32(MODEM_WIDGET_DIAG_EXCHANGE) = exchange;
        REG32(MODEM_SYSCON_TEST_CONF) = test_conf;
        __asm__ __volatile__("fence iorw, iorw" ::: "memory");
        test_conf_by_config[config] = REG32(MODEM_SYSCON_TEST_CONF);

        for (unsigned batch = 0u; batch < 6u; ++batch) {
            const unsigned first = batch * MODEM_DIAG_LANES;
            modem_diag_route_batch(first);
            results[config][batch] = modem_diag_sample();
        }
    }

    REG32(MODEM_WIDGET_DIAG_FIX) = saved_fix;
    REG32(MODEM_WIDGET_DIAG_EXCHANGE) = saved_exchange;
    REG32(MODEM_SYSCON_TEST_CONF) = saved_test_conf;
    __asm__ __volatile__("fence iorw, iorw" ::: "memory");
    continuous_iq_stats_t stats;
    continuous_iq_get_stats(&stats);
    const esp_err_t stop_err = continuous_iq_stop();
    modem_capture_irq_restore(saved_mstatus);
    c5vrx2_trace_stage_detail(390u, stop_err, saved_fix, saved_exchange,
                              stats.physical_wraps);
    c5vrx2_trace_stage_detail(391u, ESP_OK, saved_test_conf,
                              test_conf_by_config[3], MODEM_DIAG_SAMPLES);

    /* Persist results only after MAC ownership has been released. */
    for (unsigned config = 0u; config < MODEM_DIAG_CONFIGS; ++config) {
        c5vrx2_trace_stage_detail(392u + config, ESP_OK,
                                  test_conf_by_config[config], config,
                                  MODEM_DIAG_SAMPLES);
        for (unsigned batch = 0u; batch < 6u; ++batch) {
            const modem_diag_result_t *r = &results[config][batch];
            const uint32_t activity =
                (r->ever_high & 0x3fu) |
                ((r->ever_low & 0x3fu) << 8) |
                (((r->ever_high & r->ever_low) & 0x3fu) << 16);
            const uint32_t stage = 400u + config * 12u + batch * 2u;
            c5vrx2_trace_stage_detail(stage, (esp_err_t)activity,
                                      r->transitions[0], r->transitions[1],
                                      r->transitions[2]);
            c5vrx2_trace_stage_detail(stage + 1u, (esp_err_t)activity,
                                      r->transitions[3], r->transitions[4],
                                      r->transitions[5]);
        }
    }

    for (unsigned lane = 0u; lane < MODEM_DIAG_LANES; ++lane) {
        (void)gpio_reset_pin(s_modem_diag_pins[lane]);
    }
    for (unsigned config = 0u; config < MODEM_DIAG_CONFIGS; ++config) {
        for (unsigned batch = 0u; batch < 6u; ++batch) {
            const modem_diag_result_t *r = &results[config][batch];
            ESP_LOGW(TAG,
                     "MODEM DIAG config=%u signals=%u..%u us=%u "
                     "high=0x%02x low=0x%02x transitions="
                     "%u,%u,%u,%u,%u,%u",
                     config, batch * MODEM_DIAG_LANES,
                     batch * MODEM_DIAG_LANES + MODEM_DIAG_LANES - 1u,
                     (unsigned)r->elapsed_us, (unsigned)r->ever_high,
                     (unsigned)r->ever_low,
                     (unsigned)r->transitions[0],
                     (unsigned)r->transitions[1],
                     (unsigned)r->transitions[2],
                     (unsigned)r->transitions[3],
                     (unsigned)r->transitions[4],
                     (unsigned)r->transitions[5]);
        }
    }
    ESP_LOGW(TAG,
             "MODEM DIAG STOP rate=%u starts=%u rearms=%u wraps=%u "
             "triggers=%u result=%s",
             (unsigned)stats.rf_sample_rate_hz,
             (unsigned)stats.producer_start_count,
             (unsigned)stats.rearm_count, (unsigned)stats.physical_wraps,
             (unsigned)stats.trigger_count, esp_err_to_name(stop_err));
    return stop_err;
}

esp_err_t IRAM_ATTR c5vrx2_modem_capture_diagnostic_run(void)
{
    uint64_t pin_mask = 1ULL << MODEM_PARLIO_CLOCK_GPIO;
    uint32_t stored_pin_mask = 0u;
    for (unsigned signal = 0u; signal < MODEM_CAPTURE_LANES; ++signal) {
        pin_mask |= 1ULL << s_modem_capture_pins[signal];
        stored_pin_mask |= 1u << s_modem_capture_pins[signal];
    }
    if (stored_pin_mask != MODEM_CAPTURE_PIN_MASK)
        return ESP_ERR_INVALID_STATE;
    const gpio_config_t gpio_cfg = {
        .pin_bit_mask = pin_mask,
        .mode = GPIO_MODE_INPUT_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    esp_err_t err = gpio_config(&gpio_cfg);
    if (err != ESP_OK) return err;

    const uint32_t saved_fix = REG32(MODEM_WIDGET_DIAG_FIX);
    const uint32_t saved_exchange = REG32(MODEM_WIDGET_DIAG_EXCHANGE);
    for (unsigned lane = 0u; lane < MODEM_CAPTURE_LANES; ++lane) {
        esp_rom_gpio_connect_out_signal(s_modem_capture_pins[lane],
                                         MODEM_DIAG0_IDX +
                                             s_modem_capture_signals[lane],
                                         false, false);
    }
    __asm__ __volatile__("fence iorw, iorw" ::: "memory");

    /* Persist a pre-arm marker while CPU still owns all HP SRAM. If the dump
     * start never returns, this distinguishes that from an earlier Wi-Fi or
     * diagnostic-entry failure without performing flash I/O while armed. */
    modem_capture_header_t prearm = {
        .magic = MODEM_CAPTURE_MAGIC,
        .version = 3u,
        .header_bytes = sizeof(modem_capture_header_t),
        .diag_fix = saved_fix,
        .diag_exchange = saved_exchange,
        .gpio_mask = stored_pin_mask,
    };
    memset(prearm.gpio_for_diag, 0xff, sizeof(prearm.gpio_for_diag));
    for (unsigned lane = 0u; lane < MODEM_CAPTURE_LANES; ++lane) {
        prearm.gpio_for_diag[s_modem_capture_signals[lane]] =
            (uint8_t)s_modem_capture_pins[lane];
    }
    const uint32_t prearm_stage = 395u;
    memcpy(prearm.reserved, &prearm_stage, sizeof(prearm_stage));
    const uint32_t previous_stage = continuous_iq_debug_last_stage();
    const uint32_t reset_reason = (uint32_t)esp_reset_reason();
    memcpy(prearm.reserved + 4u, &previous_stage, sizeof(previous_stage));
    memcpy(prearm.reserved + 8u, &reset_reason, sizeof(reset_reason));
    const esp_partition_t *prearm_partition = esp_partition_find_first(
        ESP_PARTITION_TYPE_DATA, MODEM_CAPTURE_SUBTYPE, "diagcap");
    if (!prearm_partition) {
        err = ESP_ERR_NOT_FOUND;
        goto cleanup_pins;
    }
    err = esp_partition_erase_range(prearm_partition, 0u,
                                    prearm_partition->size);
    if (err == ESP_OK) {
        err = esp_partition_write(prearm_partition, 0u, &prearm,
                                  sizeof(prearm));
    }
    if (err != ESP_OK) goto cleanup_pins;

    const uint32_t saved_mstatus = modem_capture_irq_save_disable();
    continuous_iq_debug_mark(429u);
    err = continuous_iq_start();
    if (err != ESP_OK) {
        modem_capture_irq_restore(saved_mstatus);
        goto cleanup_pins;
    }
    continuous_iq_debug_mark(430u);

    /* Preserve simultaneous bus words. Packing GPIO bits in this hot loop
     * would reduce the capture rate and is intentionally deferred to the
     * host-side analysis. */
    const uint32_t sample_begin = esp_cpu_get_cycle_count();
    for (unsigned i = 0u; i < MODEM_CAPTURE_WORDS; ++i) {
        s_modem_capture[i] = REG32(GPIO_IN_REG);
    }
    const uint32_t sample_cycles = esp_cpu_get_cycle_count() - sample_begin;
    const uint32_t sample_us =
        sample_cycles / CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ;
    __asm__ __volatile__("fence iorw, iorw" ::: "memory");
    continuous_iq_debug_mark(431u);

    continuous_iq_stats_t stats;
    continuous_iq_get_stats(&stats);
    continuous_iq_debug_mark(432u);
    const uint32_t capture_end_ptr_mode = REG32(DUMP_PTR_MODE);
    const uint32_t capture_end_pointer = capture_end_ptr_mode & PTR_MASK;
    const uint32_t stop_begin = esp_cpu_get_cycle_count();
    const esp_err_t stop_err = continuous_iq_stop();
    const uint32_t stop_cycles = esp_cpu_get_cycle_count() - stop_begin;
    const uint32_t stop_us =
        stop_cycles / CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ;
    continuous_iq_debug_mark(433u);
    modem_capture_irq_restore(saved_mstatus);
    const uint32_t stop_ptr_mode = REG32(DUMP_PTR_MODE);
    const uint32_t stop_pointer = stop_ptr_mode & PTR_MASK;

    REG32(MODEM_WIDGET_DIAG_FIX) = saved_fix;
    REG32(MODEM_WIDGET_DIAG_EXCHANGE) = saved_exchange;
    __asm__ __volatile__("fence iorw, iorw" ::: "memory");
    for (unsigned lane = 0u; lane < MODEM_CAPTURE_LANES; ++lane) {
        (void)gpio_reset_pin(s_modem_capture_pins[lane]);
    }

    if (stop_err != ESP_OK) return stop_err;

    const void *ring = continuous_iq_ring_base();
    modem_capture_header_t header = {
        .magic = MODEM_CAPTURE_MAGIC,
        .version = 1u,
        .header_bytes = sizeof(modem_capture_header_t),
        .raw_words = MODEM_CAPTURE_WORDS,
        .ring_words = C5VRX2_RF_WORDS,
        .diag_fix = saved_fix,
        .diag_exchange = saved_exchange,
        .gpio_mask = stored_pin_mask,
        .sample_us = sample_us,
        .rf_rate_hz = stats.rf_sample_rate_hz,
        .writer_pointer = stop_pointer,
        .dump_control = stats.dump_control,
        .dump_ptr_mode = stop_ptr_mode,
        .producer_starts = stats.producer_start_count,
        .physical_wraps = stats.physical_wraps,
        .trigger_count = stats.trigger_count,
        .raw_hash = fnv1a_hash(s_modem_capture, sizeof(s_modem_capture)),
        .ring_hash = fnv1a_hash(ring, continuous_iq_ring_bytes()),
        .capture_end_pointer = capture_end_pointer,
        .capture_end_ptr_mode = capture_end_ptr_mode,
        .stop_us = stop_us,
    };
    memset(header.gpio_for_diag, 0xff, sizeof(header.gpio_for_diag));
    for (unsigned lane = 0u; lane < MODEM_CAPTURE_LANES; ++lane) {
        header.gpio_for_diag[s_modem_capture_signals[lane]] =
            (uint8_t)s_modem_capture_pins[lane];
    }

    const esp_partition_t *partition = esp_partition_find_first(
        ESP_PARTITION_TYPE_DATA, MODEM_CAPTURE_SUBTYPE, "diagcap");
    if (!partition) return ESP_ERR_NOT_FOUND;
    const size_t raw_offset = sizeof(header);
    const size_t ring_offset = raw_offset + sizeof(s_modem_capture);
    const size_t required = ring_offset + continuous_iq_ring_bytes();
    if (required > partition->size) return ESP_ERR_INVALID_SIZE;

    err = esp_partition_erase_range(partition, 0u, partition->size);
    if (err == ESP_OK) {
        err = esp_partition_write(partition, raw_offset, s_modem_capture,
                                  sizeof(s_modem_capture));
    }
    if (err == ESP_OK) {
        err = esp_partition_write(partition, ring_offset, ring,
                                  continuous_iq_ring_bytes());
    }
    /* Write the completion marker last, so a valid magic always denotes a
     * complete raw-bus plus RF-ring record. */
    if (err == ESP_OK) {
        err = esp_partition_write(partition, 0u, &header, sizeof(header));
    }

    ESP_LOGW(TAG,
             "MODEM CAPTURE result=%s samples=%u sample_us=%u rf_hz=%u "
             "capture_ptr=%u stop_ptr=%u stop_us=%u starts=%u wraps=%u "
             "triggers=%u raw_hash=%08x ring_hash=%08x",
             esp_err_to_name(err), MODEM_CAPTURE_WORDS, sample_us,
             (unsigned)stats.rf_sample_rate_hz, capture_end_pointer,
             stop_pointer, stop_us, (unsigned)stats.producer_start_count,
             (unsigned)stats.physical_wraps, (unsigned)stats.trigger_count,
             (unsigned)header.raw_hash, (unsigned)header.ring_hash);
    return err;

cleanup_pins:
    REG32(MODEM_WIDGET_DIAG_FIX) = saved_fix;
    REG32(MODEM_WIDGET_DIAG_EXCHANGE) = saved_exchange;
    for (unsigned lane = 0u; lane < MODEM_CAPTURE_LANES; ++lane) {
        (void)gpio_reset_pin(s_modem_capture_pins[lane]);
    }
    /* A failed continuous start restores CPU ownership before returning.
     * Persist a compact result as well, otherwise an erased capture is
     * indistinguishable from firmware which never reached this diagnostic. */
    modem_capture_header_t failure = {
        .magic = MODEM_CAPTURE_MAGIC,
        .version = 2u,
        .header_bytes = sizeof(modem_capture_header_t),
        .diag_fix = saved_fix,
        .diag_exchange = saved_exchange,
        .dump_control = REG32(DUMP_CTRL),
        .dump_ptr_mode = REG32(DUMP_PTR_MODE),
    };
    memset(failure.gpio_for_diag, 0xff, sizeof(failure.gpio_for_diag));
    for (unsigned lane = 0u; lane < MODEM_CAPTURE_LANES; ++lane) {
        failure.gpio_for_diag[s_modem_capture_signals[lane]] =
            (uint8_t)s_modem_capture_pins[lane];
    }
    memcpy(failure.reserved, &err, sizeof(err));
    const uint32_t failure_stage = continuous_iq_debug_last_stage();
    const uint32_t failure_reset_reason = (uint32_t)esp_reset_reason();
    memcpy(failure.reserved + 4u, &failure_stage, sizeof(failure_stage));
    memcpy(failure.reserved + 8u, &failure_reset_reason,
           sizeof(failure_reset_reason));
    const esp_partition_t *failure_partition = esp_partition_find_first(
        ESP_PARTITION_TYPE_DATA, MODEM_CAPTURE_SUBTYPE, "diagcap");
    if (failure_partition &&
        esp_partition_erase_range(failure_partition, 0u,
                                  failure_partition->size) == ESP_OK) {
        (void)esp_partition_write(failure_partition, 0u, &failure,
                                  sizeof(failure));
    }
    return err;
}

esp_err_t IRAM_ATTR c5vrx2_modem_parlio_diagnostic_run(void)
{
    uint64_t pin_mask = 1ULL << MODEM_PARLIO_CLOCK_GPIO;
    for (unsigned lane = 0u; lane < MODEM_CAPTURE_LANES; ++lane) {
        pin_mask |= 1ULL << s_modem_capture_pins[lane];
    }
    const gpio_config_t gpio_cfg = {
        .pin_bit_mask = pin_mask,
        .mode = GPIO_MODE_INPUT_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    esp_err_t err = gpio_config(&gpio_cfg);
    if (err != ESP_OK) return err;

    const uint32_t saved_fix = REG32(MODEM_WIDGET_DIAG_FIX);
    const uint32_t saved_exchange = REG32(MODEM_WIDGET_DIAG_EXCHANGE);
    const uint32_t saved_clk_out = GPIO_EXT.pin_ctrl.val;
    const uint32_t saved_clk_out_enable = PCR.ctrl_clk_out_en.val;
    for (unsigned lane = 0u; lane < MODEM_CAPTURE_LANES; ++lane) {
        esp_rom_gpio_connect_out_signal(s_modem_capture_pins[lane],
                                         MODEM_DIAG0_IDX +
                                             s_modem_capture_signals[lane],
                                         false, false);
    }
    /* Use the same PLL-derived internal 40-MHz RX clock as production. The
     * earlier 80/160-MHz external-clock probes established that C5 PARLIO RX
     * still accepts about 40 MS/s; an unused faster clock only complicates
     * this bounded byte-for-byte IQ proof. */
    __asm__ __volatile__("fence iorw, iorw" ::: "memory");
    uint32_t clock_transitions = 0u;
    uint32_t previous_clock =
        (REG32(GPIO_IN_REG) >> MODEM_PARLIO_CLOCK_GPIO) & 1u;
    for (unsigned sample = 0u; sample < 32768u; ++sample) {
        const uint32_t clock =
            (REG32(GPIO_IN_REG) >> MODEM_PARLIO_CLOCK_GPIO) & 1u;
        clock_transitions += clock != previous_clock;
        previous_clock = clock;
    }
    c5vrx2_trace_stage_detail(309u, ESP_OK, saved_clk_out,
                              GPIO_EXT.pin_ctrl.val,
                              clock_transitions);
    c5vrx2_trace_stage_detail(308u, ESP_OK, saved_clk_out_enable,
                              PCR.ctrl_clk_out_en.val,
                              REG32(0x600a9c00u));

    modem_capture_header_t prearm = {
        .magic = MODEM_CAPTURE_MAGIC,
        .version = 10u,
        .header_bytes = sizeof(modem_capture_header_t),
        .raw_words = MODEM_PARLIO_BYTES,
        .ring_words = C5VRX2_RF_WORDS,
        .diag_fix = saved_fix,
        .diag_exchange = saved_exchange,
        .gpio_mask = MODEM_CAPTURE_PIN_MASK | (1u << MODEM_PARLIO_CLOCK_GPIO),
        .rf_rate_hz = MODEM_PARLIO_SAMPLE_RATE_HZ,
    };
    memset(prearm.gpio_for_diag, 0xff, sizeof(prearm.gpio_for_diag));
    for (unsigned lane = 0u; lane < MODEM_CAPTURE_LANES; ++lane) {
        prearm.gpio_for_diag[s_modem_capture_signals[lane]] =
            (uint8_t)s_modem_capture_pins[lane];
    }
    const esp_partition_t *partition = esp_partition_find_first(
        ESP_PARTITION_TYPE_DATA, MODEM_CAPTURE_SUBTYPE, "diagcap");
    if (!partition) {
        err = ESP_ERR_NOT_FOUND;
        goto cleanup_pins_only;
    }
    err = esp_partition_erase_range(partition, 0u, partition->size);
    if (err == ESP_OK) {
        err = esp_partition_write(partition, 0u, &prearm, sizeof(prearm));
    }
    if (err != ESP_OK) goto cleanup_pins_only;
    c5vrx2_trace_stage_detail(310u, ESP_OK, MODEM_PARLIO_BYTES,
                              MODEM_PARLIO_SAMPLE_RATE_HZ, 0u);

    parlio_rx_unit_handle_t rx = NULL;
    parlio_rx_delimiter_handle_t delimiter = NULL;
    bitscrambler_handle_t rx_bs = NULL;
    const parlio_rx_unit_config_t rx_cfg = {
        .trans_queue_depth = 1u,
        .max_recv_size = MODEM_PARLIO_BYTES,
        /* The C5 RX driver advertises an external-memory-capable GDMA path;
         * that path accepts at most a 32-byte burst. This is transfer
         * granularity only and does not reduce the 80 MHz sample clock. */
        .dma_burst_size = 32u,
        .data_width = MODEM_CAPTURE_LANES,
        .clk_src = PARLIO_CLK_SRC_DEFAULT,
        .ext_clk_freq_hz = 0u,
        .exp_clk_freq_hz = MODEM_PARLIO_SAMPLE_RATE_HZ,
        .clk_in_gpio_num = -1,
        .clk_out_gpio_num = -1,
        .valid_gpio_num = -1,
        .data_gpio_nums = {
            GPIO_NUM_1, GPIO_NUM_0, GPIO_NUM_25, GPIO_NUM_7,
            GPIO_NUM_10, GPIO_NUM_5, GPIO_NUM_3, GPIO_NUM_4,
        },
        .flags = {
            .free_clk = true,
            .clk_gate_en = false,
            .allow_pd = false,
        },
    };
    err = parlio_new_rx_unit(&rx_cfg, &rx);
    c5vrx2_trace_stage_detail(311u, err, (uint32_t)(uintptr_t)rx, 0u, 0u);
    if (err != ESP_OK) goto cleanup_pins_only;

#if CONFIG_C5VRX2_MODE_MODEM_WBFM
    const bitscrambler_config_t bs_cfg = {
        .dir = BITSCRAMBLER_DIR_RX,
        .attach_to = SOC_BITSCRAMBLER_ATTACH_PARL_IO,
    };
    err = bitscrambler_new(&bs_cfg, &rx_bs);
    c5vrx2_trace_stage_detail(316u, err, 0u, 0u, 0u);
    if (err != ESP_OK) goto cleanup_rx;
    err = bitscrambler_enable(rx_bs);
    if (err == ESP_OK) err = c5vrx2_wbfm_q4_configure_delta(rx_bs);
    if (err == ESP_OK) err = bitscrambler_reset(rx_bs);
    c5vrx2_trace_stage_detail(317u, err, 0u, 0u, 0u);
    if (err != ESP_OK) goto cleanup_bs;
#endif

    const parlio_rx_soft_delimiter_config_t delimiter_cfg = {
        .sample_edge = PARLIO_SAMPLE_EDGE_POS,
        .bit_pack_order = PARLIO_BIT_PACK_ORDER_LSB,
        .eof_data_len = MODEM_PARLIO_BYTES,
        .timeout_ticks = 0u,
    };
    err = parlio_new_rx_soft_delimiter(&delimiter_cfg, &delimiter);
    c5vrx2_trace_stage_detail(312u, err,
                              (uint32_t)(uintptr_t)delimiter, 0u, 0u);
    if (err != ESP_OK) goto cleanup_rx;
    err = parlio_rx_unit_enable(rx, true);
    c5vrx2_trace_stage_detail(313u, err, 0u, 0u, 0u);
    if (err != ESP_OK) goto cleanup_delimiter;

    memset(s_modem_parlio_capture, 0xa5, sizeof(s_modem_parlio_capture));
    const parlio_receive_config_t receive_cfg = {
        .delimiter = delimiter,
        .flags = {
            .partial_rx_en = false,
            .indirect_mount = false,
        },
    };
    err = parlio_rx_unit_receive(rx, s_modem_parlio_capture,
                                 sizeof(s_modem_parlio_capture),
                                 &receive_cfg);
    c5vrx2_trace_stage_detail(314u, err, PARL_IO.rx_mode_cfg.val,
                              PARL_IO.rx_clk_cfg.val, PARL_IO.int_raw.val);
    if (err != ESP_OK) goto cleanup_enabled_rx;
    c5vrx2_trace_stage_detail(315u, ESP_OK, PARL_IO.rx_mode_cfg.val,
                              PARL_IO.rx_clk_cfg.val, PARL_IO.int_raw.val);

    const uint32_t saved_mstatus = modem_capture_irq_save_disable();
    continuous_iq_debug_mark(440u);
    err = continuous_iq_start();
    if (err != ESP_OK) {
        modem_capture_irq_restore(saved_mstatus);
        goto cleanup_enabled_rx;
    }
    continuous_iq_debug_mark(441u);

#if CONFIG_C5VRX2_MODE_MODEM_WBFM
    /* DMA and the PARLIO clock are already ready, but rx_sw_en is still
     * closed. Reset/start the attached processor only after real RF IQ is
     * active so no pre-arm MODEM_DIAG bytes can enter its persistent state. */
    err = bitscrambler_reset(rx_bs);
    if (err == ESP_OK) err = bitscrambler_start(rx_bs);
    if (err != ESP_OK) {
        (void)continuous_iq_stop();
        modem_capture_irq_restore(saved_mstatus);
        goto cleanup_enabled_rx;
    }
#endif

    const uint32_t capture_begin = esp_cpu_get_cycle_count();
    PARL_IO.rx_mode_cfg.rx_sw_en = 1u;
    __asm__ __volatile__("fence iorw, iorw" ::: "memory");
    const uint32_t capture_wait_cycles =
        MODEM_PARLIO_CAPTURE_US * CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ;
    while ((uint32_t)(esp_cpu_get_cycle_count() - capture_begin) <
           capture_wait_cycles) {
        __asm__ __volatile__("nop");
    }
    PARL_IO.rx_mode_cfg.rx_sw_en = 0u;
    __asm__ __volatile__("fence iorw, iorw" ::: "memory");
    const uint32_t capture_cycles =
        esp_cpu_get_cycle_count() - capture_begin;
    continuous_iq_debug_mark(442u);

    continuous_iq_stats_t stats;
    continuous_iq_get_stats(&stats);
    const uint32_t capture_end_ptr_mode = REG32(DUMP_PTR_MODE);
    const uint32_t capture_end_pointer = capture_end_ptr_mode & PTR_MASK;
    const uint32_t parlio_int_raw = PARL_IO.int_raw.val;
    const uint32_t parlio_rx_st0 = PARL_IO.rx_st0.val;
    const uint32_t parlio_rx_st1 = PARL_IO.rx_st1.val;
    const uint32_t stop_begin = esp_cpu_get_cycle_count();
    const esp_err_t stop_err = continuous_iq_stop();
    const uint32_t stop_cycles = esp_cpu_get_cycle_count() - stop_begin;
    modem_capture_irq_restore(saved_mstatus);
    continuous_iq_debug_mark(443u);
    if (stop_err != ESP_OK) {
        err = stop_err;
        goto cleanup_enabled_rx;
    }

    const esp_err_t receive_err = parlio_rx_unit_wait_all_done(rx, 100u);
    const esp_err_t disable_err = parlio_rx_unit_disable(rx);
    if (receive_err != ESP_OK) err = receive_err;
    else if (disable_err != ESP_OK) err = disable_err;
#if CONFIG_C5VRX2_MODE_MODEM_WBFM
    const esp_err_t bs_reset_err = bitscrambler_reset(rx_bs);
    const esp_err_t bs_disable_err = bitscrambler_disable(rx_bs);
    if (err == ESP_OK && bs_reset_err != ESP_OK) err = bs_reset_err;
    if (err == ESP_OK && bs_disable_err != ESP_OK) err = bs_disable_err;
#endif

    REG32(MODEM_WIDGET_DIAG_FIX) = saved_fix;
    REG32(MODEM_WIDGET_DIAG_EXCHANGE) = saved_exchange;
    __asm__ __volatile__("fence iorw, iorw" ::: "memory");

    const uint32_t stop_ptr_mode = REG32(DUMP_PTR_MODE);
    const uint32_t stop_pointer = stop_ptr_mode & PTR_MASK;
    const void *ring = continuous_iq_ring_base();
    modem_capture_header_t header = {
        .magic = MODEM_CAPTURE_MAGIC,
        .version = MODEM_PARLIO_WBFM_ENABLED ? 9u : 8u,
        .header_bytes = sizeof(modem_capture_header_t),
        .raw_words = MODEM_PARLIO_BYTES,
        .ring_words = C5VRX2_RF_WORDS,
        .diag_fix = saved_fix,
        .diag_exchange = saved_exchange,
        .gpio_mask = MODEM_CAPTURE_PIN_MASK | (1u << MODEM_PARLIO_CLOCK_GPIO),
        .sample_us = (uint32_t)(((uint64_t)MODEM_PARLIO_BYTES * 1000000u) /
                                MODEM_PARLIO_SAMPLE_RATE_HZ),
        .rf_rate_hz = stats.rf_sample_rate_hz,
        .writer_pointer = stop_pointer,
        .dump_control = stats.dump_control,
        .dump_ptr_mode = stop_ptr_mode,
        .producer_starts = stats.producer_start_count,
        .physical_wraps = stats.physical_wraps,
        .trigger_count = stats.trigger_count,
        .raw_hash = fnv1a_hash(s_modem_parlio_capture,
                               sizeof(s_modem_parlio_capture)),
        .ring_hash = fnv1a_hash(ring, continuous_iq_ring_bytes()),
        .capture_end_pointer = capture_end_pointer,
        .capture_end_ptr_mode = capture_end_ptr_mode,
        .stop_us = stop_cycles / CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ,
    };
    memset(header.gpio_for_diag, 0xff, sizeof(header.gpio_for_diag));
    for (unsigned lane = 0u; lane < MODEM_CAPTURE_LANES; ++lane) {
        header.gpio_for_diag[s_modem_capture_signals[lane]] =
            (uint8_t)s_modem_capture_pins[lane];
    }
    memcpy(header.reserved, &capture_cycles, sizeof(capture_cycles));
    memcpy(header.reserved + 4u, &parlio_int_raw, sizeof(parlio_int_raw));
    memcpy(header.reserved + 8u, &parlio_rx_st0, sizeof(parlio_rx_st0));
    memcpy(header.reserved + 12u, &parlio_rx_st1, sizeof(parlio_rx_st1));
    memcpy(header.reserved + 16u, &receive_err, sizeof(receive_err));
    const uint32_t sample_rate_hz = MODEM_PARLIO_SAMPLE_RATE_HZ;
    memcpy(header.reserved + 20u, &sample_rate_hz, sizeof(sample_rate_hz));

    const size_t raw_offset = sizeof(header);
    const size_t ring_offset = raw_offset + sizeof(s_modem_parlio_capture);
    const size_t required = ring_offset + continuous_iq_ring_bytes();
    if (required > partition->size) {
        err = ESP_ERR_INVALID_SIZE;
    } else {
        esp_err_t write_err = esp_partition_erase_range(partition, 0u,
                                                        partition->size);
        if (write_err == ESP_OK) {
            write_err = esp_partition_write(partition, raw_offset,
                                             s_modem_parlio_capture,
                                             sizeof(s_modem_parlio_capture));
        }
        if (write_err == ESP_OK) {
            write_err = esp_partition_write(partition, ring_offset, ring,
                                             continuous_iq_ring_bytes());
        }
        if (write_err == ESP_OK) {
            write_err = esp_partition_write(partition, 0u, &header,
                                             sizeof(header));
        }
        if (err == ESP_OK) err = write_err;
    }

    ESP_LOGW(TAG,
             "MODEM PARLIO%s result=%s bytes=%u clock=%u rf_hz=%u "
             "capture_ptr=%u stop_ptr=%u starts=%u wraps=%u triggers=%u "
             "int_raw=%08x rx_st0=%08x rx_st1=%08x",
             MODEM_PARLIO_WBFM_ENABLED ? " WBFM" : "",
             esp_err_to_name(err), MODEM_PARLIO_BYTES,
             MODEM_PARLIO_SAMPLE_RATE_HZ,
             (unsigned)stats.rf_sample_rate_hz, capture_end_pointer,
             stop_pointer, (unsigned)stats.producer_start_count,
             (unsigned)stats.physical_wraps, (unsigned)stats.trigger_count,
             (unsigned)parlio_int_raw, (unsigned)parlio_rx_st0,
             (unsigned)parlio_rx_st1);

    if (rx_bs) bitscrambler_free(rx_bs);
    if (delimiter) (void)parlio_del_rx_delimiter(delimiter);
    if (rx) (void)parlio_del_rx_unit(rx);
    goto cleanup_pins_only;

cleanup_enabled_rx:
    (void)parlio_rx_unit_disable(rx);
cleanup_delimiter:
    if (delimiter) (void)parlio_del_rx_delimiter(delimiter);
#if CONFIG_C5VRX2_MODE_MODEM_WBFM
cleanup_bs:
    if (rx_bs) {
        (void)bitscrambler_disable(rx_bs);
        bitscrambler_free(rx_bs);
    }
#endif
cleanup_rx:
    if (rx) (void)parlio_del_rx_unit(rx);
cleanup_pins_only:
    REG32(MODEM_WIDGET_DIAG_FIX) = saved_fix;
    REG32(MODEM_WIDGET_DIAG_EXCHANGE) = saved_exchange;
    GPIO_EXT.pin_ctrl.val = saved_clk_out;
    PCR.ctrl_clk_out_en.val = saved_clk_out_enable;
    for (unsigned lane = 0u; lane < MODEM_CAPTURE_LANES; ++lane) {
        (void)gpio_reset_pin(s_modem_capture_pins[lane]);
    }
    (void)gpio_reset_pin(MODEM_PARLIO_CLOCK_GPIO);
    return err;
}

esp_err_t c5vrx2_rf_wrap_diagnostic_run(void)
{
    esp_err_t err = continuous_iq_start();
    if (err != ESP_OK) return err;
    const volatile uint32_t *ring = continuous_iq_ring_base();
    uint32_t previous_pointer = REG32(DUMP_PTR_MODE) & PTR_MASK;
    unsigned wraps = 0u;
    unsigned phase_windows = 0u;
    unsigned cpu_visible_words = 0u;
    double boundary_abs = 0.0;
    double neighbor_abs = 0.0;
    float worst_difference = 0.0f;
    while (wraps < RF_WRAP_SOAK_TARGET) {
        const uint32_t pointer = REG32(DUMP_PTR_MODE) & PTR_MASK;
        if (pointer < previous_pointer && pointer >= 16u && pointer < 128u) {
            if (phase_windows < RF_PHASE_WINDOWS) {
                uint32_t window[16];
                for (unsigned i = 0; i < 8u; ++i)
                    window[i] = ring[C5VRX2_RF_WORDS - 8u + i];
                for (unsigned i = 0; i < 8u; ++i) window[8u + i] = ring[i];
                for (unsigned i = 0; i < 16u; ++i)
                    cpu_visible_words += window[i] != 0u;
                const float boundary = phase_step(window[7], window[8]);
                float neighbors = 0.0f;
                for (unsigned i = 1u; i < 15u; ++i) {
                    if (i == 8u) continue;
                    neighbors += fabsf(phase_step(window[i - 1u], window[i]));
                }
                neighbors /= 13.0f;
                const float difference = fabsf(fabsf(boundary) - neighbors);
                if (difference > worst_difference) worst_difference = difference;
                boundary_abs += fabsf(boundary);
                neighbor_abs += neighbors;
                phase_windows++;
            }
            wraps++;
        }
        previous_pointer = pointer;
        if ((REG32(DUMP_CTRL) & 0x80040000u) != 0x80000000u) {
            err = ESP_ERR_INVALID_STATE;
            break;
        }
    }
    continuous_iq_stats_t stats;
    continuous_iq_get_stats(&stats);
    const bool writer_healthy =
        err == ESP_OK && wraps == RF_WRAP_SOAK_TARGET &&
        stats.producer_start_count == 1u && stats.rearm_count == 0u &&
        stats.trigger_count == 0u &&
        (stats.dump_control & 0x80040000u) == 0x80000000u;
    const esp_err_t stop_err = continuous_iq_stop();
    /* The guard banks are not reliably HP-readable while MAC_DUMP_ALLOC owns
     * them. Check their contents only after restoring HP ownership. */
    const bool guards_after_stop = c5vrx2_rf_dump_guards_valid();
    const bool healthy = writer_healthy && stop_err == ESP_OK && guards_after_stop;
    if (!healthy && err == ESP_OK) err = ESP_ERR_INVALID_STATE;
    ESP_LOGW(TAG,
             "RF WRAP SOAK observed=%u phase_windows=%u hw_wraps=%u "
             "starts=%u rearms=%u triggers=%u ctrl=0x%08x guards=%u "
             "cpu_visible_words=%u boundary_abs=%.6f neighbor_abs=%.6f "
             "worst_delta=%.6f verdict=%s",
             wraps, phase_windows, (unsigned)stats.physical_wraps,
             (unsigned)stats.producer_start_count, (unsigned)stats.rearm_count,
             (unsigned)stats.trigger_count, (unsigned)stats.dump_control,
             guards_after_stop, cpu_visible_words,
             phase_windows ? boundary_abs / phase_windows : 0.0,
             phase_windows ? neighbor_abs / phase_windows : 0.0,
             worst_difference,
             healthy ? (cpu_visible_words != 0u
                            ? "RING_SOAK_PASS_PHASE_NOT_PROVEN"
                            : "RING_SOAK_PASS_CPU_VIEW_BLOCKED")
                     : "FAIL");
    return err;
}
