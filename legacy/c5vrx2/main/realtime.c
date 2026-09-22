#include "realtime.h"

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "driver/bitscrambler.h"
#include "driver/gpio.h"
#include "driver/parlio_bitscrambler.h"
#include "driver/parlio_rx.h"
#include "driver/parlio_tx.h"
#include "esp_attr.h"
#include "esp_cache.h"
#include "esp_log.h"
#include "esp_partition.h"
#include "esp_rom_gpio.h"
#include "esp_rom_sys.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "soc/gpio_sig_map.h"
#include "soc/soc_caps.h"

#include "calibration.h"
#include "continuous_iq.h"
#include "startup_trace.h"
#include "wbfm_q4.h"

#define MODEM_IQ_RATE_HZ 40000000u
#if CONFIG_C5VRX2_LINEAR80
#define CVBS_RATE_HZ     80000000u
#elif CONFIG_C5VRX2_WBFM_PHASE5_QUALITY || CONFIG_C5VRX2_WBFM_TRUE40 || CONFIG_C5VRX2_WBFM_PHASE5_100NS
#define CVBS_RATE_HZ     40000000u
#else
#define CVBS_RATE_HZ     20000000u
#endif
#define RAW_BLOCK_BYTES      4096u
#define RAW_RING_BLOCKS         4u
#define RAW_RING_BYTES (RAW_BLOCK_BYTES * RAW_RING_BLOCKS)

#define DUMP_CTRL       0x600a9004u
#define DUMP_PTR_MODE   0x600a9008u
#define CTRL_ENABLE     0x80000000u
#define CTRL_DONE       0x00040000u
#define PTR_MASK        0x00003fffu

static const char *TAG = "c5vrx2_rt";

/* Q[9:6], then I[9:6]. These MODEM_DIAG mappings were correlated against
 * the post-stop Q10/I10 ring on physical ESP32-C5 hardware. */
static const gpio_num_t s_iq_pins[8] = {
    GPIO_NUM_1, GPIO_NUM_0, GPIO_NUM_25, GPIO_NUM_7,
    GPIO_NUM_10, GPIO_NUM_5, GPIO_NUM_3, GPIO_NUM_4,
};
static const uint8_t s_iq_diag[8] = {6u, 7u, 8u, 9u, 16u, 17u, 18u, 19u};

/* RX-GDMA writes raw Q4/I4 at 40 MB/s. TX-GDMA reads the same bytes at
 * 40 MB/s and its BitScrambler emits two 6-bit CVBS samples per two input
 * bytes (40 MS/s). Both units derive 40 MHz from PLL_F240M / 6. Starting TX one block
 * behind RX keeps producer and consumer away from the same bytes without a
 * CPU copy or a second CVBS ring. */
static DMA_ATTR __attribute__((aligned(64))) uint8_t s_raw_ring[RAW_RING_BYTES];
#if CONFIG_C5VRX2_LIVE_SNAPSHOT_ONCE
static DRAM_ATTR __attribute__((aligned(16))) uint8_t
    s_raw_snapshot[RAW_RING_BYTES];
#endif

static parlio_rx_unit_handle_t s_rx;
static parlio_rx_delimiter_handle_t s_rx_delimiter;
static parlio_tx_unit_handle_t s_tx;

#define LIVE_CAPTURE_MAGIC 0x31564243u /* little-endian "CBV1" */
#define LIVE_CAPTURE_SUBTYPE ((esp_partition_subtype_t)0x42)

typedef struct __attribute__((packed)) {
    uint32_t magic;
    uint16_t version;
    uint16_t header_bytes;
    uint32_t payload_bytes;
    uint32_t iq_rate_hz;
    uint32_t cvbs_rate_hz;
    uint32_t writer_pointer;
    uint32_t dump_control;
    uint8_t minimum;
    uint8_t maximum;
    uint16_t reserved0;
    uint32_t sample_sum;
    uint32_t transitions;
    uint32_t reserved[6];
} live_capture_header_t;

_Static_assert(sizeof(live_capture_header_t) == 64u,
               "live capture header size");

static esp_err_t trace_step(uint32_t stage, esp_err_t err)
{
    c5vrx2_trace_stage(stage, err);
    return err;
}

static inline uint32_t reg32(uint32_t address)
{
    return *(volatile uint32_t *)(uintptr_t)address;
}

static esp_err_t route_modem_iq(void)
{
    uint64_t mask = 0u;
    for (unsigned lane = 0u; lane < 8u; ++lane) mask |= 1ULL << s_iq_pins[lane];
    const gpio_config_t cfg = {
        .pin_bit_mask = mask,
        .mode = GPIO_MODE_INPUT_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    esp_err_t err = gpio_config(&cfg);
    if (err != ESP_OK) return err;
    for (unsigned lane = 0u; lane < 8u; ++lane) {
        esp_rom_gpio_connect_out_signal(s_iq_pins[lane],
                                        MODEM_DIAG0_IDX + s_iq_diag[lane],
                                        false, false);
    }
    __asm__ __volatile__("fence iorw, iorw" ::: "memory");
    return ESP_OK;
}

static esp_err_t prepare_rx(void)
{
    const parlio_rx_unit_config_t cfg = {
        .trans_queue_depth = 1u,
        .max_recv_size = sizeof(s_raw_ring),
        .dma_burst_size = 32u,
        .data_width = 8u,
        .clk_src = PARLIO_CLK_SRC_DEFAULT,
        .ext_clk_freq_hz = 0u,
        .exp_clk_freq_hz = MODEM_IQ_RATE_HZ,
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
    esp_err_t err = parlio_new_rx_unit(&cfg, &s_rx);
    if (trace_step(11u, err) != ESP_OK) return err;

    const parlio_rx_soft_delimiter_config_t delimiter_cfg = {
#if CONFIG_C5VRX2_PARLIO_RX_NEG_EDGE
        .sample_edge = PARLIO_SAMPLE_EDGE_NEG,
#else
        .sample_edge = PARLIO_SAMPLE_EDGE_POS,
#endif
        .bit_pack_order = PARLIO_BIT_PACK_ORDER_LSB,
        /* IDF requires a non-zero soft-delimiter length even for an
         * infinite (partial_rx_en) transaction. In infinite mode this only
         * marks recurring receive boundaries; the cyclic GDMA link keeps
         * running and is not rearmed by software. */
        .eof_data_len = sizeof(s_raw_ring),
        .timeout_ticks = 0u,
    };
    err = parlio_new_rx_soft_delimiter(&delimiter_cfg, &s_rx_delimiter);
    if (trace_step(12u, err) != ESP_OK) return err;

    /* The receive side always stores raw Q4/I4. WBFM runs only on the TX
     * BitScrambler, which avoids the physically measured RX-BS throughput
     * limit and keeps the captured source available for diagnostics. */
    err = parlio_rx_unit_enable(s_rx, false);
    return trace_step(18u, err);
}

static esp_err_t prepare_tx(void)
{
    const parlio_tx_unit_config_t cfg = {
        .clk_src = PARLIO_CLK_SRC_DEFAULT,
        .clk_in_gpio_num = -1,
        .input_clk_src_freq_hz = 0u,
        .output_clk_freq_hz = CVBS_RATE_HZ,
        .data_width = 8u,
        .data_gpio_nums = {23, 24, 11, 12, 8, 9, -1, -1},
        .clk_out_gpio_num = -1,
        .valid_gpio_num = -1,
        .valid_start_delay = 0,
        .valid_stop_delay = 0,
        .trans_queue_depth = 1u,
        .max_transfer_size = sizeof(s_raw_ring),
        .dma_burst_size = 32u,
        .shift_edge = PARLIO_SHIFT_EDGE_NEG,
        .bit_pack_order = PARLIO_BIT_PACK_ORDER_LSB,
    };
    esp_err_t err = parlio_new_tx_unit(&cfg, &s_tx);
    if (trace_step(20u, err) != ESP_OK) return err;
    err = parlio_tx_unit_decorate_bitscrambler(s_tx);
    if (trace_step(21u, err) != ESP_OK) return err;
    err = parlio_tx_unit_enable(s_tx);
    return trace_step(22u, err);
}

static esp_err_t start_rx_ring(void)
{
    esp_err_t err = parlio_rx_soft_delimiter_start_stop(
        s_rx, s_rx_delimiter, true);
    if (err != ESP_OK) return err;
    const parlio_receive_config_t cfg = {
        .delimiter = s_rx_delimiter,
        .flags = {
            .partial_rx_en = true,
            .indirect_mount = false,
        },
    };
    return parlio_rx_unit_receive(s_rx, s_raw_ring, sizeof(s_raw_ring),
                                  &cfg);
}

static esp_err_t start_tx_ring(void)
{
    const c5vrx2_calibration_t *cal = c5vrx2_calibration_get();
    const parlio_transmit_config_t cfg = {
        .idle_value = cal->pedestal_code,
        .bitscrambler_program =
#if CONFIG_C5VRX2_WBFM_PHASE5_100NS
            c5vrx2_wbfm_q4_phase5_100ns_program(),
#elif CONFIG_C5VRX2_WBFM_TRUE40
            c5vrx2_wbfm_q4_true40_program(),
#elif CONFIG_C5VRX2_LINEAR80
            c5vrx2_wbfm_linear80_program(),
#elif CONFIG_C5VRX2_WBFM_PHASE5_QUALITY
            c5vrx2_wbfm_q4_phase5_program(),
#elif CONFIG_C5VRX2_WBFM_TRAJECTORY
            c5vrx2_wbfm_q4_trajectory_program(),
#else
            c5vrx2_wbfm_q4_iq5_program(),
#endif
        .flags.loop_transmission = true,
    };
    return parlio_tx_unit_transmit(s_tx, s_raw_ring,
                                   sizeof(s_raw_ring) * 8u, &cfg);
}

static void telemetry_task(void *argument)
{
    (void)argument;
    uint32_t previous = reg32(DUMP_PTR_MODE) & PTR_MASK;
    uint32_t stalls = 0u;
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(1000));
        const uint32_t current = reg32(DUMP_PTR_MODE) & PTR_MASK;
        const uint32_t control = reg32(DUMP_CTRL);
        if (current == previous) stalls++;
        previous = current;

        /* Read-only control-plane telemetry. Normal live deliberately never
         * scans or copies the DMA ring: USB/logging cannot contend for its
         * SRAM bandwidth or become part of realtime pacing. */
        ESP_LOGI(TAG,
                 "LIVE configured_iq_hz=40000000 configured_dac_hz=%u ptr=%u enable=%u done=%u "
                 "stalls=%u starts=1 rearms=0",
                 (unsigned)CVBS_RATE_HZ, (unsigned)current, (control & CTRL_ENABLE) != 0u,
                 (control & CTRL_DONE) != 0u, (unsigned)stalls);
    }
}

#if CONFIG_C5VRX2_LIVE_SNAPSHOT_ONCE
static void live_snapshot_task(void *argument)
{
    (void)argument;
    /* Allow the ring to make many laps after all transports have started. */
    vTaskDelay(pdMS_TO_TICKS(1000));
    vTaskDelay(pdMS_TO_TICKS(20));

    /* Freeze the producer before copying. Copying a 16-KiB ring while GDMA
     * overwrites it at 40 MB/s creates an artificial torn boundary and makes
     * duplicate/missing-sample analysis meaningless. Snapshot mode is a
     * bounded diagnostic, so stopping here cannot affect production-live. */
    const uint32_t writer = reg32(DUMP_PTR_MODE) & PTR_MASK;
    const uint32_t control = reg32(DUMP_CTRL);
    if (s_tx) (void)parlio_tx_unit_disable(s_tx);
    (void)parlio_rx_soft_delimiter_start_stop(s_rx, s_rx_delimiter, false);
    (void)parlio_rx_unit_disable(s_rx);
    if (continuous_iq_is_running()) (void)continuous_iq_stop();
    (void)esp_cache_msync(s_raw_ring, sizeof(s_raw_ring),
                          ESP_CACHE_MSYNC_FLAG_DIR_M2C);
    memcpy(s_raw_snapshot, s_raw_ring, sizeof(s_raw_snapshot));

    uint32_t sum = 0u;
    uint32_t transitions = 0u;
    uint8_t minimum = UINT8_MAX;
    uint8_t maximum = 0u;
    for (size_t i = 0u; i < sizeof(s_raw_snapshot); ++i) {
        const uint8_t sample = s_raw_snapshot[i];
        if (sample < minimum) minimum = sample;
        if (sample > maximum) maximum = sample;
        sum += sample;
        transitions += i != 0u && sample != s_raw_snapshot[i - 1u];
    }

    const live_capture_header_t header = {
        .magic = LIVE_CAPTURE_MAGIC,
        .version = 3u,
        .header_bytes = sizeof(live_capture_header_t),
        .payload_bytes = sizeof(s_raw_snapshot),
        .iq_rate_hz = MODEM_IQ_RATE_HZ,
        .cvbs_rate_hz = CONFIG_C5VRX2_LIVE_SNAPSHOT_RAW_Q4 ? 0u :
                        CVBS_RATE_HZ,
        .writer_pointer = writer,
        .dump_control = control,
        .minimum = minimum,
        .maximum = maximum,
        .sample_sum = sum,
        .transitions = transitions,
    };
    const esp_partition_t *partition = esp_partition_find_first(
        ESP_PARTITION_TYPE_DATA, LIVE_CAPTURE_SUBTYPE, "diagcap");
    esp_err_t result = partition ? ESP_OK : ESP_ERR_NOT_FOUND;
    if (result == ESP_OK)
        result = esp_partition_erase_range(partition, 0u,
                                           partition->erase_size * 3u);
    if (result == ESP_OK)
        result = esp_partition_write(partition, 0u, &header, sizeof(header));
    if (result == ESP_OK)
        result = esp_partition_write(partition, sizeof(header),
                                     s_raw_snapshot,
                                     sizeof(s_raw_snapshot));

    /* Six quick flashes means the snapshot is safely on flash. A slow solid
     * LED means startup, and production-live remains solid after start. */
    if (result == ESP_OK) {
        for (unsigned pulse = 0u; pulse < 6u; ++pulse) {
            gpio_set_level(GPIO_NUM_27, 1);
            vTaskDelay(pdMS_TO_TICKS(100));
            gpio_set_level(GPIO_NUM_27, 0);
            vTaskDelay(pdMS_TO_TICKS(100));
        }
    }
    gpio_set_level(GPIO_NUM_27, 1);
    for (;;) vTaskDelay(portMAX_DELAY);
}
#endif

esp_err_t c5vrx2_realtime_start(void)
{
    const c5vrx2_calibration_t *cal = c5vrx2_calibration_get();
    memset(s_raw_ring, 0, sizeof(s_raw_ring));
    (void)esp_cache_msync(s_raw_ring, sizeof(s_raw_ring),
                          ESP_CACHE_MSYNC_FLAG_DIR_C2M);

    esp_err_t err = route_modem_iq();
    if (trace_step(10u, err) != ESP_OK) return err;
    if ((err = prepare_rx()) != ESP_OK) return err;
#if !CONFIG_C5VRX2_LIVE_SNAPSHOT_RAW_Q4
    if ((err = prepare_tx()) != ESP_OK) return err;
#endif

    /* Snapshot bring-up first proves whether the live MODEM_DIAG bus is
     * independent of the autonomous SRAM dump writer. Wi-Fi/PHY has already
     * initialized and tuned the RX chain. Avoiding DUMP_ENABLE here keeps the
     * CPU, USB and flash available and isolates the DIAG->PARLIO path. */
#if CONFIG_C5VRX2_LIVE_SNAPSHOT_ONCE
    if ((err = start_rx_ring()) != ESP_OK) return err;
#else
    /* continuous_iq_start() arms the autonomous pre-trigger source once.
     * Start the AV ring immediately afterwards; normal live never rearms. */
    err = continuous_iq_start();
    if (trace_step(30u, err) != ESP_OK) return err;
    if ((err = start_rx_ring()) != ESP_OK) return err;
#endif

    /* RX and the TX-BS consume raw bytes at the same 40 MB/s. Start TX one
     * complete 4096-byte block behind RX. Their 40:20 clocks share PLL_F240M,
     * so that separation cannot drift in normal operation. */
    #if !CONFIG_C5VRX2_LIVE_SNAPSHOT_RAW_Q4
    esp_rom_delay_us(RAW_BLOCK_BYTES * 1000000u / MODEM_IQ_RATE_HZ);
    if ((err = start_tx_ring()) != ESP_OK) return err;
    #endif

#if !CONFIG_C5VRX2_LIVE_SNAPSHOT_RAW_Q4
    BaseType_t created = xTaskCreate(telemetry_task, "iq_av_stat", 4096,
                                     NULL, 1u, NULL);
    if (created != pdPASS) return ESP_ERR_NO_MEM;
#else
    BaseType_t created;
#endif
#if CONFIG_C5VRX2_LIVE_SNAPSHOT_ONCE
    created = xTaskCreate(live_snapshot_task, "raw_snap", 4096,
                          NULL, 2u, NULL);
    if (created != pdPASS) return ESP_ERR_NO_MEM;
#endif

#if CONFIG_C5VRX2_LIVE_SNAPSHOT_RAW_Q4
    ESP_LOGW(TAG,
             "RAW SNAPSHOT ACTIVE: MODEM_DIAG Q4/I4 -> PARLIO RX 40M -> "
             "GDMA/flash; WBFM and DAC bypassed");
#else
    ESP_LOGW(TAG,
             "LIVE ACTIVE: configured IQ=40000000 DAC=%u Hz; RF estimator=%u "
             "pedestal=%u gain=%u polarity=%u",
             (unsigned)CVBS_RATE_HZ, (unsigned)continuous_iq_sample_rate_hz(),
             cal->pedestal_code, cal->discriminator_gain,
             (unsigned)cal->polarity);
#endif
    return ESP_OK;
}
