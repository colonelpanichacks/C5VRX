/* Bounded diagnostic only: RF acquisition and DAC replay are sequential.
 * This does NOT observe simultaneous live RX/TX or prove live continuity. */
#include "sdkconfig.h"
#if CONFIG_C5VRX2_ISSUE11_CAPTURE
#include <string.h>
#include "driver/gpio.h"
#include "driver/parlio_rx.h"
#include "driver/parlio_tx.h"
#include "driver/parlio_bitscrambler.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_partition.h"
#include "esp_rom_gpio.h"
#include "esp_rom_sys.h"
#include "soc/gpio_sig_map.h"
#include "soc/parl_io_struct.h"
#include "continuous_iq.h"
#include "wbfm_q4.h"

#define RAW_BYTES 32768u
#define OUT_BYTES 12000u
static const int iq_pins[8] = {1, 0, 25, 7, 10, 5, 3, 4};
static const int dac_pins[8] = {23, 24, 11, 12, 8, 9, -1, -1};

static esp_err_t receive(const int *pins, uint32_t rate, uint8_t *dst,
                         size_t bytes, bool start_rf, uint32_t *status)
{
    parlio_rx_unit_handle_t rx = NULL;
    parlio_rx_delimiter_handle_t delim = NULL;
    bool enabled = false, rf = false;
    parlio_rx_unit_config_t cfg = {
        .trans_queue_depth = 1, .max_recv_size = bytes, .dma_burst_size = 32,
        .data_width = 8, .clk_src = PARLIO_CLK_SRC_DEFAULT,
        .exp_clk_freq_hz = rate, .clk_in_gpio_num = -1,
        .clk_out_gpio_num = -1, .valid_gpio_num = -1,
        .flags.free_clk = true,
    };
    memcpy(cfg.data_gpio_nums, pins, sizeof(cfg.data_gpio_nums));
    esp_err_t err = parlio_new_rx_unit(&cfg, &rx);
    if (err != ESP_OK) return err;
    const parlio_rx_soft_delimiter_config_t dc = {
        .sample_edge = PARLIO_SAMPLE_EDGE_POS,
        .bit_pack_order = PARLIO_BIT_PACK_ORDER_LSB, .eof_data_len = bytes,
    };
    err = parlio_new_rx_soft_delimiter(&dc, &delim);
    if (err != ESP_OK) goto done;
    err = parlio_rx_unit_enable(rx, true);
    if (err != ESP_OK) goto done;
    enabled = true;
    const parlio_receive_config_t rc = {.delimiter = delim};
    err = parlio_rx_unit_receive(rx, dst, bytes, &rc);
    if (err != ESP_OK) goto done;
    if (start_rf) {
        err = continuous_iq_start();
        if (err != ESP_OK) goto done;
        rf = true;
        esp_rom_delay_us(1000);
    }
    err = parlio_rx_soft_delimiter_start_stop(rx, delim, true);
    if (err == ESP_OK) err = parlio_rx_unit_wait_all_done(rx, 1000);
    *status = PARL_IO.int_raw.val;
done:
    if (enabled) {
        (void)parlio_rx_soft_delimiter_start_stop(rx, delim, false);
        (void)parlio_rx_unit_disable(rx);
    }
    if (rf) (void)continuous_iq_stop();
    if (delim) (void)parlio_del_rx_delimiter(delim);
    (void)parlio_del_rx_unit(rx);
    return err;
}

static uint32_t hash(const uint8_t *data, size_t n)
{
    uint32_t h = 2166136261u;
    for (size_t i = 0; i < n; ++i) h = (h ^ data[i]) * 16777619u;
    return h;
}

esp_err_t c5vrx2_issue11_capture(void)
{
    uint8_t *raw = heap_caps_malloc(RAW_BYTES, MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);
    uint8_t *out = heap_caps_malloc(OUT_BYTES, MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);
    if (!raw || !out) { free(raw); free(out); return ESP_ERR_NO_MEM; }
    memset(raw, 0xa5, RAW_BYTES);
    memset(out, 0xa5, OUT_BYTES);
    uint32_t h[16] = {0x31314f43u, 1, sizeof(h), RAW_BYTES, OUT_BYTES,
                      40000000, 20000000};
    h[7] = h[8] = h[9] = UINT32_MAX; /* explicitly not run */
    const uint8_t diag[8] = {6, 7, 8, 9, 16, 17, 18, 19};
    for (unsigned i = 0; i < 8; ++i) {
        gpio_set_direction(iq_pins[i], GPIO_MODE_INPUT_OUTPUT);
        esp_rom_gpio_connect_out_signal(iq_pins[i], MODEM_DIAG0_IDX + diag[i], false, false);
    }
    ESP_LOGW("issue11", "bounded chronological RF capture, then PR10 DAC replay; NOT live output capture");
    esp_err_t err = receive(iq_pins, 40000000, raw, RAW_BYTES, true, &h[10]);
    h[7] = err;
    parlio_tx_unit_handle_t tx = NULL;
    bool decorated = false, enabled = false;
    if (err != ESP_OK) goto persist;
    const parlio_tx_unit_config_t tc = {
        .clk_src = PARLIO_CLK_SRC_DEFAULT, .clk_in_gpio_num = -1,
        .output_clk_freq_hz = 20000000, .data_width = 8,
        .data_gpio_nums = {23, 24, 11, 12, 8, 9, -1, -1},
        .clk_out_gpio_num = -1, .valid_gpio_num = -1,
        .trans_queue_depth = 1, .max_transfer_size = RAW_BYTES,
        .dma_burst_size = 32, .shift_edge = PARLIO_SHIFT_EDGE_NEG,
        .bit_pack_order = PARLIO_BIT_PACK_ORDER_LSB,
    };
    err = parlio_new_tx_unit(&tc, &tx);
    if (err != ESP_OK) goto persist;
    err = parlio_tx_unit_decorate_bitscrambler(tx);
    if (err != ESP_OK) goto cleanup;
    decorated = true;
    err = parlio_tx_unit_enable(tx);
    if (err != ESP_OK) goto cleanup;
    enabled = true;
    const parlio_transmit_config_t tr = {
        .idle_value = 20,
        .bitscrambler_program = c5vrx2_wbfm_q4_trajectory_program(),
        .flags.loop_transmission = true,
    };
    err = parlio_tx_unit_transmit(tx, raw, RAW_BYTES * 8u, &tr);
    h[8] = err;
    if (err == ESP_OK) {
        esp_rom_delay_us(2000); /* steady replay, persistent BS state */
        err = receive(dac_pins, 20000000, out, OUT_BYTES, false, &h[11]);
        h[9] = err;
    }
cleanup:
    if (enabled) (void)parlio_tx_unit_disable(tx);
    if (decorated) (void)parlio_tx_unit_undecorate_bitscrambler(tx);
    if (tx) (void)parlio_del_tx_unit(tx);
persist:
    h[12] = hash(raw, RAW_BYTES);
    h[13] = hash(out, OUT_BYTES);
    h[14] = err;
    const esp_partition_t *p = esp_partition_find_first(ESP_PARTITION_TYPE_DATA, 0x42, "diagcap");
    esp_err_t saved = p ? ESP_OK : ESP_ERR_NOT_FOUND;
    const size_t total = sizeof(h) + RAW_BYTES + OUT_BYTES;
    size_t erase = p ? ((total + p->erase_size - 1) / p->erase_size) * p->erase_size : 0;
    if (p && erase > p->size) saved = ESP_ERR_INVALID_SIZE;
    if (saved == ESP_OK) saved = esp_partition_erase_range(p, 0, erase);
    if (saved == ESP_OK) saved = esp_partition_write(p, sizeof(h), raw, RAW_BYTES);
    if (saved == ESP_OK) saved = esp_partition_write(p, sizeof(h) + RAW_BYTES, out, OUT_BYTES);
    if (saved == ESP_OK) saved = esp_partition_write(p, 0, h, sizeof(h));
    uint8_t check[256];
    for (size_t pos = 0; saved == ESP_OK && pos < total; pos += sizeof(check)) {
        size_t n = total - pos < sizeof(check) ? total - pos : sizeof(check);
        saved = esp_partition_read(p, pos, check, n);
        for (size_t j = 0; saved == ESP_OK && j < n; ++j) {
            size_t k = pos + j;
            uint8_t expected = k < sizeof(h) ? ((uint8_t *)h)[k] :
                k < sizeof(h) + RAW_BYTES ? raw[k - sizeof(h)] : out[k - sizeof(h) - RAW_BYTES];
            if (check[j] != expected) saved = ESP_FAIL;
        }
    }
    ESP_LOGW("issue11", "raw=%lu tx=%lu output=%lu result=%lu flash_verify=%s raw_hash=%08lx output_hash=%08lx",
             h[7], h[8], h[9], h[14], esp_err_to_name(saved), h[12], h[13]);
    free(raw); free(out);
    return saved != ESP_OK ? saved : err;
}
#else
typedef int c5vrx2_issue11_capture_unused_t;
#endif
