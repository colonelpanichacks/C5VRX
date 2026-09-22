#include "tx_80m_oracle.h"
#include <stdio.h>
#include <string.h>
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_heap_caps.h"
#include "driver/gpio.h"
#include "driver/parlio_tx.h"
#include "driver/parlio_rx.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define TAG "oracle_80m"
#define XIAO_USER_LED GPIO_NUM_27
#define TEST_BYTES 80000u
#define RX_CAPTURE_BYTES 4000u

static const int s_dac_pins[8] = {23, 24, 11, 12, 8, 9, -1, -1};

esp_err_t c5vrx2_tx_80m_oracle_run(void)
{
    ESP_LOGI(TAG, "==================================================");
    ESP_LOGI(TAG, "STARTING PARLIO TX 80 MHz HARDWARE ORACLE");
    ESP_LOGI(TAG, "==================================================");

    // 1. Configure GPIO drive strength and direction
    for (int i = 0; i < 6; ++i) {
        gpio_set_direction((gpio_num_t)s_dac_pins[i], GPIO_MODE_INPUT_OUTPUT);
        gpio_set_drive_capability((gpio_num_t)s_dac_pins[i], GPIO_DRIVE_CAP_3);
    }

    // 2. Allocate DMA buffer with alternating pattern: 0x00, 0x3F, 0x00, 0x3F...
    uint8_t *tx_buf = heap_caps_malloc(TEST_BYTES, MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);
    if (!tx_buf) {
        ESP_LOGE(TAG, "Failed to allocate %u bytes TX DMA buffer", TEST_BYTES);
        return ESP_ERR_NO_MEM;
    }
    for (size_t i = 0; i < TEST_BYTES; ++i) {
        tx_buf[i] = (i & 1) ? 0x3F : 0x00;
    }

    // 3. Initialize PARLIO TX at 80 MHz (80000000 Hz)
    parlio_tx_unit_handle_t tx = NULL;
    const parlio_tx_unit_config_t tx_cfg = {
        .clk_src = PARLIO_CLK_SRC_DEFAULT, // PLL_F240M
        .clk_in_gpio_num = -1,
        .input_clk_src_freq_hz = 0u,
        .output_clk_freq_hz = 80000000u,
        .data_width = 8u,
        .data_gpio_nums = {23, 24, 11, 12, 8, 9, -1, -1},
        .clk_out_gpio_num = -1,
        .valid_gpio_num = -1,
        .valid_start_delay = 0,
        .valid_stop_delay = 0,
        .trans_queue_depth = 2u,
        .max_transfer_size = TEST_BYTES,
        .dma_burst_size = 32u,
        .shift_edge = PARLIO_SHIFT_EDGE_NEG,
        .bit_pack_order = PARLIO_BIT_PACK_ORDER_LSB,
    };

    esp_err_t err = parlio_new_tx_unit(&tx_cfg, &tx);
    ESP_LOGI(TAG, "parlio_new_tx_unit(80 MHz): %s (code 0x%x)", esp_err_to_name(err), err);
    if (err != ESP_OK) {
        free(tx_buf);
        return err;
    }

    err = parlio_tx_unit_enable(tx);
    ESP_LOGI(TAG, "parlio_tx_unit_enable: %s", esp_err_to_name(err));
    if (err != ESP_OK) {
        parlio_del_tx_unit(tx);
        free(tx_buf);
        return err;
    }

    // 4. Test timed one-shot transfer to measure physical rate
    const parlio_transmit_config_t tr_cfg_oneshot = {
        .idle_value = 0,
        .bitscrambler_program = NULL,
        .flags.loop_transmission = false,
    };

    ESP_LOGI(TAG, "Transmitting %u bytes at requested 80 MHz...", TEST_BYTES);
    int64_t t0 = esp_timer_get_time();
    err = parlio_tx_unit_transmit(tx, tx_buf, TEST_BYTES * 8u, &tr_cfg_oneshot);
    if (err == ESP_OK) {
        err = parlio_tx_unit_wait_all_done(tx, 500); // 500 ms timeout
    }
    int64_t t1 = esp_timer_get_time();
    int64_t dt_us = t1 - t0;

    ESP_LOGI(TAG, "One-shot transmit result: %s, elapsed time = %lld us", esp_err_to_name(err), (long long)dt_us);
    if (dt_us > 0) {
        double actual_rate_mbs = ((double)TEST_BYTES) / ((double)dt_us);
        ESP_LOGI(TAG, "===> MEASURED TX THROUGHPUT: %.2f MB/s (%.2f MS/s)", actual_rate_mbs, actual_rate_mbs);
        if (actual_rate_mbs > 70.0 && actual_rate_mbs < 90.0) {
            ESP_LOGW(TAG, "===> SUCCESS: PARLIO TX IS CLOCKING AT FULL 80 MHz (expected ~80 MB/s, got %.2f MB/s)!", actual_rate_mbs);
        } else if (actual_rate_mbs > 35.0 && actual_rate_mbs < 45.0) {
            ESP_LOGW(TAG, "===> RATE RESTRICTED: PARLIO TX clock was divided to 40 MHz (got %.2f MB/s)", actual_rate_mbs);
        } else {
            ESP_LOGW(TAG, "===> UNEXPECTED RATE: %.2f MB/s", actual_rate_mbs);
        }
    }

    // 5. Test loopback capture via PARLIO RX at 80 MS/s
    uint8_t *rx_buf = heap_caps_malloc(RX_CAPTURE_BYTES, MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);
    if (rx_buf) {
        memset(rx_buf, 0xAA, RX_CAPTURE_BYTES);
        parlio_rx_unit_handle_t rx = NULL;
        parlio_rx_delimiter_handle_t delim = NULL;
        const parlio_rx_unit_config_t rx_cfg = {
            .trans_queue_depth = 1,
            .max_recv_size = RX_CAPTURE_BYTES,
            .dma_burst_size = 32,
            .data_width = 8,
            .clk_src = PARLIO_CLK_SRC_DEFAULT,
            .exp_clk_freq_hz = 80000000u,
            .clk_in_gpio_num = -1,
            .clk_out_gpio_num = -1,
            .valid_gpio_num = -1,
            .flags.free_clk = true,
            .data_gpio_nums = {23, 24, 11, 12, 8, 9, -1, -1},
        };
        esp_err_t rx_err = parlio_new_rx_unit(&rx_cfg, &rx);
        ESP_LOGI(TAG, "parlio_new_rx_unit(80 MHz free_clk): %s", esp_err_to_name(rx_err));
        if (rx_err == ESP_OK) {
            const parlio_rx_soft_delimiter_config_t dc = {
                .sample_edge = PARLIO_SAMPLE_EDGE_POS,
                .bit_pack_order = PARLIO_BIT_PACK_ORDER_LSB,
                .eof_data_len = RX_CAPTURE_BYTES,
            };
            rx_err = parlio_new_rx_soft_delimiter(&dc, &delim);
            if (rx_err == ESP_OK) {
                rx_err = parlio_rx_unit_enable(rx, true);
                if (rx_err == ESP_OK) {
                    const parlio_receive_config_t rc = {.delimiter = delim};
                    rx_err = parlio_rx_unit_receive(rx, rx_buf, RX_CAPTURE_BYTES, &rc);
                    if (rx_err == ESP_OK) {
                        // Start TX in continuous loop mode while RX captures
                        const parlio_transmit_config_t tr_cfg_loop = {
                            .idle_value = 0,
                            .bitscrambler_program = NULL,
                            .flags.loop_transmission = true,
                        };
                        parlio_tx_unit_transmit(tx, tx_buf, TEST_BYTES * 8u, &tr_cfg_loop);
                        parlio_rx_soft_delimiter_start_stop(rx, delim, true);
                        rx_err = parlio_rx_unit_wait_all_done(rx, 200);
                        ESP_LOGI(TAG, "PARLIO RX 80 MHz capture done: %s", esp_err_to_name(rx_err));
                    }
                }
            }
        }
        if (rx_err == ESP_OK) {
            uint32_t count_00 = 0, count_3f = 0, count_other = 0;
            for (size_t i = 0; i < RX_CAPTURE_BYTES; ++i) {
                uint8_t val = rx_buf[i] & 0x3F;
                if (val == 0x00) count_00++;
                else if (val == 0x3F) count_3f++;
                else count_other++;
            }
            ESP_LOGI(TAG, "RX Samples (4000 total): [0x00: %u] [0x3F: %u] [Transitions/Intermediate: %u]",
                     (unsigned)count_00, (unsigned)count_3f, (unsigned)count_other);
            ESP_LOGI(TAG, "First 16 captured samples: %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x",
                     rx_buf[0]&0x3f, rx_buf[1]&0x3f, rx_buf[2]&0x3f, rx_buf[3]&0x3f,
                     rx_buf[4]&0x3f, rx_buf[5]&0x3f, rx_buf[6]&0x3f, rx_buf[7]&0x3f,
                     rx_buf[8]&0x3f, rx_buf[9]&0x3f, rx_buf[10]&0x3f, rx_buf[11]&0x3f,
                     rx_buf[12]&0x3f, rx_buf[13]&0x3f, rx_buf[14]&0x3f, rx_buf[15]&0x3f);
        }
        if (delim) parlio_del_rx_delimiter(delim);
        if (rx) { parlio_rx_unit_disable(rx); parlio_del_rx_unit(rx); }
        free(rx_buf);
    }

    // 6. Leave TX running in continuous loop mode at 80 MHz!
    const parlio_transmit_config_t tr_cfg_loop = {
        .idle_value = 0,
        .bitscrambler_program = NULL,
        .flags.loop_transmission = true,
    };
    err = parlio_tx_unit_transmit(tx, tx_buf, TEST_BYTES * 8u, &tr_cfg_loop);
    ESP_LOGI(TAG, "Continuous 80 MHz TX loop transmission active on GPIOs 23, 24, 11, 12, 8, 9");

    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(1000));
        gpio_set_level(XIAO_USER_LED, 0);
        vTaskDelay(pdMS_TO_TICKS(50));
        gpio_set_level(XIAO_USER_LED, 1);
        ESP_LOGI(TAG, "80 MHz TX alive: 40 MHz square wave on DAC GPIOs (12.5 ns/sample, loop active)");
    }
    return ESP_OK;
}
