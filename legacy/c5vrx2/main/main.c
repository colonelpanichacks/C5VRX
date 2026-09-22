#include "esp_log.h"
#include "driver/gpio.h"
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "wifi5.h"
#include "calibration.h"
#include "continuous_iq.h"
#include "diagnostics.h"
#include "realtime.h"
#include "rf_dump.h"
#include "startup_trace.h"
#include "wbfm_q4.h"
#include "tx_80m_oracle.h"
#if CONFIG_C5VRX2_MODE_LINEAR80_ORACLE
esp_err_t c5vrx2_linear80_oracle_run(void);
#endif

#if CONFIG_C5VRX2_ISSUE11_CAPTURE
#include <stdio.h>
esp_err_t c5vrx2_issue11_capture(void);
#endif

static const char *TAG = "c5vrx2";

#define XIAO_USER_LED GPIO_NUM_27

static void init_hardware_marker(void)
{
    const gpio_config_t cfg = {
        .pin_bit_mask = 1ULL << XIAO_USER_LED,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    if (gpio_config(&cfg) == ESP_OK)
        gpio_set_level(XIAO_USER_LED, 0); /* active-low: entering startup */
}

static void report_fatal_forever(const char *stage, esp_err_t err)
{
    for (;;) {
        ESP_LOGE(TAG, "STARTUP STOP stage=%s error=%s; device remains alive",
                 stage, esp_err_to_name(err));
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

static esp_err_t init_nvs(void)
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES ||
        err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        err = nvs_flash_erase();
        if (err == ESP_OK) err = nvs_flash_init();
    }
    return err;
}

void app_main(void)
{
    init_hardware_marker();
    /* Do this before the first scheduler delay. A CPU-only reset can leave
     * the autonomous modem dump domain armed and its SRAM view non-CPU. */
    c5vrx2_rf_dump_boot_sanitize();
#if CONFIG_C5VRX2_LIVE_SNAPSHOT_ONCE
    /* RTC_NOINIT survives an application reset.  Snapshot bring-up can lose
     * USB as soon as the modem source is enabled, so preserve the previous
     * run's last fine-grained RF marker before this boot overwrites it. */
    const uint32_t retained_rf_marker = continuous_iq_debug_last_stage();
#endif
#if CONFIG_C5VRX2_MODE_AV_STATIC
    ESP_LOGW(TAG, "C5VRX-2: static resistor-DAC diagnostic boot");
    const esp_err_t err = c5vrx2_av_static_diagnostic_start(
        CONFIG_C5VRX2_AV_STATIC_CODE);
    if (err != ESP_OK) ESP_LOGE(TAG, "AV diagnostic failed: %s",
                                esp_err_to_name(err));
    return;
#elif CONFIG_C5VRX2_MODE_AV_PAL_MONO || CONFIG_C5VRX2_MODE_AV_PAL_COLOR
    ESP_LOGW(TAG, "C5VRX-2: synthetic PAL diagnostic boot");
#if CONFIG_C5VRX2_MODE_AV_PAL_COLOR
    const bool colour = true;
#else
    const bool colour = false;
#endif
    const esp_err_t err = c5vrx2_av_pal_diagnostic_start(
        colour);
    if (err != ESP_OK) ESP_LOGE(TAG, "PAL diagnostic failed: %s",
                                esp_err_to_name(err));
    return;
#elif CONFIG_C5VRX2_MODE_TX_80M_ORACLE
    ESP_LOGW(TAG, "C5VRX-2: PARLIO TX 80 MHz hardware oracle boot");
    vTaskDelay(pdMS_TO_TICKS(1500));
    const esp_err_t err = c5vrx2_tx_80m_oracle_run();
    if (err != ESP_OK) ESP_LOGE(TAG, "80M oracle failed: %s", esp_err_to_name(err));
    return;
#else
    ESP_LOGW(TAG, "C5VRX-2: direct TX-BitScrambler WBFM receiver boot");

    /* Native USB disconnects while the XIAO resets and is enumerated again
     * only after the application has started. Give the host a short window
     * to reopen the port before entering private Wi-Fi/PHY initialization;
     * otherwise an early stall is completely silent on USB. The retained
     * marker also identifies the last completed startup operation. */
    vTaskDelay(pdMS_TO_TICKS(1500));
    ESP_LOGW(TAG, "previous retained startup marker=%u",
             (unsigned)continuous_iq_debug_last_stage());

    esp_err_t err = init_nvs();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "NVS init failed: %s", esp_err_to_name(err));
        report_fatal_forever("nvs", err);
    }

    c5vrx2_calibration_load();
#if !CONFIG_C5VRX2_MODE_MODEM_CAPTURE
    c5vrx2_trace_begin();
#endif
    c5vrx2_trace_stage(2u, ESP_OK);
#if CONFIG_C5VRX2_LIVE_SNAPSHOT_ONCE
    c5vrx2_trace_stage_detail(4u, ESP_OK, retained_rf_marker, 0u, 0u);
    if (retained_rf_marker >= 401u && retained_rf_marker <= 499u) {
        /* Recovery boot: leave the persisted marker intact and remain alive.
         * The first post-flash boot has no valid marker and runs normally. */
        gpio_set_level(XIAO_USER_LED, 1);
        for (;;) vTaskDelay(portMAX_DELAY);
    }
#endif

#if CONFIG_C5VRX2_WBFM_SELFTEST_ONCE
    /* The persisted record is authoritative for the temporary input/LUT
     * address oracle. RF and the DAC are not involved. */
    const unsigned pulses = c5vrx2_wbfm_q4_selftest_once();
    for (unsigned pulse = 0u; pulse < pulses; ++pulse) {
        gpio_set_level(XIAO_USER_LED, 1);
        vTaskDelay(pdMS_TO_TICKS(80));
        gpio_set_level(XIAO_USER_LED, 0);
        vTaskDelay(pdMS_TO_TICKS(80));
    }
    gpio_set_level(XIAO_USER_LED, 1);
    return;
#endif

#if CONFIG_C5VRX2_MODE_TX_WBFM_TEST
    /* This bounded test contains no RF input. Run it before Wi-Fi/PHY setup,
     * so a private RF-startup failure cannot hide the PARLIO TX decorator
     * result. The diagnostic persists its byte comparison before returning. */
    err = c5vrx2_tx_wbfm_diagnostic_run();
    gpio_set_level(XIAO_USER_LED, err == ESP_OK ? 1 : 0);
    if (err != ESP_OK) report_fatal_forever("tx_wbfm_test", err);
    ESP_LOGW(TAG, "TX WBFM hardware test passed and was persisted");
    return;
#endif

#if CONFIG_C5VRX2_MODE_LINEAR80_ORACLE
    err = c5vrx2_linear80_oracle_run();
    gpio_set_level(XIAO_USER_LED, err == ESP_OK ? 0 : 1);
    ESP_LOGW(TAG, "LINEAR80 ORACLE COMPLETE: %s; RF remained off",esp_err_to_name(err));
    /* This diagnostic is one-shot, not a receiver. Keep failure visibly
     * distinguishable from power loss, without repeating tests/flash writes. */
    for (;;) {
        if (err != ESP_OK) {
            gpio_set_level(XIAO_USER_LED, 0);
            vTaskDelay(pdMS_TO_TICKS(150));
            gpio_set_level(XIAO_USER_LED, 1);
        }
        vTaskDelay(pdMS_TO_TICKS(1850));
    }
#endif

    err = c5vrx2_wifi5_start_a1();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "A1 RF init failed: %s", esp_err_to_name(err));
        report_fatal_forever("wifi5_start_a1", err);
    }
    /* A visible one-second dark interval proves that Wi-Fi/PHY init returned.
     * This is deliberately synchronous startup-only instrumentation: it adds
     * no task, timer or interrupt to the realtime path. */
    gpio_set_level(XIAO_USER_LED, 1);
    vTaskDelay(pdMS_TO_TICKS(1000));
    c5vrx2_trace_stage(3u, ESP_OK);

#if CONFIG_C5VRX2_ISSUE11_CAPTURE
    ESP_LOGW(TAG, "ISSUE11 READY: countdown 3 seconds before capture...");
    for (int i = 0; i < 3; ++i) {
        gpio_set_level(XIAO_USER_LED, 0); /* LED on */
        vTaskDelay(pdMS_TO_TICKS(200));
        gpio_set_level(XIAO_USER_LED, 1); /* LED off */
        vTaskDelay(pdMS_TO_TICKS(800));
    }
    gpio_set_level(XIAO_USER_LED, 0); /* LED on during capture */
    err = c5vrx2_issue11_capture();
    gpio_set_level(XIAO_USER_LED, 1); /* LED off */
    for (int i = 0; i < 10; ++i) {
        gpio_set_level(XIAO_USER_LED, 0);
        vTaskDelay(pdMS_TO_TICKS(50));
        gpio_set_level(XIAO_USER_LED, 1);
        vTaskDelay(pdMS_TO_TICKS(50));
    }
    ESP_LOGW(TAG, "ISSUE11 CAPTURE COMPLETE (err=%s). VTX can be turned off now.", esp_err_to_name(err));
    for (;;) vTaskDelay(portMAX_DELAY);
#elif CONFIG_C5VRX2_MODE_RF_ORACLE
    err = c5vrx2_rf_oracle_diagnostic_start();
#elif CONFIG_C5VRX2_MODE_RF_WRAP
    err = c5vrx2_rf_wrap_diagnostic_run();
#elif CONFIG_C5VRX2_MODE_RF_DMA || CONFIG_C5VRX2_MODE_RF_DMA_CPU_OWNED
    err = c5vrx2_rf_dma_diagnostic_run();
#elif CONFIG_C5VRX2_MODE_MODEM_DIAG
    err = c5vrx2_modem_diag_diagnostic_run();
#elif CONFIG_C5VRX2_MODE_MODEM_CAPTURE
    err = c5vrx2_modem_capture_diagnostic_run();
#elif CONFIG_C5VRX2_MODE_MODEM_PARLIO || CONFIG_C5VRX2_MODE_MODEM_WBFM
    err = c5vrx2_modem_parlio_diagnostic_run();
#else
    err = c5vrx2_realtime_start();
#endif
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "selected receiver mode failed: %s",
                 esp_err_to_name(err));
#if CONFIG_C5VRX2_MODE_LIVE
        /* USB Serial/JTAG can disappear while the experimental RF SRAM
         * ownership is active. Use the already-proven AV path as an
         * unambiguous bring-up fault indicator: visible monochrome bars mean
         * live startup returned an error before a continuous output existed. */
        const esp_err_t av_err = c5vrx2_av_pal_diagnostic_start(false);
        if (av_err == ESP_OK) {
            ESP_LOGE(TAG, "LIVE START FAILED: showing PAL bars as fault code");
            return;
        }
        ESP_LOGE(TAG, "visual fault indicator failed: %s",
                 esp_err_to_name(av_err));
#endif
        report_fatal_forever("receiver", err);
    }
    gpio_set_level(XIAO_USER_LED, 0); /* realtime pipeline started */
    ESP_LOGI(TAG, "selected receiver mode completed/started; main task released");
#endif
}
