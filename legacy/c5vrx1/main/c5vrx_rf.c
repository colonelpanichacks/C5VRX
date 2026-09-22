/* SPDX-License-Identifier: GPL-3.0-only */

#include "c5vrx_rf.h"

#include "esp_log.h"
#include "c5vrx_channels.h"
#include "c5vrx_phy_hacks.h"

static const char *TAG = "c5vrx_rf";
static bool s_initialized;
static bool s_running;

uint16_t c5vrx_wifi5_channel_to_mhz(uint16_t channel)
{
    return (uint16_t)(5000U + 5U * channel);
}

bool c5vrx_rf_has_direct_frequency_hook(void)
{
    return c5vrx_phy_has_direct_frequency_hook();
}

esp_err_t c5vrx_rf_init(void)
{
    if (s_initialized) {
        return ESP_OK;
    }

    esp_wifi_power_domain_on();
    esp_phy_rftest_config(1);
    esp_phy_rftest_init();

    s_initialized = true;
    ESP_LOGI(TAG, "RF certification/test PHY initialized");
    ESP_LOGI(TAG, "direct-frequency hook phy_set_freq: %s",
             c5vrx_rf_has_direct_frequency_hook() ? "present" : "not linked");
    return ESP_OK;
}

esp_err_t c5vrx_rf_set_frequency_mhz(uint16_t mhz)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    return c5vrx_phy_set_frequency_mhz(mhz);
}

esp_err_t c5vrx_rf_start(const c5vrx_rx_config_t *cfg)
{
    if (!cfg) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!s_initialized) {
        esp_err_t err = c5vrx_rf_init();
        if (err != ESP_OK) {
            return err;
        }
    }

    /*
     * Important: the public ESP-IDF v6.0.2 esp_phy_cert_test.h contract
     * documents esp_phy_wifi_rx() channel values as 1..14. Older C5VRX
     * experiments passed standard 5 GHz Wi-Fi channel numbers here; that was
     * an unverified assumption. Refuse it now instead of presenting it as a
     * working 5 GHz API. The public Wi-Fi5 backend handles standard 5 GHz
     * channel bring-up.
     */
    if (cfg->wifi_channel < 1 || cfg->wifi_channel > 14) {
        ESP_LOGE(TAG,
                 "cert-test esp_phy_wifi_rx() is documented for channels 1-14; refusing unverified channel %u",
                 cfg->wifi_channel);
        return ESP_ERR_NOT_SUPPORTED;
    }

    if (s_running) {
        c5vrx_rf_stop();
    }

    ESP_LOGW(TAG,
             "Starting certification/test RX: channel=%u, HT40=%d, rate=0x%x",
             cfg->wifi_channel, cfg->ht40, (unsigned)cfg->rate);

    esp_phy_test_start_stop(3);
    esp_phy_cbw40m_en(cfg->ht40);
    esp_phy_wifi_rx(cfg->wifi_channel, cfg->rate);
    s_running = true;

    if (cfg->try_direct_frequency && cfg->center_mhz) {
        const esp_err_t tune_err = c5vrx_rf_set_frequency_mhz(cfg->center_mhz);
        if (tune_err != ESP_OK) {
            ESP_LOGW(TAG, "Direct retune failed: %s", esp_err_to_name(tune_err));
        }
    }

    return ESP_OK;
}

void c5vrx_rf_stop(void)
{
    if (!s_initialized) {
        return;
    }
    esp_phy_test_start_stop(0);
    s_running = false;
    ESP_LOGI(TAG, "RF-test RX stopped");
}

void c5vrx_rf_get_result(esp_phy_rx_result_t *out)
{
    if (!out) {
        return;
    }
    esp_phy_get_rx_result(out);
}
