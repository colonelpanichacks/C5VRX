/* SPDX-License-Identifier: GPL-3.0-only */

#include "c5vrx_wifi5.h"

#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "sdkconfig.h"

static const char *TAG = "c5vrx_wifi5";
static bool s_initialized;
static bool s_started;
static bool s_promiscuous;
static uint16_t s_requested_channel;
static bool s_ht40;

esp_err_t c5vrx_wifi5_init(void)
{
    if (s_initialized) {
        return ESP_OK;
    }

    esp_err_t err = esp_netif_init();
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        return err;
    }

    err = esp_event_loop_create_default();
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        return err;
    }

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    err = esp_wifi_init(&cfg);
    if (err != ESP_OK) {
        return err;
    }

    err = esp_wifi_set_storage(WIFI_STORAGE_RAM);
    if (err != ESP_OK) {
        return err;
    }
    err = esp_wifi_set_mode(WIFI_MODE_STA);
    if (err != ESP_OK) {
        return err;
    }
    err = esp_wifi_start();
    if (err != ESP_OK) {
        return err;
    }
    s_started = true;

#if CONFIG_SOC_WIFI_SUPPORT_5G
    err = esp_wifi_set_band_mode(WIFI_BAND_MODE_5G_ONLY);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to select 5 GHz-only Wi-Fi band: %s", esp_err_to_name(err));
        return err;
    }
#else
    ESP_LOGE(TAG, "This target does not advertise CONFIG_SOC_WIFI_SUPPORT_5G");
    return ESP_ERR_NOT_SUPPORTED;
#endif

    ESP_ERROR_CHECK_WITHOUT_ABORT(esp_wifi_set_ps(WIFI_PS_NONE));

    s_initialized = true;
    ESP_LOGI(TAG, "Public ESP-IDF Wi-Fi driver initialized in 5 GHz-only mode");
    return ESP_OK;
}

static esp_err_t configure_5g_bandwidth(bool request_ht40, bool *active_ht40)
{
    if (!active_ht40) {
        return ESP_ERR_INVALID_ARG;
    }
    *active_ht40 = false;

    /* ESP32-C5 only permits 40 MHz with 802.11an. The default 5 GHz
     * protocol set also enables 11ac/11ax, for which ESP-IDF rejects BW40. */
    wifi_protocols_t protocols = {
        .ghz_2g = WIFI_PROTOCOL_11B | WIFI_PROTOCOL_11G |
                  WIFI_PROTOCOL_11N | WIFI_PROTOCOL_11AX,
        .ghz_5g = WIFI_PROTOCOL_11A | WIFI_PROTOCOL_11N,
    };
    esp_err_t err = esp_wifi_set_protocols(WIFI_IF_STA, &protocols);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to select 5 GHz 802.11a/n protocols: %s",
                 esp_err_to_name(err));
        return err;
    }

    wifi_protocols_t protocol_readback = {0};
    err = esp_wifi_get_protocols(WIFI_IF_STA, &protocol_readback);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read back Wi-Fi protocols: %s",
                 esp_err_to_name(err));
        return err;
    }
    const uint16_t required_5g = WIFI_PROTOCOL_11A | WIFI_PROTOCOL_11N;
    if ((protocol_readback.ghz_5g & required_5g) != required_5g ||
        (protocol_readback.ghz_5g & (WIFI_PROTOCOL_11AC | WIFI_PROTOCOL_11AX)) != 0) {
        ESP_LOGE(TAG, "Unexpected 5 GHz protocol readback: 0x%04x",
                 (unsigned)protocol_readback.ghz_5g);
        return ESP_ERR_INVALID_STATE;
    }

    wifi_bandwidths_t bandwidths = {
        .ghz_2g = WIFI_BW20,
        .ghz_5g = request_ht40 ? WIFI_BW40 : WIFI_BW20,
    };
    err = esp_wifi_set_bandwidths(WIFI_IF_STA, &bandwidths);
    if (err != ESP_OK && request_ht40) {
        ESP_LOGW(TAG,
                 "40 MHz bandwidth rejected (%s); falling back to 20 MHz without aborting",
                 esp_err_to_name(err));
        bandwidths.ghz_5g = WIFI_BW20;
        err = esp_wifi_set_bandwidths(WIFI_IF_STA, &bandwidths);
    }
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure 5 GHz bandwidth: %s",
                 esp_err_to_name(err));
        return err;
    }

    wifi_bandwidths_t bandwidth_readback = {0};
    err = esp_wifi_get_bandwidths(WIFI_IF_STA, &bandwidth_readback);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read back 5 GHz bandwidth: %s",
                 esp_err_to_name(err));
        return err;
    }

    *active_ht40 = bandwidth_readback.ghz_5g == WIFI_BW40;
    if (request_ht40 && !*active_ht40) {
        ESP_LOGW(TAG, "HT40 requested but readback is 20 MHz; continuing safely");
    }
    ESP_LOGI(TAG, "5 GHz protocol=802.11a/n bandwidth=%s",
             *active_ht40 ? "40 MHz" : "20 MHz");
    return ESP_OK;
}

esp_err_t c5vrx_wifi5_start(uint16_t channel, bool ht40)
{
    esp_err_t err = c5vrx_wifi5_init();
    if (err != ESP_OK) {
        return err;
    }
    if (channel > UINT8_MAX) {
        return ESP_ERR_INVALID_ARG;
    }

    bool active_ht40 = false;
    err = configure_5g_bandwidth(ht40, &active_ht40);
    if (err != ESP_OK) {
        return err;
    }

    err = esp_wifi_set_channel((uint8_t)channel, WIFI_SECOND_CHAN_NONE);
    if (err != ESP_OK) {
        ESP_LOGE(TAG,
                 "Failed to tune standard Wi-Fi channel %u: %s. The public driver enforces its configured regulatory domain.",
                 channel, esp_err_to_name(err));
        return err;
    }

    err = esp_wifi_set_promiscuous(true);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to enable promiscuous receive: %s", esp_err_to_name(err));
        return err;
    }

    s_requested_channel = channel;
    s_ht40 = active_ht40;
    s_promiscuous = true;

    uint8_t primary = 0;
    wifi_second_chan_t secondary = WIFI_SECOND_CHAN_NONE;
    err = esp_wifi_get_channel(&primary, &secondary);
    if (err == ESP_OK) {
        ESP_LOGW(TAG,
                 "5 GHz public-driver RX active: requested ch%u, readback ch%u, BW=%s, promiscuous=1",
                 channel, primary, active_ht40 ? "40 MHz" : "20 MHz");
    }
    return err;
}

esp_err_t c5vrx_wifi5_get_status(c5vrx_wifi5_status_t *out)
{
    if (!out) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!s_initialized || !s_started) {
        return ESP_ERR_INVALID_STATE;
    }

    uint8_t primary = 0;
    wifi_second_chan_t secondary = WIFI_SECOND_CHAN_NONE;
    const esp_err_t err = esp_wifi_get_channel(&primary, &secondary);
    if (err != ESP_OK) {
        return err;
    }

    *out = (c5vrx_wifi5_status_t) {
        .requested_channel = s_requested_channel,
        .active_primary_channel = primary,
        .ht40 = s_ht40,
        .promiscuous_enabled = s_promiscuous,
    };
    return ESP_OK;
}

esp_err_t c5vrx_wifi5_verify_dedicated_receiver(
    c5vrx_wifi5_dedicated_status_t *out)
{
    if (!out) return ESP_ERR_INVALID_ARG;
    *out = (c5vrx_wifi5_dedicated_status_t) {0};
    if (!s_initialized || !s_started) return ESP_ERR_INVALID_STATE;

    /* Ensure no association/background-maintenance session survives into
     * LIVE. A disconnected STA with promiscuous RX does not scan/connect on
     * its own because C5VRX never calls esp_wifi_connect(). */
    (void)esp_wifi_disconnect();

    wifi_band_mode_t band = WIFI_BAND_MODE_AUTO;
    wifi_ps_type_t ps = WIFI_PS_MIN_MODEM;
    wifi_mode_t mode = WIFI_MODE_NULL;
    bool promiscuous = false;
    wifi_ap_record_t ap = {0};
    esp_err_t err = esp_wifi_get_band_mode(&band);
    if (err == ESP_OK) err = esp_wifi_get_ps(&ps);
    if (err == ESP_OK) err = esp_wifi_get_mode(&mode);
    if (err == ESP_OK) err = esp_wifi_get_promiscuous(&promiscuous);
    const esp_err_t ap_err = esp_wifi_sta_get_ap_info(&ap);

    out->band_5g_only = band == WIFI_BAND_MODE_5G_ONLY;
    out->power_save_disabled = ps == WIFI_PS_NONE;
    out->promiscuous_enabled = promiscuous && mode == WIFI_MODE_STA;
    out->station_disconnected = ap_err != ESP_OK;
#if defined(CONFIG_BT_ENABLED) && CONFIG_BT_ENABLED
    out->bluetooth_compiled_out = false;
#else
    out->bluetooth_compiled_out = true;
#endif
#if defined(CONFIG_IEEE802154_ENABLED) && CONFIG_IEEE802154_ENABLED
    out->ieee802154_compiled_out = false;
#else
    out->ieee802154_compiled_out = true;
#endif
    out->dedicated = err == ESP_OK && out->band_5g_only &&
        out->power_save_disabled && out->promiscuous_enabled &&
        out->station_disconnected && out->bluetooth_compiled_out &&
        out->ieee802154_compiled_out;
    ESP_LOGI(TAG,
             "LIVE radio guard 5g_only=%u ps_off=%u promisc=%u disconnected=%u bt_off=%u 802154_off=%u dedicated=%u",
             out->band_5g_only, out->power_save_disabled,
             out->promiscuous_enabled, out->station_disconnected,
             out->bluetooth_compiled_out, out->ieee802154_compiled_out,
             out->dedicated);
    return out->dedicated ? ESP_OK :
           (err == ESP_OK ? ESP_ERR_INVALID_STATE : err);
}

void c5vrx_wifi5_stop(void)
{
    if (!s_initialized) {
        return;
    }
    if (s_promiscuous) {
        esp_wifi_set_promiscuous(false);
        s_promiscuous = false;
    }
    if (s_started) {
        esp_wifi_stop();
        s_started = false;
    }
    esp_wifi_deinit();
    s_initialized = false;
}
