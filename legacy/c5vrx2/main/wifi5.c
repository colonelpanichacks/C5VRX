#include "wifi5.h"

#include <stdint.h>

#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_wifi.h"

#include "continuous_iq.h"

#define REG32(a) (*(volatile uint32_t *)(uintptr_t)(a))
#define MAC_TXQ0_CONF 0x600a4d6cu
#define MAC_TXQ_STRIDE 0x10u
#define MAC_TXQ_ENABLE 0x80000000u
#define MAC_TXQ_COUNT 5u

/* Internal but globally exported by the pinned ESP32-C5 IDF 6.0.1 pp
 * library. It performs the driver's own bookkeeping before the hardware
 * queue gates below are made receive-only. */
extern int lmac_stop_hw_txq(void);

static const char *TAG = "c5vrx2_wifi5";
static bool s_lmac_tx_stopped;

bool c5vrx2_wifi5_tx_is_quiescent(void)
{
    for (unsigned queue = 0; queue < MAC_TXQ_COUNT; ++queue) {
        if ((REG32(MAC_TXQ0_CONF - queue * MAC_TXQ_STRIDE) &
             MAC_TXQ_ENABLE) != 0u)
            return false;
    }
    return true;
}

esp_err_t c5vrx2_wifi5_lock_rx_only(void)
{
    /* The vendor bookkeeping call is not documented as re-entrant. Run it
     * once, while Wi-Fi owns its complete normal memory view. Repeated arm
     * checks only close the hardware gates again and verify their state. */
    if (!s_lmac_tx_stopped) {
        (void)lmac_stop_hw_txq();
        s_lmac_tx_stopped = true;
    }
    for (unsigned queue = 0; queue < MAC_TXQ_COUNT; ++queue)
        REG32(MAC_TXQ0_CONF - queue * MAC_TXQ_STRIDE) &= ~MAC_TXQ_ENABLE;
    __asm__ __volatile__("fence iorw, iorw" ::: "memory");
    return c5vrx2_wifi5_tx_is_quiescent() ? ESP_OK : ESP_ERR_INVALID_STATE;
}

esp_err_t c5vrx2_wifi5_start_a1(void)
{
    continuous_iq_debug_mark(301u);
    ESP_LOGI(TAG, "startup 301: esp_netif_init");
    esp_err_t err = esp_netif_init();
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) return err;
    continuous_iq_debug_mark(302u);
    ESP_LOGI(TAG, "startup 302: esp_event_loop_create_default");
    err = esp_event_loop_create_default();
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) return err;

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    continuous_iq_debug_mark(303u);
    ESP_LOGI(TAG, "startup 303: esp_wifi_init");
    if ((err = esp_wifi_init(&cfg)) != ESP_OK) return err;
    continuous_iq_debug_mark(304u);
    ESP_LOGI(TAG, "startup 304: esp_wifi_set_storage");
    if ((err = esp_wifi_set_storage(WIFI_STORAGE_RAM)) != ESP_OK) return err;
    continuous_iq_debug_mark(305u);
    ESP_LOGI(TAG, "startup 305: esp_wifi_set_mode");
    if ((err = esp_wifi_set_mode(WIFI_MODE_STA)) != ESP_OK) return err;
    continuous_iq_debug_mark(306u);
    ESP_LOGI(TAG, "startup 306: esp_wifi_start");
    if ((err = esp_wifi_start()) != ESP_OK) return err;

#if CONFIG_SOC_WIFI_SUPPORT_5G
    continuous_iq_debug_mark(307u);
    ESP_LOGI(TAG, "startup 307: esp_wifi_set_band_mode");
    if ((err = esp_wifi_set_band_mode(WIFI_BAND_MODE_5G_ONLY)) != ESP_OK)
        return err;
#else
    return ESP_ERR_NOT_SUPPORTED;
#endif

    /* The pre-trigger producer requires the RX/PHY clock domain to remain
     * alive continuously. Treat failure to disable Wi-Fi power saving as a
     * hard startup error instead of silently accepting a gated frontend. */
    continuous_iq_debug_mark(308u);
    ESP_LOGI(TAG, "startup 308: esp_wifi_set_ps");
    if ((err = esp_wifi_set_ps(WIFI_PS_NONE)) != ESP_OK) return err;
    wifi_protocols_t protocols = {
        .ghz_2g = WIFI_PROTOCOL_11B | WIFI_PROTOCOL_11G |
                  WIFI_PROTOCOL_11N | WIFI_PROTOCOL_11AX,
        .ghz_5g = WIFI_PROTOCOL_11A | WIFI_PROTOCOL_11N,
    };
    continuous_iq_debug_mark(309u);
    ESP_LOGI(TAG, "startup 309: esp_wifi_set_protocols");
    if ((err = esp_wifi_set_protocols(WIFI_IF_STA, &protocols)) != ESP_OK)
        return err;

    wifi_bandwidths_t bandwidths = {
        .ghz_2g = WIFI_BW20,
        .ghz_5g = WIFI_BW40,
    };
    continuous_iq_debug_mark(310u);
    ESP_LOGI(TAG, "startup 310: esp_wifi_set_bandwidths(WIFI_BW40)");
    err = esp_wifi_set_bandwidths(WIFI_IF_STA, &bandwidths);
    if (err != ESP_OK) {
        bandwidths.ghz_5g = WIFI_BW20;
        if ((err = esp_wifi_set_bandwidths(WIFI_IF_STA, &bandwidths)) != ESP_OK)
            return err;
    }

    /* A1 is exactly Wi-Fi channel 173 = 5865 MHz on the C5 public driver. */
    continuous_iq_debug_mark(311u);
    ESP_LOGI(TAG, "startup 311: esp_wifi_set_channel(173)");
    if ((err = esp_wifi_set_channel(173, WIFI_SECOND_CHAN_NONE)) != ESP_OK)
        return err;
    continuous_iq_debug_mark(312u);
    ESP_LOGI(TAG, "startup 312: esp_wifi_set_promiscuous");
    if ((err = esp_wifi_set_promiscuous(true)) != ESP_OK) return err;

    /* No esp_wifi_connect(), scan, AP or network interface is ever created.
     * Quiesce all five LMAC TX queues as an additional hard guarantee that
     * the selected TX_START dump trigger cannot be satisfied by this STA. */
    continuous_iq_debug_mark(313u);
    ESP_LOGI(TAG, "startup 313: lock RX-only");
    if ((err = c5vrx2_wifi5_lock_rx_only()) != ESP_OK) return err;

    uint8_t primary = 0;
    wifi_second_chan_t secondary = WIFI_SECOND_CHAN_NONE;
    continuous_iq_debug_mark(314u);
    ESP_LOGI(TAG, "startup 314: verify channel");
    if ((err = esp_wifi_get_channel(&primary, &secondary)) != ESP_OK) return err;
    if (primary != 173) return ESP_ERR_INVALID_STATE;

    continuous_iq_debug_mark(315u);

    ESP_LOGW(TAG,
             "A1 RF frontend active: 5865 MHz/ch173; MAC TX queues disabled; "
             "IQ producer is not VTX-gated");
    return ESP_OK;
}
