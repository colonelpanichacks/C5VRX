/* SPDX-License-Identifier: GPL-3.0-only */

#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"
#include "esp_phy_cert_test.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint16_t wifi_channel;
    uint16_t center_mhz;
    bool ht40;
    bool try_direct_frequency;
    esp_phy_wifi_rate_t rate;
} c5vrx_rx_config_t;

esp_err_t c5vrx_rf_init(void);
esp_err_t c5vrx_rf_start(const c5vrx_rx_config_t *cfg);
void c5vrx_rf_stop(void);
void c5vrx_rf_get_result(esp_phy_rx_result_t *out);
uint16_t c5vrx_wifi5_channel_to_mhz(uint16_t channel);
bool c5vrx_rf_has_direct_frequency_hook(void);
esp_err_t c5vrx_rf_set_frequency_mhz(uint16_t mhz);

#ifdef __cplusplus
}
#endif
