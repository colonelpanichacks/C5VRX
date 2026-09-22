/* SPDX-License-Identifier: GPL-3.0-only */

#include "c5vrx_phy_hacks.h"

#include "esp_log.h"
#include "c5vrx_channels.h"

static const char *TAG = "c5vrx_phy_hacks";

/*
 * ESP32-C5 ESP-IDF v6.0.2 disassembly shows that phy_set_freq consumes
 * TWO arguments: a0 is forwarded into phy_chip_set_chan(), while a1 is
 * stored as the signed frequency-offset field before the channel change.
 *
 * An independent ESP32-C6 implementation (Hubble Network device SDK) uses
 * the same two-argument shape: phy_set_freq(uint16_t freq_mhz, int offset).
 * The C5 call remains undocumented, so keep it weak and explicitly opt-in.
 */
extern void phy_set_freq(uint16_t freq_mhz, int offset) __attribute__((weak));

bool c5vrx_phy_has_direct_frequency_hook(void)
{
    return phy_set_freq != NULL;
}

esp_err_t c5vrx_phy_set_frequency_mhz(uint16_t mhz)
{
    if (!c5vrx_frequency_in_c5_rx_window(mhz)) {
        ESP_LOGE(TAG, "Refusing %u MHz: outside C5 specified RX window %u-%u MHz",
                 mhz, C5VRX_C5_RX_MIN_MHZ, C5VRX_C5_RX_MAX_MHZ);
        return ESP_ERR_INVALID_ARG;
    }
    if (!phy_set_freq) {
        ESP_LOGW(TAG, "Undocumented phy_set_freq hook is unavailable in the linked PHY blob");
        return ESP_ERR_NOT_SUPPORTED;
    }

    /* Integer-MHz FPV channels do not need the undocumented fine-offset unit. */
    ESP_LOGW(TAG, "EXPERIMENTAL undocumented PHY retune -> %u MHz, offset=0", mhz);
    phy_set_freq(mhz, 0);
    return ESP_OK;
}
