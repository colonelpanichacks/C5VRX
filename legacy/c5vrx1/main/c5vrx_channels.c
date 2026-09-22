/* SPDX-License-Identifier: GPL-3.0-only */

#include "c5vrx_channels.h"

#include <limits.h>
#include "esp_log.h"

static const char *TAG = "c5vrx_channels";

static const struct {
    char letter;
    const char *name;
    uint16_t mhz[C5VRX_FPV_CHANNELS_PER_BAND];
} s_bands[C5VRX_BAND_COUNT] = {
    [C5VRX_BAND_A] = {'A', "Boscam A", {5865, 5845, 5825, 5805, 5785, 5765, 5745, 5725}},
    [C5VRX_BAND_B] = {'B', "Boscam B", {5733, 5752, 5771, 5790, 5809, 5828, 5847, 5866}},
    [C5VRX_BAND_E] = {'E', "Boscam E", {5705, 5685, 5665, 5645, 5885, 5905, 5925, 5945}},
    [C5VRX_BAND_F] = {'F', "FatShark",  {5740, 5760, 5780, 5800, 5820, 5840, 5860, 5880}},
    [C5VRX_BAND_R] = {'R', "RaceBand",  {5658, 5695, 5732, 5769, 5806, 5843, 5880, 5917}},
};

static const struct {
    uint16_t channel;
    uint16_t mhz;
} s_wifi_centers[] = {
    {132, 5660}, {136, 5680}, {140, 5700}, {144, 5720},
    {149, 5745}, {153, 5765}, {157, 5785}, {161, 5805},
    {165, 5825}, {169, 5845}, {173, 5865}, {177, 5885},
};

const char *c5vrx_band_name(c5vrx_band_t band)
{
    return band < C5VRX_BAND_COUNT ? s_bands[band].name : "Unknown";
}

char c5vrx_band_letter(c5vrx_band_t band)
{
    return band < C5VRX_BAND_COUNT ? s_bands[band].letter : '?';
}

uint16_t c5vrx_band_frequency(c5vrx_band_t band, uint8_t channel)
{
    if (band >= C5VRX_BAND_COUNT || channel < 1 || channel > C5VRX_FPV_CHANNELS_PER_BAND) {
        return 0;
    }
    return s_bands[band].mhz[channel - 1];
}

bool c5vrx_get_fpv_channel(c5vrx_band_t band, uint8_t channel, c5vrx_fpv_channel_t *out)
{
    const uint16_t mhz = c5vrx_band_frequency(band, channel);
    if (!mhz || !out) {
        return false;
    }
    *out = (c5vrx_fpv_channel_t) {
        .band = band,
        .letter = c5vrx_band_letter(band),
        .name = c5vrx_band_name(band),
        .channel = channel,
        .mhz = mhz,
    };
    return true;
}

bool c5vrx_frequency_in_c5_rx_window(uint16_t mhz)
{
    return mhz >= C5VRX_C5_RX_MIN_MHZ && mhz <= C5VRX_C5_RX_MAX_MHZ;
}

bool c5vrx_plan_frequency(uint16_t mhz, c5vrx_frequency_plan_t *out)
{
    if (!out) {
        return false;
    }

    unsigned best = 0;
    int best_abs = INT_MAX;
    for (unsigned i = 0; i < sizeof(s_wifi_centers) / sizeof(s_wifi_centers[0]); ++i) {
        const int delta = (int)mhz - (int)s_wifi_centers[i].mhz;
        const int ad = delta < 0 ? -delta : delta;
        if (ad < best_abs) {
            best = i;
            best_abs = ad;
        }
    }

    *out = (c5vrx_frequency_plan_t) {
        .requested_mhz = mhz,
        .wifi_center_mhz = s_wifi_centers[best].mhz,
        .wifi_channel = s_wifi_centers[best].channel,
        .offset_mhz = (int16_t)((int)mhz - (int)s_wifi_centers[best].mhz),
        .inside_c5_rx_window = c5vrx_frequency_in_c5_rx_window(mhz),
        .exact_wifi_center = mhz == s_wifi_centers[best].mhz,
    };
    return true;
}

void c5vrx_print_fpv_coverage(void)
{
    ESP_LOGI(TAG, "Analog FPV target window: %u-%u MHz (C5 specified RX: %u-%u MHz)",
             C5VRX_ANALOG_TARGET_MIN_MHZ, C5VRX_ANALOG_TARGET_MAX_MHZ,
             C5VRX_C5_RX_MIN_MHZ, C5VRX_C5_RX_MAX_MHZ);

    for (unsigned band = 0; band < C5VRX_BAND_COUNT; ++band) {
        for (uint8_t ch = 1; ch <= C5VRX_FPV_CHANNELS_PER_BAND; ++ch) {
            c5vrx_fpv_channel_t fpv;
            c5vrx_frequency_plan_t plan;
            if (!c5vrx_get_fpv_channel((c5vrx_band_t)band, ch, &fpv) ||
                !c5vrx_plan_frequency(fpv.mhz, &plan)) {
                continue;
            }
            ESP_LOGI(TAG, "%c%u %4u MHz : %s, nearest Wi-Fi ch%u %4u MHz (%+d MHz)%s",
                     fpv.letter, fpv.channel, fpv.mhz,
                     plan.inside_c5_rx_window ? "IN " : "OUT",
                     plan.wifi_channel, plan.wifi_center_mhz, plan.offset_mhz,
                     plan.exact_wifi_center ? " EXACT" : "");
        }
    }
}
