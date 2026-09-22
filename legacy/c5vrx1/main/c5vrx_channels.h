/* SPDX-License-Identifier: GPL-3.0-only */

#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define C5VRX_FPV_CHANNELS_PER_BAND 8
#define C5VRX_C5_RX_MIN_MHZ 5180
#define C5VRX_C5_RX_MAX_MHZ 5885
#define C5VRX_ANALOG_TARGET_MIN_MHZ 5645
#define C5VRX_ANALOG_TARGET_MAX_MHZ 5885

typedef enum {
    C5VRX_BAND_A = 0,
    C5VRX_BAND_B,
    C5VRX_BAND_E,
    C5VRX_BAND_F,
    C5VRX_BAND_R,
    C5VRX_BAND_COUNT,
} c5vrx_band_t;

typedef struct {
    c5vrx_band_t band;
    char letter;
    const char *name;
    uint8_t channel;
    uint16_t mhz;
} c5vrx_fpv_channel_t;

typedef struct {
    uint16_t requested_mhz;
    uint16_t wifi_center_mhz;
    uint16_t wifi_channel;
    int16_t offset_mhz;
    bool inside_c5_rx_window;
    bool exact_wifi_center;
} c5vrx_frequency_plan_t;

const char *c5vrx_band_name(c5vrx_band_t band);
char c5vrx_band_letter(c5vrx_band_t band);
uint16_t c5vrx_band_frequency(c5vrx_band_t band, uint8_t channel);
bool c5vrx_get_fpv_channel(c5vrx_band_t band, uint8_t channel, c5vrx_fpv_channel_t *out);
bool c5vrx_frequency_in_c5_rx_window(uint16_t mhz);
bool c5vrx_plan_frequency(uint16_t mhz, c5vrx_frequency_plan_t *out);
void c5vrx_print_fpv_coverage(void);

#ifdef __cplusplus
}
#endif
