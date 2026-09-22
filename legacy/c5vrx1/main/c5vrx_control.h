/* SPDX-License-Identifier: GPL-3.0-only */

#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"
#include "c5vrx_channels.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Start the simple line-oriented C5VRX host-control protocol on stdin/stdout.
 *
 * Intended for USB Serial/JTAG console use. The protocol lets the desktop
 * flasher/control panel retune channels and trigger finite IQ captures without
 * reflashing the ESP32-C5 for every setting change.
 */
esp_err_t c5vrx_control_start(c5vrx_band_t band,
                              uint8_t channel,
                              bool ht40,
                              bool direct_tune_enabled);

#ifdef __cplusplus
}
#endif
