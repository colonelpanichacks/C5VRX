/* SPDX-License-Identifier: GPL-3.0-only */

#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/** True when the current linked PHY blob exports the undocumented phy_set_freq hook. */
bool c5vrx_phy_has_direct_frequency_hook(void);

/**
 * EXPERIMENTAL: request an arbitrary receive center using undocumented libphy.
 * This does not initialize a radio backend; call it only after RX is already active.
 */
esp_err_t c5vrx_phy_set_frequency_mhz(uint16_t mhz);

#ifdef __cplusplus
}
#endif
