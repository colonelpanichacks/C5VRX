/* SPDX-License-Identifier: GPL-3.0-only */

#pragma once

#include <stdbool.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    C5VRX_CVBS_DISPLAY_LOGO = 0,
    C5VRX_CVBS_DISPLAY_SNOW,
    C5VRX_CVBS_DISPLAY_TEST,
} c5vrx_cvbs_display_t;

/** Start the receiver's permanent PAL output in the branded logo state. */
esp_err_t c5vrx_cvbs_output_start(void);

/** Request a tear-free display-state change at the next PAL frame boundary. */
esp_err_t c5vrx_cvbs_output_set_display(c5vrx_cvbs_display_t display);

/** Current on-wire state and its stable protocol/log name. */
c5vrx_cvbs_display_t c5vrx_cvbs_output_display(void);
const char *c5vrx_cvbs_display_name(c5vrx_cvbs_display_t display);

/**
 * Start the analog-output proof generator.
 *
 * The generator streams a PAL 625/50 interlaced monochrome test raster at
 * 20 MS/s through PARLIO. It includes full horizontal/vertical sync structure,
 * branded splash/diagnostics and, when enabled in Kconfig, a PAL-frequency swinging burst
 * used only to stress analog bandwidth/locking.
 *
 * This is intentionally independent from the RF receive path so the
 * C5 -> PARLIO -> passive DAC -> 75-ohm CVBS half can be validated first.
 */
esp_err_t c5vrx_cvbs_test_start(void);

/** Leave diagnostics and return the permanent output to the logo state. */
esp_err_t c5vrx_cvbs_test_stop(void);

/** True while the PAL test stream is active. */
bool c5vrx_cvbs_test_running(void);

#ifdef __cplusplus
}
#endif
