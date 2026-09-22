#pragma once

#include <stdbool.h>
#include "esp_err.h"

#define C5VRX2_RF_DUMP_BASE 0x40830000u
#define C5VRX2_RF_WORDS     16384u

/* Recover from a CPU/watchdog reset that did not reset the autonomous modem
 * dump domain. Must run before Wi-Fi/PHY initialization or any flash-heavy
 * startup work. Normal streaming never calls this function. */
void c5vrx2_rf_dump_boot_sanitize(void);
bool c5vrx2_rf_dump_memory_reserved(void);
void c5vrx2_rf_dump_guards_init(void);
bool c5vrx2_rf_dump_guards_valid(void);
esp_err_t c5vrx2_rf_dump_prepare_mode0(void);
