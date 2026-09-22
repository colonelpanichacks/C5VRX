#pragma once

#include "esp_err.h"

/* Starts MODEM_DIAG -> PARLIO RX -> adjacent FM -> continuous DAC. */
esp_err_t c5vrx2_realtime_start(void);
