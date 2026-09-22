#pragma once

#include <stdbool.h>

#include "esp_err.h"

esp_err_t c5vrx2_wifi5_start_a1(void);
esp_err_t c5vrx2_wifi5_lock_rx_only(void);
bool c5vrx2_wifi5_tx_is_quiescent(void);
