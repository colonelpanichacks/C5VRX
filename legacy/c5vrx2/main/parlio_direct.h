#pragma once

#include <stdint.h>

#include "esp_err.h"

esp_err_t c5vrx2_parlio_direct_prepare(uint32_t output_rate_hz);
esp_err_t c5vrx2_parlio_direct_start(void);
void c5vrx2_parlio_direct_quiesce(void);
void c5vrx2_parlio_direct_destroy(void);
uint32_t c5vrx2_parlio_direct_rate_hz(void);
