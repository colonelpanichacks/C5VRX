#pragma once

#include <stdbool.h>

#include "esp_err.h"

esp_err_t c5vrx2_rf_oracle_diagnostic_start(void);
esp_err_t c5vrx2_rf_wrap_diagnostic_run(void);
esp_err_t c5vrx2_rf_dma_diagnostic_run(void);
esp_err_t c5vrx2_modem_diag_diagnostic_run(void);
esp_err_t c5vrx2_modem_capture_diagnostic_run(void);
esp_err_t c5vrx2_modem_parlio_diagnostic_run(void);
esp_err_t c5vrx2_tx_wbfm_diagnostic_run(void);
esp_err_t c5vrx2_av_static_diagnostic_start(unsigned code);
esp_err_t c5vrx2_av_pal_diagnostic_start(bool colour);
