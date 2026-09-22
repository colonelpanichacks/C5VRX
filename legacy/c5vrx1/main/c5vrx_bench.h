/* SPDX-License-Identifier: GPL-3.0-only */

#pragma once

#include <stdint.h>

#include "esp_err.h"

esp_err_t c5vrx_bench_sparse(unsigned factor);
esp_err_t c5vrx_bench_bitscrambler(uint64_t *input_bytes_per_second);
esp_err_t c5vrx_bench_parlio(void);
esp_err_t c5vrx_bench_parlio_clock(uint32_t clock_hz);
esp_err_t c5vrx_bench_usb_preview(void);
esp_err_t c5vrx_bench_pipeline(uint64_t *input_samples_per_second);
