/* SPDX-License-Identifier: GPL-3.0-only */

#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "esp_err.h"

#define C5VRX_RF_DUMP_LIB_SHA256 "0f9680b41612762d854accf2334412e3e01b206a3ec290bbbb232842fdaec7ba"
#define C5VRX_RF_DUMP_IDF_VERSION "v6.0.2"
#define C5VRX_RF_DUMP_PRE_GUARD_ADDR  0x4082ffc0u
#define C5VRX_RF_DUMP_PRE_GUARD_BYTES 64u
#define C5VRX_RF_DUMP_POST_GUARD_ADDR 0x40840000u
#define C5VRX_RF_DUMP_POST_GUARD_BYTES 64u

typedef enum {
    C5VRX_RF_DUMP_MODE_ORDINARY_RX = 0,
    C5VRX_RF_DUMP_MODE_11 = 11,
    /* Present in the vendor API, but deliberately unsupported by the split
     * producer: its setup also calls ble_rx_start(0, 0). */
    C5VRX_RF_DUMP_MODE_12 = 12,
} c5vrx_rf_dump_mode_t;

typedef struct {
    uint32_t control;
    uint32_t pointer_mode;
    uint32_t format;
    uint32_t source_mux;
    uint32_t fe_path;
    uint32_t fe_aux;
    uint16_t pointer;
    bool enabled;
    bool wrap_or_done;
} c5vrx_rf_dump_status_t;

typedef struct {
    uint32_t dump_ctrl;
    uint32_t dump_ptr_mode;
    uint32_t dump_format;
    uint32_t fe_path;
    uint32_t fe_aux;
    uint32_t fe_enable;
    uint32_t source_ctrl;
    uint32_t source_mux;
    uint32_t sram_usage;
    uint32_t modem_clock;
} c5vrx_rf_dump_registers_t;

bool c5vrx_rf_dump_producer_available(void);
bool c5vrx_rf_dump_memory_reserved(void);
bool c5vrx_rf_dump_canaries_intact(void);
bool c5vrx_rf_dump_last_canaries_ok(void);
esp_err_t c5vrx_rf_dump_configure(size_t sample_count,
                                  c5vrx_rf_dump_mode_t mode);
esp_err_t c5vrx_rf_dump_start(void);
esp_err_t c5vrx_rf_dump_get_status(c5vrx_rf_dump_status_t *status);
esp_err_t c5vrx_rf_dump_stop(void);
esp_err_t c5vrx_rf_dump_read_registers(c5vrx_rf_dump_registers_t *registers);
bool c5vrx_rf_dump_last_restore_ok(void);
