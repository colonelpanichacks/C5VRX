#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

#define C5VRX2_CALIBRATION_FLASH_OFFSET 0x00110000u
#define C5VRX2_CALIBRATION_PARTITION_SIZE 0x00001000u

typedef enum {
    C5VRX2_POLARITY_CURRENT_MINUS_PREVIOUS = 0,
    C5VRX2_POLARITY_PREVIOUS_MINUS_CURRENT = 1,
} c5vrx2_polarity_t;

typedef struct {
    uint8_t pedestal_code;
    uint8_t discriminator_gain;
    c5vrx2_polarity_t polarity;
    uint32_t output_clock_hz;
    bool loaded_from_flash;
} c5vrx2_calibration_t;

void c5vrx2_calibration_load(void);
const c5vrx2_calibration_t *c5vrx2_calibration_get(void);
