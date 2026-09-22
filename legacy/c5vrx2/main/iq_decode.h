#pragma once

#include <stdint.h>

typedef struct {
    int16_t i;
    int16_t q;
    uint32_t raw;
} c5vrx2_iq10_sample_t;

c5vrx2_iq10_sample_t c5vrx2_decode_iq10(uint32_t raw);
