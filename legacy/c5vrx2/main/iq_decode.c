#include "iq_decode.h"

static int16_t sign10(uint32_t value)
{
    value &= 0x3ffu;
    return (value & 0x200u) ? (int16_t)(value - 0x400u) : (int16_t)value;
}

c5vrx2_iq10_sample_t c5vrx2_decode_iq10(uint32_t raw)
{
    return (c5vrx2_iq10_sample_t) {
        .q = sign10(raw),
        .i = sign10(raw >> 10),
        .raw = raw,
    };
}
