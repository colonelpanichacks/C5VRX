#pragma once
#include <stdbool.h>
#include <stdint.h>

/*
 * Phase5 quality helpers shared by the live observer and the supervisory
 * range controller.  These functions NEVER pace the live path and do not
 * change RF/PHY state.
 *
 * The current hardware demodulator uses a 50 ns endpoint delta.  For each
 * three-sample 40 MS/s window (a,b,c), the exact adjacent interpretation
 * is delta(a,b)+delta(b,c).  If that unwrapped sum differs from delta(a,c),
 * the endpoint discriminator has lost one +/-2pi winding.
 */

#define DEMOD_STRONG_POWER_MIN          64
#define DEMOD_STATIC_HEAVY_WINDING_PM  180

static inline int demod_phase5_signed_delta(uint8_t previous, uint8_t current)
{
    int delta = (int)(current & 31u) - (int)(previous & 31u);
    if (delta > 15) delta -= 32;
    else if (delta < -16) delta += 32;
    return delta;
}

static inline bool demod_phase5_endpoint_loses_winding(uint8_t first,
                                                        uint8_t middle,
                                                        uint8_t last)
{
    int adjacent = demod_phase5_signed_delta(first, middle) +
                   demod_phase5_signed_delta(middle, last);
    int endpoint = demod_phase5_signed_delta(first, last);
    return adjacent != endpoint;
}

static inline int demod_winding_penalty(int winding_permille)
{
    if (winding_permille < 0) return 0;
    if (winding_permille > 1000) winding_permille = 1000;
    return winding_permille / 10;
}

static inline bool demod_static_heavy(int winding_permille)
{
    return winding_permille >= DEMOD_STATIC_HEAVY_WINDING_PM;
}
