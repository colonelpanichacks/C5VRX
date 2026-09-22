/* Host waveform test: compile with main/menu_raster.c and -lm. */
#include "menu_raster.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static menu_raster_t raster;
static uint8_t waveform[8008000];
static unsigned used, nodes;

static bool capture(void *ctx, const uint8_t *data, unsigned length)
{
    (void)ctx;
    assert(((uintptr_t)data & 3u) == 0);
    assert(length && length <= 4092 && !(length & 3u));
    assert(used + length <= sizeof(waveform));
    assert(++nodes <= MENU_MAX_NODES);
    memcpy(waveform + used, data, length);
    used += length;
    return true;
}

static bool reject(void *ctx, const uint8_t *data, unsigned length)
{
    (void)ctx; (void)data; (void)length;
    return false;
}

static void test(video_standard_t standard)
{
    bool pal = standard == VIDEO_STD_PAL;
    unsigned field = pal ? 625 : 525;
    unsigned fields = 8;
    unsigned eq = pal ? 5 : 6;
    used = nodes = 0;
    menu_raster_init(&raster, standard);
    memset(raster.text, 60, sizeof(raster.text));
    assert(menu_raster_emit(&raster, standard, capture, NULL));
    assert(!menu_raster_emit(&raster, standard, reject, NULL));
    assert(used == (pal ? 6400000u : 5338668u));
    for (unsigned h = 0; h < field * fields; ++h) {
        unsigned at = menu_half_sample(standard, h);
        unsigned next = menu_half_sample(standard, h + 1);
        unsigned pos = (h + (pal ? 5 : 0)) % field;
        double ideal = h * (pal ? 1280.0 : 11440.0 / 9.0);
        assert(fabs(at - ideal) <= 1.778);
        /* Both fields must have equalizing/broad/equalizing pulses, with
         * the second train starting halfway between horizontal sync edges. */
        unsigned width = pos < 3 * eq ?
            (pos >= eq && pos < 2 * eq ? next - at - 188 : (pal ? 94 : 92)) :
            (h % 2 == 0 ? 188 : 0);
        for (unsigned x = 0; x < next - at; ++x) {
            assert((waveform[at + x] == 0) == (x < width));
            assert(waveform[at + x] <= 60);
        }
        if (pos >= 3 * eq && !(h & 1)) {
            unsigned burst = pal ? 224 : 212;
            unsigned count = pal ? 90 : 100;
            unsigned line = h / 2 % 625 + 1;
            bool burst_enabled = !pal || ((h / 1250 % 2) ?
                (line > 5 && line < 622 && (line < 310 || line > 318)) :
                (line > 6 && line < 623 && (line < 311 || line > 319)));
            bool varies = false;
            for (unsigned x = 188; x < MENU_PREFIX_BYTES; ++x) {
                unsigned value = waveform[at + x];
                if (burst_enabled && x >= burst && x < burst + count) {
                    assert(value >= 12 && value <= 28);
                    varies |= value != 20;
                    double carrier = pal ? 709379.0 / 6400000 : 477750.0 / 5338668;
                    double swing = pal ? (h / 2 % 2 ? -0.375 : 0.375) : 0.5;
                    double ideal = 20 + 8 * sin(6.283185307179586 * ((at + x) * carrier + swing));
                    /* Quantized start phase plus integer DAC amplitude. */
                    assert(fabs(value - ideal) < 1.3);
                } else assert(value == 20);
            }
            assert(varies == burst_enabled);
        }
    }
    /* Burst samples must correlate with the selected carrier, not a luma
     * rectangle or an IQ pattern. Check all starting phases. */
    double omega = 6.283185307179586 * (pal ? 4433618.75 / 40000000 : 477750.0 / 5338668);
    for (unsigned p = 0; p < MENU_PHASES; ++p) {
        unsigned begin = pal ? 224 : 212;
        unsigned count = pal ? 90 : 100;
        for (unsigned x = begin; x < begin + count; ++x) {
            double ideal = 20 + 8 * sin(6.283185307179586 * p / MENU_PHASES + omega * x);
            assert(fabs(raster.prefix[p][x] - ideal) <= 0.501);
        }
    }
    double cycles = used * (pal ? 709379.0 / 6400000 : 477750.0 / 5338668);
    assert(fabs(cycles - round(cycles)) < 1e-8);
    if (!pal) {
        assert(fabs(477750.0 * 40000000 / used - 315000000.0 / 88) < 1.0);
        assert(fabs(used / (40000000.0 * 1001 / 30000 * 4) - 1) < 0.0000003);
    }
    printf("%s: %u samples, %u fields, %u DMA nodes; sync, burst, alignment and colour-loop closure OK\n",
           pal ? "PAL" : "NTSC", used, fields, nodes);
}

int main(void)
{
    test(VIDEO_STD_NTSC);
    test(VIDEO_STD_PAL);
    return 0;
}
