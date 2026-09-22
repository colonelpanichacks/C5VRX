#pragma once
#include <stdbool.h>
#include <stdint.h>

/* Sample counts at 40 MHz. All DMA segments are word aligned. */
#define MENU_ROWS 7u
#define MENU_FONT_HEIGHT 8u
#define MENU_TEXT_LINES (MENU_ROWS * MENU_FONT_HEIGHT)
#define MENU_PREFIX_BYTES 448u
#define MENU_TAIL_BYTES (2560u - MENU_PREFIX_BYTES)
#define MENU_TEXT_BYTES 1024u
#define MENU_PHASES 32u
#define MENU_MAX_NODES 6000u

typedef enum { VIDEO_STD_NTSC, VIDEO_STD_PAL } video_standard_t;
typedef struct {
    uint8_t prefix[MENU_PHASES][MENU_PREFIX_BYTES];
    uint8_t no_burst[MENU_PREFIX_BYTES];
    uint8_t equalizing[1280];
    uint8_t broad[1280];
    uint8_t blank[MENU_TAIL_BYTES];
    uint8_t text[MENU_TEXT_LINES][MENU_TEXT_BYTES];
} menu_raster_t;

typedef bool (*menu_segment_fn)(void *ctx, const uint8_t *data, unsigned length);
void menu_raster_init(menu_raster_t *raster, video_standard_t standard);
bool menu_raster_emit(const menu_raster_t *raster, video_standard_t standard,
                      menu_segment_fn emit, void *ctx);
uint32_t menu_half_sample(video_standard_t standard, unsigned half);
