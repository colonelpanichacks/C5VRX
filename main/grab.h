#pragma once

#include <stdbool.h>
#include <stdint.h>

/**
 * grab.c - composite video frame grabber on the raw I/Q ring (link mode).
 *
 * FM-demodulates the 40 MS/s Q4/I4 ring in software (phase step between
 * samples from a 256-entry angle table), locks onto the horizontal and
 * vertical sync of the received PAL or NTSC signal with a flywheel on both,
 * and picks GRAB_H interlaced lines of GRAB_W luma pixels out of the active
 * picture: half of the rows from each of two consecutive fields.
 *
 * Everything here is plain C on a byte ring, so test/host/grab_sim.c runs the
 * same code against synthetic video and against raw captures from hardware.
 */

#define GRAB_W 224
#define GRAB_H 168

/* Host simulator only (test/host/grab_sim.c, built with -DGRAB_SIM_COST):
 * the hot paths report the RISC-V instructions they would execute on the
 * C5 (counted from the -O2 disassembly), so that the simulator's clock
 * follows the receiver's CPU instead of the host's. Nothing on the target. */
#ifdef GRAB_SIM_COST
void grab_sim_cost(uint32_t instructions);
#define SIM_COST(n) grab_sim_cost((uint32_t)(n))
#else
#define SIM_COST(n) ((void)0)
#endif

typedef struct {
    int      rows;          /* rows captured (GRAB_H when complete) */
    int      fields;        /* fields visited */
    bool     pal;           /* standard detected from the line period */
    float    line_us;       /* tracked line period */
    int      acquire_ms;    /* time spent finding the vertical interval (0 when already locked) */
    int      grab_ms;       /* total time of the call */
    int      late;          /* rows whose samples were overwritten before use */
    int      substituted;   /* rows taken from the next line of the field because theirs had gone */
    int      given_up;      /* rows left out after one failed retry each (the display keeps the old row) */
    int      nosync;        /* rows without a horizontal sync where predicted */
    float    sync_level;    /* tracked levels, phase units per sample (256 = one turn) */
    float    blank_level;
    float    jitter_ns;     /* RMS difference between predicted and measured sync edges */
    int      windows;       /* search windows demodulated while acquiring */
    int      window_us;     /* total time spent demodulating them */
    int      row_us;        /* total time spent in captured rows (wait included) */
    int      cpu_us;        /* ... of which processing, from the line being in the ring */
    int      cb_us;         /* total time spent in the row callback */
    int      skipped;       /* rows of the first field already gone when the grab started (taken from a later field) */
    int      idle_calls;    /* idle callbacks that did work while waiting for lines */
    int      idle_us;       /* ... and the time they took */
    float    carrier_mhz;   /* centre of the video swing relative to the tuned frequency */
    float    power;         /* mean I^2+Q^2 of the samples read (4-bit units, 0..128) */
    float    clip;          /* fraction of those samples with I or Q at full scale */
    int      big_err;       /* rows whose sync edge was more than 0.5 us off the prediction */
    int      max_err;       /* largest prediction error, samples */
    int      wide_hits;     /* rows found only by the fallback search over a whole line */
    int      fifo_ovf;      /* PARLIO RX FIFO overflowed during the grab (samples lost) */
    bool     tb_bad;        /* the timebase disagreed with the GDMA: descriptor-granular ring checks */
    int      tb_lead_min_ns, tb_lead_max_ns;   /* timebase estimate minus the GDMA, at descriptor switches */
    int      missing_row;   /* first row not captured, -1 when complete */
    int      vchecks;       /* fields whose vertical interval was checked */
    int      vslips;        /* ... and found off the prediction (grid corrected) */
    int      vmisses;       /* ... and not found at all */
    const char *error;      /* NULL on success */
} grab_info_t;

/* Per-row diagnostics of the most recent rows (ring of GRAB_TRACE_LEN).
 * Shortened from upstream's 48: the trace consumer is host-side debugging
 * only and HP SRAM is the binding constraint of this integration. */
#define GRAB_TRACE_LEN 8
typedef struct {
    int32_t  line;          /* grid line index */
    int16_t  err;           /* measured - predicted sync edge, samples (clamped) */
    uint8_t  row;           /* output row */
    char     result;        /* 'o' ok, 'w' ok via wide search, 'n' no sync, 'l' late, 'r' resync */
} grab_trace_t;

/** Copy the last rows' diagnostics, oldest first; returns the count. */
int grab_trace(grab_trace_t *out, int max);

/** Build the tables. Call once, after the RX ring runs. */
void grab_init(void);

/** Forget the lock (after a channel change, for example). */
void grab_unlock(void);

/** Horizontal [1 2 1] low-pass over the pixels of each row. FM noise rises
 * with frequency, so this removes most of it (2.7x less noise in the
 * simulator) for a slightly softer picture. On by default. */
void grab_set_smoothing(bool on);
bool grab_smoothing(void);

/** Called for every row as soon as it is captured, from inside the grab
 * loop: must be quick (the next line keeps coming) and must not block. */
typedef void (*grab_row_cb_t)(void *ctx, int y, const uint8_t *row);

/** Called while the grab waits for a line that is still more than about
 * 130 us away: one bounded piece of background work (well under 100 us),
 * returning false when there is nothing to do. Must not block. */
typedef bool (*grab_idle_cb_t)(void *ctx);

/**
 * Grab one frame, one row at a time. Starts with whatever field is on air, so
 * a complete frame takes two fields. Busy-waits on the ring; call from a task
 * that may block the CPU for up to timeout_ms.
 *
 * img is a row staging buffer of at least GRAB_W bytes: every captured row is
 * written there and immediately handed to on_row (which must copy it out
 * before returning; the same staging buffer is reused for the next row).
 * Rows are NOT accumulated into a full-frame buffer, so on_row is required
 * for the image to survive; rows the grab cannot capture are simply never
 * delivered, leaving the caller's previous content in place. Returns the
 * number of rows captured; info reports the details. on_row and on_idle may
 * be NULL; both get ctx. on_idle is one bounded piece of background work
 * (well under 100 us) run while the grab waits for a line that is still more
 * than about 130 us away; return false when there is nothing to do.
 */
int grab_frame(uint8_t *img, grab_info_t *info, int timeout_ms, grab_row_cb_t on_row,
               grab_idle_cb_t on_idle, void *ctx);

/**
 * Is there analog video on the current channel? Looks for horizontal sync
 * (at least four consecutive line periods of 64.0 us or 63.6 us) for up to
 * timeout_ms. Does not lock; used by the channel scan.
 */
bool grab_probe(int timeout_ms, grab_info_t *info);
