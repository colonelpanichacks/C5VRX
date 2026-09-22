/**
 * grab.c - composite video frame grabber on the raw I/Q ring (link mode).
 *
 * Demodulation. The 16 KiB RX ring holds 409.6 us of 40 MS/s Q4/I4 samples.
 * The FM discriminator is the phase step between samples: a 256-entry table
 * gives the angle of every 4-bit I/Q point (256 units per turn) and the
 * wrapped difference of two angles is the step. That is amplitude-independent
 * and costs one table load per sample. Sums of steps telescope, so a sum
 * over n samples only needs the angle every `step` samples, as long as one
 * step stays below half a turn; every step is unwrapped around the centre of
 * the video swing (s_center), which keeps that true for carrier offsets of
 * several MHz (hardware showed -5.6 .. +3 MHz between tunings). Sync tip is
 * the lowest phase step, white the highest.
 *
 * Timing. Every ring byte has an absolute sample index S: the ring offset
 * plus 16384 x wraps, the wrap count taken from esp_timer, which runs from
 * the same crystal as the sample clock. The same timebase, anchored at a
 * descriptor switch at the start of every grab, places the GDMA's write
 * position to about a microsecond (the descriptor register alone only to
 * 102 us), so a line can be used as soon as it is in the ring and until it
 * is about to be overwritten; every descriptor switch seen checks the
 * estimate, and a grab where it disagrees falls back to descriptor bounds.
 *
 * Lock. Search windows (153.6 us, copied out of the ring first, because the
 * GDMA overwrites a window while its pulses are still being examined) give
 * the line period (PAL / NTSC), the sync and blanking levels and finally the
 * vertical interval: an equalizing pulse followed half a line later by the
 * first broad pulse. That broad pulse fixes the field start; the first
 * captured line settles whether the field starts on a line boundary or half
 * a line off it.
 *
 * Flywheels. Single sync-edge measurements are noisy (about +/- 1 us at low
 * SNR, plus FM clicks); the camera's line and field clocks are not. So the
 * line grid S(N) = S_A + (N - N_A) T is only steered a quarter of the way
 * towards each measured edge, the line period is corrected only across
 * hundreds of lines, and at the start of every field the vertical interval
 * is checked where the grid predicts it (and the grid corrected if it sits
 * up to 1.5 lines off). Without these the picture wobbled sideways and, once
 * the line period had wandered, crept up the screen.
 *
 * Capture. Output row y shows picture line p = y * 2A / GRAB_H (A active
 * lines per field): even p from the field that starts on a line boundary,
 * odd p from the other one. Each row waits until its line is in the ring,
 * finds the sync edge near the prediction (and, failing that, anywhere in
 * the line), measures sync and blanking levels on every fourth row (slow
 * average) and turns the phase advance over 9 samples into a pixel,
 * optionally smoothed [1 2 1]. A grab starts with the field on air (the rows
 * of it that are already gone come from the field after next), so a
 * complete frame takes two fields whatever the moment the grab starts. A row
 * whose line has already left the ring (the CPU fell behind in a run of
 * close rows) takes the next line of the same field, half an output row
 * lower, instead of waiting two fields; rows missed otherwise come from a
 * later field of the same parity. Every captured row is handed to a
 * callback at once, and while the grab waits for a line more than about
 * 130 us away it runs an idle callback: the link encodes and sends rows
 * there, so a frame is mostly on the wire when its grab ends.
 *
 * CPU. The C5 has no FPU and runs about 1.7 cycles per instruction on these
 * loops; the per-row path is integer except for a handful of double
 * operations (the level-derived constants are cached), and the hot loops
 * report their instruction counts to the host simulator (SIM_COST).
 */

#include "grab.h"

#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "esp_attr.h"
#include "esp_cache.h"
#include "esp_timer.h"
#include "sdkconfig.h"

#include "soc/parl_io_struct.h"
#include "video.h"

/* C5VRX integration notes (this copy vs upstream buzz/konrad-tembed):
 *  - WIN_SAMPLES/WIN_COPY shrunk 6144/8192 -> 6080/6144 to fit the freed
 *    SRAM budget (6.9 KB, not 8.2). 6144 is NOT a power of two, so the
 *    win is indexed with an explicit conditional wrap (SRC_NEXT) instead
 *    of upstream's & (WIN_COPY-1); reads never reach WIN_SAMPLES+16+64
 *    into the copy, which 6144 covers exactly (6080 + 64 headroom + 16
 *    refine tail). Ring accesses keep the pow2 RING_MASK.
 *  - Rows stream through a GRAB_W-byte staging buffer with an on_row
 *    callback (see grab.h): the caller has no 37 KB full-frame buffer.
 *  - Every direct ring read is cache-synced M2C first (the GDMA writes
 *    through AHB): ring_sync_m2c() at the four read sites below.
 *  - pulses/sorted/s_blk/done/failed moved from static to stack storage;
 *    extract_pixels' raw[] is int16 on the stack (|raw| <= ~830). */

#define RING        VIDEO_RX_RING_BYTES
#define RING_MASK   (RING - 1u)
#define SPS         40                          /* samples per microsecond */

/* Search windows. */
#define BLK         16                          /* samples per block (0.4 us) */
#define WIN_SAMPLES 6080                        /* 152 us: two line periods plus margin */
#define WIN_BLOCKS  (WIN_SAMPLES / BLK)
#define WIN_COPY      6144u                     /* private copy, indexed like the ring */
#define MAX_PULSES  48

/* Line layout relative to the sync edge E (samples). */
#define SYNC_FROM   40                          /* 1.0 .. 4.0 us: sync tip */
#define SYNC_LEN    120
#define BLANK_FROM  320                         /* 8.0 us: after the colour burst */
#define BLANK_LEN_PAL  81                       /* .. 10.0 us (back porch ends at 10.4) */
#define BLANK_LEN_NTSC 48                       /* .. 9.2 us (back porch ends at 9.4) */
#define PIX_SAMPLES 9                           /* 224 x 9 = 2016 samples = 50.4 us */
#define PIX_STEP    3                           /* phase read every 3 samples */
#define LVL_STEP    3
#define BLK_STEP    2
#define ACTIVE_PAL  452                         /* 11.3 us: 10.4 us + 0.8 us trim */
#define ACTIVE_NTSC 420                         /* 10.5 us: 9.4 us + 1.1 us trim */

/* Edge detector and flywheels. */
#define EDGE_W        16                        /* running-sum length (0.4 us) */
#define EDGE_LEAD     24                        /* samples above the threshold before an edge counts */
#define NARROW_SEARCH 40                        /* +/- 1 us around the predicted edge (BIG_ERROR) */
#define STEER         0.25                      /* share of an edge error applied to the grid */
#define BIG_ERROR     40.0                      /* 1 us: re-anchor instead of steering */
#define T_SPAN        400                       /* lines between period measurements */
#define GIVE_UP_ROWS  4                         /* rows a frame may leave out once each has failed */
#define MAX_MISSES    12                        /* consecutive rows without sync before unlock */
#define MAX_VMISSES   3                         /* consecutive fields without vertical sync */
#define RESYNC_US     1500000                   /* gap after which a grab re-acquires */
#define REANCHOR_US   100000                    /* gap after which the first row searches the whole line */
#define CATCH_LINES   3.5                       /* how far back a row may lie and still be read from the ring */
#define IDLE_MIN_US   130                       /* idle work only while the next line is at least this far off */

static const int8_t k_sign4[16] = { 0, 1, 2, 3, 4, 5, 6, 7, -8, -7, -6, -5, -4, -3, -2, -1 };

static uint8_t s_ang[256];                      /* angle of a sample byte, 256 per turn */
static uint8_t s_pow[256];                      /* I^2 + Q^2 */
static uint8_t s_clp[256];                      /* 1 if I or Q is at full scale */
static uint32_t s_pw_sum, s_pw_n, s_clip_n;     /* power statistics of the current grab */

static const uint8_t *s_ring;
static uint32_t s_max_dscr = 4092u;

/* Sample source of the demod functions: the live ring or the window copy.
 * The ring is pow2 and keeps the mask; the win (WIN_COPY, not pow2) uses
 * an explicit conditional wrap. src_index()/SRC_NEXT implement both. */
static const uint8_t *s_src;
static uint32_t s_src_mask = RING_MASK;         /* kept for the ring fast-path bound */
static bool    s_src_is_win;
static int64_t s_src_base;                      /* absolute S at win offset 0 */
static uint8_t s_win[WIN_COPY];

/* Absolute sample index -> source offset. Ring: pow2 mask. Win: anchored
 * subtraction (all win reads lie within [base, base + 2*WIN_COPY), so one
 * conditional wrap suffices -- no division in the per-call paths). */
static inline uint32_t src_index(int64_t S)
{
    if (s_src_is_win) {
        uint32_t off = (uint32_t)(S - s_src_base);
        if (off >= WIN_COPY) off -= WIN_COPY;
        return off;
    }
    return (uint32_t)S & RING_MASK;
}

/* Advance a source offset by step (< WIN_COPY), wrapping per source. */
#define SRC_NEXT(i, step)                     \
    do {                                      \
        (i) += (uint32_t)(step);              \
        if (s_src_is_win) {                   \
            if ((i) >= WIN_COPY) (i) -= WIN_COPY; \
        } else {                              \
            (i) &= RING_MASK;                 \
        }                                     \
    } while (0)

/* Step back one source offset (find_edge's history seed). */
#define SRC_PREV(i)                           \
    do {                                      \
        if (s_src_is_win) {                   \
            if ((i) == 0u) (i) = WIN_COPY - 1u; \
            else --(i);                       \
        } else {                              \
            (i) = ((i) - 1u) & RING_MASK;     \
        }                                     \
    } while (0)

/* Contiguous-run bound for the unmasked fast paths (extract_pixels). */
static inline uint32_t src_run_limit(void)
{
    return s_src_is_win ? (uint32_t)WIN_COPY : (uint32_t)RING;
}

/* Timebase: absolute sample index of the ring. */
static int64_t s_tb_t0;
static int64_t s_tb_s0;

/* Write position of the GDMA (see ring_now()). */
#define WPOS_GUARD 400                          /* 10 us */
static int64_t s_wpos;                          /* estimated sample being written, from the last ring_now() */
static int64_t s_wpos_lo, s_wpos_hi;            /* what the ring checks assume: written up to lo, overwriting from hi */
static bool    s_tb_bad;                        /* estimate disagreed with a descriptor switch: descriptor bounds only */
static int64_t s_last_sw = -1, s_last_poll_us;
static int     s_tb_lead_min, s_tb_lead_max;    /* estimate - switch, samples, at promptly seen switches */

/* Sync model. */
static bool    s_locked;
static bool    s_parity_pending;   /* first row after lock: wide search, settles F0 */
static bool    s_resync_pending;   /* first row of a later grab: wide search, corrects T */
static int64_t s_last_grab_us;
static bool    s_pal = true;
static double  s_T = 2560.0;       /* line period, samples */
static double  s_SA;               /* grid: sync edge of line s_NA */
static int32_t s_NA;
static double  s_SP;               /* period anchor: grid position of line s_NP, T_SPAN or more lines back */
static int32_t s_NP;
static bool    s_P_ok;
static double  s_F0;               /* grid position of the first broad pulse of field 0 */
static double  s_sync = -20.0;     /* tracked levels, phase units per sample */
static double  s_blank = -5.0;
static bool    s_levels_ok;
static double  s_center;           /* centre of the video swing, phase units per sample */

/* Integer values derived from the levels, for the per-row code (the C5 has
 * no FPU: every double operation is a library call). Refreshed by
 * levels_changed() whenever s_sync, s_blank or s_center change. */
static int     s_bias[4];          /* lround(s_center * step), step 0..3 */
static int     s_thr_edge;         /* find_edge() threshold */
static int32_t s_pix_c0, s_pix_m;  /* pixel scale, see extract_pixels() */
static bool    s_pix_ok;
static double  s_T_min = 2560.0 * 0.9995, s_T_max = 2560.0 * 1.0005;

static void levels_changed(void);
static bool    s_smooth = true;

/* Cache-sync an absolute-sample ring range before the CPU reads it (the RX
 * GDMA wrote it through AHB; the CPU may hold stale lines). Wrap-aware;
 * aligned to 64-byte lines like video.c's sync_dma_m2c(). A no-op where the
 * SoC keeps SRAM coherent. */
static void ring_sync_m2c(int64_t S_from, int n)
{
    if (n <= 0) return;
    uint32_t off = (uint32_t)S_from & RING_MASK;
    if ((uint32_t)n > RING) n = (int)RING;
    for (int part = 0; part < 2 && n > 0; ++part) {
        uint32_t first = (uint32_t)n;
        if (off + first > RING) first = RING - off;
        const uintptr_t a = (uintptr_t)(s_ring + off);
        const uintptr_t start = a & ~(uintptr_t)63u;
        const uintptr_t end = (a + first + 63u) & ~(uintptr_t)63u;
        (void)esp_cache_msync((void *)start, (size_t)(end - start),
                              ESP_CACHE_MSYNC_FLAG_DIR_M2C | ESP_CACHE_MSYNC_FLAG_UNALIGNED);
        n -= (int)first;
        off = 0u;
    }
}

typedef struct {
    int64_t start;   /* first sample below the threshold (block precision) */
    int     len;     /* samples */
} pulse_t;

typedef struct {
    double err_sq;
    int    err_n;
} jitter_t;

/* ------------------------------------------------------------------ trace */

static grab_trace_t s_trace[GRAB_TRACE_LEN];
static unsigned s_trace_pos, s_trace_n;

static void trace(int32_t line, double err, int row, char result)
{
    grab_trace_t *t = &s_trace[s_trace_pos];
    t->line = line;
    t->err = (int16_t)(err > 32767.0 ? 32767 : err < -32768.0 ? -32768 : (int)err);
    t->row = (uint8_t)row;
    t->result = result;
    s_trace_pos = (s_trace_pos + 1u) % GRAB_TRACE_LEN;
    if (s_trace_n < GRAB_TRACE_LEN) s_trace_n++;
}

int grab_trace(grab_trace_t *out, int max)
{
    int n = (int)s_trace_n < max ? (int)s_trace_n : max;
    unsigned start = (s_trace_pos + GRAB_TRACE_LEN - (unsigned)n) % GRAB_TRACE_LEN;
    for (int k = 0; k < n; ++k) out[k] = s_trace[(start + (unsigned)k) % GRAB_TRACE_LEN];
    return n;
}

/* ------------------------------------------------------------------ setup */

void grab_init(void)
{
    for (unsigned b = 0u; b < 256u; ++b) {
        int q = k_sign4[b & 0x0fu];
        int i = k_sign4[b >> 4];
        s_pow[b] = (uint8_t)(i * i + q * q);
        s_clp[b] = (uint8_t)((i == 7 || i == -8 || q == 7 || q == -8) ? 1u : 0u);
        if (i == 0 && q == 0) {
            s_ang[b] = 0u;
        } else {
            float a = atan2f((float)q, (float)i) * (128.0f / (float)M_PI);
            s_ang[b] = (uint8_t)((int)lroundf(a) & 0xff);
        }
    }
    s_ring = video_rx_ring();
    s_src = s_ring;
    s_src_mask = RING_MASK;
    s_src_is_win = false;
    s_max_dscr = video_rx_max_descriptor();
    levels_changed();
}

void grab_unlock(void)
{
    s_locked = false;
    s_P_ok = false;
    s_parity_pending = false;
    s_resync_pending = false;
}

void grab_set_smoothing(bool on)
{
    s_smooth = on;
}

bool grab_smoothing(void)
{
    return s_smooth;
}

/* ------------------------------------------------------------------ timebase */

static bool ring_now(int64_t *s_w);

/* (Re-)anchor the timebase at a descriptor switch. The first call defines the
 * sample numbering; later calls keep it (the lock model holds absolute sample
 * indices) and only remove any accumulated timer/sample-clock drift. */
static bool timebase_init(void)
{
    static bool s_tb_ok;
    /* Anchor at a descriptor switch seen between two reads at most 3 us
     * apart (an interrupt between them would blur it); a few tries, then
     * accept a blurred one and keep the ring checks descriptor-granular. */
    for (int attempt = 0; attempt < 4; ++attempt) {
        uint32_t a, b;
        if (!video_rx_write_offset(&a)) return false;
        int64_t t_old = esp_timer_get_time();       /* the old descriptor was still current here */
        const int64_t deadline = t_old + 2000;
        int64_t t_new;
        for (;;) {
            if (!video_rx_write_offset(&b)) return false;
            t_new = esp_timer_get_time();
            if (b != a) break;
            if (t_new > deadline) return false;     /* ring stalled */
            t_old = t_new;
        }
        const bool sharp = t_new - t_old <= 3;
        if (!sharp && attempt < 3) continue;
        int64_t s_w = (int64_t)b;
        if (s_tb_ok && !ring_now(&s_w)) return false;   /* unwrap with the old anchor */
        s_tb_t0 = (t_old + t_new) / 2;                   /* the switch fell between the two reads */
        s_tb_s0 = s_w;
        s_tb_ok = true;
        s_tb_bad = !sharp;
        s_last_sw = s_w;
        s_last_poll_us = t_new;
        s_tb_lead_min = 1 << 30;
        s_tb_lead_max = -(1 << 30);
        return ring_now(&s_w);
    }
    return false;
}

/* Where the GDMA writes. The descriptor it is in (s_w) is exact but coarse
 * (102 us); the timebase gives the sample being written to about a
 * microsecond (the anchor is a descriptor switch; esp_timer and the sample
 * clock run from the same crystal). s_wpos is that estimate, limited to the
 * current descriptor; the ring checks use it with a guard of WPOS_GUARD
 * samples either way. Every descriptor switch seen promptly (within 3 us)
 * checks the estimate: it must be within the guard of the switch, less 1 us,
 * allowing for when between the two polls the switch happened; when it is
 * not, the rest of the grab falls back to the descriptor bounds. */

/* Absolute sample index of the first byte of the descriptor being written. */
static bool ring_now(int64_t *s_w)
{
    uint32_t off;
    SIM_COST(90);
    if (!video_rx_write_offset(&off)) return false;
    int64_t now = esp_timer_get_time();
    int64_t est = s_tb_s0 + (now - s_tb_t0) * SPS;
    int64_t k = est - (int64_t)off + (int64_t)(RING / 2u);
    k = (k >= 0) ? k / RING : -((-k + RING - 1) / RING);
    *s_w = (int64_t)off + k * (int64_t)RING;
    if (*s_w != s_last_sw) {
        const int gap = (int)(now - s_last_poll_us);
        if (s_last_sw >= 0 && gap <= 3) {
            /* The GDMA is now between s_w and s_w + gap us of samples. */
            int lead = (int)(est - *s_w);
            if (lead < s_tb_lead_min) s_tb_lead_min = lead;
            if (lead > s_tb_lead_max) s_tb_lead_max = lead;
            if (lead >= WPOS_GUARD - SPS || lead - gap * SPS <= -(WPOS_GUARD - SPS)) {
#ifdef GRAB_DEBUG
                printf("  tb check: lead %d samples gap %d us s_w %lld last_sw %lld\n", lead, gap,
                       (long long)*s_w, (long long)s_last_sw);
#endif
                s_tb_bad = true;
            }
        }
        s_last_sw = *s_w;
    }
    s_last_poll_us = now;
    const int64_t d_end = *s_w + (int64_t)s_max_dscr;
    s_wpos = est < *s_w ? *s_w : est > d_end ? d_end : est;
    if (s_tb_bad) {
        s_wpos_lo = *s_w;                       /* complete up to the descriptor being written */
        s_wpos_hi = d_end + 64;                 /* which may be written to its end */
    } else {
        s_wpos_lo = s_wpos - WPOS_GUARD < *s_w ? *s_w : s_wpos - WPOS_GUARD;
        s_wpos_hi = s_wpos + WPOS_GUARD > d_end + 64 ? d_end + 64 : s_wpos + WPOS_GUARD;
    }
    return true;
}

/* Oldest sample still in the ring at the last ring_now(). (s_w only names
 * the call; the bounds come from it.) */
static inline int64_t oldest_valid(int64_t s_w)
{
    (void)s_w;
    return s_wpos_hi - (int64_t)RING;
}

/* Reading oldest to newest at less than the sample rate, the newest sample
 * is the one closest to being overwritten: is it still there? */
static inline bool newest_intact(int64_t S_newest, int64_t s_w_now)
{
    (void)s_w_now;
    return S_newest + (int64_t)RING > s_wpos_hi;
}

/* Background work of the current grab (see grab_frame()). */
static grab_idle_cb_t s_idle;
static void *s_idle_ctx;
static grab_info_t *s_idle_info;

/* Busy-wait until sample S_need is in the ring (or the deadline passes).
 * While it is more than IDLE_MIN_US away, the wait runs the idle callback
 * instead of spinning. */
static bool wait_for(int64_t S_need, int64_t t_end, int64_t *s_w)
{
    for (;;) {
        if (!ring_now(s_w)) return false;
        if (s_wpos_lo >= S_need) return true;
        int64_t now = esp_timer_get_time();
        if (now > t_end) return false;
        if (s_idle) {
            int64_t writing = s_tb_s0 + (now - s_tb_t0) * SPS;        /* sample being written now */
            if (S_need - writing > (int64_t)IDLE_MIN_US * SPS && s_idle(s_idle_ctx)) {
                s_idle_info->idle_calls++;
                s_idle_info->idle_us += (int)(esp_timer_get_time() - now);
            }
        }
    }
}

/* ----------------------------------------------------------------- demod */

static inline int bias_for(int step)
{
    return s_bias[step];
}

static void levels_changed(void)
{
    SIM_COST(900);                                  /* soft double arithmetic, lround x 6 */
    for (int k = 0; k < 4; ++k) s_bias[k] = (int)lround(s_center * k);
    s_thr_edge = (int)lround((double)EDGE_W * (s_sync + s_blank) * 0.5);
    /* Pixel scale: 4 x the phase advance over PIX_SAMPLES samples (the
     * [1 2 1] sum) minus that of blanking, 255 at nominal white. */
    double white = (s_blank - s_sync) * (s_pal ? (7.0 / 3.0) : 2.5);    /* per sample, above blanking */
    s_pix_ok = white >= 1.0;
    if (s_pix_ok) {
        s_pix_c0 = (int32_t)lround(4.0 * PIX_SAMPLES * s_blank);
        s_pix_m = (int32_t)lround(255.0 * 1024.0 / (4.0 * PIX_SAMPLES * white));
    }
}

static void set_center(double c)
{
    s_center = c;
    levels_changed();
}

/* Phase advance over samples [S, S + n), reading the angle every `step`
 * samples (n a multiple of step). */
static IRAM_ATTR int32_t sum_dphi_step(int64_t S, int n, int step)
{
    const int bias = bias_for(step);
    uint32_t i = src_index(S - 1);
    uint8_t prev = s_ang[s_src[i]];
    int32_t acc = 0;
    SIM_COST(20 + 10 * (n / step));
    for (int k = n / step; k > 0; --k) {
        SRC_NEXT(i, step);
        uint8_t a = s_ang[s_src[i]];
        acc += (int8_t)(uint8_t)(a - prev - bias) + bias;
        prev = a;
    }
    return acc;
}

static IRAM_ATTR void block_demod(int64_t S0, int nblk, int16_t *out)
{
    const uint8_t *src = s_src;
    const uint8_t *ang = s_ang;
    const int bias = bias_for(BLK_STEP);
    SIM_COST(20 + 78 * nblk);
    uint32_t i = src_index(S0 - 1);
    uint8_t prev = ang[src[i]];
    for (int b = 0; b < nblk; ++b) {
        int32_t acc = 0;
        for (int k = 0; k < BLK / BLK_STEP; ++k) {
            SRC_NEXT(i, BLK_STEP);
            uint8_t a = ang[src[i]];
            acc += (int8_t)(uint8_t)(a - prev - bias) + bias;
            prev = a;
        }
        out[b] = (int16_t)acc;
    }
}

static int16_t s_blk[WIN_BLOCKS];

/* Edge threshold for find_edge(): midway between sync and blanking, scaled
 * to the running-sum length. */
static inline int thr_edge(void)
{
    return s_thr_edge;
}

/* First falling crossing of the EDGE_W-sample running sum below thr in
 * [S_from, S_to), after at least EDGE_LEAD samples above it and followed by
 * sync level (a sync pulse lasts 4.7 us; an FM click, a noise-driven phase
 * slip of a whole turn, dips below the threshold for ~0.1 us only).
 * Sub-sample edge position. Hot loop: 32-bit counter, the bias kept out of
 * the running sum (moved into the threshold). */
static IRAM_ATTR bool find_edge(int64_t S_from, int64_t S_to, int thr, double *edge)
{
    const int bias = bias_for(1);
    const uint8_t *src = s_src;
    const uint8_t *ang = s_ang;
    const int thr0 = thr - EDGE_W * bias;
    uint32_t i = src_index(S_from - EDGE_W);
    uint32_t pi = i;
    SRC_PREV(pi);
    uint8_t prev = ang[src[pi]];
    int8_t win[EDGE_W];
    int sum = 0;
    for (int k = 0; k < EDGE_W; ++k) {
        uint8_t a = ang[src[i]];
        win[k] = (int8_t)(uint8_t)(a - prev - bias);
        sum += win[k];
        prev = a;
        SRC_NEXT(i, 1u);
    }
    const int count = (int)(S_to - S_from);
    int above = 0, prev_sum = sum;
    SIM_COST(200 + 18 * count);
    for (int j = 0; j < count; ++j) {
        uint8_t a = ang[src[i]];
        int d = (int8_t)(uint8_t)(a - prev - bias);
        prev = a;
        SRC_NEXT(i, 1u);
        const int k = j & (EDGE_W - 1);
        sum += d - win[k];
        win[k] = (int8_t)d;
        if (sum >= thr0) {
            above++;
        } else {
            if (above >= EDGE_LEAD) {
                int64_t s = S_from + j;
                int32_t tail = sum_dphi_step(s + 16, 96, 3);   /* 0.4 .. 2.8 us after the crossing */
                if (tail * EDGE_W < thr * 96) {
                    double fr = (double)(prev_sum - thr0) / (double)(prev_sum - sum);
                    *edge = (double)(s - 1) + fr - (EDGE_W - 1) * 0.5;
                    return true;
                }
            }
            above = 0;
        }
        prev_sum = sum;
    }
    return false;
}

/* Sync and blanking level of the line whose sync edge is at E. */
static void line_levels(double E, double *sy, double *bl)
{
    int64_t e = (int64_t)llround(E);
    int blen = s_pal ? BLANK_LEN_PAL : BLANK_LEN_NTSC;
    *sy = (double)sum_dphi_step(e + SYNC_FROM, SYNC_LEN, LVL_STEP) / (double)SYNC_LEN;
    *bl = (double)sum_dphi_step(e + BLANK_FROM, blen, LVL_STEP) / (double)blen;
}

static void track_levels(double sy, double bl)
{
    SIM_COST(700);                                  /* soft double arithmetic */
    if (bl - sy < 2.0) return;                 /* implausible line */
    if (!s_levels_ok) {
        s_sync = sy;
        s_blank = bl;
        s_levels_ok = true;
    } else {
        s_sync += 0.2 * (sy - s_sync);
        s_blank += 0.2 * (bl - s_blank);
    }
    double ratio = s_pal ? (7.0 / 3.0) : 2.5;
    double white = s_blank + ratio * (s_blank - s_sync);
    set_center(0.5 * (s_sync + white));
}

/* Grid position (lines, half-lines allowed) to sample index. */
static inline double grid_at(double x)
{
    return s_SA + (x - (double)s_NA) * s_T;
}

/* ------------------------------------------------------------ vertical */

/* A broad pulse (the vertical interval: 27 us at sync level, twice a line)
 * at grid position x? Normal lines show 4.7 us of sync and then picture,
 * equalizing pulses 2.35 us: neither keeps the mean over 2 .. 17 us low. */
static bool broad_at(double x)
{
    int64_t s = (int64_t)llround(grid_at(x)) + 2 * SPS;
    int32_t sum = sum_dphi_step(s, 15 * SPS, 3);
    return (double)sum < 0.5 * (s_sync + s_blank) * (15.0 * SPS);
}

typedef enum { V_OK, V_SLIP, V_MISS, V_SKIP } vcheck_t;

/* Where is the first broad pulse of the field predicted at grid position
 * `start`? Looks at start and up to 1.5 lines either side in half-line steps
 * (the first broad pulse is the one whose half-line predecessor is not
 * broad). Returns V_SKIP when that part of the signal is no longer in the
 * ring. */
static vcheck_t check_vertical(double start, double *shift, int64_t t_end)
{
    static const int order[7] = { 0, -1, 1, -2, 2, -3, 3 };
    int64_t S_lo = (int64_t)floor(grid_at(start - 2.0)) - 8;
    int64_t S_hi = (int64_t)ceil(grid_at(start + 1.5)) + 17 * SPS + 8;
    int64_t s_w;
    if (!wait_for(S_hi, t_end, &s_w) || S_lo < oldest_valid(s_w)) return V_SKIP;
    ring_sync_m2c(S_lo, (int)(S_hi - S_lo));
    s_src = s_ring;
    s_src_mask = RING_MASK;
    s_src_is_win = false;
    vcheck_t result = V_MISS;
    for (int n = 0; n < 7; ++n) {
        double x = start + 0.5 * order[n];
        if (broad_at(x) && !broad_at(x - 0.5)) {
            *shift = 0.5 * order[n];
            result = order[n] == 0 ? V_OK : V_SLIP;
            break;
        }
    }
    if (!ring_now(&s_w) || S_lo < oldest_valid(s_w)) return V_SKIP;   /* overwritten meanwhile */
    return result;
}

/* ------------------------------------------------------------- acquisition */

static int cmp_i16(const void *a, const void *b)
{
    return (int)*(const int16_t *)a - (int)*(const int16_t *)b;
}

/* Runs of blocks below thr (single-block gaps bridged); runs touching the
 * window edges are dropped. */
static int find_pulses(const int16_t *blk, int nblk, int thr, int64_t S0, pulse_t *p, int maxp)
{
    int n = 0;
    for (int b = 0; b < nblk && n < maxp; ) {
        if (blk[b] >= thr) { ++b; continue; }
        int s = b;
        while (b < nblk && (blk[b] < thr || (b + 1 < nblk && blk[b + 1] < thr))) ++b;
        if (s > 0 && b < nblk) {
            p[n].start = S0 + (int64_t)s * BLK;
            p[n].len = (b - s) * BLK;
            ++n;
        }
    }
    return n;
}

static inline bool is_broad(const pulse_t *p) { return p->len >= 15 * SPS; }
static inline bool is_short(const pulse_t *p) { return p->len >= 1 * SPS && p->len < 7 * SPS / 2; }
static inline bool is_hsync(const pulse_t *p) { return p->len >= 7 * SPS / 2 && p->len <= 7 * SPS; }

static bool refine(const pulse_t *p, double *edge)
{
    return find_edge(p->start - 3 * BLK, p->start + 2 * BLK, thr_edge(), edge);
}

/* Copy [S0 - 64, S0 + WIN_SAMPLES) into s_win with the ring's index mapping
 * and switch the demod functions to it. False if the GDMA overwrote the
 * newest part while copying. */
static bool copy_window(int64_t S0)
{
    SIM_COST(3200 + 6 * (WIN_SAMPLES / 16));
    ring_sync_m2c(S0 - 64, WIN_SAMPLES + 64);
    uint32_t w = 0u;
    for (int64_t S = S0 - 64; S < S0 + WIN_SAMPLES; ) {
        uint32_t r = (uint32_t)S & RING_MASK;
        uint32_t n = (uint32_t)(S0 + WIN_SAMPLES - S);
        if (n > RING - r) n = RING - r;
        if (n > WIN_COPY - w) n = WIN_COPY - w;
        memcpy(s_win + w, s_ring + r, n);
        S += n;
        w += n;
        if (w >= WIN_COPY) w -= WIN_COPY;
    }
    int64_t s_w2;
    if (!ring_now(&s_w2) || !newest_intact(S0 + WIN_SAMPLES - 1, s_w2)) return false;
    s_src = s_win;
    s_src_base = S0 - 64;
    s_src_is_win = true;
    return true;
}

/* Find horizontal sync (line period, standard, levels) and, unless
 * hsync_only, the vertical interval; the latter locks the grid. */
static bool acquire(int timeout_ms, grab_info_t *info, bool hsync_only)
{
    int64_t t_end = esp_timer_get_time() + (int64_t)timeout_ms * 1000;
    int64_t last_w = -1;
    double sum_T = 0.0;
    int n_T = 0;
    bool ok = false;
    /* Static, not stack: the preview task stack is 4 KB (IDF xTaskCreate
     * measures in bytes), and this trio alone is ~2.3 KB. .bss is
     * affordable since row streaming freed the frame buffer. */
    static pulse_t pulses[MAX_PULSES];
    static int16_t sorted[WIN_BLOCKS];

    s_levels_ok = false;
    while (esp_timer_get_time() < t_end) {
        int64_t s_w;
        if (!ring_now(&s_w)) { info->error = "ring pointer lost"; break; }
        if (s_w == last_w) continue;             /* wait for a fresh descriptor */
        last_w = s_w;
        int64_t S0 = s_w - WIN_SAMPLES;
        int64_t tw = esp_timer_get_time();
        if (!copy_window(S0)) { info->late++; continue; }
        for (int64_t S = S0; S < S0 + WIN_SAMPLES; S += 16) {
            uint8_t b = s_win[src_index(S)];
            s_pw_sum += s_pow[b];
            s_clip_n += s_clp[b];
            s_pw_n++;
        }
        block_demod(S0, WIN_BLOCKS, s_blk);
        info->windows++;
        info->window_us += (int)(esp_timer_get_time() - tw);

        /* Block threshold: tracked levels, or percentiles of this window. */
        int thr;
        if (s_levels_ok) {
            thr = (int)lround(BLK * (s_sync + s_blank) * 0.5);
        } else {
            memcpy(sorted, s_blk, sizeof(sorted));
            qsort(sorted, WIN_BLOCKS, sizeof(sorted[0]), cmp_i16);
            SIM_COST(15000);                        /* qsort of 384 blocks (about 110 us on hardware) */
            /* Re-centre the unwrap on this window first (unknown carrier
             * offset); the next window is demodulated around it. */
            double med = (double)sorted[WIN_BLOCKS / 2] / BLK;
            if (fabs(med - s_center) > 6.0) {
                set_center(med);
                continue;
            }
            int32_t low = 0;
            int nlow = WIN_BLOCKS / 20;
            for (int k = 0; k < nlow; ++k) low += sorted[k];
            int sync_blk = low / nlow;
            int hi = sorted[WIN_BLOCKS * 98 / 100];
            if (hi - sync_blk < 4 * BLK) continue;   /* flat: no carrier or no video */
            thr = sync_blk + (hi - sync_blk) / 5;
            s_sync = (double)sync_blk / BLK;
            s_blank = (double)(2 * thr - sync_blk) / BLK;   /* provisional, midpoint = thr */
            levels_changed();
        }

        int np = find_pulses(s_blk, WIN_BLOCKS, thr, S0, pulses, MAX_PULSES);
        SIM_COST(8 * WIN_BLOCKS + 300 * np);

        /* Horizontal sync: line period and levels from consecutive pulses
         * whose level windows lie inside the copy. */
        double prev_edge = -1.0;
        for (int k = 0; k < np; ++k) {
            if (!is_hsync(&pulses[k]) || pulses[k].start + 440 > S0 + WIN_SAMPLES) { prev_edge = -1.0; continue; }
            double e;
            if (!refine(&pulses[k], &e)) { prev_edge = -1.0; continue; }
            double sy, bl;
            line_levels(e, &sy, &bl);
            track_levels(sy, bl);
            if (prev_edge > 0.0) {
                double sp = e - prev_edge;
                if (sp > 2480.0 && sp < 2620.0) {
                    sum_T += sp;
                    n_T++;
                }
            }
            prev_edge = e;
        }
        if (n_T < 4) continue;
        s_pal = sum_T / n_T > 2551.0;
        s_T = s_pal ? 2560.0 : 2542.4;           /* nominal; tracked from here on */
        s_T_min = s_T * 0.9995;                  /* the camera's line clock is within 500 ppm */
        s_T_max = s_T * 1.0005;
        levels_changed();                        /* the white level depends on the standard */
        if (hsync_only) { ok = true; break; }

        /* Vertical interval: short pulse then the first broad pulse half a
         * line later, or the last broad pulse followed by a short one. */
        double half = s_T * 0.5, tol = 3.0 * SPS;
        int fb = -1, lb = -1;
        for (int k = 0; k < np; ++k) {
            if (is_broad(&pulses[k])) {
                if (fb < 0) fb = k;
                lb = k;
            }
        }
        if (fb < 0) continue;
        double sv = -1.0;
        if (fb >= 1 && is_short(&pulses[fb - 1]) &&
            fabs((double)(pulses[fb].start - pulses[fb - 1].start) - half) < tol) {
            double e;
            if (refine(&pulses[fb], &e)) sv = e;
        } else if (lb + 1 < np && is_short(&pulses[lb + 1]) &&
                   fabs((double)(pulses[lb + 1].start - pulses[lb].start) - half) < tol) {
            double e;
            int n_broad = s_pal ? 5 : 6;
            if (refine(&pulses[lb], &e)) sv = e - (double)(n_broad - 1) * half;
        }
        if (sv < 0.0) continue;

        /* Tentative grid: line 0 starts at the broad pulse (a field starting
         * on a line boundary). The first captured row settles the parity. */
        s_SA = sv;
        s_NA = 0;
        s_F0 = 0.0;
        s_locked = true;
        s_parity_pending = true;
        ok = true;
        break;
    }
    s_src = s_ring;
    s_src_mask = RING_MASK;
    s_src_is_win = false;
    if (!ok && !info->error)
        info->error = n_T < 4 ? "no horizontal sync (no carrier or no video?)" : "no vertical interval found";
    return ok;
}

/* ------------------------------------------------------------------ capture */

/* GRAB_W pixels of PIX_SAMPLES samples from S_act: phase advance per pixel,
 * optionally smoothed [1 2 1], scaled so that 0 = blanking and 255 =
 * nominal white: v = ((4 raw - c0) m) >> 10 (see capture_row()). This is the
 * per-row hot loop (672 angle reads): unrolled, no index masking unless the
 * row wraps around the ring end, 32-bit arithmetic only (the C5 has no FPU
 * and no fast 64-bit multiply), and the power statistics for the gain
 * control come from 28 samples per row. */
static IRAM_ATTR void extract_pixels(int64_t S_act, uint8_t *dst, int32_t c0, int32_t m)
{
    /* Static: the task stack is 4 KB and this is 448 B on the deepest path
     * (per-row pixel extraction). int16: raw telescopes to (a3 - prev)
     * <= 255 plus 3*bias, |raw| < 2^15 by construction. */
    static int16_t raw[GRAB_W];
    const uint8_t *src = s_src;
    const uint8_t *ang = s_ang;
    const int bias = bias_for(PIX_STEP);
    const int32_t bias3 = 3 * bias;
    uint32_t i = src_index(S_act - 1);
    SIM_COST(GRAB_W * (28 + 12) + 28 * 8 + 100);
    if (i + (uint32_t)(GRAB_W * PIX_SAMPLES) < src_run_limit()) {
        const uint8_t *p = src + i;
        uint8_t prev = ang[p[0]];
        for (int px = 0; px < GRAB_W; ++px) {
            uint8_t a1 = ang[p[3]], a2 = ang[p[6]], a3 = ang[p[9]];
            raw[px] = (int8_t)(uint8_t)(a1 - prev - bias) + (int8_t)(uint8_t)(a2 - a1 - bias) +
                      (int8_t)(uint8_t)(a3 - a2 - bias) + bias3;
            prev = a3;
            p += PIX_SAMPLES;
        }
        uint32_t pw = 0u, clipped = 0u;
        for (int k = 0; k < GRAB_W * PIX_SAMPLES; k += 8 * PIX_SAMPLES) {
            uint8_t b = src[i + 1u + (uint32_t)k];
            pw += s_pow[b];
            clipped += s_clp[b];
        }
        s_pw_sum += pw;
        s_clip_n += clipped;
        s_pw_n += (uint32_t)(GRAB_W / 8);
    } else {
        uint8_t prev = ang[src[i]];
        for (int px = 0; px < GRAB_W; ++px) {
            int32_t acc = 0;
            for (int k = 0; k < PIX_SAMPLES / PIX_STEP; ++k) {
                SRC_NEXT(i, PIX_STEP);
                uint8_t a = ang[src[i]];
                acc += (int8_t)(uint8_t)(a - prev - bias) + bias;
                prev = a;
            }
            raw[px] = acc;
        }
    }
    if (s_smooth) {
        int32_t left = raw[0], cur = raw[0];
        for (int px = 0; px < GRAB_W - 1; ++px) {
            int32_t right = raw[px + 1];
            int32_t v = ((left + 2 * cur + right - c0) * m) >> 10;
            dst[px] = (uint8_t)(v < 0 ? 0 : v > 255 ? 255 : v);
            left = cur;
            cur = right;
        }
        int32_t v = ((left + 3 * cur - c0) * m) >> 10;
        dst[GRAB_W - 1] = (uint8_t)(v < 0 ? 0 : v > 255 ? 255 : v);
    } else {
        for (int px = 0; px < GRAB_W; ++px) {
            int32_t v = ((4 * raw[px] - c0) * m) >> 10;
            dst[px] = (uint8_t)(v < 0 ? 0 : v > 255 ? 255 : v);
        }
    }
}

static unsigned s_level_rows;

#ifdef GRAB_DEBUG
/* Host debugging only (grab_sim built with -DGRAB_DEBUG): per-row timing,
 * printed after the grab. */
#define GRAB_DEBUG_MAX 2048
typedef struct { int row; int32_t N; int64_t t; int lag, margin, proc; } grab_dbg_t;
static grab_dbg_t g_dbg[GRAB_DEBUG_MAX];
static int g_dbg_n;
void grab_debug_dump(void)
{
    for (int k = 0; k < g_dbg_n; ++k)
        printf("  row %3d N %6ld  t %8lld us  lag %5d us  margin %5d us  proc %4d us\n", g_dbg[k].row,
               (long)g_dbg[k].N, (long long)g_dbg[k].t, g_dbg[k].lag, g_dbg[k].margin, g_dbg[k].proc);
    g_dbg_n = 0;
}
#endif

typedef enum {
    ROW_OK,
    ROW_BEHIND,         /* the line has already left the ring: a later one may do */
    ROW_FAILED,         /* no sync, overwritten while in use, or out of time */
} row_result_t;

static row_result_t capture_row(int32_t N, int row, uint8_t *dst, grab_info_t *info, jitter_t *jit, int64_t t_end)
{
    SIM_COST(1500);                                 /* grid and flywheel: about 20 soft double operations */
    s_src = s_ring;
    s_src_mask = RING_MASK;
    s_src_is_win = false;
    double S_pred = s_SA + (double)(N - s_NA) * s_T;
    const int64_t S_p = (int64_t)floor(S_pred);
    const int T_int = (int)s_T;
    bool first_row = s_parity_pending || s_resync_pending;
    int half = first_row ? T_int / 2 + 64 : NARROW_SEARCH;
    int64_t S_from = S_p - half - EDGE_LEAD;
    int64_t S_to = S_p + half;
    int64_t S_need = S_p + half + T_int + 128;

    /* Wait for the line (rows near the bottom of the next field can be more
     * than a field away: a fixed short deadline here once let the field loop
     * run ahead of real time and starve such a row for the whole grab). */
    int64_t s_w;
    if (!wait_for(S_need, t_end, &s_w)) return ROW_FAILED;
    const int64_t t_ready = esp_timer_get_time();
#ifdef GRAB_DEBUG
    if (g_dbg_n < GRAB_DEBUG_MAX) {
        grab_dbg_t *d = &g_dbg[g_dbg_n++];
        d->row = row;
        d->N = N;
        d->t = t_ready;
        d->lag = (int)((s_wpos - S_need) / SPS);
        d->margin = (int)((S_from - EDGE_W - 16 - oldest_valid(s_w)) / SPS);
        d->proc = -1;
    }
#endif
    if (S_from - EDGE_W - 16 < oldest_valid(s_w)) { trace(N, 0.0, row, 'l'); return ROW_BEHIND; }

    /* Cover the widest possible read: the +/-1 us narrow search and the
     * whole-line fallback, plus find_edge's EDGE_W + 1-sample lookback. */
    ring_sync_m2c(S_p - T_int / 2 - EDGE_W - 64, T_int + EDGE_W + 128);
    double E;
    bool wide = false;
    if (!find_edge(S_from, S_to, thr_edge(), &E)) {
        /* Not within +/-2.5 us: look over the whole line before giving up. */
        int64_t W_from = S_p - T_int / 2;
        int64_t W_to = S_p + T_int / 2;
        if (first_row || W_from - EDGE_W - 16 < oldest_valid(s_w) ||
            !find_edge(W_from, W_to, thr_edge(), &E)) {
            info->nosync++;
            trace(N, 0.0, row, 'n');
            return ROW_FAILED;
        }
        wide = true;
        info->wide_hits++;
    }
    double e = E - S_pred;
    if (!first_row) {
        if (fabs(e) > 20.0) info->big_err++;
        if ((int)fabs(e) > info->max_err) info->max_err = (int)fabs(e);
    }
    trace(N, e, row, first_row ? 'r' : wide ? 'w' : 'o');

    /* Flywheel (see the file comment). The grid follows the edges with
     * STEER; the line period comes from the steered grid itself over
     * T_SPAN lines or more (the grid tracks the true sync positions, so its
     * slope is the true period even while s_T is still off, and over that
     * span the edge noise averages out). Without it the period stayed at
     * nominal while steering hid the error row to row, and a row retried a
     * frame later landed hundreds of samples off (camera 120 ppm off). */
    double S_line;
    if (s_parity_pending) {
        /* Label this line N; the field start sits on the grid or half a
         * line off it. */
        double f0 = (double)N - (E - s_SA) / s_T;
        s_F0 = floor(f0 * 2.0 + 0.5) * 0.5;
        s_parity_pending = false;
        S_line = E;
        s_P_ok = false;
    } else if (s_resync_pending || wide || fabs(e) > BIG_ERROR) {
        if (s_resync_pending && N - s_NA >= T_SPAN && fabs(e) < s_T * 0.25) s_T += 0.5 * e / (double)(N - s_NA);
        s_resync_pending = false;
        S_line = E;
    } else {
        jit->err_sq += e * e;
        jit->err_n++;
        S_line = S_pred + STEER * e;
    }
    if (!s_P_ok) {
        s_SP = S_line;
        s_NP = N;
        s_P_ok = true;
    } else if (N - s_NP >= T_SPAN) {
        double Tm = (S_line - s_SP) / (double)(N - s_NP);
        if (Tm > s_T_min && Tm < s_T_max) s_T += 0.5 * (Tm - s_T);
        s_SP = S_line;
        s_NP = N;
    }
    if (s_T < s_T_min) s_T = s_T_min;
    if (s_T > s_T_max) s_T = s_T_max;
    s_SA = S_line;
    s_NA = N;

    /* The levels drift slowly: measuring them on every fourth row is plenty
     * (and saves a third of the row's floating-point work). */
    const int64_t S_int = (int64_t)llround(S_line);
    if ((++s_level_rows & 3) == 0) {
        double sy, bl;
        line_levels(S_line, &sy, &bl);
        track_levels(sy, bl);
    }
    if (!s_pix_ok) return ROW_FAILED;
    ring_sync_m2c(S_int + (s_pal ? ACTIVE_PAL : ACTIVE_NTSC) - 64,
                  GRAB_W * PIX_SAMPLES + 128);
    extract_pixels(S_int + (s_pal ? ACTIVE_PAL : ACTIVE_NTSC), dst, s_pix_c0, s_pix_m);

    /* The newest sample used must not have been overwritten meanwhile. */
    bool intact = ring_now(&s_w) && newest_intact(S_int + 2560, s_w);
    int64_t t_done = esp_timer_get_time();
    info->cpu_us += (int)(t_done - t_ready);
#ifdef GRAB_DEBUG
    if (g_dbg_n > 0) g_dbg[g_dbg_n - 1].proc = (int)(t_done - t_ready);
#endif
    if (!intact) { info->late++; return ROW_FAILED; }
    return ROW_OK;
}

static void finish_info(grab_info_t *info, int64_t t0, const jitter_t *jit, const uint8_t *done)
{
    info->pal = s_pal;
    info->line_us = (float)(s_T / SPS);
    info->sync_level = (float)s_sync;
    info->blank_level = (float)s_blank;
    info->carrier_mhz = (float)(s_center * (SPS / 256.0));      /* 256 phase units per sample = 40 MHz */
    info->jitter_ns = jit && jit->err_n ? (float)(sqrt(jit->err_sq / jit->err_n) * 25.0) : 0.0f;
    info->grab_ms = (int)((esp_timer_get_time() - t0) / 1000);
    info->power = s_pw_n ? (float)s_pw_sum / (float)s_pw_n : 0.0f;
    info->clip = s_pw_n ? (float)s_clip_n / (float)s_pw_n : 0.0f;
    info->fifo_ovf = PARL_IO.int_raw.rx_fifo_wovf_int_raw ? 1 : 0;
    info->tb_bad = s_tb_bad;
    info->tb_lead_min_ns = s_tb_lead_min <= s_tb_lead_max ? s_tb_lead_min * 25 : 0;
    info->tb_lead_max_ns = s_tb_lead_min <= s_tb_lead_max ? s_tb_lead_max * 25 : 0;
    info->missing_row = -1;
    if (done) {
        for (int y = 0; y < GRAB_H; ++y)
            if (!done[y]) { info->missing_row = y; break; }
    }
}

int grab_frame(uint8_t *img, grab_info_t *info, int timeout_ms, grab_row_cb_t on_row, grab_idle_cb_t on_idle, void *ctx)
{
    /* Stack, not static: SRAM is the binding constraint (integration notes
     * above). Only img[0..GRAB_W) is used: every row is staged there and
     * handed to on_row at once (grab.h). */
    uint8_t done[GRAB_H], failed[GRAB_H];
    memset(info, 0, sizeof(*info));
    memset(done, 0, sizeof(done));
    memset(failed, 0, sizeof(failed));
    s_idle = NULL;                      /* not while acquiring: the search windows are timed */
    s_idle_ctx = ctx;
    s_idle_info = info;
    s_pw_sum = s_pw_n = s_clip_n = 0u;
    PARL_IO.int_clr.rx_fifo_wovf_int_clr = 1;
    int64_t t0 = esp_timer_get_time();
    int64_t t_end = t0 + (int64_t)timeout_ms * 1000;

    if (!timebase_init()) {
        info->error = "RX ring not moving";
        finish_info(info, t0, NULL, NULL);
        return 0;
    }
    /* Between grabs the grid is only extrapolated. Back to back (streaming)
     * that is good to a fraction of a microsecond and the narrow search
     * simply goes on; after a pause of more than REANCHOR_US one search over
     * the whole line re-anchors it (costly: rows behind it can fall late),
     * after a long one look for the vertical interval again. */
    if (s_locked) {
        int64_t gap = t0 - s_last_grab_us;
        if (gap > RESYNC_US) grab_unlock();
        else if (gap > REANCHOR_US) s_resync_pending = true;
    }
    if (!s_locked) {
        if (!acquire(timeout_ms, info, false)) {
            finish_info(info, t0, NULL, NULL);
            s_last_grab_us = esp_timer_get_time();
            return 0;
        }
        info->acquire_ms = (int)((esp_timer_get_time() - t0) / 1000);
    }

    const double lpf = s_pal ? 312.5 : 262.5;
    const double first = s_pal ? 22.0 : 17.0;       /* active lines after the first broad pulse */
    const int active = s_pal ? 288 : 240;           /* active lines per field */
    const int picture = 2 * active;                 /* interlaced picture lines */
    jitter_t jit = { 0.0, 0 };
    int misses = 0, vmiss_run = 0;
    int64_t f = INT64_MIN;
    s_idle = on_idle;

    while (info->rows < GRAB_H && esp_timer_get_time() < t_end && s_locked) {
        int64_t s_w;
        if (!ring_now(&s_w)) break;
        /* Start with the field that holds the oldest line still in the ring;
         * its rows before that line come from the field after next. */
        const double n_old = (double)s_NA + ((double)s_w - s_SA) / s_T - CATCH_LINES;
        const int64_t f_now = (int64_t)floor((n_old - s_F0) / lpf);
        if (f < f_now) f = f_now;
        double start = s_F0 + (double)f * lpf;
        info->fields++;

        /* Vertical flywheel: is the field where the grid says? (Not in the
         * field the lock was just found in; skipped when the vertical
         * interval has already left the ring.) */
        if (!s_parity_pending) {
            double shift = 0.0;
            vcheck_t v = check_vertical(start, &shift, t_end);
            if (v != V_SKIP) info->vchecks++;
            if (v == V_OK) {
                vmiss_run = 0;
            } else if (v == V_SLIP) {
                info->vslips++;
                s_F0 += shift;
                start += shift;
                vmiss_run = 0;
            } else if (v == V_MISS) {
                info->vmisses++;
                if (++vmiss_run >= MAX_VMISSES) {
                    grab_unlock();
                    info->error = "lost vertical sync";
                    break;
                }
            }
        }
        int parity = (fabs(start - floor(start + 0.5)) < 0.25) ? 0 : 1;
        const double F0_field = s_F0;
        const int32_t N_base = (int32_t)floor(start + first + 0.5);   /* line of picture line p = N_base + p / 2 */
        const int32_t N_min = (int32_t)ceil(n_old);                    /* older lines have left the ring */

        for (int y = 0; y < GRAB_H && s_locked; ++y) {
            if (done[y]) continue;
            int p = y * picture / GRAB_H;
            if ((p & 1) != parity) continue;
            int32_t N = N_base + (p >> 1);
            if (N < N_min) { info->skipped++; continue; }
            SIM_COST(100);                          /* loop bookkeeping */
            int nosync_before = info->nosync;
            int64_t tr = esp_timer_get_time();
            uint8_t *dst = img;     /* GRAB_W-byte row staging; see grab.h */
            row_result_t r = capture_row(N, y, dst, info, &jit, t_end);
            /* Behind (a burst of close rows, an interrupt): the next line of
             * the same field, half an output row lower, beats waiting two
             * fields for this one. */
            for (int k = 1; r == ROW_BEHIND && k <= 2 && (p >> 1) + k < active; ++k) {
                r = capture_row(N + k, y, dst, info, &jit, t_end);
                if (r == ROW_OK) info->substituted++;
            }
            if (r == ROW_BEHIND) info->late++;
            if (r != ROW_OK) failed[y] = 1;
            if (r == ROW_OK) {
                done[y] = 1;
                info->rows++;
                int64_t tc = esp_timer_get_time();
                info->row_us += (int)(tc - tr);
                misses = 0;
                if (on_row) {
                    on_row(ctx, y, dst);
                    info->cb_us += (int)(esp_timer_get_time() - tc);
                }
            } else if (info->nosync != nosync_before && ++misses >= MAX_MISSES) {
                /* Only missing syncs mean the model is wrong; late rows just
                 * mean the CPU fell behind and are retried in a later field. */
                grab_unlock();
                info->error = "lost sync";
            }
            if (esp_timer_get_time() > t_end) break;
            if (s_F0 != F0_field) break;     /* first row after the lock moved the field start by half a line */
        }
        if (s_F0 != F0_field) continue;      /* same field again, on the corrected grid */
        f++;

        /* A few rows that failed once each are not worth two more fields:
         * the display keeps the previous frame's row there. */
        int missing = 0, retried = 0;
        for (int y = 0; y < GRAB_H; ++y) {
            if (done[y]) continue;
            missing++;
            retried += failed[y];
        }
        if (missing > 0 && missing <= GIVE_UP_ROWS && retried == missing) {
            info->given_up = missing;
            break;
        }
    }

    s_idle = NULL;
    finish_info(info, t0, &jit, done);
    s_last_grab_us = esp_timer_get_time();
    if (info->rows < GRAB_H - info->given_up && !info->error) info->error = "timeout";
    return info->rows;
}

bool grab_probe(int timeout_ms, grab_info_t *info)
{
    memset(info, 0, sizeof(*info));
    s_pw_sum = s_pw_n = s_clip_n = 0u;
    int64_t t0 = esp_timer_get_time();
    grab_unlock();
    bool ok = timebase_init() && acquire(timeout_ms, info, true);
    finish_info(info, t0, NULL, NULL);
    return ok;
}
