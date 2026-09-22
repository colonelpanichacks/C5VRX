/**
 * grab_minisim.c - minimal host bench for main/grab.c (C5VRX integration).
 *
 * A trimmed port of the upstream test/host/grab_sim.c signal chain (synthetic
 * PAL/NTSC composite -> FM -> Q4/I4 bytes -> 16 KiB ring with the PARLIO
 * descriptor layout, time advanced by SIM_COST instruction accounting) with
 * the UART/link-encoder layer removed: rows are reassembled from the row
 * callback exactly the way main/preview.c consumes them (GRAB_W-byte staging
 * buffer, copy out in the callback), and the standard pattern metrics
 * (vertical marker line, horizontal black diagonal, grey-bar noise, edge
 * width) are checked on the assembled frame.
 *
 * This validates the integration adaptations against synthetic CVBS:
 * WIN_SAMPLES/WIN_COPY 6080/6144, the row-streaming img contract, and
 * PAL + NTSC lock/capture/stability. It does NOT build main/preview.c.
 *
 * build: cc -O2 -DGRAB_SIM_COST -Itest/host/stubs -Imain \
 *          test/host/grab_minisim.c main/grab.c -lm -o /tmp/grab_minisim
 * usage: grab_minisim [--std pal|ntsc] [--frames N] [--noise SIGMA] [--cfo MHZ]
 *                     [--ppm PPM] [--nosignal] [--seed N]
 */

#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#include "grab.h"
#include "video.h"

#define RING VIDEO_RX_RING_BYTES
#define FS   40e6

static uint8_t g_ring[RING];
static int64_t g_written;
static double  g_time_us = 1000.0;
static double  g_phase;
static bool    g_pal = true;
static double  g_noise = 0.6;
static double  g_cfo = 0.3e6;
static double  g_radius = 5.0;
static double  g_hz_per_volt = 7.8e6;
static double  g_ppm;
static double  g_line_us;
static int     g_lines;
static int64_t g_start_offset = 123457;
static bool    g_nosignal;
static uint32_t g_rng = 2463534242u;
static double  g_cost_instr;
static bool    g_cb_violation;
static int     g_last_cb_row = -1;

#define CPI_C5       1.73
#define C5_MHZ       240.0
#define TIMER_INSTR  60

void grab_sim_cost(uint32_t instructions)
{
    g_cost_instr += instructions;
}

/* Host monotonic clock, microseconds (reporting only). */
static double host_now_us(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (double)ts.tv_sec * 1e6 + (double)ts.tv_nsec * 1e-3;
}

/* ------------------------------------------------------------ stub API */

static void advance_to(int64_t S);
static void model_clock(void);

const uint8_t *video_rx_ring(void) { return g_ring; }
uint32_t video_rx_max_descriptor(void) { return 4092u; }

bool video_rx_write_offset(uint32_t *offset)
{
    if (g_nosignal) return false;       /* RX never started */
    g_cost_instr += 10;
    model_clock();
    uint32_t pos = (uint32_t)(g_written % RING);
    static const uint32_t starts[5] = { 0u, 4092u, 8184u, 12276u, 16368u };
    uint32_t s = 0u;
    for (int i = 0; i < 5; ++i)
        if (pos >= starts[i]) s = starts[i];
    *offset = s;
    return true;
}

/* ------------------------------------------------------------ test signal */

static double pattern(double x, double y)       /* 0..1 luma, x, y in [0,1) */
{
    if (fabs(y - 0.5) < 0.004) return 1.0;                  /* marker line */
    double cx = x - 0.5, cy = (y - 0.5) * 0.75;
    double r = sqrt(cx * cx + cy * cy);
    if (fabs(r - 0.28) < 0.012) return 1.0;                 /* white ring */
    if (fabs(x - y) < 0.01) return 0.0;                     /* black diagonal */
    if (y < 0.33) return floor(x * 8.0) / 7.0;              /* 8 grey bars */
    if (y < 0.66) return x;                                 /* horizontal ramp */
    return ((int)(x * 16) + (int)(y * 12)) & 1 ? 0.85 : 0.15;   /* checkerboard */
}

static char pal_half(int L, int h)
{
    if (L == 623) return h ? 'e' : 'n';
    if (L == 624 || L == 625) return 'e';
    if (L == 1 || L == 2) return 'b';
    if (L == 3) return h ? 'e' : 'b';
    if (L == 4 || L == 5) return 'e';
    if (L == 311 || L == 312) return 'e';
    if (L == 313) return h ? 'b' : 'e';
    if (L == 314 || L == 315) return 'b';
    if (L == 316 || L == 317) return 'e';
    if (L == 318) return h ? 'x' : 'e';
    return 'n';
}

static char ntsc_half(int L, int h)
{
    int hl = (L - 1) * 2 + h;
    if (hl < 6) return 'e';
    if (hl < 12) return 'b';
    if (hl < 18) return 'e';
    int h2 = hl - 525;
    if (h2 >= 0 && h2 < 6) return 'e';
    if (h2 >= 6 && h2 < 12) return 'b';
    if (h2 >= 12 && h2 < 18) return 'e';
    return 'n';
}

static double composite(int64_t s)
{
    double t_us = (double)(s + g_start_offset) / 40.0;
    double frame_us = g_line_us * g_lines;
    double tf = fmod(t_us, frame_us);
    int L = (int)(tf / g_line_us) + 1;
    double tl = tf - (L - 1) * g_line_us;
    int h = tl >= g_line_us * 0.5 ? 1 : 0;
    double th = tl - h * g_line_us * 0.5;
    char k = g_pal ? pal_half(L, h) : ntsc_half(L, h);
    const double SYNC = -0.3, BLANK = 0.0;
    switch (k) {
    case 'e': return th < 2.35 ? SYNC : BLANK;
    case 'b': return th < (g_line_us * 0.5 - 4.7) ? SYNC : BLANK;
    case 'x': return BLANK;
    default: break;
    }
    if (tl < 4.7) return SYNC;
    double a0 = g_pal ? 10.5 : 9.4, alen = g_pal ? 52.0 : 52.6;
    int first1 = g_pal ? 23 : 21, last1 = g_pal ? 310 : 262;
    int first2 = g_pal ? 336 : 284, last2 = g_pal ? 622 : 524;
    int row = -1, rows = g_pal ? 576 : 484;
    if (L >= first1 && L <= last1) row = 2 * (L - first1);
    else if (L >= first2 && L <= last2) row = 2 * (L - first2) + 1;
    if (row < 0 || tl < a0 || tl >= a0 + alen) return BLANK;
    return 0.7 * pattern((tl - a0) / alen, (double)row / rows);
}

#define GAUSS_N  65536
#define SIN_N    4096
static float    g_gauss[GAUSS_N];
static float    g_sin[SIN_N + SIN_N / 4];

static double gauss_slow(void)
{
    double u1 = (rand() + 1.0) / (RAND_MAX + 2.0), u2 = (rand() + 1.0) / (RAND_MAX + 2.0);
    return sqrt(-2.0 * log(u1)) * cos(2.0 * M_PI * u2);
}

static void tables_init(void)
{
    for (int k = 0; k < GAUSS_N; ++k) g_gauss[k] = (float)gauss_slow();
    for (int k = 0; k < SIN_N + SIN_N / 4; ++k) g_sin[k] = (float)sin(2.0 * M_PI * k / SIN_N);
}

static inline float gauss(void)
{
    g_rng ^= g_rng << 13;
    g_rng ^= g_rng >> 17;
    g_rng ^= g_rng << 5;
    return g_gauss[g_rng & (GAUSS_N - 1)];
}

static int q4(double v)
{
    int n = (int)lround(v);
    return n < -8 ? -8 : n > 7 ? 7 : n;
}

static void advance_to(int64_t S)
{
    while (g_written < S) {
        double amp = g_radius;
        double f = g_cfo + g_hz_per_volt * composite(g_written);
        g_phase += f / FS;
        g_phase -= floor(g_phase);
        int k = (int)(g_phase * SIN_N) & (SIN_N - 1);
        int i = q4(amp * g_sin[k + SIN_N / 4] + g_noise * gauss());
        int q = q4(amp * g_sin[k] + g_noise * gauss());
        g_ring[g_written % RING] = (uint8_t)(((i & 15) << 4) | (q & 15));
        g_written++;
    }
}

static void model_clock(void)
{
    g_time_us += g_cost_instr * CPI_C5 / C5_MHZ;
    g_cost_instr = 0.0;
    advance_to((int64_t)(g_time_us * 40.0));
}

int64_t esp_timer_get_time(void)
{
    g_cost_instr += TIMER_INSTR;
    model_clock();
    return (int64_t)g_time_us;
}

/* ------------------------------------------------------------ display side */

static uint8_t g_img[GRAB_H][GRAB_W];

/* Reassemble rows from the staging buffer, exactly like preview.c's row
 * callback must: copy out before returning. Also verify the staging contract
 * (every call carries the row just captured, and the buffer is not aliased
 * across rows in a way that would corrupt a synchronous copy). */
static void on_row(void *ctx, int y, const uint8_t *row)
{
    (void)ctx;
    if (y < 0 || y >= GRAB_H) { g_cb_violation = true; return; }
    memcpy(g_img[y], row, GRAB_W);
    if (y <= g_last_cb_row && !(y == 0 && g_last_cb_row == GRAB_H - 1))
        ;   /* row order may revisit across fields; the done[] map prevents repeats */
    g_last_cb_row = y;
}

typedef struct {
    int marker_row;      /* row of the white marker line (expected ~84) */
    double diag_col;     /* column of the black diagonal on row 70 (~93) */
    double noise;        /* std of flat bar 6 */
    double edge_px;      /* 10..90 % width of the bar5/bar6 edge */
} metrics_t;

static metrics_t measure(void)
{
    metrics_t m = { -1, -1.0, 0.0, 0.0 };
    double best = -1.0;
    for (int y = 60; y < 110; ++y) {
        double s = 0.0;
        for (int x = 16; x < 48; ++x) s += g_img[y][x];
        if (s > best) { best = s; m.marker_row = y; }
    }
    {
        int y = 70, lo = 60, hi = 130, arg = lo;
        for (int x = lo; x < hi; ++x) if (g_img[y][x] < g_img[y][arg]) arg = x;
        m.diag_col = arg;
    }
    double sum = 0.0, sum2 = 0.0;
    int n = 0;
    for (int y = 8; y < 46; ++y)
        for (int x = 172; x < 193; ++x) { sum += g_img[y][x]; sum2 += (double)g_img[y][x] * g_img[y][x]; n++; }
    double mean = sum / n;
    m.noise = sqrt(sum2 / n - mean * mean);
    double prof[20] = { 0 };
    for (int y = 8; y < 46; ++y)
        for (int k = 0; k < 20; ++k) prof[k] += g_img[y][158 + k];
    double lo_v = prof[0], hi_v = prof[19];
    double t10 = lo_v + 0.1 * (hi_v - lo_v), t90 = lo_v + 0.9 * (hi_v - lo_v);
    double x10 = -1, x90 = -1;
    for (int k = 1; k < 20; ++k) {
        if (x10 < 0 && prof[k] >= t10) x10 = k - 1 + (t10 - prof[k - 1]) / (prof[k] - prof[k - 1] + 1e-9);
        if (x90 < 0 && prof[k] >= t90) x90 = k - 1 + (t90 - prof[k - 1]) / (prof[k] - prof[k - 1] + 1e-9);
    }
    m.edge_px = (x10 >= 0 && x90 >= 0) ? x90 - x10 : -1.0;
    return m;
}

/* ------------------------------------------------------------ main */

int main(int argc, char **argv)
{
    int frames = 4, seed = 1;
    for (int a = 1; a < argc; ++a) {
        const char *o = argv[a];
        const char *v = a + 1 < argc ? argv[a + 1] : "";
        if (!strcmp(o, "--std")) { g_pal = strcmp(v, "ntsc") != 0; ++a; }
        else if (!strcmp(o, "--frames")) { frames = atoi(v); ++a; }
        else if (!strcmp(o, "--noise")) { g_noise = atof(v); ++a; }
        else if (!strcmp(o, "--cfo")) { g_cfo = atof(v) * 1e6; ++a; }
        else if (!strcmp(o, "--ppm")) { g_ppm = atof(v); ++a; }
        else if (!strcmp(o, "--seed")) { seed = atoi(v); ++a; }
        else if (!strcmp(o, "--nosignal")) { g_nosignal = true; }
        else { fprintf(stderr, "unknown option %s\n", o); return 2; }
    }
    g_line_us = (g_pal ? 64.0 : 63.5556) * (1.0 + g_ppm * 1e-6);
    g_lines = g_pal ? 625 : 525;
    srand((unsigned)seed);
    tables_init();
    g_rng += (uint32_t)seed * 2654435761u;
    if (!g_nosignal) advance_to((int64_t)(g_time_us * 40.0));
    grab_init();

    if (g_nosignal) {
        grab_info_t info;
        bool found = grab_probe(60, &info);
        printf("probe: %s (error=%s)\n", found ? "VIDEO" : "none",
               info.error ? info.error : "none");
        return found ? 1 : 0;
    }

    static uint8_t stage[GRAB_W];       /* the row-streaming contract under test */
    int complete = 0, good = 0, marker_first = -1, marker_moves = 0;
    double diag_first = -1.0, diag_max_dev = 0.0, noise_sum = 0.0, edge_sum = 0.0;
    int measured = 0;
    double t0 = host_now_us();
    for (int n = 0; n < frames; ++n) {
        memset(g_img, 0, sizeof(g_img));
        grab_info_t info;
        int rows = grab_frame(stage, &info, 400, on_row, NULL, NULL);
        printf("frame %d: rows=%d %s T=%.4fus fields=%d acq=%dms grab=%dms late=%d nosync=%d wide=%d big=%d "
               "jit=%.0fns v=%d/%d/%d skip=%d sub=%d gave=%d win=%d err=%s\n",
               n, rows, info.pal ? "PAL" : "NTSC", (double)info.line_us, info.fields,
               info.acquire_ms, info.grab_ms, info.late, info.nosync, info.wide_hits, info.big_err,
               (double)info.jitter_ns, info.vchecks, info.vslips, info.vmisses,
               info.skipped, info.substituted, info.given_up,
               info.windows, info.error ? info.error : "none");
        if (rows == GRAB_H && !info.error) {
            metrics_t m = measure();
            printf("   marker=%d diag=%.0f noise=%.1f edge=%.2fpx\n",
                   m.marker_row, m.diag_col, m.noise, m.edge_px);
            if (marker_first < 0) { marker_first = m.marker_row; diag_first = m.diag_col; }
            else {
                if (m.marker_row != marker_first) marker_moves++;
                if (fabs(m.diag_col - diag_first) > diag_max_dev) diag_max_dev = fabs(m.diag_col - diag_first);
            }
            noise_sum += m.noise;
            edge_sum += m.edge_px;
            measured++;
        }
        if (rows == GRAB_H) complete++;
        if (rows > 0 && !info.error) good++;    /* complete, or a few rows given up */
    }
    double wall = host_now_us() - t0;
    printf("summary: %s good %d/%d (complete %d), marker moved in %d, diagonal drift %.0f px, "
           "noise %.2f, edge %.2f px, cb_violation=%d (%.0f ms wall)\n",
           g_pal ? "PAL" : "NTSC", good, frames, complete, marker_moves, diag_max_dev,
           measured ? noise_sum / measured : 0.0, measured ? edge_sum / measured : 0.0,
           g_cb_violation ? 1 : 0, wall / 1000.0);

    int ok = good == frames && !g_cb_violation && measured >= frames / 2 &&
             marker_first >= 82 && marker_first <= 86 && marker_moves == 0 &&
             diag_max_dev <= 2.0;
    printf("%s\n", ok ? "PASS" : "FAIL");
    return ok ? 0 : 1;
}
