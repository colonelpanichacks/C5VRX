#include "continuous_iq.h"

#include <string.h>

#include "esp_attr.h"
#include "esp_cpu.h"
#include "esp_timer.h"

#include "wifi5.h"
#include "startup_trace.h"

#define REG32(a) (*(volatile uint32_t *)(uintptr_t)(a))

#define DUMP_CTRL       0x600a9004u
#define DUMP_PTR_MODE   0x600a9008u
#define HP_SRAM_USAGE   0x60095004u

#define CTRL_ENABLE     0x80000000u
#define CTRL_START      0x00080000u
#define CTRL_DONE       0x00040000u
#define CTRL_DUMP_FIRST 0x00020000u
#define PTR_MASK        0x00003fffu
#define SELECTOR_MASK   0x01fe0000u
#define TX_START_SELECT 0x00060000u

#define READER_GUARD_WORDS 256u
#if CONFIG_C5VRX2_MODE_MODEM_PARLIO || CONFIG_C5VRX2_MODE_MODEM_WBFM || \
    CONFIG_C5VRX2_MODE_LIVE
/* The PARLIO paths run while the autonomous dump/diagnostic source is active.
 * Keep the one-time cadence measurement comfortably below the observed ~5 ms
 * CPU-lockup boundary. LIVE used to fall through to 4 ms here and could lock
 * before start_rx_ring() or the USB/snapshot tasks ever ran. Half a
 * millisecond still observes about 40 RF-ring wraps. */
#define RATE_MEASURE_US     500u
#else
#define RATE_MEASURE_US    4000u
#endif
#define OBSERVER_PERIOD_US 50u
#define CPU_CYCLES_PER_US  CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ

typedef struct {
    bool running;
    bool span_out;
    bool next_span_continuous;
    uint32_t saved_sram_usage;
    uint32_t last_pointer;
    uint32_t physical_wraps;
    uint32_t overruns;
    uint32_t ambiguous_wraps;
    uint32_t discontinuity_epoch;
    uint32_t producer_start_count;
    uint32_t trigger_count;
    uint32_t rf_sample_rate_hz;
    bool done_latched;
    esp_timer_handle_t observer_timer;
    int64_t last_poll_us;
    uint64_t producer_words;
    uint64_t consumer_words;
    iq_span_t outstanding;
} continuous_iq_state_t;

static continuous_iq_state_t s_iq;

#define DEBUG_STAGE_MAGIC 0xc51a6e2du
typedef struct {
    uint32_t magic;
    uint32_t stage;
    uint32_t inverse_stage;
} debug_stage_record_t;

static RTC_NOINIT_ATTR volatile debug_stage_record_t s_debug_stage;

void IRAM_ATTR continuous_iq_debug_mark(uint32_t stage)
{
    s_debug_stage.stage = stage;
    s_debug_stage.inverse_stage = ~stage;
    s_debug_stage.magic = DEBUG_STAGE_MAGIC;
    __asm__ __volatile__("fence iorw, iorw" ::: "memory");
}

uint32_t continuous_iq_debug_last_stage(void)
{
    return s_debug_stage.magic == DEBUG_STAGE_MAGIC &&
           s_debug_stage.inverse_stage == ~s_debug_stage.stage ?
               s_debug_stage.stage : 0u;
}

static inline IRAM_ATTR void fence_io(void)
{
    __asm__ __volatile__("fence iorw, iorw" ::: "memory");
}

static inline IRAM_ATTR uint32_t writer_pointer(void)
{
    return REG32(DUMP_PTR_MODE) & PTR_MASK;
}

static void note_discontinuity(void)
{
    s_iq.discontinuity_epoch++;
    s_iq.next_span_continuous = false;
}

static IRAM_ATTR void observe_control(uint32_t control)
{
    const bool done = (control & CTRL_DONE) != 0u;
    if (done && !s_iq.done_latched) s_iq.trigger_count++;
    s_iq.done_latched = done;
}

/* This tracker is intentionally for a hot polling consumer.  A slow caller
 * cannot distinguish identical pointer values separated by one or more full
 * physical wraps, so elapsed time is used to reject that ambiguity rather
 * than fabricating continuity. */
static bool update_producer(void)
{
    const int64_t now = esp_timer_get_time();
    const uint32_t current = writer_pointer();
    const uint32_t delta = (current - s_iq.last_pointer) & PTR_MASK;
    const int64_t elapsed = now - s_iq.last_poll_us;

    if (s_iq.rf_sample_rate_hz != 0u && elapsed > 0) {
        const uint64_t expected =
            (uint64_t)s_iq.rf_sample_rate_hz * (uint64_t)elapsed / 1000000u;
        if (expected >= C5VRX2_RF_WORDS) {
            s_iq.ambiguous_wraps++;
            s_iq.last_pointer = current;
            s_iq.last_poll_us = now;
            note_discontinuity();
            return false;
        }
    }

    if (current < s_iq.last_pointer && delta != 0u)
        s_iq.physical_wraps++;
    s_iq.producer_words += delta;
    s_iq.last_pointer = current;
    s_iq.last_poll_us = now;
    return true;
}

#if !CONFIG_C5VRX2_MODE_MODEM_CAPTURE && !CONFIG_C5VRX2_MODE_MODEM_PARLIO && \
    !CONFIG_C5VRX2_MODE_MODEM_WBFM && !CONFIG_C5VRX2_MODE_LIVE
static void observe_producer(void *argument)
{
    (void)argument;
    if (!s_iq.running) return;
    (void)update_producer();
    observe_control(REG32(DUMP_CTRL));
}
#endif

static IRAM_ATTR esp_err_t measure_rate(void)
{
    continuous_iq_debug_mark(420u);
    const uint32_t begin = esp_cpu_get_cycle_count();
    continuous_iq_debug_mark(421u);
    uint32_t now = begin;
    uint32_t last = writer_pointer();
    continuous_iq_debug_mark(422u);
    uint32_t words = 0u;
    uint32_t wraps = 0u;

    do {
        const uint32_t current = writer_pointer();
        const uint32_t delta = (current - last) & PTR_MASK;
        if (current < last && delta != 0u) wraps++;
        words += delta;
        last = current;
        if ((REG32(DUMP_CTRL) & CTRL_DONE) != 0u)
            return ESP_ERR_INVALID_STATE;
        now = esp_cpu_get_cycle_count();
    } while ((uint32_t)(now - begin) <
             RATE_MEASURE_US * CPU_CYCLES_PER_US);

    continuous_iq_debug_mark(425u);
    const uint32_t elapsed_cycles = now - begin;
    if (elapsed_cycles == 0u || words < C5VRX2_RF_WORDS)
        return ESP_ERR_TIMEOUT;

    const uint32_t elapsed_us = elapsed_cycles / CPU_CYCLES_PER_US;
    if (elapsed_us == 0u) return ESP_ERR_TIMEOUT;
    /* Keep this calculation 32-bit: the MAC-owned active window must not
     * call a flash-resident compiler helper for 64-bit division. */
    s_iq.rf_sample_rate_hz =
        (words / elapsed_us) * 1000000u +
        ((words % elapsed_us) * 1000000u) / elapsed_us;
    s_iq.last_pointer = last;
    /* No esp_timer call is permitted while the modem owns its dump SRAM.
     * Normal live consumption of this CPU-invisible ring is disabled. */
    s_iq.last_poll_us = 0;
    /* Keep the logical low bits aligned with the physical SRAM address.  The
     * measured word count is a cadence observation, not an absolute ring
     * position. */
    s_iq.producer_words = last;
    s_iq.physical_wraps = wraps;
    s_iq.consumer_words = last;
    return ESP_OK;
}

esp_err_t IRAM_ATTR continuous_iq_start(void)
{
    continuous_iq_debug_mark(401u);
    c5vrx2_trace_stage(101u, ESP_OK);
    if (s_iq.running) return ESP_ERR_INVALID_STATE;
    if (!c5vrx2_rf_dump_memory_reserved()) return ESP_ERR_INVALID_STATE;
    continuous_iq_debug_mark(402u);
    c5vrx2_trace_stage(102u, ESP_OK);

    esp_err_t err = c5vrx2_rf_dump_prepare_mode0();
    if (err != ESP_OK) return err;
    continuous_iq_debug_mark(403u);
    c5vrx2_trace_stage(103u, ESP_OK);

    memset(&s_iq, 0, sizeof(s_iq));
    s_iq.next_span_continuous = false;
    c5vrx2_rf_dump_guards_init();
    continuous_iq_debug_mark(404u);
    c5vrx2_trace_stage(104u, ESP_OK);

    /* C5 trigmode=TX_START (5) selects 0x00060000 in PTR_MODE.  The C5
     * v6.0.1 vendor routine ignores its historical dump_trig argument, so it
     * cannot itself expose a dump-first TX_START combination.  Preserve the
     * verified TX_START selector and explicitly request the historical
     * pre-trigger state with CTRL_DUMP_FIRST; no software START is issued. */
    uint32_t selector = REG32(DUMP_PTR_MODE);
    selector = (selector & ~SELECTOR_MASK) | TX_START_SELECT;
    REG32(DUMP_PTR_MODE) = selector;

    uint32_t control = REG32(DUMP_CTRL);
    control &= ~(CTRL_ENABLE | CTRL_START | CTRL_DONE);
    control |= CTRL_DUMP_FIRST;
    control = (control & ~0x0001ffffu) | C5VRX2_RF_WORDS;
    REG32(DUMP_CTRL) = control;
    continuous_iq_debug_mark(405u);
    c5vrx2_trace_stage(105u, ESP_OK);

    /* Quiesce LMAC while CPU still owns all HP SRAM.  lmac_stop_hw_txq() is
     * private vendor code and may use internal Wi-Fi state in the bank which
     * is about to be granted to the modem dump engine.  Once this returns no
     * software path below can enqueue TX: only register writes remain before
     * dump ENABLE, so TX_START stays an impossible trigger. */
    err = c5vrx2_wifi5_lock_rx_only();
    if (err != ESP_OK) return err;
    continuous_iq_debug_mark(406u);
    c5vrx2_trace_stage(106u, ESP_OK);

    /* Reproduce the vendor wrapper's SRAM grant once.  The linker/heap
     * reservation keeps all HP stacks and objects outside this 64 KiB bank. */
    s_iq.saved_sram_usage = REG32(HP_SRAM_USAGE);
    continuous_iq_debug_mark(407u);
    c5vrx2_trace_stage_detail(107u, ESP_OK, s_iq.saved_sram_usage,
                              control, selector);
#if CONFIG_C5VRX2_MODE_RF_DMA_CPU_OWNED || \
    CONFIG_C5VRX2_MODE_MODEM_PARLIO || \
    CONFIG_C5VRX2_MODE_MODEM_WBFM || CONFIG_C5VRX2_MODE_LIVE
    /* MODEM_DIAG is LIVE's sample transport. Keep HP CPU ownership so USB,
     * interrupts and normal code remain available; the dump engine is used
     * only to keep the verified RF/diagnostic source configured and armed.
     * Both physical windows remain excluded from the heap. */
    REG32(HP_SRAM_USAGE) =
        (s_iq.saved_sram_usage & 0xfffef0ffu) | 0x00010000u;
#else
    REG32(HP_SRAM_USAGE) =
        (s_iq.saved_sram_usage & 0xfffef0ffu) | 0x00010200u;
#endif
    fence_io();
    continuous_iq_debug_mark(408u);

    /* Do not write startup_trace (and therefore SPI flash) while MAC owns
     * the RF dump SRAM. On the XIAO C5 that observability path can stall or
     * fault immediately after HP_SRAM_USAGE changes. Persist diagnostics only
     * after continuous_iq_stop() has restored CPU ownership. */

    /* DUMP FIRST + never-occurring TX_START starts by ENABLE only.  A START
     * pulse here would select the finite software-trigger lifecycle again. */
    REG32(DUMP_CTRL) = control | CTRL_ENABLE;
    fence_io();
    s_iq.running = true;
    s_iq.producer_start_count = 1u;
    s_iq.last_pointer = writer_pointer();
    s_iq.last_poll_us = 0;
    continuous_iq_debug_mark(409u);
    err = measure_rate();
    if (err != ESP_OK) {
        (void)continuous_iq_stop();
        return err;
    }
    continuous_iq_debug_mark(410u);
#if !CONFIG_C5VRX2_MODE_MODEM_CAPTURE && !CONFIG_C5VRX2_MODE_MODEM_PARLIO && \
    !CONFIG_C5VRX2_MODE_MODEM_WBFM && !CONFIG_C5VRX2_MODE_LIVE
    const esp_timer_create_args_t observer_args = {
        .callback = observe_producer,
        .name = "iq_ptr",
    };
    err = esp_timer_create(&observer_args, &s_iq.observer_timer);
    if (err == ESP_OK)
        err = esp_timer_start_periodic(s_iq.observer_timer,
                                       OBSERVER_PERIOD_US);
    if (err != ESP_OK) {
        (void)continuous_iq_stop();
        return err;
    }
#endif
    continuous_iq_debug_mark(411u);
    return ESP_OK;
}

bool continuous_iq_acquire(iq_span_t *span)
{
    if (!span || !s_iq.running || s_iq.span_out) return false;
    if (!update_producer()) return false;

    uint64_t available = s_iq.producer_words - s_iq.consumer_words;
    if (available >= C5VRX2_RF_WORDS - READER_GUARD_WORDS) {
        s_iq.overruns++;
        note_discontinuity();
        s_iq.consumer_words = s_iq.producer_words - C5VRX2_RF_WORDS / 2u;
        available = C5VRX2_RF_WORDS / 2u;
    }
    if (available <= READER_GUARD_WORDS) return false;

    const uint32_t physical =
        (uint32_t)(s_iq.consumer_words & (C5VRX2_RF_WORDS - 1u));
    size_t words = (size_t)(available - READER_GUARD_WORDS);
    const size_t until_wrap = C5VRX2_RF_WORDS - physical;
    if (words > until_wrap) words = until_wrap;

    *span = (iq_span_t) {
        .data = (const volatile uint32_t *)(uintptr_t)
                    (C5VRX2_RF_DUMP_BASE + physical * sizeof(uint32_t)),
        .words = words,
        .absolute_start = s_iq.consumer_words,
        .discontinuity_epoch = s_iq.discontinuity_epoch,
        .continuous_from_previous = s_iq.next_span_continuous,
    };
    s_iq.outstanding = *span;
    s_iq.span_out = true;
    return true;
}

void continuous_iq_release(const iq_span_t *span)
{
    if (!span || !s_iq.span_out) return;
    if (span->absolute_start != s_iq.outstanding.absolute_start ||
        span->words > s_iq.outstanding.words) {
        note_discontinuity();
        s_iq.span_out = false;
        return;
    }
    s_iq.consumer_words += span->words;
    s_iq.next_span_continuous = true;
    s_iq.span_out = false;
}

esp_err_t IRAM_ATTR continuous_iq_stop(void)
{
    if (!s_iq.running) return ESP_ERR_INVALID_STATE;
    /* Stop the producer first. This is an explicit diagnostic/retune stop,
     * never a normal ring wrap. It also freezes the SRAM before ownership is
     * returned and before any observer teardown can block. */
    REG32(DUMP_CTRL) &= ~CTRL_ENABLE;
    fence_io();
    if (s_iq.observer_timer) {
        (void)esp_timer_stop(s_iq.observer_timer);
        (void)esp_timer_delete(s_iq.observer_timer);
        s_iq.observer_timer = NULL;
    }
    REG32(HP_SRAM_USAGE) = s_iq.saved_sram_usage;
    fence_io();
    s_iq.running = false;
    s_iq.span_out = false;
    note_discontinuity();
    return ESP_OK;
}

bool continuous_iq_is_running(void)
{
    return s_iq.running;
}

void IRAM_ATTR continuous_iq_get_stats(continuous_iq_stats_t *stats)
{
    if (!stats) return;
    /* Capture mode intentionally has no 50-us observer timer. Sample DONE
     * here as well so a trigger occurring after rate measurement cannot be
     * reported as a false zero in the persisted diagnostic. */
    const uint32_t control = REG32(DUMP_CTRL);
    if (s_iq.running) observe_control(control);
    *stats = (continuous_iq_stats_t) {
        .rf_sample_rate_hz = s_iq.rf_sample_rate_hz,
        .writer_pointer = writer_pointer(),
        .dump_control = control,
        .producer_words = s_iq.producer_words,
        .consumer_words = s_iq.consumer_words,
        .physical_wraps = s_iq.physical_wraps,
        .overruns = s_iq.overruns,
        .ambiguous_wraps = s_iq.ambiguous_wraps,
        .discontinuity_epoch = s_iq.discontinuity_epoch,
        .producer_start_count = s_iq.producer_start_count,
        .rearm_count = 0u,
        .trigger_count = s_iq.trigger_count,
    };
}

uint32_t continuous_iq_sample_rate_hz(void)
{
    return s_iq.rf_sample_rate_hz;
}

const void *continuous_iq_ring_base(void)
{
    return (const void *)(uintptr_t)C5VRX2_RF_DUMP_BASE;
}

size_t continuous_iq_ring_bytes(void)
{
    return C5VRX2_RF_WORDS * sizeof(uint32_t);
}

esp_err_t continuous_iq_wait_base_lead(uint32_t lead_words,
                                       uint32_t tolerance_words,
                                       uint32_t timeout_us)
{
    if (!s_iq.running || lead_words >= C5VRX2_RF_WORDS ||
        tolerance_words >= C5VRX2_RF_WORDS)
        return ESP_ERR_INVALID_ARG;
    const int64_t begin = esp_timer_get_time();
    do {
        const uint32_t pointer = writer_pointer();
        const uint32_t distance =
            (pointer - lead_words) & (C5VRX2_RF_WORDS - 1u);
        if (distance <= tolerance_words) return ESP_OK;
        if ((REG32(DUMP_CTRL) & (CTRL_ENABLE | CTRL_DONE)) != CTRL_ENABLE)
            return ESP_ERR_INVALID_STATE;
    } while ((uint32_t)(esp_timer_get_time() - begin) < timeout_us);
    return ESP_ERR_TIMEOUT;
}
