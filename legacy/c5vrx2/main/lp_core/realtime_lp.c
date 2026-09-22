#include <stdbool.h>
#include <stdint.h>
#include "riscv/rvruntime-frames.h"
#include "ulp_lp_core_cpu_freq_shared.h"

#define REG32(a) (*(volatile uint32_t *)(uintptr_t)(a))
#define DUMP_CTRL        0x600a9004u
#define DUMP_PTR         0x600a9008u
#define PAU_REGDMA_CONF  0x60093000u
#define PAU_CURRENT_LINK 0x6009300cu
#define PAU_PERI_ADDR    0x60093010u
#define PAU_MEM_ADDR     0x60093014u
#define PAU_INT_RAW      0x6009301cu
#define PAU_INT_CLR      0x60093020u
#define HP_SRAM_USAGE    0x60095004u
#define PARLIO_TX_CLOCK  0x600960b4u

#define CTRL_ENABLE      0x80000000u
#define CTRL_START       0x00080000u
#define CTRL_DONE        0x00040000u
#define PTR_MASK         0x00003fffu
#define PARLIO_CLK_EN    0x00040000u

#define PAU_START        0x00000008u
#define PAU_TO_MEM       0x00000010u
#define PAU_LINK_SEL_M   0x000001e0u
#define PAU_LINK3        0x00000060u
#define PAU_DONE_RAW     0x00000001u
#define PAU_ERROR_RAW    0x00000002u
#define PAU_TIMEOUT_CYCLES 8192u

#define REGDMA_WRITE_HEAD     0x40020000u
#define REGDMA_WRITE_HEAD_EOF 0xc0020000u

#define CMD_NONE       0u
#define CMD_CONTINUOUS 1u
#define CMD_STOP       2u

#define STATE_BOOT     0u
#define STATE_READY    1u
#define STATE_RUNNING  2u
#define STATE_ERROR    3u
#define STATE_STOPPED  4u

#define TELEMETRY_MAGIC 0x43355232u /* "C5R2" */
#define WRITER_STALL_US 250000u
#define REARM_ADVANCE_US 250000u

#define FAIL_NONE         0u
#define FAIL_INITIAL_LEAD 1u
#define FAIL_WRITER_STALL 2u
#define FAIL_PAU_CHAIN    3u
#define FAIL_RESTART_RESET 4u

volatile uint32_t c5vrx2_command;
volatile uint32_t c5vrx2_state;
volatile uint32_t c5vrx2_runs;
volatile uint32_t c5vrx2_rearms;
volatile uint32_t c5vrx2_rearm_failures;
volatile uint32_t c5vrx2_blocks;
volatile uint32_t c5vrx2_gap_cycles_last;
volatile uint32_t c5vrx2_gap_cycles_min;
volatile uint32_t c5vrx2_gap_cycles_max;
volatile uint32_t c5vrx2_gap_cycles_total;
volatile uint32_t c5vrx2_fill_cycles_last;
volatile uint32_t c5vrx2_fill_cycles_min;
volatile uint32_t c5vrx2_fill_cycles_max;
volatile uint32_t c5vrx2_fill_cycles_total;
volatile uint32_t c5vrx2_telemetry_magic;
volatile uint32_t c5vrx2_last_pointer;
volatile uint32_t c5vrx2_saved_ownership;
volatile uint32_t c5vrx2_fault_cause;
volatile uint32_t c5vrx2_fault_address;
volatile uint32_t c5vrx2_fault_pc;
volatile uint32_t c5vrx2_stage;
volatile uint32_t c5vrx2_observed_words;
volatile uint32_t c5vrx2_observed_wraps;
volatile uint32_t c5vrx2_observed_cycles;
volatile uint32_t c5vrx2_pointer_changes;
volatile uint32_t c5vrx2_done_seen;
volatile uint32_t c5vrx2_fail_reason;
volatile uint32_t c5vrx2_fail_control;
volatile uint32_t c5vrx2_fail_pointer_mode;
volatile uint32_t c5vrx2_start_control;
volatile uint32_t c5vrx2_regdma_link_root;
volatile uint32_t c5vrx2_regdma_nodes[4][7];
volatile uint32_t c5vrx2_regdma_conf;
volatile uint32_t c5vrx2_regdma_int_raw;
volatile uint32_t c5vrx2_regdma_current_link;
volatile uint32_t c5vrx2_regdma_peri_addr;
volatile uint32_t c5vrx2_regdma_mem_addr;
volatile uint32_t c5vrx2_regdma_timed_out;
volatile uint32_t c5vrx2_reset_pointer;
volatile uint32_t c5vrx2_reset_cycles;
volatile uint32_t c5vrx2_advance_pointer;
volatile uint32_t c5vrx2_activity_pauses;
volatile uint32_t c5vrx2_activity_resumes;
volatile uint32_t c5vrx2_pause_cycles_last;
volatile uint32_t c5vrx2_pause_cycles_max;
volatile uint32_t c5vrx2_pause_active;
volatile uint32_t c5vrx2_run_cycles;

static inline void fence_io(void)
{
    __asm__ __volatile__("fence iorw, iorw" ::: "memory");
}

static inline uint32_t cycles(void)
{
    uint32_t v;
    __asm__ __volatile__("csrr %0, mcycle" : "=r"(v));
    return v;
}

static inline uint32_t cycles_for_us(uint32_t us)
{
    return us * LP_CORE_CYCLES_PER_US_NUM / LP_CORE_CYCLES_PER_US_DENOM;
}

static inline uint32_t pointer_mode(void)
{
    return REG32(DUMP_PTR);
}

static inline uint32_t pointer(void)
{
    return pointer_mode() & PTR_MASK;
}

/* The four masked WRITE nodes are the physically proven C5 donor chain.  They
 * live in LP SRAM because HP SRAM is lent to the RF dump writer while the
 * chain executes.  Hardware entry addresses point at word 2 (the node head). */
static void prepare_regdma_chain(void)
{
    for (uint32_t i = 0u; i < 4u; ++i) {
        for (uint32_t j = 0u; j < 7u; ++j)
            c5vrx2_regdma_nodes[i][j] = 0u;
        c5vrx2_regdma_nodes[i][2] = i == 3u ?
            REGDMA_WRITE_HEAD_EOF : REGDMA_WRITE_HEAD;
        c5vrx2_regdma_nodes[i][3] = i == 3u ? 0u :
            (uint32_t)(uintptr_t)&c5vrx2_regdma_nodes[i + 1u][2];
        c5vrx2_regdma_nodes[i][4] = DUMP_CTRL;
    }
    c5vrx2_regdma_nodes[0][5] = 0u;
    c5vrx2_regdma_nodes[0][6] = CTRL_ENABLE;
    c5vrx2_regdma_nodes[1][5] = CTRL_ENABLE;
    c5vrx2_regdma_nodes[1][6] = CTRL_ENABLE;
    c5vrx2_regdma_nodes[2][5] = CTRL_START;
    c5vrx2_regdma_nodes[2][6] = CTRL_START;
    c5vrx2_regdma_nodes[3][5] = 0u;
    c5vrx2_regdma_nodes[3][6] = CTRL_START;
    c5vrx2_regdma_link_root =
        (uint32_t)(uintptr_t)&c5vrx2_regdma_nodes[0][2];
    fence_io();
}

/* Startup is intentionally direct.  Every realtime boundary below uses PAU
 * REGDMA, keeping the 48 MHz LP core out of the four timing-sensitive writes. */
static inline void start_writer_once(void)
{
    uint32_t c = REG32(DUMP_CTRL) &
                 ~(CTRL_ENABLE | CTRL_START | CTRL_DONE);
    REG32(DUMP_CTRL) = c;
    fence_io();
    c |= CTRL_ENABLE;
    REG32(DUMP_CTRL) = c;
    REG32(DUMP_CTRL) = c | CTRL_START;
    REG32(DUMP_CTRL) = c;
    fence_io();
    c5vrx2_start_control = REG32(DUMP_CTRL);
}

/* The PAU chain proved reliable, but its roughly 600-cycle restart latency is
 * long enough for the fixed-rate consumer to eat into its ring lead. Issue the
 * exact same four clean control images directly from the already parked LP
 * core. DONE must never be copied back into the control register. */
static inline void rearm_writer_direct(void)
{
    uint32_t c = REG32(DUMP_CTRL) &
                 ~(CTRL_ENABLE | CTRL_START | CTRL_DONE);
    REG32(DUMP_CTRL) = c;
    fence_io();
    c |= CTRL_ENABLE;
    REG32(DUMP_CTRL) = c;
    REG32(DUMP_CTRL) = c | CTRL_START;
    REG32(DUMP_CTRL) = c;
    fence_io();
}

static inline bool rearm_regdma_now(void)
{
    REG32(PAU_INT_CLR) = PAU_DONE_RAW | PAU_ERROR_RAW;
    uint32_t conf = REG32(PAU_REGDMA_CONF);
    conf &= ~(PAU_START | PAU_TO_MEM | PAU_LINK_SEL_M);
    conf |= PAU_LINK3;
    REG32(PAU_REGDMA_CONF) = conf;
    fence_io();
    REG32(PAU_REGDMA_CONF) = conf | PAU_START;
    fence_io();

    const uint32_t started = cycles();
    uint32_t raw;
    do {
        raw = REG32(PAU_INT_RAW);
        if ((raw & PAU_ERROR_RAW) != 0u) break;
    } while ((raw & PAU_DONE_RAW) == 0u &&
             (uint32_t)(cycles() - started) < PAU_TIMEOUT_CYCLES);

    c5vrx2_regdma_conf = REG32(PAU_REGDMA_CONF);
    c5vrx2_regdma_int_raw = raw;
    c5vrx2_regdma_current_link = REG32(PAU_CURRENT_LINK);
    c5vrx2_regdma_peri_addr = REG32(PAU_PERI_ADDR);
    c5vrx2_regdma_mem_addr = REG32(PAU_MEM_ADDR);
    c5vrx2_regdma_timed_out =
        (raw & (PAU_DONE_RAW | PAU_ERROR_RAW)) == 0u;

    REG32(PAU_REGDMA_CONF) = conf & ~PAU_LINK_SEL_M;
    if ((raw & PAU_ERROR_RAW) == 0u) REG32(PAU_INT_CLR) = PAU_DONE_RAW;
    fence_io();
    return (raw & PAU_DONE_RAW) != 0u &&
           (raw & PAU_ERROR_RAW) == 0u;
}

void __attribute__((noreturn)) ulp_lp_core_panic_handler(RvExcFrame *frame,
                                                         int cause)
{
    c5vrx2_fault_cause = (uint32_t)cause;
    c5vrx2_fault_address = (uint32_t)frame->mtval;
    c5vrx2_fault_pc = (uint32_t)frame->mepc;
    REG32(PARLIO_TX_CLOCK) &= ~PARLIO_CLK_EN;
    REG32(DUMP_CTRL) &= ~CTRL_ENABLE;
    fence_io();
    if (c5vrx2_stage >= 2u) {
        REG32(HP_SRAM_USAGE) = c5vrx2_saved_ownership;
        fence_io();
    }
    c5vrx2_state = STATE_ERROR;
    c5vrx2_command = CMD_NONE;
    for (;;) __asm__ __volatile__("nop");
}

static void run_continuous(void)
{
    uint32_t final_state = STATE_ERROR;
    c5vrx2_runs++;
    c5vrx2_rearms = 0u;
    c5vrx2_rearm_failures = 0u;
    c5vrx2_blocks = 0u;
    c5vrx2_gap_cycles_last = 0u;
    c5vrx2_gap_cycles_min = UINT32_MAX;
    c5vrx2_gap_cycles_max = 0u;
    c5vrx2_gap_cycles_total = 0u;
    c5vrx2_fill_cycles_last = 0u;
    c5vrx2_fill_cycles_min = UINT32_MAX;
    c5vrx2_fill_cycles_max = 0u;
    c5vrx2_fill_cycles_total = 0u;
    c5vrx2_telemetry_magic = TELEMETRY_MAGIC;
    c5vrx2_observed_words = 0u;
    c5vrx2_observed_wraps = 0u;
    c5vrx2_observed_cycles = 0u;
    c5vrx2_pointer_changes = 0u;
    c5vrx2_done_seen = 0u;
    c5vrx2_fail_reason = FAIL_NONE;
    c5vrx2_fail_control = 0u;
    c5vrx2_fail_pointer_mode = 0u;
    c5vrx2_start_control = 0u;
    c5vrx2_regdma_conf = 0u;
    c5vrx2_regdma_int_raw = 0u;
    c5vrx2_regdma_current_link = 0u;
    c5vrx2_regdma_peri_addr = 0u;
    c5vrx2_regdma_mem_addr = 0u;
    c5vrx2_regdma_timed_out = 0u;
    c5vrx2_reset_pointer = PTR_MASK;
    c5vrx2_reset_cycles = 0u;
    c5vrx2_advance_pointer = PTR_MASK;
    c5vrx2_activity_pauses = 0u;
    c5vrx2_activity_resumes = 0u;
    c5vrx2_pause_cycles_last = 0u;
    c5vrx2_pause_cycles_max = 0u;
    c5vrx2_pause_active = 0u;
    c5vrx2_run_cycles = 0u;
    c5vrx2_stage = 1u;

    c5vrx2_saved_ownership = REG32(HP_SRAM_USAGE);
    REG32(HP_SRAM_USAGE) =
        (c5vrx2_saved_ownership & 0xfffef0ffu) | 0x00010200u;
    fence_io();
    c5vrx2_stage = 2u;

    start_writer_once();
    c5vrx2_stage = 3u;

    uint32_t previous = pointer();
    uint32_t lead = 0u;
    const uint32_t lead_start = cycles();
    while (lead < 8192u) {
        const uint32_t current = pointer();
        if (current > previous) {
            const uint32_t delta = current - previous;
            lead += delta;
            c5vrx2_observed_words += delta;
            c5vrx2_pointer_changes++;
            previous = current;
        }
        if ((uint32_t)(cycles() - lead_start) > cycles_for_us(WRITER_STALL_US)) {
            c5vrx2_fail_reason = FAIL_INITIAL_LEAD;
            goto fail;
        }
    }

    /* HP prepared a cyclic RF SRAM -> BitScrambler -> PARLIO transaction and
     * left it backpressured. Start its 20 MHz clock only after half a capture
     * block is real, giving the consumer its initial producer lead. */
    REG32(PARLIO_TX_CLOCK) |= PARLIO_CLK_EN;
    fence_io();
    c5vrx2_state = STATE_RUNNING;
    c5vrx2_stage = 4u;

    const uint32_t run_started = cycles();
    uint32_t fill_started = lead_start;
    uint32_t last_progress = run_started;
    uint32_t pause_started = 0u;
    bool paused = false;
    for (;;) {
        if (c5vrx2_command == CMD_STOP) goto stopped;
        const uint32_t now = cycles();
        const uint32_t current = pointer();
        const uint32_t control = REG32(DUMP_CTRL);

        c5vrx2_run_cycles = now - run_started;

        if ((control & CTRL_DONE) != 0u) c5vrx2_done_seen = 1u;
        if (current > previous) {
            const uint32_t delta = current - previous;
            c5vrx2_observed_words += delta;
            c5vrx2_pointer_changes++;
            c5vrx2_last_pointer = current;
            previous = current;
            last_progress = now;
            if (paused) {
                const uint32_t pause_cycles = now - pause_started;
                c5vrx2_pause_cycles_last = pause_cycles;
                if (pause_cycles > c5vrx2_pause_cycles_max)
                    c5vrx2_pause_cycles_max = pause_cycles;
                c5vrx2_activity_resumes++;
                c5vrx2_pause_active = 0u;
                paused = false;
            }
        }

        if ((control & CTRL_DONE) != 0u && current == PTR_MASK) {
            const uint32_t completed_at = now;

            /* HOT PATH: the parked LP core performs the same four modem writes
             * as the reliable REGDMA chain without PAU setup/poll latency.
             * PARLIO must keep its fixed 20 MHz timebase throughout: stopping
             * it here stretches CVBS lines and destroys H-sync/chroma lock. */
            rearm_writer_direct();

            const uint32_t fill = completed_at - fill_started;
            c5vrx2_fill_cycles_last = fill;
            c5vrx2_fill_cycles_total += fill;
            if (fill < c5vrx2_fill_cycles_min)
                c5vrx2_fill_cycles_min = fill;
            if (fill > c5vrx2_fill_cycles_max)
                c5vrx2_fill_cycles_max = fill;
            c5vrx2_blocks++;

            /* A departure from 16383 alone is only controller reset.  Require
             * a subsequent distinct pointer value before declaring that the
             * RF producer really wrote samples in the next generation. */
            uint32_t reset_pointer = PTR_MASK;
            uint32_t reset_at = 0u;
            for (;;) {
                const uint32_t restarted = pointer();
                if (reset_pointer == PTR_MASK && restarted != PTR_MASK) {
                    reset_pointer = restarted;
                    reset_at = cycles();
                    c5vrx2_reset_pointer = reset_pointer;
                    c5vrx2_reset_cycles = reset_at - completed_at;
                } else if (reset_pointer != PTR_MASK &&
                           restarted != reset_pointer) {
                    const uint32_t restarted_at = cycles();
                    if (paused) {
                        const uint32_t pause_cycles =
                            restarted_at - pause_started;
                        c5vrx2_pause_cycles_last = pause_cycles;
                        if (pause_cycles > c5vrx2_pause_cycles_max)
                            c5vrx2_pause_cycles_max = pause_cycles;
                        c5vrx2_activity_resumes++;
                        c5vrx2_pause_active = 0u;
                        paused = false;
                    }
                    const uint32_t gap = restarted_at - completed_at;
                    c5vrx2_gap_cycles_last = gap;
                    c5vrx2_gap_cycles_total += gap;
                    if (gap < c5vrx2_gap_cycles_min)
                        c5vrx2_gap_cycles_min = gap;
                    if (gap > c5vrx2_gap_cycles_max)
                        c5vrx2_gap_cycles_max = gap;
                    c5vrx2_rearms++;
                    c5vrx2_advance_pointer = restarted;
                    c5vrx2_last_pointer = restarted;
                    c5vrx2_observed_words +=
                        (restarted - reset_pointer) & PTR_MASK;
                    c5vrx2_pointer_changes++;
                    previous = restarted;
                    fill_started = restarted_at;
                    last_progress = restarted_at;
                    break;
                }
                const uint32_t waited = cycles() - completed_at;
                if (reset_pointer == PTR_MASK &&
                    waited > cycles_for_us(REARM_ADVANCE_US)) {
                    /* PAU completed but the dump controller itself never
                     * reset: this is a genuine rearm failure. */
                    c5vrx2_fail_reason = FAIL_RESTART_RESET;
                    goto fail;
                }
                if (reset_pointer != PTR_MASK && !paused &&
                    waited > cycles_for_us(WRITER_STALL_US)) {
                    /* Mode 0 can remain armed at its reset pointer while RF
                     * activity is absent.  That is not a failed rearm: keep
                     * waiting so a later VTX burst resumes this generation
                     * without another software trigger. */
                    paused = true;
                    pause_started = reset_at;
                    c5vrx2_activity_pauses++;
                    c5vrx2_pause_active = 1u;
                }
            }

            /* `now` was sampled before the synchronous REGDMA rearm.  Do not
             * feed that stale timestamp into the inactivity detector below:
             * restarted_at is newer and unsigned subtraction would otherwise
             * manufacture one pause/resume pair at every block boundary. */
            continue;
        }

        /* Mode 0 is RF-activity dependent. Keep the writer armed for the full
         * diagnostic window and distinguish an inactive interval from a
         * failed REGDMA rearm. A later pointer change proves activity resumed
         * in the same generation without another software trigger. */
        if (!paused &&
            (uint32_t)(now - last_progress) > cycles_for_us(WRITER_STALL_US)) {
            paused = true;
            pause_started = last_progress;
            c5vrx2_activity_pauses++;
            c5vrx2_pause_active = 1u;
        }
    }

fail:
    c5vrx2_rearm_failures++;
    c5vrx2_fail_control = REG32(DUMP_CTRL);
    c5vrx2_fail_pointer_mode = pointer_mode();
    final_state = STATE_ERROR;
    goto restore;
stopped:
    if (paused) {
        const uint32_t pause_cycles = cycles() - pause_started;
        c5vrx2_pause_cycles_last = pause_cycles;
        if (pause_cycles > c5vrx2_pause_cycles_max)
            c5vrx2_pause_cycles_max = pause_cycles;
    }
    final_state = STATE_STOPPED;
restore:
    REG32(PARLIO_TX_CLOCK) &= ~PARLIO_CLK_EN;
    REG32(DUMP_CTRL) &= ~CTRL_ENABLE;
    fence_io();
    REG32(HP_SRAM_USAGE) = c5vrx2_saved_ownership;
    fence_io();
    c5vrx2_stage = 0u;
    c5vrx2_command = CMD_NONE;
    fence_io();
    c5vrx2_state = final_state;
}

int main(void)
{
    c5vrx2_fault_cause = 0u;
    c5vrx2_fault_address = 0u;
    c5vrx2_fault_pc = 0u;
    c5vrx2_stage = 0u;
    prepare_regdma_chain();
    c5vrx2_command = CMD_NONE;
    c5vrx2_state = STATE_READY;
    for (;;) {
        if (c5vrx2_command == CMD_CONTINUOUS) {
            c5vrx2_command = CMD_NONE;
            run_continuous();
        }
        __asm__ __volatile__("nop");
    }
}
