#include "rf_dump.h"

#include <stdint.h>
#include "esp_heap_caps.h"
#include "esp_memory_utils.h"
#include "heap_memory_layout.h"

#define REG32(a) (*(volatile uint32_t *)(uintptr_t)(a))

#define DUMP_CTRL      0x600a9004u
#define DUMP_PTR_MODE  0x600a9008u
#define DUMP_FORMAT    0x600a9018u
#define FE_PATH        0x600a20b4u
#define FE_ENABLE      0x600a0800u
#define SOURCE_CTRL    0x600a08ccu
#define SOURCE_MUX     0x600a70b8u
#define MODEM_CLOCK    0x600a9c04u
#define CTRL_ENABLE    0x80000000u
#define CTRL_START     0x00080000u
#define CTRL_DONE      0x00040000u
#define CTRL_DUMP_FIRST 0x00020000u
#define HP_SRAM_USAGE  0x60095004u
#define HP_DUMP_FIELDS 0x00010f00u

#define PRE_GUARD_ADDR  0x4082ffc0u
#define POST_GUARD_ADDR 0x40850000u
#define POST_GUARD_END  0x40850040u
#define GUARD_WORDS     16u
#define GUARD_SEED      0xc5a55a2cu

/* MAC_DUMP_ALLOC hands the MAC both 64 KiB SRAM banks beginning at
 * 0x40830000, even though ordinary IQ words occupy only the first bank.
 * Reserving merely through 0x40840000 lets Wi-Fi/USB/task allocations land in
 * the second bank and causes an immediate CPU lockup at the ownership write. */
SOC_RESERVE_MEMORY_REGION(PRE_GUARD_ADDR, POST_GUARD_END, c5vrx2_rf_dump_ram);
extern char _bss_end;

void IRAM_ATTR c5vrx2_rf_dump_boot_sanitize(void)
{
    /* The modem dump block can outlive a CPU-only reset. Leaving ENABLE or
     * MAC ownership set makes the next esp_wifi_init() touch an HP SRAM view
     * which is still disconnected from the CPU, ending in a silent lockup.
     * This is reset recovery only: it is never used at a normal ring wrap. */
    REG32(DUMP_CTRL) &=
        ~(CTRL_ENABLE | CTRL_START | CTRL_DONE | CTRL_DUMP_FIRST);
    __asm__ __volatile__("fence iorw, iorw" ::: "memory");

    /* SRAM_USAGE=0 is the documented CPU HP-memory view. Clear the adjacent
     * MAC_DUMP_ALLOC selector as well; continuous_iq_start() will establish
     * the verified live mapping exactly once after Wi-Fi/PHY is healthy. */
    REG32(HP_SRAM_USAGE) &= ~HP_DUMP_FIELDS;
    __asm__ __volatile__("fence iorw, iorw" ::: "memory");
}

static uint32_t guard_value(unsigned index)
{
    return GUARD_SEED ^ (0x9e3779b9u * (index + 1u));
}

void c5vrx2_rf_dump_guards_init(void)
{
    volatile uint32_t *pre = (volatile uint32_t *)(uintptr_t)PRE_GUARD_ADDR;
    volatile uint32_t *post = (volatile uint32_t *)(uintptr_t)POST_GUARD_ADDR;
    for (unsigned i = 0; i < GUARD_WORDS; ++i) {
        pre[i] = guard_value(i);
        post[i] = guard_value(i + GUARD_WORDS);
    }
    __asm__ __volatile__("fence iorw, iorw" ::: "memory");
}

bool c5vrx2_rf_dump_guards_valid(void)
{
    const volatile uint32_t *pre =
        (const volatile uint32_t *)(uintptr_t)PRE_GUARD_ADDR;
    const volatile uint32_t *post =
        (const volatile uint32_t *)(uintptr_t)POST_GUARD_ADDR;
    for (unsigned i = 0; i < GUARD_WORDS; ++i) {
        if (pre[i] != guard_value(i) ||
            post[i] != guard_value(i + GUARD_WORDS))
            return false;
    }
    return true;
}

bool c5vrx2_rf_dump_memory_reserved(void)
{
    const volatile soc_reserved_region_t *r = &reserved_region_c5vrx2_rf_dump_ram;
    return r->start == PRE_GUARD_ADDR && r->end == POST_GUARD_END &&
           (uintptr_t)&_bss_end <= PRE_GUARD_ADDR &&
           esp_ptr_internal((const void *)(uintptr_t)C5VRX2_RF_DUMP_BASE) &&
           esp_ptr_dma_capable((const void *)(uintptr_t)C5VRX2_RF_DUMP_BASE) &&
           !heap_caps_check_integrity_addr(C5VRX2_RF_DUMP_BASE, false);
}

static void set_format_mode0(void)
{
    uint32_t v = REG32(DUMP_FORMAT);
    v = (v & 0xff03ffffu) | 0x006c0000u;
    REG32(DUMP_FORMAT) = v;
    v = REG32(DUMP_FORMAT);
    v = (v & 0xfffc0fffu) | 0x0001a000u;
    REG32(DUMP_FORMAT) = v;
    v = REG32(DUMP_FORMAT);
    v = (v & 0xfffff03fu) | 0x00000640u;
    REG32(DUMP_FORMAT) = v;
    v = REG32(DUMP_FORMAT);
    v = (v & 0xffffffc0u) | 0x18u;
    REG32(DUMP_FORMAT) = v | 0x01000000u;
}

esp_err_t c5vrx2_rf_dump_prepare_mode0(void)
{
    if (!c5vrx2_rf_dump_memory_reserved()) return ESP_ERR_INVALID_STATE;
    if (REG32(DUMP_CTRL) & CTRL_ENABLE) return ESP_ERR_INVALID_STATE;

    /* Exact ordinary-RX source/format subset recovered from the pinned C5
     * RF-test path. Ownership and ENABLE are intentionally separate so the
     * continuous driver can reproduce the vendor-oracle state atomically. */
    REG32(SOURCE_CTRL) &= 0xff87ffffu;
    REG32(SOURCE_MUX) = (REG32(SOURCE_MUX) & 0xfffffff8u) | 1u;
    REG32(MODEM_CLOCK) = UINT32_MAX;
    REG32(FE_ENABLE) |= 4u;

    uint32_t control = REG32(DUMP_CTRL);
    control = (control & ~0x0001ffffu) | C5VRX2_RF_WORDS;
    control &= ~0x00320000u; /* mode bit17 + recovered control bits20..21 */
    REG32(DUMP_CTRL) = control;
    set_format_mode0();

    REG32(FE_PATH) &= ~1u;
    REG32(DUMP_PTR_MODE) |= 0x01e00000u;
    __asm__ __volatile__("fence iorw, iorw" ::: "memory");
    return ESP_OK;
}
