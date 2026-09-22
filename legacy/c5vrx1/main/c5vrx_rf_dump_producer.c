/* SPDX-License-Identifier: GPL-3.0-only */

#include "c5vrx_rf_dump_producer.h"

#include "sdkconfig.h"
#include "esp_idf_version.h"
#include "esp_heap_caps.h"
#include "esp_memory_utils.h"
#include "heap_memory_layout.h"

#define REG32(address) (*(volatile uint32_t *)(uintptr_t)(address))

#define DUMP_CTRL       0x600a9004u
#define DUMP_PTR_MODE   0x600a9008u
#define DUMP_FORMAT     0x600a9018u
#define FE_PATH         0x600a20b4u
#define FE_AUX          0x600a20acu
#define FE_ENABLE       0x600a0800u
#define SOURCE_CTRL     0x600a08ccu
#define SOURCE_MUX      0x600a70b8u
#define HP_SRAM_USAGE   0x60095004u
#define MODEM_CLOCK     0x600a9c04u

#define CTRL_LENGTH_MASK 0x0001ffffu
#define CTRL_MODE_BIT    0x00020000u
#define CTRL_STATUS_BIT  0x00040000u
#define CTRL_SW_TRIGGER_BIT 0x00080000u
#define CTRL_ENABLE_BIT  0x80000000u
#define MODE_SELECT_MASK 0x01fe0000u

extern void phy_pbus_clear_reg(void);

static bool s_configured;
static bool s_running;
static c5vrx_rf_dump_mode_t s_mode;
static bool s_last_restore_ok = true;
static bool s_last_canaries_ok = true;
static bool s_canaries_armed;
static struct {
    uint32_t dump_ctrl, dump_ptr_mode, dump_format;
    uint32_t fe_path, fe_aux, fe_enable, source_ctrl, source_mux;
    uint32_t hp_sram_usage, modem_clock;
} s_saved;

/* The RF-test writer does not target an abstract peripheral FIFO: it writes
 * this fixed 64 KiB HP-SRAM window. Reserve it before heap initialization so
 * ordinary allocations can never become silent RF-writer victims. A link-time
 * assertion separately rejects static .bss overlap. */
SOC_RESERVE_MEMORY_REGION(C5VRX_RF_DUMP_PRE_GUARD_ADDR,
                          C5VRX_RF_DUMP_POST_GUARD_ADDR +
                              C5VRX_RF_DUMP_POST_GUARD_BYTES,
                          c5vrx_rf_dump_ram);

/* SOC_RESERVE_MEMORY_REGION() only marks its input section as compiler-used.
 * Application components are linked from archives, so --gc-sections can still
 * discard that otherwise-unreferenced section.  This volatile read gives the
 * reservation record a live relocation and keeps it inside IDF's collected
 * .reserved_memory_address table. */
static bool reservation_record_matches(void)
{
    const volatile soc_reserved_region_t *const region =
        &reserved_region_c5vrx_rf_dump_ram;
    return region->start == C5VRX_RF_DUMP_PRE_GUARD_ADDR &&
           region->end == C5VRX_RF_DUMP_POST_GUARD_ADDR +
                          C5VRX_RF_DUMP_POST_GUARD_BYTES;
}

extern char _bss_end;

bool c5vrx_rf_dump_producer_available(void)
{
#if CONFIG_C5VRX_EXPERIMENTAL_RF_DUMP_PRODUCER && \
    ESP_IDF_VERSION == ESP_IDF_VERSION_VAL(6, 0, 2)
    return c5vrx_rf_dump_memory_reserved();
#else
    return false;
#endif
}

bool c5vrx_rf_dump_memory_reserved(void)
{
    /* The address-capability checks alone do not prove that heap initialization
     * honored the reservation.  Reject captures if either endpoint belongs to
     * a registered heap, preventing RF DMA from corrupting FreeRTOS state. */
    return reservation_record_matches() &&
           (uintptr_t)&_bss_end <= C5VRX_RF_DUMP_PRE_GUARD_ADDR &&
           esp_ptr_internal((const void *)(uintptr_t)C5VRX_RF_DUMP_PRE_GUARD_ADDR) &&
           esp_ptr_dma_capable((const void *)(uintptr_t)0x40830000u) &&
           esp_ptr_internal((const void *)(uintptr_t)0x40830000u) &&
           esp_ptr_internal((const void *)(uintptr_t)0x4083ffffu) &&
           esp_ptr_internal((const void *)(uintptr_t)
                            (C5VRX_RF_DUMP_POST_GUARD_ADDR +
                             C5VRX_RF_DUMP_POST_GUARD_BYTES - 1u)) &&
           !heap_caps_check_integrity_addr(C5VRX_RF_DUMP_PRE_GUARD_ADDR, false) &&
           !heap_caps_check_integrity_addr(0x40830000u, false) &&
           !heap_caps_check_integrity_addr(0x4083ffffu, false) &&
           !heap_caps_check_integrity_addr(
               C5VRX_RF_DUMP_POST_GUARD_ADDR +
               C5VRX_RF_DUMP_POST_GUARD_BYTES - 1u, false);
}

static uint32_t canary_value(unsigned index, bool post)
{
    return (post ? 0xa5c50000u : 0x5a3a0000u) ^
           (0x9e3779b9u * (index + 1u));
}

static void seed_canaries(void)
{
    volatile uint32_t *const pre =
        (volatile uint32_t *)(uintptr_t)C5VRX_RF_DUMP_PRE_GUARD_ADDR;
    volatile uint32_t *const post =
        (volatile uint32_t *)(uintptr_t)C5VRX_RF_DUMP_POST_GUARD_ADDR;
    for (unsigned i = 0; i < C5VRX_RF_DUMP_PRE_GUARD_BYTES / 4u; ++i)
        pre[i] = canary_value(i, false);
    for (unsigned i = 0; i < C5VRX_RF_DUMP_POST_GUARD_BYTES / 4u; ++i)
        post[i] = canary_value(i, true);
    __asm__ __volatile__("fence iorw, iorw" ::: "memory");
    s_canaries_armed = true;
    s_last_canaries_ok = true;
}

bool c5vrx_rf_dump_canaries_intact(void)
{
    if (!s_canaries_armed) return false;
    volatile const uint32_t *const pre =
        (volatile const uint32_t *)(uintptr_t)C5VRX_RF_DUMP_PRE_GUARD_ADDR;
    volatile const uint32_t *const post =
        (volatile const uint32_t *)(uintptr_t)C5VRX_RF_DUMP_POST_GUARD_ADDR;
    for (unsigned i = 0; i < C5VRX_RF_DUMP_PRE_GUARD_BYTES / 4u; ++i)
        if (pre[i] != canary_value(i, false)) return false;
    for (unsigned i = 0; i < C5VRX_RF_DUMP_POST_GUARD_BYTES / 4u; ++i)
        if (post[i] != canary_value(i, true)) return false;
    return true;
}

bool c5vrx_rf_dump_last_canaries_ok(void) { return s_last_canaries_ok; }

static c5vrx_rf_dump_registers_t read_registers(void)
{
    return (c5vrx_rf_dump_registers_t) {
        .dump_ctrl = REG32(DUMP_CTRL),
        .dump_ptr_mode = REG32(DUMP_PTR_MODE),
        .dump_format = REG32(DUMP_FORMAT),
        .fe_path = REG32(FE_PATH),
        .fe_aux = REG32(FE_AUX),
        .fe_enable = REG32(FE_ENABLE),
        .source_ctrl = REG32(SOURCE_CTRL),
        .source_mux = REG32(SOURCE_MUX),
        .sram_usage = REG32(HP_SRAM_USAGE),
        .modem_clock = REG32(MODEM_CLOCK),
    };
}

esp_err_t c5vrx_rf_dump_read_registers(c5vrx_rf_dump_registers_t *registers)
{
    if (!registers) return ESP_ERR_INVALID_ARG;
    *registers = read_registers();
    return ESP_OK;
}

bool c5vrx_rf_dump_last_restore_ok(void) { return s_last_restore_ok; }

static void set_format_fields(uint32_t top, uint32_t middle,
                              uint32_t low, uint32_t bottom)
{
    uint32_t value = REG32(DUMP_FORMAT);
    value = (value & 0xff03ffffu) | top;
    REG32(DUMP_FORMAT) = value;
    value = REG32(DUMP_FORMAT);
    value = (value & 0xfffc0fffu) | middle;
    REG32(DUMP_FORMAT) = value;
    value = REG32(DUMP_FORMAT);
    value = (value & 0xfffff03fu) | low;
    REG32(DUMP_FORMAT) = value;
    value = REG32(DUMP_FORMAT);
    value = (value & 0xffffffc0u) | bottom;
    REG32(DUMP_FORMAT) = value;
}

esp_err_t c5vrx_rf_dump_configure(size_t sample_count,
                                  c5vrx_rf_dump_mode_t mode)
{
    if (!c5vrx_rf_dump_producer_available()) return ESP_ERR_NOT_SUPPORTED;
    if (s_configured || s_running) return ESP_ERR_INVALID_STATE;
    if (sample_count == 0 || sample_count > 16384u) return ESP_ERR_INVALID_ARG;
    /* Mode 12 is not just a register variant: the pinned vendor routine calls
     * ble_rx_start(0, 0) after arming it. Reconstructing only its MMIO writes
     * would be incomplete, so fail closed rather than invent BLE setup. */
    if (mode != C5VRX_RF_DUMP_MODE_ORDINARY_RX &&
        mode != C5VRX_RF_DUMP_MODE_11)
        return ESP_ERR_NOT_SUPPORTED;

    /* Snapshot only values actually touched below. Refuse to take ownership of
     * an already-running engine, then restore this snapshot after the audited
     * vendor teardown. These are observed values, not guessed reset values. */
    s_saved.dump_ctrl = REG32(DUMP_CTRL);
    if (s_saved.dump_ctrl & CTRL_ENABLE_BIT) return ESP_ERR_INVALID_STATE;
    s_saved.dump_ptr_mode = REG32(DUMP_PTR_MODE);
    s_saved.dump_format = REG32(DUMP_FORMAT);
    s_saved.fe_path = REG32(FE_PATH);
    s_saved.fe_aux = REG32(FE_AUX);
    s_saved.fe_enable = REG32(FE_ENABLE);
    s_saved.source_ctrl = REG32(SOURCE_CTRL);
    s_saved.source_mux = REG32(SOURCE_MUX);
    s_saved.hp_sram_usage = REG32(HP_SRAM_USAGE);
    s_saved.modem_clock = REG32(MODEM_CLOCK);
    s_last_restore_ok = false;
    seed_canaries();

    /* Exact set_dump_mode(0): all three supported trigger modes use the
     * ordinary receive source; nonzero set_dump_mode arguments are identical. */
    REG32(SOURCE_CTRL) &= 0xff87ffffu;
    REG32(SOURCE_MUX) = (REG32(SOURCE_MUX) & 0xfffffff8u) | 1u;

    /* Exact automatic-gain subset of C5 v6.0.2 adctrig. The historical
     * sample_80m write is intentionally absent: vendor control flow overwrites
     * those same bits before enable, so forcing them would be a new value. */
    REG32(MODEM_CLOCK) = UINT32_MAX;
    REG32(FE_ENABLE) |= 4u;
    REG32(DUMP_CTRL) = (REG32(DUMP_CTRL) & ~CTRL_LENGTH_MASK) |
                       ((uint32_t)sample_count & CTRL_LENGTH_MASK);
    REG32(DUMP_CTRL) &= ~CTRL_MODE_BIT;

    if (mode == C5VRX_RF_DUMP_MODE_ORDINARY_RX) {
        set_format_fields(0x006c0000u, 0x0001a000u, 0x00000640u, 0x18u);
    } else if (mode == C5VRX_RF_DUMP_MODE_11) {
        set_format_fields(0x005c0000u, 0x00016000u, 0x00000540u, 0x14u);
    }

    REG32(DUMP_FORMAT) |= 0x01000000u;
    /* Public C5 headers identify 0x60095004 as HP SRAM ownership, not a
     * sample clock: [11:8]=2 grants the MAC/dump writer the selected memory
     * bank and bit 16 adds the vendor-observed 64 KiB offset. */
    REG32(HP_SRAM_USAGE) =
        (REG32(HP_SRAM_USAGE) & 0xfffff0ffu) | 0x00000200u;
    REG32(HP_SRAM_USAGE) |= 0x00010000u;
    REG32(DUMP_CTRL) &= ~0x00300000u;
    REG32(DUMP_CTRL) &= ~CTRL_MODE_BIT;

    REG32(FE_PATH) &= ~1u;
    if (mode == C5VRX_RF_DUMP_MODE_ORDINARY_RX) {
        /* Exact mode-0 branch: unlike modes 1..12 it ORs, rather than clears,
         * the overlapping selector. This is why arbitrary rate forcing is not
         * part of this reconstruction. */
        REG32(DUMP_PTR_MODE) |= 0x01e00000u;
    } else {
        REG32(DUMP_PTR_MODE) =
            (REG32(DUMP_PTR_MODE) & ~MODE_SELECT_MASK) |
            0x00160000u;
    }

    __asm__ __volatile__("fence iorw, iorw" ::: "memory");
    s_mode = mode;
    s_configured = true;
    return ESP_OK;
}

esp_err_t c5vrx_rf_dump_start(void)
{
    if (!c5vrx_rf_dump_producer_available()) return ESP_ERR_NOT_SUPPORTED;
    if (!s_configured || s_running) return ESP_ERR_INVALID_STATE;
    REG32(DUMP_CTRL) |= CTRL_ENABLE_BIT;
    /* Exact mode-0 software trigger from the pinned C5 adctrig routine. This
     * occurs after enable and proves that a decoded Wi-Fi packet trigger is
     * not required for the ordinary bounded capture. */
    if (s_mode == C5VRX_RF_DUMP_MODE_ORDINARY_RX) {
        REG32(DUMP_CTRL) |= CTRL_SW_TRIGGER_BIT;
        REG32(DUMP_CTRL) &= ~CTRL_SW_TRIGGER_BIT;
    }
    __asm__ __volatile__("fence iorw, iorw" ::: "memory");
    s_running = true;
    return ESP_OK;
}

esp_err_t c5vrx_rf_dump_get_status(c5vrx_rf_dump_status_t *status)
{
    if (!status) return ESP_ERR_INVALID_ARG;
    if (!c5vrx_rf_dump_producer_available()) return ESP_ERR_NOT_SUPPORTED;
    const uint32_t control = REG32(DUMP_CTRL);
    const uint32_t pointer_mode = REG32(DUMP_PTR_MODE);
    *status = (c5vrx_rf_dump_status_t) {
        .control = control, .pointer_mode = pointer_mode,
        .format = REG32(DUMP_FORMAT), .source_mux = REG32(SOURCE_MUX),
        .fe_path = REG32(FE_PATH), .fe_aux = REG32(FE_AUX),
        .pointer = (uint16_t)pointer_mode,
        .enabled = (control & CTRL_ENABLE_BIT) != 0,
        .wrap_or_done = (control & CTRL_STATUS_BIT) != 0,
    };
    return ESP_OK;
}

esp_err_t c5vrx_rf_dump_stop(void)
{
    if (!c5vrx_rf_dump_producer_available()) return ESP_ERR_NOT_SUPPORTED;
    if (!s_configured) return ESP_ERR_INVALID_STATE;
    REG32(DUMP_CTRL) &= ~CTRL_ENABLE_BIT;
    __asm__ __volatile__("fence iorw, iorw" ::: "memory");
    s_last_canaries_ok = c5vrx_rf_dump_canaries_intact();
    REG32(HP_SRAM_USAGE) &= 0xfffff0ffu;
    REG32(HP_SRAM_USAGE) &= 0xfffeffffu;
    phy_pbus_clear_reg();
    REG32(DUMP_PTR_MODE) = s_saved.dump_ptr_mode;
    REG32(DUMP_FORMAT) = s_saved.dump_format;
    REG32(FE_PATH) = s_saved.fe_path;
    REG32(FE_AUX) = s_saved.fe_aux;
    REG32(FE_ENABLE) = s_saved.fe_enable;
    REG32(SOURCE_CTRL) = s_saved.source_ctrl;
    REG32(SOURCE_MUX) = s_saved.source_mux;
    REG32(HP_SRAM_USAGE) = s_saved.hp_sram_usage;
    REG32(MODEM_CLOCK) = s_saved.modem_clock;
    REG32(DUMP_CTRL) = s_saved.dump_ctrl & ~CTRL_ENABLE_BIT;
    __asm__ __volatile__("fence iorw, iorw" ::: "memory");

    const c5vrx_rf_dump_registers_t restored = read_registers();
    s_last_restore_ok =
        restored.dump_ctrl == (s_saved.dump_ctrl & ~CTRL_ENABLE_BIT) &&
        restored.dump_ptr_mode == s_saved.dump_ptr_mode &&
        restored.dump_format == s_saved.dump_format &&
        restored.fe_path == s_saved.fe_path &&
        restored.fe_aux == s_saved.fe_aux &&
        restored.fe_enable == s_saved.fe_enable &&
        restored.source_ctrl == s_saved.source_ctrl &&
        restored.source_mux == s_saved.source_mux &&
        restored.sram_usage == s_saved.hp_sram_usage &&
        restored.modem_clock == s_saved.modem_clock;
    s_running = false;
    s_configured = false;
    s_canaries_armed = false;
    return s_last_restore_ok && s_last_canaries_ok ? ESP_OK :
                                                    ESP_ERR_INVALID_STATE;
}
