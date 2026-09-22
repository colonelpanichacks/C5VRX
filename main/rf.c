/**
 * rf.c - ESP32-C5 Wi-Fi/PHY receive-only frontend initialization.
 *
 * Configures the RF frontend to receive at 5865 MHz (channel 173) / BW40
 * and routes MODEM_DIAG Q4/I4 to the PARLIO RX GPIO pins.
 *
 * Reference: Seamless Golden 16K (proven best live build).
 * Derived from C5VRX-2 wifi5.c -- stripped of all research/debug baggage.
 *
 * IMPORTANT: BW40 failure returns an error. NO BW20 fallback.
 */

#include "rf.h"

#include <stdint.h>
#include <stdbool.h>

#include "driver/gpio.h"
#include "esp_err.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_rom_gpio.h"
#include "esp_wifi.h"
#include "nvs_flash.h"
#include "soc/gpio_sig_map.h"
#include "modem/modem_syscon_reg.h"
#include "esp_heap_caps.h"
#include "esp_memory_utils.h"
#include "heap_memory_layout.h"
#include "esp_rom_sys.h"
#include "esp_private/wifi_os_adapter.h"

/* Fixed receiver configuration -- not configurable at runtime. */
#define RF_CHANNEL_NUMBER   173u
#define RF_BANDWIDTH        WIFI_BW40

/* Runtime analog filter state; startup remains BW40. */
static bool s_analog_bw40 = true;

/* MAC TX queue hardware registers (IDF-pinned: ESP32-C5, IDF 6.0.x).
 * Identical to C5VRX-2 wifi5.c proven addresses. */
#define REG32(a)         (*(volatile uint32_t *)(uintptr_t)(a))
#define MAC_TXQ0_CONF    0x600a4d6cu
#define MAC_TXQ_STRIDE   0x10u
#define MAC_TXQ_ENABLE   0x80000000u
#define MAC_TXQ_COUNT    5u

/* RX digital filter register (0x600A0430[21:18]) */
#define RX_FILTER_REG       0x600A0430u
#define RX_FILTER_SHIFT     18u
#define RX_FILTER_MASK      (0xFu << RX_FILTER_SHIFT)
#define ADC_RATE_REG        0x600A0448u
#define RX_GAIN_STATUS_REG  0x600A702Cu
#define RX_IQ_CORR_REG      0x600A0438u
#define ADC_RATE_SEL_MASK   0x3u

/* Continuous modem front-end un-gating registers.
 * Required to keep the C5 ADC / modem continuously clocking 80 MS/s IQ
 * into MODEM_DIAG when no 802.11 Wi-Fi packets are present. */
#define DUMP_CTRL       0x600a9004u
#define DUMP_PTR_MODE   0x600a9008u
#define DUMP_FORMAT     0x600a9018u
#define FE_PATH         0x600a20b4u
#define FE_ENABLE       0x600a0800u
#define SOURCE_CTRL     0x600a08ccu
#define SOURCE_MUX      0x600a70b8u
#define MODEM_CLOCK     0x600a9c04u
#define CTRL_ENABLE     0x80000000u
#define CTRL_DUMP_FIRST 0x00020000u
#define TX_START_SELECT 0x00060000u
#define SELECTOR_MASK   0x01fe0000u
#define HP_SRAM_USAGE   0x60095004u

/* MODEM_DIAG lane mapping: Q[9:6] on DIAG[6:9], I[9:6] on DIAG[16:19].
 * GPIO mapping correlated against physical ESP32-C5 hardware captures.
 * These GPIOs connect to the PARLIO RX data_gpio_nums[] array (same order).
 *
 * OUI-SPY mod: Q[8] (DIAG8) is routed to GPIO2 (XIAO MTMS back pad) instead
 * of GPIO25. The bus is GPIO-matrix routed and never leaves the chip, so the
 * lane works on any unconnected pad. This frees GPIO25 (XIAO D2 pad) for the
 * stock OUI-SPY buzzer. */
static const gpio_num_t s_iq_pins[8] = {
    GPIO_NUM_1, GPIO_NUM_0, GPIO_NUM_2, GPIO_NUM_7,    /* Q[9:6] */
    GPIO_NUM_10, GPIO_NUM_5, GPIO_NUM_3, GPIO_NUM_4,   /* I[9:6] */
};
static const uint8_t s_iq_diag[8] = {
    6u, 7u, 8u, 9u,     /* DIAG[6:9]  = Q[9:6] */
    16u, 17u, 18u, 19u, /* DIAG[16:19] = I[9:6] */
};

/* Internal vendor symbol -- globally exported by the pinned IDF 6.0.x
 * pp (protocol processing) library for ESP32-C5. */
extern int lmac_stop_hw_txq(void);

static const char *TAG = "c5vrx3_rf";

/* ARC (vendor-aware receive chain) state: the gain-table description and
 * receive tuple captured read-only from the pinned vendor PHY after init
 * and every successful retune, published as one generation counter. */
static arc_gain_table_t s_arc_gain_table;
static rf_phy_snapshot_t s_arc_receive_tuple;
static uint32_t s_arc_generation;
static uint8_t s_current_gain_val = 52u;

static void arc_capture_vendor_state(void);

/**
 * Disable all 5 LMAC MAC TX hardware queues.
 * Called once after Wi-Fi start to ensure the frontend is receive-only.
 */
static esp_err_t lock_rx_only(void)
{
    (void)lmac_stop_hw_txq();
    for (unsigned q = 0u; q < MAC_TXQ_COUNT; ++q)
        REG32(MAC_TXQ0_CONF - q * MAC_TXQ_STRIDE) &= ~MAC_TXQ_ENABLE;
    __asm__ __volatile__("fence iorw, iorw" ::: "memory");
    /* Verify all queues are disabled. */
    for (unsigned q = 0u; q < MAC_TXQ_COUNT; ++q) {
        if ((REG32(MAC_TXQ0_CONF - q * MAC_TXQ_STRIDE) & MAC_TXQ_ENABLE) != 0u)
            return ESP_ERR_INVALID_STATE;
    }
    return ESP_OK;
}

/**
 * Route MODEM_DIAG DIAG[6:9] and DIAG[16:19] to the GPIO pins used by
 * PARLIO RX. Called after Wi-Fi initializes the PHY clock domain.
 */
static esp_err_t route_modem_iq(void)
{
    uint64_t mask = 0u;
    for (unsigned lane = 0u; lane < 8u; ++lane)
        mask |= 1ULL << s_iq_pins[lane];
    const gpio_config_t cfg = {
        .pin_bit_mask = mask,
        .mode = GPIO_MODE_INPUT_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    esp_err_t err = gpio_config(&cfg);
    if (err != ESP_OK) return err;
    for (unsigned lane = 0u; lane < 8u; ++lane) {
        esp_rom_gpio_connect_out_signal(s_iq_pins[lane],
                                        MODEM_DIAG0_IDX + s_iq_diag[lane],
                                        false, false);
    }
    __asm__ __volatile__("fence iorw, iorw" ::: "memory");
    return ESP_OK;
}

static void rf_enable_continuous_modem(void)
{
    /* Keep CPU ownership of HP SRAM */
    REG32(HP_SRAM_USAGE) = (REG32(HP_SRAM_USAGE) & 0xfffef0ffu) | 0x00010000u;

    /* Un-gate modem clocks and force front-end active. */
    REG32(SOURCE_CTRL) &= 0xff87ffffu;
    REG32(SOURCE_MUX) = (REG32(SOURCE_MUX) & 0xfffffff8u) | 1u;
    REG32(MODEM_CLOCK) = UINT32_MAX;
    REG32(FE_ENABLE) |= 4u;
    REG32(FE_PATH) &= ~1u;

    /* Configure DUMP_FORMAT mode 0 (proven golden RF dump configuration) */
    uint32_t v = REG32(DUMP_FORMAT);
    v = (v & 0xff03ffffu) | 0x006c0000u;
    REG32(DUMP_FORMAT) = v;
    v = (REG32(DUMP_FORMAT) & 0xfffc0fffu) | 0x0001a000u;
    REG32(DUMP_FORMAT) = v;
    v = (REG32(DUMP_FORMAT) & 0xfffff03fu) | 0x00000640u;
    REG32(DUMP_FORMAT) = v;
    v = (REG32(DUMP_FORMAT) & 0xffffffc0u) | 0x18u;
    REG32(DUMP_FORMAT) = v | 0x01000000u;

    /* Set TX_START selector in pre-trigger circular mode (TX_START_SELECT = 0x00060000).
     * Because MAC TX queues are quiescent, TX_START never fires. With CTRL_DUMP_FIRST,
     * the hardware continuously streams pre-trigger samples onto the MODEM_DIAG bus.
     * Crucial: 0x01e00000 software trigger bits are masked out. */
    REG32(DUMP_PTR_MODE) = (REG32(DUMP_PTR_MODE) & ~SELECTOR_MASK) | TX_START_SELECT;

    /* Control: CTRL_DUMP_FIRST, length 16384, ENABLE */
    uint32_t ctrl = REG32(DUMP_CTRL);
    ctrl &= ~(CTRL_ENABLE | 0x00080000u | 0x00040000u); /* Clear ENABLE, START, DONE */
    ctrl |= CTRL_DUMP_FIRST;
    ctrl = (ctrl & ~0x0001ffffu) | 16384u;
    REG32(DUMP_CTRL) = ctrl;

    __asm__ __volatile__("fence iorw, iorw" ::: "memory");

    /* Arm dump engine with ENABLE only */
    REG32(DUMP_CTRL) = ctrl | CTRL_ENABLE;

    __asm__ __volatile__("fence iorw, iorw" ::: "memory");
}

/* Track all timers created/armed by the closed-source Wi-Fi stack */
typedef struct {
    void *timer;
    void *fn;
    void *arg;
    uint32_t period_ms;
    bool repeat;
    bool armed;
    uint32_t arm_count;
} tracked_timer_t;

#define MAX_TRACKED_TIMERS 32
static tracked_timer_t s_tracked_timers[MAX_TRACKED_TIMERS];
static size_t s_num_tracked_timers = 0;
static wifi_osi_funcs_t s_custom_osi_funcs;

static tracked_timer_t *find_or_create_timer_slot(void *timer)
{
    for (size_t i = 0; i < s_num_tracked_timers; ++i) {
        if (s_tracked_timers[i].timer == timer) return &s_tracked_timers[i];
    }
    if (s_num_tracked_timers < MAX_TRACKED_TIMERS) {
        tracked_timer_t *slot = &s_tracked_timers[s_num_tracked_timers++];
        memset(slot, 0, sizeof(*slot));
        slot->timer = timer;
        return slot;
    }
    return NULL;
}

static void tracked_timer_setfn(void *ptimer, void *pfunction, void *parg)
{
    tracked_timer_t *slot = find_or_create_timer_slot(ptimer);
    if (slot) {
        slot->fn = pfunction;
        slot->arg = parg;
    }
    g_wifi_osi_funcs._timer_setfn(ptimer, pfunction, parg);
}

static void tracked_timer_arm(void *timer, uint32_t tmout, bool repeat)
{
    tracked_timer_t *slot = find_or_create_timer_slot(timer);
    if (slot) {
        slot->period_ms = tmout;
        slot->repeat = repeat;
        slot->armed = true;
        slot->arm_count++;
    }
    g_wifi_osi_funcs._timer_arm(timer, tmout, repeat);
}

static void tracked_timer_arm_us(void *ptimer, uint32_t us, bool repeat)
{
    tracked_timer_t *slot = find_or_create_timer_slot(ptimer);
    if (slot) {
        slot->period_ms = (us + 500u) / 1000u;
        slot->repeat = repeat;
        slot->armed = true;
        slot->arm_count++;
    }
    g_wifi_osi_funcs._timer_arm_us(ptimer, us, repeat);
}

static void tracked_timer_disarm(void *timer)
{
    tracked_timer_t *slot = find_or_create_timer_slot(timer);
    if (slot) {
        slot->armed = false;
    }
    g_wifi_osi_funcs._timer_disarm(timer);
}

static void tracked_timer_done(void *ptimer)
{
    tracked_timer_t *slot = find_or_create_timer_slot(ptimer);
    if (slot) {
        slot->armed = false;
    }
    g_wifi_osi_funcs._timer_done(ptimer);
}

void rf_dump_tracked_timers(void)
{
    printf("\n=== WI-FI VENDOR TIMERS INVENTORY (%u tracked) ===\n", (unsigned)s_num_tracked_timers);
    for (size_t i = 0; i < s_num_tracked_timers; ++i) {
        printf(" [%u] fn=0x%08lx period=%4lu ms repeat=%d armed=%d arms=%lu\n",
               (unsigned)i,
               (unsigned long)(uintptr_t)s_tracked_timers[i].fn,
               (unsigned long)s_tracked_timers[i].period_ms,
               s_tracked_timers[i].repeat ? 1 : 0,
               s_tracked_timers[i].armed ? 1 : 0,
               (unsigned long)s_tracked_timers[i].arm_count);
    }
    printf("==================================================\n\n");
    fflush(stdout);
}

static esp_err_t init_nvs(void)
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES ||
        err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        err = nvs_flash_erase();
        if (err == ESP_OK) err = nvs_flash_init();
    }
    return err;
}

esp_err_t rf_start(void)
{
    /* NVS is required by ESP-IDF Wi-Fi/PHY initialization. */
    esp_err_t err = init_nvs();
    if (err != ESP_OK) return err;

    /* esp_netif_init + default event loop are required by esp_wifi_init().
     * Tolerant of ESP_ERR_INVALID_STATE (already initialized by IDF). */
    err = esp_netif_init();
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) return err;
    err = esp_event_loop_create_default();
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) return err;

    /* Install tracked OSI functions to inventory all Wi-Fi vendor timers */
    s_custom_osi_funcs = g_wifi_osi_funcs;
    s_custom_osi_funcs._timer_setfn = tracked_timer_setfn;
    s_custom_osi_funcs._timer_arm = tracked_timer_arm;
    s_custom_osi_funcs._timer_arm_us = tracked_timer_arm_us;
    s_custom_osi_funcs._timer_disarm = tracked_timer_disarm;
    s_custom_osi_funcs._timer_done = tracked_timer_done;

    /* Initialize Wi-Fi driver with RAM-only storage -- no NVS needed.
     * Crucial: sta_disconnected_pm MUST be false. By default, ESP-IDF enables
     * power management for disconnected stations, periodically shutting down
     * RF, PHY, and BB when idle, which causes periodic loss of MODEM_DIAG clocking. */
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    cfg.osi_funcs = &s_custom_osi_funcs;
    cfg.sta_disconnected_pm = false;
    if ((err = esp_wifi_init(&cfg)) != ESP_OK) return err;
    if ((err = esp_wifi_set_storage(WIFI_STORAGE_RAM)) != ESP_OK) return err;
    if ((err = esp_wifi_set_mode(WIFI_MODE_STA)) != ESP_OK) return err;
    if ((err = esp_wifi_start()) != ESP_OK) return err;

    /* Dump initial Wi-Fi timers armed during startup */
    rf_dump_tracked_timers();

    /* Force 5 GHz band only. */
#if CONFIG_SOC_WIFI_SUPPORT_5G
    if ((err = esp_wifi_set_band_mode(WIFI_BAND_MODE_5G_ONLY)) != ESP_OK)
        return err;
#else
    return ESP_ERR_NOT_SUPPORTED;
#endif

    /* No power saving -- PHY clock must remain alive at all times. */
    if ((err = esp_wifi_set_ps(WIFI_PS_NONE)) != ESP_OK) return err;

    /* Restrict 5 GHz protocols. */
    wifi_protocols_t protocols = {
        .ghz_2g = WIFI_PROTOCOL_11B | WIFI_PROTOCOL_11G |
                  WIFI_PROTOCOL_11N | WIFI_PROTOCOL_11AX,
        .ghz_5g = WIFI_PROTOCOL_11A | WIFI_PROTOCOL_11N,
    };
    if ((err = esp_wifi_set_protocols(WIFI_IF_STA, &protocols)) != ESP_OK)
        return err;

    /* Set BW40 on 5 GHz. Hard failure if not available -- NO BW20 fallback.
     * BW40 is a fixed hardware requirement for MODEM_DIAG IQ precision. */
    wifi_bandwidths_t bandwidths = {
        .ghz_2g = WIFI_BW20,
        .ghz_5g = RF_BANDWIDTH,
    };
    err = esp_wifi_set_bandwidths(WIFI_IF_STA, &bandwidths);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "BW40 not available (err=%s). No BW20 fallback.", esp_err_to_name(err));
        return err;  /* Hard failure. BW20 produces degraded Q4/I4. */
    }

    /* Channel 173 = 5865 MHz. */
    if ((err = esp_wifi_set_channel(RF_CHANNEL_NUMBER, WIFI_SECOND_CHAN_NONE)) != ESP_OK)
        return err;

    /* Promiscuous mode keeps the RX path and MODEM_DIAG bus active.
     * Zero filter mask prevents LMAC from buffering packets or firing software interrupts. */
    if ((err = esp_wifi_set_promiscuous(true)) != ESP_OK) return err;
    wifi_promiscuous_filter_t filter = { .filter_mask = 0 };
    (void)esp_wifi_set_promiscuous_filter(&filter);

    /* Hardware-disable all 5 LMAC TX queues. Receive-only from here on. */
    if ((err = lock_rx_only()) != ESP_OK) return err;

    /* Verify channel lock. */
    uint8_t primary = 0u;
    wifi_second_chan_t secondary = WIFI_SECOND_CHAN_NONE;
    if ((err = esp_wifi_get_channel(&primary, &secondary)) != ESP_OK) return err;
    if (primary != RF_CHANNEL_NUMBER) {
        ESP_LOGE(TAG, "Channel mismatch: got %u, expected %u", primary, RF_CHANNEL_NUMBER);
        return ESP_ERR_INVALID_STATE;
    }

    /* Route MODEM_DIAG to PARLIO RX GPIO pins. */
    if ((err = route_modem_iq()) != ESP_OK) return err;

    /* Un-gate modem ADC clock and force continuous sampling. */
    rf_enable_continuous_modem();

    /* Keep the vendor Wi-Fi packet AGC out of the analog-FM receive path.
     * C5VRX has its own slow analog-video gain controller below; leaving the
     * packet AGC enabled lets the closed PHY hunt/recalibrate independently,
     * which invalidates our gain model and can desensitize weak-signal receive. */
    extern void phy_disable_agc(void);
    extern void phy_rfagc_disable(void);
    phy_disable_agc();
    phy_rfagc_disable();

    /* Boot wide for full analog-FM video bandwidth. Runtime BW20/AUTO is
     * explicitly opt-in from the native menu; BW40 remains the safe default. */
    extern void phy_wifi_fbw_sel(uint32_t val);
    phy_wifi_fbw_sel(s_analog_bw40 ? 1u : 0u);

    /* Force high-sensitivity sweet-spot gain (index 52).
     * Provides sensitive reception of weak carriers out of the box while
     * active AGC dynamically manages gain tracking and overload protection. */
    extern void phy_force_rx_gain(bool enable, uint8_t gain_idx);
    phy_force_rx_gain(true, 52);

    /* Vendor PHY initialization has now generated both valid RX gain tables
     * and completed its own calibration. Capture that state read-only before
     * C5VRX freezes receiver ownership. */
    arc_capture_vendor_state();

    /* Disable PHY PLL / RXCAL tracking timer if compiled in, so it never
     * recalibrates RF / RX hardware during continuous analog video reception.
     * With CONFIG_ESP_PHY_DISABLE_PLL_TRACK=y, the tracking timer is omitted entirely. */
#if !CONFIG_ESP_PHY_DISABLE_PLL_TRACK
    extern void phy_track_pll_deinit(void);
    phy_track_pll_deinit();
#endif

    ESP_EARLY_LOGW(TAG, "RF ready: 5865 MHz / ch%u / BW40 / gain=forced(52) / sta_disconnected_pm=0 / pll_track=disabled",
                   RF_CHANNEL_NUMBER);
    return ESP_OK;
}

extern void phy_wifi_fbw_sel(uint32_t val);
extern void phy_force_rx_gain(bool enable, uint8_t gain_idx);
extern void phy_disable_agc(void);
extern void phy_rfagc_disable(void);
extern void phy_set_freq(uint16_t freq_mhz, int offset);
extern void phy_chip_set_chan_offset(int offset_khz);
extern void phy_fft_scale_force(bool force_en, int8_t force_value);

/* Read-only C5 PHY observations. Estimator/calibration routines are not called
 * while live because they reconfigure clocks and receive state. */
extern int phy_get_noise_floor(void) __attribute__((weak));
extern int phy_get_rssi(void) __attribute__((weak));

/* Last gain value actually forced into the PHY; -1 = never forced. Lets the
 * CFO/offset path skip redundant phy_force_rx_gain() calls, which each cause
 * a visible Q/dc dip in the demodulated baseband. (OUI-SPY mod) */
static int s_last_forced_gain = -1;

/* Standard FPV Channel Table: 6 Bands x 8 Channels = 48 Channels
 * RaceBand (R), Boscam A (A), Boscam B (B), Boscam E (E), FatShark (F), LowBand (L) */
static const fpv_channel_t s_fpv_channels[FPV_BAND_COUNT][8] = {
    [FPV_BAND_R] = { /* RaceBand (R1..R8) */
        { "R1", 5658 }, { "R2", 5695 }, { "R3", 5732 }, { "R4", 5769 },
        { "R5", 5806 }, { "R6", 5843 }, { "R7", 5880 }, { "R8", 5917 },
    },
    [FPV_BAND_A] = { /* Boscam A (A1..A8) - Default A1 is 5865 MHz */
        { "A1", 5865 }, { "A2", 5845 }, { "A3", 5825 }, { "A4", 5805 },
        { "A5", 5785 }, { "A6", 5765 }, { "A7", 5745 }, { "A8", 5725 },
    },
    [FPV_BAND_B] = { /* Boscam B (B1..B8) */
        { "B1", 5733 }, { "B2", 5752 }, { "B3", 5771 }, { "B4", 5790 },
        { "B5", 5809 }, { "B6", 5828 }, { "B7", 5847 }, { "B8", 5866 },
    },
    [FPV_BAND_E] = { /* Boscam E (E1..E8) */
        { "E1", 5705 }, { "E2", 5685 }, { "E3", 5665 }, { "E4", 5645 },
        { "E5", 5885 }, { "E6", 5905 }, { "E7", 5925 }, { "E8", 5945 },
    },
    [FPV_BAND_F] = { /* FatShark / Airwave (F1..F8) */
        { "F1", 5740 }, { "F2", 5760 }, { "F3", 5780 }, { "F4", 5800 },
        { "F5", 5820 }, { "F6", 5840 }, { "F7", 5860 }, { "F8", 5880 },
    },
    [FPV_BAND_L] = { /* LowBand / Band D (L1..L8) */
        { "L1", 5362 }, { "L2", 5399 }, { "L3", 5436 }, { "L4", 5473 },
        { "L5", 5510 }, { "L6", 5547 }, { "L7", 5584 }, { "L8", 5621 },
    },
};

static const char *s_band_names[FPV_BAND_COUNT] = {
    [FPV_BAND_R] = "RaceBand (R)",
    [FPV_BAND_A] = "Boscam A (A)",
    [FPV_BAND_B] = "Boscam B (B)",
    [FPV_BAND_E] = "Boscam E (E)",
    [FPV_BAND_F] = "FatShark (F)",
    [FPV_BAND_L] = "LowBand (L)",
};

static fpv_band_t s_current_band = FPV_BAND_A;
static uint8_t s_current_channel_idx = 0; /* 0..7 (Default A1: 5865 MHz) */
static uint16_t s_current_freq_mhz = 5865u;
static int s_current_offset_khz = 0;

#define C5_WIFI5_MIN_MHZ 5180u
#define C5_WIFI5_MAX_MHZ 5885u

typedef struct {
    uint8_t channel;
    uint16_t mhz;
} wifi5_center_t;

/* Public ESP-IDF 5 GHz centers used as the supported RF bootstrap.
 * Non-exact FPV centers are experimental and are retuned only after first
 * placing the closed PHY on the nearest known-good public center. */
static const wifi5_center_t s_wifi5_centers[] = {
    {132, 5660}, {136, 5680}, {140, 5700}, {144, 5720},
    {149, 5745}, {153, 5765}, {157, 5785}, {161, 5805},
    {165, 5825}, {169, 5845}, {173, 5865}, {177, 5885},
};

static bool plan_wifi5_center(uint16_t freq_mhz, uint8_t *channel, uint16_t *center_mhz)
{
    if (freq_mhz < C5_WIFI5_MIN_MHZ || freq_mhz > C5_WIFI5_MAX_MHZ) {
        return false;
    }

    unsigned best = 0;
    int best_delta = 0x7fffffff;
    for (unsigned i = 0; i < sizeof(s_wifi5_centers) / sizeof(s_wifi5_centers[0]); ++i) {
        int d = (int)freq_mhz - (int)s_wifi5_centers[i].mhz;
        if (d < 0) d = -d;
        if (d < best_delta) {
            best_delta = d;
            best = i;
        }
    }

    if (channel) *channel = s_wifi5_centers[best].channel;
    if (center_mhz) *center_mhz = s_wifi5_centers[best].mhz;
    return true;
}

void rf_set_analog_bandwidth(bool bw40)
{
    s_analog_bw40 = bw40;
    phy_wifi_fbw_sel(bw40 ? 1u : 0u);
}

bool rf_get_analog_bandwidth(void)
{
    return s_analog_bw40;
}

void rf_set_rx_gain(bool force, uint8_t gain_idx)
{
    if (force) {
        s_current_gain_val = gain_idx;
        s_last_forced_gain = gain_idx;
    }
    phy_force_rx_gain(force, gain_idx);
}

uint32_t rf_get_rx_gain_reg(void)
{
    return REG32(RX_GAIN_STATUS_REG);
}

/* OUI-SPY: last gain value actually forced into the PHY (-1 = none yet).
 * Lets video.c count the rf.c hidden gain re-assert on offset writes for
 * issue #28 lag correlation exactly when it really happens. */
int rf_get_last_forced_gain(void)
{
    return s_last_forced_gain;
}

void rf_get_phy_snapshot(rf_phy_snapshot_t *snapshot)
{
    if (!snapshot) return;

    snapshot->gain_reg = REG32(RX_GAIN_STATUS_REG);
    snapshot->rx_filter_reg = REG32(RX_FILTER_REG);
    snapshot->adc_rate_reg = REG32(ADC_RATE_REG);
    snapshot->source_mux_reg = REG32(SOURCE_MUX);
    snapshot->iq_correction_reg = REG32(RX_IQ_CORR_REG);
    snapshot->rx_filter_mode =
        (uint8_t)((snapshot->rx_filter_reg & RX_FILTER_MASK) >> RX_FILTER_SHIFT);
    snapshot->adc_rate_sel =
        (uint8_t)(snapshot->adc_rate_reg & ADC_RATE_SEL_MASK);
    snapshot->iq_correction =
        arc_iq_correction_decode(snapshot->iq_correction_reg);
    if (!arc_gain_tuple_decode(&s_arc_gain_table, s_current_gain_val,
                               &snapshot->gain_tuple)) {
        snapshot->gain_tuple = (arc_gain_tuple_t){0};
        snapshot->gain_tuple.gain_index = s_current_gain_val;
    }
}

static void arc_capture_vendor_state(void)
{
    arc_phy_capture_gain_table(&s_arc_gain_table);
    rf_get_phy_snapshot(&s_arc_receive_tuple);
    /* Publish last so readers never associate a new generation with a tuple
     * that is still being filled. The controller is the sole retune owner. */
    ++s_arc_generation;
}

const rf_phy_snapshot_t *rf_get_arc_receive_tuple(void)
{
    return &s_arc_receive_tuple;
}

void rf_set_fft_scale_force(bool force, int8_t value)
{
    /* The symbol is exported by the ESP32-C5 ROM PHY and is also used by
     * Espressif's CSI gain-control design. Keep it lab-only: whether it is
     * upstream of raw MODEM_DIAG is exactly what the FFT probe measures. */
    phy_fft_scale_force(force, value);
}


bool rf_try_get_noise_floor_dbm(int *dbm)
{
    if (!dbm || !phy_get_noise_floor) return false;
    int value = phy_get_noise_floor();
    /* Reject impossible values instead of feeding an ABI mismatch into AUTO. */
    if (value < -140 || value > -20) return false;
    *dbm = value;
    return true;
}

bool rf_try_get_wideband_rssi_dbm(int *dbm)
{
    if (!dbm || !phy_get_rssi) return false;
    int value = phy_get_rssi();
    if (value < -140 || value > 10) return false;
    *dbm = value;
    return true;
}

const arc_gain_table_t *rf_get_arc_gain_table(void)
{
    return &s_arc_gain_table;
}

uint8_t rf_get_arc_survival_gain(void)
{
    return arc_gain_highest_rf_stage_start(&s_arc_gain_table);
}

uint32_t rf_get_arc_generation(void)
{
    return s_arc_generation;
}

const fpv_channel_t *rf_get_current_channel(void)
{
    return &s_fpv_channels[s_current_band][s_current_channel_idx];
}

size_t rf_get_channel_index(void)
{
    return (size_t)s_current_band * 8u + s_current_channel_idx;
}

size_t rf_get_channel_count(void)
{
    return FPV_BAND_COUNT * 8u;
}

fpv_band_t rf_get_current_band(void)
{
    return s_current_band;
}

const char *rf_get_band_name(fpv_band_t band)
{
    if (band >= FPV_BAND_COUNT) return "Unknown";
    return s_band_names[band];
}

uint8_t rf_get_current_channel_number(void)
{
    return s_current_channel_idx + 1u;
}

uint16_t rf_get_frequency_mhz(void)
{
    return s_current_freq_mhz;
}

int rf_get_frequency_offset_khz(void)
{
    return s_current_offset_khz;
}

void rf_set_frequency_offset_khz(int offset_khz)
{
    /* Strict clamping: +/- 1500 kHz (+/- 1.5 MHz) maximum.
     * Adjacent FPV channels are at least 19-20 MHz apart. Clamping strictly
     * to +/- 1.5 MHz guarantees 100% that tuning is locked to the selected
     * channel and can NEVER hop or switch to another channel. */
    if (offset_khz < -1500) offset_khz = -1500;
    if (offset_khz > 1500)  offset_khz = 1500;

    s_current_offset_khz = offset_khz;
    phy_chip_set_chan_offset(offset_khz);
    /* Re-forcing the same gain on every CFO tweak punches a Q/dc dip into
     * the baseband; only re-assert when the gain actually changed. (OUI-SPY) */
    if ((int)s_current_gain_val != s_last_forced_gain) {
        phy_force_rx_gain(true, s_current_gain_val);
        s_last_forced_gain = s_current_gain_val;
    }
}

void rf_step_frequency_offset_khz(int delta_khz)
{
    rf_set_frequency_offset_khz(s_current_offset_khz + delta_khz);
}

esp_err_t rf_set_channel(size_t index)
{
    if (index >= FPV_BAND_COUNT * 8u) {
        return ESP_ERR_INVALID_ARG;
    }

    fpv_band_t new_band = (fpv_band_t)(index / 8u);
    uint8_t new_idx = (uint8_t)(index % 8u);
    uint16_t requested_mhz = s_fpv_channels[new_band][new_idx].freq_mhz;

    uint8_t wifi_channel = 0;
    uint16_t wifi_center_mhz = 0;
    if (!plan_wifi5_center(requested_mhz, &wifi_channel, &wifi_center_mhz)) {
        printf("[RF:TUNE] Refusing %u MHz: outside ESP32-C5 5 GHz operating window %u-%u MHz\n",
               requested_mhz, C5_WIFI5_MIN_MHZ, C5_WIFI5_MAX_MHZ);
        return ESP_ERR_NOT_SUPPORTED;
    }

    /* Always establish a supported/public RF center first. Exact-overlap FPV
     * channels (e.g. A1/A2/...) need no undocumented frequency call at all. */
    esp_err_t err = esp_wifi_set_channel(wifi_channel, WIFI_SECOND_CHAN_NONE);
    if (err != ESP_OK) {
        return err;
    }

    uint8_t verify_primary = 0;
    wifi_second_chan_t verify_secondary = WIFI_SECOND_CHAN_NONE;
    err = esp_wifi_get_channel(&verify_primary, &verify_secondary);
    if (err != ESP_OK || verify_primary != wifi_channel) {
        return (err != ESP_OK) ? err : ESP_ERR_INVALID_STATE;
    }

    if (requested_mhz != wifi_center_mhz) {
        /* EXPERIMENTAL: two-argument ABI is known, but arbitrary-frequency
         * semantics still require RF hardware validation. Starting from the
         * nearest public center minimizes the size of this undocumented step. */
        phy_set_freq(requested_mhz, 0);
    }

    rf_enable_continuous_modem();

    /* Public/undocumented retune paths can touch PHY receive state. Re-assert
     * the currently selected analog bandwidth after every channel change. */
    phy_disable_agc();
    phy_rfagc_disable();
    phy_wifi_fbw_sel(s_analog_bw40 ? 1u : 0u);
    phy_force_rx_gain(true, s_current_gain_val);
    s_last_forced_gain = s_current_gain_val;

    /* A channel change may make the vendor PHY regenerate its active RX gain
     * table and calibrated receive state. Recapture only after the retune and
     * all receive-state reassertions succeeded, then publish one generation
     * change so ARC cannot keep stale spans/maxima or controller state. */
    arc_capture_vendor_state();

    /* Commit logical state only after the supported bootstrap succeeded. */
    s_current_band = new_band;
    s_current_channel_idx = new_idx;
    s_current_freq_mhz = requested_mhz;
    s_current_offset_khz = 0;

    return ESP_OK;
}

esp_err_t rf_cycle_channel(void)
{
    size_t start = rf_get_channel_index();
    for (size_t step = 1; step <= FPV_BAND_COUNT * 8u; ++step) {
        size_t next = (start + step) % (FPV_BAND_COUNT * 8u);
        esp_err_t err = rf_set_channel(next);
        if (err == ESP_OK) return ESP_OK;
        if (err != ESP_ERR_NOT_SUPPORTED) return err;
    }
    return ESP_ERR_NOT_FOUND;
}

void rf_cycle_band(void)
{
    fpv_band_t start_band = s_current_band;
    uint8_t channel_idx = s_current_channel_idx;
    for (unsigned step = 1; step <= FPV_BAND_COUNT; ++step) {
        fpv_band_t band = (fpv_band_t)((start_band + step) % FPV_BAND_COUNT);
        esp_err_t err = rf_set_channel((size_t)band * 8u + channel_idx);
        if (err == ESP_OK) return;
        if (err != ESP_ERR_NOT_SUPPORTED) return;
    }
}

void rf_cycle_channel_in_band(void)
{
    fpv_band_t band = s_current_band;
    uint8_t start_idx = s_current_channel_idx;
    for (unsigned step = 1; step <= 8u; ++step) {
        uint8_t idx = (uint8_t)((start_idx + step) % 8u);
        esp_err_t err = rf_set_channel((size_t)band * 8u + idx);
        if (err == ESP_OK) return;
        if (err != ESP_ERR_NOT_SUPPORTED) return;
    }
}
