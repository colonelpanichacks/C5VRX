#pragma once
#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"
#include "arc_phy.h"

/**
 * rf_start() - Initialize the ESP32-C5 Wi-Fi/PHY receive-only frontend.
 *
 * Configures:
 *   - 5 GHz band only (WIFI_BAND_MODE_5G_ONLY)
 *   - Channel 173 = 5865 MHz / BW40
 *   - No power saving (WIFI_PS_NONE)
 *   - Promiscuous RX to keep MODEM_DIAG active
 *   - All 5 LMAC TX queues hardware-disabled (receive-only)
 *   - MODEM_DIAG DIAG[6:9] (Q[9:6]) and DIAG[16:19] (I[9:6]) routed to GPIO
 *
 * Returns ESP_OK on success.
 * Returns an error if BW40 is not available -- NO BW20 fallback.
 * Returns an error if channel verification fails.
 */
esp_err_t rf_start(void);

/**
 * Dump all vendor timers intercepted during Wi-Fi operation.
 */
void rf_dump_tracked_timers(void);

/**
 * Control PHY receiver frontend gain.
 * force = true sets fixed gain index (0 = min gain / max attenuation, ~30-60 = high gain).
 * force = false restores automatic / default PHY gain.
 */
/** Runtime analog receive filter: true=BW40, false=BW20. */
void rf_set_analog_bandwidth(bool bw40);
bool rf_get_analog_bandwidth(void);

void rf_set_rx_gain(bool force, uint8_t gain_idx);
uint32_t rf_get_rx_gain_reg(void);
int rf_get_last_forced_gain(void);

/**
 * Read-only snapshot of known ESP32-C5 RX PHY registers used for lab
 * characterization. No undocumented PHY function is called by this helper.
 */
typedef struct {
    uint32_t gain_reg;
    uint32_t rx_filter_reg;
    uint32_t adc_rate_reg;
    uint32_t source_mux_reg;
    uint32_t iq_correction_reg;
    uint8_t rx_filter_mode;
    uint8_t adc_rate_sel;
    arc_iq_correction_t iq_correction;
    arc_gain_tuple_t gain_tuple;
} rf_phy_snapshot_t;

void rf_get_phy_snapshot(rf_phy_snapshot_t *snapshot);
/* Vendor-calibrated, read-only receive tuple captured atomically with the ARC
 * gain table after PHY init or a successful channel retune. It is diagnostic
 * state only; undocumented fields are never swept or reapplied in LOCK. */
const rf_phy_snapshot_t *rf_get_arc_receive_tuple(void);

/**
 * Force/release the PHY FFT scaling stage. This is exposed only so the lab can
 * prove whether FFT gain sits upstream or downstream of the MODEM_DIAG Q4/I4
 * tap. Production receive control must not depend on this until hardware data
 * demonstrates a Q4/I4 effect.
 */
void rf_set_fft_scale_force(bool force, int8_t value);

/**
 * Experimental PHY observability/control used only by explicit RX profiles.
 * These wrappers keep undocumented symbols isolated in rf.c.
 *
 * Hardware AGC mode is opt-in and reversible. Normal C5VRX operation keeps
 * Espressif packet AGC disabled and uses the fixed-gain C5VRX controller.
 */
bool rf_try_get_noise_floor_dbm(int *dbm);
bool rf_try_get_wideband_rssi_dbm(int *dbm);
const arc_gain_table_t *rf_get_arc_gain_table(void);
uint8_t rf_get_arc_survival_gain(void);
/* Changes after every successful PHY init/channel retune and lets the ARC
 * supervisor discard controller state derived from an older vendor table. */
uint32_t rf_get_arc_generation(void);

/**
 * FPV Channel and Carrier Frequency Fine-Tuning:
 */
typedef struct {
    const char *name;     /* e.g. "A1", "R5", etc. */
    uint16_t freq_mhz;   /* Base channel center frequency in MHz */
} fpv_channel_t;

typedef enum {
    FPV_BAND_R = 0,  /* RaceBand (R1..R8) */
    FPV_BAND_A = 1,  /* Boscam A (A1..A8) */
    FPV_BAND_B = 2,  /* Boscam B (B1..B8) */
    FPV_BAND_E = 3,  /* Boscam E (E1..E8) */
    FPV_BAND_F = 4,  /* FatShark / Airwave (F1..F8) */
    FPV_BAND_L = 5,  /* LowBand (L1..L8) */
    FPV_BAND_COUNT = 6
} fpv_band_t;

const fpv_channel_t *rf_get_current_channel(void);
size_t rf_get_channel_index(void);
size_t rf_get_channel_count(void);
esp_err_t rf_set_channel(size_t index);
esp_err_t rf_cycle_channel(void);

fpv_band_t rf_get_current_band(void);
const char *rf_get_band_name(fpv_band_t band);
void rf_cycle_band(void);
uint8_t rf_get_current_channel_number(void);
void rf_cycle_channel_in_band(void);

uint16_t rf_get_frequency_mhz(void);
int rf_get_frequency_offset_khz(void);
void rf_set_frequency_offset_khz(int offset_khz);
void rf_step_frequency_offset_khz(int delta_khz);
