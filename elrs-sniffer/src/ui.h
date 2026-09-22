// =============================================================================
// ui.h — onboard display UI, styled after the OUI-SPY dashboard dark theme
// (tools/flask_app.py :root palette, verified from the repo):
//   --bg #050805  --panel #0a120a  --dim #33502f  --txt #9fdc9f
//   --grn #39ff6a  --amb #ffb000  --red #ff4a3d  (cyan #31c8ff accent)
// Layout (T-Embed ST7789 portrait 170x320):
//   top    — packets/s + rate + RSSI/SNR + lock state
//   middle — 4 stick bar meters (+ AUX1 arm tick)
//   bottom — latest telemetry lines + LQ + link fingerprint
// =============================================================================
#pragma once
#include <stdint.h>
#include <stdbool.h>

#define UI_TLM_LINES 3

// Panel driver: LilyGo ships these 0.96" panels interchangeably, labelled
// SSD1306, and some are actually SH1106 (classic gotcha — SH1106 on the
// SSD1306 init sequence = dark screen while the I2C ACK probe still passes).
// Both U8G2 drivers are compiled in; the active one is chosen at runtime and
// persisted (default SH1106).
typedef enum { OLED_DRV_SSD1306 = 0, OLED_DRV_SH1106 = 1 } oled_driver_t;

typedef struct {
    bool     has_rc;
    bool     radio_ok;          // false -> fault banner, stats carry "radio":0
    uint32_t ch_us[4];          // stick pulse widths 988..2012
    bool     armed;             // AUX1 high
    float    rssi_dbm;
    float    snr_db;
    uint32_t pps;               // demodulated ELRS-classified packets/sec
    uint32_t lq_permille;       // observed/expected while locked
    char     rate[16];          // e.g. "LoRa 250Hz"
    bool     iq_inverted;
    bool     locked;
    uint32_t freq_hz;
    char     tlm[UI_TLM_LINES][32];
    char     ident[28];         // link fingerprint, honest about weakness
    char     fault[24];         // persistent fault text (e.g. "RADIO FAULT")
} ui_state_t;

#ifdef ARDUINO
// Boot-time sequencing (all bounded — see main.cpp):
//   ui_probe()  -> Wire setup + bounded ACK check (driver-agnostic)
//   ui_start(drv)-> init the chosen driver + 1.5 s splash naming the driver
//   ui_show_fault() -> static fault screen when radio init failed
// Runtime driver switch (both toggle paths call ui_toggle_driver):
//   re-inits with the other driver, redraws the splash, blocks 1.5 s.
bool ui_probe();
void ui_start(oled_driver_t drv);
void ui_show_fault(const char *what, const char *detail);
void ui_render(const ui_state_t *st);
oled_driver_t ui_toggle_driver();
oled_driver_t ui_get_driver();
const char *ui_driver_name(oled_driver_t drv);
#else
static inline bool ui_probe() { return false; }
static inline void ui_start(oled_driver_t) {}
static inline void ui_show_fault(const char *, const char *) {}
static inline void ui_render(const ui_state_t *) {}
static inline oled_driver_t ui_toggle_driver() { return OLED_DRV_SH1106; }
static inline oled_driver_t ui_get_driver() { return OLED_DRV_SH1106; }
static inline const char *ui_driver_name(oled_driver_t) { return "none"; }
#endif
