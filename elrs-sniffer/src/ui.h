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
//   ui_probe()  -> Wire setup + bounded ACK check for the SSD1306
//   ui_start()  -> u8g2 init + splash (only after probe succeeds)
//   ui_show_fault() -> static fault screen when radio init failed
bool ui_probe();
void ui_start();
void ui_show_fault(const char *what, const char *detail);
void ui_render(const ui_state_t *st);
#else
static inline bool ui_probe() { return false; }
static inline void ui_start() {}
static inline void ui_show_fault(const char *, const char *) {}
static inline void ui_render(const ui_state_t *) {}
#endif
