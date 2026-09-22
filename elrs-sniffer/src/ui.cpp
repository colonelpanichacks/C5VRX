// ui.cpp — onboard 128x64 OLED (SSD1306/SH1106 dual-driver), OUI-SPY dark theme
// (see ui.h). LilyGo ships the 0.96" panels interchangeably; some units
// labelled SSD1306 are actually SH1106 (dark screen on the SSD1306 init
// while the I2C ACK still passes). Both U8G2 drivers are compiled in and
// the active one is picked at runtime, persisted by main.cpp, default SH1106.
//
// Layout (128x64):
//   y=8   <rate> <IQ>                 [scan|LOCK]      (inverted when ARMED)
//   y=16  <pps>pps <rssi>dBm <snr>dB
//   y=24  <freq MHz> LQ<x>% [uid3]
//   y=27..50  four vertical stick meters (R/P/T/Y)
//   y=62  telemetry line (rotates through tlm[0..2] once per second)
#include "ui.h"
#ifdef ARDUINO
#include "board_pins.h"
#include <Arduino.h>
#if SNIFFER_HAS_OLED
#include <U8g2lib.h>
#include <Wire.h>

static U8G2_SSD1306_128X64_NONAME_F_HW_I2C oled_ssd(U8G2_R0, U8X8_PIN_NONE);
static U8G2_SH1106_128X64_NONAME_F_HW_I2C oled_sh(U8G2_R0, U8X8_PIN_NONE);

static U8G2 *g_oled = &oled_sh;
static oled_driver_t g_drv = OLED_DRV_SH1106;

static uint8_t tlm_rot;

const char *ui_driver_name(oled_driver_t drv)
{
    return drv == OLED_DRV_SH1106 ? "sh1106" : "ssd1306";
}

oled_driver_t ui_get_driver()
{
    return g_drv;
}

// Bounded OLED presence check: set up I2C with a transaction timeout, then
// look for an ACK at the panel address (driver-agnostic — an SH1106 answers
// here too, which is exactly why driver selection is a runtime toggle).
// Wire transactions are the only part that can block; u8g2's init is
// write-only.
bool ui_probe()
{
    Wire.begin(PIN_OLED_SDA, PIN_OLED_SCL);
    Wire.setTimeOut(100); // bound any single transaction to 100 ms
    Wire.beginTransmission(OLED_I2C_ADDR);
    return Wire.endTransmission() == 0;
}

static void ui_begin(oled_driver_t drv)
{
    g_drv = drv;
    g_oled = (drv == OLED_DRV_SH1106) ? (U8G2 *)&oled_sh : (U8G2 *)&oled_ssd;
    g_oled->begin();
    g_oled->setFont(u8g2_font_5x7_tr);
}

// Distinctive splash with the active driver named in the corner, held 1.5 s
// so it is visible on both boot and manual toggle.
static void ui_splash()
{
    char tag[24];
    snprintf(tag, sizeof(tag), "OLED:%s", ui_driver_name(g_drv));
    int tw = strlen(tag) * 6;

    g_oled->clearBuffer();
    g_oled->drawFrame(0, 0, 128, 64);
    g_oled->drawBox(0, 0, 128, 10); // solid top bar = "driver banner" marker
    g_oled->setDrawColor(2);
    g_oled->drawStr(4, 8, "ELRS SNIFFER");
    g_oled->setDrawColor(1);
    g_oled->drawStr(6, 24, "passive 2.4GHz");
    g_oled->drawStr(6, 36, "sweeping ...");
    g_oled->drawBox(126 - tw - 3, 52, tw + 5, 10);
    g_oled->drawStr(126 - tw - 1, 60, tag);
    g_oled->sendBuffer();
    delay(1500);
}

void ui_start(oled_driver_t drv)
{
    ui_begin(drv);
    ui_splash();
}

oled_driver_t ui_toggle_driver()
{
    ui_begin(g_drv == OLED_DRV_SH1106 ? OLED_DRV_SSD1306 : OLED_DRV_SH1106);
    ui_splash();
    return g_drv;
}

void ui_show_fault(const char *what, const char *detail)
{
    g_oled->clearBuffer();
    g_oled->setDrawColor(2); // inverted
    g_oled->drawBox(0, 0, 128, 11);
    g_oled->setDrawColor(1);
    g_oled->drawStr(4, 9, what);
    g_oled->drawStr(0, 22, detail != NULL ? detail : "");
    g_oled->drawStr(0, 40, "serial JSON active");
    g_oled->drawStr(0, 50, "stats continue 1/s");
    g_oled->sendBuffer();
}

void ui_render(const ui_state_t *st)
{
    char line[40];

    g_oled->clearBuffer();

    // header row: rate + IQ + state
    snprintf(line, sizeof(line), "%s %s", st->rate, st->iq_inverted ? "IQi" : "IQn");
    g_oled->drawStr(0, 8, line);
    const char *state = st->locked ? "LOCK" : "scan";
    int w = g_oled->getStrWidth(state);
    if (st->armed) { // armed: inverted banner, louder than lock state
        g_oled->setDrawColor(2);
        g_oled->drawBox(127 - w - 2, 0, w + 3, 9);
        g_oled->drawStr(128 - w - 1, 8, "ARM");
        g_oled->setDrawColor(1);
    } else {
        g_oled->drawStr(128 - w - 1, 8, state);
    }

    // stats row
    snprintf(line, sizeof(line), "%lupps %ddBm %+ddB", (unsigned long)st->pps,
             (int)st->rssi_dbm, (int)st->snr_db);
    g_oled->drawStr(0, 16, line);

    // freq + LQ row (+ short link uid when known)
    snprintf(line, sizeof(line), "%lu.%03lu LQ%lu.%lu%%",
             (unsigned long)(st->freq_hz / 1000000),
             (unsigned long)((st->freq_hz / 1000) % 1000),
             (unsigned long)(st->lq_permille / 10),
             (unsigned long)(st->lq_permille % 10));
    g_oled->drawStr(0, 24, line);
    // ident is "link <uid6> <rate>" — show the uid token when present
    const char *u = strchr(st->ident, ' ');
    if (u && u[1] && strchr(u + 1, ' ')) {
        char uid7[7] = { 0 };
        strncpy(uid7, u + 1, 6);
        g_oled->drawStr(128 - 37, 24, uid7);
    }

    // stick meters
    for (int i = 0; i < 4; i++) {
        int x = 6 + i * 26, y = 27, wdt = 20, h = 24;
        uint32_t us = st->has_rc ? st->ch_us[i] : 1500;
        int fill = (int)((us - 988) * (h - 2) / (2012 - 988));
        fill = constrain(fill, 0, h - 2);
        g_oled->drawFrame(x, y, wdt, h);
        // center tick at 1500us
        int mid = y + h - 1 - (1500 - 988) * (h - 2) / (2012 - 988);
        g_oled->drawHLine(x + 1, mid, wdt - 2);
        g_oled->drawBox(x + 1, y + h - 1 - fill, wdt - 2, fill);
    }

    // rotating telemetry line (fault banner wins when set)
    if (st->fault[0]) {
        g_oled->setDrawColor(2);
        g_oled->drawBox(0, 54, 128, 10);
        g_oled->setDrawColor(1);
        g_oled->drawStr(2, 62, st->fault);
    } else {
        if (st->tlm[tlm_rot][0] == 0) tlm_rot = 0;
        g_oled->drawStr(0, 62, st->tlm[tlm_rot]);
        tlm_rot = (tlm_rot + 1) % UI_TLM_LINES;
    }

    g_oled->sendBuffer();
}
#else // SNIFFER_HAS_OLED
bool ui_probe() { return false; }
void ui_start(oled_driver_t) {}
void ui_show_fault(const char *, const char *) {}
void ui_render(const ui_state_t *) {}
oled_driver_t ui_toggle_driver() { return OLED_DRV_SH1106; }
oled_driver_t ui_get_driver() { return OLED_DRV_SH1106; }
const char *ui_driver_name(oled_driver_t) { return "none"; }
#endif
#endif // ARDUINO
