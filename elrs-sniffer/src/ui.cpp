// ui.cpp — onboard SSD1306 128x64 OLED, OUI-SPY dark theme (see ui.h).
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

// U8G2_SSD1306_128X64_NONAME_F_HW_I2C — full-buffer, HW I2C; same
// constructor as LilyGo's own factory sketch (utilities.h DISPLAY_MODEL).
static U8G2_SSD1306_128X64_NONAME_F_HW_I2C oled(U8G2_R0, U8X8_PIN_NONE);

static uint8_t tlm_rot;

void ui_init()
{
    Wire.begin(PIN_OLED_SDA, PIN_OLED_SCL);
    oled.begin();
    oled.setFont(u8g2_font_5x7_tr);
    oled.clearBuffer();
    oled.drawStr(6, 20, "ELRS SNIFFER");
    oled.drawStr(6, 32, "passive 2.4GHz");
    oled.drawStr(6, 44, "sweeping ...");
    oled.sendBuffer();
}

void ui_render(const ui_state_t *st)
{
    char line[40];

    oled.clearBuffer();

    // header row: rate + IQ + state
    snprintf(line, sizeof(line), "%s %s", st->rate, st->iq_inverted ? "IQi" : "IQn");
    oled.drawStr(0, 8, line);
    const char *state = st->locked ? "LOCK" : "scan";
    int w = oled.getStrWidth(state);
    if (st->armed) { // armed: inverted banner, louder than lock state
        oled.setDrawColor(2);
        oled.drawBox(127 - w - 2, 0, w + 3, 9);
        oled.drawStr(128 - w - 1, 8, "ARM");
        oled.setDrawColor(1);
    } else {
        oled.drawStr(128 - w - 1, 8, state);
    }

    // stats row
    snprintf(line, sizeof(line), "%lupps %ddBm %+ddB", (unsigned long)st->pps,
             (int)st->rssi_dbm, (int)st->snr_db);
    oled.drawStr(0, 16, line);

    // freq + LQ row (+ short link uid when known)
    snprintf(line, sizeof(line), "%lu.%03lu LQ%lu.%lu%%",
             (unsigned long)(st->freq_hz / 1000000),
             (unsigned long)((st->freq_hz / 1000) % 1000),
             (unsigned long)(st->lq_permille / 10),
             (unsigned long)(st->lq_permille % 10));
    oled.drawStr(0, 24, line);
    // ident is "link <uid6> <rate>" — show the uid token when present
    const char *u = strchr(st->ident, ' ');
    if (u && u[1] && strchr(u + 1, ' ')) {
        char uid7[7] = { 0 };
        strncpy(uid7, u + 1, 6);
        oled.drawStr(128 - 37, 24, uid7);
    }

    // stick meters
    for (int i = 0; i < 4; i++) {
        int x = 6 + i * 26, y = 27, wdt = 20, h = 24;
        uint32_t us = st->has_rc ? st->ch_us[i] : 1500;
        int fill = (int)((us - 988) * (h - 2) / (2012 - 988));
        fill = constrain(fill, 0, h - 2);
        oled.drawFrame(x, y, wdt, h);
        // center tick at 1500us
        int mid = y + h - 1 - (1500 - 988) * (h - 2) / (2012 - 988);
        oled.drawHLine(x + 1, mid, wdt - 2);
        oled.drawBox(x + 1, y + h - 1 - fill, wdt - 2, fill);
    }

    // rotating telemetry line
    if (st->tlm[tlm_rot][0] == 0) tlm_rot = 0;
    oled.drawStr(0, 62, st->tlm[tlm_rot]);
    tlm_rot = (tlm_rot + 1) % UI_TLM_LINES;

    oled.sendBuffer();
}
#else // SNIFFER_HAS_OLED
void ui_init() {}
void ui_render(const ui_state_t *) {}
#endif
#endif // ARDUINO
