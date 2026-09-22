// ui.cpp — onboard display, OUI-SPY dark theme (see ui.h for the palette).
#include "ui.h"
#ifdef ARDUINO
#include "board_pins.h"
#include <Arduino.h>
#if SNIFFER_HAS_TFT
#include <TFT_eSPI.h>

// RGB565 conversions of the OUI-SPY dashboard palette:
static const uint16_t C_BG     = 0x0020; // #050805
static const uint16_t C_PANEL  = 0x0841; // #0a120a
static const uint16_t C_DIM    = 0x1A69; // #33502f (dim green)
static const uint16_t C_TXT    = 0x9EF3; // #9fdc9f
static const uint16_t C_GRN    = 0xE7ED; // #39ff6a
static const uint16_t C_AMB    = 0xF580; // #ffb000
static const uint16_t C_RED    = 0xFA47; // #ff4a3d
static const uint16_t C_CYN    = 0xC65F; // #31c8ff

static TFT_eSPI tft;

void ui_init()
{
    pinMode(PIN_TFT_LED, OUTPUT);
    digitalWrite(PIN_TFT_LED, HIGH);   // board power hold
    tft.init();
    tft.setRotation(0);                // 170 wide x 320 tall portrait
    tft.fillScreen(C_BG);
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
    ledcAttach(PIN_TFT_BL, 5000, 8);
    ledcWrite(PIN_TFT_BL, 255);
#else
    ledcSetup(0, 5000, 8);
    ledcAttachPin(PIN_TFT_BL, 0);
    ledcWrite(0, 255);
#endif
    tft.setTextColor(C_TXT, C_BG);
    tft.setTextDatum(TL_DATUM);
    tft.drawString("ELRS SNIFFER", 6, 6, 4);
    tft.setTextFont(2);
    tft.drawString("sweeping 2.4GHz ...", 6, 30);
}

// vertical stick meter: value 988..2012us -> bar around a center mark
static void stick_bar(int x, int y, int w, int h, uint32_t us, const char *label)
{
    tft.drawRect(x, y, w, h, C_DIM);
    int fill = (int)((us - 988) * (h - 4) / (2012 - 988));
    fill = constrain(fill, 0, h - 4);
    int cy = y + h - 2 - fill;
    tft.fillRect(x + 1, y + 1, w - 2, h - 2, C_PANEL);
    // center line at 1500us
    int mid = y + h - 2 - (int)((1500 - 988) * (h - 4) / (2012 - 988));
    tft.drawFastHLine(x + 1, mid, w - 2, C_DIM);
    tft.fillRect(x + 2, cy, w - 4, y + h - 2 - cy, C_GRN);
    tft.setTextColor(C_TXT, C_BG);
    tft.setTextDatum(BC_DATUM);
    tft.drawString(label, x + w / 2, y - 2);
    tft.setTextDatum(TL_DATUM);
}

void ui_render(const ui_state_t *st)
{
    char line[48];

    // ---- top: stats strip ----
    tft.fillRect(0, 46, 170, 44, C_PANEL);
    tft.setTextFont(2);
    snprintf(line, sizeof(line), "%s%s %s", st->locked ? "" : "~", st->rate,
             st->iq_inverted ? "IQi" : "IQn");
    tft.setTextColor(st->locked ? C_GRN : C_AMB, C_PANEL);
    tft.drawString(line, 4, 50);
    snprintf(line, sizeof(line), "%lu pkt/s  %ddBm  %+.1fdB",
             (unsigned long)st->pps, (int)st->rssi_dbm, (double)st->snr_db);
    tft.setTextColor(C_TXT, C_PANEL);
    tft.drawString(line, 4, 64);
    snprintf(line, sizeof(line), "%lu.%03lu MHz  LQ %lu.%01lu%%",
             (unsigned long)(st->freq_hz / 1000000),
             (unsigned long)((st->freq_hz / 1000) % 1000),
             (unsigned long)(st->lq_permille / 10),
             (unsigned long)(st->lq_permille % 10));
    tft.setTextColor(st->locked ? C_TXT : C_DIM, C_PANEL);
    tft.drawString(line, 4, 78);

    // ---- middle: stick bars ----
    const char *labels[4] = { "R", "P", "T", "Y" };
    for (int i = 0; i < 4; i++) {
        uint32_t us = st->has_rc ? st->ch_us[i] : 1500;
        stick_bar(8 + i * 40, 116, 32, 130, us, labels[i]);
    }
    // arm tick under the bars
    tft.fillRect(8, 252, 152, 12, C_PANEL);
    tft.setTextColor(st->armed ? C_RED : C_DIM, C_PANEL);
    tft.drawString(st->armed ? "ARMED (AUX1 hi)" : "disarmed", 10, 253);

    // ---- bottom: telemetry + identity ----
    tft.fillRect(0, 272, 170, 48, C_PANEL);
    tft.setTextColor(C_DIM, C_PANEL);
    tft.drawString(st->ident, 4, 274);
    for (int i = 0; i < UI_TLM_LINES; i++) {
        tft.setTextColor(i == 0 ? C_CYN : C_TXT, C_PANEL);
        tft.drawString(st->tlm[i], 4, 286 + i * 11);
    }
}
#else // SNIFFER_HAS_TFT
void ui_init() {}
void ui_render(const ui_state_t *) {}
#endif
#endif // ARDUINO
