// =============================================================================
// main.cpp — ELRS sniffer: sweep SX1280 rates on the ELRS sync channel,
// classify packets, decode sticks + telemetry, drive display + USB JSON.
//
// Boot-observability contract (v0.2.2): the very first thing setup() does is
// bring up USB CDC and print a banner — before radio, before display. Every
// init after that is bounded; any failure becomes a JSON error event + a
// fault screen, and the main loop (1 Hz stats JSON) keeps running no matter
// what, so "alive, no packets" and "dead" are always distinguishable.
// =============================================================================
#include <Arduino.h>
#include <Preferences.h>
#include "board_pins.h"
#include "elrs_defs.h"
#include "elrs_parse.h"
#include "sniffer_radio.h"
#include "ui.h"

#define RADIO_INIT_TIMEOUT_MS 15000u
#define RADIO_PROBE_TIMEOUT_MS 5000u
#define RADIO_RETRY_MS 15000u
#define DWELL_MS 1500u             // fixed dwell per rate (sync can be seconds apart)
#define REDWELL_RSSI_DB -80        // dwells hotter than this get one repeat
#define RSSI_SAMPLE_MS 100u        // ~10 Hz live energy sampling
#define PREF_NAMESPACE "elrs-sniffer"

static elrs_decode_ctx_t dctx;
static sniffer_sweep_t sweep;
static uint8_t step_idx;
static bool radio_ok;
static bool oled_ok;
static bool locked;

static uint32_t step_entered_ms;
static uint32_t last_pkt_ms;
static uint32_t last_stats_ms;
static uint32_t window_pkts;      // ELRS-classified packets this second
static float last_rssi = -128.0f;
static float last_snr = 0.0f;
static uint32_t lock_total_pkts;

// live energy (GET_RSSIINST sampling): per-dwell max, per-stats-window max,
// and last sample. -128 = no sample yet (radio faulted) — with a working RF
// front end the noise floor itself reads ~ -120, so these are discriminators.
static float dwell_rssi_max = -128.0f;
static float window_rssi_max = -128.0f;
static float rssi_now = -128.0f;
static uint32_t last_sample_ms;
static bool redwell_scheduled;

static ui_state_t uist;

// Configure the radio for a sweep step. The "dwell" event is emitted when a
// dwell ENDS (dwell_advance), carrying that dwell's rssi_max — so the serial
// log prints an energy fingerprint per rate as the sweep runs.
static void dwell_begin(uint8_t i)
{
    step_idx = i % sweep.count;
    const sniffer_step_t &s = sweep.steps[step_idx];
    g_radio.apply(s, ELRS_2G4_SYNC_FREQ_HZ);
    g_radio.start_rx();
    step_entered_ms = millis();
    dwell_rssi_max = -128.0f;
    snprintf(uist.rate, sizeof(uist.rate), "%s", s.rate->name);
    uist.iq_inverted = s.iq_inverted;
    uist.freq_hz = ELRS_2G4_SYNC_FREQ_HZ;
}

// End the current dwell: print its energy fingerprint, then either re-dwell
// the same rate once (it showed energy — catch the sync window) or advance.
static void dwell_advance()
{
    const sniffer_step_t &s = sweep.steps[step_idx];
    Serial.printf("{\"t\":\"dwell\",\"step\":%u,\"rate\":\"%s\",\"iq\":\"%c\",\"legacy\":%u,"
                  "\"rssi_max\":%d}\n",
                  step_idx, s.rate->name, s.iq_inverted ? 'i' : 'n',
                  s.legacy_2x ? 1 : 0, (int)dwell_rssi_max);
    if (dwell_rssi_max > REDWELL_RSSI_DB && !redwell_scheduled) {
        redwell_scheduled = true;   // hot dwell: listen once more, same rate
        dwell_begin(step_idx);
    } else {
        redwell_scheduled = false;
        dwell_begin(step_idx + 1);
    }
}

// jump the sweep straight to a rate index heard in a sync packet
static void goto_rate(uint8_t rate_index, bool iq_inverted)
{
    for (uint8_t pass = 0; pass < 2; pass++) {
        for (uint8_t i = 0; i < sweep.count; i++) {
            const sniffer_step_t &s = sweep.steps[i];
            bool iq_match = s.iq_inverted == (pass == 0 ? iq_inverted : !iq_inverted);
            if (!s.legacy_2x && s.rate->rate_index == rate_index && iq_match) {
                dwell_begin(i);
                return;
            }
        }
    }
}

static void on_sync(const elrs_packet_t &pkt)
{
    bool good = pkt.cls == ELRS_PKT_CLASS_CRC_OK;
    Serial.printf("{\"t\":\"sync\",\"ok\":%u,\"fhss\":%u,\"nonce\":%u,\"rateIdx\":%u,"
                  "\"swMode\":%u,\"tlmRatio\":%u,\"uid\":\"%02x%02x%02x\"}\n",
                  good ? 1 : 0, pkt.sync.fhss_index, pkt.sync.nonce,
                  pkt.sync.rate_index, pkt.sync.switch_mode, pkt.sync.tlm_ratio,
                  pkt.sync.uid3, pkt.sync.uid4, pkt.sync.uid5);
    if (!good) return;
    if (!locked) {
        locked = true;
        lock_total_pkts = 0;
        Serial.println("{\"t\":\"lock\"}");
    }
    snprintf(uist.ident, sizeof(uist.ident), "link %02x%02x%02x %s",
             pkt.sync.uid3, pkt.sync.uid4, pkt.sync.uid5, uist.rate);
    last_pkt_ms = millis();
    if (pkt.sync.rate_index <= 9) goto_rate(pkt.sync.rate_index, uist.iq_inverted);
}

static void on_tlm(const elrs_packet_t &pkt)
{
    const elrs_telemetry_t *tm = &pkt.tlm;
    if (pkt.linkstats.valid) {
        const elrs_linkstats_t *ls = &pkt.linkstats;
        uist.lq_permille = ls->lq * 10;
        snprintf(uist.tlm[0], sizeof(uist.tlm[0]), "LQ %3u  rssi -%u/%u dBm  %+d dB",
                 ls->lq, ls->rssi1_db, ls->rssi2_db, ls->snr_db);
        Serial.printf("{\"t\":\"linkstats\",\"lq\":%u,\"rssi1\":-%u,\"rssi2\":-%u,\"snr\":%d}\n",
                      ls->lq, ls->rssi1_db, ls->rssi2_db, ls->snr_db);
    }
    if (tm->valid) {
        switch (tm->type) {
        case CRSF_FRAMETYPE_GPS:
            snprintf(uist.tlm[1], sizeof(uist.tlm[1]), "GPS %ld,%ld sats %u",
                     (long)(tm->lat_e7 / 10000000), (long)(tm->lon_e7 / 10000000), tm->sats);
            Serial.printf("{\"t\":\"gps\",\"lat_e7\":%ld,\"lon_e7\":%ld,\"spd_kmh10\":%u,\"sats\":%u}\n",
                          (long)tm->lat_e7, (long)tm->lon_e7, tm->gspeed_kmh10, tm->sats);
            break;
        case CRSF_FRAMETYPE_BATTERY:
            snprintf(uist.tlm[1], sizeof(uist.tlm[1]), "BAT %u.%uV %u.%uA",
                     tm->batt_mv10 / 10, tm->batt_mv10 % 10, tm->batt_ma10 / 10, tm->batt_ma10 % 10);
            Serial.printf("{\"t\":\"batt\",\"v10\":%u,\"a10\":%u,\"mah\":%lu}\n",
                          tm->batt_mv10, tm->batt_ma10, (unsigned long)tm->batt_mah);
            break;
        case CRSF_FRAMETYPE_ATTITUDE:
            snprintf(uist.tlm[2], sizeof(uist.tlm[2]), "ATT p%dr%d y%d",
                     tm->pitch / 100, tm->roll / 100, tm->yaw / 100);
            Serial.printf("{\"t\":\"atti\",\"p\":%d,\"r\":%d,\"y\":%d}\n", tm->pitch, tm->roll, tm->yaw);
            break;
        case CRSF_FRAMETYPE_FLIGHT_MODE:
            snprintf(uist.tlm[2], sizeof(uist.tlm[2]), "FM %s", tm->flight_mode);
            Serial.printf("{\"t\":\"fm\",\"m\":\"%s\"}\n", tm->flight_mode);
            break;
        default:
            Serial.printf("{\"t\":\"tlm\",\"ft\":%u}\n", tm->type);
            break;
        }
    }
}

static void on_rc(const elrs_packet_t &pkt)
{
    uist.has_rc = true;
    for (int i = 0; i < 4; i++) {
        if (pkt.rc.has_ch[i]) {
            uist.ch_us[i] = elrs_crsfval_to_us(pkt.rc.ch[i]);
        }
    }
    if (pkt.rc.has_ch[4]) {
        uist.armed = pkt.rc.ch[4] > (CRSF_VAL_MIN + CRSF_VAL_MAX) / 2;
    }
}

static void stats_tick()
{
    uint32_t now = millis();
    uint32_t dt = now - last_stats_ms;
    if (dt < 1000) return;
    last_stats_ms = now;

    uist.pps = window_pkts * 1000 / dt;
    window_pkts = 0;

    // observed-vs-expected ratio while locked (the only time it means anything)
    if (locked) {
        const elrs_rate_t *r = sweep.steps[step_idx].rate;
        uint32_t expected = 1000000 / r->interval_us;
        uint32_t lq = expected ? (uint32_t)uist.pps * 1000 / expected : 0;
        uist.lq_permille = lq > 1000 ? 1000 : lq;
    }
    uist.rssi_dbm = window_rssi_max;
    uist.snr_db = last_snr;
    uist.locked = locked;
    uist.radio_ok = radio_ok;
    if (!radio_ok) {
        snprintf(uist.fault, sizeof(uist.fault), "RADIO FAULT");
    } else if (!oled_ok) {
        uist.fault[0] = 0; // radio fine, no display: nothing to draw on anyway
    } else {
        uist.fault[0] = 0;
    }
    last_rssi = -128.0f; // reset peak-hold for the next window
    window_rssi_max = -128.0f;

    Serial.printf("{\"t\":\"stats\",\"ms\":%lu,\"rate\":\"%s\",\"iq\":\"%c\",\"rssi\":%d,"
                  "\"rssi_now\":%d,\"snr10\":%d,\"pps\":%lu,\"lq_permille\":%lu,\"lock\":%u,\"radio\":%u,"
                  "\"ch\":[%lu,%lu,%lu,%lu],\"arm\":%u",
                  (unsigned long)now, uist.rate, uist.iq_inverted ? 'i' : 'n',
                  (int)uist.rssi_dbm, (int)rssi_now, (int)(last_snr * 10),
                  (unsigned long)uist.pps,
                  (unsigned long)uist.lq_permille,
                  locked ? 1 : 0, radio_ok ? 1 : 0,
                  (unsigned long)uist.ch_us[0], (unsigned long)uist.ch_us[1],
                  (unsigned long)uist.ch_us[2], (unsigned long)uist.ch_us[3],
                  uist.armed ? 1 : 0);
    if (dctx.uid_known) {
        Serial.printf(",\"uid\":\"%02x%02x%02x\"", dctx.uid3, dctx.uid4, dctx.uid5);
    }
    Serial.println("}");

    if (oled_ok) ui_render(&uist);
}

// ---- LED boot/fault marker (GPIO37 onboard LED, active HIGH) -------------
// Visible without serial. Pattern (also documented in flash.md):
//   setup entry: ON solid 1 s ("app started") -> OFF
//   radio init:  200 ms blink while waiting
//   loop:        radio fault = 50 ms fast blink forever;
//                locked = solid ON; unlocked = 1 Hz heartbeat (50 ms ON/s)
#if defined(PIN_BOARD_LED)
static void led_boot_marker_start()
{
    pinMode(PIN_BOARD_LED, OUTPUT);
    digitalWrite(PIN_BOARD_LED, HIGH);
}

static void led_boot_marker_done(uint32_t started_ms)
{
    uint32_t elapsed = millis() - started_ms;
    if (elapsed < 1000) delay(1000 - elapsed);
    digitalWrite(PIN_BOARD_LED, LOW);
}

static void led_update(bool radio_ok, bool locked)
{
    if (!radio_ok) {
        digitalWrite(PIN_BOARD_LED, (millis() % 100) < 50 ? HIGH : LOW);
    } else if (locked) {
        digitalWrite(PIN_BOARD_LED, HIGH);
    } else {
        digitalWrite(PIN_BOARD_LED, (millis() % 1000) < 50 ? HIGH : LOW);
    }
}
#else
static void led_boot_marker_start() {}
static void led_boot_marker_done(uint32_t) {}
static void led_update(bool, bool) {}
#endif

// ---- bounded radio init + pin auto-probe -----------------------------------
// RadioLib 6.6 already bounds each SPI transaction (1 s timeout), but run the
// whole init pinned to core 0 behind a wall-clock bound anyway: if anything
// pathological stalls, the main loop on core 1 keeps emitting stats. A
// stalled task may remain on core 0 — harmless (nothing else lives there).
static volatile bool g_radio_task_done;
static volatile int16_t g_radio_task_result;
static const radio_pin_set_t *g_task_pin_set;

static void radio_init_task(void *)
{
    g_radio_task_result = g_radio.begin(g_task_pin_set);
    g_radio_task_done = true;
    vTaskDelete(NULL);
}

// Returns true only on RADIOLIB_ERR_NONE. Polarity matters: begin() returns
// a RadioLib status (0 = success); the previous bool-shaped path reported a
// WORKING radio as "begin err 1" and a dead one as ok. Don't reintroduce.
static bool radio_init_bounded(const radio_pin_set_t *ps, uint32_t timeout_ms,
                               char *err, size_t errlen)
{
    g_task_pin_set = ps;
    g_radio_task_done = false;
    if (xTaskCreatePinnedToCore(radio_init_task, "radio_init", 4096, NULL, 1,
                                NULL, 0) != pdPASS) {
        g_radio_task_result = g_radio.begin(ps); // fallback: RadioLib's own bounds
        g_radio_task_done = true;
    }
    uint32_t t0 = millis();
    uint32_t last_blink = 0;
    bool led_state = false;
    while (!g_radio_task_done && millis() - t0 < timeout_ms) {
        delay(10);
#if defined(PIN_BOARD_LED)
        if (millis() - last_blink >= 200) { // init-in-progress blink
            last_blink = millis();
            led_state = !led_state;
            digitalWrite(PIN_BOARD_LED, led_state ? HIGH : LOW);
        }
#endif
    }
    if (!g_radio_task_done) {
        snprintf(err, errlen, "timeout %lus (no SX1280 ACK)", timeout_ms / 1000);
        return false; // init task still spinning on core 0; left to die there
    }
    if (g_radio_task_result != RADIOLIB_ERR_NONE) {
        snprintf(err, errlen, "RadioLib begin err %d", (int)g_radio_task_result);
        return false;
    }
    return true;
}

// ---- OLED driver preference ------------------------------------------------
static int oled_pref_read()
{
    Preferences pr;
    if (!pr.begin(PREF_NAMESPACE, true)) return OLED_DRV_SH1106; // default
    int v = pr.getUChar("oled_drv", OLED_DRV_SH1106);
    pr.end();
    return (v == OLED_DRV_SSD1306 || v == OLED_DRV_SH1106) ? v : OLED_DRV_SH1106;
}

static void oled_pref_write(int drv)
{
    Preferences pr;
    if (pr.begin(PREF_NAMESPACE, false)) {
        pr.putUChar("oled_drv", (uint8_t)drv);
        pr.end();
    }
}

// Toggle the panel driver (serial 'D' / long-press BOOT), persist, report.
static void toggle_oled_driver()
{
    if (!oled_ok) return;
    oled_driver_t d = ui_toggle_driver(); // re-init + 1.5 s splash
    oled_pref_write(d);
    Serial.printf("{\"t\":\"event\",\"what\":\"oled_driver\",\"drv\":\"%s\"}\n",
                  ui_driver_name(d));
}

// ---- pin-set cache (Preferences/NVS) --------------------------------------
static int pin_cache_read()
{
    Preferences pr;
    if (!pr.begin(PREF_NAMESPACE, true)) return -1;
    int v = pr.getUChar("pinset", 0);
    pr.end();
    return (v >= 1 && v <= (int)RADIO_PIN_SET_COUNT) ? v - 1 : -1;
}

static void pin_cache_write(int idx)
{
    Preferences pr;
    if (pr.begin(PREF_NAMESPACE, false)) {
        pr.putUChar("pinset", (uint8_t)(idx + 1));
        pr.end();
    }
}

static void fmt_pins(const radio_pin_set_t *ps, char *buf, size_t n)
{
    snprintf(buf, n, "NSS=%d SCK=%d MISO=%d MOSI=%d RST=%d DIO1=%d BUSY=%d",
             ps->nss, ps->sck, ps->miso, ps->mosi, ps->rst, ps->dio1, ps->busy);
}

// Try one pin set, emit a probe event, return success.
static bool try_pin_set(int idx, uint32_t timeout_ms, bool cached, char *err, size_t errlen)
{
    const radio_pin_set_t *ps = &RADIO_PIN_SETS[idx];
    char pinbuf[72];
    fmt_pins(ps, pinbuf, sizeof(pinbuf));
    bool ok = radio_init_bounded(ps, timeout_ms, err, errlen);
    Serial.printf("{\"t\":\"probe\",\"set\":\"%s\",\"pins\":\"%s\",\"ok\":%u%s}\n",
                  ps->name, pinbuf, ok ? 1 : 0, cached ? ",\"cached\":1" : "");
    return ok;
}

// Probe all sets starting with `first_idx`; on success cache + return the set.
static const radio_pin_set_t *probe_pin_sets(int first_idx, uint32_t timeout_ms,
                                             bool cached_first, char *err, size_t errlen)
{
    for (uint8_t k = 0; k < RADIO_PIN_SET_COUNT; k++) {
        int idx = (first_idx + k) % RADIO_PIN_SET_COUNT;
        if (try_pin_set(idx, timeout_ms, cached_first && k == 0, err, errlen)) {
            pin_cache_write(idx);
            return &RADIO_PIN_SETS[idx];
        }
    }
    return NULL;
}

static const radio_pin_set_t *g_working_pins = NULL;
static bool g_force_reprobe = false;

static void report_radio_fault(const char *detail)
{
    char pinbuf[72];
    if (g_working_pins) fmt_pins(g_working_pins, pinbuf, sizeof(pinbuf));
    else snprintf(pinbuf, sizeof(pinbuf), "none");
    Serial.printf("{\"t\":\"error\",\"what\":\"radio_init\",\"detail\":\"%s\",\"pins\":\"%s\"}\n",
                  detail, pinbuf);
    if (oled_ok) ui_show_fault("RADIO FAULT", detail);
    snprintf(uist.rate, sizeof(uist.rate), "rf fault");
}

// ---- boot observability: first serial output, before ANY init ------------
static void early_banner()
{
    Serial.begin(460800);
    uint32_t t0 = millis();
    while (!Serial && millis() - t0 < 1500) delay(10); // brief wait, never block
    Serial.println();
    Serial.printf("elrs-sniffer %s boot | %s\n", ELRS_SNIFFER_VERSION, SNIFFER_BOARD_NAME);
#if PIN_LORA_RXEN != -1
    Serial.println("variant: SX1280-PA (RF switch driven: RXEN=21 TXEN=10)");
#else
    Serial.println("variant: SX1280 non-PA (no RF switch)");
#endif
    Serial.printf("radio pins: NSS=%d SCK=%d MISO=%d MOSI=%d RST=%d DIO1=%d BUSY=%d\n",
                  PIN_LORA_NSS, PIN_LORA_SCK, PIN_LORA_MISO, PIN_LORA_MOSI,
                  PIN_LORA_RST, PIN_LORA_DIO1, PIN_LORA_BUSY);
    Serial.printf("chip: flash %u MB @ %u MHz\n",
                  (unsigned)(ESP.getFlashChipSize() >> 20),
                  (unsigned)(ESP.getFlashChipSpeed() / 1000000));
    Serial.println("passive Rx only - link uid = fingerprint, not identity");
    Serial.printf("{\"t\":\"boot\",\"v\":\"%s\",\"board\":\"%s\",\"flash_mb\":%u,"
                  "\"variant\":\"%s\"}\n", ELRS_SNIFFER_VERSION, SNIFFER_BOARD_NAME,
                  (unsigned)(ESP.getFlashChipSize() >> 20),
#if PIN_LORA_RXEN != -1
                  "sx1280pa");
#else
                  "sx1280");
#endif
    Serial.flush();
}

void setup()
{
    led_boot_marker_start();
    uint32_t boot_mark_t0 = millis();
    early_banner(); // MUST NOT be preceded by anything that can stall
    led_boot_marker_done(boot_mark_t0); // 1 s ON = "app started"

    memset(&uist, 0, sizeof(uist));
    uist.ident[0] = 0;
    snprintf(uist.rate, sizeof(uist.rate), "boot");
    for (int i = 0; i < UI_TLM_LINES; i++) uist.tlm[i][0] = 0;

    elrs_decode_init(&dctx);
    sniffer_sweep_build(&sweep);

    // OLED: bounded probe, splash only on success; driver from prefs
    // (default SH1106 — these panels ship interchangeably and mislabelled)
    int oled_drv = oled_pref_read();
    oled_ok = ui_probe();
    if (oled_ok) {
        ui_start((oled_driver_t)oled_drv);
    } else {
        Serial.printf("{\"t\":\"error\",\"what\":\"oled_init\",\"detail\":\"no ACK at 0x%02x SDA=%d SCL=%d\"}\n",
                      OLED_I2C_ADDR, PIN_OLED_SDA, PIN_OLED_SCL);
    }
#if defined(PIN_BUTTON)
    pinMode(PIN_BUTTON, INPUT_PULLUP); // active low; long-press toggles driver
#endif

    // Radio: pin auto-probe (cached set first), bounded per candidate.
    int cached_idx = pin_cache_read();
    int first_idx = (cached_idx >= 0) ? cached_idx : (int)RADIO_PIN_SET_DEFAULT;
    Serial.printf("radio probe: %u sets, starting at \"%s\"%s\n", RADIO_PIN_SET_COUNT,
                  RADIO_PIN_SETS[first_idx].name, cached_idx >= 0 ? " (cached)" : "");
    static char radio_err[96];
    g_working_pins = probe_pin_sets(first_idx, RADIO_PROBE_TIMEOUT_MS, cached_idx >= 0,
                                    radio_err, sizeof(radio_err));
    radio_ok = (g_working_pins != NULL);
    if (!radio_ok) {
        // last error + the possibility the module isn't an SX1280 at all
        char detail[120];
        snprintf(detail, sizeof(detail), "%s - if all sets fail the module may not be an SX1280 (check the RF can marking; an LR1121 variant needs different firmware)",
                 radio_err);
        report_radio_fault(detail);
    }

    char ready_extra[64] = { 0 };
    if (g_working_pins) {
        int off = snprintf(ready_extra, sizeof(ready_extra), ",\"pins\":\"%s\"",
                           g_working_pins->name);
        if (off > 0 && oled_ok && (size_t)off < sizeof(ready_extra)) {
            snprintf(ready_extra + off, sizeof(ready_extra) - off,
                     ",\"oled_drv\":\"%s\"", ui_driver_name(ui_get_driver()));
        }
    } else if (oled_ok) {
        snprintf(ready_extra, sizeof(ready_extra), ",\"oled_drv\":\"%s\"",
                 ui_driver_name(ui_get_driver()));
    }
    Serial.printf("{\"t\":\"ready\",\"steps\":%u,\"sync_freq\":%lu,\"radio\":%u,\"oled\":%u%s}\n",
                  sweep.count, (unsigned long)(ELRS_2G4_SYNC_FREQ_HZ / 1000000),
                  radio_ok ? 1 : 0, oled_ok ? 1 : 0, ready_extra);
    if (radio_ok) dwell_begin(0);
    last_stats_ms = millis();
}

void loop()
{
    // console commands: P = radio pin re-probe, D = toggle OLED driver
    if (Serial.available()) {
        int c = Serial.read();
        if (c == 'P' || c == 'p') g_force_reprobe = true;
        else if (c == 'D' || c == 'd') toggle_oled_driver();
    }

#if defined(PIN_BUTTON)
    // BOOT button (GPIO0, active low): hold >= 1.5 s toggles the OLED driver.
    // Edge-armed + one-shot per press = debounce by construction.
    static bool btn_last = HIGH, btn_fired = false;
    static uint32_t btn_press_t = 0;
    bool btn = digitalRead(PIN_BUTTON);
    if (btn == LOW) {
        if (btn_last == HIGH) { btn_press_t = millis(); btn_fired = false; }
        else if (!btn_fired && millis() - btn_press_t >= 1500) {
            btn_fired = true;
            toggle_oled_driver();
        }
    }
    btn_last = btn;
#endif

    if (g_force_reprobe) {
        g_force_reprobe = false;
        locked = false;
        radio_ok = false;
        Serial.println("{\"t\":\"probe\",\"info\":\"manual sweep start\"}");
        static char err[96];
        const radio_pin_set_t *ps = probe_pin_sets(0, RADIO_PROBE_TIMEOUT_MS, false,
                                                   err, sizeof(err));
        if (ps) {
            g_working_pins = ps;
            radio_ok = true;
            Serial.println("{\"t\":\"radio_up\"}");
            dwell_begin(0);
        } else {
            report_radio_fault(err);
        }
    }

    // retry a faulted radio — start from the cached/working set again
    static uint32_t last_radio_retry = 0;
    if (!radio_ok && millis() - last_radio_retry > RADIO_RETRY_MS) {
        last_radio_retry = millis();
        static char err[96];
        int start = pin_cache_read();
        if (start < 0) start = (int)RADIO_PIN_SET_DEFAULT;
        const radio_pin_set_t *ps = probe_pin_sets(start, RADIO_PROBE_TIMEOUT_MS,
                                                   start >= 0, err, sizeof(err));
        if (ps) {
            g_working_pins = ps;
            radio_ok = true;
            Serial.println("{\"t\":\"radio_up\"}");
            dwell_begin(0);
        } // stay quiet otherwise: the 1 Hz stats line already says radio:0
    }

    if (radio_ok) {
        // live energy sampling (~10 Hz) — the RF-path discriminator
        if (millis() - last_sample_ms >= RSSI_SAMPLE_MS) {
            last_sample_ms = millis();
            float db;
            if (g_radio.rssiInst(db) == RADIOLIB_ERR_NONE) {
                rssi_now = db;
                if (db > window_rssi_max) window_rssi_max = db;
                if (db > dwell_rssi_max) dwell_rssi_max = db;
            }
        }

        uint8_t buf[ELRS_OTA8_LEN];
        float rssi, snr;
        size_t want = sweep.steps[step_idx].rate->payload;

        if (g_radio.read_packet(buf, want, rssi, snr)) {
            elrs_packet_t pkt;
            bool ok = elrs_decode_packet(&dctx, buf, want, &pkt);
            if (rssi > last_rssi) last_rssi = rssi; // peak-hold for the stats window
            last_snr = snr;
            if (ok) { // classified only: junk must not extend dwells/locks
                last_pkt_ms = millis();
                window_pkts++;
                if (locked) lock_total_pkts++;
                switch (pkt.type) {
                case ELRS_PKT_SYNC: on_sync(pkt); break;
                case ELRS_PKT_TLM:  on_tlm(pkt); break;
                case ELRS_PKT_RCDATA: on_rc(pkt); break;
                default: break; // MSP: counted in pps only
                }
            }
        }

        if (!locked && millis() - step_entered_ms > DWELL_MS) {
            dwell_advance();
        }
        if (locked && millis() - last_pkt_ms > 5000) {
            locked = false;
            Serial.println("{\"t\":\"unlock\",\"why\":\"timeout\"}");
            dwell_advance();
        }
    }

    led_update(radio_ok, locked);
    stats_tick(); // always runs — alive-with-no-radio still emits 1 Hz JSON
}
