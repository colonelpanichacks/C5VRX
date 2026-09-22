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
#define DWELL_MIN_MS 2000u        // initial dwell per rate (sync can be seconds apart)
#define DWELL_CHUNK_MS 2000u      // adaptive dwell grows in these steps...
#define DWELL_MAX_MS 35000u       // ...up to this cap (connected sync can be slow)
#define EXTEND_RSSI_DB -85        // hot dwells (or any packets) extend
#define AWAIT_PKT_MS 3000u        // packets-for-this-long-without-sync = patient
#define AWAIT_LOG_MS 5000u        // awaiting_sync heartbeat interval
#define DWELL_WATCHDOG_GRACE_MS 2000u // wedged dwell -> restart + advance
#define RSSI_SAMPLE_MS 200u        // <=5 Hz live energy sampling
#define RSSI_MAX_FAILS 3           // consecutive SPI errors -> stop for this dwell
#define PKT_DEBUG_MIN_MS 500u      // sync-first debug line rate limit (2/s)
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
static uint32_t window_pkts;      // CRC-OK packets this second (stats.pps)
static uint32_t window_rx;        // raw RxDone this second (stats.rx_per_s)
static float last_rssi = -128.0f;
static float last_snr = 0.0f;
static uint32_t lock_total_pkts;

// packet-path instrumentation: cumulative (since boot) + per-dwell.
// rx = RxDone demods; crc_ok = passed ELRS software CRC; types = parses.
static uint32_t n_rx, n_crc_ok, n_rc, n_msp, n_sync, n_tlm, n_unk;
static uint32_t dwell_rx, dwell_crc_ok, dwell_rc, dwell_msp, dwell_sync, dwell_tlm, dwell_unk;
static uint32_t last_pkt_debug_ms;      // crc-ok pkt hex (2/s)
static uint32_t last_rawpkt_ms;         // sync-classified raw hex (2/s)

// live energy (GET_RSSIINST sampling): per-dwell max, per-stats-window max,
// and last sample. -128 = no sample yet (radio faulted) — with a working RF
// front end the noise floor itself reads ~ -120, so these are discriminators.
static float dwell_rssi_max = -128.0f;
static float window_rssi_max = -128.0f;
static float rssi_now = -128.0f;
static uint32_t last_sample_ms;
static uint32_t dwell_len_ms;      // current adaptive dwell length
static uint32_t dwell_pkts;        // classified packets this dwell
static uint32_t dwell_first_pkt_ms;
static uint32_t last_await_log_ms;
static uint8_t last_good_step;     // re-enter here after a lock drops
static bool sampling_enabled;   // per-dwell; cleared after RSSI_MAX_FAILS errors
static uint8_t sample_fails;
static bool g_rx_busy;          // read_packet SPI in progress — don't sample

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
    dwell_len_ms = DWELL_MIN_MS;
    dwell_pkts = 0;
    dwell_first_pkt_ms = 0;
    dwell_rx = dwell_crc_ok = dwell_rc = dwell_msp = dwell_sync = dwell_tlm = dwell_unk = 0;
    sampling_enabled = true;
    sample_fails = 0;
    snprintf(uist.rate, sizeof(uist.rate), "%s", s.rate->name);
    uist.iq_inverted = s.iq_inverted;
    uist.freq_hz = ELRS_2G4_SYNC_FREQ_HZ;
}

static const char *pkt_type_name(uint8_t t)
{
    switch (t) {
    case ELRS_PKT_RCDATA: return "rc";
    case ELRS_PKT_SYNC:   return "sync";
    case ELRS_PKT_TLM:    return "tlm";
    default:              return "msp";
    }
}

static void to_hex(const uint8_t *d, size_t n, char *out)
{
    static const char h[] = "0123456789abcdef";
    for (size_t i = 0; i < n; i++) {
        out[2 * i] = h[d[i] >> 4];
        out[2 * i + 1] = h[d[i] & 0x0F];
    }
    out[2 * n] = 0;
}

// End the current dwell: print its energy + packet fingerprint and advance.
static void dwell_advance()
{
    const sniffer_step_t &s = sweep.steps[step_idx];
    Serial.printf("{\"t\":\"dwell\",\"step\":%u,\"rate\":\"%s\",\"iq\":\"%c\",\"legacy\":%u,"
                  "\"rssi_max\":%d,\"rx\":%lu,\"crc_ok\":%lu,"
                  "\"types\":{\"rc\":%lu,\"msp\":%lu,\"sync\":%lu,\"tlm\":%lu,\"unk\":%lu}}\n",
                  step_idx, s.rate->name, s.iq_inverted ? 'i' : 'n',
                  s.legacy_2x ? 1 : 0, (int)dwell_rssi_max,
                  (unsigned long)dwell_rx, (unsigned long)dwell_crc_ok,
                  (unsigned long)dwell_rc, (unsigned long)dwell_msp,
                  (unsigned long)dwell_sync, (unsigned long)dwell_tlm,
                  (unsigned long)dwell_unk);
    dwell_begin(step_idx + 1);
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
    last_good_step = step_idx; // sweep re-enters here first after a drop
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

    uist.pps = window_pkts * 1000 / dt;   // CRC-OK per second
    uint32_t rx_per_s = window_rx * 1000 / dt;
    window_pkts = 0;
    window_rx = 0;

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
                  "\"rssi_now\":%d,\"snr10\":%d,\"pps\":%lu,\"rx_per_s\":%lu,"
                  "\"lq_permille\":%lu,\"lock\":%u,\"radio\":%u,"
                  "\"rx\":%lu,\"crc_ok\":%lu,"
                  "\"types\":{\"rc\":%lu,\"msp\":%lu,\"sync\":%lu,\"tlm\":%lu,\"unk\":%lu},"
                  "\"ch\":[%lu,%lu,%lu,%lu],\"arm\":%u",
                  (unsigned long)now, uist.rate, uist.iq_inverted ? 'i' : 'n',
                  (int)uist.rssi_dbm, (int)rssi_now, (int)(last_snr * 10),
                  (unsigned long)uist.pps, (unsigned long)rx_per_s,
                  (unsigned long)uist.lq_permille,
                  locked ? 1 : 0, radio_ok ? 1 : 0,
                  (unsigned long)n_rx, (unsigned long)n_crc_ok,
                  (unsigned long)n_rc, (unsigned long)n_msp,
                  (unsigned long)n_sync, (unsigned long)n_tlm, (unsigned long)n_unk,
                  (unsigned long)uist.ch_us[0], (unsigned long)uist.ch_us[1],
                  (unsigned long)uist.ch_us[2], (unsigned long)uist.ch_us[3],
                  uist.armed ? 1 : 0);
    if (dctx.uid_known) {
        Serial.printf(",\"uid\":\"%02x%02x%02x\"", dctx.uid3, dctx.uid4, dctx.uid5);
    }
    Serial.println("}");

    if (oled_ok) ui_render(&uist);
}

// ---- LED marker (GPIO37 onboard LED, active HIGH) --------------------------
// STRICTLY receive-activity (user requirement, flash.md table):
//   boot:      ON solid 1 s = "app started" (one-time, pre-loop marker)
//   sweep:     OFF while pps == 0 (no heartbeat)
//   packets:   blink at ~min(pps,5) Hz
//   locked:    solid ON
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

static void led_update(bool locked, uint32_t pps)
{
    if (locked) {
        digitalWrite(PIN_BOARD_LED, HIGH);
    } else if (pps == 0) {
        digitalWrite(PIN_BOARD_LED, LOW);
    } else {
        uint32_t hz = pps > 5 ? 5 : pps;
        uint32_t period = 1000 / hz;
        digitalWrite(PIN_BOARD_LED, (millis() % period) < (period / 2) ? HIGH : LOW);
    }
}
#else
static void led_boot_marker_start() {}
static void led_boot_marker_done(uint32_t) {}
static void led_update(bool, uint32_t) {}
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
static bool g_verbose = false;      // V: print every packet as rawpkt (10/s)
static int g_park_step = -1;        // R <step>: park sweep; -1 = auto-sweep

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
    // console commands: P = radio pin re-probe, D = toggle OLED driver,
    // V = verbose rawpkt toggle, R [step] = park sweep / resume
    static bool r_pending = false;
    static uint8_t r_digits = 0;
    static int r_num = 0;
    while (Serial.available()) {
        int c = Serial.read();
        if (r_pending) {
            if (c >= '0' && c <= '9' && r_digits < 2) {
                r_num = r_num * 10 + (c - '0');
                r_digits++;
                continue;
            }
            if (c == ' ') continue; // "R 1" — spaces ignored while pending
            // finalize the R command
            if (r_digits == 0) {
                g_park_step = -1;
                Serial.println("{\"t\":\"event\",\"what\":\"sweep_resume\"}");
            } else {
                g_park_step = r_num % sweep.count;
                const sniffer_step_t &s = sweep.steps[g_park_step];
                Serial.printf("{\"t\":\"event\",\"what\":\"parked\",\"step\":%d,\"rate\":\"%s\",\"iq\":\"%c\"}\n",
                              g_park_step, s.rate->name, s.iq_inverted ? 'i' : 'n');
                if (!locked) dwell_begin((uint8_t)g_park_step);
            }
            r_pending = false; r_num = 0; r_digits = 0;
            continue;
        }
        if (c == 'P' || c == 'p') g_force_reprobe = true;
        else if (c == 'D' || c == 'd') toggle_oled_driver();
        else if (c == 'V' || c == 'v') {
            g_verbose = !g_verbose;
            Serial.printf("{\"t\":\"event\",\"what\":\"%s\"}\n", g_verbose ? "verbose_on" : "verbose_off");
        } else if (c == 'R' || c == 'r') {
            r_pending = true; r_num = 0; r_digits = 0;
        }
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
        // live energy sampling (<=5 Hz) — the RF-path discriminator.
        // Guarded: only when RX is running and no packet SPI is in flight;
        // three consecutive SPI errors stop sampling for THIS dwell (the
        // rssi values stay stale) instead of hammering a wedged radio.
        if (sampling_enabled && !g_rx_busy && !locked &&
            millis() - last_sample_ms >= RSSI_SAMPLE_MS) {
            last_sample_ms = millis();
            float db;
            if (g_radio.rssiInst(db) == RADIOLIB_ERR_NONE) {
                sample_fails = 0;
                rssi_now = db;
                if (db > window_rssi_max) window_rssi_max = db;
                if (db > dwell_rssi_max) dwell_rssi_max = db;
            } else if (++sample_fails >= RSSI_MAX_FAILS) {
                sampling_enabled = false;
            }
        }

        uint8_t buf[ELRS_OTA8_LEN];
        float rssi, snr;
        size_t want = sweep.steps[step_idx].rate->payload;

        g_rx_busy = true;
        bool got = g_radio.read_packet(buf, want, rssi, snr);
        g_rx_busy = false;
        if (got) {
            n_rx++; dwell_rx++; window_rx++;
            elrs_packet_t pkt;
            bool ok = elrs_decode_packet(&dctx, buf, want, &pkt);
            if (rssi > last_rssi) last_rssi = rssi; // peak-hold for the stats window
            last_snr = snr;
            if (!ok) {
                n_unk++; dwell_unk++;
            } else {
                last_pkt_ms = millis();
                dwell_pkts++;
                if (!dwell_first_pkt_ms) dwell_first_pkt_ms = millis();
                // verbose (V): EVERY demodded packet as rawpkt, 10/s
                if (g_verbose && millis() - last_rawpkt_ms >= 100) {
                    last_rawpkt_ms = millis();
                    char hex[2 * ELRS_OTA8_LEN + 1];
                    to_hex(buf, want, hex);
                    Serial.printf("{\"t\":\"rawpkt\",\"cls\":%u,\"hex\":\"%s\"}\n",
                                  (unsigned)pkt.cls, hex);
                }
                switch (pkt.type) {
                case ELRS_PKT_SYNC:
                    n_sync++; dwell_sync++;
                    // raw ground-truth: FULL hex of every sync-classified
                    // packet (validated or not), <=2/s — for CRC forensics.
                    if (millis() - last_rawpkt_ms >= PKT_DEBUG_MIN_MS) {
                        last_rawpkt_ms = millis();
                        char hex[2 * ELRS_OTA8_LEN + 1];
                        to_hex(buf, want, hex);
                        Serial.printf("{\"t\":\"rawpkt\",\"cls\":%u,\"hex\":\"%s\"}\n",
                                      (unsigned)pkt.cls, hex);
                    }
                    on_sync(pkt);
                    break;
                case ELRS_PKT_TLM:
                    n_tlm++; dwell_tlm++; on_tlm(pkt); break;
                case ELRS_PKT_RCDATA:
                    n_rc++; dwell_rc++; on_rc(pkt); break;
                default:
                    n_msp++; dwell_msp++; break;
                }
                if (pkt.cls == ELRS_PKT_CLASS_CRC_OK) {
                    n_crc_ok++; dwell_crc_ok++; window_pkts++;
                    if (locked) lock_total_pkts++;
                    // sync-first debug: SHOW validated packets (2/s cap)
                    if (millis() - last_pkt_debug_ms >= PKT_DEBUG_MIN_MS) {
                        last_pkt_debug_ms = millis();
                        char hex[2 * 16 + 1];
                        size_t hn = want < 16 ? want : 16;
                        to_hex(buf, hn, hex);
                        Serial.printf("{\"t\":\"pkt\",\"type\":\"%s\",\"len\":%u,\"hex\":\"%s\"}\n",
                                      pkt_type_name(pkt.type), (unsigned)want, hex);
                    }
                }
            }
        }

        const sniffer_step_t &cur = sweep.steps[step_idx];
        if (!locked) {
            // adaptive dwell: hot (or packet-bearing) dwells extend in
            // 2 s chunks up to the cap — a sync can be up to ~16 s away.
            if (millis() - step_entered_ms > dwell_len_ms) {
                bool active = dwell_rssi_max > EXTEND_RSSI_DB || dwell_pkts > 0;
                if (active && dwell_len_ms < DWELL_MAX_MS) {
                    dwell_len_ms += DWELL_CHUNK_MS;
                    Serial.printf("{\"t\":\"dwell_ext\",\"rate\":\"%s\",\"iq\":\"%c\",\"rssi_max\":%d,\"dwell_ms\":%u}\n",
                                  cur.rate->name, cur.iq_inverted ? 'i' : 'n',
                                  (int)dwell_rssi_max, dwell_len_ms);
                } else if (g_park_step < 0) {
                    dwell_advance(); // parked (R <step>): stay seated
                }
            }
            // patience heartbeat: packets flowing for >3 s, no sync yet
            if (dwell_pkts > 0 && dwell_first_pkt_ms &&
                millis() - dwell_first_pkt_ms > AWAIT_PKT_MS &&
                millis() - last_pkt_ms < 1500 &&
                millis() - last_await_log_ms >= AWAIT_LOG_MS) {
                last_await_log_ms = millis();
                Serial.printf("{\"t\":\"event\",\"what\":\"awaiting_sync\",\"rate\":\"%s\",\"iq\":\"%c\"}\n",
                              cur.rate->name, cur.iq_inverted ? 'i' : 'n');
            }
        }
        // dwell watchdog: past cap+grace with no progress -> restart + move on
        if (!locked && radio_ok && millis() - step_entered_ms > dwell_len_ms + DWELL_WATCHDOG_GRACE_MS) {
            Serial.printf("{\"t\":\"error\",\"what\":\"dwell_timeout\",\"rate\":\"%s\",\"iq\":\"%c\",\"ms\":%lu}\n",
                          cur.rate->name, cur.iq_inverted ? 'i' : 'n',
                          (unsigned long)(millis() - step_entered_ms));
            g_radio.recover(cur, ELRS_2G4_SYNC_FREQ_HZ);
            dwell_advance();
        }
        if (locked && millis() - last_pkt_ms > 5000) {
            locked = false;
            Serial.println("{\"t\":\"unlock\",\"why\":\"timeout\"}");
            // re-enter sweep: parked step if set, else last good rate+IQ
            dwell_begin(g_park_step >= 0 ? (uint8_t)g_park_step : last_good_step);
        }
    }

    led_update(locked, uist.pps);
    stats_tick(); // always runs — alive-with-no-radio still emits 1 Hz JSON
}
