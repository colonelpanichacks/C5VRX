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
#include "board_pins.h"
#include "elrs_defs.h"
#include "elrs_parse.h"
#include "sniffer_radio.h"
#include "ui.h"

#define RADIO_INIT_TIMEOUT_MS 15000u
#define RADIO_RETRY_MS 15000u

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

static ui_state_t uist;

// dwell: short while blind, stretch once anything ELRS-ish demods
static uint32_t current_dwell_ms()
{
    uint32_t since_pkt = millis() - last_pkt_ms;
    if (since_pkt < 1500) return 4000;   // something is here — listen longer
    return 700;
}

static void apply_step(uint8_t i)
{
    step_idx = i % sweep.count;
    const sniffer_step_t &s = sweep.steps[step_idx];
    g_radio.apply(s, ELRS_2G4_SYNC_FREQ_HZ);
    g_radio.start_rx();
    step_entered_ms = millis();
    snprintf(uist.rate, sizeof(uist.rate), "%s", s.rate->name);
    uist.iq_inverted = s.iq_inverted;
    uist.freq_hz = ELRS_2G4_SYNC_FREQ_HZ;
    Serial.printf("{\"t\":\"dwell\",\"step\":%u,\"rate\":\"%s\",\"iq\":\"%c\",\"legacy\":%u}\n",
                  step_idx, s.rate->name, s.iq_inverted ? 'i' : 'n', s.legacy_2x ? 1 : 0);
}

// jump the sweep straight to a rate index heard in a sync packet
static void goto_rate(uint8_t rate_index, bool iq_inverted)
{
    for (uint8_t pass = 0; pass < 2; pass++) {
        for (uint8_t i = 0; i < sweep.count; i++) {
            const sniffer_step_t &s = sweep.steps[i];
            bool iq_match = s.iq_inverted == (pass == 0 ? iq_inverted : !iq_inverted);
            if (!s.legacy_2x && s.rate->rate_index == rate_index && iq_match) {
                apply_step(i);
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
    uist.rssi_dbm = last_rssi;
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
#if defined(PIN_BOARD_LED)
    digitalWrite(PIN_BOARD_LED, locked ? HIGH : LOW);
#endif

    Serial.printf("{\"t\":\"stats\",\"ms\":%lu,\"rate\":\"%s\",\"iq\":\"%c\",\"rssi\":%d,"
                  "\"snr10\":%d,\"pps\":%lu,\"lq_permille\":%lu,\"lock\":%u,\"radio\":%u,"
                  "\"ch\":[%lu,%lu,%lu,%lu],\"arm\":%u",
                  (unsigned long)now, uist.rate, uist.iq_inverted ? 'i' : 'n',
                  (int)last_rssi, (int)(last_snr * 10), (unsigned long)uist.pps,
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

// ---- bounded radio init --------------------------------------------------
// RadioLib 6.6 already bounds each SPI transaction (1 s timeout), but run the
// whole init pinned to core 0 behind a wall-clock bound anyway: if anything
// pathological stalls, the main loop on core 1 keeps emitting stats. A
// stalled task may remain on core 0 — harmless (nothing else lives there).
static volatile bool g_radio_task_done;
static volatile int g_radio_task_result;

static void radio_init_task(void *)
{
    g_radio_task_result = g_radio.begin();
    g_radio_task_done = true;
    vTaskDelete(NULL);
}

static bool radio_init_bounded(char *err, size_t errlen)
{
    g_radio_task_done = false;
    if (xTaskCreatePinnedToCore(radio_init_task, "radio_init", 4096, NULL, 1,
                                NULL, 0) != pdPASS) {
        g_radio_task_result = g_radio.begin(); // fallback: RadioLib's own bounds
        g_radio_task_done = true;
    }
    uint32_t t0 = millis();
    while (!g_radio_task_done && millis() - t0 < RADIO_INIT_TIMEOUT_MS) delay(10);
    if (!g_radio_task_done) {
        snprintf(err, errlen, "timeout %us (no SX1280 ACK)", RADIO_INIT_TIMEOUT_MS / 1000);
        return false; // init task still spinning on core 0; left to die there
    }
    if (g_radio_task_result != 0) {
        snprintf(err, errlen, "RadioLib begin err %d", (int)g_radio_task_result);
        return false;
    }
    return true;
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
    early_banner(); // MUST NOT be preceded by anything that can stall

    memset(&uist, 0, sizeof(uist));
    uist.ident[0] = 0;
    snprintf(uist.rate, sizeof(uist.rate), "boot");
    for (int i = 0; i < UI_TLM_LINES; i++) uist.tlm[i][0] = 0;

#if defined(PIN_BOARD_LED)
    pinMode(PIN_BOARD_LED, OUTPUT);
    digitalWrite(PIN_BOARD_LED, LOW); // LED_ON = HIGH: lit only when locked
#endif
    elrs_decode_init(&dctx);
    sniffer_sweep_build(&sweep);

    // OLED: bounded probe, splash only on success
    oled_ok = ui_probe();
    if (oled_ok) {
        ui_start();
    } else {
        Serial.printf("{\"t\":\"error\",\"what\":\"oled_init\",\"detail\":\"no ACK at 0x%02x SDA=%d SCL=%d\"}\n",
                      OLED_I2C_ADDR, PIN_OLED_SDA, PIN_OLED_SCL);
    }

    // Radio: bounded init, fault is recoverable and reported both ways
    static char radio_err[72];
    radio_ok = radio_init_bounded(radio_err, sizeof(radio_err));
    if (!radio_ok) {
        Serial.printf("{\"t\":\"error\",\"what\":\"radio_init\",\"detail\":\"%s\",\"pins\":\"NSS=%d SCK=%d MISO=%d MOSI=%d RST=%d DIO1=%d BUSY=%d\"}\n",
                      radio_err, PIN_LORA_NSS, PIN_LORA_SCK, PIN_LORA_MISO,
                      PIN_LORA_MOSI, PIN_LORA_RST, PIN_LORA_DIO1, PIN_LORA_BUSY);
        if (oled_ok) ui_show_fault("RADIO FAULT", radio_err);
        snprintf(uist.rate, sizeof(uist.rate), "rf fault");
    }

    Serial.printf("{\"t\":\"ready\",\"steps\":%u,\"sync_freq\":%lu,\"radio\":%u,\"oled\":%u}\n",
                  sweep.count, (unsigned long)(ELRS_2G4_SYNC_FREQ_HZ / 1000000),
                  radio_ok ? 1 : 0, oled_ok ? 1 : 0);
    if (radio_ok) apply_step(0);
    last_stats_ms = millis();
}

void loop()
{
    // retry a faulted radio — the fault may be transient
    static uint32_t last_radio_retry = 0;
    if (!radio_ok && millis() - last_radio_retry > RADIO_RETRY_MS) {
        last_radio_retry = millis();
        static char err[72];
        if (radio_init_bounded(err, sizeof(err))) {
            radio_ok = true;
            Serial.println("{\"t\":\"radio_up\"}");
            apply_step(0);
        } // stay quiet otherwise: the 1 Hz stats line already says radio:0
    }

    if (radio_ok) {
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

        if (!locked && millis() - step_entered_ms > current_dwell_ms()) {
            apply_step(step_idx + 1);
        }
        if (locked && millis() - last_pkt_ms > 5000) {
            locked = false;
            Serial.println("{\"t\":\"unlock\",\"why\":\"timeout\"}");
            apply_step(step_idx + 1);
        }
    }

    stats_tick(); // always runs — alive-with-no-radio still emits 1 Hz JSON
}
