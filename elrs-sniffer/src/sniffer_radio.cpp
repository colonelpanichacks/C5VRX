// sniffer_radio.cpp — see sniffer_radio.h
#include "sniffer_radio.h"
#include "board_pins.h"
#include "elrs_parse.h" // uidMacSeed / crc-init helpers (elrs_defs.h cited)
void sniffer_sweep_build(sniffer_sweep_t *sw)
{
    sw->count = 0;
    // Round-4 order: LoRa FIRST. The self-seeded CRC14 validator is
    // mathematically bulletproof on LoRa (no junk passes), LoRa 250 is the
    // stock Pocket default, and quiet LoRa dwells are cheap (2 s). FLRC
    // promiscuous discovery is the fallback for FLRC-only links. Order:
    // 250, 500, 150, 50 (IQ pairs), FLRC trio, 8ch variants, legacy rows.
    static const uint8_t order3x[ELRS_RATES_3X_COUNT] = { 2, 0, 3, 5, 1, 4 };
    //                         table idx ->   250, 500, 150, 50, 333-8, 100-8
    for (uint8_t k = 0; k < 4; k++) { // the four main LoRa rates, IQ pairs
        uint8_t i = order3x[k];
        sw->steps[sw->count++] = { &ELRS_RATES_3X[i], false, false };
        sw->steps[sw->count++] = { &ELRS_RATES_3X[i], true, false };
    }
    for (uint8_t i = 0; i < ELRS_RATES_FLRC_COUNT; i++) {
        sw->steps[sw->count++] = { &ELRS_RATES_FLRC[i], false, false };
    }
    for (uint8_t k = 4; k < ELRS_RATES_3X_COUNT; k++) { // 8ch variants
        uint8_t i = order3x[k];
        sw->steps[sw->count++] = { &ELRS_RATES_3X[i], false, false };
        sw->steps[sw->count++] = { &ELRS_RATES_3X[i], true, false };
    }
    for (uint8_t i = 0; i < ELRS_RATES_2X_COUNT; i++) {
        sw->steps[sw->count++] = { &ELRS_RATES_2X[i], false, true };
        sw->steps[sw->count++] = { &ELRS_RATES_2X[i], true, true };
    }
}


#ifdef ARDUINO
#include <Arduino.h>
#include <RadioLib.h>

// (SnifferSX1280 lives in sniffer_radio.h — main.cpp needs it for the
// round-15 stock-RadioLib reference RX baseline.)

const radio_pin_set_t RADIO_PIN_SETS[] = {
    // name            nss sck miso mosi rst dio1 busy rxen txen
    { "v12-sx1280",     7,  5,   3,   6,  8,   9,  36,  -1,  -1 },
    { "v12-sx1280-pa",  7,  5,   3,   6,  8,   9,  36,  21,  10 },
    { "altB",           7,  5,   3,   6, 12,  14,  13,  -1,  -1 },
    { "altB-pa",        7,  5,   3,   6, 12,  14,  13,  21,  10 },
};
const uint8_t RADIO_PIN_SET_COUNT = sizeof(RADIO_PIN_SETS) / sizeof(RADIO_PIN_SETS[0]);
#if PIN_LORA_RXEN != -1
const uint8_t RADIO_PIN_SET_DEFAULT = 1; // v12-sx1280-pa
#else
const uint8_t RADIO_PIN_SET_DEFAULT = 0; // v12-sx1280
#endif

volatile bool SnifferRadio::dio1_fired = false;

static bool g_y925_state = true; // 'Y' toggle: write 0x925 SF config
void sniffer_set_y925(bool en) { g_y925_state = en; }
bool sniffer_get_y925() { return g_y925_state; }

void SnifferRadio::on_dio1()
{
    dio1_fired = true;
}

SnifferRadio g_radio;

int16_t SnifferRadio::begin()
{
    return begin(&RADIO_PIN_SETS[RADIO_PIN_SET_DEFAULT]);
}

int16_t SnifferRadio::begin(const radio_pin_set_t *ps)
{
    // T3-S3 radio SPI bus (FSPI) at verified pins 5/3/6/7 — board_pins.h.
    spi = new SPIClass(FSPI);
    spi->begin(ps->sck, ps->miso, ps->mosi, ps->nss);
    mod = new Module(ps->nss, ps->dio1, ps->rst, ps->busy, *spi,
                     SPISettings(8000000, MSBFIRST, SPI_MODE0));
    radio = new SnifferSX1280(mod);
    // placeholder params; every dwell reconfigures via apply()
    int16_t st = radio->begin(2441.4, 812.5, 9, 7);
    if (st != RADIOLIB_ERR_NONE) return st;
    // ELRS SX1280.cpp Begin(): register 0x0891 |= 0xC0 (high sensitivity)
    mod->SPIwriteRegister(0x0891, mod->SPIreadRegister(0x0891) | 0xC0);
    if (ps->rxen != -1) {
        // PA variant: H658 FEM antenna switch. RadioLib only toggles these
        // pins inside its own receive()/transmit() — the sniffer's raw-SPI
        // RX path bypasses those, so the switch must be driven explicitly:
        // RXEN high connects the antenna to the LNA path. Without this the
        // FEM leaves the antenna disconnected in RX (live RSSI, zero signal).
        radio->setRfSwitchPins(ps->rxen, ps->txen);
        rf_rxen = ps->rxen;
        rf_txen = ps->txen;
        pinMode(rf_rxen, OUTPUT);
        pinMode(rf_txen, OUTPUT);
        digitalWrite(rf_rxen, HIGH);
        digitalWrite(rf_txen, LOW);
    }
    radio->setDio1Action(on_dio1);
    return RADIOLIB_ERR_NONE;
}

// GET_STATUS chipmode (bits 7:5): 2=STDBY_RC 3=STDBY_XOSC 4=FS 5=RX 6=TX
const char *sniffer_chipmode_name(uint8_t st)
{
    switch ((st >> 5) & 0x07) {
    case 2: return "stdby_rc";
    case 3: return "stdby_xosc";
    case 4: return "fs";
    case 5: return "rx";
    case 6: return "tx";
    default: return "?";
    }
}

static uint8_t chipmode_read(Module *m)
{
    uint8_t st = 0;
    if (m) m->SPIreadStream(RADIOLIB_SX128X_CMD_GET_STATUS, &st, 1);
    return st;
}


bool SnifferRadio::apply(const sniffer_step_t &step, uint32_t freq_hz)
{
    const elrs_rate_t *r = step.rate;
    payload_len = r->payload;
    // SMOKING-GUN FIX (round 13): the SX1280 IGNORES configuration written
    // while it is in RX — ELRS always SetMode(STDBY_RC) before reconfig
    // (SX1280.cpp L79/L156-188). Without this, every dwell silently ran the
    // PREVIOUS dwell's modem config (FLRC junk self-perpetuated; LoRa dwells
    // never became LoRa). Raw ops, RadioLib-free.
    uint8_t st_before = chipmode_read(mod);
    uint8_t standby[1] = { 0x00 }; // SetStandby(STDBY_RC)
    mod->SPIwriteStream(0x80, standby, 1);
    delay(2); // ELRS uses 1500us for the standby transition
    if (r->flrc) {
        // FLRC branch — ELRS SX1280.cpp Config/SetPacketParamsFLRC semantics
        // with register-level writes where RadioLib stores/rewrites
        // differently. The previous version wedged here SILENTLY: RadioLib's
        // setSyncWord/setCRC RE-SEND SetPacketParams from RadioLib's own
        // stored fields, and the stored preamble was never set -> 0x00 ->
        // invalid -> chip fault. Every call's return code is now logged.
        int16_t rc_fq, rc_mp, rc_pp, rc_sw, rc_crc;
        static_cast<SnifferSX1280 *>(radio)->setPacketType(RADIOLIB_SX128X_PACKET_TYPE_FLRC);
        rc_fq = radio->setFrequency(freq_hz / 1000000.0);
        // SetModulationParamsFLRC: {BR0.65/BW0.6, CR 1/2, BT 1.0}
        uint8_t mp[3] = { r->bw, r->cr, r->sf }; // flrc rows carry bw/cr/bt bytes
        rc_mp = mod->SPIwriteStream(RADIOLIB_SX128X_CMD_SET_MODULATION_PARAMS, mp, 3);
        // SetPacketParamsFLRC: preamble 32 -> ((32/4)-1)<<4 = 0x70; P32S sync;
        // match SWM1; fixed len; 3-byte CRC; whitening off. RadioLib's
        // setPacketParamsGFSK(pre,syncLen,match,crc,whiten,len,hdr) transmits
        // {pre,syncLen,match,hdr,len,crc,whiten} — exactly ELRS's order.
        rc_pp = static_cast<SnifferSX1280 *>(radio)->setPacketParamsGFSK(
            0x70, 0x04, 0x10, RADIOLIB_SX128X_GFSK_FLRC_CRC_3_BYTE,
            0x08, payload_len, RADIOLIB_SX128X_GFSK_FLRC_PACKET_FIXED);
        if (flrc_discovery) {
            // discovery: no sync-word match, CRC off — pure demod, parser gates
            rc_pp = static_cast<SnifferSX1280 *>(radio)->setPacketParamsGFSK(
                0x70, 0x00, 0x00, RADIOLIB_SX128X_GFSK_FLRC_CRC_OFF,
                0x08, payload_len, RADIOLIB_SX128X_GFSK_FLRC_PACKET_FIXED);
            uint8_t st = 0;
            mod->SPIreadStream(RADIOLIB_SX128X_CMD_GET_STATUS, &st, 1);
            Serial.printf("{\"t\":\"dbg\",\"what\":\"flrc_discovery\",\"fq\":%d,\"mp\":%d,"
                          "\"pp\":%d,\"status\":%u}\n",
                          (int)rc_fq, (int)rc_mp, (int)rc_pp, st);
            return rc_fq == 0 && rc_mp == 0 && rc_pp == 0;
        }
        // sync word at ELRS's REG_FLRC_SYNC_WORD (0x9CF), 4 bytes MSB-first,
        // DS 16.4 first-two-byte swap — direct write because RadioLib's
        // setSyncWord targets 0x9C5 and reverses byte order.
        uint8_t sw[4] = { flrc_sw[0], flrc_sw[1], flrc_sw[2], flrc_sw[3] };
        if ((sw[0] == 0x8C && sw[1] == 0x38) || (sw[0] == 0x63 && sw[1] == 0x0E)) {
            uint8_t t = sw[0]; sw[0] = sw[1]; sw[1] = t;
        }
        mod->SPIwriteRegisterBurst(0x09CF, sw, 4); rc_sw = 0;
        // FLRC CRC seed at REG_FLRC_CRC_SEED (0x9C8) = OtaCrcInitializer;
        // polynomial register untouched (ELRS never writes it).
        uint8_t seed[2] = { (uint8_t)(flrc_seed >> 8), (uint8_t)(flrc_seed & 0xFF) };
        mod->SPIwriteRegisterBurst(0x09C8, seed, 2); rc_crc = 0;
        uint8_t st = 0;
        mod->SPIreadStream(RADIOLIB_SX128X_CMD_GET_STATUS, &st, 1);
        Serial.printf("{\"t\":\"dbg\",\"what\":\"flrc_setup\",\"fq\":%d,\"mp\":%d,"
                      "\"pp\":%d,\"sw\":%d,\"crc\":%d,\"status\":%u}\n",
                      (int)rc_fq, (int)rc_mp, (int)rc_pp, (int)rc_sw, (int)rc_crc, st);
        return rc_fq == 0 && rc_mp == 0 && rc_pp == 0 && rc_sw == 0 && rc_crc == 0;
    }
    // LoRa branch: packet type FIRST — an FLRC dwell leaves the chip in FLRC
    // mode, and the modulation params below are only valid in LoRa mode
    // (REGRESSION: without this, the first FLRC dwell silenced every later
    // LoRa dwell: rx=0 on all rates, RSSIINST still reporting). Then the
    // individual setters write the SetModulationParams pieces; packet params
    // (implicit header, fixed length, CRC OFF, IQ) go out in one explicit
    // call — exactly the ELRS air config from SX1280.cpp SetPacketParamsLoRa.
    static_cast<SnifferSX1280 *>(radio)->setPacketType(RADIOLIB_SX128X_PACKET_TYPE_LORA);
    // RAW SetModulationParams {sf, bw, cr} — bypasses RadioLib's setter
    // chain entirely (no transformation, no stored-state drift); the audit's
    // "verify what hits the register" is satisfied by construction + the
    // lora_regs readback probe.
    uint8_t mp[3] = { r->sf, r->bw, r->cr };
    mod->SPIwriteStream(RADIOLIB_SX128X_CMD_SET_MODULATION_PARAMS, mp, 3);
    // RadioLib OMITS REG_SF_ADDITIONAL_CONFIG (0x925) — ELRS writes it after
    // every SetModulationParams (SX1280.cpp:283-299). 'Y' toggles it live.
    if (g_y925_state)
        mod->SPIwriteRegister(ELRS_REG_SF_ADDITIONAL_CONFIG,
                              elrs_sf_additional_config(r->sf));
    radio->setFrequency(freq_hz / 1000000.0);
    static_cast<SnifferSX1280 *>(radio)->setPacketParamsLoRa(
        r->preamble, RADIOLIB_SX128X_LORA_HEADER_IMPLICIT, payload_len,
        0x00, step.iq_inverted ? RADIOLIB_SX128X_LORA_IQ_INVERTED
                               : RADIOLIB_SX128X_LORA_IQ_STANDARD);
    uint8_t st_after = chipmode_read(mod);
    Serial.printf("{\"t\":\"dbg\",\"what\":\"dwell_setup\",\"rate\":\"%s\","
                  "\"sf\":\"%02x\",\"bw\":\"%02x\",\"cr\":\"%02x\","
                  "\"cm_before\":\"%s\",\"cm_after\":\"%s\",\"raw\":1}\n",
                  r->name, r->sf, r->bw, r->cr,
                  sniffer_chipmode_name(st_before), sniffer_chipmode_name(st_after));
    return true;
}

void SnifferRadio::setFlrcIdentity(const uint8_t uid[6])
{
    uint32_t seed32 = elrs_uid_mac_seed(uid[2], uid[3], uid[4], uid[5]);
    flrc_sw[0] = (uint8_t)(seed32 >> 24);
    flrc_sw[1] = (uint8_t)(seed32 >> 16);
    flrc_sw[2] = (uint8_t)(seed32 >> 8);
    flrc_sw[3] = (uint8_t)(seed32 & 0xFF);
    flrc_seed = elrs_crc_init_from_uid(uid[4], uid[5]);
    flrc_id_ok = true;
}

void SnifferRadio::start_rx()
{
    dio1_fired = false;
    // H658 FEM: keep the antenna switch on the RX/LNA path (RadioLib's own
    // switching is bypassed by the raw-SPI receive path).
    if (rf_rxen != -1) {
        digitalWrite(rf_rxen, HIGH);
        digitalWrite(rf_txen, LOW);
    }
    // ELRS RXnb semantics: SetMode(RX_CONT) = SetRx(periodBase 0x01,
    // count 0xFFFF ~4.1 s), re-armed constantly (main.cpp 2 s watchdog).
    // Round 15: the skip-if-RX shortcut is REMOVED — it may have mis-fired
    // and left the chip parked in FS after expiry (a deafness candidate).
    uint8_t rx[3] = { 0x01, 0xFF, 0xFF }; // base 62.5us, count 0xFFFF
    mod->SPIwriteStream(RADIOLIB_SX128X_CMD_SET_RX, rx, 3);
    uint8_t clr[2] = { 0xFF, 0xFF };
    mod->SPIwriteStream(RADIOLIB_SX128X_CMD_CLEAR_IRQ_STATUS, clr, 2);
}

bool SnifferRadio::recover(const sniffer_step_t &step, uint32_t freq_hz)
{
    if (radio == NULL) return false;
    // re-apply + restart RX; every RadioLib call here has an internal
    // BUSY-pin timeout and returns an error rather than waiting forever
    bool ok = apply(step, freq_hz);
    start_rx();
    return ok;
}

bool SnifferRadio::tune(uint32_t freq_hz)
{
    if (radio == NULL) return false;
    return radio->setFrequency(freq_hz / 1000000.0) == RADIOLIB_ERR_NONE;
}

bool SnifferRadio::read_packet(uint8_t *buf, size_t len, float &rssi, float &snr,
                               uint16_t *irq_out)
{
    if (!dio1_fired) return false;
    dio1_fired = false;
    if (irq_out) *irq_out = irq_status(); // IRQ word at RxDone, pre-clear
    int16_t st = radio->readData(buf, len);
    rssi = radio->getRSSI(); // per-frame packet-status RSSI (prompt, not the poll)
    snr = radio->getSNR();
    start_rx(); // readData drops to standby; resume continuous RX
    return st == RADIOLIB_ERR_NONE;
}

// sync RadioLib's stored LoRa packet params (protected members) so the
// STOCK startReceive (reference 'T' mode) re-sends OUR config, not begin()'s
void SnifferSX1280::storeLoRaParams(uint8_t preamble, uint8_t hdr, uint8_t len, uint8_t crc, uint8_t iq)
{
    this->preambleLengthLoRa = preamble;
    this->headerType = hdr;
    this->payloadLen = len;
    this->crcLoRa = crc;
    (void)iq; // invertIQEnabled is private; begin() leaves it STANDARD (0x40)
}

void SnifferRadio::standby()
{
    uint8_t p[1] = { 0x00 }; // SetStandby(STDBY_RC)
    if (mod) mod->SPIwriteStream(0x80, p, 1);
    delay(2);
}

uint16_t SnifferRadio::irq_status()
{
    uint8_t st[2] = { 0, 0 };
    if (mod) mod->SPIreadStream(RADIOLIB_SX128X_CMD_GET_IRQ_STATUS, st, 2);
    return (uint16_t)(st[0] << 8 | st[1]);
}

int16_t SnifferRadio::rssiInst(float &rssi_dbm)
{
    if (mod == NULL) return RADIOLIB_ERR_WRONG_MODEM; // radio not constructed yet
    uint8_t v = 0;
    // SX1280_RADIO_GET_RSSIINST = 0x1F (SX1280_Regs.h); rssi = -v/2 dBm
    int16_t st = mod->SPIreadStream(0x1F, &v, 1);
    if (st == RADIOLIB_ERR_NONE) rssi_dbm = -v / 2.0f;
    return st;
}
#endif // ARDUINO
