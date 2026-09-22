// sniffer_radio.cpp — see sniffer_radio.h
#include "sniffer_radio.h"
#include "board_pins.h"
#include "elrs_parse.h" // uidMacSeed / crc-init helpers (elrs_defs.h cited)
#ifdef ARDUINO
#include <Arduino.h>
#include <RadioLib.h>

// RadioLib keeps the packet-params writer protected; the sniffer needs exact
// control (implicit header + fixed length + radio CRC OFF + per-dwell IQ),
// so widen it (and the packet-type setter) via inheritance. The register
// tweak uses the Module handle SnifferRadio owns — SX128x::mod is private.
class SnifferSX1280 : public SX1280 {
public:
    using SX1280::SX1280;
    using SX1280::setPacketParamsLoRa;
    using SX1280::setPacketParamsGFSK;
    using SX1280::setPacketType;
};

void sniffer_sweep_build(sniffer_sweep_t *sw)
{
    sw->count = 0;
    // Likelihood-ordered first pass — most-deployed LoRa rates first, each
    // with normal IQ then inverted (invertIQ = UID[5]&1 splits real links
    // ~50/50). FLRC trio (single polarity — ELRS ignores InvertIQ for the
    // FLRC branch) sits after the four main LoRa pairs; 8ch variants and
    // legacy 2.x rows last.
    static const uint8_t order3x[ELRS_RATES_3X_COUNT] = { 2, 0, 3, 5, 1, 4 };
    //                         table idx ->   250, 500, 150, 50, 333-8, 100-8
    for (uint8_t k = 0; k < 4; k++) { // the four main rates, IQ pairs
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
        // PA variant: antenna switch enables (verified SX1280PA_PingPong.ino)
        radio->setRfSwitchPins(ps->rxen, ps->txen);
    }
    radio->setDio1Action(on_dio1);
    return RADIOLIB_ERR_NONE;
}

bool SnifferRadio::apply(const sniffer_step_t &step, uint32_t freq_hz)
{
    const elrs_rate_t *r = step.rate;
    payload_len = r->payload;
    if (r->flrc) {
        // FLRC branch — exact ELRS SX1280.cpp Config/SetPacketParamsFLRC:
        //   modparams {BR0.65_BW0.6 (0x86), CR 1/2 (0x00), BT 1.0 (0x10)}
        //   preamble 32 -> AGCPreambleLength ((32/4)-1)<<4 = 0x70
        //   32-bit sync word = uidMacSeed, match SWM1, fixed 8B payload,
        //   3-byte radio CRC seeded with OtaCrcInitializer, whitening off.
        static_cast<SnifferSX1280 *>(radio)->setPacketType(RADIOLIB_SX128X_PACKET_TYPE_FLRC);
        radio->setFrequency(freq_hz / 1000000.0);
        uint8_t mp[3] = { r->bw, r->cr, r->sf }; // flrc rows carry bw/bt/cr bytes
        mod->SPIwriteStream(RADIOLIB_SX128X_CMD_SET_MODULATION_PARAMS, mp, 3);
        // sync word with the DS 16.4 erratum swap (SX1280.cpp SetPacketParamsFLRC)
        uint8_t sw[4] = { flrc_sw[0], flrc_sw[1], flrc_sw[2], flrc_sw[3] };
        if ((sw[0] == 0x8C && sw[1] == 0x38) || (sw[0] == 0x63 && sw[1] == 0x0E)) {
            uint8_t t = sw[0]; sw[0] = sw[1]; sw[1] = t;
        }
        radio->setSyncWord(sw, 4);
        static_cast<SnifferSX1280 *>(radio)->setPacketParamsGFSK(0x70, 0x04, 0x10, RADIOLIB_SX128X_GFSK_FLRC_CRC_3_BYTE,
                                   0x08, payload_len, RADIOLIB_SX128X_GFSK_FLRC_PACKET_FIXED);
        // CRC seed = OtaCrcInitializer; polynomial left at chip default 0x1021
        // (ELRS never writes the FLRC poly register)
        radio->setCRC(3, flrc_seed, 0x1021);
        return true;
    }
    // LoRa branch: individual setters write the SetModulationParams pieces;
    // packet params (implicit header, fixed length, CRC OFF, IQ) go out in
    // one explicit call — exactly the ELRS air config from SX1280.cpp
    // SetPacketParamsLoRa. ELRS InvertIQ==true is register 0x00 (INVERTED).
    radio->setBandwidth(r->bw == 0x18 ? 812.5f : r->bw == 0x26 ? 406.25f : 203.125f);
    radio->setSpreadingFactor(r->sf >> 4);         // 0x50->5 .. 0x90->9
    radio->setCodingRate(r->cr, true);             // LI variants, raw values match
    radio->setPreambleLength(r->preamble);
    radio->setFrequency(freq_hz / 1000000.0);
    static_cast<SnifferSX1280 *>(radio)->setPacketParamsLoRa(
        r->preamble, RADIOLIB_SX128X_LORA_HEADER_IMPLICIT, payload_len,
        0x00, step.iq_inverted ? RADIOLIB_SX128X_LORA_IQ_INVERTED
                               : RADIOLIB_SX128X_LORA_IQ_STANDARD);
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
    radio->startReceive(RADIOLIB_SX128X_RX_TIMEOUT_INF,
                        RADIOLIB_SX128X_IRQ_RX_DONE, RADIOLIB_SX128X_IRQ_RX_DONE);
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

bool SnifferRadio::read_packet(uint8_t *buf, size_t len, float &rssi, float &snr)
{
    if (!dio1_fired) return false;
    dio1_fired = false;
    int16_t st = radio->readData(buf, len);
    rssi = radio->getRSSI();
    snr = radio->getSNR();
    start_rx(); // readData drops to standby; resume continuous RX
    return st == RADIOLIB_ERR_NONE;
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
