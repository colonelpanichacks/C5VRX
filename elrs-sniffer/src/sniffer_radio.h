// =============================================================================
// sniffer_radio.h — RadioLib glue for passive SX1280 ELRS sniffing.
//
// ELRS air config applied per dwell (SX1280.cpp Config/SetPacketParamsLoRa):
//   LoRa, implicit header, fixed payload (8 or 13), RADIO CRC OFF, LI coding
//   rates, IQ per the invertIQ rule (UID[5]&1 — so both polarities are swept).
// The SX1280 LoRa modem has no sync word: every preamble that demods raises
// RX_DONE, hence the software-CRC / plausibility layer in elrs_parse.
// =============================================================================
#pragma once
#include <stdint.h>
#include <stddef.h>
#include "elrs_defs.h"

#define SNIFFER_MAX_STEPS (2 * (ELRS_RATES_3X_COUNT + ELRS_RATES_2X_COUNT))

typedef struct {
    const elrs_rate_t *rate;
    bool iq_inverted;      // ELRS invertIQ rule is per-link; sweep both
    bool legacy_2x;        // legacy row -> packet marked unvalidated
} sniffer_step_t;

typedef struct {
    sniffer_step_t steps[SNIFFER_MAX_STEPS];
    uint8_t count;
} sniffer_sweep_t;

// Build the dwell list: current 3.x rates first (both IQ polarities), then
// legacy 2.x variants. FLRC rates are deliberately absent — their 32-bit sync
// word is UID-derived (see elrs_defs.h); hunting them is a post-capture TODO.
void sniffer_sweep_build(sniffer_sweep_t *sw);

// ---- pin-set table for the startup auto-probe ------------------------------
// The V1.2 SX1280 wiring per LilyGo's factory sources, plus the alternates
// that circulate for T3-S3 boards (set B: RST=12/DIO1=14/BUSY=13). All share
// the SPI bus (SCK=5, MISO=3, MOSI=6, NSS=7). -1 = pin not fitted/driven.
typedef struct {
    const char *name;
    int8_t nss, sck, miso, mosi, rst, dio1, busy, rxen, txen;
} radio_pin_set_t;

extern const radio_pin_set_t RADIO_PIN_SETS[];
extern const uint8_t RADIO_PIN_SET_COUNT;
extern const uint8_t RADIO_PIN_SET_DEFAULT; // compile-time pins from board_pins.h

#ifdef ARDUINO
#include <RadioLib.h>

class SnifferRadio {
public:
    // SPI + radio init with an explicit pin set; returns the RadioLib
    // status code (RADIOLIB_ERR_NONE == 0 on success — a plain bool has
    // bitten us here before: success reads as "err 1").
    int16_t begin(const radio_pin_set_t *ps);
    int16_t begin(); // default: RADIO_PIN_SETS[RADIO_PIN_SET_DEFAULT]
    // Apply a dwell step: modulation, fixed length, CRC off, IQ, frequency.
    bool apply(const sniffer_step_t &step, uint32_t freq_hz);
    // (Re)start continuous RX with DIO1 -> RX_DONE only.
    void start_rx();
    // Called from loop; when a packet demodded, fills buf/len/rssi/snr.
    bool read_packet(uint8_t *buf, size_t len, float &rssi, float &snr);

    static volatile bool dio1_fired;
    static void on_dio1();

private:
    SPIClass *spi;
    SX1280 *radio;
    uint8_t payload_len;
};

extern SnifferRadio g_radio;
#endif // ARDUINO
