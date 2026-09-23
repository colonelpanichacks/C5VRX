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

#define SNIFFER_MAX_STEPS (2 * (ELRS_RATES_3X_COUNT + ELRS_RATES_2X_COUNT) + ELRS_RATES_FLRC_COUNT)

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
void sniffer_set_y925(bool en);
bool sniffer_get_y925();

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
    // Force-restart the radio on a step (re-apply + start_rx). Every command
    // path underneath is bounded by RadioLib's busy-pin timeout — this is
    // the dwell watchdog's recovery hammer; it cannot hang the caller.
    bool recover(const sniffer_step_t &step, uint32_t freq_hz);
    // FLRC discovery mode (unknown UID): no sync-word match and radio CRC
    // OFF — the keyed 32-bit sync word and the seeded CRC both depend on
    // the (unknown) UID, so exact filtering is impossible until a sync
    // packet leaks UID[3..5]. Discovery takes raw demod bursts and lets the
    // parser's structure gates do the filtering. (The SX1280 FLRC has no
    // usable 16-bit-match-any mode; this is the physically real equivalent.)
    void setFlrcDiscovery(bool d) { flrc_discovery = d; }
    bool flrcDiscovery() const { return flrc_discovery; }
    // Retune only (FHSS hop-following). Bounded by RadioLib internals.
    bool tune(uint32_t freq_hz);
    // FLRC identity: 32-bit sync word = uidMacSeedGet(UID), radio CRC seed =
    // OtaCrcInitializer — both derived from the bind-phrase UID (elrs_defs.h
    // citations). Without the right phrase, FLRC demods all fail the radio
    // CRC and the dwell stays silent (by design).
    void setFlrcIdentity(const uint8_t uid[6]);
    // (Re)start continuous RX with DIO1 -> RX_DONE only.
    void start_rx();
    // Called from loop; when a packet demodded, fills buf/len/rssi/snr and
    // the IRQ status word captured at RxDone (before readData clears it).
    bool read_packet(uint8_t *buf, size_t len, float &rssi, float &snr,
                     uint16_t *irq_out = NULL);
    Module *mod_ptr() { return mod; }
    // Non-destructive read of the IRQ status register (for error-IRQ
    // counting between packets; does NOT clear).
    uint16_t irq_status();
    // Instantaneous RSSI via the SX1280 GET_RSSIINST command (0x1F, per
    // SX1280_Regs.h / datasheet) — unlike the packet-status based getRSSI()
    // this is live energy in any RX state, which is what the sweep
    // diagnostic needs. rssi in dBm; returns the RadioLib status.
    int16_t rssiInst(float &rssi_dbm);

    static volatile bool dio1_fired;
    static void on_dio1();

private:
    SPIClass *spi = nullptr;
    Module *mod = nullptr;
    SX1280 *radio = nullptr;
    uint8_t payload_len = 0;
    uint8_t flrc_sw[4] = { 0, 0, 0, 3 };  // uidMacSeedGet (default-phrase UID)
    uint16_t flrc_seed = 3;               // OtaCrcInitializer for the FLRC radio CRC
    bool flrc_id_ok = false;
    bool flrc_discovery = false;
};

extern SnifferRadio g_radio;
#endif // ARDUINO
