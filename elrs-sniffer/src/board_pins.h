// =============================================================================
// board_pins.h — LilyGo T3-S3 (ESP32-S3) SX1280 2.4 GHz variant
//
// [VERIFIED] against the manufacturer's own source:
//   Xinyuan-LilyGO/LilyGo-LoRa-Series, examples/T3S3Factory/utilities.h
//   (T3_S3_V1_2_SX1280 / T3_S3_V1_2_SX1280_PA blocks, fetched 2026-09-22)
//   and examples/LoRa/T3S3/SX1280PA_PingPong/SX1280PA_PingPong.ino
//   (RF-switch via radio.setRfSwitchPins(RX=21, TX=10)).
//
// Two hardware variants share this pinout; the PA variant adds an external
// antenna-switch PA driven by two GPIOs. Non-PA is the common "T3-S3 2.4G
// without PA" listing; uncomment T3S3_SX1280_PA if the unit has the PA.
// =============================================================================
#pragma once

#if !defined(BOARD_LILYGO_T3S3_SX1280)
#define BOARD_LILYGO_T3S3_SX1280
#endif

// --- SSD1306 OLED, 0.96" 128x64, I2C [VERIFIED: factory sketch addr 0x3c] ---
#define PIN_OLED_SDA   18
#define PIN_OLED_SCL   17
#define OLED_I2C_ADDR  0x3C
// If the panel shows garbage/columns of junk, it may be an SH1106 — swap the
// constructor in ui.cpp (U8G2_SSD1306_... -> U8G2_SH1106_128X64_NONAME_F_HW_I2C).

// --- SX1280 radio, SPI (FSPI) [VERIFIED: utilities.h T3_S3_V1_2_SX1280*] ---
// Re-verified verbatim from LilyGo-LoRa-Series master on 2026-09-22
// (examples/T3S3Factory/utilities.h + LoRa/T3S3/SX1280*_PingPong). Alternate
// wirings circulate (RST=12/DIO1=14/BUSY=13) — the firmware auto-probes
// both families at boot (RADIO_PIN_SETS in sniffer_radio.cpp), so a board
// with either wiring comes up; these compile-time pins remain the default.
#define PIN_LORA_NSS   7    // RADIO_CS_PIN
#define PIN_LORA_SCK   5    // RADIO_SCLK_PIN
#define PIN_LORA_MISO  3    // RADIO_MISO_PIN
#define PIN_LORA_MOSI  6    // RADIO_MOSI_PIN
#define PIN_LORA_RST   8    // RADIO_RST_PIN
#define PIN_LORA_DIO1  9    // RADIO_DIO1_PIN  (IRQ)
#define PIN_LORA_BUSY 36    // RADIO_BUSY_PIN

// --- PA-variant antenna switch enables [VERIFIED: SX1280PA_PingPong.ino] ---
// Non-PA boards have no switch: leave -1. PA boards: RX=IO21, TX=IO10 and
// MUST use these (the PA is deaf until RXEN is driven).
#if defined(T3S3_SX1280_PA)
#define PIN_LORA_RXEN  21   // RADIO_RX_PIN
#define PIN_LORA_TXEN  10   // RADIO_TX_PIN
#define SNIFFER_BOARD_NAME "LilyGo T3-S3 SX1280-PA"
#else
#define PIN_LORA_RXEN  -1
#define PIN_LORA_TXEN  -1
#define SNIFFER_BOARD_NAME "LilyGo T3-S3 SX1280"
#endif

// --- Extras on the same header [VERIFIED: utilities.h] ---
#define PIN_BOARD_LED  37   // onboard LED, HIGH = on (lock indicator)
#define PIN_VBAT_ADC    1   // battery divider (100k/100k), ADC1_CH0
#define PIN_BUTTON      0   // BOOT button

#define SNIFFER_HAS_OLED 1
