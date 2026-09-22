// =============================================================================
// board_pins.h — pin definitions for the ELRS sniffer hardware candidates.
//
// !!! UNVERIFIED HARDWARE — see BOARD.md !!!
// The exact LilyGo model and its SX1280 wiring must be confirmed against the
// physical unit before flashing. Sections are tagged:
//   [VERIFIED]   — taken from the manufacturer's own example source
//   [GUESS]      — placeholder until the board is in hand / inspected
// =============================================================================
#pragma once

#if defined(BOARD_LILYGO_T_EMBED_S3)

// ---------------------------------------------------------------------------
// LilyGo T-Embed (ESP32-S3, 1.9" ST7789 170x320 portrait / 320x170 landscape)
//
// DISPLAY PINS [VERIFIED] — from Xinyuan-LilyGO/T-Embed factory sketch,
// examples/factory/pin_config.h @ main:
//   PIN_LCD_CS=10  PIN_LCD_DC=13  PIN_LCD_CLK=12  PIN_LCD_MOSI=11
//   PIN_LCD_RES=9  PIN_LCD_BL=15  (panel: 320x170 in the factory's landscape
//   orientation; ST7789V)
//
// SX1280 PINS [GUESS] — STOCK T-Embed has NO SX1280. Its sub-GHz radio is a
// CC1101 (RADIO_CS_PIN=17 in the factory sketch). The 2.4 GHz ELRS radio on
// Konrad's ecosystem unit is assumed to be wired like a LilyGo T3S3 SX1280
// (the usual LilyGo 2.4G arrangement); CONFIRM WITH A MULTIMETER / SCHEMATIC.
// If the T-Embed runs the SX1280 on the SD-card SPI bus (pins 38-41) or the
// NFC/RFID SPI, change the SPI section below accordingly.
// ---------------------------------------------------------------------------

// --- Display (ST7789, SPI) [VERIFIED pins]
#define PIN_TFT_CS    10
#define PIN_TFT_DC    13
#define PIN_TFT_RST   9
#define PIN_TFT_SCK   12
#define PIN_TFT_MOSI  11
#define PIN_TFT_BL    15
#define PIN_TFT_LED   46   // [VERIFIED] PIN_POWER_ON: hold HIGH to power the board

// --- TFT_eSPI setup block (must precede #include <TFT_eSPI.h>) ---
// Panel: ST7789V 170x320 portrait. [VERIFIED] pins from the factory sketch;
// offsets/driver choice may need a tweak once real hardware is on the bench
// (if the image is shifted, try rotation or CGRAM offsets — see BOARD.md).
#define USER_SETUP_LOADED
#define ST7789_DRIVER
#define TFT_WIDTH  170
#define TFT_HEIGHT 320
#define TFT_CS   PIN_TFT_CS
#define TFT_DC   PIN_TFT_DC
#define TFT_RST  PIN_TFT_RST
#define TFT_SCLK PIN_TFT_SCK
#define TFT_MOSI PIN_TFT_MOSI
#define TFT_BL   PIN_TFT_BL
#define TFT_BACKLIGHT_ON HIGH
#define LOAD_GLCD
#define LOAD_FONT2
#define LOAD_FONT4
#define SPI_FREQUENCY 40000000
#define SNIFFER_HAS_TFT 1

// --- SX1280 radio [GUESS — T3S3-style wiring, MUST confirm]
#define PIN_LORA_NSS   7
#define PIN_LORA_SCK   5
#define PIN_LORA_MOSI  6
#define PIN_LORA_MISO  3
#define PIN_LORA_BUSY 13
#define PIN_LORA_DIO1 14
#define PIN_LORA_RST  12
#define PIN_LORA_TXEN -1   // PA variant only; -1 = not fitted
#define PIN_LORA_RXEN -1

#define SNIFFER_BOARD_NAME "LilyGo T-Embed S3 [radio pins GUESSED]"

#elif defined(BOARD_LILYGO_T3S3_2G4)

// ---------------------------------------------------------------------------
// LilyGo T3S3 2.4G (ESP32-S3 + SX1280, SSD1306 0.96" OLED over I2C).
// Radio pins match LilyGo-LoRa-Series / Meshtastic tlora-t3s3-2-4g variant
// [GUESS until flashed on real hardware]. OLED support is TODO — the TFT
// sections below are inert with this board selected.
// ---------------------------------------------------------------------------
#define PIN_LORA_NSS   7
#define PIN_LORA_SCK   5
#define PIN_LORA_MOSI  6
#define PIN_LORA_MISO  3
#define PIN_LORA_BUSY 13
#define PIN_LORA_DIO1 14
#define PIN_LORA_RST  12
#define PIN_LORA_TXEN -1
#define PIN_LORA_RXEN -1
#define PIN_OLED_SDA  18
#define PIN_OLED_SCL  17

#define SNIFFER_BOARD_NAME "LilyGo T3S3 2.4G [OLED UI TODO]"
#define SNIFFER_HAS_TFT 0

#else
#error "Select a board: BOARD_LILYGO_T_EMBED_S3 (see BOARD.md)"
#endif

// RadioLib SPI: T-Embed has the display and the (modded) radio on separate
// SPI peripherals; RadioLib gets its own SPI bus instance when the pins
// differ from the TFT's.
#if SNIFFER_HAS_TFT && defined(PIN_LORA_SCK) && (PIN_LORA_SCK != PIN_TFT_SCK)
#define SNIFFER_RADIO_HSPI 1
#endif
