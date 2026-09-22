#pragma once
#include <stdbool.h>
#include "esp_err.h"

/* Stock OUI-SPY buzzer pad: XIAO D2 = GPIO25 on ESP32-C5 (GPIO3 on the S3).
 * The MODEM_DIAG Q[8] lane that normally sits on GPIO25 was moved to GPIO2
 * (MTMS back pad), so the buzzer keeps its original carrier-board pad. */
#define BUZZER_GPIO_NUM  25

typedef enum {
    BUZZER_EVENT_BOOT = 0,
    BUZZER_EVENT_TICK,        /* short blip: channel step / cursor move */
    BUZZER_EVENT_BAND,        /* double blip: band change */
    BUZZER_EVENT_MENU_OPEN,
    BUZZER_EVENT_MENU_CLOSE,
    BUZZER_EVENT_LOCK,        /* carrier acquired: rising chirp */
    BUZZER_EVENT_LOST,        /* carrier lost: falling chirp */
} buzzer_event_t;

esp_err_t buzzer_init(void);
void buzzer_event(buzzer_event_t event);       /* non-blocking, drop-safe */
void buzzer_set_enabled(bool enabled);
bool buzzer_is_enabled(void);

/* Foxhunt proximity cadence, updated by the AGC task:
 *   interval_ms < 0 -> silent
 *   interval_ms = 0 -> steady continuous tone (transmitter is right here)
 *   interval_ms > 0 -> one beep every interval_ms (larger = farther away)
 *   freq_hz > 0 sets the beep/tone pitch (tracks carrier level). */
void buzzer_set_proximity(int interval_ms, int freq_hz);
