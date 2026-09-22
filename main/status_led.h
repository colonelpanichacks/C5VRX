#pragma once

/* XIAO ESP32-C5 onboard user LED (yellow) on GPIO27.
 * Heartbeat blink while searching, solid on while a carrier is present. */

typedef enum {
    STATUS_LED_OFF = 0,
    STATUS_LED_HEARTBEAT,   /* no carrier: slow blink, "listening" */
    STATUS_LED_SOLID,       /* carrier present */
    STATUS_LED_SCAN,        /* channel scanner running: fast blink */
} status_led_mode_t;

void status_led_init(void);
void status_led_set(status_led_mode_t mode);

/* Call every 50 ms from the AGC task tick. */
void status_led_tick(void);
