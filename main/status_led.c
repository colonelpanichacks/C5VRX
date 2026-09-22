/**
 * status_led.c - XIAO ESP32-C5 onboard user LED (GPIO27) receiver status.
 *
 * HEARTBEAT: ~1.5 s slow blink while the AGC is in SEARCH (no carrier) --
 *            the "no drone" indicator, replacing the old buzzer heartbeat.
 * SOLID:     a carrier is being received (LEARN/TRACK).
 *
 * Plain GPIO toggling driven by the AGC task tick. No timer, no PWM,
 * no interaction with the IQ/DMA realtime path.
 */

#include "status_led.h"

#include "driver/gpio.h"

#define STATUS_LED_GPIO       GPIO_NUM_27
#define STATUS_LED_ACTIVE_LOW 1   /* XIAO user LED is wired to 3V3 */

/* Tick counts at one tick per 50 ms AGC cycle:
 * heartbeat: 2 ticks on (100 ms), 28 ticks off -> 1.5 s period
 * scan:      2 ticks on, 2 ticks off -> 200 ms fast blink */
#define HEARTBEAT_ON_TICKS    2
#define HEARTBEAT_PERIOD_TICKS 30
#define SCAN_ON_TICKS         2
#define SCAN_PERIOD_TICKS     4

static status_led_mode_t s_mode = STATUS_LED_OFF;
static int s_phase = 0;

static void led_write(int on)
{
#if STATUS_LED_ACTIVE_LOW
    gpio_set_level(STATUS_LED_GPIO, on ? 0 : 1);
#else
    gpio_set_level(STATUS_LED_GPIO, on ? 1 : 0);
#endif
}

void status_led_init(void)
{
    const gpio_config_t cfg = {
        .pin_bit_mask = 1ULL << STATUS_LED_GPIO,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&cfg);
    s_mode = STATUS_LED_OFF;
    s_phase = 0;
    led_write(0);
}

void status_led_set(status_led_mode_t mode)
{
    if (s_mode == mode) return;
    s_mode = mode;
    s_phase = 0;
    if (mode == STATUS_LED_SOLID) {
        led_write(1);
    } else if (mode == STATUS_LED_OFF) {
        led_write(0);
    }
}

void status_led_tick(void)
{
    int on_ticks, period_ticks;
    if (s_mode == STATUS_LED_HEARTBEAT) {
        on_ticks = HEARTBEAT_ON_TICKS;
        period_ticks = HEARTBEAT_PERIOD_TICKS;
    } else if (s_mode == STATUS_LED_SCAN) {
        on_ticks = SCAN_ON_TICKS;
        period_ticks = SCAN_PERIOD_TICKS;
    } else {
        return;
    }
    led_write(s_phase < on_ticks ? 1 : 0);
    if (++s_phase >= period_ticks) s_phase = 0;
}
