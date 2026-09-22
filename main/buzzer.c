/**
 * buzzer.c - OUI-SPY piezo buzzer feedback for C5VRX-3.
 *
 * Two audio layers, both fully off the realtime path:
 *   1. Event jingles (boot, channel/band change, menu, carrier lock/lost),
 *      queued and sequenced by a low-priority task.
 *   2. Foxhunt proximity cadence driven by the AGC task's signal metrics:
 *      steady tone when the transmitter is close, beeps that slow down
 *      as the signal gets weaker, silence when muted.
 *
 * LEDC generates the waveform in hardware; the CPU only reconfigures duty
 * and frequency. No DMA, no IQ pacing, no interaction with PARLIO/GDMA.
 */

#include "buzzer.h"

#include <stdint.h>

#include "driver/ledc.h"
#include "esp_check.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

#define BUZZER_LEDC_TIMER       LEDC_TIMER_0
#define BUZZER_LEDC_CHANNEL     LEDC_CHANNEL_0
#define BUZZER_LEDC_MODE        LEDC_LOW_SPEED_MODE
#define BUZZER_LEDC_RES         LEDC_TIMER_10_BIT
#define BUZZER_DUTY_ON          512u    /* 50% of 1023 */
#define BUZZER_DUTY_OFF         0u

#define PROX_TONE_HZ            2200u
#define PROX_BEEP_MS            45u

static QueueHandle_t s_events;
static volatile bool s_enabled = true;
static volatile int s_prox_interval_ms = -1; /* silent until first AGC update */
static volatile uint32_t s_prox_freq_hz = PROX_TONE_HZ;

static void tone_on(uint32_t freq_hz)
{
    ledc_set_freq(BUZZER_LEDC_MODE, BUZZER_LEDC_TIMER, freq_hz);
    ledc_set_duty(BUZZER_LEDC_MODE, BUZZER_LEDC_CHANNEL, BUZZER_DUTY_ON);
    ledc_update_duty(BUZZER_LEDC_MODE, BUZZER_LEDC_CHANNEL);
}

static void tone_off(void)
{
    ledc_set_duty(BUZZER_LEDC_MODE, BUZZER_LEDC_CHANNEL, BUZZER_DUTY_OFF);
    ledc_update_duty(BUZZER_LEDC_MODE, BUZZER_LEDC_CHANNEL);
}

static void tone_ms(uint32_t freq_hz, uint32_t ms)
{
    tone_on(freq_hz);
    vTaskDelay(pdMS_TO_TICKS(ms));
    tone_off();
}

static void gap_ms(uint32_t ms)
{
    vTaskDelay(pdMS_TO_TICKS(ms));
}

/* Runs on the buzzer task only; may block, it owns the buzzer timeline. */
static void play_event(buzzer_event_t event)
{
    if (!s_enabled) return;
    switch (event) {
    case BUZZER_EVENT_BOOT:
        tone_ms(1568, 60); gap_ms(40); tone_ms(2093, 100);
        break;
    case BUZZER_EVENT_TICK:
        tone_ms(2093, 25);
        break;
    case BUZZER_EVENT_BAND:
        tone_ms(2093, 25); gap_ms(35); tone_ms(2093, 25);
        break;
    case BUZZER_EVENT_MENU_OPEN:
        tone_ms(1319, 50); gap_ms(25); tone_ms(1760, 70);
        break;
    case BUZZER_EVENT_MENU_CLOSE:
        tone_ms(1760, 50); gap_ms(25); tone_ms(1319, 70);
        break;
    case BUZZER_EVENT_LOCK:
        tone_ms(1760, 45); gap_ms(20); tone_ms(2217, 45); gap_ms(20); tone_ms(2637, 80);
        break;
    case BUZZER_EVENT_LOST:
        tone_ms(1976, 60); gap_ms(25); tone_ms(1319, 90);
        break;
    default:
        break;
    }
}

static void buzzer_task(void *arg)
{
    (void)arg;
    buzzer_event_t event;
    bool prox_tone_active = false;
    int64_t prox_tone_off_us = 0;
    int64_t prox_next_beep_us = 0;

    for (;;) {
        if (xQueueReceive(s_events, &event, pdMS_TO_TICKS(5)) == pdTRUE) {
            play_event(event);
            prox_tone_active = false;
            prox_next_beep_us = esp_timer_get_time();
            continue;
        }

        int interval = s_prox_interval_ms;
        if (!s_enabled || interval < 0) {
            if (prox_tone_active) {
                tone_off();
                prox_tone_active = false;
            }
            /* MUST yield: a prio-2 task spinning here without a delay
             * starves every prio-1 task on the single-core C5 (console,
             * preview, bring-up) while looking perfectly healthy from
             * prio-3 AGC telemetry. */
            vTaskDelay(pdMS_TO_TICKS(5));
            continue;
        }

        int64_t now = esp_timer_get_time();
        uint32_t freq = s_prox_freq_hz;
        if (interval == 0) {
            /* Steady tone: transmitter is very close. */
            if (!prox_tone_active) {
                tone_on(freq);
                prox_tone_active = true;
            }
        } else {
            if (prox_tone_active && now >= prox_tone_off_us) {
                tone_off();
                prox_tone_active = false;
            }
            if (!prox_tone_active && now >= prox_next_beep_us) {
                tone_on(freq);
                prox_tone_active = true;
                prox_tone_off_us = now + (int64_t)PROX_BEEP_MS * 1000;
                prox_next_beep_us = now + (int64_t)interval * 1000;
            }
        }
        /* Yield every pass: the steady-tone and inter-beep wait paths must
         * not spin at prio 2 either (same starvation bug as above). */
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

esp_err_t buzzer_init(void)
{
    const ledc_timer_config_t timer_cfg = {
        .speed_mode      = BUZZER_LEDC_MODE,
        .duty_resolution = BUZZER_LEDC_RES,
        .timer_num       = BUZZER_LEDC_TIMER,
        .freq_hz         = PROX_TONE_HZ,
        .clk_cfg         = LEDC_AUTO_CLK,
    };
    ESP_RETURN_ON_ERROR(ledc_timer_config(&timer_cfg), "buzzer", "ledc timer failed");

    const ledc_channel_config_t channel_cfg = {
        .gpio_num   = BUZZER_GPIO_NUM,
        .speed_mode = BUZZER_LEDC_MODE,
        .channel    = BUZZER_LEDC_CHANNEL,
        .timer_sel  = BUZZER_LEDC_TIMER,
        .duty       = BUZZER_DUTY_OFF,
        .hpoint     = 0,
    };
    ESP_RETURN_ON_ERROR(ledc_channel_config(&channel_cfg), "buzzer", "ledc channel failed");

    s_events = xQueueCreate(8, sizeof(buzzer_event_t));
    ESP_RETURN_ON_FALSE(s_events != NULL, ESP_ERR_NO_MEM, "buzzer", "queue alloc failed");

    ESP_RETURN_ON_FALSE(
        xTaskCreate(buzzer_task, "buzzer", 2048, NULL, 2, NULL) == pdPASS,
        ESP_ERR_NO_MEM, "buzzer", "task create failed");
    return ESP_OK;
}

void buzzer_event(buzzer_event_t event)
{
    if (s_events) {
        (void)xQueueSend(s_events, &event, 0); /* drop oldest-silent if saturated */
    }
}

void buzzer_set_enabled(bool enabled)
{
    s_enabled = enabled;
}

bool buzzer_is_enabled(void)
{
    return s_enabled;
}

void buzzer_set_proximity(int interval_ms, int freq_hz)
{
    if (freq_hz > 0) s_prox_freq_hz = (uint32_t)freq_hz;
    s_prox_interval_ms = interval_ms;
}
