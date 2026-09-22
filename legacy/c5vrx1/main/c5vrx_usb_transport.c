/* SPDX-License-Identifier: GPL-3.0-only */

#include "c5vrx_usb_transport.h"

#include <stdarg.h>
#include <stdio.h>

#include "driver/usb_serial_jtag.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

static StaticSemaphore_t s_tx_mutex_storage;
static SemaphoreHandle_t s_tx_mutex;
/* Formatting this on each caller's stack consumed half of the 4 KiB USB
 * control-task stack.  A queued capture can log while that task is already
 * deep in the RF capture path, which caused a reproducible stack exception.
 * The transport mutex also makes a shared formatting buffer safe. */
static char s_printf_buffer[2048];

#define USB_TOTAL_WRITE_DEADLINE_MS 2500u

esp_err_t c5vrx_usb_transport_init(void)
{
    if (!s_tx_mutex) {
        s_tx_mutex = xSemaphoreCreateMutexStatic(&s_tx_mutex_storage);
    }
    return s_tx_mutex ? ESP_OK : ESP_ERR_NO_MEM;
}

static esp_err_t write_locked(const uint8_t *data, size_t size,
                              TickType_t packet_start,
                              TickType_t packet_deadline)
{
    size_t offset = 0;
    unsigned empty_writes = 0;
    while (offset < size) {
        if (xTaskGetTickCount() - packet_start >= packet_deadline)
            return ESP_ERR_TIMEOUT;
        const int written = usb_serial_jtag_write_bytes(
            data + offset, size - offset, pdMS_TO_TICKS(100));
        if (written < 0) return ESP_FAIL;
        if (written == 0) {
            if (++empty_writes >= 20u) return ESP_ERR_TIMEOUT;
            vTaskDelay(1);
            continue;
        }
        empty_writes = 0;
        offset += (size_t)written;
    }
    return ESP_OK;
}

esp_err_t c5vrx_usb_writev(const c5vrx_usb_iovec_t *parts, size_t part_count)
{
    if ((!parts && part_count) || !s_tx_mutex) return ESP_ERR_INVALID_STATE;
    if (xSemaphoreTake(s_tx_mutex, pdMS_TO_TICKS(2500)) != pdTRUE)
        return ESP_ERR_TIMEOUT;

    esp_err_t result = ESP_OK;
    const TickType_t packet_start = xTaskGetTickCount();
    const TickType_t packet_deadline =
        pdMS_TO_TICKS(USB_TOTAL_WRITE_DEADLINE_MS);
    for (size_t i = 0; i < part_count; ++i) {
        if (!parts[i].size) continue;
        if (!parts[i].data) {
            result = ESP_ERR_INVALID_ARG;
            break;
        }
        result = write_locked(parts[i].data, parts[i].size,
                              packet_start, packet_deadline);
        if (result != ESP_OK) break;
    }
    xSemaphoreGive(s_tx_mutex);
    return result;
}

esp_err_t c5vrx_usb_write(const void *data, size_t size)
{
    const c5vrx_usb_iovec_t part = {.data = data, .size = size};
    return c5vrx_usb_writev(&part, 1u);
}

int c5vrx_usb_vprintf(const char *format, va_list args)
{
    if (!s_tx_mutex) return -1;
    if (xSemaphoreTake(s_tx_mutex, pdMS_TO_TICKS(2500)) != pdTRUE) return -1;

    const int required = vsnprintf(
        s_printf_buffer, sizeof(s_printf_buffer), format, args);
    if (required < 0) {
        xSemaphoreGive(s_tx_mutex);
        return required;
    }
    const size_t count = (size_t)required < sizeof(s_printf_buffer)
                             ? (size_t)required
                             : sizeof(s_printf_buffer) - 1u;
    const esp_err_t result = write_locked(
        (const uint8_t *)s_printf_buffer, count, xTaskGetTickCount(),
        pdMS_TO_TICKS(USB_TOTAL_WRITE_DEADLINE_MS));
    xSemaphoreGive(s_tx_mutex);
    return result == ESP_OK ? required : -1;
}

int c5vrx_usb_printf(const char *format, ...)
{
    va_list args;
    va_start(args, format);
    const int result = c5vrx_usb_vprintf(format, args);
    va_end(args);
    return result;
}
