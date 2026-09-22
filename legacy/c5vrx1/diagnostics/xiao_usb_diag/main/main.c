#include <string.h>
#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/usb_serial_jtag.h"
#include "esp_err.h"

#define RX_BUF 256

static void usb_write_line(const char *s)
{
    if (!s) return;
    usb_serial_jtag_write_bytes(s, strlen(s), pdMS_TO_TICKS(100));
}

void app_main(void)
{
    /* Use the IDF-provided default initializer instead of naming fields that
     * differ between ESP-IDF point releases. Override only the ring sizes we
     * actually care about for this diagnostic. */
    usb_serial_jtag_driver_config_t cfg = USB_SERIAL_JTAG_DRIVER_CONFIG_DEFAULT();
    cfg.rx_buffer_size = 1024;
    cfg.tx_buffer_size = 1024;

    esp_err_t err = usb_serial_jtag_driver_install(&cfg);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        while (1) {
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }

    usb_write_line("C5VRX_RAWUSB_READY protocol=1\n");

    char line[RX_BUF];
    size_t used = 0;
    TickType_t last_heartbeat = xTaskGetTickCount();

    for (;;) {
        uint8_t buf[64];
        int n = usb_serial_jtag_read_bytes(buf, sizeof(buf), pdMS_TO_TICKS(20));
        for (int i = 0; i < n; ++i) {
            char c = (char)buf[i];
            if (c == '\r') continue;
            if (c == '\n') {
                line[used] = '\0';
                if (strcmp(line, "PING") == 0) {
                    usb_write_line("C5VRX_PONG rawusb=1\n");
                } else if (strcmp(line, "STATUS") == 0) {
                    usb_write_line("C5VRX_USB_DIAG_STATUS app=running rawusb=1 rf=0 wifi=0 stdio=0\n");
                } else if (used) {
                    usb_write_line("C5VRX_USB_DIAG_ECHO ");
                    usb_serial_jtag_write_bytes(line, used, pdMS_TO_TICKS(100));
                    usb_write_line("\n");
                }
                used = 0;
            } else if (used + 1 < sizeof(line)) {
                line[used++] = c;
            } else {
                used = 0;
            }
        }

        TickType_t now = xTaskGetTickCount();
        if ((now - last_heartbeat) >= pdMS_TO_TICKS(1000)) {
            last_heartbeat = now;
            if (usb_serial_jtag_is_connected()) {
                usb_write_line("C5VRX_RAWUSB_HEARTBEAT app=running\n");
            }
        }
    }
}
