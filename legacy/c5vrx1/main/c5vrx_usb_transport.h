/* SPDX-License-Identifier: GPL-3.0-only */
#pragma once

#include <stdarg.h>
#include <stddef.h>

#include "esp_err.h"

typedef struct {
    const void *data;
    size_t size;
} c5vrx_usb_iovec_t;

esp_err_t c5vrx_usb_transport_init(void);
esp_err_t c5vrx_usb_write(const void *data, size_t size);
esp_err_t c5vrx_usb_writev(const c5vrx_usb_iovec_t *parts, size_t part_count);
int c5vrx_usb_printf(const char *format, ...) __attribute__((format(printf, 1, 2)));
int c5vrx_usb_vprintf(const char *format, va_list args);
