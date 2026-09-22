#pragma once
/* Host stub: only the fields grab.c touches. */
typedef struct {
    struct { unsigned rx_fifo_wovf_int_clr; } int_clr;
    struct { unsigned rx_fifo_wovf_int_raw; } int_raw;
} parl_io_dev_t;
static parl_io_dev_t PARL_IO;
