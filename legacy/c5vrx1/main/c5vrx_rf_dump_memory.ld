/* SPDX-License-Identifier: GPL-3.0-only */

/* Fail the link before any RF build can place static application state in the
 * fixed vendor dump window. Runtime heap reservation is emitted by
 * c5vrx_rf_dump_producer.c through SOC_RESERVE_MEMORY_REGION(). */
ASSERT(_bss_end <= 0x4082ffc0,
       "C5VRX: static DRAM overlaps RF dump guard/ring 0x4082ffc0..0x4084003f")
