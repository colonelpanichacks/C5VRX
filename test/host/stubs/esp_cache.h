#pragma once
/* Host stub: cache sync is a no-op on the simulator (the "DMA" writes the
 * ring through host memory that the CPU reads directly). */
#include <stddef.h>
#define ESP_CACHE_MSYNC_FLAG_DIR_M2C   (1u << 0)
#define ESP_CACHE_MSYNC_FLAG_UNALIGNED (1u << 1)
static inline int esp_cache_msync(void *addr, size_t size, unsigned flags)
{
    (void)addr; (void)size; (void)flags;
    return 0;
}
