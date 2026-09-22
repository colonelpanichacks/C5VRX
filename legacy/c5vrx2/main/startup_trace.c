#include "startup_trace.h"

#include <string.h>

#include "esp_partition.h"
#include "esp_timer.h"

#include "continuous_iq.h"

#define REG32(a) (*(volatile uint32_t *)(uintptr_t)(a))
#define TRACE_SUBTYPE ((esp_partition_subtype_t)0x41)
#define TRACE_MAGIC 0x52543243u
#define TRACE_VERSION 1u
#define DUMP_PTR_MODE 0x600a9008u

typedef struct __attribute__((packed)) {
    uint32_t magic;
    uint16_t version;
    uint16_t bytes;
    uint32_t sequence;
    uint32_t stage;
    int32_t error;
    uint32_t time_us;
    uint32_t rf_rate_hz;
    uint32_t writer_pointer;
    uint32_t dump_control;
    uint32_t dump_ptr_mode;
    uint32_t producer_starts;
    uint32_t physical_wraps;
    uint32_t trigger_count;
    uint32_t reserved[3];
} trace_record_t;

_Static_assert(sizeof(trace_record_t) == 64u, "trace record size");

static const esp_partition_t *s_partition;
static uint32_t s_sequence;

void c5vrx2_trace_begin(void)
{
    s_partition = esp_partition_find_first(
        ESP_PARTITION_TYPE_DATA, TRACE_SUBTYPE, "trace");
    s_sequence = 0u;
    if (!s_partition) return;
    if (esp_partition_erase_range(s_partition, 0u, s_partition->erase_size) !=
        ESP_OK) {
        s_partition = NULL;
        return;
    }
    c5vrx2_trace_stage(1u, ESP_OK);
}

void c5vrx2_trace_stage(uint32_t stage, esp_err_t error)
{
    c5vrx2_trace_stage_detail(stage, error, 0u, 0u, 0u);
}

void c5vrx2_trace_stage_detail(uint32_t stage, esp_err_t error,
                               uint32_t detail0, uint32_t detail1,
                               uint32_t detail2)
{
    /* SPI-flash writes are forbidden while the autonomous modem dump owns
     * the HP SRAM banks. Besides perturbing realtime operation, this was
     * observed to make native USB/startup fail nondeterministically. */
    if (continuous_iq_is_running()) return;
    if (!s_partition ||
        (s_sequence + 1u) * sizeof(trace_record_t) > s_partition->size)
        return;

    continuous_iq_stats_t stats;
    memset(&stats, 0, sizeof(stats));
    continuous_iq_get_stats(&stats);
    const trace_record_t record = {
        .magic = TRACE_MAGIC,
        .version = TRACE_VERSION,
        .bytes = sizeof(trace_record_t),
        .sequence = s_sequence,
        .stage = stage,
        .error = error,
        .time_us = (uint32_t)esp_timer_get_time(),
        .rf_rate_hz = stats.rf_sample_rate_hz,
        .writer_pointer = stats.writer_pointer,
        .dump_control = stats.dump_control,
        .dump_ptr_mode = REG32(DUMP_PTR_MODE),
        .producer_starts = stats.producer_start_count,
        .physical_wraps = stats.physical_wraps,
        .trigger_count = stats.trigger_count,
        .reserved = {detail0, detail1, detail2},
    };
    if (esp_partition_write(s_partition,
                            s_sequence * sizeof(trace_record_t),
                            &record, sizeof(record)) == ESP_OK)
        s_sequence++;
}
