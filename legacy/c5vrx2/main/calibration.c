#include "calibration.h"

#include <stddef.h>
#include <string.h>

#include "esp_log.h"
#include "esp_partition.h"

#define CALIBRATION_MAGIC 0x41433543u /* little-endian "C5CA" */
#define CALIBRATION_VERSION 1u
#define CALIBRATION_SUBTYPE ((esp_partition_subtype_t)0x40)

typedef struct __attribute__((packed)) {
    uint32_t magic;
    uint16_t version;
    uint16_t length;
    uint8_t pedestal_code;
    uint8_t discriminator_gain;
    uint8_t polarity;
    uint8_t reserved;
    uint32_t output_clock_hz;
    uint32_t crc32;
} calibration_record_t;

_Static_assert(sizeof(calibration_record_t) == 20u,
               "calibration record layout changed");

static const char *TAG = "c5vrx2_cal";
static c5vrx2_calibration_t s_calibration;

static uint32_t crc32_ieee(const void *data, size_t length)
{
    const uint8_t *bytes = data;
    uint32_t crc = UINT32_MAX;
    for (size_t i = 0; i < length; ++i) {
        crc ^= bytes[i];
        for (unsigned bit = 0; bit < 8u; ++bit)
            crc = (crc >> 1u) ^ (0xedb88320u & (0u - (crc & 1u)));
    }
    return crc ^ UINT32_MAX;
}

static void load_defaults(void)
{
    s_calibration = (c5vrx2_calibration_t) {
        .pedestal_code = 20u,
        .discriminator_gain = 2u,
        .polarity = C5VRX2_POLARITY_CURRENT_MINUS_PREVIOUS,
        .output_clock_hz = 20000000u,
        .loaded_from_flash = false,
    };
}

void c5vrx2_calibration_load(void)
{
    load_defaults();
    const esp_partition_t *partition = esp_partition_find_first(
        ESP_PARTITION_TYPE_DATA, CALIBRATION_SUBTYPE, "calib");
    if (!partition) {
        ESP_LOGW(TAG, "calibration partition absent; using measured defaults");
        return;
    }

    calibration_record_t record;
    const esp_err_t err = esp_partition_read(partition, 0u, &record,
                                             sizeof(record));
    const uint32_t expected_crc = crc32_ieee(
        &record, offsetof(calibration_record_t, crc32));
    const bool valid = err == ESP_OK &&
        record.magic == CALIBRATION_MAGIC &&
        record.version == CALIBRATION_VERSION &&
        record.length == sizeof(record) &&
        record.crc32 == expected_crc &&
        record.pedestal_code >= 12u && record.pedestal_code <= 28u &&
        record.discriminator_gain >= 1u &&
        record.discriminator_gain <= 4u &&
        record.polarity <= C5VRX2_POLARITY_PREVIOUS_MINUS_CURRENT &&
        /* 10 MHz is accepted only as a diagnostic consumer-rate test. It
         * does not preserve the 80/4 MHz timebase of received CVBS. */
        record.output_clock_hz >= 10000000u &&
        record.output_clock_hz <= 20500000u;
    if (!valid) {
        ESP_LOGW(TAG, "calibration empty/invalid; using measured defaults");
        return;
    }

    s_calibration = (c5vrx2_calibration_t) {
        .pedestal_code = record.pedestal_code,
        .discriminator_gain = record.discriminator_gain,
        .polarity = (c5vrx2_polarity_t)record.polarity,
        .output_clock_hz = record.output_clock_hz,
        .loaded_from_flash = true,
    };
    ESP_LOGW(TAG,
             "calibration loaded: pedestal=%u gain=%ux polarity=%s clock=%u Hz",
             s_calibration.pedestal_code,
             s_calibration.discriminator_gain,
             s_calibration.polarity ==
                 C5VRX2_POLARITY_CURRENT_MINUS_PREVIOUS ?
                     "current-minus-previous" : "previous-minus-current",
             (unsigned)s_calibration.output_clock_hz);
}

const c5vrx2_calibration_t *c5vrx2_calibration_get(void)
{
    return &s_calibration;
}
