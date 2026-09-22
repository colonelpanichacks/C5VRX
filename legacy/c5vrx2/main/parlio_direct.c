#include "parlio_direct.h"

#include <stdbool.h>
#include <stdint.h>

#include "driver/parlio_bitscrambler.h"
#include "driver/parlio_tx.h"
#include "parlio_priv.h"

#include "continuous_iq.h"
#include "wbfm_direct.h"
#include "calibration.h"

static parlio_tx_unit_handle_t s_tx;
static uint32_t s_output_rate_hz;
static bool s_started;

esp_err_t c5vrx2_parlio_direct_prepare(uint32_t output_rate_hz)
{
    if (s_tx || output_rate_hz == 0u) return ESP_ERR_INVALID_STATE;
    esp_err_t err;

    /* XIAO ESP32-C5 D4..D9, LSB-first DAC ordering proven by C5VRX. */
    const parlio_tx_unit_config_t cfg = {
        .clk_src = PARLIO_CLK_SRC_DEFAULT,
        .clk_in_gpio_num = -1,
        .input_clk_src_freq_hz = 0,
        .output_clk_freq_hz = output_rate_hz,
        .data_width = 8,
        .data_gpio_nums = {23, 24, 11, 12, 8, 9, -1, -1},
        .clk_out_gpio_num = -1,
        .valid_gpio_num = -1,
        .valid_start_delay = 0,
        .valid_stop_delay = 0,
        .trans_queue_depth = 1,
        .max_transfer_size = continuous_iq_ring_bytes(),
        .dma_burst_size = 32,
        .shift_edge = PARLIO_SHIFT_EDGE_NEG,
        .bit_pack_order = PARLIO_BIT_PACK_ORDER_LSB,
    };
    if ((err = parlio_new_tx_unit(&cfg, &s_tx)) != ESP_OK) goto fail;
    /* Use the IDF-owned decoration so the transform is reset and started in
     * parlio_tx_do_transaction(), after FIFO reset and immediately before
     * GDMA. The pinned 6.0.1 private handle is needed only to install the
     * runtime-calibrated LUT which the public decoration API does not expose. */
    if ((err = parlio_tx_unit_decorate_bitscrambler(s_tx)) != ESP_OK)
        goto fail;
    if ((err = c5vrx2_wbfm_direct_configure(s_tx->bs_handle)) != ESP_OK)
        goto fail;
    if ((err = parlio_tx_unit_enable(s_tx)) != ESP_OK) goto fail;

    s_output_rate_hz = output_rate_hz;
    return ESP_OK;

fail:
    c5vrx2_parlio_direct_destroy();
    return err;
}

esp_err_t c5vrx2_parlio_direct_start(void)
{
    if (!s_tx || s_started) return ESP_ERR_INVALID_STATE;
    const c5vrx2_calibration_t *cal = c5vrx2_calibration_get();

    const parlio_transmit_config_t tx_cfg = {
        .idle_value = cal->pedestal_code,
        .bitscrambler_program = c5vrx2_wbfm_direct_program(),
        .flags.loop_transmission = true,
    };
    const esp_err_t err = parlio_tx_unit_transmit(
        s_tx,
        continuous_iq_ring_base(),
        continuous_iq_ring_bytes() * 8u,
        &tx_cfg);
    if (err == ESP_OK) s_started = true;
    return err;
}

void c5vrx2_parlio_direct_destroy(void)
{
    c5vrx2_parlio_direct_quiesce();
    if (s_tx) {
        (void)parlio_tx_unit_disable(s_tx);
        (void)parlio_tx_unit_undecorate_bitscrambler(s_tx);
        (void)parlio_del_tx_unit(s_tx);
        s_tx = NULL;
    }
    s_output_rate_hz = 0u;
    s_started = false;
}

void c5vrx2_parlio_direct_quiesce(void)
{
    /* A loop transmission is intentionally never stopped in the healthy
     * realtime path. Unit disable during explicit teardown is the only
     * supported quiesce operation; no idle/restart exists between buffers. */
}

uint32_t c5vrx2_parlio_direct_rate_hz(void)
{
    return s_output_rate_hz;
}
