#pragma once
const void *c5vrx2_wbfm_linear80_program(void);

#include <stddef.h>
#include <stdint.h>

#include "driver/bitscrambler.h"
#include "esp_err.h"

esp_err_t c5vrx2_wbfm_q4_configure(bitscrambler_handle_t handle);
esp_err_t c5vrx2_wbfm_q4_configure_delta(bitscrambler_handle_t handle);
esp_err_t c5vrx2_wbfm_q4_configure_lut3(bitscrambler_handle_t handle);
esp_err_t c5vrx2_wbfm_q4_configure_iq5(bitscrambler_handle_t handle);
esp_err_t c5vrx2_wbfm_q4_configure_phase5(bitscrambler_handle_t handle);
esp_err_t c5vrx2_wbfm_q4_load_tx_lut(void);
esp_err_t c5vrx2_wbfm_q4_load_tx_lut3(void);
esp_err_t c5vrx2_wbfm_q4_load_tx_iq5(void);
uint32_t c5vrx2_wbfm_q4_verify_tx_iq5_lut(uint32_t *actual_hash,
                                          uint32_t *expected_hash);
size_t c5vrx2_wbfm_q4_reference(const uint8_t *input, size_t input_bytes,
                                uint8_t *output, size_t output_bytes);
size_t c5vrx2_wbfm_q4_fast_reference(const uint8_t *input,
                                     size_t input_bytes, uint8_t *output,
                                     size_t output_bytes);
size_t c5vrx2_wbfm_q4_lut3_reference(const uint8_t *input,
                                     size_t input_bytes, uint8_t *output,
                                     size_t output_bytes);
size_t c5vrx2_wbfm_q4_iq5_reference(const uint8_t *input,
                                    size_t input_bytes, uint8_t *output,
                                    size_t output_bytes);
size_t c5vrx2_wbfm_q4_phase5_reference(const uint8_t *input,
                                       size_t input_bytes, uint8_t *output,
                                       size_t output_bytes);
uint8_t c5vrx2_wbfm_q4_phase5_value(uint8_t packed);
const void *c5vrx2_wbfm_q4_program(void);
const void *c5vrx2_wbfm_q4_fast_program(void);
const void *c5vrx2_wbfm_q4_lut3_program(void);
const void *c5vrx2_wbfm_q4_iq5_program(void);
const void *c5vrx2_wbfm_q4_phase5_program(void);
#if CONFIG_C5VRX2_WBFM_SELFTEST_ONCE
/* Returns the first failing hardware stage (1..7), 8 when every stage and the
 * complete program pass, or 9 if only the integrated production program (or
 * persistence) fails after the isolated stages pass. */
unsigned c5vrx2_wbfm_q4_selftest_once(void);
#endif

esp_err_t c5vrx2_wbfm_q4_configure_trajectory(bitscrambler_handle_t handle);
const void *c5vrx2_wbfm_q4_trajectory_program(void);

esp_err_t c5vrx2_wbfm_q4_configure_true40(bitscrambler_handle_t handle);
const void *c5vrx2_wbfm_q4_true40_program(void);

const void *c5vrx2_wbfm_q4_phase5_100ns_program(void);
