idf_component_register(
    SRCS
        "main.c"
        "startup_trace.c"
        "calibration.c"
        "continuous_iq.c"
        "diagnostics.c"
        "iq_decode.c"
        "wifi5.c"
        "rf_dump.c"
        "wbfm_direct.c"
        "wbfm_q4.c"
        "trajectory_reference.c"
        "true40_reference.c"
        "parlio_direct.c"
        "realtime.c"
        "issue11_capture.c"
        "tx_80m_oracle.c"
        "linear80_oracle.c"
    INCLUDE_DIRS "."
    # diagnostics.c intentionally inspects the C5 PARLIO unit/GDMA handle for
    # byte-exact hardware oracles; this repository is pinned to IDF v6.0.1.
    PRIV_INCLUDE_DIRS
        "${IDF_PATH}/components/esp_driver_parlio/src"
    REQUIRES
        nvs_flash
        esp_partition
        esp_phy
        esp_wifi
        esp_event
        esp_netif
        esp_driver_parlio
        esp_driver_gpio
        esp_driver_bitscrambler
        esp_driver_dma
        esp_mm
        esp_pm
        esp_timer
        heap
)

target_bitscrambler_add_src("c5vrx2_wbfm_direct6_4to1.bsasm")
target_bitscrambler_add_src("c5vrx2_wbfm_q4_2to1.bsasm")
target_bitscrambler_add_src("c5vrx2_wbfm_q4_fast_2to1.bsasm")
target_bitscrambler_add_src("c5vrx2_wbfm_q4_lut3_2to1.bsasm")
target_bitscrambler_add_src("c5vrx2_wbfm_q4_iq5_2to1.bsasm")
target_bitscrambler_add_src("c5vrx2_wbfm_q4_phase5_2to1.bsasm")
target_bitscrambler_add_src("c5vrx2_phase5_linear80.bsasm")
target_bitscrambler_add_src("c5vrx2_phase5_lookup_probe.bsasm")
target_bitscrambler_add_src("c5vrx2_tx_2to1_probe.bsasm")
target_bitscrambler_add_src("c5vrx2_q4_phase.bsasm")
target_bitscrambler_add_src("c5vrx2_q4_negative.bsasm")
target_bitscrambler_add_src("c5vrx2_q4_state.bsasm")
target_bitscrambler_add_src("c5vrx2_q4_delta.bsasm")
target_bitscrambler_add_src("c5vrx2_q4_pairsum.bsasm")
target_bitscrambler_add_src("c5vrx2_counter_lut.bsasm")
target_bitscrambler_add_src("c5vrx2_input_passthrough.bsasm")
target_bitscrambler_add_src("c5vrx2_q4_address_conventional.bsasm")
target_bitscrambler_add_src("c5vrx2_counter_lut_conventional.bsasm")

target_link_options(${COMPONENT_LIB} INTERFACE
    "-T${CMAKE_CURRENT_LIST_DIR}/rf_dump_memory.ld")
set_property(TARGET ${COMPONENT_LIB} APPEND PROPERTY LINK_DEPENDS
    "${CMAKE_CURRENT_LIST_DIR}/rf_dump_memory.ld")

target_bitscrambler_add_src("c5vrx2_wbfm_q4_trajectory_2to1.bsasm")
target_bitscrambler_add_src("c5vrx2_wbfm_q4_true40_2to1.bsasm")
target_bitscrambler_add_src("c5vrx2_wbfm_q4_phase5_100ns_2to1.bsasm")
