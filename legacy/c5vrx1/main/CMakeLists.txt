# SPDX-License-Identifier: GPL-3.0-only

idf_component_register(
    SRCS
        "c5vrx_main.c"
        "c5vrx_rf.c"
        "c5vrx_wifi5.c"
        "c5vrx_phy_hacks.c"
        "c5vrx_adc_dump.c"
        "c5vrx_lp_capture.S"
        "c5vrx_rf_dump_producer.c"
        "c5vrx_rf_probes.c"
        "c5vrx_channels.c"
        "c5vrx_control.c"
        "c5vrx_cvbs_out.c"
        "c5vrx_wbfm_hw.c"
        "c5vrx_stream.c"
        "c5vrx_live_pipeline.c"
        "c5vrx_capabilities.c"
        "c5vrx_live_ring_source.c"
        "c5vrx_ring_tracker.c"
        "c5vrx_cvbs_live_out.c"
        "c5vrx_cvbs_sync.c"
        "c5vrx_cvbs_levels.c"
        "c5vrx_av_clock.c"
        "c5vrx_usb_preview.c"
        "c5vrx_usb_transport.c"
        "c5vrx_bench.c"
    INCLUDE_DIRS "."
    REQUIRES
        nvs_flash
        esp_phy
        esp_wifi
        esp_event
        esp_netif
        esp_driver_gpio
        esp_driver_parlio
        esp_driver_bitscrambler
        esp_driver_usb_serial_jtag
        esp_timer
        esp_app_format
        heap
)

target_bitscrambler_add_src("c5vrx_wbfm_4to1.bsasm")
target_bitscrambler_add_src("c5vrx_wbfm_phase8_4to1.bsasm")

# The vendor adctrig() capture path is used by the normal finite-capture
# commands as well as the optional reconstructed producer.  Its fixed 64 KiB
# HP-SRAM destination must therefore be protected in every firmware profile.
target_link_options(${COMPONENT_LIB} INTERFACE
    "-T${CMAKE_CURRENT_LIST_DIR}/c5vrx_rf_dump_memory.ld")
set_property(TARGET ${COMPONENT_LIB} APPEND PROPERTY LINK_DEPENDS
    "${CMAKE_CURRENT_LIST_DIR}/c5vrx_rf_dump_memory.ld")

if(CONFIG_C5VRX_EXPERIMENTAL_RF_DUMP_PRODUCER)
    set(C5VRX_AUDITED_RFTEST "${IDF_PATH}/components/esp_phy/lib/esp32c5/librftest.a")
    if(NOT EXISTS "${C5VRX_AUDITED_RFTEST}")
        message(FATAL_ERROR "Experimental RF dump producer requires ESP32-C5 librftest.a")
    endif()
    file(SHA256 "${C5VRX_AUDITED_RFTEST}" C5VRX_RFTEST_SHA256)
    if(NOT C5VRX_RFTEST_SHA256 STREQUAL "0f9680b41612762d854accf2334412e3e01b206a3ec290bbbb232842fdaec7ba")
        message(FATAL_ERROR "Experimental RF dump producer refused unreviewed librftest.a SHA-256 ${C5VRX_RFTEST_SHA256}")
    endif()
endif()
