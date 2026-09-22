idf_component_register(
    SRCS
        "main.c"
        "arc_phy.c"
        "rf.c"
        "video.c"
        "menu_raster.c"
        "buzzer.c"
        "status_led.c"
        "grab.c"
        "preview.c"
    INCLUDE_DIRS "."
    REQUIRES
        esp_wifi
        nvs_flash
        esp_event
        esp_netif
        esp_driver_parlio
        esp_driver_gpio
        esp_driver_bitscrambler
        esp_driver_usb_serial_jtag
        esp_driver_ledc
        vfs
        esp_rom
        esp_mm
        esp_timer
        freertos
        heap
        hal
        esp_hw_support
)

# BitScrambler programs: proven Phase5 default + opt-in Trajectory v2
# weak-signal experiment (runtime A/B via console 'T').
target_bitscrambler_add_src("fm.bsasm")
target_bitscrambler_add_src("fm_traj.bsasm")
