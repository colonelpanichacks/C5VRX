idf_component_register(
    SRCS
        "main.c"
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

# One BitScrambler program: the proven Phase5 FM demodulator.
target_bitscrambler_add_src("fm.bsasm")
