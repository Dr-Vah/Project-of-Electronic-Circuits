idf_component_register(
    SRCS "main.c" "camera_display.c"
    INCLUDE_DIRS "."
    PRIV_REQUIRES usb esp_timer esp_driver_gpio esp_driver_spi esp_psram
)
