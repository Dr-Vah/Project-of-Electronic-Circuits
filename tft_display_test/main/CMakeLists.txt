idf_component_register(
    SRCS "main.c" "tft_display.c"
    INCLUDE_DIRS "."
    REQUIRES esp_driver_gpio esp_driver_spi esp_timer esp_rom
)
