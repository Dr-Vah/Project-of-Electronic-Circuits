idf_component_register(SRCS "tft_display.c"
                       INCLUDE_DIRS "include"
                       REQUIRES car_control ultrasonic esp_driver_gpio
                                esp_driver_spi)
