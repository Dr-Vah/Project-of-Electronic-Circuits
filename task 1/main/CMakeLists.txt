idf_component_register(SRCS "car_control.c" "main.c" "ultrasonic_sensor.c"
                            "tft_display.c"
                       INCLUDE_DIRS "."
                       REQUIRES esp_driver_gpio esp_driver_ledc esp_driver_pcnt
                                esp_driver_spi esp_timer)

