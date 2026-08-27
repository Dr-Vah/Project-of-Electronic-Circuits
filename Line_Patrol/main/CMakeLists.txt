idf_component_register(SRCS "patrol.c" "car_control.c"
                       INCLUDE_DIRS "."
                       REQUIRES esp_driver_gpio esp_driver_ledc esp_timer)
