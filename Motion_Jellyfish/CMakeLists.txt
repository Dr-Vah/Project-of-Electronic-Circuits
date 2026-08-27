idf_component_register(SRCS "car_control.c"
                       INCLUDE_DIRS "include"
                       REQUIRES esp_driver_gpio esp_driver_i2c esp_driver_ledc esp_timer)
