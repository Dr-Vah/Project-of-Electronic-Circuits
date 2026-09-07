idf_component_register(SRCS "ball_transport_controller.c"
                            "ball_target_detector.c"
                            "car_control.c" "main.c" "ultrasonic_sensor.c"
                            "tft_display.c" "camera_line_sensor.c"
                       INCLUDE_DIRS "."
                       REQUIRES esp_driver_gpio esp_driver_ledc esp_driver_pcnt
                                esp_driver_spi esp_timer esp_psram usb)

