idf_component_register(SRCS "ball_transport_controller.c"
                            "white_ball_detector.cpp"
                            "ball_detector.c"
                            "orange_ball_detector.c"
                            "black_target_detector.c"
                            "car_control.c" "main.c" "ultrasonic_sensor.c"
                            "tft_display.c" "camera_line_sensor.c"
                       INCLUDE_DIRS "."
                       REQUIRES esp_driver_gpio esp_driver_ledc esp_driver_pcnt
                                esp_driver_spi esp_timer esp_psram usb)

