idf_component_register(
    SRCS "main.c" "camera_stream_server.c" "ball_transport_controller.c"
         "../white_ball_detector.cpp" "../ball_detector.c"
         "../orange_ball_detector.c"
         "../black_target_detector.c"
         "../white_ball_display.c"
    INCLUDE_DIRS ".."
    PRIV_REQUIRES usb esp_timer esp_psram esp_driver_gpio esp_driver_spi
                  esp_wifi esp_event esp_netif esp_http_server nvs_flash
                  car_control
)
