idf_component_register(
    SRCS "main.c" "camera_stream_server.c"
         "../white_ball_detector.cpp" "../white_ball_display.c"
    INCLUDE_DIRS ".."
    PRIV_REQUIRES usb esp_timer esp_psram esp_driver_gpio esp_driver_spi
                  esp_wifi esp_event esp_netif esp_http_server nvs_flash
)

