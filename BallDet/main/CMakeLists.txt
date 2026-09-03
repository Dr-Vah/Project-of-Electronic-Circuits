idf_component_register(SRCS "main.c" "ball_detector.c" "wifi_stream.c"
                            "camera_display.c"
                       INCLUDE_DIRS "."
                       REQUIRES usb esp_timer esp_driver_gpio esp_driver_spi esp_psram
                                esp_wifi esp_event esp_netif nvs_flash esp_http_server)
