idf_component_register(SRCS "camera_stream_server.c"
                       INCLUDE_DIRS "include"
                       REQUIRES esp_wifi esp_event esp_netif esp_http_server
                                nvs_flash esp_psram)
