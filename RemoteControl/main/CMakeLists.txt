idf_component_register(SRCS "tft_display.c" "main.c" "car_control.c" "fpv.c" "fpv_tcp.c" "naiwa_render.c" "naiwa_assets.c" "ble_remote.c"
    INCLUDE_DIRS "."
    REQUIRES esp_driver_gpio esp_driver_ledc esp_driver_pcnt esp_driver_spi esp_timer
             esp_wifi esp_event esp_netif nvs_flash esp_http_server esp_https_server json usb_host_uvc bt lwip
    EMBED_FILES "index.html" "tls/omni-root.cer"
    EMBED_TXTFILES "tls/server-cert.pem" "tls/server-key.pem")
