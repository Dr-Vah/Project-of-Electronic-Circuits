idf_component_register(SRCS "camera_uvc.c"
                       INCLUDE_DIRS "include"
                       REQUIRES line_vision camera_debug usb esp_timer
                       PRIV_REQUIRES esp_psram)
