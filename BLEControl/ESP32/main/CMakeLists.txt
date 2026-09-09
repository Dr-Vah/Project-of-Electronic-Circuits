idf_component_register(
    SRCS "main.c"
    INCLUDE_DIRS "."
    REQUIRES car_control
             bt
             nvs_flash
             esp_timer)
