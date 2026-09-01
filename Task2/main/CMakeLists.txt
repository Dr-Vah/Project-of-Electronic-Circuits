idf_component_register(SRCS "app_main.c"
                       INCLUDE_DIRS "."
                       REQUIRES car_control ultrasonic camera_uvc
                                course_control tft_display)
