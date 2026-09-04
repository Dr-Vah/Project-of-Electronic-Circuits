#pragma once

#include <stdint.h>
#include "esp_err.h"

#define CAMERA_DISPLAY_WIDTH  128U
#define CAMERA_DISPLAY_HEIGHT 160U

esp_err_t camera_display_init(void);
esp_err_t camera_display_draw_rgb565(const uint16_t *pixels,
                                     unsigned width, unsigned height);

