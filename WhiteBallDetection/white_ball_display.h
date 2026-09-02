#ifndef WHITE_BALL_DISPLAY_H
#define WHITE_BALL_DISPLAY_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"
#include "white_ball_detector.h"

#ifdef __cplusplus
extern "C" {
#endif

/** Initialize the 128x160 ST7735S debug screen. */
esp_err_t white_ball_display_init(void);

/**
 * Display a colour RGB565 camera frame in real time.
 *
 * The image keeps its aspect ratio and is centred on the screen. When ball is
 * valid, a green rectangle and cyan centre cross are drawn over the detected
 * white ball. Passing NULL for ball displays the camera image without overlay.
 */
esp_err_t white_ball_display_show_rgb565(
    const uint16_t *pixels,
    size_t width,
    size_t height,
    bool rgb565_byte_swapped,
    const white_ball_result_t *ball);

#ifdef __cplusplus
}
#endif

#endif

