#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

#define BT_TARGET_COUNT 2

typedef struct {
    uint16_t x;
    uint16_t y;
    uint16_t radius;
    int32_t score;
    uint8_t coverage;
} bt_ball_t;

typedef struct {
    uint16_t x;
    uint16_t y;
    uint16_t x0;
    uint16_t y0;
    uint16_t x1;
    uint16_t y1;
    uint32_t area;
    int32_t score;
    uint8_t mean_luminance;
} bt_target_t;

typedef struct {
    uint16_t frame_width;
    uint16_t frame_height;
    bool ball_valid;
    bool orange_ball_valid;
    bool target_valid;
    bt_ball_t ball; /* Stable white ball. */
    bt_ball_t orange_ball;
    bt_target_t target; /* Stable left black target. */
    uint8_t target_count;
    bt_target_t targets[BT_TARGET_COUNT]; /* Current-frame left, then right. */
} bt_detector_result_t;

/* Detector state. Allocate one instance statically or in persistent memory. */
typedef struct {
    bool ball_has_candidate;
    bt_ball_t ball_candidate;
    uint8_t ball_consecutive_frames;
    bool orange_has_candidate;
    bt_ball_t orange_candidate;
    uint8_t orange_consecutive_frames;
    bool target_has_candidate;
    bt_target_t target_candidate;
    uint8_t target_consecutive_frames;
    uint8_t ball_misses;
    uint8_t orange_misses;
    uint8_t target_misses;
    bool ball_valid;
    bool orange_valid;
    bool target_valid;
    bool have_ball_hint;
    bt_ball_t ball_hint;
    bt_ball_t stable_ball;
    bt_ball_t stable_orange;
    bt_target_t stable_target;
} bt_detector_t;

/* Clear all temporal tracking state. */
void bt_detector_init(bt_detector_t *detector);

/*
 * Process one frame and reproduce BallGoalNav's white-ball, orange-ball and
 * two-black-block detection pipeline.
 *
 * pixels must contain width*height RGB565 pixels in row-major order. Each
 * uint16_t is byte-swapped (the format emitted by esp_jpeg when
 * swap_color_bytes=1). Coordinates in result use the original frame size.
 * Call this function sequentially for every frame with the same detector.
 */
esp_err_t bt_detector_process_rgb565(bt_detector_t *detector,
                                     const uint16_t *pixels,
                                     uint16_t width,
                                     uint16_t height,
                                     bt_detector_result_t *result);

/*
 * Draw the current on-device overlay into a byte-swapped RGB565 image:
 * cyan target boxes, green/red white ball, orange/yellow orange ball,
 * blue reference-to-target line, and magenta vehicle reference marker.
 * The destination may be a scaled copy of the detected frame; coordinates are
 * scaled automatically from result->frame_width/frame_height.
 */
esp_err_t bt_visualize_rgb565(uint16_t *pixels,
                              uint16_t width,
                              uint16_t height,
                              const bt_detector_result_t *result);

/* Serialize the result consumed by the supplied browser visualization page. */
esp_err_t bt_detector_result_to_json(const bt_detector_result_t *result,
                                     char *json,
                                     size_t capacity);

/*
 * HTML page matching the current canvas overlay. It polls /frame.jpg for the
 * camera JPEG and /detection for JSON produced by the function above.
 */
const char *bt_detector_visualization_html(void);

#ifdef __cplusplus
}
#endif
