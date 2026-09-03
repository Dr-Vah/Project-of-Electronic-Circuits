#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

typedef struct {
    uint16_t x;
    uint16_t y;
    uint16_t radius;
    int32_t score;
    uint8_t coverage;
} ball_detection_t;

typedef struct {
    bool found;
    ball_detection_t ball;
} ball_detector_result_t;

typedef struct {
    bool has_candidate;
    ball_detection_t candidate;
    uint8_t consecutive_frames;
    uint8_t required_frames;
    uint16_t max_center_jump;
} ball_detector_tracker_t;

/* pixels are byte-swapped RGB565 as produced by esp_jpeg with
 * swap_color_bytes=1. The function works at at most 320 pixels wide and
 * reports coordinates in the original input frame. */
esp_err_t ball_detector_process_rgb565(const uint16_t *pixels,
                                       uint16_t width, uint16_t height,
                                       const ball_detection_t *search_hint,
                                       ball_detector_result_t *result);

void ball_detector_tracker_init(ball_detector_tracker_t *tracker,
                                uint8_t required_frames,
                                uint16_t max_center_jump);

bool ball_detector_tracker_update(ball_detector_tracker_t *tracker,
                                  const ball_detector_result_t *result,
                                  ball_detection_t *stable_ball);
