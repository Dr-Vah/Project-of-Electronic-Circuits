#ifndef ORANGE_BALL_DETECTOR_H
#define ORANGE_BALL_DETECTOR_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "white_ball_detector.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    bool rgb565_byte_swapped;
    uint8_t minimum_red;
    uint8_t minimum_red_green_difference;
    uint8_t minimum_red_blue_difference;
    uint16_t minimum_area_px;
    float maximum_area_fraction;
    float minimum_aspect_ratio;
    float maximum_aspect_ratio;
    float minimum_fill_ratio;
    float roi_top_fraction;
    float roi_bottom_fraction;
    uint8_t required_confirmation_frames;
    uint16_t maximum_center_jump_px;
} orange_ball_config_t;

typedef struct {
    orange_ball_config_t config;
    uint8_t mask[WHITE_BALL_MAX_PIXELS];
    uint16_t queue[WHITE_BALL_MAX_PIXELS];
    white_ball_result_t previous_candidate;
    white_ball_result_t last_stable_result;
    uint8_t confirmation_count;
    uint8_t missed_frames;
} orange_ball_detector_t;

void orange_ball_default_config(orange_ball_config_t *config);

bool orange_ball_detector_init(orange_ball_detector_t *detector,
                               const orange_ball_config_t *config);

/** Detect the saturated red-orange ball and return the common ball result. */
bool orange_ball_detect_rgb565(orange_ball_detector_t *detector,
                               const uint16_t *pixels,
                               size_t width,
                               size_t height,
                               white_ball_result_t *result);

#ifdef __cplusplus
}
#endif

#endif

