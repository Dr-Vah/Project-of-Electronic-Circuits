#ifndef BLACK_TARGET_DETECTOR_H
#define BLACK_TARGET_DETECTOR_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "white_ball_detector.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    bool valid;
    uint8_t confirmation_count;
    uint16_t area_px;
    uint16_t left;
    uint16_t top;
    uint16_t right;
    uint16_t bottom;
    /* -1 is the left edge, 0 is the image centre, +1 is the right edge. */
    float center_x;
    /* 0 is the top edge and 1 is the bottom edge. */
    float center_y;
    float area_fraction;
} black_target_result_t;

typedef struct {
    bool rgb565_byte_swapped;
    uint8_t maximum_luma;
    uint16_t minimum_area_px;
    float maximum_area_fraction;
    float roi_top_fraction;
    float roi_bottom_fraction;
    uint8_t required_confirmation_frames;
} black_target_config_t;

typedef struct {
    black_target_config_t config;
    uint8_t mask[WHITE_BALL_MAX_PIXELS];
    uint16_t queue[WHITE_BALL_MAX_PIXELS];
    black_target_result_t previous_candidate;
    uint8_t confirmation_count;
    uint8_t missed_frames;
} black_target_detector_t;

void black_target_default_config(black_target_config_t *config);

bool black_target_detector_init(black_target_detector_t *detector,
                                const black_target_config_t *config);

/** Detect a connected dark floor region. With finite preferred coordinates,
 * choose the farther-away region horizontally closest to the selected ball;
 * otherwise choose the largest region.
 */
bool black_target_detect_rgb565(black_target_detector_t *detector,
                                const uint16_t *pixels,
                                size_t width,
                                size_t height,
                                float preferred_center_x,
                                float preferred_center_y,
                                black_target_result_t *result);

#ifdef __cplusplus
}
#endif

#endif
