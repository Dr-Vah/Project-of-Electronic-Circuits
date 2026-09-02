#ifndef WHITE_BALL_DETECTOR_H
#define WHITE_BALL_DETECTOR_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define WHITE_BALL_MAX_WIDTH 240U
#define WHITE_BALL_MAX_HEIGHT 160U
#define WHITE_BALL_MAX_PIXELS (WHITE_BALL_MAX_WIDTH * WHITE_BALL_MAX_HEIGHT)

typedef struct {
    bool valid;
    uint8_t luma_threshold;
    uint8_t chroma_threshold;
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
    float diameter_px;
    float luma_contrast;
    float chroma_contrast;
    float circular_edge_score;
    float shadow_score;
    float confidence;
} white_ball_result_t;

typedef struct {
    bool rgb565_byte_swapped;
    uint16_t minimum_area_px;
    float maximum_area_fraction;
    uint8_t minimum_luma;
    uint8_t minimum_dim_luma;
    uint8_t local_highlight_difference;
    uint8_t maximum_luma;
    uint8_t maximum_chroma;
    uint8_t minimum_local_luma_contrast;
    uint8_t shadow_luma_difference;
    float minimum_aspect_ratio;
    float minimum_fill_ratio;
    float minimum_circular_edge_score;
    float minimum_confidence;
    float roi_top_fraction;
    float roi_bottom_fraction;
} white_ball_config_t;

/* Detector working memory. Allocate this once, not on a small task stack. */
typedef struct {
    white_ball_config_t config;
    uint8_t mask[WHITE_BALL_MAX_PIXELS];
    uint16_t queue[WHITE_BALL_MAX_PIXELS];
    white_ball_result_t previous_candidate;
    uint8_t confirmation_count;
    uint8_t missed_frames;
} white_ball_detector_t;

void white_ball_default_config(white_ball_config_t *config);

bool white_ball_detector_init(white_ball_detector_t *detector,
                              const white_ball_config_t *config);

/**
 * Detect the most likely white ball in an RGB565 frame.
 *
 * The detector uses low saturation relative to the local paper background,
 * connected-component shape, circular edge evidence and nearby shadow support.
 * It performs no camera, motor or patrol control.
 */
bool white_ball_detect_rgb565(white_ball_detector_t *detector,
                              const uint16_t *pixels,
                              size_t width,
                              size_t height,
                              white_ball_result_t *result);

#ifdef __cplusplus
}
#endif

#endif

