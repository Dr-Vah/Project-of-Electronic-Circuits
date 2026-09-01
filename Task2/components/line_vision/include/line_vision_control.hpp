#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define LINE_VISION_MAX_SCAN_ROWS 8U

/**
 * Calibration for one horizontal image scan row.
 *
 * row_fraction is measured from the top of the image (0..1). forward_m is
 * the ground distance from the chassis origin. visible_width_m is the ground
 * width covered by the complete image at that row.
 */
typedef struct {
    float row_fraction;
    float forward_m;
    float visible_width_m;
} line_vision_row_calibration_t;

typedef struct {
    uint8_t scan_row_count;
    uint8_t minimum_valid_rows;
    line_vision_row_calibration_t rows[LINE_VISION_MAX_SCAN_ROWS];

    /* RGB565 detection parameters. Luma values use the range 0..255. */
    uint8_t adaptive_threshold_offset;
    uint8_t minimum_threshold;
    uint8_t maximum_threshold;
    uint8_t minimum_line_width_px;
    uint8_t maximum_line_width_px;
    float maximum_row_jump_fraction;
    bool rgb565_byte_swapped;

    /* Camera_Lazy-style four-channel virtual infrared strip. The strip is a
     * near-field rectangle split left-to-right into four equal channels. */
    uint8_t virtual_ir_threshold;
    uint8_t virtual_ir_active_percent;
    uint8_t virtual_ir_finish_percent;
    float virtual_ir_roi_top_fraction;
    float virtual_ir_roi_bottom_fraction;
    float virtual_ir_width_fraction;

    /* Chassis coordinates: +x right, +y forward. */
    float image_center_fraction;
    float lookahead_m;
    float minimum_lookahead_m;
    float lookahead_curvature_gain;
    float nominal_forward_speed_mps;
    float minimum_forward_speed_mps;
    float maximum_lateral_speed_mps;
    float maximum_angular_speed_rad_s;
    float lateral_speed_gain;
    float lateral_angular_gain;
    float heading_angular_gain;

    /* Set to -1 when positive image/ground x must command a right turn. */
    float angular_command_sign;
    float minimum_control_confidence;
} line_vision_control_config_t;

typedef struct {
    float x_m;
    float y_m;
    float image_x;
    float image_y;
} line_vision_point_t;

typedef struct {
    bool line_valid;
    bool control_valid;
    bool corner_detected;
    bool corner_turning;
    bool middle_lower_line_present;
    bool virtual_ir_valid;
    bool virtual_ir_all_black;
    uint8_t threshold;
    uint8_t point_count;
    uint8_t virtual_ir_raw_pattern;
    uint8_t virtual_ir_pattern; /* Current frame; no temporal steering filter. */
    uint8_t virtual_ir_black_percent[4];
    float confidence;
    float virtual_ir_error;
    line_vision_point_t points[LINE_VISION_MAX_SCAN_ROWS];

    /* Fitted ground curve: x(y) = a*y*y + b*y + c. */
    float curve_a;
    float curve_b;
    float curve_c;
    float lateral_error_m;
    float heading_error_rad;
    float curvature_inv_m;
    float active_lookahead_m;

    /* Piecewise-line corner estimate. Positive angle means the path bends
     * toward positive image/ground x (the right side of the image). */
    float corner_angle_rad;
    float corner_distance_m;

    /* Suggested chassis command; the caller decides whether to apply it. */
    float command_vx_mps;
    float command_vy_mps;
    float command_omega_rad_s;
    int64_t timestamp_us;
} line_vision_control_result_t;

typedef struct line_vision_control_context *line_vision_control_handle_t;

/** Fill a safe initial configuration. Ground distances are starting estimates
 * and must be replaced by measurements from the installed camera. */
void line_vision_control_default_config(line_vision_control_config_t *config);

line_vision_control_handle_t line_vision_control_create(
    const line_vision_control_config_t *config);

void line_vision_control_destroy(line_vision_control_handle_t handle);

/**
 * Detect the dark line in one RGB565 frame and calculate a chassis command.
 * This function performs no motor I/O and is intended to run in the frame
 * processing task, never in the USB frame callback.
 */
bool line_vision_control_process_rgb565(
    line_vision_control_handle_t handle,
    const uint16_t *pixels,
    size_t width,
    size_t height,
    int64_t timestamp_us,
    line_vision_control_result_t *result);

#ifdef __cplusplus
}
#endif
