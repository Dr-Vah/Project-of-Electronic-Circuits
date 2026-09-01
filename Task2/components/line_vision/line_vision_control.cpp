#include "line_vision_control.hpp"

#include <math.h>
#include <stdlib.h>
#include <string.h>

struct line_vision_control_context {
    line_vision_control_config_t config;
    float previous_center_fraction;
    bool has_previous_center;
    uint8_t corner_candidate_frames;
    uint8_t corner_missed_frames;
    int8_t corner_candidate_direction;
    bool corner_latched;
    float latched_corner_angle_rad;
    float latched_corner_distance_m;
    uint8_t corner_gap_frames;
    uint8_t corner_reacquire_frames;
    uint8_t corner_turn_frames;
    bool corner_turning;
};

namespace {

float clamp_float(float value, float low, float high)
{
    return value < low ? low : (value > high ? high : value);
}

struct line_fit_result_t {
    float slope;
    float intercept;
    float mean_squared_error;
};

bool fit_line(const line_vision_point_t *points, uint8_t begin, uint8_t end,
              line_fit_result_t *fit)
{
    const uint8_t count = end - begin;
    if (count < 2U || fit == nullptr) return false;
    float sum_y = 0.0f, sum_x = 0.0f, sum_yy = 0.0f, sum_yx = 0.0f;
    for (uint8_t i = begin; i < end; ++i) {
        const float y = points[i].y_m;
        const float x = points[i].x_m;
        sum_y += y;
        sum_x += x;
        sum_yy += y * y;
        sum_yx += y * x;
    }
    const float denominator = count * sum_yy - sum_y * sum_y;
    if (fabsf(denominator) < 1.0e-6f) return false;
    fit->slope = (count * sum_yx - sum_y * sum_x) / denominator;
    fit->intercept = (sum_x - fit->slope * sum_y) / count;
    float squared_error = 0.0f;
    for (uint8_t i = begin; i < end; ++i) {
        const float residual = points[i].x_m -
                               (fit->slope * points[i].y_m + fit->intercept);
        squared_error += residual * residual;
    }
    fit->mean_squared_error = squared_error / count;
    return true;
}

bool detect_piecewise_corner(const line_vision_point_t *points, uint8_t count,
                             float *angle_out, float *distance_out)
{
    if (count < 5U || angle_out == nullptr || distance_out == nullptr) {
        return false;
    }
    float best_score = 0.0f;
    float best_angle = 0.0f;
    float best_distance = 0.0f;
    for (uint8_t split = 2U; split + 2U <= count; ++split) {
        line_fit_result_t near_fit = {};
        line_fit_result_t far_fit = {};
        if (!fit_line(points, 0U, split, &near_fit) ||
            !fit_line(points, split, count, &far_fit)) {
            continue;
        }
        const float angle = atanf(far_fit.slope) - atanf(near_fit.slope);
        const float corner_y =
            0.5f * (points[split - 1U].y_m + points[split].y_m);
        const float near_x = near_fit.slope * corner_y + near_fit.intercept;
        const float far_x = far_fit.slope * corner_y + far_fit.intercept;
        const float join_gap = fabsf(near_x - far_x);
        if (join_gap > 0.14f) continue;
        const float fit_penalty =
            8.0f * (near_fit.mean_squared_error +
                    far_fit.mean_squared_error) + 1.5f * join_gap;
        const float score = fabsf(angle) - fit_penalty;
        if (score > best_score) {
            best_score = score;
            best_angle = angle;
            best_distance = corner_y;
        }
    }
    /* Perspective compresses the apparent angle, so this threshold is lower
     * than the physical course angle. One valid frame latches the warning. */
    if (best_score < 0.32f || fabsf(best_angle) < 0.32f) return false;
    *angle_out = best_angle;
    *distance_out = best_distance;
    return true;
}

void clear_corner_tracking(line_vision_control_handle_t context)
{
    context->corner_candidate_frames = 0U;
    context->corner_missed_frames = 0U;
    context->corner_candidate_direction = 0;
    context->corner_latched = false;
    context->corner_gap_frames = 0U;
    context->corner_reacquire_frames = 0U;
    context->corner_turn_frames = 0U;
    context->corner_turning = false;
}

void update_corner_tracking(line_vision_control_handle_t context,
                            line_vision_control_result_t *result,
                            bool middle_lower_line_present)
{
    float raw_angle = 0.0f;
    float raw_distance = 0.0f;
    const bool raw_corner = detect_piecewise_corner(
        result->points, result->point_count, &raw_angle, &raw_distance);
    if (!context->corner_turning && raw_corner) {
        const int8_t direction = raw_angle >= 0.0f ? 1 : -1;
        if (direction == context->corner_candidate_direction) {
            if (context->corner_candidate_frames < UINT8_MAX) {
                ++context->corner_candidate_frames;
            }
        } else {
            context->corner_candidate_direction = direction;
            context->corner_candidate_frames = 1U;
        }
        context->corner_missed_frames = 0U;
        context->latched_corner_angle_rad = raw_angle;
        context->latched_corner_distance_m = raw_distance;
        if (context->corner_candidate_frames >= 1U) {
            context->corner_latched = true;
        }
    } else if (!context->corner_turning && context->corner_latched) {
        if (middle_lower_line_present) {
            if (++context->corner_missed_frames >= 15U) {
                clear_corner_tracking(context);
            }
        }
    } else if (!context->corner_turning) {
        context->corner_candidate_frames = 0U;
        context->corner_candidate_direction = 0;
    }

    /* Detection is an early warning. After it, one frame with either
     * middle-lower row missing starts the memorized turn immediately. */
    if (context->corner_latched && !context->corner_turning) {
        if (!middle_lower_line_present) {
            if (context->corner_gap_frames < UINT8_MAX) {
                ++context->corner_gap_frames;
            }
            if (context->corner_gap_frames >= 1U) {
                context->corner_turning = true;
                context->corner_turn_frames = 0U;
                context->corner_reacquire_frames = 0U;
            }
        } else {
            context->corner_gap_frames = 0U;
        }
    } else if (context->corner_turning) {
        if (context->corner_turn_frames < UINT8_MAX) {
            ++context->corner_turn_frames;
        }
        if (middle_lower_line_present && result->point_count >= 3U) {
            if (context->corner_reacquire_frames < UINT8_MAX) {
                ++context->corner_reacquire_frames;
            }
        } else {
            context->corner_reacquire_frames = 0U;
        }
        if (context->corner_reacquire_frames >= 3U ||
            context->corner_turn_frames >= 45U) {
            clear_corner_tracking(context);
        }
    }

    result->corner_detected = context->corner_latched;
    result->corner_turning = context->corner_turning;
    result->middle_lower_line_present = middle_lower_line_present;
    if (context->corner_latched) {
        result->corner_angle_rad = context->latched_corner_angle_rad;
        result->corner_distance_m = context->latched_corner_distance_m;
    }
}

uint8_t rgb565_luma(uint16_t pixel, bool byte_swapped)
{
    if (byte_swapped) {
        pixel = static_cast<uint16_t>((pixel << 8) | (pixel >> 8));
    }
    const uint32_t red = ((pixel >> 11) & 0x1fU) * 255U / 31U;
    const uint32_t green = ((pixel >> 5) & 0x3fU) * 255U / 63U;
    const uint32_t blue = (pixel & 0x1fU) * 255U / 31U;
    return static_cast<uint8_t>((77U * red + 150U * green + 29U * blue) >> 8);
}

void detect_virtual_ir(line_vision_control_handle_t context,
                       const uint16_t *pixels, size_t width, size_t height,
                       line_vision_control_result_t *result)
{
    const line_vision_control_config_t &config = context->config;
    const size_t y0 = static_cast<size_t>(
        config.virtual_ir_roi_top_fraction * (height - 1U));
    size_t y1 = static_cast<size_t>(
        config.virtual_ir_roi_bottom_fraction * height);
    if (y1 > height) y1 = height;
    if (y1 <= y0) return;

    size_t roi_width = static_cast<size_t>(
        config.virtual_ir_width_fraction * width);
    if (roi_width < 4U) roi_width = 4U;
    if (roi_width > width) roi_width = width;
    const size_t x0 = (width - roi_width) / 2U;
    uint32_t dark[4] = {};
    uint32_t total[4] = {};

    for (size_t y = y0; y < y1; ++y) {
        for (size_t x = x0; x < x0 + roi_width; ++x) {
            size_t channel = (x - x0) * 4U / roi_width;
            if (channel > 3U) channel = 3U;
            ++total[channel];
            if (rgb565_luma(pixels[y * width + x],
                            config.rgb565_byte_swapped) <=
                config.virtual_ir_threshold) {
                ++dark[channel];
            }
        }
    }

    static const float weights[4] = {-3.0f, -1.0f, 1.0f, 3.0f};
    float weighted_sum = 0.0f;
    uint8_t active_count = 0U;
    for (uint8_t channel = 0U; channel < 4U; ++channel) {
        result->virtual_ir_black_percent[channel] =
            total[channel] == 0U
                ? 0U
                : static_cast<uint8_t>(dark[channel] * 100U /
                                       total[channel]);
        const bool raw_active = total[channel] != 0U &&
            dark[channel] * 100U >=
                total[channel] * config.virtual_ir_active_percent;
        if (raw_active) {
            result->virtual_ir_raw_pattern |=
                static_cast<uint8_t>(1U << (3U - channel));
        }
        if (raw_active) {
            result->virtual_ir_pattern |=
                static_cast<uint8_t>(1U << (3U - channel));
            weighted_sum += weights[channel];
            ++active_count;
        }
    }

    result->virtual_ir_all_black = true;
    for (uint8_t channel = 0U; channel < 4U; ++channel) {
        if (result->virtual_ir_black_percent[channel] <
            config.virtual_ir_finish_percent) {
            result->virtual_ir_all_black = false;
            break;
        }
    }
    result->virtual_ir_valid = active_count != 0U &&
                               !result->virtual_ir_all_black;
    if (active_count != 0U) {
        result->virtual_ir_error = weighted_sum / active_count;
    }
}

bool apply_virtual_ir_command(const line_vision_control_config_t &config,
                              line_vision_control_result_t *result)
{
    if (!result->virtual_ir_valid) return false;

    result->control_valid = true;
    result->command_vx_mps = 0.0f;
    result->command_vy_mps = config.nominal_forward_speed_mps;
    float omega = 0.0f;
    if (result->virtual_ir_error <= -1.5f) {
        omega = 0.28f;
        result->command_vy_mps = 0.75f * config.nominal_forward_speed_mps;
    } else if (result->virtual_ir_error >= 1.5f) {
        omega = -0.28f;
        result->command_vy_mps = 0.75f * config.nominal_forward_speed_mps;
    } else if (result->virtual_ir_pattern == 0x04U) {
        omega = 0.10f;
    } else if (result->virtual_ir_pattern == 0x02U) {
        omega = -0.10f;
    }
    result->command_omega_rad_s = omega;
    return true;
}

uint8_t adaptive_threshold(const line_vision_control_config_t &config,
                           const uint16_t *pixels, size_t width, size_t height)
{
    uint64_t luma_sum = 0;
    size_t sample_count = 0;
    for (uint8_t i = 0; i < config.scan_row_count; ++i) {
        size_t row = static_cast<size_t>(
            config.rows[i].row_fraction * static_cast<float>(height - 1U));
        if (row >= height) row = height - 1U;
        for (size_t x = 0; x < width; x += 2U) {
            luma_sum += rgb565_luma(pixels[row * width + x],
                                    config.rgb565_byte_swapped);
            ++sample_count;
        }
    }
    if (sample_count == 0U) return config.minimum_threshold;

    int threshold = static_cast<int>(luma_sum / sample_count) -
                    config.adaptive_threshold_offset;
    if (threshold < config.minimum_threshold) {
        threshold = config.minimum_threshold;
    }
    if (threshold > config.maximum_threshold) {
        threshold = config.maximum_threshold;
    }
    return static_cast<uint8_t>(threshold);
}

bool find_line_center(const line_vision_control_config_t &config,
                      const uint16_t *pixels, size_t width, size_t row,
                      uint8_t threshold, float expected_center,
                      float *center_out, float *contrast_out)
{
    bool found = false;
    int best_length = 0;
    float best_center = 0.0f;
    float best_score = -1.0e9f;
    float best_contrast = 0.0f;
    int run_start = -1;
    uint32_t run_luma_sum = 0;
    uint32_t run_weight_sum = 0;
    float run_weighted_x_sum = 0.0f;

    for (size_t x = 0; x <= width; ++x) {
        const bool at_end = x == width;
        const uint8_t luma = at_end
                                 ? 255U
                                 : rgb565_luma(pixels[row * width + x],
                                               config.rgb565_byte_swapped);
        const bool dark = !at_end && luma < threshold;

        if (dark) {
            if (run_start < 0) {
                run_start = static_cast<int>(x);
                run_luma_sum = 0;
                run_weight_sum = 0;
                run_weighted_x_sum = 0.0f;
            }
            run_luma_sum += luma;
            const uint32_t weight =
                static_cast<uint32_t>(threshold - luma) + 1U;
            run_weight_sum += weight;
            run_weighted_x_sum += static_cast<float>(x * weight);
            continue;
        }

        if (run_start < 0) continue;
        const int run_length = static_cast<int>(x) - run_start;
        if (run_length >= config.minimum_line_width_px &&
            run_length <= config.maximum_line_width_px) {
            const float center = run_weight_sum == 0U
                                     ? run_start + 0.5f * (run_length - 1)
                                     : run_weighted_x_sum / run_weight_sum;
            const float distance = fabsf(center - expected_center);
            const float mean_luma =
                static_cast<float>(run_luma_sum) / run_length;
            const float contrast = threshold - mean_luma;
            const float score = contrast + 0.75f * run_length - 0.35f * distance;
            if (score > best_score) {
                best_score = score;
                found = true;
                best_length = run_length;
                best_center = center;
                best_contrast = contrast;
            }
        }
        run_start = -1;
        run_luma_sum = 0;
        run_weight_sum = 0;
        run_weighted_x_sum = 0.0f;
    }

    if (!found || best_length == 0) return false;
    *center_out = best_center;
    *contrast_out = best_contrast;
    return true;
}

bool solve_three_by_three(float matrix[3][4], float solution[3])
{
    for (int column = 0; column < 3; ++column) {
        int pivot = column;
        for (int row = column + 1; row < 3; ++row) {
            if (fabsf(matrix[row][column]) > fabsf(matrix[pivot][column])) {
                pivot = row;
            }
        }
        if (fabsf(matrix[pivot][column]) < 1.0e-7f) return false;
        if (pivot != column) {
            for (int item = column; item < 4; ++item) {
                const float temporary = matrix[column][item];
                matrix[column][item] = matrix[pivot][item];
                matrix[pivot][item] = temporary;
            }
        }
        const float divisor = matrix[column][column];
        for (int item = column; item < 4; ++item) {
            matrix[column][item] /= divisor;
        }
        for (int row = 0; row < 3; ++row) {
            if (row == column) continue;
            const float factor = matrix[row][column];
            for (int item = column; item < 4; ++item) {
                matrix[row][item] -= factor * matrix[column][item];
            }
        }
    }
    for (int row = 0; row < 3; ++row) solution[row] = matrix[row][3];
    return true;
}

bool fit_quadratic(const line_vision_point_t *points, uint8_t count,
                   float *a, float *b, float *c)
{
    if (count < 3U) return false;
    float sy = 0.0f, sy2 = 0.0f, sy3 = 0.0f, sy4 = 0.0f;
    float sx = 0.0f, sxy = 0.0f, sy2x = 0.0f;
    for (uint8_t i = 0; i < count; ++i) {
        const float y = points[i].y_m;
        const float x = points[i].x_m;
        const float y2 = y * y;
        sy += y;
        sy2 += y2;
        sy3 += y2 * y;
        sy4 += y2 * y2;
        sx += x;
        sxy += x * y;
        sy2x += x * y2;
    }
    float system[3][4] = {
        {sy4, sy3, sy2, sy2x},
        {sy3, sy2, sy, sxy},
        {sy2, sy, static_cast<float>(count), sx},
    };
    float solution[3] = {};
    if (!solve_three_by_three(system, solution)) return false;
    *a = solution[0];
    *b = solution[1];
    *c = solution[2];
    return true;
}

bool config_is_valid(const line_vision_control_config_t &config)
{
    if (config.scan_row_count < 3U ||
        config.scan_row_count > LINE_VISION_MAX_SCAN_ROWS ||
        config.minimum_valid_rows < 3U ||
        config.minimum_valid_rows > config.scan_row_count ||
        config.minimum_line_width_px == 0U ||
        config.maximum_line_width_px < config.minimum_line_width_px ||
        config.virtual_ir_active_percent == 0U ||
        config.virtual_ir_active_percent > 100U ||
        config.virtual_ir_finish_percent <=
            config.virtual_ir_active_percent ||
        config.virtual_ir_finish_percent > 100U ||
        config.virtual_ir_roi_top_fraction < 0.0f ||
        config.virtual_ir_roi_bottom_fraction > 1.0f ||
        config.virtual_ir_roi_top_fraction >=
            config.virtual_ir_roi_bottom_fraction ||
        config.virtual_ir_width_fraction <= 0.0f ||
        config.virtual_ir_width_fraction > 1.0f ||
        config.lookahead_m <= 0.0f ||
        config.minimum_lookahead_m <= 0.0f ||
        config.minimum_lookahead_m > config.lookahead_m ||
        config.lookahead_curvature_gain < 0.0f ||
        config.image_center_fraction < 0.0f ||
        config.image_center_fraction > 1.0f) {
        return false;
    }
    for (uint8_t i = 0; i < config.scan_row_count; ++i) {
        if (config.rows[i].row_fraction < 0.0f ||
            config.rows[i].row_fraction > 1.0f ||
            config.rows[i].forward_m <= 0.0f ||
            config.rows[i].visible_width_m <= 0.0f) {
            return false;
        }
    }
    return true;
}

}  // namespace

extern "C" void line_vision_control_default_config(
    line_vision_control_config_t *config)
{
    if (config == nullptr) return;
    memset(config, 0, sizeof(*config));
    config->scan_row_count = 8U;
    config->minimum_valid_rows = 3U;

    /* Initial estimates only. Measure these values on the actual chassis.
     * Keep every scan row in the lower 60% of the image. The camera can see
     * much farther than the controller needs, and the upper part frequently
     * contains a later section of the course. */
    const line_vision_row_calibration_t defaults[8] = {
        {0.92f, 0.20f, 0.32f},
        {0.84f, 0.24f, 0.36f},
        {0.76f, 0.29f, 0.41f},
        {0.68f, 0.35f, 0.47f},
        {0.60f, 0.42f, 0.54f},
        {0.52f, 0.50f, 0.62f},
        {0.46f, 0.60f, 0.70f},
        {0.40f, 0.72f, 0.80f},
    };
    memcpy(config->rows, defaults, sizeof(defaults));

    config->adaptive_threshold_offset = 38U;
    config->minimum_threshold = 35U;
    config->maximum_threshold = 175U;
    config->minimum_line_width_px = 2U;
    config->maximum_line_width_px = 70U;
    config->maximum_row_jump_fraction = 0.24f;
    config->rgb565_byte_swapped = true;
    config->virtual_ir_threshold = 85U;
    config->virtual_ir_active_percent = 8U;
    config->virtual_ir_finish_percent = 70U;
    config->virtual_ir_roi_top_fraction = 0.72f;
    config->virtual_ir_roi_bottom_fraction = 0.96f;
    config->virtual_ir_width_fraction = 0.60f;
    config->image_center_fraction = 0.5f;

    config->lookahead_m = 0.36f;
    config->minimum_lookahead_m = 0.24f;
    config->lookahead_curvature_gain = 0.06f;
    config->nominal_forward_speed_mps = 0.080f;
    config->minimum_forward_speed_mps = 0.012f;
    config->maximum_lateral_speed_mps = 0.04f;
    config->maximum_angular_speed_rad_s = 0.45f;
    config->lateral_speed_gain = 0.50f;
    config->lateral_angular_gain = 2.0f;
    config->heading_angular_gain = 1.2f;
    config->angular_command_sign = -1.0f;
    config->minimum_control_confidence = 0.34f;
}

extern "C" line_vision_control_handle_t line_vision_control_create(
    const line_vision_control_config_t *config)
{
    if (config == nullptr || !config_is_valid(*config)) return nullptr;
    auto *context = static_cast<line_vision_control_context *>(
        calloc(1, sizeof(line_vision_control_context)));
    if (context == nullptr) return nullptr;
    context->config = *config;
    context->previous_center_fraction = config->image_center_fraction;
    return context;
}

extern "C" void line_vision_control_destroy(
    line_vision_control_handle_t handle)
{
    free(handle);
}

extern "C" bool line_vision_control_process_rgb565(
    line_vision_control_handle_t handle, const uint16_t *pixels,
    size_t width, size_t height, int64_t timestamp_us,
    line_vision_control_result_t *result)
{
    if (handle == nullptr || pixels == nullptr || result == nullptr ||
        width < 8U || height < 8U) {
        return false;
    }
    memset(result, 0, sizeof(*result));
    result->timestamp_us = timestamp_us;
    const line_vision_control_config_t &config = handle->config;
    result->threshold = adaptive_threshold(config, pixels, width, height);
    detect_virtual_ir(handle, pixels, width, height, result);

    float expected_center = handle->has_previous_center
                                ? handle->previous_center_fraction * (width - 1U)
                                : config.image_center_fraction * (width - 1U);
    float contrast_sum = 0.0f;
    float accepted_center_sum = 0.0f;
    uint8_t missed_before_anchor = 0;
    uint8_t consecutive_missed_rows = 0;
    uint8_t accepted_row_mask = 0U;

    for (uint8_t row_index = 0; row_index < config.scan_row_count;
         ++row_index) {
        const line_vision_row_calibration_t &calibration =
            config.rows[row_index];
        size_t row = static_cast<size_t>(
            calibration.row_fraction * static_cast<float>(height - 1U));
        if (row >= height) row = height - 1U;

        float center = 0.0f;
        float contrast = 0.0f;
        if (!find_line_center(config, pixels, width, row, result->threshold,
                              expected_center, &center, &contrast)) {
            if (result->point_count == 0U) {
                if (++missed_before_anchor >= 3U) break;
            } else if (++consecutive_missed_rows >= 2U) {
                break;
            }
            continue;
        }
        const float jump = fabsf(center - expected_center) /
                           static_cast<float>(width);
        if (result->point_count > 0U &&
            jump > config.maximum_row_jump_fraction) {
            if (++consecutive_missed_rows >= 2U) break;
            continue;
        }

        line_vision_point_t &point = result->points[result->point_count++];
        accepted_row_mask |= static_cast<uint8_t>(1U << row_index);
        point.image_x = center;
        point.image_y = static_cast<float>(row);
        point.y_m = calibration.forward_m;
        point.x_m =
            (center / static_cast<float>(width - 1U) -
             config.image_center_fraction) *
            calibration.visible_width_m;
        expected_center = center;
        consecutive_missed_rows = 0;
        accepted_center_sum += center;
        contrast_sum += contrast;
    }

    const uint8_t middle_lower_mask =
        static_cast<uint8_t>((1U << 3U) | (1U << 4U));
    const bool middle_lower_line_present =
        (accepted_row_mask & middle_lower_mask) == middle_lower_mask;
    update_corner_tracking(handle, result, middle_lower_line_present);
    const bool has_virtual_ir_command =
        apply_virtual_ir_command(config, result);

    if (result->point_count < config.minimum_valid_rows ||
        !fit_quadratic(result->points, result->point_count,
                       &result->curve_a, &result->curve_b,
                       &result->curve_c)) {
        handle->has_previous_center = false;
        if (result->corner_turning) {
            result->control_valid = true;
            result->command_vy_mps = config.minimum_forward_speed_mps;
            result->command_omega_rad_s = clamp_float(
                config.angular_command_sign * 0.85f *
                    result->corner_angle_rad,
                -0.35f, 0.35f);
        } else if (!has_virtual_ir_command) {
            result->control_valid = false;
        }
        return true;
    }

    handle->previous_center_fraction =
        accepted_center_sum /
        (result->point_count * static_cast<float>(width - 1U));
    handle->has_previous_center = true;
    result->line_valid = true;

    const float row_fraction =
        static_cast<float>(result->point_count) / config.scan_row_count;
    const float contrast_fraction = clamp_float(
        contrast_sum / (result->point_count * 45.0f), 0.0f, 1.0f);
    result->confidence = 0.75f * row_fraction + 0.25f * contrast_fraction;

    float minimum_observed_y = result->points[0].y_m;
    float maximum_observed_y = result->points[0].y_m;
    for (uint8_t i = 1; i < result->point_count; ++i) {
        if (result->points[i].y_m < minimum_observed_y) {
            minimum_observed_y = result->points[i].y_m;
        }
        if (result->points[i].y_m > maximum_observed_y) {
            maximum_observed_y = result->points[i].y_m;
        }
    }

    const float nominal_y = clamp_float(config.lookahead_m,
                                        minimum_observed_y,
                                        maximum_observed_y);
    const float nominal_slope =
        2.0f * result->curve_a * nominal_y + result->curve_b;
    const float nominal_curvature =
        2.0f * result->curve_a /
        powf(1.0f + nominal_slope * nominal_slope, 1.5f);
    const float curvature_fraction = clamp_float(
        fabsf(nominal_curvature) * config.lookahead_curvature_gain,
        0.0f, 1.0f);
    const float requested_y =
        config.lookahead_m -
        curvature_fraction *
            (config.lookahead_m - config.minimum_lookahead_m);
    const float y = clamp_float(requested_y, minimum_observed_y,
                                maximum_observed_y);
    result->active_lookahead_m = y;
    result->lateral_error_m =
        result->curve_a * y * y + result->curve_b * y + result->curve_c;
    const float slope = 2.0f * result->curve_a * y + result->curve_b;
    result->heading_error_rad = atanf(slope);
    result->curvature_inv_m =
        2.0f * result->curve_a /
        powf(1.0f + slope * slope, 1.5f);

    if (!has_virtual_ir_command && !result->corner_turning) return true;

    float corner_slowdown = 0.0f;
    if (result->corner_turning) {
        result->command_vx_mps = 0.0f;
        result->command_omega_rad_s = clamp_float(
            config.angular_command_sign * 0.85f *
                result->corner_angle_rad,
            -0.35f, 0.35f);
        corner_slowdown = 1.0f;
    } else if (result->corner_detected) {
        /* Warning phase: the four-channel near-field strip remains in charge.
         * The far branch is remembered but cannot pull the car early. */
        corner_slowdown = 0.25f;
    }

    const float curve_slowdown = clamp_float(
        fabsf(result->curvature_inv_m) * 0.18f, 0.0f, 1.0f);
    const float steering_slowdown = clamp_float(
        fabsf(result->command_omega_rad_s) /
            config.maximum_angular_speed_rad_s,
        0.0f, 1.0f);
    float slowdown = curve_slowdown > steering_slowdown
                               ? curve_slowdown
                               : steering_slowdown;
    if (corner_slowdown > slowdown) slowdown = corner_slowdown;
    result->command_vy_mps =
        config.nominal_forward_speed_mps -
        slowdown * (config.nominal_forward_speed_mps -
                    config.minimum_forward_speed_mps);
    return true;
}
