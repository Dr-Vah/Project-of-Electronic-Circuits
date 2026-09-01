#include "line_vision_control.hpp"

#include <assert.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <vector>

namespace {

uint16_t byte_swap(uint16_t value)
{
    return static_cast<uint16_t>((value << 8) | (value >> 8));
}

void draw_line(std::vector<uint16_t> &frame, size_t width, size_t height,
               int center_x)
{
    const uint16_t white = byte_swap(0xffffU);
    const uint16_t black = byte_swap(0x0000U);
    for (uint16_t &pixel : frame) pixel = white;
    for (size_t y = 0; y < height; ++y) {
        for (int offset = -2; offset <= 2; ++offset) {
            const int x = center_x + offset;
            if (x >= 0 && x < static_cast<int>(width)) {
                frame[y * width + static_cast<size_t>(x)] = black;
            }
        }
    }
}

void draw_curve(std::vector<uint16_t> &frame, size_t width, size_t height)
{
    const uint16_t white = byte_swap(0xffffU);
    const uint16_t black = byte_swap(0x0000U);
    for (uint16_t &pixel : frame) pixel = white;
    for (size_t y = 0; y < height; ++y) {
        const float distance_from_bottom =
            static_cast<float>(height - 1U - y) / height;
        const int center_x = 40 + static_cast<int>(
                                      20.0f * distance_from_bottom *
                                      distance_from_bottom);
        for (int offset = -2; offset <= 2; ++offset) {
            const int x = center_x + offset;
            if (x >= 0 && x < static_cast<int>(width)) {
                frame[y * width + static_cast<size_t>(x)] = black;
            }
        }
    }
}

void draw_corner(std::vector<uint16_t> &frame, size_t width, size_t height)
{
    const uint16_t white = byte_swap(0xffffU);
    const uint16_t black = byte_swap(0x0000U);
    for (uint16_t &pixel : frame) pixel = white;
    for (size_t y = 0; y < height; ++y) {
        const int center_x = y >= height / 2U
                                 ? 40
                                 : 40 + static_cast<int>((height / 2U - y) * 1.1f);
        for (int offset = -2; offset <= 2; ++offset) {
            const int x = center_x + offset;
            if (x >= 0 && x < static_cast<int>(width)) {
                frame[y * width + static_cast<size_t>(x)] = black;
            }
        }
    }
}

void erase_scan_row(std::vector<uint16_t> &frame, size_t width,
                    size_t height, float row_fraction)
{
    const uint16_t white = byte_swap(0xffffU);
    const size_t row = static_cast<size_t>(
        row_fraction * static_cast<float>(height - 1U));
    for (size_t x = 0; x < width; ++x) {
        frame[row * width + x] = white;
    }
}

void draw_all_black(std::vector<uint16_t> &frame)
{
    const uint16_t black = byte_swap(0x0000U);
    for (uint16_t &pixel : frame) pixel = black;
}

void process_three(line_vision_control_handle_t vision,
                   const std::vector<uint16_t> &frame,
                   size_t width, size_t height, int64_t first_timestamp,
                   line_vision_control_result_t *result)
{
    for (int frame_number = 0; frame_number < 3; ++frame_number) {
        assert(line_vision_control_process_rgb565(
            vision, frame.data(), width, height,
            first_timestamp + frame_number, result));
    }
}

}  // namespace

int main()
{
    constexpr size_t width = 80U;
    constexpr size_t height = 60U;
    std::vector<uint16_t> frame(width * height);

    line_vision_control_config_t config;
    line_vision_control_default_config(&config);
    assert(config.virtual_ir_threshold == 85U);
    assert(config.virtual_ir_roi_top_fraction >= 0.70f);
    assert(config.lookahead_m <= 0.36f);
    for (uint8_t row = 0; row < config.scan_row_count; ++row) {
        assert(config.rows[row].row_fraction >= 0.40f);
        assert(config.rows[row].forward_m <= 0.72f);
    }
    line_vision_control_handle_t vision =
        line_vision_control_create(&config);
    assert(vision != nullptr);

    line_vision_control_result_t result = {};
    draw_line(frame, width, height, 40);
    process_three(vision, frame, width, height, 1000, &result);
    assert(result.line_valid);
    assert(result.control_valid);
    assert(result.virtual_ir_valid);
    assert(result.virtual_ir_pattern == 0x06U);
    assert(result.point_count == config.scan_row_count);
    assert(fabsf(result.lateral_error_m) < 0.02f);
    assert(result.command_vx_mps == 0.0f);
    assert(fabsf(result.command_omega_rad_s) < 0.01f);

    draw_line(frame, width, height, 52);
    assert(line_vision_control_process_rgb565(
        vision, frame.data(), width, height, 2000, &result));
    assert(result.control_valid);
    assert(result.virtual_ir_valid);
    assert(result.virtual_ir_error > 0.0f);
    assert(result.lateral_error_m > 0.02f);
    assert(result.command_vx_mps == 0.0f);
    assert(result.command_omega_rad_s < 0.0f);

    draw_curve(frame, width, height);
    process_three(vision, frame, width, height, 2500, &result);
    assert(result.control_valid);
    assert(result.active_lookahead_m <= config.lookahead_m);
    assert(result.command_vy_mps <= config.nominal_forward_speed_mps);

    draw_corner(frame, width, height);
    assert(line_vision_control_process_rgb565(
        vision, frame.data(), width, height, 2600, &result));
    assert(result.control_valid);
    assert(result.corner_detected);
    assert(!result.corner_turning);
    assert(result.corner_angle_rad > 0.0f);
    assert(fabsf(result.command_omega_rad_s) < 0.05f);
    assert(result.command_vy_mps < config.nominal_forward_speed_mps);

    /* The nearer middle-lower row still sees the incoming line, while the
     * farther one has passed its endpoint. This should trigger earlier than
     * waiting for the entire lower field to become blank. */
    erase_scan_row(frame, width, height, config.rows[4].row_fraction);
    assert(line_vision_control_process_rgb565(
        vision, frame.data(), width, height, 3000, &result));
    assert(result.line_valid);
    assert(result.control_valid);
    assert(result.corner_turning);
    assert(result.command_vx_mps == 0.0f);
    assert(fabsf(result.command_vy_mps -
                 config.minimum_forward_speed_mps) < 1.0e-5f);
    assert(result.command_omega_rad_s < 0.0f);

    draw_line(frame, width, height, 40);
    process_three(vision, frame, width, height, 3100, &result);
    assert(result.line_valid);
    assert(result.control_valid);
    assert(!result.corner_detected);
    assert(!result.corner_turning);

    draw_all_black(frame);
    assert(line_vision_control_process_rgb565(
        vision, frame.data(), width, height, 3200, &result));
    assert(result.virtual_ir_all_black);
    assert(result.virtual_ir_pattern == 0x0FU);
    assert(!result.virtual_ir_valid);
    assert(!result.control_valid);

    line_vision_control_destroy(vision);
    puts("line_vision_control_test: PASS");
    return 0;
}
