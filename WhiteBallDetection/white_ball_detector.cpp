#include "white_ball_detector.h"

#include <limits.h>
#include <string.h>

namespace {

constexpr uint8_t kRequiredConfirmationFrames = 2U;
constexpr uint8_t kMaximumBridgedMisses = 1U;
constexpr uint16_t kMaximumCenterJumpPx = 24U;

uint16_t clamp_u16(uint32_t value)
{
    return value > UINT16_MAX ? UINT16_MAX : static_cast<uint16_t>(value);
}

uint16_t subtract_clamped(uint16_t value, uint16_t amount)
{
    return value > amount ? static_cast<uint16_t>(value - amount) : 0U;
}

uint16_t add_clamped(uint16_t value, uint16_t amount, size_t limit)
{
    const size_t sum = static_cast<size_t>(value) + amount;
    return static_cast<uint16_t>(sum < limit ? sum : limit - 1U);
}

bool is_same_locked_ball(const ball_detection_t &locked,
                         const ball_detection_t &detected)
{
    const int dx = static_cast<int>(detected.x) - locked.x;
    const int dy = static_cast<int>(detected.y) - locked.y;
    const int permitted_jump = kMaximumCenterJumpPx + locked.radius / 2U;
    const uint16_t smaller = detected.radius < locked.radius
        ? detected.radius : locked.radius;
    const uint16_t larger = detected.radius > locked.radius
        ? detected.radius : locked.radius;
    return dx * dx + dy * dy <= permitted_jump * permitted_jump &&
           smaller > 0U && larger <= 2U * smaller + 2U;
}

white_ball_result_t adapt_detection(const white_ball_detector_t *detector,
                                    const ball_detection_t &ball,
                                    size_t width, size_t height)
{
    const float support = static_cast<float>(ball.coverage) / 32.0f;
    const uint32_t circle_area =
        (314U * static_cast<uint32_t>(ball.radius) * ball.radius + 50U) / 100U;
    return {
        .valid = true,
        .luma_threshold = detector->config.minimum_luma,
        .chroma_threshold = detector->config.maximum_chroma,
        .confirmation_count = detector->tracker.consecutive_frames,
        .area_px = clamp_u16(circle_area),
        .left = subtract_clamped(ball.x, ball.radius),
        .top = subtract_clamped(ball.y, ball.radius),
        .right = add_clamped(ball.x, ball.radius, width),
        .bottom = add_clamped(ball.y, ball.radius, height),
        .center_x = width <= 1U
            ? 0.0f
            : 2.0f * static_cast<float>(ball.x) /
                  static_cast<float>(width - 1U) - 1.0f,
        .center_y = height <= 1U
            ? 0.0f
            : static_cast<float>(ball.y) / static_cast<float>(height - 1U),
        .diameter_px = 2.0f * ball.radius,
        .luma_contrast = 0.0f,
        .chroma_contrast = 0.0f,
        .circular_edge_score = support,
        .shadow_score = 0.0f,
        .confidence = support,
    };
}

void record_miss(white_ball_detector_t *detector,
                 white_ball_result_t *result)
{
    const ball_detector_result_t missing = {};
    ball_detector_tracker_update(&detector->tracker, &missing, nullptr);
    if (detector->consecutive_misses < UINT8_MAX) {
        ++detector->consecutive_misses;
    }

    /* The first dropped frame keeps the locked result. The second releases
     * the local-search hint, matching the teammate detector task. */
    if (detector->have_search_hint &&
        detector->consecutive_misses <= kMaximumBridgedMisses &&
        detector->last_stable_result.valid) {
        *result = detector->last_stable_result;
        result->confidence *= 0.70f;
        return;
    }

    detector->have_search_hint = false;
    memset(&detector->search_hint, 0, sizeof(detector->search_hint));
    memset(&detector->last_stable_result, 0,
           sizeof(detector->last_stable_result));
    ball_detector_tracker_init(&detector->tracker,
                               kRequiredConfirmationFrames,
                               kMaximumCenterJumpPx);
}

}  // namespace

extern "C" void white_ball_default_config(white_ball_config_t *config)
{
    if (config == nullptr) return;
    *config = {
        .rgb565_byte_swapped = true,
        .minimum_area_px = 4U,
        .maximum_area_fraction = 0.04f,
        .minimum_luma = 0U,
        .minimum_dim_luma = 0U,
        .local_highlight_difference = 0U,
        .maximum_luma = 255U,
        .maximum_chroma = 22U,
        .minimum_local_luma_contrast = 0U,
        .shadow_luma_difference = 0U,
        .minimum_aspect_ratio = 0.0f,
        .minimum_fill_ratio = 0.0f,
        .minimum_circular_edge_score = 13.0f / 32.0f,
        .minimum_confidence = 13.0f / 32.0f,
        .roi_top_fraction = 0.0f,
        .roi_bottom_fraction = 1.0f,
    };
}

extern "C" bool white_ball_detector_init(
    white_ball_detector_t *detector, const white_ball_config_t *config)
{
    if (detector == nullptr || config == nullptr ||
        !config->rgb565_byte_swapped) {
        return false;
    }
    memset(detector, 0, sizeof(*detector));
    detector->config = *config;
    ball_detector_tracker_init(&detector->tracker,
                               kRequiredConfirmationFrames,
                               kMaximumCenterJumpPx);
    return true;
}

extern "C" bool white_ball_detect_rgb565(
    white_ball_detector_t *detector, const uint16_t *pixels, size_t width,
    size_t height, white_ball_result_t *result)
{
    if (detector == nullptr || pixels == nullptr || result == nullptr ||
        width < 32U || height < 24U || width > WHITE_BALL_MAX_WIDTH ||
        height > WHITE_BALL_MAX_HEIGHT || width > UINT16_MAX ||
        height > UINT16_MAX) {
        return false;
    }
    memset(result, 0, sizeof(*result));
    result->luma_threshold = detector->config.minimum_luma;
    result->chroma_threshold = detector->config.maximum_chroma;

    ball_detector_result_t raw = {};
    const esp_err_t error = ball_detector_process_rgb565(
        pixels, static_cast<uint16_t>(width), static_cast<uint16_t>(height),
        detector->have_search_hint ? &detector->search_hint : nullptr, &raw);
    if (error != ESP_OK) return false;

    if (!raw.found ||
        (detector->have_search_hint &&
         !is_same_locked_ball(detector->search_hint, raw.ball))) {
        record_miss(detector, result);
        return true;
    }

    detector->consecutive_misses = 0U;
    ball_detection_t stable = {};
    if (!ball_detector_tracker_update(&detector->tracker, &raw, &stable)) {
        /* After one bridged miss the tracker needs one frame to rebuild its
         * consecutive count. Keep publishing the locked ball during that
         * reacquisition, just as BallDet's result queue retains its value. */
        if (detector->have_search_hint &&
            detector->last_stable_result.valid) {
            detector->search_hint = raw.ball;
            *result = detector->last_stable_result;
            result->confidence *= 0.80f;
        }
        return true;
    }

    detector->search_hint = stable;
    detector->have_search_hint = true;
    *result = adapt_detection(detector, stable, width, height);
    detector->last_stable_result = *result;
    return true;
}
