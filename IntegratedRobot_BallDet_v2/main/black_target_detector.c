#include "black_target_detector.h"

#include <math.h>
#include <string.h>

#define MAXIMUM_BRIDGED_MISSES 4U

typedef struct {
    uint16_t area;
    uint16_t left;
    uint16_t top;
    uint16_t right;
    uint16_t bottom;
    uint32_t sum_x;
    uint32_t sum_y;
} component_t;

static uint8_t pixel_luma(uint16_t pixel, bool byte_swapped)
{
    if (byte_swapped) {
        pixel = (uint16_t)((pixel << 8) | (pixel >> 8));
    }
    const uint8_t red = (uint8_t)(((pixel >> 11) & 0x1fU) * 255U / 31U);
    const uint8_t green = (uint8_t)(((pixel >> 5) & 0x3fU) * 255U / 63U);
    const uint8_t blue = (uint8_t)((pixel & 0x1fU) * 255U / 31U);
    return (uint8_t)((77U * red + 150U * green + 29U * blue) >> 8);
}

static float clamp01(float value)
{
    if (value < 0.0f) return 0.0f;
    if (value > 1.0f) return 1.0f;
    return value;
}

static bool candidates_match(const black_target_result_t *a,
                             const black_target_result_t *b)
{
    return fabsf(a->center_x - b->center_x) <= 0.28f &&
           fabsf(a->center_y - b->center_y) <= 0.20f;
}

void black_target_default_config(black_target_config_t *config)
{
    if (config == NULL) return;
    *config = (black_target_config_t) {
        .rgb565_byte_swapped = true,
        /* The black tape in the current camera exposure is 72..100 luma.
         * A threshold of 70 discarded it and left only the darker furniture
         * shadow. */
        .maximum_luma = 105U,
        .minimum_area_px = 30U,
        .maximum_area_fraction = 0.35f,
        /* Detection runs after the raw browser image is rotated by 180
         * degrees. In the supplied scene both stopping patches therefore
         * appear near the top, while furniture moves toward the bottom. */
        /* Door/furniture shadows occupy the first 10% after rotation. */
        .roi_top_fraction = 0.10f,
        .roi_bottom_fraction = 0.92f,
        .required_confirmation_frames = 2U,
    };
}

bool black_target_detector_init(black_target_detector_t *detector,
                                const black_target_config_t *config)
{
    if (detector == NULL || config == NULL ||
        config->minimum_area_px == 0U ||
        config->maximum_area_fraction <= 0.0f ||
        config->maximum_area_fraction > 1.0f ||
        config->roi_top_fraction < 0.0f ||
        config->roi_bottom_fraction > 1.0f ||
        config->roi_top_fraction >= config->roi_bottom_fraction ||
        config->required_confirmation_frames == 0U) {
        return false;
    }
    memset(detector, 0, sizeof(*detector));
    detector->config = *config;
    return true;
}

bool black_target_detect_rgb565(black_target_detector_t *detector,
                                const uint16_t *pixels,
                                size_t width,
                                size_t height,
                                float preferred_center_x,
                                float preferred_center_y,
                                black_target_result_t *result)
{
    if (detector == NULL || pixels == NULL || result == NULL ||
        width == 0U || height == 0U ||
        width > WHITE_BALL_MAX_WIDTH || height > WHITE_BALL_MAX_HEIGHT) {
        return false;
    }

    memset(result, 0, sizeof(*result));
    const size_t pixel_count = width * height;
    memset(detector->mask, 0, pixel_count);
    size_t y0 = (size_t)(detector->config.roi_top_fraction * height);
    size_t y1 = (size_t)(detector->config.roi_bottom_fraction * height);
    if (y1 > height) y1 = height;

    for (size_t y = y0; y < y1; ++y) {
        for (size_t x = 0U; x < width; ++x) {
            const size_t index = y * width + x;
            detector->mask[index] =
                pixel_luma(pixels[index],
                           detector->config.rgb565_byte_swapped) <=
                        detector->config.maximum_luma
                    ? 1U
                    : 0U;
        }
    }

    component_t best = {0};
    float best_selection_distance = INFINITY;
    const bool target_locked = detector->previous_candidate.valid &&
        detector->confirmation_count >=
            detector->config.required_confirmation_frames;
    for (size_t y = y0; y < y1; ++y) {
        for (size_t x = 0U; x < width; ++x) {
            const size_t seed = y * width + x;
            if (detector->mask[seed] != 1U) continue;

            size_t head = 0U;
            size_t tail = 0U;
            detector->queue[tail++] = (uint16_t)seed;
            detector->mask[seed] = 2U;
            component_t current = {
                .left = (uint16_t)x,
                .top = (uint16_t)y,
                .right = (uint16_t)x,
                .bottom = (uint16_t)y,
            };

            while (head < tail) {
                const size_t index = detector->queue[head++];
                const size_t cx = index % width;
                const size_t cy = index / width;
                ++current.area;
                current.sum_x += (uint32_t)cx;
                current.sum_y += (uint32_t)cy;
                if (cx < current.left) current.left = (uint16_t)cx;
                if (cx > current.right) current.right = (uint16_t)cx;
                if (cy < current.top) current.top = (uint16_t)cy;
                if (cy > current.bottom) current.bottom = (uint16_t)cy;

                const int dx[4] = {-1, 1, 0, 0};
                const int dy[4] = {0, 0, -1, 1};
                for (size_t n = 0U; n < 4U; ++n) {
                    const int nx = (int)cx + dx[n];
                    const int ny = (int)cy + dy[n];
                    if (nx < 0 || ny < (int)y0 || nx >= (int)width ||
                        ny >= (int)y1) {
                        continue;
                    }
                    const size_t neighbour = (size_t)ny * width + (size_t)nx;
                    if (detector->mask[neighbour] == 1U) {
                        detector->mask[neighbour] = 2U;
                        detector->queue[tail++] = (uint16_t)neighbour;
                    }
                }
            }

            const float fraction = (float)current.area / (float)pixel_count;
            const float component_center_x =
                2.0f * ((float)current.sum_x / current.area + 0.5f) /
                    (float)width -
                1.0f;
            const float component_center_y =
                ((float)current.sum_y / current.area + 0.5f) /
                (float)height;
            const float horizontal_distance = isfinite(preferred_center_x)
                ? fabsf(component_center_x - preferred_center_x) : 0.0f;
            const float lock_dx = target_locked
                ? fabsf(component_center_x -
                         detector->previous_candidate.center_x) : 0.0f;
            const float lock_dy = target_locked
                ? fabsf(component_center_y -
                         detector->previous_candidate.center_y) : 0.0f;
            const bool matches_lock = !target_locked ||
                (lock_dx <= 0.28f && lock_dy <= 0.20f);
            const float selection_distance = target_locked
                ? lock_dx + lock_dy
                : (isfinite(preferred_center_x)
                       ? horizontal_distance
                       : -(float)current.area);
            const bool lies_beyond_selected_ball =
                target_locked || !isfinite(preferred_center_y) ||
                component_center_y + 0.04f < preferred_center_y;
            const bool is_near_selected_ball =
                target_locked || !isfinite(preferred_center_y) ||
                preferred_center_y - component_center_y <= 0.30f;
            if (current.area >= detector->config.minimum_area_px &&
                fraction <= detector->config.maximum_area_fraction &&
                lies_beyond_selected_ball &&
                is_near_selected_ball && matches_lock &&
                selection_distance < best_selection_distance) {
                best = current;
                best_selection_distance = selection_distance;
            }
        }
    }

    if (best.area == 0U) {
        /* Preserve acquisition/lock across one or two bad JPEG frames. */
        if (detector->confirmation_count > 0U &&
            detector->missed_frames < MAXIMUM_BRIDGED_MISSES) {
            ++detector->missed_frames;
            if (detector->confirmation_count >=
                detector->config.required_confirmation_frames) {
                *result = detector->previous_candidate;
                result->confirmation_count = detector->confirmation_count;
                result->valid = true;
            }
            return true;
        }
        if (!target_locked) {
            detector->confirmation_count = 0U;
            detector->missed_frames = 0U;
            memset(&detector->previous_candidate, 0,
                   sizeof(detector->previous_candidate));
        }
        return true;
    }

    black_target_result_t candidate = {
        .area_px = best.area,
        .left = best.left,
        .top = best.top,
        .right = best.right,
        .bottom = best.bottom,
        .center_x = 2.0f * ((float)best.sum_x / best.area + 0.5f) /
                        (float)width - 1.0f,
        .center_y = ((float)best.sum_y / best.area + 0.5f) / (float)height,
        .area_fraction = clamp01((float)best.area / (float)pixel_count),
    };

    if (detector->confirmation_count > 0U &&
        candidates_match(&candidate, &detector->previous_candidate)) {
        if (detector->confirmation_count < UINT8_MAX) {
            ++detector->confirmation_count;
        }
    } else {
        detector->confirmation_count = 1U;
    }
    detector->missed_frames = 0U;
    candidate.confirmation_count = detector->confirmation_count;
    candidate.valid = detector->confirmation_count >=
                      detector->config.required_confirmation_frames;
    detector->previous_candidate = candidate;
    *result = candidate;
    return true;
}

