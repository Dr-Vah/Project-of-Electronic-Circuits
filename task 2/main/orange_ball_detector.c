#include "orange_ball_detector.h"

#include <math.h>
#include <string.h>

#define ORANGE_BRIDGED_MISSES 1U

static void unpack_rgb565(uint16_t wire, bool swapped,
                          uint8_t *red, uint8_t *green, uint8_t *blue)
{
    const uint16_t rgb = swapped
        ? (uint16_t)((wire << 8) | (wire >> 8)) : wire;
    *red = (uint8_t)(((rgb >> 11) & 31U) * 255U / 31U);
    *green = (uint8_t)(((rgb >> 5) & 63U) * 255U / 63U);
    *blue = (uint8_t)((rgb & 31U) * 255U / 31U);
}

static bool is_orange(const orange_ball_config_t *config, uint16_t pixel)
{
    uint8_t red, green, blue;
    unpack_rgb565(pixel, config->rgb565_byte_swapped, &red, &green, &blue);
    return red >= config->minimum_red &&
           (int)red - green >= config->minimum_red_green_difference &&
           (int)red - blue >= config->minimum_red_blue_difference &&
           (int)green + 12 >= blue;
}

static bool candidates_are_near(const white_ball_result_t *first,
                                const white_ball_result_t *second,
                                uint16_t maximum_jump)
{
    if (!first->valid || !second->valid) return false;
    const float dx = 0.5f * (first->left + first->right) -
                     0.5f * (second->left + second->right);
    const float dy = 0.5f * (first->top + first->bottom) -
                     0.5f * (second->top + second->bottom);
    return dx * dx + dy * dy <= (float)maximum_jump * maximum_jump;
}

void orange_ball_default_config(orange_ball_config_t *config)
{
    if (config == NULL) return;
    *config = (orange_ball_config_t) {
        .rgb565_byte_swapped = true,
        .minimum_red = 145U,
        .minimum_red_green_difference = 38U,
        .minimum_red_blue_difference = 55U,
        .minimum_area_px = 12U,
        .maximum_area_fraction = 0.04f,
        .minimum_aspect_ratio = 0.55f,
        .maximum_aspect_ratio = 1.65f,
        .minimum_fill_ratio = 0.48f,
        .roi_top_fraction = 0.12f,
        .roi_bottom_fraction = 1.0f,
        .required_confirmation_frames = 2U,
        .maximum_center_jump_px = 40U,
    };
}

bool orange_ball_detector_init(orange_ball_detector_t *detector,
                               const orange_ball_config_t *config)
{
    if (detector == NULL || config == NULL) return false;
    memset(detector, 0, sizeof(*detector));
    detector->config = *config;
    return true;
}

bool orange_ball_detect_rgb565(orange_ball_detector_t *detector,
                               const uint16_t *pixels,
                               size_t width,
                               size_t height,
                               white_ball_result_t *result)
{
    if (detector == NULL || pixels == NULL || result == NULL ||
        width == 0U || height == 0U || width > WHITE_BALL_MAX_WIDTH ||
        height > WHITE_BALL_MAX_HEIGHT) {
        return false;
    }
    memset(result, 0, sizeof(*result));
    result->luma_threshold = detector->config.minimum_red;
    result->chroma_threshold = detector->config.minimum_red_green_difference;

    const size_t count = width * height;
    memset(detector->mask, 0, count);
    size_t roi_top = (size_t)(detector->config.roi_top_fraction * height);
    size_t roi_bottom = (size_t)(detector->config.roi_bottom_fraction * height);
    if (roi_bottom > height) roi_bottom = height;
    for (size_t y = roi_top; y < roi_bottom; ++y) {
        for (size_t x = 0U; x < width; ++x) {
            const size_t index = y * width + x;
            detector->mask[index] = is_orange(&detector->config,
                                               pixels[index]) ? 1U : 0U;
        }
    }

    white_ball_result_t best = {0};
    float best_score = 0.0f;
    const size_t maximum_area =
        (size_t)(detector->config.maximum_area_fraction * count);
    for (size_t start = 0U; start < count; ++start) {
        if (detector->mask[start] != 1U) continue;
        size_t head = 0U, tail = 0U;
        detector->queue[tail++] = (uint16_t)start;
        detector->mask[start] = 2U;
        size_t area = 0U;
        size_t left = width, right = 0U, top = height, bottom = 0U;
        while (head < tail) {
            const size_t index = detector->queue[head++];
            const size_t x = index % width;
            const size_t y = index / width;
            ++area;
            if (x < left) left = x;
            if (x > right) right = x;
            if (y < top) top = y;
            if (y > bottom) bottom = y;
            const size_t neighbours[4] = {
                x > 0U ? index - 1U : index,
                x + 1U < width ? index + 1U : index,
                y > 0U ? index - width : index,
                y + 1U < height ? index + width : index,
            };
            for (size_t n = 0U; n < 4U; ++n) {
                const size_t next = neighbours[n];
                if (next != index && detector->mask[next] == 1U) {
                    detector->mask[next] = 2U;
                    detector->queue[tail++] = (uint16_t)next;
                }
            }
        }
        if (area < detector->config.minimum_area_px || area > maximum_area) {
            continue;
        }
        const size_t box_width = right - left + 1U;
        const size_t box_height = bottom - top + 1U;
        const float aspect = (float)box_width / (float)box_height;
        const float fill = (float)area / (float)(box_width * box_height);
        if (aspect < detector->config.minimum_aspect_ratio ||
            aspect > detector->config.maximum_aspect_ratio ||
            fill < detector->config.minimum_fill_ratio) {
            continue;
        }
        white_ball_result_t candidate = {
            .valid = true,
            .luma_threshold = detector->config.minimum_red,
            .chroma_threshold = detector->config.minimum_red_green_difference,
            .area_px = area > UINT16_MAX ? UINT16_MAX : (uint16_t)area,
            .left = (uint16_t)left,
            .top = (uint16_t)top,
            .right = (uint16_t)right,
            .bottom = (uint16_t)bottom,
            .center_x = width <= 1U ? 0.0f :
                2.0f * (0.5f * (left + right)) / (float)(width - 1U) - 1.0f,
            .center_y = height <= 1U ? 0.0f :
                (0.5f * (top + bottom)) / (float)(height - 1U),
            .diameter_px = 0.5f * (box_width + box_height),
            .circular_edge_score = aspect <= 1.0f ? aspect : 1.0f / aspect,
            .confidence = fill,
        };
        float score = (float)area * fill * candidate.circular_edge_score;
        if (candidates_are_near(&detector->last_stable_result, &candidate,
                                detector->config.maximum_center_jump_px)) {
            score *= 2.0f;
        }
        if (score > best_score) {
            best = candidate;
            best_score = score;
        }
    }

    if (!best.valid) {
        detector->confirmation_count = 0U;
        if (++detector->missed_frames <= ORANGE_BRIDGED_MISSES &&
            detector->last_stable_result.valid) {
            *result = detector->last_stable_result;
            result->confidence *= 0.70f;
        }
        return true;
    }

    detector->missed_frames = 0U;
    if (candidates_are_near(&detector->previous_candidate, &best,
                            detector->config.maximum_center_jump_px)) {
        if (detector->confirmation_count < UINT8_MAX) {
            ++detector->confirmation_count;
        }
    } else {
        detector->confirmation_count = 1U;
    }
    detector->previous_candidate = best;
    detector->previous_candidate.valid = true;
    if (detector->confirmation_count <
        detector->config.required_confirmation_frames) {
        return true;
    }
    best.confirmation_count = detector->confirmation_count;
    *result = best;
    detector->last_stable_result = best;
    return true;
}

