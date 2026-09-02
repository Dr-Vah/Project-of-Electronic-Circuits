#include "white_ball_detector.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>

namespace {

struct pixel_info_t {
    uint8_t luma;
    uint8_t chroma;
};

struct component_t {
    uint16_t area;
    uint16_t left;
    uint16_t top;
    uint16_t right;
    uint16_t bottom;
    uint32_t sum_x;
    uint32_t sum_y;
    uint32_t sum_luma;
    uint32_t sum_chroma;
};

struct surroundings_t {
    float luma_contrast;
    float chroma_contrast;
    float shadow_score;
};

pixel_info_t decode_pixel(uint16_t pixel, bool byte_swapped)
{
    if (byte_swapped) {
        pixel = static_cast<uint16_t>((pixel << 8) | (pixel >> 8));
    }
    const uint8_t red = static_cast<uint8_t>(
        ((pixel >> 11) & 0x1fU) * 255U / 31U);
    const uint8_t green = static_cast<uint8_t>(
        ((pixel >> 5) & 0x3fU) * 255U / 63U);
    const uint8_t blue = static_cast<uint8_t>(
        (pixel & 0x1fU) * 255U / 31U);
    uint8_t maximum = red;
    if (green > maximum) maximum = green;
    if (blue > maximum) maximum = blue;
    uint8_t minimum = red;
    if (green < minimum) minimum = green;
    if (blue < minimum) minimum = blue;
    return {
        .luma = static_cast<uint8_t>(
            (77U * red + 150U * green + 29U * blue) >> 8),
        .chroma = static_cast<uint8_t>(maximum - minimum),
    };
}

float clamp01(float value)
{
    if (value < 0.0f) return 0.0f;
    if (value > 1.0f) return 1.0f;
    return value;
}

void close_small_gaps(white_ball_detector_t *detector, size_t width,
                      size_t height, size_t roi_y0, size_t roi_y1)
{
    /* A ping-pong-ball is only several pixels wide at 240x160. JPEG colour
     * noise can split it into several neutral-colour islands, so apply one
     * 3x3 closing pass before connected-component analysis. The BFS queue is
     * idle here and is large enough to serve as a temporary byte buffer. */
    uint8_t *expanded = reinterpret_cast<uint8_t *>(detector->queue);
    const size_t pixel_count = width * height;
    memset(expanded, 0, pixel_count);

    const size_t first_y = roi_y0 < 1U ? 1U : roi_y0;
    const size_t last_y = roi_y1 > height - 1U ? height - 1U : roi_y1;
    for (size_t y = first_y; y < last_y; ++y) {
        for (size_t x = 1U; x + 1U < width; ++x) {
            bool neighbour_is_white = false;
            for (int dy = -1; dy <= 1 && !neighbour_is_white; ++dy) {
                for (int dx = -1; dx <= 1; ++dx) {
                    const size_t neighbour =
                        static_cast<size_t>(static_cast<int>(y) + dy) * width +
                        static_cast<size_t>(static_cast<int>(x) + dx);
                    if (detector->mask[neighbour] != 0U) {
                        neighbour_is_white = true;
                        break;
                    }
                }
            }
            expanded[y * width + x] = neighbour_is_white ? 1U : 0U;
        }
    }

    memset(detector->mask, 0, pixel_count);
    for (size_t y = first_y; y < last_y; ++y) {
        for (size_t x = 1U; x + 1U < width; ++x) {
            bool fully_surrounded = true;
            for (int dy = -1; dy <= 1 && fully_surrounded; ++dy) {
                for (int dx = -1; dx <= 1; ++dx) {
                    const size_t neighbour =
                        static_cast<size_t>(static_cast<int>(y) + dy) * width +
                        static_cast<size_t>(static_cast<int>(x) + dx);
                    if (expanded[neighbour] == 0U) {
                        fully_surrounded = false;
                        break;
                    }
                }
            }
            detector->mask[y * width + x] = fully_surrounded ? 1U : 0U;
        }
    }
}

size_t clamp_coordinate(int value, size_t limit)
{
    if (value < 0) return 0U;
    if (static_cast<size_t>(value) >= limit) return limit - 1U;
    return static_cast<size_t>(value);
}

bool is_local_highlight(const white_ball_detector_t *detector,
                        const uint16_t *pixels, size_t width, size_t height,
                        size_t x, size_t y, const pixel_info_t &center)
{
    /* At the current 160x120 decode size the ping-pong ball is about 7-10
     * pixels wide. Samples five pixels away normally fall on the paper. Use
     * the brightest of four opposing-pair averages: a nearby black course
     * line can darken one pair, but cannot make a paper pixel look locally
     * bright through all directions. */
    const int radius = 5;
    if (x < (size_t)radius || y < (size_t)radius ||
        x + radius >= width || y + radius >= height ||
        center.luma < detector->config.minimum_dim_luma) {
        return false;
    }
    static const int8_t pairs[4][4] = {
        {-5, 0, 5, 0}, {0, -5, 0, 5},
        {-4, -4, 4, 4}, {-4, 4, 4, -4},
    };
    uint16_t background_luma = 0U;
    for (const auto &pair : pairs) {
        const size_t first =
            (size_t)((int)y + pair[1]) * width + (size_t)((int)x + pair[0]);
        const size_t second =
            (size_t)((int)y + pair[3]) * width + (size_t)((int)x + pair[2]);
        const pixel_info_t a = decode_pixel(
            pixels[first], detector->config.rgb565_byte_swapped);
        const pixel_info_t b = decode_pixel(
            pixels[second], detector->config.rgb565_byte_swapped);
        const uint16_t pair_average = ((uint16_t)a.luma + b.luma) / 2U;
        if (pair_average > background_luma) background_luma = pair_average;
    }
    return center.luma >=
        background_luma + detector->config.local_highlight_difference;
}

surroundings_t measure_surroundings(
    const white_ball_detector_t *detector, const uint16_t *pixels,
    size_t width, size_t height, const component_t &component)
{
    const int box_width = component.right - component.left + 1;
    const int box_height = component.bottom - component.top + 1;
    const int margin = (box_width > box_height ? box_width : box_height) / 2 + 2;
    const size_t x0 = clamp_coordinate((int)component.left - margin, width);
    const size_t y0 = clamp_coordinate((int)component.top - margin, height);
    const size_t x1 = clamp_coordinate((int)component.right + margin, width);
    const size_t y1 = clamp_coordinate((int)component.bottom + margin, height);
    const float inner_luma =
        static_cast<float>(component.sum_luma) / component.area;
    const float inner_chroma =
        static_cast<float>(component.sum_chroma) / component.area;

    uint32_t outer_luma_sum = 0U;
    uint32_t outer_chroma_sum = 0U;
    uint32_t outer_count = 0U;
    uint32_t shadow_count = 0U;
    for (size_t y = y0; y <= y1; ++y) {
        for (size_t x = x0; x <= x1; ++x) {
            if (x >= component.left && x <= component.right &&
                y >= component.top && y <= component.bottom) {
                continue;
            }
            const pixel_info_t pixel = decode_pixel(
                pixels[y * width + x], detector->config.rgb565_byte_swapped);
            outer_luma_sum += pixel.luma;
            outer_chroma_sum += pixel.chroma;
            ++outer_count;
            if (pixel.luma + detector->config.shadow_luma_difference <
                inner_luma) {
                ++shadow_count;
            }
        }
    }

    if (outer_count == 0U) return {};
    const float outer_chroma =
        static_cast<float>(outer_chroma_sum) / outer_count;
    const float outer_luma =
        static_cast<float>(outer_luma_sum) / outer_count;
    return {
        .luma_contrast = inner_luma - outer_luma,
        .chroma_contrast = outer_chroma - inner_chroma,
        .shadow_score = clamp01(
            4.0f * static_cast<float>(shadow_count) / outer_count),
    };
}

float circular_edge_score(const white_ball_detector_t *detector,
                          const uint16_t *pixels, size_t width, size_t height,
                          const component_t &component)
{
    /* Fixed-point unit vectors avoid a full Hough accumulator. Each direction
     * compares one sample just inside and one just outside the expected edge. */
    static const int16_t directions[16][2] = {
        {256, 0}, {237, 98}, {181, 181}, {98, 237},
        {0, 256}, {-98, 237}, {-181, 181}, {-237, 98},
        {-256, 0}, {-237, -98}, {-181, -181}, {-98, -237},
        {0, -256}, {98, -237}, {181, -181}, {237, -98},
    };
    const int center_x = static_cast<int>(component.sum_x / component.area);
    const int center_y = static_cast<int>(component.sum_y / component.area);
    const int box_width = component.right - component.left + 1;
    const int box_height = component.bottom - component.top + 1;
    int radius = (box_width + box_height + 2) / 4;
    if (radius < 2) radius = 2;

    float score_sum = 0.0f;
    uint8_t samples = 0U;
    for (const auto &direction : directions) {
        const int inner_radius = radius > 2 ? radius - 1 : radius;
        const int outer_radius = radius + 1;
        const int inner_x = center_x + direction[0] * inner_radius / 256;
        const int inner_y = center_y + direction[1] * inner_radius / 256;
        const int outer_x = center_x + direction[0] * outer_radius / 256;
        const int outer_y = center_y + direction[1] * outer_radius / 256;
        if (inner_x < 0 || inner_y < 0 || outer_x < 0 || outer_y < 0 ||
            inner_x >= static_cast<int>(width) ||
            outer_x >= static_cast<int>(width) ||
            inner_y >= static_cast<int>(height) ||
            outer_y >= static_cast<int>(height)) {
            continue;
        }
        const pixel_info_t inner = decode_pixel(
            pixels[static_cast<size_t>(inner_y) * width + inner_x],
            detector->config.rgb565_byte_swapped);
        const pixel_info_t outer = decode_pixel(
            pixels[static_cast<size_t>(outer_y) * width + outer_x],
            detector->config.rgb565_byte_swapped);
        const int luma_edge = abs((int)inner.luma - (int)outer.luma);
        const int chroma_edge = abs((int)inner.chroma - (int)outer.chroma);
        score_sum += clamp01((luma_edge + chroma_edge - 6.0f) / 24.0f);
        ++samples;
    }
    return samples == 0U ? 0.0f : score_sum / samples;
}

void evaluate_component(const white_ball_detector_t *detector,
                        const uint16_t *pixels, size_t width, size_t height,
                        const component_t &component,
                        white_ball_result_t *best)
{
    const white_ball_config_t &config = detector->config;
    if (component.area < config.minimum_area_px ||
        component.area > config.maximum_area_fraction * width * height) {
        return;
    }
    if (component.left == 0U || component.top == 0U ||
        component.right + 1U >= width || component.bottom + 1U >= height) {
        return;
    }

    const float box_width = component.right - component.left + 1U;
    const float box_height = component.bottom - component.top + 1U;
    const float aspect = box_width < box_height ? box_width / box_height
                                                : box_height / box_width;
    const float fill = component.area / (box_width * box_height);
    if (aspect < config.minimum_aspect_ratio ||
        fill < config.minimum_fill_ratio) return;

    const surroundings_t surroundings = measure_surroundings(
        detector, pixels, width, height, component);
    if (surroundings.luma_contrast <
        config.minimum_local_luma_contrast) return;
    const float edge = circular_edge_score(
        detector, pixels, width, height, component);
    if (edge < config.minimum_circular_edge_score) return;
    const float size_score = clamp01(
        (component.area - config.minimum_area_px + 1.0f) /
        (config.minimum_area_px * 5.0f));
    const float roundness = clamp01(
        0.55f * aspect + 0.45f *
        (1.0f - fabsf(fill - 0.78f) / 0.78f));
    const float brightness_score = clamp01(
        surroundings.luma_contrast /
        (2.0f * config.minimum_local_luma_contrast));
    const float confidence = clamp01(
        0.10f * size_score + 0.25f * roundness +
        0.30f * brightness_score + 0.25f * edge +
        0.10f * surroundings.shadow_score);
    if (confidence < config.minimum_confidence ||
        (best->valid && confidence <= best->confidence)) return;

    const float center_x = static_cast<float>(component.sum_x) / component.area;
    const float center_y = static_cast<float>(component.sum_y) / component.area;
    *best = {
        .valid = true,
        .luma_threshold = config.minimum_luma,
        .chroma_threshold = config.maximum_chroma,
        .confirmation_count = 0U,
        .area_px = component.area,
        .left = component.left,
        .top = component.top,
        .right = component.right,
        .bottom = component.bottom,
        .center_x = width <= 1U
                        ? 0.0f
                        : 2.0f * center_x / (width - 1U) - 1.0f,
        .center_y = height <= 1U ? 0.0f : center_y / (height - 1U),
        .diameter_px = 0.5f * (box_width + box_height),
        .luma_contrast = surroundings.luma_contrast,
        .chroma_contrast = surroundings.chroma_contrast,
        .circular_edge_score = edge,
        .shadow_score = surroundings.shadow_score,
        .confidence = confidence,
    };
}

bool same_target(const white_ball_result_t &previous,
                 const white_ball_result_t &current)
{
    if (!previous.valid || !current.valid) return false;
    const float dx = previous.center_x - current.center_x;
    const float dy = previous.center_y - current.center_y;
    const float position_distance = sqrtf(dx * dx + dy * dy);
    const float diameter_ratio =
        previous.diameter_px > current.diameter_px
            ? previous.diameter_px / current.diameter_px
            : current.diameter_px / previous.diameter_px;
    return position_distance <= 0.08f && diameter_ratio <= 1.6f;
}

void apply_temporal_confirmation(white_ball_detector_t *detector,
                                 const white_ball_result_t &candidate,
                                 white_ball_result_t *result)
{
    if (candidate.valid) {
        if (same_target(detector->previous_candidate, candidate)) {
            if (detector->confirmation_count < UINT8_MAX) {
                ++detector->confirmation_count;
            }
        } else {
            detector->confirmation_count = 1U;
        }
        detector->missed_frames = 0U;
        detector->previous_candidate = candidate;
        *result = candidate;
        result->confirmation_count = detector->confirmation_count;
        result->valid = detector->confirmation_count >= 3U;
        return;
    }

    if (detector->confirmation_count >= 3U &&
        detector->missed_frames < 1U &&
        detector->previous_candidate.valid) {
        ++detector->missed_frames;
        *result = detector->previous_candidate;
        result->confirmation_count = detector->confirmation_count;
        result->confidence *= 0.75f;
        return;
    }
    detector->confirmation_count = 0U;
    detector->missed_frames = 0U;
    memset(&detector->previous_candidate, 0,
           sizeof(detector->previous_candidate));
}

}  // namespace

extern "C" void white_ball_default_config(white_ball_config_t *config)
{
    if (config == nullptr) return;
    *config = {
        .rgb565_byte_swapped = true,
        .minimum_area_px = 4U,
        .maximum_area_fraction = 0.04f,
        /* Measured from the current camera return: ball luma 227-230,
         * paper luma 177-192. The ball is slightly blue, so colour purity is
         * only used to reject strongly coloured objects such as the LED. */
        .minimum_luma = 210U,
        .minimum_dim_luma = 120U,
        .local_highlight_difference = 6U,
        .maximum_luma = 255U,
        .maximum_chroma = 64U,
        .minimum_local_luma_contrast = 6U,
        .shadow_luma_difference = 16U,
        .minimum_aspect_ratio = 0.35f,
        .minimum_fill_ratio = 0.30f,
        .minimum_circular_edge_score = 0.12f,
        .minimum_confidence = 0.50f,
        /* After the installed camera's 180-degree correction the course
         * surface occupies the lower part of the frame. Ignore room clutter. */
        .roi_top_fraction = 0.25f,
        .roi_bottom_fraction = 0.90f,
    };
}

extern "C" bool white_ball_detector_init(
    white_ball_detector_t *detector, const white_ball_config_t *config)
{
    if (detector == nullptr || config == nullptr ||
        config->minimum_area_px == 0U ||
        config->maximum_area_fraction <= 0.0f ||
        config->maximum_area_fraction >= 1.0f ||
        config->minimum_dim_luma > config->minimum_luma ||
        config->local_highlight_difference == 0U ||
        config->minimum_local_luma_contrast == 0U ||
        config->roi_top_fraction < 0.0f ||
        config->roi_bottom_fraction > 1.0f ||
        config->roi_top_fraction >= config->roi_bottom_fraction) {
        return false;
    }
    memset(detector, 0, sizeof(*detector));
    detector->config = *config;
    return true;
}

extern "C" bool white_ball_detect_rgb565(
    white_ball_detector_t *detector, const uint16_t *pixels, size_t width,
    size_t height, white_ball_result_t *result)
{
    if (detector == nullptr || pixels == nullptr || result == nullptr ||
        width == 0U || height == 0U || width > WHITE_BALL_MAX_WIDTH ||
        height > WHITE_BALL_MAX_HEIGHT) return false;

    const size_t pixel_count = width * height;
    memset(detector->mask, 0, pixel_count);
    const size_t roi_y0 = static_cast<size_t>(
        detector->config.roi_top_fraction * height);
    size_t roi_y1 = static_cast<size_t>(
        detector->config.roi_bottom_fraction * height);
    if (roi_y1 > height) roi_y1 = height;
    for (size_t y = roi_y0; y < roi_y1; ++y) {
        for (size_t x = 0U; x < width; ++x) {
            const size_t index = y * width + x;
            const pixel_info_t pixel = decode_pixel(
                pixels[index], detector->config.rgb565_byte_swapped);
            const bool directly_lit =
                pixel.luma >= detector->config.minimum_luma;
            const bool shadow_highlight = is_local_highlight(
                detector, pixels, width, height, x, y, pixel);
            detector->mask[index] = static_cast<uint8_t>(
                (directly_lit || shadow_highlight) &&
                pixel.luma <= detector->config.maximum_luma &&
                pixel.chroma <= detector->config.maximum_chroma);
        }
    }
    close_small_gaps(detector, width, height, roi_y0, roi_y1);

    white_ball_result_t candidate = {};
    candidate.luma_threshold = detector->config.minimum_luma;
    candidate.chroma_threshold = detector->config.maximum_chroma;
    for (size_t seed = 0U; seed < pixel_count; ++seed) {
        if (detector->mask[seed] == 0U) continue;
        detector->mask[seed] = 0U;
        size_t head = 0U;
        size_t tail = 0U;
        detector->queue[tail++] = static_cast<uint16_t>(seed);
        component_t component = {
            .area = 0U,
            .left = static_cast<uint16_t>(seed % width),
            .top = static_cast<uint16_t>(seed / width),
            .right = static_cast<uint16_t>(seed % width),
            .bottom = static_cast<uint16_t>(seed / width),
            .sum_x = 0U,
            .sum_y = 0U,
            .sum_luma = 0U,
            .sum_chroma = 0U,
        };

        while (head < tail) {
            const size_t index = detector->queue[head++];
            const size_t x = index % width;
            const size_t y = index / width;
            const pixel_info_t pixel = decode_pixel(
                pixels[index], detector->config.rgb565_byte_swapped);
            ++component.area;
            component.sum_x += x;
            component.sum_y += y;
            component.sum_luma += pixel.luma;
            component.sum_chroma += pixel.chroma;
            if (x < component.left) component.left = static_cast<uint16_t>(x);
            if (x > component.right) component.right = static_cast<uint16_t>(x);
            if (y < component.top) component.top = static_cast<uint16_t>(y);
            if (y > component.bottom) component.bottom = static_cast<uint16_t>(y);

            for (int dy = -1; dy <= 1; ++dy) {
                for (int dx = -1; dx <= 1; ++dx) {
                    if (dx == 0 && dy == 0) continue;
                    const int nx = static_cast<int>(x) + dx;
                    const int ny = static_cast<int>(y) + dy;
                    if (nx < 0 || ny < 0 || nx >= static_cast<int>(width) ||
                        ny >= static_cast<int>(height)) continue;
                    const size_t neighbour = static_cast<size_t>(ny) * width +
                                             static_cast<size_t>(nx);
                    if (detector->mask[neighbour] == 0U) continue;
                    detector->mask[neighbour] = 0U;
                    detector->queue[tail++] =
                        static_cast<uint16_t>(neighbour);
                }
            }
        }
        evaluate_component(detector, pixels, width, height, component,
                           &candidate);
    }

    memset(result, 0, sizeof(*result));
    result->luma_threshold = detector->config.minimum_luma;
    result->chroma_threshold = detector->config.maximum_chroma;
    apply_temporal_confirmation(detector, candidate, result);
    return true;
}

