#include "ball_detector.h"

#include <limits.h>
#include <stdlib.h>
#include <string.h>

#include "esp_heap_caps.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define BALL_WORK_WIDTH 320
#define BALL_SAMPLES 32

static const int16_t s_ux[BALL_SAMPLES] = {
    1024,1004,946,851,724,569,392,200,0,-200,-392,-569,-724,-851,-946,-1004,
    -1024,-1004,-946,-851,-724,-569,-392,-200,0,200,392,569,724,851,946,1004
};
static const int16_t s_uy[BALL_SAMPLES] = {
    0,200,392,569,724,851,946,1004,1024,1004,946,851,724,569,392,200,
    0,-200,-392,-569,-724,-851,-946,-1004,-1024,-1004,-946,-851,-724,-569,-392,-200
};

static inline int iabs(int value) { return value < 0 ? -value : value; }
static inline int imin(int a, int b) { return a < b ? a : b; }
static inline int imax(int a, int b) { return a > b ? a : b; }
static inline int iclamp_gradient(int value)
{
    return value < -127 ? -127 : (value > 127 ? 127 : value);
}

static void unpack_wire_rgb565(uint16_t wire, uint8_t *gray, uint8_t *chroma)
{
    const uint16_t rgb = (uint16_t)((wire << 8) | (wire >> 8));
    const int r = ((rgb >> 11) & 31) * 255 / 31;
    const int g = ((rgb >> 5) & 63) * 255 / 63;
    const int b = (rgb & 31) * 255 / 31;
    *gray = (uint8_t)((77 * r + 150 * g + 29 * b) >> 8);
    *chroma = (uint8_t)(imax(r, imax(g, b)) - imin(r, imin(g, b)));
}

static void *ball_alloc(size_t bytes, bool clear)
{
    void *memory = clear
        ? heap_caps_calloc(1, bytes, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT)
        : heap_caps_malloc(bytes, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!memory) memory = clear ? calloc(1, bytes) : malloc(bytes);
    return memory;
}

esp_err_t ball_detector_process_rgb565(const uint16_t *pixels,
                                       uint16_t width, uint16_t height,
                                       const ball_detection_t *search_hint,
                                       ball_detector_result_t *result)
{
    if (!pixels || !result || width < 32 || height < 24) return ESP_ERR_INVALID_ARG;
    memset(result, 0, sizeof(*result));

    const int work_w = width > BALL_WORK_WIDTH ? BALL_WORK_WIDTH : width;
    const int work_h = (int)height * work_w / width;
    const size_t count = (size_t)work_w * work_h;
    uint8_t *gray = ball_alloc(count, false);
    uint8_t *chroma = ball_alloc(count, false);
    int8_t *gradient_memory = heap_caps_calloc(
        count * 2, 1, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    if (!gradient_memory) gradient_memory = ball_alloc(count * 2, true);
    int8_t *gx = gradient_memory;
    int8_t *gy = gradient_memory ? gradient_memory + count : NULL;
    if (!gray || !chroma || !gradient_memory) {
        free(gray); free(chroma); free(gradient_memory);
        return ESP_ERR_NO_MEM;
    }

    /* Integer area resampling is identical in structure to the host test. */
    for (int y = 0; y < work_h; ++y) {
        const int sy0 = y * height / work_h;
        const int sy1 = imax(sy0 + 1, (y + 1) * height / work_h);
        for (int x = 0; x < work_w; ++x) {
            const int sx0 = x * width / work_w;
            const int sx1 = imax(sx0 + 1, (x + 1) * width / work_w);
            uint32_t gray_sum = 0, chroma_sum = 0, samples = 0;
            for (int sy = sy0; sy < sy1; ++sy) {
                for (int sx = sx0; sx < sx1; ++sx) {
                    uint8_t lum, chr;
                    unpack_wire_rgb565(pixels[(size_t)sy * width + sx], &lum, &chr);
                    gray_sum += lum;
                    chroma_sum += chr;
                    ++samples;
                }
            }
            gray[(size_t)y * work_w + x] = (uint8_t)(gray_sum / samples);
            chroma[(size_t)y * work_w + x] = (uint8_t)(chroma_sum / samples);
        }
    }

    for (int y = 1; y + 1 < work_h; ++y) {
        for (int x = 1; x + 1 < work_w; ++x) {
            const size_t p = (size_t)y * work_w + x;
#define G(DX,DY) ((int)gray[(size_t)(y+(DY))*work_w+x+(DX)])
            gx[p] = (int8_t)iclamp_gradient(-G(-1,-1)+G(1,-1)-2*G(-1,0)+2*G(1,0)-G(-1,1)+G(1,1));
            gy[p] = (int8_t)iclamp_gradient(-G(-1,-1)-2*G(0,-1)-G(1,-1)+G(-1,1)+2*G(0,1)+G(1,1));
#undef G
        }
    }

    /* Fixed camera + fixed physical ball: fit from the 11 supplied images at
     * 320 px working width is r ~= 15 - 11*y/h. Search only a +/-5 px band,
     * scaled when a smaller native frame is supplied. */
    const int min_r = imax(3, 6 * work_w / BALL_WORK_WIDTH);
    const int max_r = imax(min_r + 1, 20 * work_w / BALL_WORK_WIDTH);
    const bool local_search = search_hint != NULL && search_hint->radius != 0;
    const int hint_x = local_search ? (int)search_hint->x * work_w / width : 0;
    const int hint_y = local_search ? (int)search_hint->y * work_h / height : 0;
    const int hint_r = local_search ? imax(1,
        (int)search_hint->radius * work_w / width) : 0;
    const int tracking_half_window = imax(32, hint_r * 2);
    int32_t best_score = INT32_MIN;
    int best_x = 0, best_y = 0, best_r = 0, best_support = 0;

    for (int r = min_r; r <= max_r; ++r) {
        const int margin = (3 * r + 1) / 2 + 2;
        for (int cy = margin; cy + margin < work_h; cy += 2) {
            if (local_search && iabs(cy - hint_y) > tracking_half_window) continue;
            const int expected_320 = imax(6, imin(15,
                15 - 11 * cy / work_h));
            const int row_min_r = imax(min_r,
                (expected_320 - 5) * work_w / BALL_WORK_WIDTH);
            const int row_max_r = imin(max_r, imax(row_min_r + 1,
                (expected_320 + 5) * work_w / BALL_WORK_WIDTH));
            if (r < row_min_r || r > row_max_r) continue;
            if (local_search && iabs(r - hint_r) > 3) continue;
            int cx_begin = margin;
            int cx_end = work_w - margin - 1;
            if (local_search) {
                cx_begin = imax(cx_begin, hint_x - tracking_half_window);
                cx_end = imin(cx_end, hint_x + tracking_half_window);
            }
            for (int cx = cx_begin; cx <= cx_end; cx += 2) {
                const int half = imax(1, r/2);
                const int side = imax(1, r/3);
                const int quick_upper =
                    (gray[(size_t)(cy-half)*work_w+cx-side] +
                     gray[(size_t)(cy-half)*work_w+cx] +
                     gray[(size_t)(cy-half)*work_w+cx+side]) / 3;
                const int quick_lower =
                    (gray[(size_t)(cy+half)*work_w+cx-side] +
                     gray[(size_t)(cy+half)*work_w+cx] +
                     gray[(size_t)(cy+half)*work_w+cx+side]) / 3;
                if (quick_lower - quick_upper < 5) continue;
                int support = 0, edge_sum = 0, top_positive = 0, bottom_negative = 0;
                int quadrant[4] = {0};
                int upper = 0, lower = 0, upper_n = 0, lower_n = 0;
                int outer_sum = 0, inner_sum = 0, chroma_sum = 0, outer_texture = 0;
                for (int k = 0; k < BALL_SAMPLES; ++k) {
                    const int bx = cx + (r * s_ux[k] + 512) / 1024;
                    const int by = cy + (r * s_uy[k] + 512) / 1024;
                    const size_t bp = (size_t)by * work_w + bx;
                    const int signed_radial = (gx[bp]*s_ux[k] + gy[bp]*s_uy[k]) / 1024;
                    const int radial = iabs(signed_radial);
                    const int tangent = iabs((-gx[bp]*s_uy[k] + gy[bp]*s_ux[k]) / 1024);
                    if (radial >= 28 && radial * 3 >= tangent * 2) {
                        ++support; ++quadrant[k/8]; edge_sum += imin(radial, 255);
                    }
                    if (s_uy[k] < -300 && signed_radial >= 20) ++top_positive;
                    if (s_uy[k] > 300 && signed_radial <= -20) ++bottom_negative;
                    const int ix = cx + (r*5*s_ux[k] + 4096) / 8192;
                    const int iy = cy + (r*5*s_uy[k] + 4096) / 8192;
                    const int ox = cx + (r*11*s_ux[k] + 5120) / 10240;
                    const int oy = cy + (r*11*s_uy[k] + 5120) / 10240;
                    const int tx = cx + (r*3*s_ux[k] + 1024) / 2048;
                    const int ty = cy + (r*3*s_uy[k] + 1024) / 2048;
                    const int value = gray[(size_t)iy*work_w+ix];
                    inner_sum += value;
                    chroma_sum += chroma[(size_t)iy*work_w+ix];
                    outer_sum += gray[(size_t)oy*work_w+ox];
                    outer_texture += iabs(gx[(size_t)ty*work_w+tx]) + iabs(gy[(size_t)ty*work_w+tx]);
                    if (s_uy[k] < -300) { upper += value; ++upper_n; }
                    if (s_uy[k] > 300) { lower += value; ++lower_n; }
                }
                int min_quadrant = quadrant[0];
                for (int q = 1; q < 4; ++q) min_quadrant = imin(min_quadrant, quadrant[q]);
                if (support < 13 || min_quadrant < 1 || top_positive < 2 || bottom_negative < 2) continue;
                const int upper_mean = upper / upper_n;
                const int lower_mean = lower / lower_n;
                const int inner_mean = inner_sum / BALL_SAMPLES;
                const int outer_mean = outer_sum / BALL_SAMPLES;
                const int vertical = lower_mean - upper_mean;
                const bool close_upper_ball = cy * 3 < work_h &&
                    r * BALL_WORK_WIDTH >= 13 * work_w;
                const int minimum_outer = close_upper_ball ? 145 : 160;
                const int maximum_chroma = close_upper_ball ? 25 : 22;
                if (lower_mean < 105 || outer_mean < minimum_outer || vertical < 7 ||
                    chroma_sum/BALL_SAMPLES > maximum_chroma ||
                    outer_texture/BALL_SAMPLES > 45) continue;
                const int32_t score = edge_sum + support*70 + vertical*34
                                    - iabs(inner_mean-outer_mean)*5 - outer_texture/3;
                if (score > best_score) {
                    best_score = score; best_x = cx; best_y = cy;
                    best_r = r; best_support = support;
                }
            }
        }
        /* Let the idle task feed the watchdog between radius passes. */
        vTaskDelay(1);
    }

    if (best_score != INT32_MIN) {
        result->found = true;
        result->ball.x = (uint16_t)((uint32_t)best_x * width / work_w);
        result->ball.y = (uint16_t)((uint32_t)best_y * height / work_h);
        result->ball.radius = (uint16_t)((uint32_t)best_r * width / work_w);
        result->ball.score = best_score;
        result->ball.coverage = (uint8_t)best_support;
    }
    free(gray); free(chroma); free(gradient_memory);
    return ESP_OK;
}

void ball_detector_tracker_init(ball_detector_tracker_t *tracker,
                                uint8_t required_frames,
                                uint16_t max_center_jump)
{
    if (!tracker) return;
    memset(tracker, 0, sizeof(*tracker));
    tracker->required_frames = required_frames ? required_frames : 1;
    tracker->max_center_jump = max_center_jump;
}

bool ball_detector_tracker_update(ball_detector_tracker_t *tracker,
                                  const ball_detector_result_t *result,
                                  ball_detection_t *stable_ball)
{
    if (!tracker || !result || !result->found) {
        if (tracker) tracker->consecutive_frames = 0;
        return false;
    }
    const ball_detection_t detected = result->ball;
    bool near = false;
    if (tracker->has_candidate) {
        const int dx = (int)detected.x - tracker->candidate.x;
        const int dy = (int)detected.y - tracker->candidate.y;
        near = dx*dx + dy*dy <= (int)tracker->max_center_jump * tracker->max_center_jump;
    }
    if (!near) {
        tracker->candidate = detected;
        tracker->has_candidate = true;
        tracker->consecutive_frames = 1;
        return tracker->required_frames == 1;
    }
    if (tracker->consecutive_frames < tracker->required_frames) ++tracker->consecutive_frames;
    tracker->candidate.x = (uint16_t)((3U*tracker->candidate.x + detected.x)/4U);
    tracker->candidate.y = (uint16_t)((3U*tracker->candidate.y + detected.y)/4U);
    tracker->candidate.radius = (uint16_t)((3U*tracker->candidate.radius + detected.radius)/4U);
    tracker->candidate.score = detected.score;
    tracker->candidate.coverage = detected.coverage;
    if (tracker->consecutive_frames < tracker->required_frames) return false;
    if (stable_ball) *stable_ball = tracker->candidate;
    return true;
}
