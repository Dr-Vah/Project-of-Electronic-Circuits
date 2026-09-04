#include "ball_target_detector.h"

#include <limits.h>
#include <math.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "esp_heap_caps.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define BT_WORK_WIDTH 320
#define BT_BALL_SAMPLES 32
#define BT_BALL_REQUIRED_FRAMES 2
#define BT_BALL_MAX_CENTER_JUMP 24
#define BT_ORANGE_REQUIRED_FRAMES 2
#define BT_ORANGE_MAX_CENTER_JUMP 32
#define BT_ORANGE_MIN_RED 165
#define BT_ORANGE_MIN_RED_MINUS_GREEN 30
#define BT_ORANGE_MIN_GREEN_MINUS_BLUE 5
#define BT_TARGET_REQUIRED_FRAMES 2
#define BT_TARGET_MAX_CENTER_JUMP 120
#define BT_DARK_THRESHOLD 110
#define BT_TARGET_MAX_BOX_WIDTH 48
#define BT_TARGET_MAX_BOX_HEIGHT 20

static const char s_visualization_html[] =
    "<!doctype html><html><head><meta charset='utf-8'>"
    "<meta name='viewport' content='width=device-width,initial-scale=1'>"
    "<title>Ball and Target Detector</title><style>"
    "body{margin:0;background:#111;color:#eee;font-family:system-ui;text-align:center}"
    "h2{margin:12px 0 4px}.view{position:relative;display:inline-block;max-width:100vw}"
    "#cam{display:block;max-width:100vw;height:auto}#mark{position:absolute;inset:0}"
    "#status{margin:8px;font:16px monospace}</style></head><body>"
    "<h2>ESP32-S3 Ball and Target Detector</h2><div class='view'>"
    "<img id='cam'><canvas id='mark'></canvas></div><div id='status'>waiting...</div>"
    "<script>const img=document.getElementById('cam'),cv=document.getElementById('mark'),"
    "ctx=cv.getContext('2d'),st=document.getElementById('status');let det=null,busy=false;"
    "function overlay(){const w=img.clientWidth,h=img.clientHeight;"
    "if(cv.width!==w||cv.height!==h){cv.width=w;cv.height=h}ctx.clearRect(0,0,w,h);"
    "if(!det||!det.width||!det.height)return;const sx=w/det.width,sy=h/det.height,cx=w*.43;"
    "ctx.strokeStyle='#ff00ff';ctx.lineWidth=3;ctx.beginPath();ctx.moveTo(cx-4,2);"
    "ctx.lineTo(cx+4,2);ctx.moveTo(cx,0);ctx.lineTo(cx,6);ctx.stroke();"
    "if(det.target){const t=det.target;ctx.strokeStyle='#2080ff';ctx.beginPath();"
    "ctx.moveTo(cx,0);ctx.lineTo(t.x*sx,t.y*sy);ctx.stroke()}"
    "ctx.strokeStyle='#00dfff';ctx.lineWidth=3;for(const t of det.targets||[])"
    "ctx.strokeRect(t.x0*sx,t.y0*sy,(t.x1-t.x0)*sx,(t.y1-t.y0)*sy);"
    "if(det.ball){const b=det.ball;ctx.strokeStyle='#00ff38';ctx.beginPath();"
    "ctx.arc(b.x*sx,b.y*sy,b.radius*sx,0,Math.PI*2);ctx.stroke();"
    "ctx.fillStyle='#ff2020';ctx.beginPath();ctx.arc(b.x*sx,b.y*sy,3,0,Math.PI*2);ctx.fill()}"
    "if(det.orange){const b=det.orange;ctx.strokeStyle='#ff8c00';ctx.beginPath();"
    "ctx.arc(b.x*sx,b.y*sy,b.radius*sx,0,Math.PI*2);ctx.stroke();"
    "ctx.fillStyle='#ffff00';ctx.beginPath();ctx.arc(b.x*sx,b.y*sy,3,0,Math.PI*2);ctx.fill()}"
    "st.textContent=`white=${!!det.ball} orange=${!!det.orange} targets=${(det.targets||[]).length}`;}"
    "async function frame(){if(busy)return;busy=true;try{const r=await fetch('/frame.jpg?t='+Date.now(),{cache:'no-store'});"
    "if(r.ok){const b=await r.blob(),u=URL.createObjectURL(b);img.onload=()=>{URL.revokeObjectURL(u);overlay()};img.src=u}}"
    "catch(e){st.textContent='frame connection lost'}finally{busy=false;setTimeout(frame,30)}}"
    "async function detection(){try{const r=await fetch('/detection?t='+Date.now(),{cache:'no-store'});"
    "if(r.ok){det=await r.json();overlay()}}catch(e){}setTimeout(detection,150)}"
    "frame();detection();window.onresize=overlay;</script></body></html>";

typedef struct {
    uint16_t source_width;
    uint16_t source_height;
    uint16_t width;
    uint16_t height;
    uint8_t *storage;
    uint8_t *gray;
    uint8_t *chroma;
    uint8_t *red;
    uint8_t *green;
    uint8_t *blue;
} bt_preprocessed_frame_t;

typedef struct {
    bool found;
    bt_ball_t ball;
} bt_raw_ball_result_t;

typedef struct {
    bool found;
    uint8_t count;
    bt_target_t target;
    bt_target_t targets[BT_TARGET_COUNT];
} bt_raw_target_result_t;

typedef struct {
    bool valid;
    int x0;
    int x1;
    int y0;
    int y1;
    int x;
    int y;
    int area;
    int mean;
    int32_t score;
} bt_target_candidate_t;

static const int16_t s_circle_x[BT_BALL_SAMPLES] = {
    1024, 1004, 946, 851, 724, 569, 392, 200,
    0, -200, -392, -569, -724, -851, -946, -1004,
    -1024, -1004, -946, -851, -724, -569, -392, -200,
    0, 200, 392, 569, 724, 851, 946, 1004,
};

static const int16_t s_circle_y[BT_BALL_SAMPLES] = {
    0, 200, 392, 569, 724, 851, 946, 1004,
    1024, 1004, 946, 851, 724, 569, 392, 200,
    0, -200, -392, -569, -724, -851, -946, -1004,
    -1024, -1004, -946, -851, -724, -569, -392, -200,
};

static inline int bt_abs(int value)
{
    return value < 0 ? -value : value;
}

static inline int bt_min(int a, int b)
{
    return a < b ? a : b;
}

static inline int bt_max(int a, int b)
{
    return a > b ? a : b;
}

static inline int bt_clamp_gradient(int value)
{
    return value < -127 ? -127 : (value > 127 ? 127 : value);
}

static void *bt_alloc(size_t bytes, bool clear, uint32_t capabilities)
{
    void *memory = clear ? heap_caps_calloc(1, bytes, capabilities)
                         : heap_caps_malloc(bytes, capabilities);
    if (memory == NULL) {
        memory = clear ? calloc(1, bytes) : malloc(bytes);
    }
    return memory;
}

static esp_err_t bt_preprocess_rgb565(const uint16_t *pixels,
                                      uint16_t width,
                                      uint16_t height,
                                      bt_preprocessed_frame_t *frame)
{
    if (pixels == NULL || frame == NULL || width < 32 || height < 24) {
        return ESP_ERR_INVALID_ARG;
    }
    memset(frame, 0, sizeof(*frame));

    const int work_width = width > BT_WORK_WIDTH ? BT_WORK_WIDTH : width;
    const int work_height = (int)height * work_width / width;
    const size_t pixel_count = (size_t)work_width * work_height;
    uint8_t *storage = bt_alloc(pixel_count * 5, false,
                                MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (storage == NULL) {
        return ESP_ERR_NO_MEM;
    }

    frame->source_width = width;
    frame->source_height = height;
    frame->width = (uint16_t)work_width;
    frame->height = (uint16_t)work_height;
    frame->storage = storage;
    frame->gray = storage;
    frame->chroma = storage + pixel_count;
    frame->red = storage + pixel_count * 2;
    frame->green = storage + pixel_count * 3;
    frame->blue = storage + pixel_count * 4;

    for (int y = 0; y < work_height; ++y) {
        const int source_y0 = y * height / work_height;
        const int source_y1 = bt_max(source_y0 + 1,
                                     (y + 1) * height / work_height);
        for (int x = 0; x < work_width; ++x) {
            const int source_x0 = x * width / work_width;
            const int source_x1 = bt_max(source_x0 + 1,
                                         (x + 1) * width / work_width);
            uint32_t sum_r = 0;
            uint32_t sum_g = 0;
            uint32_t sum_b = 0;
            uint32_t sum_gray = 0;
            uint32_t sum_chroma = 0;
            uint32_t samples = 0;

            for (int source_y = source_y0; source_y < source_y1; ++source_y) {
                for (int source_x = source_x0; source_x < source_x1; ++source_x) {
                    const uint16_t wire =
                        pixels[(size_t)source_y * width + source_x];
                    const uint16_t rgb =
                        (uint16_t)((wire << 8) | (wire >> 8));
                    const int r = ((rgb >> 11) & 31U) * 255U / 31U;
                    const int g = ((rgb >> 5) & 63U) * 255U / 63U;
                    const int b = (rgb & 31U) * 255U / 31U;
                    sum_r += r;
                    sum_g += g;
                    sum_b += b;
                    sum_gray += (77U * r + 150U * g + 29U * b) >> 8;
                    sum_chroma += bt_max(r, bt_max(g, b)) -
                                  bt_min(r, bt_min(g, b));
                    ++samples;
                }
            }

            const size_t output = (size_t)y * work_width + x;
            frame->red[output] = (uint8_t)(sum_r / samples);
            frame->green[output] = (uint8_t)(sum_g / samples);
            frame->blue[output] = (uint8_t)(sum_b / samples);
            frame->gray[output] = (uint8_t)(sum_gray / samples);
            frame->chroma[output] = (uint8_t)(sum_chroma / samples);
        }
    }
    return ESP_OK;
}

static void bt_preprocess_release(bt_preprocessed_frame_t *frame)
{
    if (frame == NULL) {
        return;
    }
    free(frame->storage);
    memset(frame, 0, sizeof(*frame));
}

static esp_err_t bt_detect_white_ball(const bt_preprocessed_frame_t *frame,
                                      const bt_ball_t *search_hint,
                                      bt_raw_ball_result_t *result)
{
    if (frame == NULL || frame->storage == NULL || result == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    memset(result, 0, sizeof(*result));

    const int source_width = frame->source_width;
    const int source_height = frame->source_height;
    const int width = frame->width;
    const int height = frame->height;
    const size_t pixel_count = (size_t)width * height;
    const uint8_t *gray = frame->gray;
    const uint8_t *chroma = frame->chroma;
    int8_t *gradient = heap_caps_calloc(
        pixel_count * 2, 1, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    if (gradient == NULL) {
        gradient = bt_alloc(pixel_count * 2, true,
                            MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    }
    if (gradient == NULL) {
        return ESP_ERR_NO_MEM;
    }
    int8_t *gradient_x = gradient;
    int8_t *gradient_y = gradient + pixel_count;

    for (int y = 1; y + 1 < height; ++y) {
        for (int x = 1; x + 1 < width; ++x) {
            const size_t p = (size_t)y * width + x;
#define BT_GRAY(DX, DY) ((int)gray[(size_t)(y + (DY)) * width + x + (DX)])
            gradient_x[p] = (int8_t)bt_clamp_gradient(
                -BT_GRAY(-1, -1) + BT_GRAY(1, -1) -
                2 * BT_GRAY(-1, 0) + 2 * BT_GRAY(1, 0) -
                BT_GRAY(-1, 1) + BT_GRAY(1, 1));
            gradient_y[p] = (int8_t)bt_clamp_gradient(
                -BT_GRAY(-1, -1) - 2 * BT_GRAY(0, -1) - BT_GRAY(1, -1) +
                BT_GRAY(-1, 1) + 2 * BT_GRAY(0, 1) + BT_GRAY(1, 1));
#undef BT_GRAY
        }
    }

    const int minimum_radius = bt_max(3, 6 * width / BT_WORK_WIDTH);
    const int maximum_radius =
        bt_max(minimum_radius + 1, 20 * width / BT_WORK_WIDTH);
    const bool local_search = search_hint != NULL && search_hint->radius != 0;
    const int hint_x = local_search
        ? (int)search_hint->x * width / source_width : 0;
    const int hint_y = local_search
        ? (int)search_hint->y * height / source_height : 0;
    const int hint_radius = local_search
        ? bt_max(1, (int)search_hint->radius * width / source_width) : 0;
    const int tracking_half_window = bt_max(32, hint_radius * 2);
    int32_t best_score = INT32_MIN;
    int best_x = 0;
    int best_y = 0;
    int best_radius = 0;
    int best_support = 0;

    for (int radius = minimum_radius; radius <= maximum_radius; ++radius) {
        const int margin = (3 * radius + 1) / 2 + 2;
        for (int center_y = margin; center_y + margin < height; center_y += 2) {
            if (local_search &&
                bt_abs(center_y - hint_y) > tracking_half_window) {
                continue;
            }
            const int expected_radius_320 = bt_max(
                6, bt_min(15, 15 - 11 * center_y / height));
            const int row_minimum_radius = bt_max(
                minimum_radius,
                (expected_radius_320 - 5) * width / BT_WORK_WIDTH);
            const int row_maximum_radius = bt_min(
                maximum_radius,
                bt_max(row_minimum_radius + 1,
                       (expected_radius_320 + 5) * width / BT_WORK_WIDTH));
            if (radius < row_minimum_radius || radius > row_maximum_radius) {
                continue;
            }
            if (local_search && bt_abs(radius - hint_radius) > 3) {
                continue;
            }

            int center_x_begin = margin;
            int center_x_end = width - margin - 1;
            if (local_search) {
                center_x_begin = bt_max(center_x_begin,
                                        hint_x - tracking_half_window);
                center_x_end = bt_min(center_x_end,
                                      hint_x + tracking_half_window);
            }

            for (int center_x = center_x_begin;
                 center_x <= center_x_end; center_x += 2) {
                const int half = bt_max(1, radius / 2);
                const int side = bt_max(1, radius / 3);
                const int quick_upper =
                    (gray[(size_t)(center_y - half) * width + center_x - side] +
                     gray[(size_t)(center_y - half) * width + center_x] +
                     gray[(size_t)(center_y - half) * width + center_x + side]) / 3;
                const int quick_lower =
                    (gray[(size_t)(center_y + half) * width + center_x - side] +
                     gray[(size_t)(center_y + half) * width + center_x] +
                     gray[(size_t)(center_y + half) * width + center_x + side]) / 3;
                if (quick_lower - quick_upper < 5) {
                    continue;
                }

                int support = 0;
                int edge_sum = 0;
                int top_positive = 0;
                int bottom_negative = 0;
                int quadrant[4] = {0};
                int upper = 0;
                int lower = 0;
                int upper_count = 0;
                int lower_count = 0;
                int outer_sum = 0;
                int inner_sum = 0;
                int chroma_sum = 0;
                int outer_texture = 0;

                for (int sample = 0; sample < BT_BALL_SAMPLES; ++sample) {
                    const int unit_x = s_circle_x[sample];
                    const int unit_y = s_circle_y[sample];
                    const int boundary_x =
                        center_x + (radius * unit_x + 512) / 1024;
                    const int boundary_y =
                        center_y + (radius * unit_y + 512) / 1024;
                    const size_t boundary =
                        (size_t)boundary_y * width + boundary_x;
                    const int signed_radial =
                        (gradient_x[boundary] * unit_x +
                         gradient_y[boundary] * unit_y) / 1024;
                    const int radial = bt_abs(signed_radial);
                    const int tangent = bt_abs(
                        (-gradient_x[boundary] * unit_y +
                         gradient_y[boundary] * unit_x) / 1024);
                    if (radial >= 28 && radial * 3 >= tangent * 2) {
                        ++support;
                        ++quadrant[sample / 8];
                        edge_sum += bt_min(radial, 255);
                    }
                    if (unit_y < -300 && signed_radial >= 20) {
                        ++top_positive;
                    }
                    if (unit_y > 300 && signed_radial <= -20) {
                        ++bottom_negative;
                    }

                    const int inner_x =
                        center_x + (radius * 5 * unit_x + 4096) / 8192;
                    const int inner_y =
                        center_y + (radius * 5 * unit_y + 4096) / 8192;
                    const int outer_x =
                        center_x + (radius * 11 * unit_x + 5120) / 10240;
                    const int outer_y =
                        center_y + (radius * 11 * unit_y + 5120) / 10240;
                    const int texture_x =
                        center_x + (radius * 3 * unit_x + 1024) / 2048;
                    const int texture_y =
                        center_y + (radius * 3 * unit_y + 1024) / 2048;
                    const int value = gray[(size_t)inner_y * width + inner_x];
                    inner_sum += value;
                    chroma_sum += chroma[(size_t)inner_y * width + inner_x];
                    outer_sum += gray[(size_t)outer_y * width + outer_x];
                    outer_texture +=
                        bt_abs(gradient_x[(size_t)texture_y * width + texture_x]) +
                        bt_abs(gradient_y[(size_t)texture_y * width + texture_x]);
                    if (unit_y < -300) {
                        upper += value;
                        ++upper_count;
                    }
                    if (unit_y > 300) {
                        lower += value;
                        ++lower_count;
                    }
                }

                int minimum_quadrant = quadrant[0];
                for (int quadrant_index = 1; quadrant_index < 4;
                     ++quadrant_index) {
                    minimum_quadrant = bt_min(minimum_quadrant,
                                              quadrant[quadrant_index]);
                }
                if (support < 13 || minimum_quadrant < 1 ||
                    top_positive < 2 || bottom_negative < 2) {
                    continue;
                }

                const int upper_mean = upper / upper_count;
                const int lower_mean = lower / lower_count;
                const int inner_mean = inner_sum / BT_BALL_SAMPLES;
                const int outer_mean = outer_sum / BT_BALL_SAMPLES;
                const int vertical_difference = lower_mean - upper_mean;
                const bool close_upper_ball = center_y * 3 < height &&
                    radius * BT_WORK_WIDTH >= 13 * width;
                const int minimum_outer = close_upper_ball ? 145 : 160;
                const int maximum_chroma = close_upper_ball ? 25 : 22;
                if (lower_mean < 105 || outer_mean < minimum_outer ||
                    vertical_difference < 7 ||
                    chroma_sum / BT_BALL_SAMPLES > maximum_chroma ||
                    outer_texture / BT_BALL_SAMPLES > 45) {
                    continue;
                }

                const int32_t score =
                    edge_sum + support * 70 + vertical_difference * 34 -
                    bt_abs(inner_mean - outer_mean) * 5 - outer_texture / 3;
                if (score > best_score) {
                    best_score = score;
                    best_x = center_x;
                    best_y = center_y;
                    best_radius = radius;
                    best_support = support;
                }
            }
        }
        vTaskDelay(1);
    }

    if (best_score != INT32_MIN) {
        result->found = true;
        result->ball.x =
            (uint16_t)((uint32_t)best_x * source_width / width);
        result->ball.y =
            (uint16_t)((uint32_t)best_y * source_height / height);
        result->ball.radius =
            (uint16_t)((uint32_t)best_radius * source_width / width);
        result->ball.score = best_score;
        result->ball.coverage = (uint8_t)best_support;
    }

    free(gradient);
    return ESP_OK;
}

static bool bt_is_orange(const bt_preprocessed_frame_t *frame, size_t pixel)
{
    const int red = frame->red[pixel];
    const int green = frame->green[pixel];
    const int blue = frame->blue[pixel];
    return red >= BT_ORANGE_MIN_RED &&
           red - green >= BT_ORANGE_MIN_RED_MINUS_GREEN &&
           green - blue >= BT_ORANGE_MIN_GREEN_MINUS_BLUE;
}

static esp_err_t bt_detect_orange_ball(const bt_preprocessed_frame_t *frame,
                                       bt_raw_ball_result_t *result)
{
    if (frame == NULL || frame->storage == NULL || result == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    memset(result, 0, sizeof(*result));

    const int width = frame->width;
    const int height = frame->height;
    const size_t pixel_count = (size_t)width * height;
    uint8_t *visited = bt_alloc(pixel_count, true,
                                MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    uint32_t *stack = bt_alloc(pixel_count * sizeof(*stack), false,
                               MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (visited == NULL || stack == NULL) {
        free(visited);
        free(stack);
        return ESP_ERR_NO_MEM;
    }

    int best_x = 0;
    int best_y = 0;
    int best_width = 0;
    int best_height = 0;
    int best_area = 0;
    int32_t best_score = INT32_MIN;

    for (int seed_y = 2; seed_y + 2 < height; ++seed_y) {
        for (int seed_x = 2; seed_x + 2 < width; ++seed_x) {
            const int seed = seed_y * width + seed_x;
            if (visited[seed] || !bt_is_orange(frame, (size_t)seed)) {
                continue;
            }
            visited[seed] = 1;
            size_t stack_size = 0;
            stack[stack_size++] = (uint32_t)seed;
            int minimum_x = seed_x;
            int maximum_x = seed_x;
            int minimum_y = seed_y;
            int maximum_y = seed_y;
            int area = 0;
            int sum_x = 0;
            int sum_y = 0;

            while (stack_size != 0) {
                const int index = (int)stack[--stack_size];
                const int x = index % width;
                const int y = index / width;
                ++area;
                sum_x += x;
                sum_y += y;
                minimum_x = bt_min(minimum_x, x);
                maximum_x = bt_max(maximum_x, x);
                minimum_y = bt_min(minimum_y, y);
                maximum_y = bt_max(maximum_y, y);

                for (int offset_y = -1; offset_y <= 1; ++offset_y) {
                    for (int offset_x = -1; offset_x <= 1; ++offset_x) {
                        if (offset_x == 0 && offset_y == 0) {
                            continue;
                        }
                        const int next_x = x + offset_x;
                        const int next_y = y + offset_y;
                        if (next_x < 2 || next_x + 2 >= width ||
                            next_y < 2 || next_y + 2 >= height) {
                            continue;
                        }
                        const int next = next_y * width + next_x;
                        if (!visited[next] &&
                            bt_is_orange(frame, (size_t)next)) {
                            visited[next] = 1;
                            stack[stack_size++] = (uint32_t)next;
                        }
                    }
                }
            }

            const int box_width = maximum_x - minimum_x + 1;
            const int box_height = maximum_y - minimum_y + 1;
            const int box_area = box_width * box_height;
            if (area < 18 || box_width < 5 || box_height < 5 ||
                box_width > 60 || box_height > 60 ||
                box_width * 2 < box_height || box_height * 2 < box_width ||
                area * 4 < box_area) {
                continue;
            }

            const int round_penalty = bt_abs(box_width - box_height);
            const int32_t score = area * 5 - round_penalty * 20;
            if (score > best_score) {
                best_score = score;
                best_x = sum_x / area;
                best_y = sum_y / area;
                best_width = box_width;
                best_height = box_height;
                best_area = area;
            }
        }
    }

    if (best_score != INT32_MIN) {
        result->found = true;
        result->ball.x = (uint16_t)((uint32_t)best_x *
                                    frame->source_width / width);
        result->ball.y = (uint16_t)((uint32_t)best_y *
                                    frame->source_height / height);
        result->ball.radius = (uint16_t)(
            (uint32_t)(best_width + best_height) * frame->source_width /
            (4U * width));
        result->ball.score = best_score;
        result->ball.coverage =
            (uint8_t)(best_area * 100 / (best_width * best_height));
    }

    free(visited);
    free(stack);
    return ESP_OK;
}

static void bt_export_target(const bt_target_candidate_t *candidate,
                             const bt_preprocessed_frame_t *frame,
                             bt_target_t *output)
{
    output->x = (uint16_t)((uint32_t)candidate->x *
                           frame->source_width / frame->width);
    output->y = (uint16_t)((uint32_t)candidate->y *
                           frame->source_height / frame->height);
    output->x0 = (uint16_t)((uint32_t)candidate->x0 *
                            frame->source_width / frame->width);
    output->y0 = (uint16_t)((uint32_t)candidate->y0 *
                            frame->source_height / frame->height);
    output->x1 = (uint16_t)((uint32_t)(candidate->x1 + 1) *
                            frame->source_width / frame->width);
    output->y1 = (uint16_t)((uint32_t)(candidate->y1 + 1) *
                            frame->source_height / frame->height);
    output->area = (uint32_t)((uint64_t)candidate->area *
        frame->source_width * frame->source_height /
        ((uint32_t)frame->width * frame->height));
    output->score = candidate->score;
    output->mean_luminance = (uint8_t)candidate->mean;
}

static esp_err_t bt_detect_black_targets(const bt_preprocessed_frame_t *frame,
                                         bt_raw_target_result_t *result)
{
    if (frame == NULL || frame->storage == NULL || result == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    memset(result, 0, sizeof(*result));

    const int width = frame->width;
    const int height = frame->height;
    const int roi_x0 = width * 3 / 100;
    const int roi_x1 = width * 97 / 100;
    const int roi_y0 = height * 50 / 100;
    const int roi_y1 = height * 94 / 100;
    const size_t pixel_count = (size_t)width * height;
    uint8_t *visited = bt_alloc(pixel_count, true,
                                MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    uint32_t *stack = bt_alloc(pixel_count * sizeof(*stack), false,
                               MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (visited == NULL || stack == NULL) {
        free(visited);
        free(stack);
        return ESP_ERR_NO_MEM;
    }

    bt_target_candidate_t best[BT_TARGET_COUNT] = {{0}, {0}};
    for (int seed_y = roi_y0; seed_y < roi_y1; ++seed_y) {
        for (int seed_x = roi_x0; seed_x < roi_x1; ++seed_x) {
            const int seed = seed_y * width + seed_x;
            if (visited[seed] || frame->gray[seed] > BT_DARK_THRESHOLD) {
                continue;
            }

            visited[seed] = 1;
            size_t stack_size = 0;
            stack[stack_size++] = (uint32_t)seed;
            int minimum_x = seed_x;
            int maximum_x = seed_x;
            int minimum_y = seed_y;
            int maximum_y = seed_y;
            int area = 0;
            int sum_x = 0;
            int sum_y = 0;
            int sum_gray = 0;
            bool touches_bottom = false;

            while (stack_size != 0) {
                const int index = (int)stack[--stack_size];
                const int x = index % width;
                const int y = index / width;
                ++area;
                sum_x += x;
                sum_y += y;
                sum_gray += frame->gray[index];
                minimum_x = bt_min(minimum_x, x);
                maximum_x = bt_max(maximum_x, x);
                minimum_y = bt_min(minimum_y, y);
                maximum_y = bt_max(maximum_y, y);
                if (y + 1 >= roi_y1) {
                    touches_bottom = true;
                }

                for (int offset_y = -1; offset_y <= 1; ++offset_y) {
                    for (int offset_x = -1; offset_x <= 1; ++offset_x) {
                        if (offset_x == 0 && offset_y == 0) {
                            continue;
                        }
                        const int next_x = x + offset_x;
                        const int next_y = y + offset_y;
                        if (next_x < roi_x0 || next_x >= roi_x1 ||
                            next_y < roi_y0 || next_y >= roi_y1) {
                            continue;
                        }
                        const int next = next_y * width + next_x;
                        if (!visited[next] &&
                            frame->gray[next] <= BT_DARK_THRESHOLD) {
                            visited[next] = 1;
                            stack[stack_size++] = (uint32_t)next;
                        }
                    }
                }
            }

            const int box_width = maximum_x - minimum_x + 1;
            const int box_height = maximum_y - minimum_y + 1;
            const int box_area = box_width * box_height;
            if (area < 8 || box_width < 5 || box_height < 2 ||
                touches_bottom || box_width > BT_TARGET_MAX_BOX_WIDTH ||
                box_height > BT_TARGET_MAX_BOX_HEIGHT ||
                box_width > box_height * 9 || box_height > box_width * 2 ||
                area * 5 < box_area) {
                continue;
            }

            const int center_x = sum_x / area;
            const int center_y = sum_y / area;
            const int mean = sum_gray / area;
            const int side = center_x < width / 2 ? 0 : 1;
            const int32_t score = area * 3 +
                (BT_DARK_THRESHOLD - mean) * 3 + center_y;
            if (!best[side].valid || score > best[side].score) {
                best[side] = (bt_target_candidate_t){
                    .valid = true,
                    .x0 = minimum_x,
                    .x1 = maximum_x,
                    .y0 = minimum_y,
                    .y1 = maximum_y,
                    .x = center_x,
                    .y = center_y,
                    .area = area,
                    .mean = mean,
                    .score = score,
                };
            }
        }
    }

    for (int side = 0; side < BT_TARGET_COUNT; ++side) {
        if (best[side].valid) {
            bt_export_target(&best[side], frame,
                             &result->targets[result->count++]);
        }
    }
    if (result->count == BT_TARGET_COUNT) {
        result->found = true;
        result->target = result->targets[0];
    }

    free(visited);
    free(stack);
    return ESP_OK;
}

static bool bt_update_ball_tracker(bt_detector_t *detector,
                                   const bt_raw_ball_result_t *result,
                                   bt_ball_t *stable_ball)
{
    if (result == NULL || !result->found) {
        detector->ball_consecutive_frames = 0;
        return false;
    }

    const bt_ball_t detected = result->ball;
    bool near = false;
    if (detector->ball_has_candidate) {
        const int dx = (int)detected.x - detector->ball_candidate.x;
        const int dy = (int)detected.y - detector->ball_candidate.y;
        near = dx * dx + dy * dy <=
            BT_BALL_MAX_CENTER_JUMP * BT_BALL_MAX_CENTER_JUMP;
    }
    if (!near) {
        detector->ball_candidate = detected;
        detector->ball_has_candidate = true;
        detector->ball_consecutive_frames = 1;
        return false;
    }

    if (detector->ball_consecutive_frames < BT_BALL_REQUIRED_FRAMES) {
        ++detector->ball_consecutive_frames;
    }
    detector->ball_candidate.x = (uint16_t)(
        (3U * detector->ball_candidate.x + detected.x) / 4U);
    detector->ball_candidate.y = (uint16_t)(
        (3U * detector->ball_candidate.y + detected.y) / 4U);
    detector->ball_candidate.radius = (uint16_t)(
        (3U * detector->ball_candidate.radius + detected.radius) / 4U);
    detector->ball_candidate.score = detected.score;
    detector->ball_candidate.coverage = detected.coverage;
    if (detector->ball_consecutive_frames < BT_BALL_REQUIRED_FRAMES) {
        return false;
    }
    if (stable_ball != NULL) {
        *stable_ball = detector->ball_candidate;
    }
    return true;
}

static bool bt_update_orange_tracker(bt_detector_t *detector,
                                     const bt_raw_ball_result_t *result,
                                     bt_ball_t *stable_ball)
{
    if (result == NULL || !result->found) {
        detector->orange_consecutive_frames = 0;
        return false;
    }

    const bt_ball_t detected = result->ball;
    bool near = false;
    if (detector->orange_has_candidate) {
        const int dx = (int)detected.x - detector->orange_candidate.x;
        const int dy = (int)detected.y - detector->orange_candidate.y;
        near = dx * dx + dy * dy <=
            BT_ORANGE_MAX_CENTER_JUMP * BT_ORANGE_MAX_CENTER_JUMP;
    }
    if (!near) {
        detector->orange_candidate = detected;
        detector->orange_has_candidate = true;
        detector->orange_consecutive_frames = 1;
        return false;
    }

    if (detector->orange_consecutive_frames < BT_ORANGE_REQUIRED_FRAMES) {
        ++detector->orange_consecutive_frames;
    }
    detector->orange_candidate.x = (uint16_t)(
        (3U * detector->orange_candidate.x + detected.x) / 4U);
    detector->orange_candidate.y = (uint16_t)(
        (3U * detector->orange_candidate.y + detected.y) / 4U);
    detector->orange_candidate.radius = (uint16_t)(
        (3U * detector->orange_candidate.radius + detected.radius) / 4U);
    detector->orange_candidate.score = detected.score;
    detector->orange_candidate.coverage = detected.coverage;
    if (detector->orange_consecutive_frames < BT_ORANGE_REQUIRED_FRAMES) {
        return false;
    }
    if (stable_ball != NULL) {
        *stable_ball = detector->orange_candidate;
    }
    return true;
}

static bool bt_update_target_tracker(bt_detector_t *detector,
                                     const bt_raw_target_result_t *result,
                                     bt_target_t *stable_target)
{
    if (result == NULL || !result->found) {
        detector->target_consecutive_frames = 0;
        return false;
    }

    const bt_target_t detected = result->target;
    bool near = false;
    if (detector->target_has_candidate) {
        const int dx = (int)detected.x - detector->target_candidate.x;
        const int dy = (int)detected.y - detector->target_candidate.y;
        near = dx * dx + dy * dy <=
            BT_TARGET_MAX_CENTER_JUMP * BT_TARGET_MAX_CENTER_JUMP;
    }
    if (!near) {
        detector->target_candidate = detected;
        detector->target_has_candidate = true;
        detector->target_consecutive_frames = 1;
        return false;
    }

    if (detector->target_consecutive_frames < BT_TARGET_REQUIRED_FRAMES) {
        ++detector->target_consecutive_frames;
    }
#define BT_SMOOTH_TARGET(FIELD) detector->target_candidate.FIELD = \
    (uint16_t)((3U * detector->target_candidate.FIELD + detected.FIELD) / 4U)
    BT_SMOOTH_TARGET(x);
    BT_SMOOTH_TARGET(y);
    BT_SMOOTH_TARGET(x0);
    BT_SMOOTH_TARGET(y0);
    BT_SMOOTH_TARGET(x1);
    BT_SMOOTH_TARGET(y1);
#undef BT_SMOOTH_TARGET
    detector->target_candidate.area = detected.area;
    detector->target_candidate.score = detected.score;
    detector->target_candidate.mean_luminance = detected.mean_luminance;
    if (detector->target_consecutive_frames < BT_TARGET_REQUIRED_FRAMES) {
        return false;
    }
    if (stable_target != NULL) {
        *stable_target = detector->target_candidate;
    }
    return true;
}

void bt_detector_init(bt_detector_t *detector)
{
    if (detector != NULL) {
        memset(detector, 0, sizeof(*detector));
    }
}

esp_err_t bt_detector_process_rgb565(bt_detector_t *detector,
                                     const uint16_t *pixels,
                                     uint16_t width,
                                     uint16_t height,
                                     bt_detector_result_t *result)
{
    if (detector == NULL || pixels == NULL || result == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    memset(result, 0, sizeof(*result));
    result->frame_width = width;
    result->frame_height = height;

    bt_preprocessed_frame_t frame = {0};
    esp_err_t error = bt_preprocess_rgb565(pixels, width, height, &frame);
    if (error != ESP_OK) {
        return error;
    }

    bt_raw_ball_result_t raw_ball = {0};
    error = bt_detect_white_ball(
        &frame, detector->have_ball_hint ? &detector->ball_hint : NULL,
        &raw_ball);
    if (error != ESP_OK) {
        bt_preprocess_release(&frame);
        return error;
    }

    if (raw_ball.found) {
        detector->ball_misses = 0;
        if (bt_update_ball_tracker(detector, &raw_ball,
                                   &detector->stable_ball)) {
            detector->ball_valid = true;
            detector->ball_hint = detector->stable_ball;
            detector->have_ball_hint = true;
        } else if (!detector->ball_valid) {
            detector->have_ball_hint = false;
        }
    } else {
        bt_update_ball_tracker(detector, &raw_ball, NULL);
        if (detector->ball_misses < UINT8_MAX) {
            ++detector->ball_misses;
        }
        if (detector->ball_misses >= 2) {
            detector->ball_valid = false;
            detector->have_ball_hint = false;
            memset(&detector->ball_hint, 0, sizeof(detector->ball_hint));
        }
    }

    bt_raw_ball_result_t raw_orange = {0};
    bt_raw_target_result_t raw_targets = {0};
    if (detector->ball_valid) {
        error = bt_detect_black_targets(&frame, &raw_targets);
        if (error != ESP_OK) {
            bt_preprocess_release(&frame);
            return error;
        }
        error = bt_detect_orange_ball(&frame, &raw_orange);
        if (error != ESP_OK) {
            bt_preprocess_release(&frame);
            return error;
        }

        if (raw_orange.found) {
            detector->orange_misses = 0;
            if (bt_update_orange_tracker(detector, &raw_orange,
                                         &detector->stable_orange)) {
                detector->orange_valid = true;
            }
        } else {
            bt_update_orange_tracker(detector, &raw_orange, NULL);
            if (detector->orange_misses < UINT8_MAX) {
                ++detector->orange_misses;
            }
            if (detector->orange_misses >= 2) {
                detector->orange_valid = false;
            }
        }

        if (raw_targets.found) {
            detector->target_misses = 0;
            if (bt_update_target_tracker(detector, &raw_targets,
                                         &detector->stable_target)) {
                detector->target_valid = true;
            }
        } else {
            bt_update_target_tracker(detector, &raw_targets, NULL);
            if (detector->target_misses < UINT8_MAX) {
                ++detector->target_misses;
            }
            if (detector->target_misses >= 2) {
                detector->target_valid = false;
            }
        }
    } else {
        bt_update_orange_tracker(detector, &raw_orange, NULL);
        bt_update_target_tracker(detector, &raw_targets, NULL);
        detector->orange_valid = false;
        detector->target_valid = false;
        detector->orange_misses = 0;
        detector->target_misses = 0;
    }

    result->ball_valid = detector->ball_valid;
    result->orange_ball_valid = detector->orange_valid;
    result->target_valid = detector->target_valid;
    result->ball = detector->stable_ball;
    result->orange_ball = detector->stable_orange;
    result->target = detector->stable_target;
    result->target_count = raw_targets.count;
    result->targets[0] = raw_targets.targets[0];
    result->targets[1] = raw_targets.targets[1];

    bt_preprocess_release(&frame);
    return ESP_OK;
}

static uint16_t bt_rgb565_to_wire(uint16_t rgb565)
{
    return (uint16_t)((rgb565 << 8) | (rgb565 >> 8));
}

static void bt_set_wire_pixel(uint16_t *pixels, int width, int height,
                              int x, int y, uint16_t rgb565)
{
    if (x >= 0 && x < width && y >= 0 && y < height) {
        pixels[(size_t)y * width + x] = bt_rgb565_to_wire(rgb565);
    }
}

static void bt_draw_line(uint16_t *pixels, int width, int height,
                         int x0, int y0, int x1, int y1, uint16_t color)
{
    const int delta_x = bt_abs(x1 - x0);
    const int step_x = x0 < x1 ? 1 : -1;
    const int delta_y = -bt_abs(y1 - y0);
    const int step_y = y0 < y1 ? 1 : -1;
    int error = delta_x + delta_y;
    for (;;) {
        bt_set_wire_pixel(pixels, width, height, x0, y0, color);
        if (x0 == x1 && y0 == y1) {
            break;
        }
        const int twice_error = 2 * error;
        if (twice_error >= delta_y) {
            error += delta_y;
            x0 += step_x;
        }
        if (twice_error <= delta_x) {
            error += delta_x;
            y0 += step_y;
        }
    }
}

static void bt_draw_ball(uint16_t *pixels, int width, int height,
                         const bt_ball_t *ball, uint16_t outline,
                         uint16_t center)
{
    const int radius = ball->radius;
    for (int thickness = -1; thickness <= 1; ++thickness) {
        const int draw_radius = radius + thickness;
        if (draw_radius < 1) {
            continue;
        }
        for (int degree = 0; degree < 360; degree += 4) {
            const float angle = degree * 0.0174532925f;
            const int x = (int)lroundf(ball->x + draw_radius * cosf(angle));
            const int y = (int)lroundf(ball->y + draw_radius * sinf(angle));
            bt_set_wire_pixel(pixels, width, height, x, y, outline);
        }
    }
    for (int y = -2; y <= 2; ++y) {
        for (int x = -2; x <= 2; ++x) {
            bt_set_wire_pixel(pixels, width, height,
                              (int)ball->x + x, (int)ball->y + y, center);
        }
    }
}

static void bt_draw_target(uint16_t *pixels, int width, int height,
                           const bt_target_t *target)
{
    for (int thickness = 0; thickness < 2; ++thickness) {
        const int x0 = (int)target->x0 - thickness;
        const int y0 = (int)target->y0 - thickness;
        const int x1 = (int)target->x1 + thickness;
        const int y1 = (int)target->y1 + thickness;
        bt_draw_line(pixels, width, height, x0, y0, x1, y0, 0x07FF);
        bt_draw_line(pixels, width, height, x1, y0, x1, y1, 0x07FF);
        bt_draw_line(pixels, width, height, x1, y1, x0, y1, 0x07FF);
        bt_draw_line(pixels, width, height, x0, y1, x0, y0, 0x07FF);
    }
}

static bt_ball_t bt_scale_ball(const bt_ball_t *source,
                               uint16_t source_width,
                               uint16_t source_height,
                               uint16_t width,
                               uint16_t height)
{
    bt_ball_t output = *source;
    output.x = (uint16_t)((uint32_t)source->x * width / source_width);
    output.y = (uint16_t)((uint32_t)source->y * height / source_height);
    output.radius = (uint16_t)((uint32_t)source->radius * width / source_width);
    if (output.radius < 2) {
        output.radius = 2;
    }
    return output;
}

static bt_target_t bt_scale_target(const bt_target_t *source,
                                   uint16_t source_width,
                                   uint16_t source_height,
                                   uint16_t width,
                                   uint16_t height)
{
    bt_target_t output = *source;
    output.x = (uint16_t)((uint32_t)source->x * width / source_width);
    output.y = (uint16_t)((uint32_t)source->y * height / source_height);
    output.x0 = (uint16_t)((uint32_t)source->x0 * width / source_width);
    output.x1 = (uint16_t)((uint32_t)source->x1 * width / source_width);
    output.y0 = (uint16_t)((uint32_t)source->y0 * height / source_height);
    output.y1 = (uint16_t)((uint32_t)source->y1 * height / source_height);
    return output;
}

esp_err_t bt_visualize_rgb565(uint16_t *pixels,
                              uint16_t width,
                              uint16_t height,
                              const bt_detector_result_t *result)
{
    if (pixels == NULL || result == NULL || width == 0 || height == 0 ||
        result->frame_width == 0 || result->frame_height == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    const int reference_x = width * 43 / 100;
    if (result->target_valid) {
        const bt_target_t target = bt_scale_target(
            &result->target, result->frame_width, result->frame_height,
            width, height);
        bt_draw_line(pixels, width, height, reference_x, 0,
                     target.x, target.y, 0x001F);
    }
    for (uint8_t index = 0;
         index < result->target_count && index < BT_TARGET_COUNT; ++index) {
        const bt_target_t target = bt_scale_target(
            &result->targets[index], result->frame_width, result->frame_height,
            width, height);
        bt_draw_target(pixels, width, height, &target);
    }
    if (result->ball_valid) {
        const bt_ball_t ball = bt_scale_ball(
            &result->ball, result->frame_width, result->frame_height,
            width, height);
        bt_draw_ball(pixels, width, height, &ball, 0x07E0, 0xF800);
    }
    if (result->orange_ball_valid) {
        const bt_ball_t ball = bt_scale_ball(
            &result->orange_ball, result->frame_width, result->frame_height,
            width, height);
        bt_draw_ball(pixels, width, height, &ball, 0xFD20, 0xFFE0);
    }
    for (int offset = -4; offset <= 4; ++offset) {
        bt_set_wire_pixel(pixels, width, height,
                          reference_x + offset, 2, 0xF81F);
        bt_set_wire_pixel(pixels, width, height,
                          reference_x, 2 + offset, 0xF81F);
    }
    return ESP_OK;
}

static bool bt_json_append(char **cursor, size_t *remaining,
                           const char *format, ...)
{
    va_list arguments;
    va_start(arguments, format);
    const int length = vsnprintf(*cursor, *remaining, format, arguments);
    va_end(arguments);
    if (length < 0 || (size_t)length >= *remaining) {
        if (*remaining != 0) {
            (*cursor)[*remaining - 1] = '\0';
        }
        return false;
    }
    *cursor += length;
    *remaining -= (size_t)length;
    return true;
}

esp_err_t bt_detector_result_to_json(const bt_detector_result_t *result,
                                     char *json,
                                     size_t capacity)
{
    if (result == NULL || json == NULL || capacity == 0) {
        return ESP_ERR_INVALID_ARG;
    }
    char *cursor = json;
    size_t remaining = capacity;
#define BT_JSON(...) do { \
    if (!bt_json_append(&cursor, &remaining, __VA_ARGS__)) \
        return ESP_ERR_INVALID_SIZE; \
} while (0)
    BT_JSON("{\"width\":%u,\"height\":%u,\"ball\":",
            result->frame_width, result->frame_height);
    if (result->ball_valid) {
        BT_JSON("{\"x\":%u,\"y\":%u,\"radius\":%u,\"score\":%ld,\"coverage\":%u}",
                result->ball.x, result->ball.y, result->ball.radius,
                (long)result->ball.score, result->ball.coverage);
    } else {
        BT_JSON("null");
    }
    BT_JSON(",\"orange\":");
    if (result->orange_ball_valid) {
        BT_JSON("{\"x\":%u,\"y\":%u,\"radius\":%u,\"score\":%ld,\"coverage\":%u}",
                result->orange_ball.x, result->orange_ball.y,
                result->orange_ball.radius, (long)result->orange_ball.score,
                result->orange_ball.coverage);
    } else {
        BT_JSON("null");
    }
    BT_JSON(",\"target\":");
    if (result->target_valid) {
        BT_JSON("{\"x\":%u,\"y\":%u,\"x0\":%u,\"y0\":%u,"
                "\"x1\":%u,\"y1\":%u,\"area\":%lu,\"score\":%ld,\"mean\":%u}",
                result->target.x, result->target.y,
                result->target.x0, result->target.y0,
                result->target.x1, result->target.y1,
                (unsigned long)result->target.area,
                (long)result->target.score, result->target.mean_luminance);
    } else {
        BT_JSON("null");
    }
    BT_JSON(",\"targets\":[");
    for (uint8_t index = 0;
         index < result->target_count && index < BT_TARGET_COUNT; ++index) {
        const bt_target_t *target = &result->targets[index];
        BT_JSON("%s{\"x\":%u,\"y\":%u,\"x0\":%u,\"y0\":%u,"
                "\"x1\":%u,\"y1\":%u}",
                index ? "," : "", target->x, target->y,
                target->x0, target->y0, target->x1, target->y1);
    }
    BT_JSON("]}");
#undef BT_JSON
    return ESP_OK;
}

const char *bt_detector_visualization_html(void)
{
    return s_visualization_html;
}
