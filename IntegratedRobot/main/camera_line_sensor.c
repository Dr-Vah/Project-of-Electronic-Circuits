#include "camera_line_sensor.h"

#include <assert.h>
#include <inttypes.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>

#include "esp_check.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "jpeg_decoder.h"
#include "usb/usb_host.h"
#include "usb/uvc_host.h"

#include "black_target_detector.h"
#include "ball_transport_controller.h"
#include "ball_target_detector.h"
#include "tft_display.h"

/* TFT is reserved for the wheel-speed/distance monitor in both missions. */
#define CAMERA_TFT_PREVIEW_ENABLED 0
#include "white_ball_detector.h"

#define TAG "camera_line"
#define CAMERA_TASK_PRIORITY 12
#define DISPLAY_TASK_PRIORITY 5
#define FRAME_BUFFER_COUNT 3
#define MIN_MJPEG_BUFFER_SIZE (40U * 1024U)
#define JPEG_WORK_BUFFER_SIZE 4096U
#define CAMERA_DECODE_MAX_WIDTH 320U
#define CAMERA_DECODE_MAX_HEIGHT 240U

/* Keep these equal to the calibrated Camera_Display values. */
#define LINE_BINARY_THRESHOLD 125U
#define LINE_ROI_TOP_PERCENT 0U
#define LINE_ROI_BOTTOM_PERCENT 8U
#define LINE_ROI_WIDTH_PERCENT 30U
#define LINE_ROI_CENTER_OFFSET_PERCENT (-7)
#define LINE_ACTIVE_DARK_PERCENT 20U
#define LINE_NEAR_ROW_WEIGHT 3U
#define LINE_FAR_ROW_WEIGHT 1U
#define LINE_BOX_THICKNESS 2U
#define CAMERA_DISPLAY_PERIOD_US 100000LL

_Static_assert(LINE_ROI_TOP_PERCENT < LINE_ROI_BOTTOM_PERCENT,
               "ROI top must be above ROI bottom");
_Static_assert(LINE_ROI_BOTTOM_PERCENT <= 100U,
               "ROI bottom must be <= 100 percent");
_Static_assert(LINE_ROI_WIDTH_PERCENT >= CAMERA_LINE_CHANNEL_COUNT &&
               LINE_ROI_WIDTH_PERCENT <= 100U,
               "ROI width must be 4..100 percent");

static QueueHandle_t s_frame_queue;
static uvc_host_stream_hdl_t s_stream;
static uvc_host_stream_config_t s_stream_config;
static volatile bool s_device_connected;
static bool s_stream_task_started;
static portMUX_TYPE s_state_lock = portMUX_INITIALIZER_UNLOCKED;
static camera_line_state_t s_state;
static int64_t s_last_display_us = 0;
static volatile bool s_ball_mode = false;

static void rotate_frame_180(uint16_t *pixels, size_t width, size_t height)
{
    size_t front = 0U;
    size_t back = width * height - 1U;
    while (front < back) {
        const uint16_t temporary = pixels[front];
        pixels[front++] = pixels[back];
        pixels[back--] = temporary;
    }
}

static void rotate_ball_result_180(white_ball_result_t *ball,
                                   size_t width, size_t height)
{
    if (ball == NULL || !ball->valid) return;
    const uint16_t old_left = ball->left;
    const uint16_t old_top = ball->top;
    ball->left = (uint16_t)(width - 1U - ball->right);
    ball->right = (uint16_t)(width - 1U - old_left);
    ball->top = (uint16_t)(height - 1U - ball->bottom);
    ball->bottom = (uint16_t)(height - 1U - old_top);
    ball->center_x = -ball->center_x;
    ball->center_y = 1.0f - ball->center_y;
}

/* Keep the controller/display in its original 180-degree-rotated frame.
 * BallDet v2 runs on the unrotated camera image, including its left target. */
static white_ball_result_t adapt_bt_ball(const bt_ball_t *ball, bool valid,
                                        uint16_t width, uint16_t height,
                                        float diameter_scale)
{
    white_ball_result_t result = {0};
    if (!valid) return result;
    const uint32_t area = (314U * ball->radius * ball->radius + 50U) / 100U;
    result.valid = true;
    result.confirmation_count = 1;
    result.area_px = area > UINT16_MAX ? UINT16_MAX : (uint16_t)area;
    result.left = ball->x > ball->radius ? ball->x - ball->radius : 0;
    result.top = ball->y > ball->radius ? ball->y - ball->radius : 0;
    result.right = (uint32_t)ball->x + ball->radius < width
        ? ball->x + ball->radius : width - 1U;
    result.bottom = (uint32_t)ball->y + ball->radius < height
        ? ball->y + ball->radius : height - 1U;
    result.center_x = 2.0f * ball->x / (width - 1U) - 1.0f;
    result.center_y = (float)ball->y / (height - 1U);
    /* Preserve the controller's existing pixel-diameter capture thresholds. */
    result.diameter_px = 2.0f * ball->radius * diameter_scale;
    rotate_ball_result_180(&result, width, height);
    return result;
}

static black_target_result_t adapt_bt_target(const bt_target_t *target,
                                             bool valid, uint16_t width,
                                             uint16_t height)
{
    black_target_result_t result = {0};
    if (!valid) return result;
    result.valid = true;
    result.confirmation_count = 1;
    result.area_px = target->area > UINT16_MAX
        ? UINT16_MAX : (uint16_t)target->area;
    /* BallDet v2's x1/y1 are exclusive; controller bounds are inclusive. */
    result.left = width - target->x1;
    result.right = width - 1U - target->x0;
    result.top = height - target->y1;
    result.bottom = height - 1U - target->y0;
    result.center_x = 1.0f - 2.0f * target->x / (width - 1U);
    result.center_y = 1.0f - (float)target->y / (height - 1U);
    result.area_fraction = (float)target->area / ((uint32_t)width * height);
    return result;
}

static float interval_to_fps(uint32_t interval)
{
    return interval ? 10000000.0f / (float)interval : 0.0f;
}

static uint32_t slowest_interval(const uvc_host_frame_info_t *info)
{
    /* Prefer the slowest (longest) frame interval.  A lower frame rate reduces
     * USB bandwidth and per-frame JPEG decode load, avoiding the intermittent
     * stream stalls that freeze the debug image. */
    uint32_t selected = info->default_interval;
    if (info->interval_type == 0) return info->interval_max > selected ? info->interval_max : selected;
    size_t count = info->interval_type;
    if (count > CONFIG_UVC_INTERVAL_ARRAY_SIZE) count = CONFIG_UVC_INTERVAL_ARRAY_SIZE;
    for (size_t i = 0; i < count; ++i) if (info->interval[i] > selected) selected = info->interval[i];
    return selected;
}

static esp_jpeg_image_scale_t choose_legacy_decode_scale(
    const uvc_host_stream_format_t *format)
{
    if ((format->h_res + 1U) / 2U <= WHITE_BALL_MAX_WIDTH &&
        (format->v_res + 1U) / 2U <= WHITE_BALL_MAX_HEIGHT) {
        return JPEG_IMAGE_SCALE_1_2;
    }
    if ((format->h_res + 3U) / 4U <= WHITE_BALL_MAX_WIDTH &&
        (format->v_res + 3U) / 4U <= WHITE_BALL_MAX_HEIGHT) {
        return JPEG_IMAGE_SCALE_1_4;
    }
    return JPEG_IMAGE_SCALE_1_8;
}

static esp_jpeg_image_scale_t choose_decode_scale(
    const uvc_host_stream_format_t *format)
{
    if (format->h_res <= CAMERA_DECODE_MAX_WIDTH &&
        format->v_res <= CAMERA_DECODE_MAX_HEIGHT) return JPEG_IMAGE_SCALE_0;
    if ((format->h_res + 1U) / 2U <= CAMERA_DECODE_MAX_WIDTH &&
        (format->v_res + 1U) / 2U <= CAMERA_DECODE_MAX_HEIGHT) {
        return JPEG_IMAGE_SCALE_1_2;
    }
    if ((format->h_res + 3U) / 4U <= CAMERA_DECODE_MAX_WIDTH &&
        (format->v_res + 3U) / 4U <= CAMERA_DECODE_MAX_HEIGHT) {
        return JPEG_IMAGE_SCALE_1_4;
    }
    return JPEG_IMAGE_SCALE_1_8;
}

static uint16_t rgb565_to_wire(uint16_t rgb565)
{
    return (uint16_t)((rgb565 << 8) | (rgb565 >> 8));
}

static void set_wire_pixel(uint16_t *pixels, uint16_t width, uint16_t height,
                           unsigned x, unsigned y, uint16_t rgb565)
{
    if (x < width && y < height) pixels[(size_t)y * width + x] = rgb565_to_wire(rgb565);
}

static void draw_detection_box(uint16_t *pixels, uint16_t width,
                               uint16_t height, uint16_t left, uint16_t top,
                               uint16_t right, uint16_t bottom,
                               uint16_t colour)
{
    if (left >= width || top >= height) return;
    if (right >= width) right = width - 1U;
    if (bottom >= height) bottom = height - 1U;
    for (uint16_t x = left; x <= right; ++x) {
        set_wire_pixel(pixels, width, height, x, top, colour);
        set_wire_pixel(pixels, width, height, x, bottom, colour);
    }
    for (uint16_t y = top; y <= bottom; ++y) {
        set_wire_pixel(pixels, width, height, left, y, colour);
        set_wire_pixel(pixels, width, height, right, y, colour);
    }
}

static void analyse_rgb565(uint16_t *pixels, uint16_t width, uint16_t height)
{
    if (width < CAMERA_LINE_CHANNEL_COUNT || height == 0) return;
    const unsigned y0 = (unsigned)height * LINE_ROI_TOP_PERCENT / 100U;
    const unsigned y1 = (unsigned)height * LINE_ROI_BOTTOM_PERCENT / 100U;
    const unsigned roi_width = (unsigned)width * LINE_ROI_WIDTH_PERCENT / 100U;
    if (y1 <= y0 || roi_width == 0) return;
    int x0_signed = ((int)width - (int)roi_width) / 2 +
        (int)width * LINE_ROI_CENTER_OFFSET_PERCENT / 100;
    if (x0_signed < 0) x0_signed = 0;
    if ((unsigned)x0_signed + roi_width > width) x0_signed = (int)width - (int)roi_width;
    const unsigned x0 = (unsigned)x0_signed;
    const unsigned x1 = x0 + roi_width;
    uint32_t dark[CAMERA_LINE_CHANNEL_COUNT] = {0};
    uint32_t total[CAMERA_LINE_CHANNEL_COUNT] = {0};
    uint32_t x_sum[CAMERA_LINE_CHANNEL_COUNT] = {0};

    /* Centroid of the dark line is measured over a trimmed ROI that excludes
     * both sides, so a line appearing at the edge while turning does not pull
     * the centroid away from the lane centre. */
    const unsigned exclude = roi_width * 20U / 100U;
    const unsigned cx0 = x0 + exclude;
    const unsigned cx1 = x1 - exclude;
    uint32_t line_dark = 0;
    uint32_t line_x_sum = 0;

    for (unsigned y = 0; y < height; ++y) {
        unsigned row_weight = LINE_FAR_ROW_WEIGHT;
        if (y >= y0 && y < y1) {
            /* The raw patrol image is upside down, so its top rows are
             * closest to the chassis.  Weight them more heavily while
             * retaining the full calibrated ROI for noise tolerance. */
            const unsigned roi_rows = y1 - y0;
            const unsigned row_offset = y - y0;
            row_weight = roi_rows <= 1U
                ? LINE_NEAR_ROW_WEIGHT
                : LINE_NEAR_ROW_WEIGHT -
                      (LINE_NEAR_ROW_WEIGHT - LINE_FAR_ROW_WEIGHT) *
                          row_offset / (roi_rows - 1U);
        }
        for (unsigned x = 0; x < width; ++x) {
            const uint16_t wire = pixels[(size_t)y * width + x];
            const uint16_t rgb = (uint16_t)((wire << 8) | (wire >> 8));
            const unsigned red = (rgb >> 11) & 31U;
            const unsigned green = (rgb >> 5) & 63U;
            const unsigned blue = rgb & 31U;
            const unsigned luminance = (77U * red * 255U / 31U +
                150U * green * 255U / 63U + 29U * blue * 255U / 31U) >> 8;
            const bool is_dark = luminance <= LINE_BINARY_THRESHOLD;
            pixels[(size_t)y * width + x] = is_dark ? rgb565_to_wire(0x0000) :
                                                       rgb565_to_wire(0xFFFF);
            if (x >= x0 && x < x1 && y >= y0 && y < y1) {
                unsigned channel = (x - x0) * CAMERA_LINE_CHANNEL_COUNT / roi_width;
                if (channel >= CAMERA_LINE_CHANNEL_COUNT) channel = CAMERA_LINE_CHANNEL_COUNT - 1U;
                total[channel] += row_weight;
                if (is_dark) {
                    dark[channel] += row_weight;
                    x_sum[channel] += x * row_weight;
                }
            }
            if (x >= cx0 && x < cx1 && y >= y0 && y < y1 && is_dark) {
                line_dark += row_weight;
                line_x_sum += x * row_weight;
            }
        }
    }

    camera_line_state_t next = {.valid = true};
    for (unsigned i = 0; i < CAMERA_LINE_CHANNEL_COUNT; ++i) {
        next.dark_percent[i] = total[i] ? (uint8_t)(dark[i] * 100U / total[i]) : 0;
        next.black[i] = total[i] && dark[i] * 100U >= total[i] * LINE_ACTIVE_DARK_PERCENT;
        next.centroid_x[i] = dark[i] ? (uint16_t)(x_sum[i] / dark[i]) : 0;
    }

    next.line_pixels = line_dark;
    if (line_dark != 0) {
        const float centroid = (float)line_x_sum / (float)line_dark;
        const float centre = ((float)cx0 + (float)cx1) / 2.0f;
        const float half_width = ((float)cx1 - (float)cx0) / 2.0f;
        /* +1 when the line is image-right, -1 when image-left. */
        next.line_error = (centroid - centre) / half_width;
    } else {
        next.line_error = 0.0f;
    }

    portENTER_CRITICAL(&s_state_lock);
    next.sequence = s_state.sequence + 1U;
    next.timestamp_us = esp_timer_get_time();
    s_state = next;
    portEXIT_CRITICAL(&s_state_lock);

    static const uint16_t box_colours[CAMERA_LINE_CHANNEL_COUNT] = {
        0x07FF, 0xFFE0, 0xF81F, 0x07E0,
    };
    for (unsigned channel = 0; channel < CAMERA_LINE_CHANNEL_COUNT; ++channel) {
        const unsigned box_x0 = x0 + channel * roi_width / CAMERA_LINE_CHANNEL_COUNT;
        const unsigned box_x1 = x0 + (channel + 1U) * roi_width /
            CAMERA_LINE_CHANNEL_COUNT - 1U;
        const unsigned channel_width = box_x1 - box_x0 + 1U;
        unsigned thickness = channel_width / 4U;
        if (thickness == 0) thickness = 1U;
        if (thickness > LINE_BOX_THICKNESS) thickness = LINE_BOX_THICKNESS;
        for (unsigned edge = 0; edge < thickness; ++edge) {
            for (unsigned x = box_x0; x <= box_x1; ++x) {
                set_wire_pixel(pixels, width, height, x, y0 + edge,
                               box_colours[channel]);
                set_wire_pixel(pixels, width, height, x, y1 - 1U - edge,
                               box_colours[channel]);
            }
            for (unsigned y = y0; y < y1; ++y) {
                set_wire_pixel(pixels, width, height, box_x0 + edge, y,
                               box_colours[channel]);
                set_wire_pixel(pixels, width, height, box_x1 - edge, y,
                               box_colours[channel]);
            }
        }
    }
    /* Actual centroid ROI: trim 20% off each side of the four-channel strip.
     * Cyan bounds, magenta reference axis, red measured line centroid. */
    draw_detection_box(pixels, width, height, cx0, y0, cx1 - 1U,
                       y1 - 1U, 0x07FF);
    const unsigned axis_x = (cx0 + cx1) / 2U;
    for (unsigned y = y0; y < height / 3U; ++y) {
        set_wire_pixel(pixels, width, height, axis_x, y, 0xF81F);
    }
    if (line_dark != 0) {
        const unsigned measured_x = line_x_sum / line_dark;
        for (unsigned y = y0; y < height / 5U; ++y) {
            set_wire_pixel(pixels, width, height, measured_x, y, 0xF800);
        }
    }
    /* Throttle the debug display so the vision pipeline is not blocked on the
     * slow SPI framebuffer write every frame.  The display stays at a small,
     * fixed refresh rate while decoding and state updates run at full speed. */
    const int64_t now_us = esp_timer_get_time();
    if (now_us - s_last_display_us >= CAMERA_DISPLAY_PERIOD_US) {
        s_last_display_us = now_us;
        if (CAMERA_TFT_PREVIEW_ENABLED) {
            ESP_ERROR_CHECK_WITHOUT_ABORT(tft_display_show_camera_debug(
            pixels, width, height, next.dark_percent, next.black));
        }
    }
}

static bool frame_callback(const uvc_host_frame_t *frame, void *user_ctx)
{
    (void)user_ctx;
    if (frame->vs_format.format != UVC_VS_FORMAT_MJPEG) return true;
    const uvc_host_frame_t *queued = frame;
    return xQueueSendToBack(s_frame_queue, &queued, 0) == pdPASS ? false : true;
}

static void frame_task(void *arg)
{
    (void)arg;
    uint16_t *framebuffer = heap_caps_malloc(CAMERA_DECODE_MAX_WIDTH * CAMERA_DECODE_MAX_HEIGHT * sizeof(uint16_t), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    uint8_t *work = heap_caps_malloc(JPEG_WORK_BUFFER_SIZE, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    assert(framebuffer && work);
    bt_detector_t detector;
    bt_detector_init(&detector);
    ball_color_t previous_color = BALL_COLOR_WHITE;
    int64_t next_ball_log_us = 0;
    for (;;) {
        uvc_host_frame_t *frame = NULL;
        if (xQueueReceive(s_frame_queue, &frame, portMAX_DELAY) != pdPASS) continue;
        const bool ball_mode = s_ball_mode;
        esp_jpeg_image_cfg_t config = {
            .indata = frame->data, .indata_size = frame->data_len,
            .outbuf = (uint8_t *)framebuffer,
            .outbuf_size = CAMERA_DECODE_MAX_WIDTH * CAMERA_DECODE_MAX_HEIGHT * sizeof(uint16_t),
            .out_format = JPEG_IMAGE_FORMAT_RGB565,
            /* Keep the original task-2 latency during line tracking.  The
             * higher resolution is only needed by ball detection. */
            .out_scale = ball_mode
                             ? choose_decode_scale(&frame->vs_format)
                             : JPEG_IMAGE_SCALE_1_4,
            .flags = {.swap_color_bytes = 1},
            .advanced = {.working_buffer = work, .working_buffer_size = JPEG_WORK_BUFFER_SIZE},
        };
        esp_jpeg_image_output_t output = {0};
        if (esp_jpeg_decode(&config, &output) == ESP_OK &&
            output.width <= CAMERA_DECODE_MAX_WIDTH &&
            output.height <= CAMERA_DECODE_MAX_HEIGHT) {
            if (!ball_mode) {
                analyse_rgb565(framebuffer, output.width, output.height);
            } else {
                const ball_color_t active_color =
                    ball_transport_controller_requested_color();
                if (active_color != previous_color) {
                    previous_color = active_color;
                    /* Reacquire the target for the new delivery leg. */
                    detector.target_has_candidate = false;
                    detector.target_valid = false;
                    detector.target_consecutive_frames = 0;
                    ESP_LOGI(TAG, "vision switched to orange ball");
                }
                bt_detector_result_t detection = {0};
                /* The controller requests orange only after white delivery
                 * and back-away finish. Reapply after any detector reset. */
                detector.white_delivery_complete =
                    active_color == BALL_COLOR_ORANGE;
                const esp_err_t detection_error = bt_detector_process_rgb565(
                    &detector, framebuffer, output.width, output.height,
                    &detection);
                const unsigned legacy_divisor =
                    1U << choose_legacy_decode_scale(&frame->vs_format);
                const unsigned legacy_width =
                    (frame->vs_format.h_res + legacy_divisor - 1U) /
                    legacy_divisor;
                const float diameter_scale = (float)legacy_width / output.width;
                const bool detection_ok = detection_error == ESP_OK;
                const bt_ball_t *selected = active_color == BALL_COLOR_WHITE
                    ? &detection.ball : &detection.orange_ball;
                const bool ball_valid = active_color == BALL_COLOR_WHITE
                    ? detection.ball_valid : detection.orange_ball_valid;
                white_ball_result_t ball = adapt_bt_ball(
                    selected, detection_ok && ball_valid, output.width,
                    output.height, diameter_scale);
                black_target_result_t target = adapt_bt_target(
                    &detection.target, detection_ok && detection.target_valid,
                    output.width, output.height);
                rotate_frame_180(framebuffer, output.width, output.height);
                {
                    const int64_t now_us = esp_timer_get_time();
                    ball_transport_controller_submit(
                        &ball, &target, active_color, output.width,
                        output.height, now_us, detection.black_ahead);
                    if (now_us >= next_ball_log_us) {
                        next_ball_log_us = now_us + 1500000LL;
                        ESP_LOGI(TAG,
                                 "%s ball=%d center=(%+.2f,%+.2f) "
                                 "target=%d center=(%+.2f,%+.2f) "
                                 "bottom=%.2f box=%ux%u left_mask=%d "
                                 "black_ahead=%d dark=%u%%",
                                 active_color == BALL_COLOR_WHITE
                                     ? "white" : "orange",
                                 ball.valid, ball.center_x, ball.center_y,
                                 target.valid, target.center_x,
                                 target.center_y,
                                 target.valid ? (float)(target.bottom + 1U) /
                                     output.height : 0.0f,
                                 target.valid ? (unsigned)(target.right - target.left + 1U) : 0U,
                                 target.valid ? (unsigned)(target.bottom - target.top + 1U) : 0U,
                                 detector.white_left_mask_active,
                                 detection.black_ahead,
                                 (unsigned)detection.black_ahead_percent);
                    }
                }
                if (!detection_ok) {
                    const bool keep_left_mask = detector.white_left_mask_active;
                    bt_detector_init(&detector);
                    detector.white_left_mask_active = keep_left_mask;
                    ESP_LOGW(TAG, "BallDet v2 failed: %s",
                             esp_err_to_name(detection_error));
                }

                if (ball.valid) {
                    draw_detection_box(
                        framebuffer, output.width, output.height,
                        ball.left, ball.top, ball.right, ball.bottom,
                        active_color == BALL_COLOR_WHITE ? 0x07E0 : 0xFD20);
                }
                if (target.valid) {
                    draw_detection_box(
                        framebuffer, output.width, output.height,
                        target.left, target.top, target.right, target.bottom,
                        0xF800);
                }
                const int64_t display_us = esp_timer_get_time();
                if (display_us - s_last_display_us >=
                    CAMERA_DISPLAY_PERIOD_US) {
                    s_last_display_us = display_us;
                    if (CAMERA_TFT_PREVIEW_ENABLED) {
            ESP_ERROR_CHECK_WITHOUT_ABORT(
                        tft_display_show_camera_frame(
                            framebuffer, output.width, output.height));
        }
                }
            }
        }
        if (s_stream) ESP_ERROR_CHECK_WITHOUT_ABORT(uvc_host_frame_return(s_stream, frame));
        vTaskDelay(1);
    }
}

static void stream_event_callback(const uvc_host_stream_event_data_t *event, void *user_ctx)
{
    (void)user_ctx;
    if (event->type == UVC_HOST_DEVICE_DISCONNECTED) {
        ESP_LOGW(TAG, "camera disconnected");
        s_device_connected = false;
        ESP_ERROR_CHECK_WITHOUT_ABORT(uvc_host_stream_close(event->device_disconnected.stream_hdl));
        s_stream_task_started = false;
    }
}

static void stream_task(void *arg)
{
    const uvc_host_stream_config_t *config = arg;
    s_stream = NULL;
    esp_err_t result = uvc_host_stream_open(config, pdMS_TO_TICKS(5000), &s_stream);
    if (result == ESP_OK) result = uvc_host_stream_start(s_stream);
    if (result != ESP_OK) {
        ESP_LOGE(TAG, "camera stream start failed: %s", esp_err_to_name(result));
        if (s_stream) uvc_host_stream_close(s_stream);
        s_stream = NULL; s_stream_task_started = false; vTaskDelete(NULL); return;
    }
    ESP_LOGI(TAG, "camera streaming: GPIO19=D-, GPIO20=D+");
    while (s_device_connected) vTaskDelay(pdMS_TO_TICKS(1000));
    s_stream = NULL;
    vTaskDelete(NULL);
}

static void driver_event_callback(const uvc_host_driver_event_data_t *event, void *user_ctx)
{
    (void)user_ctx;
    if (event->type != UVC_HOST_DRIVER_EVENT_DEVICE_CONNECTED || s_stream_task_started) return;
    const uint8_t address = event->device_connected.dev_addr;
    const uint8_t stream_index = event->device_connected.uvc_stream_index;
    size_t count = event->device_connected.frame_info_num;
    uvc_host_frame_info_t *modes = calloc(count, sizeof(*modes));
    if (!modes) return;
    if (uvc_host_get_frame_list(address, stream_index, (uvc_host_frame_info_t (*)[])modes, &count) != ESP_OK) { free(modes); return; }
    size_t best = SIZE_MAX; uint64_t best_pixels = UINT64_MAX;
    for (size_t i = 0; i < count; ++i) {
        uint64_t pixels = (uint64_t)modes[i].h_res * modes[i].v_res;
        if (modes[i].format == UVC_VS_FORMAT_MJPEG && pixels < best_pixels) { best = i; best_pixels = pixels; }
    }
    if (best == SIZE_MAX) { ESP_LOGE(TAG, "camera has no MJPEG stream"); free(modes); return; }
    const uint32_t interval = slowest_interval(&modes[best]);
    size_t frame_size = (size_t)modes[best].h_res * modes[best].v_res * 2U / 6U;
    if (frame_size < MIN_MJPEG_BUFFER_SIZE) frame_size = MIN_MJPEG_BUFFER_SIZE;
    s_stream_config = (uvc_host_stream_config_t){
        .event_cb = stream_event_callback, .frame_cb = frame_callback,
        .usb = {.dev_addr = address, .vid = UVC_HOST_ANY_VID, .pid = UVC_HOST_ANY_PID, .uvc_stream_index = stream_index},
        .vs_format = {.h_res = modes[best].h_res, .v_res = modes[best].v_res, .fps = interval_to_fps(interval), .format = UVC_VS_FORMAT_MJPEG},
        .advanced = {.number_of_frame_buffers = FRAME_BUFFER_COUNT, .frame_size = frame_size, .frame_heap_caps = MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT, .number_of_urbs = 4, .urb_size = 10U * 1024U},
    };
    ESP_LOGI(TAG, "UVC mode MJPEG %ux%u @ %.1f fps", modes[best].h_res, modes[best].v_res, interval_to_fps(interval));
    free(modes); s_device_connected = true; s_stream_task_started = true;
    if (xTaskCreatePinnedToCore(stream_task, "uvc_stream", 4096, &s_stream_config, CAMERA_TASK_PRIORITY - 1, NULL, tskNO_AFFINITY) != pdPASS) { s_device_connected = false; s_stream_task_started = false; }
}

static void usb_library_task(void *arg)
{
    (void)arg;
    for (;;) { uint32_t flags = 0; ESP_ERROR_CHECK(usb_host_lib_handle_events(portMAX_DELAY, &flags)); if (flags & USB_HOST_LIB_EVENT_FLAGS_NO_CLIENTS) usb_host_device_free_all(); }
}

esp_err_t camera_line_sensor_init(void)
{
    if (heap_caps_get_total_size(MALLOC_CAP_SPIRAM) == 0) return ESP_ERR_NOT_FOUND;
    s_frame_queue = xQueueCreate(1, sizeof(uvc_host_frame_t *));
    if (!s_frame_queue) return ESP_ERR_NO_MEM;
    if (xTaskCreatePinnedToCore(frame_task, "camera_line", 6144, NULL, CAMERA_TASK_PRIORITY, NULL, tskNO_AFFINITY) != pdPASS) return ESP_ERR_NO_MEM;
    const usb_host_config_t host = {.skip_phy_setup = false, .intr_flags = ESP_INTR_FLAG_LOWMED};
    ESP_RETURN_ON_ERROR(usb_host_install(&host), TAG, "install USB host");
    if (xTaskCreatePinnedToCore(usb_library_task, "usb_events", 4096, NULL, CAMERA_TASK_PRIORITY + 1, NULL, tskNO_AFFINITY) != pdPASS) return ESP_ERR_NO_MEM;
    const uvc_host_driver_config_t driver = {.driver_task_stack_size = 4096, .driver_task_priority = CAMERA_TASK_PRIORITY + 2, .xCoreID = tskNO_AFFINITY, .create_background_task = true, .event_cb = driver_event_callback};
    return uvc_host_install(&driver);
}

bool camera_line_sensor_get_state(camera_line_state_t *state)
{
    if (!state) return false;
    portENTER_CRITICAL(&s_state_lock);
    *state = s_state;
    portEXIT_CRITICAL(&s_state_lock);
    return state->valid;
}

void camera_line_sensor_enable_ball_mode(void)
{
    portENTER_CRITICAL(&s_state_lock);
    s_state.valid = false;
    s_ball_mode = true;
    portEXIT_CRITICAL(&s_state_lock);
    ESP_LOGI(TAG, "camera pipeline switched from line tracking to "
                 "BallDet v2");
}
