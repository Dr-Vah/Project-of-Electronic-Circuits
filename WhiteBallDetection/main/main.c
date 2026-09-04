#include <assert.h>
#include <inttypes.h>
#include <math.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "esp_err.h"
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
#include "camera_display.h"
#include "orange_ball_detector.h"
#include "white_ball_detector.h"
#include "wifi_stream.h"

#define USB_TASK_PRIORITY 15
#define FRAME_BUFFER_COUNT 4
#define MIN_MJPEG_BUFFER_SIZE (128U * 1024U)
#define JPEG_WORK_BUFFER_SIZE 4096U
#define RGB_FRAME_WIDTH WHITE_BALL_MAX_WIDTH
#define RGB_FRAME_HEIGHT WHITE_BALL_MAX_HEIGHT
#define RGB_FRAME_PIXELS (RGB_FRAME_WIDTH * RGB_FRAME_HEIGHT)
#define CAMERA_ROTATE_180 1

static const char *TAG = "white_ball_app";
static QueueHandle_t s_frame_queue;
static volatile bool s_device_connected;
static bool s_stream_task_started;
static uvc_host_stream_hdl_t s_stream;
static uvc_host_stream_config_t s_stream_config;

static uint16_t rgb565_to_wire(uint16_t rgb565)
{
    return (uint16_t)((rgb565 << 8) | (rgb565 >> 8));
}

static void set_wire_pixel(uint16_t *pixels, unsigned width, unsigned height,
                           unsigned x, unsigned y, uint16_t rgb565)
{
    if (x < width && y < height) {
        pixels[(size_t)y * width + x] = rgb565_to_wire(rgb565);
    }
}

static void draw_ball_circle(uint16_t *pixels, unsigned width, unsigned height,
                             const ball_detection_t *circle)
{
    /* Green circle and red centre; framebuffer is already byte-swapped for SPI. */
    const int r = circle->radius;
    for (int thickness = -1; thickness <= 1; ++thickness) {
        const int draw_r = r + thickness;
        if (draw_r < 1) continue;
        for (int degree = 0; degree < 360; degree += 4) {
            const float a = degree * 0.0174532925f;
            const int x = (int)lroundf(circle->x + draw_r * cosf(a));
            const int y = (int)lroundf(circle->y + draw_r * sinf(a));
            set_wire_pixel(pixels, width, height, x, y, 0x07E0);
        }
    }
    for (int y = -2; y <= 2; ++y)
        for (int x = -2; x <= 2; ++x)
            set_wire_pixel(pixels, width, height, circle->x + x, circle->y + y, 0xF800);
}

static void downscale_for_display(const uint16_t *source, unsigned source_width,
                                  unsigned source_height, uint16_t *display,
                                  unsigned *display_width, unsigned *display_height)
{
    unsigned out_width, out_height;
    if ((uint64_t)source_width * CAMERA_DISPLAY_HEIGHT >
        (uint64_t)source_height * CAMERA_DISPLAY_WIDTH) {
        out_width = CAMERA_DISPLAY_WIDTH;
        out_height = source_height * out_width / source_width;
    } else {
        out_height = CAMERA_DISPLAY_HEIGHT;
        out_width = source_width * out_height / source_height;
    }
    if (out_width == 0) out_width = 1;
    if (out_height == 0) out_height = 1;
    for (unsigned y = 0; y < out_height; ++y) {
        const unsigned sy = y * source_height / out_height;
        for (unsigned x = 0; x < out_width; ++x) {
            const unsigned sx = x * source_width / out_width;
            display[(size_t)y * out_width + x] =
                source[(size_t)sy * source_width + sx];
        }
    }
    *display_width = out_width;
    *display_height = out_height;
}

static ball_detection_t ball_result_to_detection(
    const white_ball_result_t *ball)
{
    ball_detection_t detection = {0};
    if (ball == NULL || !ball->valid) return detection;
    detection.x = (uint16_t)(((uint32_t)ball->left + ball->right) / 2U);
    detection.y = (uint16_t)(((uint32_t)ball->top + ball->bottom) / 2U);
    detection.radius = (uint16_t)lroundf(ball->diameter_px * 0.5f);
    if (detection.radius == 0U) detection.radius = 1U;
    float coverage = ball->circular_edge_score * 32.0f;
    if (coverage < 0.0f) coverage = 0.0f;
    if (coverage > 32.0f) coverage = 32.0f;
    detection.coverage = (uint8_t)lroundf(coverage);
    return detection;
}

static float interval_to_fps(uint32_t interval)
{
    return interval == 0U ? 0.0f : 10000000.0f / interval;
}

static uint32_t slowest_interval(const uvc_host_frame_info_t *info)
{
    uint32_t selected = info->default_interval;
    if (info->interval_type == 0U) {
        if (info->interval_max > selected) selected = info->interval_max;
    } else {
        size_t count = info->interval_type;
        if (count > CONFIG_UVC_INTERVAL_ARRAY_SIZE) {
            count = CONFIG_UVC_INTERVAL_ARRAY_SIZE;
        }
        for (size_t index = 0U; index < count; ++index) {
            if (info->interval[index] > selected) {
                selected = info->interval[index];
            }
        }
    }
    return selected;
}

static esp_jpeg_image_scale_t choose_decode_scale(
    const uvc_host_stream_format_t *format)
{
    if ((format->h_res + 1U) / 2U <= RGB_FRAME_WIDTH &&
        (format->v_res + 1U) / 2U <= RGB_FRAME_HEIGHT) {
        return JPEG_IMAGE_SCALE_1_2;
    }
    if ((format->h_res + 3U) / 4U <= RGB_FRAME_WIDTH &&
        (format->v_res + 3U) / 4U <= RGB_FRAME_HEIGHT) {
        return JPEG_IMAGE_SCALE_1_4;
    }
    return JPEG_IMAGE_SCALE_1_8;
}

static void rotate_frame_180(uint16_t *pixels, size_t width, size_t height)
{
#if CAMERA_ROTATE_180
    size_t front = 0U;
    size_t back = width * height - 1U;
    while (front < back) {
        const uint16_t temporary = pixels[front];
        pixels[front++] = pixels[back];
        pixels[back--] = temporary;
    }
#else
    (void)pixels;
    (void)width;
    (void)height;
#endif
}

static void rotate_ball_result_180(white_ball_result_t *ball,
                                   size_t width, size_t height)
{
#if CAMERA_ROTATE_180
    if (ball == NULL || !ball->valid) return;
    const uint16_t old_left = ball->left;
    const uint16_t old_top = ball->top;
    ball->left = (uint16_t)(width - 1U - ball->right);
    ball->right = (uint16_t)(width - 1U - old_left);
    ball->top = (uint16_t)(height - 1U - ball->bottom);
    ball->bottom = (uint16_t)(height - 1U - old_top);
    ball->center_x = -ball->center_x;
    ball->center_y = 1.0f - ball->center_y;
#else
    (void)ball;
    (void)width;
    (void)height;
#endif
}

static bool frame_callback(const uvc_host_frame_t *frame, void *user_context)
{
    (void)user_context;
    if (frame->vs_format.format != UVC_VS_FORMAT_MJPEG) return true;
    const uvc_host_frame_t *queued = frame;
    if (xQueueSendToBack(s_frame_queue, &queued, 0) != pdPASS) {
        return true;
    }
    return false;
}

static void frame_processing_task(void *argument)
{
    (void)argument;
    uint16_t *framebuffer = heap_caps_malloc(
        RGB_FRAME_PIXELS * sizeof(uint16_t),
        MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    uint16_t *displaybuffer = heap_caps_malloc(
        CAMERA_DISPLAY_WIDTH * CAMERA_DISPLAY_HEIGHT * sizeof(uint16_t),
        MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    uint8_t *jpeg_work = heap_caps_malloc(
        JPEG_WORK_BUFFER_SIZE, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    white_ball_detector_t *detector = heap_caps_calloc(
        1U, sizeof(*detector), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    orange_ball_detector_t *orange_detector = heap_caps_calloc(
        1U, sizeof(*orange_detector), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    black_target_detector_t *target_detectors = heap_caps_calloc(
        2U, sizeof(*target_detectors), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    assert(framebuffer != NULL && displaybuffer != NULL &&
           jpeg_work != NULL && detector != NULL &&
           orange_detector != NULL && target_detectors != NULL);

    white_ball_config_t detector_config;
    white_ball_default_config(&detector_config);
    assert(white_ball_detector_init(detector, &detector_config));
    orange_ball_config_t orange_config;
    orange_ball_default_config(&orange_config);
    assert(orange_ball_detector_init(orange_detector, &orange_config));
    black_target_config_t target_config;
    black_target_default_config(&target_config);
    assert(black_target_detector_init(&target_detectors[BALL_COLOR_WHITE],
                                      &target_config));
    assert(black_target_detector_init(&target_detectors[BALL_COLOR_ORANGE],
                                      &target_config));

    int64_t next_log_us = 0;
    ball_color_t previous_color = BALL_COLOR_WHITE;
    bool have_ball_anchor[2] = {false, false};
    float ball_anchor_x[2] = {0.0f, 0.0f};
    float ball_anchor_y[2] = {0.0f, 0.0f};
    bool target_label_announced[2] = {false, false};
    while (true) {
        uvc_host_frame_t *frame = NULL;
        if (xQueueReceive(s_frame_queue, &frame, portMAX_DELAY) != pdPASS) {
            continue;
        }

        wifi_stream_publish_jpeg(frame->data, frame->data_len,
                                 frame->vs_format.h_res,
                                 frame->vs_format.v_res);

        esp_jpeg_image_cfg_t jpeg_config = {
            .indata = frame->data,
            .indata_size = frame->data_len,
            .outbuf = (uint8_t *)framebuffer,
            .outbuf_size = RGB_FRAME_PIXELS * sizeof(uint16_t),
            .out_format = JPEG_IMAGE_FORMAT_RGB565,
            .out_scale = choose_decode_scale(&frame->vs_format),
            .flags = {
                .swap_color_bytes = 1,
            },
            .advanced = {
                .working_buffer = jpeg_work,
                .working_buffer_size = JPEG_WORK_BUFFER_SIZE,
            },
        };
        esp_jpeg_image_output_t output = {0};
        const esp_err_t decode_result = esp_jpeg_decode(&jpeg_config, &output);
        if (decode_result == ESP_OK && output.width <= RGB_FRAME_WIDTH &&
            output.height <= RGB_FRAME_HEIGHT) {
            white_ball_result_t balls[2] = {0};
            black_target_result_t targets[2] = {0};
            const ball_color_t active_color =
                ball_transport_controller_requested_color();
            if (active_color != previous_color) {
                previous_color = active_color;
                ESP_LOGI(TAG,
                         "vision switched to orange ball; using target "
                         "label captured at start");
            }
            bool detection_ok[2] = {true, true};
            if (active_color == BALL_COLOR_WHITE) {
                detection_ok[BALL_COLOR_WHITE] = white_ball_detect_rgb565(
                    detector, framebuffer, output.width, output.height,
                    &balls[BALL_COLOR_WHITE]);
                /* While still at the start, detect the orange ball as well
                 * so its same-side black patch gets a permanent independent
                 * label before the chassis changes viewpoint. */
                detection_ok[BALL_COLOR_ORANGE] = orange_ball_detect_rgb565(
                    orange_detector, framebuffer, output.width, output.height,
                    &balls[BALL_COLOR_ORANGE]);
            } else {
                detection_ok[BALL_COLOR_ORANGE] = orange_ball_detect_rgb565(
                    orange_detector, framebuffer, output.width, output.height,
                    &balls[BALL_COLOR_ORANGE]);
            }

            /* BallDet's browser shows the unrotated JPEG, so publish the
             * active detection before rotating the controller framebuffer. */
            ball_detection_t stream_ball = {0};
            const bool stream_ball_valid =
                detection_ok[active_color] && balls[active_color].valid;
            if (stream_ball_valid) {
                stream_ball = ball_result_to_detection(&balls[active_color]);
                if (active_color == BALL_COLOR_WHITE &&
                    detector->have_search_hint) {
                    stream_ball.score = detector->search_hint.score;
                    stream_ball.coverage = detector->search_hint.coverage;
                }
            }
            wifi_stream_publish_detection(
                stream_ball_valid,
                stream_ball_valid ? &stream_ball : NULL,
                output.width, output.height);

            rotate_frame_180(framebuffer, output.width, output.height);
            rotate_ball_result_180(&balls[BALL_COLOR_WHITE],
                                   output.width, output.height);
            rotate_ball_result_180(&balls[BALL_COLOR_ORANGE],
                                   output.width, output.height);
            const bool map_both_targets = active_color == BALL_COLOR_WHITE;
            for (int color = BALL_COLOR_WHITE;
                 color <= BALL_COLOR_ORANGE; ++color) {
                if (!detection_ok[color] ||
                    (!map_both_targets && color != active_color)) {
                    continue;
                }
                if (balls[color].valid) {
                    have_ball_anchor[color] = true;
                    ball_anchor_x[color] = balls[color].center_x;
                    ball_anchor_y[color] = balls[color].center_y;
                }
                if (have_ball_anchor[color]) {
                    black_target_detect_rgb565(
                        &target_detectors[color], framebuffer, output.width,
                        output.height, ball_anchor_x[color],
                        ball_anchor_y[color], &targets[color]);
                }
                if (targets[color].valid &&
                    !target_label_announced[color]) {
                    target_label_announced[color] = true;
                    ESP_LOGI(TAG,
                             "start target labelled: %s target at "
                             "(%+.2f,%.2f), area=%u",
                             color == BALL_COLOR_WHITE ? "LEFT/WHITE"
                                                       : "RIGHT/ORANGE",
                             targets[color].center_x,
                             targets[color].center_y,
                             targets[color].area_px);
                }
            }

            white_ball_result_t ball = balls[active_color];
            black_target_result_t target = targets[active_color];
            if (active_color == BALL_COLOR_WHITE &&
                !(target_label_announced[BALL_COLOR_WHITE] &&
                  target_label_announced[BALL_COLOR_ORANGE])) {
                /* Do not leave the start until both world-side target labels
                 * are known. This prevents reassignment after the turn. */
                memset(&target, 0, sizeof(target));
            }
            const bool ball_detection_ok = detection_ok[active_color];
            if (ball_detection_ok) {
                const int64_t frame_time_us = esp_timer_get_time();
                ball_transport_controller_submit(
                    &ball, &target, active_color, output.width, output.height,
                    frame_time_us);
                const int64_t now_us = frame_time_us;
                if (now_us >= next_log_us) {
                    next_log_us = now_us + 1500000LL;
                    ESP_LOGI(TAG,
                             "%s=%d detector=%s confirm=%u "
                             "center=(%+.2f,%.2f) area=%u diameter=%.1f "
                             "circle_support=%.2f confidence=%.2f "
                             "black=%d center=(%+.2f,%.2f) "
                             "area=%u",
                             active_color == BALL_COLOR_WHITE
                                 ? "white" : "orange",
                             ball.valid,
                             active_color == BALL_COLOR_WHITE
                                 ? "BallDet" : "OrangeCC",
                             ball.confirmation_count, ball.center_x,
                             ball.center_y, ball.area_px, ball.diameter_px,
                             ball.circular_edge_score, ball.confidence,
                             target.valid, target.center_x,
                             target.center_y, target.area_px);
                }
            }

            unsigned display_width = 0U, display_height = 0U;
            downscale_for_display(framebuffer, output.width, output.height,
                                  displaybuffer, &display_width,
                                  &display_height);
            if (ball_detection_ok && ball.valid) {
                ball_detection_t screen_ball =
                    ball_result_to_detection(&ball);
                screen_ball.x = (uint16_t)(
                    (uint32_t)screen_ball.x * display_width / output.width);
                screen_ball.y = (uint16_t)(
                    (uint32_t)screen_ball.y * display_height / output.height);
                screen_ball.radius = (uint16_t)(
                    (uint32_t)screen_ball.radius * display_width /
                    output.width);
                if (screen_ball.radius < 2U) screen_ball.radius = 2U;
                draw_ball_circle(displaybuffer, display_width,
                                 display_height, &screen_ball);
            }
            ESP_ERROR_CHECK_WITHOUT_ABORT(camera_display_draw_rgb565(
                displaybuffer, display_width, display_height));
        } else {
            wifi_stream_publish_detection(false, NULL,
                                           frame->vs_format.h_res,
                                           frame->vs_format.v_res);
            ESP_LOGW(TAG, "JPEG decode failed: %s, output=%ux%u",
                     esp_err_to_name(decode_result), output.width,
                     output.height);
        }

        if (s_stream != NULL) {
            ESP_ERROR_CHECK_WITHOUT_ABORT(
                uvc_host_frame_return(s_stream, frame));
        }
        vTaskDelay(1);
    }
}

static void stream_event_callback(const uvc_host_stream_event_data_t *event,
                                  void *user_context)
{
    (void)user_context;
    switch (event->type) {
    case UVC_HOST_TRANSFER_ERROR:
        ESP_LOGE(TAG, "USB transfer error: %s",
                 esp_err_to_name(event->transfer_error.error));
        break;
    case UVC_HOST_DEVICE_DISCONNECTED:
        ESP_LOGW(TAG, "camera disconnected");
        s_device_connected = false;
        ESP_ERROR_CHECK_WITHOUT_ABORT(
            uvc_host_stream_close(event->device_disconnected.stream_hdl));
        s_stream_task_started = false;
        break;
    case UVC_HOST_FRAME_BUFFER_OVERFLOW:
        ESP_LOGW(TAG, "camera frame buffer overflow");
        break;
    case UVC_HOST_FRAME_BUFFER_UNDERFLOW:
        ESP_LOGW(TAG, "no free camera frame buffer");
        break;
    default:
        break;
    }
}

static void stream_task(void *argument)
{
    const uvc_host_stream_config_t *config = argument;
    s_stream = NULL;
    esp_err_t result = uvc_host_stream_open(
        config, pdMS_TO_TICKS(5000), &s_stream);
    if (result != ESP_OK) {
        ESP_LOGE(TAG, "open camera stream failed: %s", esp_err_to_name(result));
        s_stream_task_started = false;
        vTaskDelete(NULL);
        return;
    }
    result = uvc_host_stream_start(s_stream);
    if (result != ESP_OK) {
        ESP_LOGE(TAG, "start camera stream failed: %s", esp_err_to_name(result));
        uvc_host_stream_close(s_stream);
        s_stream = NULL;
        s_stream_task_started = false;
        vTaskDelete(NULL);
        return;
    }
    ESP_LOGI(TAG, "camera streaming started");
    while (s_device_connected) vTaskDelay(pdMS_TO_TICKS(1000));
    s_stream = NULL;
    vTaskDelete(NULL);
}

static void driver_event_callback(const uvc_host_driver_event_data_t *event,
                                  void *user_context)
{
    (void)user_context;
    if (event->type != UVC_HOST_DRIVER_EVENT_DEVICE_CONNECTED ||
        s_stream_task_started) return;

    const uint8_t address = event->device_connected.dev_addr;
    const uint8_t stream_index = event->device_connected.uvc_stream_index;
    size_t count = event->device_connected.frame_info_num;
    if (count == 0U) return;
    uvc_host_frame_info_t *modes = calloc(count, sizeof(*modes));
    if (modes == NULL) return;
    esp_err_t result = uvc_host_get_frame_list(
        address, stream_index, (uvc_host_frame_info_t (*)[])modes, &count);
    if (result != ESP_OK) {
        free(modes);
        return;
    }

    size_t best = SIZE_MAX;
    uint64_t best_pixels = UINT64_MAX;
    for (size_t index = 0U; index < count; ++index) {
        const uint64_t pixels =
            (uint64_t)modes[index].h_res * modes[index].v_res;
        if (modes[index].format == UVC_VS_FORMAT_MJPEG &&
            pixels < best_pixels) {
            best = index;
            best_pixels = pixels;
        }
    }
    if (best == SIZE_MAX) {
        ESP_LOGE(TAG, "camera has no MJPEG mode");
        free(modes);
        return;
    }

    const uint32_t interval = slowest_interval(&modes[best]);
    size_t frame_size =
        (size_t)modes[best].h_res * modes[best].v_res * 2U / 6U;
    if (frame_size < MIN_MJPEG_BUFFER_SIZE) frame_size = MIN_MJPEG_BUFFER_SIZE;
    s_stream_config = (uvc_host_stream_config_t) {
        .event_cb = stream_event_callback,
        .frame_cb = frame_callback,
        .usb = {
            .dev_addr = address,
            .vid = UVC_HOST_ANY_VID,
            .pid = UVC_HOST_ANY_PID,
            .uvc_stream_index = stream_index,
        },
        .vs_format = {
            .h_res = modes[best].h_res,
            .v_res = modes[best].v_res,
            .fps = interval_to_fps(interval),
            .format = UVC_VS_FORMAT_MJPEG,
        },
        .advanced = {
            .number_of_frame_buffers = FRAME_BUFFER_COUNT,
            .frame_size = frame_size,
            .frame_heap_caps = MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT,
            .number_of_urbs = 4,
            .urb_size = 10U * 1024U,
        },
    };
    ESP_LOGI(TAG, "camera connected: MJPEG %ux%u @ %.1f fps",
             modes[best].h_res, modes[best].v_res,
             interval_to_fps(interval));
    free(modes);

    s_device_connected = true;
    s_stream_task_started = true;
    if (xTaskCreatePinnedToCore(stream_task, "uvc_stream", 4096,
                                &s_stream_config, USB_TASK_PRIORITY - 2,
                                NULL, tskNO_AFFINITY) != pdPASS) {
        s_device_connected = false;
        s_stream_task_started = false;
    }
}

static void usb_library_task(void *argument)
{
    (void)argument;
    while (true) {
        uint32_t flags = 0U;
        ESP_ERROR_CHECK(usb_host_lib_handle_events(portMAX_DELAY, &flags));
        if ((flags & USB_HOST_LIB_EVENT_FLAGS_NO_CLIENTS) != 0U) {
            usb_host_device_free_all();
        }
    }
}

void app_main(void)
{
    ESP_LOGI(TAG, "two-ball transport starting (white, then orange)");
    ESP_ERROR_CHECK(camera_display_init());
    if (heap_caps_get_total_size(MALLOC_CAP_SPIRAM) == 0U) {
        ESP_LOGE(TAG, "PSRAM unavailable");
        return;
    }
    ESP_ERROR_CHECK(ball_transport_controller_start());
    ESP_ERROR_CHECK(wifi_stream_start());
    s_frame_queue = xQueueCreate(1U, sizeof(uvc_host_frame_t *));
    assert(s_frame_queue != NULL);
    assert(xTaskCreatePinnedToCore(frame_processing_task, "white_ball", 6144,
                                   NULL, USB_TASK_PRIORITY - 3, NULL,
                                   tskNO_AFFINITY) == pdPASS);

    const usb_host_config_t host_config = {
        .skip_phy_setup = false,
        .intr_flags = ESP_INTR_FLAG_LOWMED,
    };
    ESP_ERROR_CHECK(usb_host_install(&host_config));
    assert(xTaskCreatePinnedToCore(usb_library_task, "usb_events", 4096,
                                   NULL, USB_TASK_PRIORITY, NULL,
                                   tskNO_AFFINITY) == pdPASS);
    const uvc_host_driver_config_t driver_config = {
        .driver_task_stack_size = 4096,
        .driver_task_priority = USB_TASK_PRIORITY + 1,
        .xCoreID = tskNO_AFFINITY,
        .create_background_task = true,
        .event_cb = driver_event_callback,
    };
    ESP_ERROR_CHECK(uvc_host_install(&driver_config));
    ESP_LOGI(TAG, "waiting for UVC camera on GPIO19/20");
}
