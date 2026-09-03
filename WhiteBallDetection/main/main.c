#include <assert.h>
#include <inttypes.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdlib.h>

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

#include "camera_stream_server.h"
#include "black_target_detector.h"
#include "white_ball_detector.h"
#include "white_ball_display.h"

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
    uint8_t *jpeg_work = heap_caps_malloc(
        JPEG_WORK_BUFFER_SIZE, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    white_ball_detector_t *detector = heap_caps_calloc(
        1U, sizeof(*detector), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    black_target_detector_t *target_detector = heap_caps_calloc(
        1U, sizeof(*target_detector), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    assert(framebuffer != NULL && jpeg_work != NULL && detector != NULL &&
           target_detector != NULL);

    white_ball_config_t detector_config;
    white_ball_default_config(&detector_config);
    assert(white_ball_detector_init(detector, &detector_config));
    black_target_config_t target_config;
    black_target_default_config(&target_config);
    assert(black_target_detector_init(target_detector, &target_config));

    int64_t next_log_us = 0;
    while (true) {
        uvc_host_frame_t *frame = NULL;
        if (xQueueReceive(s_frame_queue, &frame, portMAX_DELAY) != pdPASS) {
            continue;
        }

        camera_stream_server_publish_jpeg(frame->data, frame->data_len);

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
            rotate_frame_180(framebuffer, output.width, output.height);
            white_ball_result_t ball = {0};
            black_target_result_t target = {0};
            if (white_ball_detect_rgb565(detector, framebuffer, output.width,
                                         output.height, &ball)) {
                /* Search for a stopping patch only after a ball provides a
                 * spatial anchor, avoiding unrelated room shadows. */
                if (ball.valid) {
                    black_target_detect_rgb565(
                        target_detector, framebuffer, output.width,
                        output.height, ball.center_x, ball.center_y, &target);
                } else {
                    black_target_detector_init(target_detector,
                                               &target_config);
                }

                /* Detection uses the 180-degree corrected frame while the
                 * browser receives the raw JPEG. Convert both boxes back. */
                float ball_left = 0.0f, ball_top = 0.0f;
                float ball_right = 0.0f, ball_bottom = 0.0f;
                float target_left = 0.0f, target_top = 0.0f;
                float target_right = 0.0f, target_bottom = 0.0f;
                if (ball.valid) {
                    ball_left =
                        1.0f - (float)(ball.right + 1U) / output.width;
                    ball_top =
                        1.0f - (float)(ball.bottom + 1U) / output.height;
                    ball_right = 1.0f - (float)ball.left / output.width;
                    ball_bottom = 1.0f - (float)ball.top / output.height;
                }
                if (target.valid) {
                    target_left =
                        1.0f - (float)(target.right + 1U) / output.width;
                    target_top =
                        1.0f - (float)(target.bottom + 1U) / output.height;
                    target_right =
                        1.0f - (float)target.left / output.width;
                    target_bottom =
                        1.0f - (float)target.top / output.height;
                }
                camera_stream_server_publish_detection(
                    ball.valid, ball_left, ball_top, ball_right, ball_bottom,
                    ball.confidence, target.valid, target_left, target_top,
                    target_right, target_bottom);
                ESP_ERROR_CHECK_WITHOUT_ABORT(
                    white_ball_display_show_rgb565(
                        framebuffer, output.width, output.height,
                        detector_config.rgb565_byte_swapped, &ball,
                        &target));
                const int64_t now_us = esp_timer_get_time();
                if (now_us >= next_log_us) {
                    next_log_us = now_us + 500000LL;
                    ESP_LOGI(TAG,
                             "white=%d luma>=%u|local+%u chroma<=%u confirm=%u "
                             "center=(%+.2f,%.2f) area=%u diameter=%.1f "
                             "local_luma=%.1f local_chroma=%.1f "
                             "edge=%.2f shadow=%.2f "
                             "confidence=%.2f black=%d center=(%+.2f,%.2f) "
                             "area=%u",
                             ball.valid, ball.luma_threshold,
                             detector_config.local_highlight_difference,
                             ball.chroma_threshold,
                             ball.confirmation_count, ball.center_x,
                             ball.center_y, ball.area_px, ball.diameter_px,
                             ball.luma_contrast,
                             ball.chroma_contrast,
                             ball.circular_edge_score, ball.shadow_score,
                             ball.confidence, target.valid, target.center_x,
                             target.center_y, target.area_px);
                }
            }
        } else {
            camera_stream_server_publish_detection(
                false, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
                false, 0.0f, 0.0f, 0.0f, 0.0f);
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
    ESP_LOGI(TAG, "standalone white-ball camera starting");
    ESP_ERROR_CHECK(white_ball_display_init());
    if (heap_caps_get_total_size(MALLOC_CAP_SPIRAM) == 0U) {
        ESP_LOGE(TAG, "PSRAM unavailable");
        return;
    }
    ESP_ERROR_CHECK(camera_stream_server_init());
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

