/* USB UVC diagnostic for ESP32-S3 and a composite camera/audio device. */

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
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "jpeg_decoder.h"
#include "usb/usb_host.h"
#include "usb/uvc_host.h"

#include "camera_display.h"
#include "ball_detector.h"
#include "wifi_stream.h"

static const char *TAG = "camera_uvc_test";

#define USB_TASK_PRIORITY 15
#define FRAME_BUFFER_COUNT 3
#define MIN_MJPEG_BUFFER_SIZE (40U * 1024U)
#define JPEG_WORK_BUFFER_SIZE 4096U

/* Camera line-detection parameters. Edit these values on the PC, rebuild and
 * flash. Dark pixels (luminance <= threshold) are treated as line pixels. */
#define LINE_BINARY_THRESHOLD       85U
#define LINE_ROI_TOP_PERCENT        0U
#define LINE_ROI_BOTTOM_PERCENT     10U
#define LINE_ROI_WIDTH_PERCENT      20U
#define LINE_ROI_CENTER_OFFSET_PERCENT (-7)
#define LINE_ACTIVE_DARK_PERCENT    10U
#define LINE_CHANNEL_COUNT          4U
#define LINE_BOX_THICKNESS          2U

_Static_assert(LINE_ROI_TOP_PERCENT < LINE_ROI_BOTTOM_PERCENT,
               "ROI top must be above ROI bottom");
_Static_assert(LINE_ROI_BOTTOM_PERCENT <= 100U,
               "ROI bottom percentage must be <= 100");
_Static_assert(LINE_ROI_WIDTH_PERCENT >= LINE_CHANNEL_COUNT &&
               LINE_ROI_WIDTH_PERCENT <= 100U,
               "ROI width must be between 4 and 100 percent");
_Static_assert(LINE_ROI_CENTER_OFFSET_PERCENT >= -100 &&
               LINE_ROI_CENTER_OFFSET_PERCENT <= 100,
               "ROI horizontal offset must be between -100 and 100 percent");

typedef struct {
    uint32_t dark_pixels[LINE_CHANNEL_COUNT];
    uint32_t total_pixels[LINE_CHANNEL_COUNT];
    uint16_t centroid_x[LINE_CHANNEL_COUNT];
    bool active[LINE_CHANNEL_COUNT];
} line_vision_state_t;

typedef struct {
    const uint16_t *pixels;
    uint16_t width;
    uint16_t height;
} detector_frame_t;

typedef struct {
    bool valid;
    ball_detection_t ball;
    uint16_t source_width;
    uint16_t source_height;
} detector_message_t;

static bool s_stream_task_started;
static volatile bool s_device_connected;
static uvc_host_stream_config_t s_stream_config;
static uvc_host_stream_hdl_t s_stream;
static QueueHandle_t s_frame_queue;
static QueueHandle_t s_detector_frame_queue;
static QueueHandle_t s_detector_result_queue;
static SemaphoreHandle_t s_detector_buffer_available;
static uint16_t *s_detector_buffer;
static size_t s_detector_buffer_capacity;
static ball_detector_tracker_t s_ball_tracker;

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

static void __attribute__((unused)) process_line_frame(
    uint16_t *pixels, unsigned width, unsigned height,
    line_vision_state_t *state)
{
    memset(state, 0, sizeof(*state));
    const unsigned y0 = height * LINE_ROI_TOP_PERCENT / 100U;
    const unsigned y1 = height * LINE_ROI_BOTTOM_PERCENT / 100U;
    const unsigned roi_width = width * LINE_ROI_WIDTH_PERCENT / 100U;
    int x_roi0_signed = (int)(width - roi_width) / 2 +
        (int)width * LINE_ROI_CENTER_OFFSET_PERCENT / 100;
    if (x_roi0_signed < 0) x_roi0_signed = 0;
    if ((unsigned)x_roi0_signed + roi_width > width) {
        x_roi0_signed = (int)(width - roi_width);
    }
    const unsigned x_roi0 = (unsigned)x_roi0_signed;
    const unsigned x_roi1 = x_roi0 + roi_width;
    uint32_t x_sum[LINE_CHANNEL_COUNT] = {0};

    for (unsigned y = 0; y < height; ++y) {
        for (unsigned x = 0; x < width; ++x) {
            const size_t index = (size_t)y * width + x;
            const uint16_t wire = pixels[index];
            const uint16_t rgb = (uint16_t)((wire << 8) | (wire >> 8));
            const unsigned red = (rgb >> 11) & 31U;
            const unsigned green = (rgb >> 5) & 63U;
            const unsigned blue = rgb & 31U;
            const unsigned luminance =
                (77U * red * 255U / 31U + 150U * green * 255U / 63U +
                 29U * blue * 255U / 31U) >> 8;
            const bool dark = luminance <= LINE_BINARY_THRESHOLD;
            pixels[index] = dark ? rgb565_to_wire(0x0000) :
                                   rgb565_to_wire(0xFFFF);

            if (x >= x_roi0 && x < x_roi1 && y >= y0 && y < y1) {
                unsigned channel =
                    (x - x_roi0) * LINE_CHANNEL_COUNT / roi_width;
                if (channel >= LINE_CHANNEL_COUNT) channel = LINE_CHANNEL_COUNT - 1U;
                ++state->total_pixels[channel];
                if (dark) {
                    ++state->dark_pixels[channel];
                    x_sum[channel] += x;
                }
            }
        }
    }

    /* Cyan, yellow, magenta and green: visible on both black and white. */
    static const uint16_t box_color[LINE_CHANNEL_COUNT] = {
        0x07FF, 0xFFE0, 0xF81F, 0x07E0,
    };
    for (unsigned channel = 0; channel < LINE_CHANNEL_COUNT; ++channel) {
        const unsigned x0 = x_roi0 +
            channel * roi_width / LINE_CHANNEL_COUNT;
        const unsigned x1 = x_roi0 +
            (channel + 1U) * roi_width / LINE_CHANNEL_COUNT - 1U;
        const unsigned channel_width = x1 - x0 + 1U;
        unsigned box_thickness = channel_width / 4U;
        if (box_thickness == 0) box_thickness = 1U;
        if (box_thickness > LINE_BOX_THICKNESS) {
            box_thickness = LINE_BOX_THICKNESS;
        }
        for (unsigned thickness = 0; thickness < box_thickness; ++thickness) {
            for (unsigned x = x0; x <= x1; ++x) {
                set_wire_pixel(pixels, width, height, x, y0 + thickness,
                               box_color[channel]);
                if (y1 > thickness) {
                    set_wire_pixel(pixels, width, height, x, y1 - 1U - thickness,
                                   box_color[channel]);
                }
            }
            for (unsigned y = y0; y < y1; ++y) {
                set_wire_pixel(pixels, width, height, x0 + thickness, y,
                               box_color[channel]);
                if (x1 >= thickness) {
                    set_wire_pixel(pixels, width, height, x1 - thickness, y,
                                   box_color[channel]);
                }
            }
        }
        const uint32_t dark = state->dark_pixels[channel];
        const uint32_t total = state->total_pixels[channel];
        state->centroid_x[channel] = dark ? (uint16_t)(x_sum[channel] / dark) : 0;
        state->active[channel] = total != 0 &&
            dark * 100U >= total * LINE_ACTIVE_DARK_PERCENT;
    }
}

static const char *format_name(enum uvc_host_stream_format format)
{
    switch (format) {
    case UVC_VS_FORMAT_DEFAULT: return "DEFAULT";
    case UVC_VS_FORMAT_MJPEG: return "MJPEG";
    case UVC_VS_FORMAT_YUY2: return "YUY2";
    case UVC_VS_FORMAT_H264: return "H264";
    case UVC_VS_FORMAT_H265: return "H265";
    case UVC_VS_FORMAT_NV12: return "NV12";
    default: return "UNKNOWN";
    }
}

static float interval_to_fps(uint32_t interval)
{
    return interval == 0 ? 0.0f : 10000000.0f / (float)interval;
}

static uint32_t slowest_interval(const uvc_host_frame_info_t *info)
{
    uint32_t selected = info->default_interval;
    if (info->interval_type == 0) {
        if (info->interval_max > selected) selected = info->interval_max;
    } else {
        size_t count = info->interval_type;
        if (count > CONFIG_UVC_INTERVAL_ARRAY_SIZE) count = CONFIG_UVC_INTERVAL_ARRAY_SIZE;
        for (size_t i = 0; i < count; ++i) {
            if (info->interval[i] > selected) selected = info->interval[i];
        }
    }
    return selected;
}

static void print_frame_info(const uvc_host_frame_info_t *info, size_t index)
{
    ESP_LOGI(TAG, "mode[%u]: %s %ux%u, default %.1f fps",
             (unsigned)index, format_name(info->format), info->h_res,
             info->v_res, interval_to_fps(info->default_interval));
    if (info->interval_type == 0) {
        ESP_LOGI(TAG, "  continuous: %.1f..%.1f fps, step=%" PRIu32,
                 interval_to_fps(info->interval_max),
                 interval_to_fps(info->interval_min), info->interval_step);
    } else {
        size_t count = info->interval_type;
        if (count > CONFIG_UVC_INTERVAL_ARRAY_SIZE) count = CONFIG_UVC_INTERVAL_ARRAY_SIZE;
        for (size_t i = 0; i < count; ++i) {
            ESP_LOGI(TAG, "  rate[%u]: %.1f fps", (unsigned)i,
                     interval_to_fps(info->interval[i]));
        }
    }
}

static bool frame_callback(const uvc_host_frame_t *frame, void *user_ctx)
{
    (void)user_ctx;
    if (frame->vs_format.format != UVC_VS_FORMAT_MJPEG) return true;
    const uvc_host_frame_t *queued_frame = frame;
    if (xQueueSendToBack(s_frame_queue, &queued_frame, 0) != pdPASS) {
        return true;  /* Display is busy: drop this frame without blocking USB. */
    }
    return false;     /* Processing task returns it with uvc_host_frame_return(). */
}

static void detector_task(void *argument)
{
    (void)argument;
    uint32_t frames = 0;
    uint32_t hits = 0;
    uint64_t elapsed_us = 0;
    uint32_t local_frames = 0;
    uint64_t local_elapsed_us = 0;
    uint32_t global_frames = 0;
    uint64_t global_elapsed_us = 0;
    uint8_t consecutive_misses = 0;
    bool have_search_hint = false;
    ball_detection_t search_hint = {0};
    int64_t statistics_start = esp_timer_get_time();

    while (true) {
        detector_frame_t frame;
        if (xQueueReceive(s_detector_frame_queue, &frame, portMAX_DELAY) != pdPASS) {
            continue;
        }

        ball_detector_result_t detection = {0};
        ball_detection_t stable = {0};
        const bool used_local_search = have_search_hint;
        const int64_t started = esp_timer_get_time();
        const esp_err_t error = ball_detector_process_rgb565(
            frame.pixels, frame.width, frame.height,
            used_local_search ? &search_hint : NULL, &detection);
        const uint64_t frame_elapsed_us =
            (uint64_t)(esp_timer_get_time() - started);
        elapsed_us += frame_elapsed_us;
        if (used_local_search) {
            ++local_frames;
            local_elapsed_us += frame_elapsed_us;
        } else {
            ++global_frames;
            global_elapsed_us += frame_elapsed_us;
        }
        ++frames;

        if (error == ESP_OK && detection.found) {
            ++hits;
            consecutive_misses = 0;
            if (ball_detector_tracker_update(&s_ball_tracker, &detection, &stable)) {
                search_hint = stable;
                have_search_hint = true;
                const detector_message_t message = {
                    .valid = true,
                    .ball = stable,
                    .source_width = frame.width,
                    .source_height = frame.height,
                };
                xQueueOverwrite(s_detector_result_queue, &message);
                wifi_stream_publish_detection(true, &stable,
                                              frame.width, frame.height);
                ESP_LOGI(TAG,
                         "BALL stable center=(%u,%u) r=%u score=%" PRId32
                         " support=%u/32",
                         stable.x, stable.y, stable.radius, stable.score,
                         stable.coverage);
            } else if (have_search_hint) {
                search_hint = detection.ball;
            }
        } else {
            ball_detector_result_t missing = {0};
            ball_detector_tracker_update(&s_ball_tracker, &missing, NULL);
            if (consecutive_misses < UINT8_MAX) ++consecutive_misses;
            if (consecutive_misses == 2) {
                have_search_hint = false;
                memset(&search_hint, 0, sizeof(search_hint));
                const detector_message_t message = {.valid = false};
                xQueueOverwrite(s_detector_result_queue, &message);
                wifi_stream_publish_detection(false, NULL,
                                              frame.width, frame.height);
            }
            if (error != ESP_OK) {
                ESP_LOGW(TAG, "ball detector failed: %s", esp_err_to_name(error));
            }
        }

        xSemaphoreGive(s_detector_buffer_available);

        const int64_t now = esp_timer_get_time();
        if (now - statistics_start >= 1000000) {
            const double seconds = (double)(now - statistics_start) / 1000000.0;
            ESP_LOGI(TAG,
                     "BALL worker: processed=%.1f fps, hits=%" PRIu32
                     ", average=%" PRIu64 " ms, global=%" PRIu64
                     " ms (%" PRIu32 "), local=%" PRIu64 " ms (%" PRIu32 ")",
                     frames / seconds, hits,
                     frames ? elapsed_us / frames / 1000U : 0U,
                     global_frames ? global_elapsed_us / global_frames / 1000U : 0U,
                     global_frames,
                     local_frames ? local_elapsed_us / local_frames / 1000U : 0U,
                     local_frames);
            statistics_start = now;
            frames = 0;
            hits = 0;
            elapsed_us = 0;
            local_frames = 0;
            local_elapsed_us = 0;
            global_frames = 0;
            global_elapsed_us = 0;
        }
    }
}

static void frame_processing_task(void *argument)
{
    (void)argument;
    uint16_t *framebuffer = NULL;
    size_t framebuffer_capacity = 0;
    uint16_t *displaybuffer = heap_caps_malloc(
        CAMERA_DISPLAY_WIDTH * CAMERA_DISPLAY_HEIGHT * sizeof(uint16_t),
        MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    uint8_t *work_buffer = heap_caps_malloc(JPEG_WORK_BUFFER_SIZE,
                                            MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    assert(displaybuffer != NULL && work_buffer != NULL);

    int64_t statistics_start = esp_timer_get_time();
    uint32_t received = 0;
    uint32_t displayed = 0;
    uint32_t decode_failed = 0;
    bool displayed_ball_valid = false;
    detector_message_t latest_ball = {0};
    line_vision_state_t line_state = {0};
    while (true) {
        uvc_host_frame_t *frame = NULL;
        if (xQueueReceive(s_frame_queue, &frame, portMAX_DELAY) != pdPASS) continue;
        ++received;
        wifi_stream_publish_jpeg(frame->data, frame->data_len,
                                 frame->vs_format.h_res,
                                 frame->vs_format.v_res);

        esp_jpeg_image_cfg_t jpeg_config = {
            .indata = frame->data,
            .indata_size = frame->data_len,
            .out_format = JPEG_IMAGE_FORMAT_RGB565,
            .out_scale = JPEG_IMAGE_SCALE_0,
            .flags = {
                .swap_color_bytes = 1,
            },
            .advanced = {
                .working_buffer = work_buffer,
                .working_buffer_size = JPEG_WORK_BUFFER_SIZE,
            },
        };
        esp_jpeg_image_output_t output = {0};
        esp_err_t result = esp_jpeg_get_image_info(&jpeg_config, &output);
        if (result == ESP_OK && output.output_len > framebuffer_capacity) {
            uint16_t *larger = heap_caps_realloc(
                framebuffer, output.output_len,
                MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
            if (larger == NULL) {
                ESP_LOGE(TAG, "cannot allocate %u-byte full-resolution frame",
                         (unsigned)output.output_len);
                result = ESP_ERR_NO_MEM;
            } else {
                framebuffer = larger;
                framebuffer_capacity = output.output_len;
            }
        }
        if (result == ESP_OK) {
            jpeg_config.outbuf = (uint8_t *)framebuffer;
            jpeg_config.outbuf_size = framebuffer_capacity;
            result = esp_jpeg_decode(&jpeg_config, &output);
        }
        if (result == ESP_OK) {
            if (xSemaphoreTake(s_detector_buffer_available, 0) == pdTRUE) {
                if (output.output_len > s_detector_buffer_capacity) {
                    uint16_t *larger = heap_caps_realloc(
                        s_detector_buffer, output.output_len,
                        MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
                    if (larger != NULL) {
                        s_detector_buffer = larger;
                        s_detector_buffer_capacity = output.output_len;
                    }
                }
                if (s_detector_buffer != NULL &&
                    output.output_len <= s_detector_buffer_capacity) {
                    memcpy(s_detector_buffer, framebuffer, output.output_len);
                    const detector_frame_t detector_frame = {
                        .pixels = s_detector_buffer,
                        .width = output.width,
                        .height = output.height,
                    };
                    if (xQueueSend(s_detector_frame_queue, &detector_frame, 0) != pdPASS) {
                        xSemaphoreGive(s_detector_buffer_available);
                    }
                } else {
                    ESP_LOGW(TAG, "cannot allocate detector snapshot (%u bytes)",
                             (unsigned)output.output_len);
                    xSemaphoreGive(s_detector_buffer_available);
                }
            }

            detector_message_t message;
            while (xQueueReceive(s_detector_result_queue, &message, 0) == pdPASS) {
                displayed_ball_valid = message.valid;
                if (message.valid) latest_ball = message;
            }
            unsigned display_width = 0, display_height = 0;
            downscale_for_display(framebuffer, output.width, output.height,
                                  displaybuffer, &display_width, &display_height);
            if (displayed_ball_valid && latest_ball.source_width != 0 &&
                latest_ball.source_height != 0) {
                ball_detection_t screen_ball = latest_ball.ball;
                screen_ball.x = (uint16_t)((uint32_t)latest_ball.ball.x * display_width /
                                           latest_ball.source_width);
                screen_ball.y = (uint16_t)((uint32_t)latest_ball.ball.y * display_height /
                                           latest_ball.source_height);
                screen_ball.radius = (uint16_t)((uint32_t)latest_ball.ball.radius * display_width /
                                                latest_ball.source_width);
                if (screen_ball.radius < 2) screen_ball.radius = 2;
                draw_ball_circle(displaybuffer, display_width, display_height, &screen_ball);
            }
            result = camera_display_draw_rgb565(displaybuffer, display_width,
                                                 display_height);
            if (result == ESP_OK) {
                ++displayed;
            } else {
                ESP_LOGE(TAG, "LCD frame transfer failed: %s",
                         esp_err_to_name(result));
            }
        } else {
            ++decode_failed;
            ESP_LOGW(TAG, "JPEG decode failed or size mismatch: %s, %ux%u",
                     esp_err_to_name(result), output.width, output.height);
        }
        if (s_stream != NULL) {
            ESP_ERROR_CHECK_WITHOUT_ABORT(uvc_host_frame_return(s_stream, frame));
        }

        /* The camera produces frames faster than JPEG decode + full-screen SPI
         * refresh.  Without a short blocked period the queue is never empty,
         * so this high-priority task can starve the core's IDLE task and trip
         * the task watchdog even though streaming remains healthy. */
        /* Use one explicit scheduler tick.  With the IDF default 100 Hz tick,
         * pdMS_TO_TICKS(2) rounds down to zero and does not block at all. */
        vTaskDelay(1);

        const int64_t now = esp_timer_get_time();
        if (now - statistics_start >= 1000000) {
            const double seconds = (double)(now - statistics_start) / 1000000.0;
            ESP_LOGI(TAG,
                     "DISPLAY OK: received=%.1f fps, shown=%.1f fps, "
                     "decode failures=%" PRIu32 ", free PSRAM=%u",
                     received / seconds, displayed / seconds, decode_failed,
                     (unsigned)heap_caps_get_free_size(MALLOC_CAP_SPIRAM));
            ESP_LOGI(TAG,
                     "LINE T=%u ROI_Y=%u..%u%% ROI_W=%u%% OFFSET=%+d%% | "
                     "CH1=%s %u%% | CH2=%s %u%% | "
                     "CH3=%s %u%% | CH4=%s %u%%",
                     LINE_BINARY_THRESHOLD, LINE_ROI_TOP_PERCENT,
                     LINE_ROI_BOTTOM_PERCENT, LINE_ROI_WIDTH_PERCENT,
                     LINE_ROI_CENTER_OFFSET_PERCENT,
                     line_state.active[0] ? "ON" : "OFF",
                     line_state.total_pixels[0] ? (unsigned)(line_state.dark_pixels[0] * 100U / line_state.total_pixels[0]) : 0,
                     line_state.active[1] ? "ON" : "OFF",
                     line_state.total_pixels[1] ? (unsigned)(line_state.dark_pixels[1] * 100U / line_state.total_pixels[1]) : 0,
                     line_state.active[2] ? "ON" : "OFF",
                     line_state.total_pixels[2] ? (unsigned)(line_state.dark_pixels[2] * 100U / line_state.total_pixels[2]) : 0,
                     line_state.active[3] ? "ON" : "OFF",
                     line_state.total_pixels[3] ? (unsigned)(line_state.dark_pixels[3] * 100U / line_state.total_pixels[3]) : 0);
            statistics_start = now;
            received = 0;
            displayed = 0;
            decode_failed = 0;
        }
    }
}

static void stream_event_callback(const uvc_host_stream_event_data_t *event,
                                  void *user_ctx)
{
    (void)user_ctx;
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
        ESP_LOGW(TAG, "frame buffer overflow; increase frame_size");
        break;
    case UVC_HOST_FRAME_BUFFER_UNDERFLOW:
        ESP_LOGW(TAG, "no free frame buffer");
        break;
#ifdef UVC_HOST_SUSPEND_RESUME_API_SUPPORTED
    case UVC_HOST_DEVICE_SUSPENDED:
        ESP_LOGW(TAG, "camera suspended");
        break;
    case UVC_HOST_DEVICE_RESUMED:
        ESP_LOGI(TAG, "camera resumed");
        break;
#endif
    default:
        break;
    }
}

static void stream_task(void *argument)
{
    const uvc_host_stream_config_t *config = argument;
    s_stream = NULL;
    ESP_LOGI(TAG, "opening exact camera mode: %s %ux%u @ %.1f fps",
             format_name(config->vs_format.format), config->vs_format.h_res,
             config->vs_format.v_res, config->vs_format.fps);
    esp_err_t result = uvc_host_stream_open(config, pdMS_TO_TICKS(5000),
                                            &s_stream);
    if (result != ESP_OK) {
        ESP_LOGE(TAG, "stream open failed: %s", esp_err_to_name(result));
        s_stream_task_started = false;
        vTaskDelete(NULL);
        return;
    }

    uvc_host_desc_print(s_stream);
    uvc_host_stream_format_t selected = {0};
    ESP_ERROR_CHECK_WITHOUT_ABORT(uvc_host_stream_format_get(s_stream, &selected));
    ESP_LOGI(TAG, "negotiated: %s %ux%u @ %.1f fps",
             format_name(selected.format), selected.h_res, selected.v_res, selected.fps);
    result = uvc_host_stream_start(s_stream);
    if (result != ESP_OK) {
        ESP_LOGE(TAG, "stream start failed: %s", esp_err_to_name(result));
        uvc_host_stream_close(s_stream);
        s_stream = NULL;
        s_stream_task_started = false;
        vTaskDelete(NULL);
        return;
    }
    ESP_LOGI(TAG, "streaming started; statistics will print once per second");
    while (s_device_connected) vTaskDelay(pdMS_TO_TICKS(1000));
    s_stream = NULL;
    vTaskDelete(NULL);
}

static void driver_event_callback(const uvc_host_driver_event_data_t *event,
                                  void *user_ctx)
{
    (void)user_ctx;
    if (event->type != UVC_HOST_DRIVER_EVENT_DEVICE_CONNECTED) return;

    const uint8_t dev_addr = event->device_connected.dev_addr;
    const uint8_t stream_index = event->device_connected.uvc_stream_index;
    size_t count = event->device_connected.frame_info_num;
    ESP_LOGI(TAG, "UVC camera found: address=%u, stream=%u, modes=%u",
             dev_addr, stream_index, (unsigned)count);
    if (count == 0) {
        ESP_LOGE(TAG, "driver found no video frame descriptors");
        return;
    }

    uvc_host_frame_info_t *modes = calloc(count, sizeof(*modes));
    if (modes == NULL) {
        ESP_LOGE(TAG, "cannot allocate mode list");
        return;
    }
    esp_err_t result = uvc_host_get_frame_list(
        dev_addr, stream_index, (uvc_host_frame_info_t (*)[])modes, &count);
    if (result != ESP_OK) {
        ESP_LOGE(TAG, "cannot read camera modes: %s", esp_err_to_name(result));
        free(modes);
        return;
    }

    size_t best = SIZE_MAX;
    size_t fallback = SIZE_MAX;
    uint64_t best_pixels = UINT64_MAX;
    uint64_t fallback_pixels = 0;
    for (size_t i = 0; i < count; ++i) {
        print_frame_info(&modes[i], i);
        const uint64_t pixels = (uint64_t)modes[i].h_res * modes[i].v_res;
        if (modes[i].format == UVC_VS_FORMAT_MJPEG) {
            if (pixels > fallback_pixels) {
                fallback = i;
                fallback_pixels = pixels;
            }
            /* The validated detector works at 320 px wide. Prefer the least
             * expensive mode that preserves at least that much information. */
            if (modes[i].h_res >= 320 && pixels < best_pixels) {
                best = i;
                best_pixels = pixels;
            }
        }
    }
    if (best == SIZE_MAX) best = fallback;
    if (best == SIZE_MAX) {
        ESP_LOGE(TAG, "camera advertises no MJPEG mode; raw video exceeds ESP32-S3 full-speed bandwidth");
        free(modes);
        return;
    }
    if (s_stream_task_started) {
        ESP_LOGW(TAG, "stream task already active; ignoring duplicate event");
        free(modes);
        return;
    }

    const uint32_t interval = slowest_interval(&modes[best]);
    size_t frame_size = (size_t)modes[best].h_res * modes[best].v_res * 2U / 6U;
    if (frame_size < MIN_MJPEG_BUFFER_SIZE) frame_size = MIN_MJPEG_BUFFER_SIZE;
    s_stream_config = (uvc_host_stream_config_t) {
        .event_cb = stream_event_callback,
        .frame_cb = frame_callback,
        .usb = {
            .dev_addr = dev_addr,
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
    free(modes);

    s_device_connected = true;
    s_stream_task_started = true;
    if (xTaskCreatePinnedToCore(stream_task, "uvc_stream", 4096,
                                &s_stream_config, USB_TASK_PRIORITY - 2, NULL,
                                tskNO_AFFINITY) != pdPASS) {
        ESP_LOGE(TAG, "cannot create stream task");
        s_device_connected = false;
        s_stream_task_started = false;
    }
}

static void usb_library_task(void *argument)
{
    (void)argument;
    while (true) {
        uint32_t flags = 0;
        ESP_ERROR_CHECK(usb_host_lib_handle_events(portMAX_DELAY, &flags));
        if ((flags & USB_HOST_LIB_EVENT_FLAGS_NO_CLIENTS) != 0) {
            usb_host_device_free_all();
        }
    }
}

void app_main(void)
{
    ESP_LOGI(TAG, "native UVC 2.x diagnostic starting");
    ESP_LOGI(TAG, "camera wiring: D-=GPIO19, D+=GPIO20, 5V and common GND");
    ESP_LOGI(TAG, "free internal RAM=%u, free PSRAM=%u",
             (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL),
             (unsigned)heap_caps_get_free_size(MALLOC_CAP_SPIRAM));
    if (heap_caps_get_total_size(MALLOC_CAP_SPIRAM) == 0) {
        ESP_LOGE(TAG, "PSRAM unavailable; camera buffers cannot be allocated");
        return;
    }

    ball_detector_tracker_init(&s_ball_tracker, 2, 24);
    ESP_ERROR_CHECK(camera_display_init());
    ESP_ERROR_CHECK(wifi_stream_start());
    s_frame_queue = xQueueCreate(1, sizeof(uvc_host_frame_t *));
    s_detector_frame_queue = xQueueCreate(1, sizeof(detector_frame_t));
    s_detector_result_queue = xQueueCreate(1, sizeof(detector_message_t));
    s_detector_buffer_available = xSemaphoreCreateBinary();
    assert(s_frame_queue != NULL && s_detector_frame_queue != NULL &&
           s_detector_result_queue != NULL && s_detector_buffer_available != NULL);
    xSemaphoreGive(s_detector_buffer_available);
    assert(xTaskCreatePinnedToCore(detector_task, "ball_detector", 6144,
                                   NULL, 5, NULL, 1) == pdPASS);
    assert(xTaskCreatePinnedToCore(frame_processing_task, "frame_display", 6144,
                                   NULL, USB_TASK_PRIORITY - 3, NULL,
                                   0) == pdPASS);

    const usb_host_config_t host_config = {
        .skip_phy_setup = false,
        .intr_flags = ESP_INTR_FLAG_LOWMED,
    };
    ESP_ERROR_CHECK(usb_host_install(&host_config));
    assert(xTaskCreatePinnedToCore(usb_library_task, "usb_events", 4096, NULL,
                                   USB_TASK_PRIORITY, NULL,
                                   tskNO_AFFINITY) == pdPASS);

    const uvc_host_driver_config_t driver_config = {
        .driver_task_stack_size = 4096,
        .driver_task_priority = USB_TASK_PRIORITY + 1,
        .xCoreID = tskNO_AFFINITY,
        .create_background_task = true,
        .event_cb = driver_event_callback,
    };
    ESP_ERROR_CHECK(uvc_host_install(&driver_config));
    ESP_LOGI(TAG, "waiting for the USB camera...");
}


