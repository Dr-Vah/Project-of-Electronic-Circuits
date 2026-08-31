/* USB UVC diagnostic for ESP32-S3 and a composite camera/audio device. */

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

#include "camera_display.h"

static const char *TAG = "camera_uvc_test";

#define USB_TASK_PRIORITY 15
#define FRAME_BUFFER_COUNT 3
#define MIN_MJPEG_BUFFER_SIZE (40U * 1024U)
#define JPEG_WORK_BUFFER_SIZE 4096U

static bool s_stream_task_started;
static volatile bool s_device_connected;
static uvc_host_stream_config_t s_stream_config;
static uvc_host_stream_hdl_t s_stream;
static QueueHandle_t s_frame_queue;

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

static void frame_processing_task(void *argument)
{
    (void)argument;
    uint16_t *framebuffer = heap_caps_malloc(
        CAMERA_DISPLAY_WIDTH * CAMERA_DISPLAY_HEIGHT * sizeof(uint16_t),
        MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    uint8_t *work_buffer = heap_caps_malloc(JPEG_WORK_BUFFER_SIZE,
                                            MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    assert(framebuffer != NULL && work_buffer != NULL);

    int64_t statistics_start = esp_timer_get_time();
    uint32_t received = 0;
    uint32_t displayed = 0;
    uint32_t decode_failed = 0;
    while (true) {
        uvc_host_frame_t *frame = NULL;
        if (xQueueReceive(s_frame_queue, &frame, portMAX_DELAY) != pdPASS) continue;
        ++received;

        esp_jpeg_image_cfg_t jpeg_config = {
            .indata = frame->data,
            .indata_size = frame->data_len,
            .outbuf = (uint8_t *)framebuffer,
            .outbuf_size = CAMERA_DISPLAY_WIDTH * CAMERA_DISPLAY_HEIGHT *
                           sizeof(uint16_t),
            .out_format = JPEG_IMAGE_FORMAT_RGB565,
            .out_scale = JPEG_IMAGE_SCALE_1_4,
            .flags = {
                .swap_color_bytes = 1,
            },
            .advanced = {
                .working_buffer = work_buffer,
                .working_buffer_size = JPEG_WORK_BUFFER_SIZE,
            },
        };
        esp_jpeg_image_output_t output = {0};
        esp_err_t result = esp_jpeg_decode(&jpeg_config, &output);
        if (result == ESP_OK && output.width <= CAMERA_DISPLAY_WIDTH &&
            output.height <= CAMERA_DISPLAY_HEIGHT) {
            result = camera_display_draw_rgb565(framebuffer, output.width,
                                                 output.height);
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
    uint64_t best_pixels = UINT64_MAX;
    for (size_t i = 0; i < count; ++i) {
        print_frame_info(&modes[i], i);
        const uint64_t pixels = (uint64_t)modes[i].h_res * modes[i].v_res;
        if (modes[i].format == UVC_VS_FORMAT_MJPEG && pixels < best_pixels) {
            best = i;
            best_pixels = pixels;
        }
    }
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

    ESP_ERROR_CHECK(camera_display_init());
    s_frame_queue = xQueueCreate(1, sizeof(uvc_host_frame_t *));
    assert(s_frame_queue != NULL);
    assert(xTaskCreatePinnedToCore(frame_processing_task, "frame_display", 6144,
                                   NULL, USB_TASK_PRIORITY - 3, NULL,
                                   tskNO_AFFINITY) == pdPASS);

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
