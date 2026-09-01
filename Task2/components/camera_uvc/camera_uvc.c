#include "camera_uvc.h"
#include "camera_stream_server.h"

#include <assert.h>
#include <inttypes.h>
#include <stddef.h>
#include <stdint.h>
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

#define USB_TASK_PRIORITY 15
#define FRAME_BUFFER_COUNT 4
#define MIN_MJPEG_BUFFER_SIZE (128U * 1024U)
#define JPEG_WORK_BUFFER_SIZE 4096U
#define VISION_FRAME_WIDTH 80U
#define VISION_FRAME_HEIGHT 60U
#define VISION_FRAME_PIXELS (VISION_FRAME_WIDTH * VISION_FRAME_HEIGHT)
#define FRAME_SIGNAL_TIMEOUT_MS 500U
#define STREAM_RESTART_DELAY_MS 200U
#define STREAM_OPEN_RETRY_MS 500U
#define CAMERA_STATS_PERIOD_MS 1000U
/* The installed UVC module is physically upside down. Keep this correction
 * aligned with the 180-degree CSS rotation in camera_stream_server.c. */
#define CAMERA_ROTATE_180 1

static const char *TAG = "camera_uvc";

static bool s_started;
static bool s_stream_task_started;
static volatile bool s_device_connected;
static uvc_host_stream_config_t s_stream_config;
static uvc_host_stream_hdl_t s_stream;
static QueueHandle_t s_frame_queue;
static volatile bool s_frame_in_processing;
static volatile TickType_t s_last_frame_tick;
static portMUX_TYPE s_sample_lock = portMUX_INITIALIZER_UNLOCKED;
static camera_vision_sample_t s_latest_sample;

typedef struct {
    uint32_t received_frames;
    uint32_t processed_frames;
    uint32_t dropped_frames;
    uint32_t decode_attempts;
    uint32_t decode_failures;
    uint32_t stream_stalls;
    uint64_t decode_total_us;
    uint32_t decode_max_us;
} camera_interval_stats_t;

static portMUX_TYPE s_stats_lock = portMUX_INITIALIZER_UNLOCKED;
static camera_interval_stats_t s_interval_stats;

static void stats_record_frame(bool dropped)
{
    portENTER_CRITICAL(&s_stats_lock);
    ++s_interval_stats.received_frames;
    if (dropped) {
        ++s_interval_stats.dropped_frames;
    }
    portEXIT_CRITICAL(&s_stats_lock);
}

static void stats_record_decode(uint32_t elapsed_us, bool failed,
                                bool processed)
{
    portENTER_CRITICAL(&s_stats_lock);
    ++s_interval_stats.decode_attempts;
    s_interval_stats.decode_total_us += elapsed_us;
    if (elapsed_us > s_interval_stats.decode_max_us) {
        s_interval_stats.decode_max_us = elapsed_us;
    }
    if (failed) {
        ++s_interval_stats.decode_failures;
    }
    if (processed) {
        ++s_interval_stats.processed_frames;
    }
    portEXIT_CRITICAL(&s_stats_lock);
}

static void stats_record_stall(void)
{
    portENTER_CRITICAL(&s_stats_lock);
    ++s_interval_stats.stream_stalls;
    portEXIT_CRITICAL(&s_stats_lock);
}

static camera_interval_stats_t stats_take_interval(void)
{
    camera_interval_stats_t stats;
    portENTER_CRITICAL(&s_stats_lock);
    stats = s_interval_stats;
    memset(&s_interval_stats, 0, sizeof(s_interval_stats));
    portEXIT_CRITICAL(&s_stats_lock);
    return stats;
}

static void rotate_rgb565_180(uint16_t *pixels, size_t width, size_t height)
{
#if CAMERA_ROTATE_180
    const size_t pixel_count = width * height;
    for (size_t front = 0, back = pixel_count - 1U;
         front < back; ++front, --back) {
        const uint16_t temporary = pixels[front];
        pixels[front] = pixels[back];
        pixels[back] = temporary;
    }
#else
    (void)pixels;
    (void)width;
    (void)height;
#endif
}

static float interval_to_fps(uint32_t interval)
{
    return interval == 0 ? 0.0f : 10000000.0f / (float)interval;
}

static uint32_t slowest_interval(const uvc_host_frame_info_t *info)
{
    uint32_t selected = info->default_interval;
    if (info->interval_type == 0) {
        if (info->interval_max > selected) {
            selected = info->interval_max;
        }
    } else {
        size_t count = info->interval_type;
        if (count > CONFIG_UVC_INTERVAL_ARRAY_SIZE) {
            count = CONFIG_UVC_INTERVAL_ARRAY_SIZE;
        }
        for (size_t i = 0; i < count; ++i) {
            if (info->interval[i] > selected) {
                selected = info->interval[i];
            }
        }
    }
    return selected;
}

static bool frame_callback(const uvc_host_frame_t *frame, void *user_ctx)
{
    (void)user_ctx;
    s_last_frame_tick = xTaskGetTickCount();
    if (frame->vs_format.format != UVC_VS_FORMAT_MJPEG) {
        return true;
    }

    const uvc_host_frame_t *queued_frame = frame;
    if (xQueueSendToBack(s_frame_queue, &queued_frame, 0) != pdPASS) {
        stats_record_frame(true);
        return true;
    }
    stats_record_frame(false);
    return false;
}

static esp_jpeg_image_scale_t select_decode_scale(
    const uvc_host_stream_format_t *format)
{
    (void)format;
    return JPEG_IMAGE_SCALE_1_8;
}

static void publish_sample(const line_vision_control_result_t *line,
                           int64_t timestamp_us)
{
    portENTER_CRITICAL(&s_sample_lock);
    s_latest_sample.frame_received = true;
    ++s_latest_sample.sequence;
    s_latest_sample.frame_timestamp_us = timestamp_us;
    s_latest_sample.line = *line;
    portEXIT_CRITICAL(&s_sample_lock);
}

bool camera_uvc_get_latest(camera_vision_sample_t *sample)
{
    if (sample == NULL) {
        return false;
    }
    portENTER_CRITICAL(&s_sample_lock);
    *sample = s_latest_sample;
    portEXIT_CRITICAL(&s_sample_lock);
    return sample->frame_received;
}

static void frame_processing_task(void *argument)
{
    (void)argument;
    uint16_t *framebuffer = heap_caps_malloc(
        VISION_FRAME_PIXELS * sizeof(uint16_t),
        MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    uint8_t *work_buffer = heap_caps_malloc(
        JPEG_WORK_BUFFER_SIZE, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    assert(framebuffer != NULL && work_buffer != NULL);

    line_vision_control_config_t vision_config;
    line_vision_control_default_config(&vision_config);
    line_vision_control_handle_t vision =
        line_vision_control_create(&vision_config);
    assert(vision != NULL);

    while (true) {
        uvc_host_frame_t *frame = NULL;
        if (xQueueReceive(s_frame_queue, &frame, portMAX_DELAY) != pdPASS) {
            continue;
        }
        s_frame_in_processing = true;

        /* This returns immediately without copying while PC transmission is
         * disabled, so normal visual control does not pay the Wi-Fi cost. */
        camera_stream_server_publish_jpeg(frame->data, frame->data_len);

        esp_jpeg_image_cfg_t jpeg_config = {
            .indata = frame->data,
            .indata_size = frame->data_len,
            .outbuf = (uint8_t *)framebuffer,
            .outbuf_size = VISION_FRAME_PIXELS * sizeof(uint16_t),
            .out_format = JPEG_IMAGE_FORMAT_RGB565,
            .out_scale = select_decode_scale(&frame->vs_format),
            .flags = {
                .swap_color_bytes = 1,
            },
            .advanced = {
                .working_buffer = work_buffer,
                .working_buffer_size = JPEG_WORK_BUFFER_SIZE,
            },
        };
        esp_jpeg_image_output_t output = {0};
        const int64_t decode_start_us = esp_timer_get_time();
        const esp_err_t decode_result = esp_jpeg_decode(&jpeg_config, &output);
        const int64_t decode_elapsed_us =
            esp_timer_get_time() - decode_start_us;
        bool processed = false;
        const bool decode_valid =
            decode_result == ESP_OK && output.width <= VISION_FRAME_WIDTH &&
            output.height <= VISION_FRAME_HEIGHT;
        if (decode_valid) {
            rotate_rgb565_180(framebuffer, output.width, output.height);
            line_vision_control_result_t line = {0};
            const int64_t now_us = esp_timer_get_time();
            if (line_vision_control_process_rgb565(
                    vision, framebuffer, output.width, output.height,
                    now_us, &line)) {
                camera_stream_server_publish_virtual_ir(
                    line.virtual_ir_raw_pattern, line.virtual_ir_pattern,
                    line.virtual_ir_black_percent, line.virtual_ir_error);
                publish_sample(&line, now_us);
                processed = true;
            }
        } else {
            ESP_LOGW(TAG, "JPEG decode failed: %s, %ux%u",
                     esp_err_to_name(decode_result), output.width,
                     output.height);
        }
        stats_record_decode((uint32_t)decode_elapsed_us,
                            !decode_valid, processed);

        if (s_stream != NULL) {
            ESP_ERROR_CHECK_WITHOUT_ABORT(
                uvc_host_frame_return(s_stream, frame));
        }
        s_frame_in_processing = false;
        vTaskDelay(1);
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
        break;
    case UVC_HOST_FRAME_BUFFER_OVERFLOW:
        ESP_LOGW(TAG, "frame buffer overflow");
        break;
    case UVC_HOST_FRAME_BUFFER_UNDERFLOW:
        ESP_LOGW(TAG, "no free UVC frame buffer");
        break;
    default:
        break;
    }
}

static void wait_for_owned_frames(void)
{
    /* The processing task owns every frame in the queue. Closing the stream
     * before it returns them makes uvc_host_stream_close() fail and can also
     * break the following reconnect attempt. */
    while (s_frame_in_processing ||
           uxQueueMessagesWaiting(s_frame_queue) > 0U) {
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

static void stream_task(void *argument)
{
    const uvc_host_stream_config_t *config = argument;
    TickType_t last_stats_tick = xTaskGetTickCount();
    while (s_device_connected) {
        s_stream = NULL;
        esp_err_t result = uvc_host_stream_open(
            config, pdMS_TO_TICKS(5000), &s_stream);
        if (result != ESP_OK) {
            ESP_LOGE(TAG, "stream open failed: %s; retrying",
                     esp_err_to_name(result));
            vTaskDelay(pdMS_TO_TICKS(STREAM_OPEN_RETRY_MS));
            continue;
        }

        uvc_host_stream_format_t selected = {0};
        ESP_ERROR_CHECK_WITHOUT_ABORT(
            uvc_host_stream_format_get(s_stream, &selected));
        ESP_LOGI(TAG, "camera mode: %ux%u @ %.1f fps",
                 selected.h_res, selected.v_res, selected.fps);
        result = uvc_host_stream_start(s_stream);
        if (result == ESP_OK) {
            s_last_frame_tick = xTaskGetTickCount();
            while (s_device_connected) {
                vTaskDelay(pdMS_TO_TICKS(50));
                const TickType_t now_tick = xTaskGetTickCount();
                const TickType_t silent_ticks = now_tick - s_last_frame_tick;
                const TickType_t stats_ticks = now_tick - last_stats_tick;
                if (stats_ticks >= pdMS_TO_TICKS(CAMERA_STATS_PERIOD_MS)) {
                    const camera_interval_stats_t stats =
                        stats_take_interval();
                    const float elapsed_s =
                        (float)stats_ticks / (float)configTICK_RATE_HZ;
                    const float decode_average_ms =
                        stats.decode_attempts == 0U
                            ? 0.0f
                            : (float)stats.decode_total_us /
                                  (1000.0f * stats.decode_attempts);
                    ESP_LOGI(TAG,
                             "camera stats: rx=%.1f fps processed=%.1f fps "
                             "dropped=%.1f fps decode_avg=%.1f ms "
                             "decode_max=%.1f ms failures=%" PRIu32
                             " stalls=%" PRIu32,
                             stats.received_frames / elapsed_s,
                             stats.processed_frames / elapsed_s,
                             stats.dropped_frames / elapsed_s,
                             decode_average_ms,
                             stats.decode_max_us / 1000.0f,
                             stats.decode_failures, stats.stream_stalls);
                    last_stats_tick = now_tick;
                }
                if (silent_ticks >=
                    pdMS_TO_TICKS(FRAME_SIGNAL_TIMEOUT_MS)) {
                    stats_record_stall();
                    ESP_LOGW(TAG,
                             "no camera frames for %u ms; restarting stream",
                             FRAME_SIGNAL_TIMEOUT_MS);
                    break;
                }
            }
        } else {
            ESP_LOGE(TAG, "stream start failed: %s; reopening",
                     esp_err_to_name(result));
        }

        if (s_device_connected && result == ESP_OK) {
            const esp_err_t stop_result = uvc_host_stream_stop(s_stream);
            if (stop_result != ESP_OK) {
                ESP_LOGW(TAG, "stream stop before restart failed: %s",
                         esp_err_to_name(stop_result));
            }
        }
        wait_for_owned_frames();
        result = uvc_host_stream_close(s_stream);
        if (result != ESP_OK) {
            ESP_LOGE(TAG, "stream close failed: %s",
                     esp_err_to_name(result));
        }
        s_stream = NULL;

        if (s_device_connected) {
            ESP_LOGI(TAG, "camera stream closed; reconnecting");
            vTaskDelay(pdMS_TO_TICKS(STREAM_RESTART_DELAY_MS));
        }
    }

    ESP_LOGI(TAG, "camera stream closed; ready for USB reconnect");
    s_stream_task_started = false;
    vTaskDelete(NULL);
}

static void driver_event_callback(const uvc_host_driver_event_data_t *event,
                                  void *user_ctx)
{
    (void)user_ctx;
    if (event->type != UVC_HOST_DRIVER_EVENT_DEVICE_CONNECTED ||
        s_stream_task_started) {
        return;
    }

    const uint8_t dev_addr = event->device_connected.dev_addr;
    const uint8_t stream_index = event->device_connected.uvc_stream_index;
    size_t count = event->device_connected.frame_info_num;
    ESP_LOGI(TAG, "UVC camera enumerated: address=%u stream=%u modes=%u",
             (unsigned)dev_addr, (unsigned)stream_index, (unsigned)count);
    if (count == 0) {
        ESP_LOGE(TAG, "camera has no video frame descriptors");
        return;
    }

    uvc_host_frame_info_t *modes = calloc(count, sizeof(*modes));
    if (modes == NULL) {
        ESP_LOGE(TAG, "cannot allocate camera mode list");
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
        const uint64_t pixels =
            (uint64_t)modes[i].h_res * modes[i].v_res;
        if (modes[i].format == UVC_VS_FORMAT_MJPEG && pixels < best_pixels) {
            best = i;
            best_pixels = pixels;
        }
    }
    if (best == SIZE_MAX) {
        ESP_LOGE(TAG, "camera advertises no MJPEG mode");
        free(modes);
        return;
    }

    const uint32_t interval = slowest_interval(&modes[best]);
    size_t frame_size =
        (size_t)modes[best].h_res * modes[best].v_res * 2U / 6U;
    if (frame_size < MIN_MJPEG_BUFFER_SIZE) {
        frame_size = MIN_MJPEG_BUFFER_SIZE;
    }
    s_stream_config = (uvc_host_stream_config_t){
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
                                &s_stream_config, USB_TASK_PRIORITY - 2,
                                NULL, tskNO_AFFINITY) != pdPASS) {
        s_device_connected = false;
        s_stream_task_started = false;
        ESP_LOGE(TAG, "cannot create UVC stream task");
    }
}

static void usb_library_task(void *argument)
{
    (void)argument;
    int previous_device_count = -1;
    while (true) {
        uint32_t flags = 0;
        const esp_err_t event_result =
            usb_host_lib_handle_events(pdMS_TO_TICKS(500), &flags);
        if (event_result != ESP_OK && event_result != ESP_ERR_TIMEOUT) {
            ESP_LOGE(TAG, "USB library event handling failed: %s",
                     esp_err_to_name(event_result));
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }
        if ((flags & USB_HOST_LIB_EVENT_FLAGS_NO_CLIENTS) != 0) {
            usb_host_device_free_all();
        }

        uint8_t addresses[4] = {0};
        int device_count = 0;
        const esp_err_t list_result = usb_host_device_addr_list_fill(
            (int)(sizeof(addresses) / sizeof(addresses[0])), addresses,
            &device_count);
        if (list_result == ESP_OK && device_count != previous_device_count) {
            previous_device_count = device_count;
            if (device_count == 0) {
                ESP_LOGI(TAG, "USB host device count: 0");
            } else {
                ESP_LOGI(TAG, "USB host enumerated %d device(s); first address=%u",
                         device_count, (unsigned)addresses[0]);
            }
        }
    }
}

esp_err_t camera_uvc_start(void)
{
    if (s_started) {
        return ESP_ERR_INVALID_STATE;
    }
    if (heap_caps_get_total_size(MALLOC_CAP_SPIRAM) == 0) {
        ESP_LOGE(TAG, "PSRAM unavailable");
        return ESP_ERR_NO_MEM;
    }

    /* Keep only one pending frame. A longer queue retains every UVC buffer
     * while JPEG decode is running and starves the USB receiver. New frames
     * are safely returned by frame_callback() when this queue is full. */
    s_frame_queue = xQueueCreate(1, sizeof(uvc_host_frame_t *));
    if (s_frame_queue == NULL) {
        return ESP_ERR_NO_MEM;
    }
    if (xTaskCreatePinnedToCore(frame_processing_task, "camera_vision", 6144,
                                NULL, USB_TASK_PRIORITY - 3, NULL,
                                tskNO_AFFINITY) != pdPASS) {
        return ESP_ERR_NO_MEM;
    }

    const usb_host_config_t host_config = {
        .skip_phy_setup = false,
        .intr_flags = ESP_INTR_FLAG_LOWMED,
    };
    esp_log_level_set("USB HOST", ESP_LOG_DEBUG);
    esp_log_level_set("HUB", ESP_LOG_DEBUG);
    esp_log_level_set("ENUM", ESP_LOG_DEBUG);
    ESP_RETURN_ON_ERROR(usb_host_install(&host_config), TAG,
                        "install USB host failed");
    ESP_LOGI(TAG, "native USB Host PHY active: GPIO19=D-, GPIO20=D+");
    if (xTaskCreatePinnedToCore(usb_library_task, "usb_events", 4096,
                                NULL, USB_TASK_PRIORITY, NULL,
                                tskNO_AFFINITY) != pdPASS) {
        return ESP_ERR_NO_MEM;
    }

    const uvc_host_driver_config_t driver_config = {
        .driver_task_stack_size = 4096,
        .driver_task_priority = USB_TASK_PRIORITY + 1,
        .xCoreID = tskNO_AFFINITY,
        .create_background_task = true,
        .event_cb = driver_event_callback,
    };
    ESP_RETURN_ON_ERROR(uvc_host_install(&driver_config), TAG,
                        "install UVC host failed");
    s_started = true;
    ESP_LOGI(TAG, "waiting for USB camera on GPIO19/20");
    return ESP_OK;
}
