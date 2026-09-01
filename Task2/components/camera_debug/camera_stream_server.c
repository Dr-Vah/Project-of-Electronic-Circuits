#include "camera_stream_server.h"

#include <inttypes.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "esp_event.h"
#include "esp_heap_caps.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "nvs_flash.h"

#define CAMERA_AP_SSID "ESP32-Camera-Debug"
#define CAMERA_AP_PASSWORD "camera123"
#define CAMERA_AP_CHANNEL 6
#define CAMERA_AP_MAX_CLIENTS 2
#define STREAM_BOUNDARY "esp32camera"

static const char *TAG = "camera_web";
static SemaphoreHandle_t s_frame_mutex;
static uint8_t *s_latest_jpeg;
static size_t s_latest_length;
static size_t s_latest_capacity;
static uint32_t s_frame_sequence;
static bool s_stream_enabled;
static uint8_t s_virtual_ir_raw_pattern;
static uint8_t s_virtual_ir_pattern;
static uint8_t s_virtual_ir_black_percent[4];
static float s_virtual_ir_error;

static const char s_index_html[] =
    "<!doctype html><html lang=zh-CN><head><meta charset=utf-8>"
    "<meta name=viewport content='width=device-width,initial-scale=1'>"
    "<title>ESP32 摄像头调试</title><style>"
    "body{margin:0;background:#101318;color:#eef;font:16px system-ui;text-align:center}"
    "main{max-width:900px;margin:auto;padding:18px}h1{font-size:22px}"
    "img{width:100%;height:auto;image-rendering:auto;background:#000;border-radius:8px;transform:rotate(180deg)}"
    "button{font-size:16px;padding:9px 16px;margin:6px;border:0;border-radius:6px}"
    "#on{background:#2b8;color:white}#off{background:#d55;color:white}"
    "#s{color:#9bd;margin:10px}</style></head><body><main>"
    "<h1>ESP32 摄像头调试</h1><div id=s>读取状态…</div><div id=ir></div>"
    "<button id=on onclick='setVideo(1)'>开启视频回传</button>"
    "<button id=off onclick='setVideo(0)'>关闭视频回传</button>"
    "<img src=/stream alt='camera stream'>"
    "<script>async function setVideo(v){await fetch('/control?enabled='+v,{cache:'no-store'});"
    "await update()}async function update(){try{let r=await fetch('/status',{cache:'no-store'});"
    "let j=await r.json();document.querySelector('#s').textContent=j.enabled?"
    "`视频回传已开启 · ${j.frames} 帧 · JPEG ${j.bytes} 字节`:"
    "'视频回传已关闭（巡线仍在运行）';"
    "document.querySelector('#ir').textContent="
    "`虚拟红外 raw=0x${j.ir_raw.toString(16)} filtered=0x${j.ir_pattern.toString(16)} "
    "· 黑色占比 [${j.ir_percent.join('%, ')}%] · error=${j.ir_error.toFixed(1)}`}"
    "catch(e){}}setInterval(update,500);update()</script>"
    "</main></body></html>";

static esp_err_t index_handler(httpd_req_t *request)
{
    httpd_resp_set_type(request, "text/html; charset=utf-8");
    httpd_resp_set_hdr(request, "Cache-Control", "no-store");
    return httpd_resp_send(request, s_index_html, HTTPD_RESP_USE_STRLEN);
}

static esp_err_t status_handler(httpd_req_t *request)
{
    uint32_t sequence;
    size_t length;
    bool enabled;
    uint8_t ir_raw;
    uint8_t ir_pattern;
    uint8_t ir_percent[4];
    float ir_error;
    xSemaphoreTake(s_frame_mutex, portMAX_DELAY);
    sequence = s_frame_sequence;
    length = s_latest_length;
    enabled = s_stream_enabled;
    ir_raw = s_virtual_ir_raw_pattern;
    ir_pattern = s_virtual_ir_pattern;
    memcpy(ir_percent, s_virtual_ir_black_percent, sizeof(ir_percent));
    ir_error = s_virtual_ir_error;
    xSemaphoreGive(s_frame_mutex);

    char json[224];
    const int written = snprintf(json, sizeof(json),
                                 "{\"enabled\":%s,\"frames\":%" PRIu32
                                 ",\"bytes\":%u,\"ir_raw\":%u,"
                                 "\"ir_pattern\":%u,\"ir_percent\":[%u,%u,%u,%u],"
                                 "\"ir_error\":%.2f}",
                                 enabled ? "true" : "false", sequence,
                                 (unsigned)length, ir_raw, ir_pattern,
                                 ir_percent[0], ir_percent[1], ir_percent[2],
                                 ir_percent[3], ir_error);
    httpd_resp_set_type(request, "application/json");
    httpd_resp_set_hdr(request, "Cache-Control", "no-store");
    return httpd_resp_send(request, json, written);
}

static esp_err_t control_handler(httpd_req_t *request)
{
    char query[32] = {0};
    char value[8] = {0};
    if (httpd_req_get_url_query_str(request, query, sizeof(query)) != ESP_OK ||
        httpd_query_key_value(query, "enabled", value, sizeof(value)) != ESP_OK ||
        (strcmp(value, "0") != 0 && strcmp(value, "1") != 0)) {
        return httpd_resp_send_err(request, HTTPD_400_BAD_REQUEST,
                                   "enabled must be 0 or 1");
    }

    camera_stream_server_set_enabled(strcmp(value, "1") == 0);
    httpd_resp_set_type(request, "application/json");
    httpd_resp_set_hdr(request, "Cache-Control", "no-store");
    return httpd_resp_sendstr(request,
        camera_stream_server_is_enabled() ? "{\"enabled\":true}" :
                                            "{\"enabled\":false}");
}

static esp_err_t stream_handler(httpd_req_t *request)
{
    httpd_resp_set_type(request,
        "multipart/x-mixed-replace;boundary=" STREAM_BOUNDARY);
    httpd_resp_set_hdr(request, "Cache-Control", "no-store, no-cache");
    httpd_resp_set_hdr(request, "Access-Control-Allow-Origin", "*");

    uint8_t *copy = NULL;
    size_t copy_capacity = 0;
    uint32_t last_sequence = 0;
    esp_err_t result = ESP_OK;

    while (result == ESP_OK) {
        size_t required;
        uint32_t sequence;
        bool enabled;
        xSemaphoreTake(s_frame_mutex, portMAX_DELAY);
        required = s_latest_length;
        sequence = s_frame_sequence;
        enabled = s_stream_enabled;
        xSemaphoreGive(s_frame_mutex);

        if (!enabled || required == 0 || sequence == last_sequence) {
            vTaskDelay(pdMS_TO_TICKS(20));
            continue;
        }
        if (required > copy_capacity) {
            uint8_t *larger = heap_caps_malloc(required,
                                                MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
            if (larger == NULL) {
                ESP_LOGE(TAG, "cannot allocate %u-byte HTTP frame copy",
                         (unsigned)required);
                result = ESP_ERR_NO_MEM;
                break;
            }
            free(copy);
            copy = larger;
            copy_capacity = required;
        }

        size_t length = 0;
        xSemaphoreTake(s_frame_mutex, portMAX_DELAY);
        if (s_latest_length <= copy_capacity && s_frame_sequence != last_sequence) {
            length = s_latest_length;
            sequence = s_frame_sequence;
            memcpy(copy, s_latest_jpeg, length);
        }
        xSemaphoreGive(s_frame_mutex);
        if (length == 0) continue;

        char header[128];
        const int header_length = snprintf(
            header, sizeof(header),
            "--" STREAM_BOUNDARY "\r\nContent-Type: image/jpeg\r\n"
            "Content-Length: %u\r\n\r\n", (unsigned)length);
        result = httpd_resp_send_chunk(request, header, header_length);
        if (result == ESP_OK) result = httpd_resp_send_chunk(request,
                                                              (const char *)copy,
                                                              length);
        if (result == ESP_OK) result = httpd_resp_send_chunk(request, "\r\n", 2);
        last_sequence = sequence;
    }

    free(copy);
    ESP_LOGI(TAG, "browser stream disconnected");
    return result;
}

static esp_err_t start_http_server(void)
{
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.stack_size = 6144;
    config.max_uri_handlers = 4;
    config.lru_purge_enable = true;

    httpd_handle_t server = NULL;
    esp_err_t result = httpd_start(&server, &config);
    if (result != ESP_OK) return result;

    const httpd_uri_t index_uri = {
        .uri = "/", .method = HTTP_GET, .handler = index_handler,
    };
    const httpd_uri_t stream_uri = {
        .uri = "/stream", .method = HTTP_GET, .handler = stream_handler,
    };
    const httpd_uri_t status_uri = {
        .uri = "/status", .method = HTTP_GET, .handler = status_handler,
    };
    const httpd_uri_t control_uri = {
        .uri = "/control", .method = HTTP_GET, .handler = control_handler,
    };
    if ((result = httpd_register_uri_handler(server, &index_uri)) != ESP_OK ||
        (result = httpd_register_uri_handler(server, &stream_uri)) != ESP_OK ||
        (result = httpd_register_uri_handler(server, &status_uri)) != ESP_OK ||
        (result = httpd_register_uri_handler(server, &control_uri)) != ESP_OK) {
        httpd_stop(server);
        return result;
    }
    return ESP_OK;
}

esp_err_t camera_stream_server_init(bool enabled_by_default)
{
    s_frame_mutex = xSemaphoreCreateMutex();
    if (s_frame_mutex == NULL) return ESP_ERR_NO_MEM;
    s_stream_enabled = enabled_by_default;

    esp_err_t result = nvs_flash_init();
    if (result == ESP_ERR_NVS_NO_FREE_PAGES ||
        result == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        result = nvs_flash_init();
    }
    if (result != ESP_OK) return result;
    if ((result = esp_netif_init()) != ESP_OK) return result;
    if ((result = esp_event_loop_create_default()) != ESP_OK) return result;
    if (esp_netif_create_default_wifi_ap() == NULL) return ESP_FAIL;

    wifi_init_config_t wifi_init = WIFI_INIT_CONFIG_DEFAULT();
    if ((result = esp_wifi_init(&wifi_init)) != ESP_OK) return result;

    wifi_config_t wifi_config = {0};
    memcpy(wifi_config.ap.ssid, CAMERA_AP_SSID, sizeof(CAMERA_AP_SSID));
    memcpy(wifi_config.ap.password, CAMERA_AP_PASSWORD,
           sizeof(CAMERA_AP_PASSWORD));
    wifi_config.ap.ssid_len = strlen(CAMERA_AP_SSID);
    wifi_config.ap.channel = CAMERA_AP_CHANNEL;
    wifi_config.ap.max_connection = CAMERA_AP_MAX_CLIENTS;
    wifi_config.ap.authmode = WIFI_AUTH_WPA2_PSK;
    wifi_config.ap.pmf_cfg.required = true;

    if ((result = esp_wifi_set_mode(WIFI_MODE_AP)) != ESP_OK) return result;
    if ((result = esp_wifi_set_config(WIFI_IF_AP, &wifi_config)) != ESP_OK) return result;
    if ((result = esp_wifi_start()) != ESP_OK) return result;
    if ((result = start_http_server()) != ESP_OK) return result;

    ESP_LOGI(TAG, "PC preview ready: connect Wi-Fi '%s' (password '%s')",
             CAMERA_AP_SSID, CAMERA_AP_PASSWORD);
    ESP_LOGI(TAG, "then open http://192.168.4.1/ in a browser");
    ESP_LOGI(TAG, "JPEG transmission is %s",
             enabled_by_default ? "enabled" : "disabled");
    return ESP_OK;
}

void camera_stream_server_set_enabled(bool enabled)
{
    if (s_frame_mutex == NULL) return;
    xSemaphoreTake(s_frame_mutex, portMAX_DELAY);
    if (s_stream_enabled != enabled) {
        s_stream_enabled = enabled;
        s_latest_length = 0;
        ++s_frame_sequence;
    }
    xSemaphoreGive(s_frame_mutex);
    ESP_LOGI(TAG, "JPEG transmission %s", enabled ? "enabled" : "disabled");
}

bool camera_stream_server_is_enabled(void)
{
    if (s_frame_mutex == NULL) return false;
    xSemaphoreTake(s_frame_mutex, portMAX_DELAY);
    const bool enabled = s_stream_enabled;
    xSemaphoreGive(s_frame_mutex);
    return enabled;
}

void camera_stream_server_publish_jpeg(const uint8_t *jpeg, size_t length)
{
    if (s_frame_mutex == NULL || jpeg == NULL || length == 0) return;

    xSemaphoreTake(s_frame_mutex, portMAX_DELAY);
    const bool enabled = s_stream_enabled;
    xSemaphoreGive(s_frame_mutex);
    if (!enabled) return;

    if (length > s_latest_capacity) {
        uint8_t *larger = heap_caps_malloc(length,
                                            MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
        if (larger == NULL) {
            ESP_LOGW(TAG, "dropping oversized %u-byte JPEG", (unsigned)length);
            return;
        }
        xSemaphoreTake(s_frame_mutex, portMAX_DELAY);
        free(s_latest_jpeg);
        s_latest_jpeg = larger;
        s_latest_capacity = length;
        xSemaphoreGive(s_frame_mutex);
    }

    xSemaphoreTake(s_frame_mutex, portMAX_DELAY);
    if (s_stream_enabled) {
        memcpy(s_latest_jpeg, jpeg, length);
        s_latest_length = length;
        ++s_frame_sequence;
    }
    xSemaphoreGive(s_frame_mutex);
}

void camera_stream_server_publish_virtual_ir(
    uint8_t raw_pattern, uint8_t filtered_pattern,
    const uint8_t black_percent[4], float error)
{
    if (s_frame_mutex == NULL || black_percent == NULL) return;
    xSemaphoreTake(s_frame_mutex, portMAX_DELAY);
    s_virtual_ir_raw_pattern = raw_pattern;
    s_virtual_ir_pattern = filtered_pattern;
    memcpy(s_virtual_ir_black_percent, black_percent,
           sizeof(s_virtual_ir_black_percent));
    s_virtual_ir_error = error;
    xSemaphoreGive(s_frame_mutex);
}
