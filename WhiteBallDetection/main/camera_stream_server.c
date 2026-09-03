#include "camera_stream_server.h"

#include <inttypes.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "chassis_camera_geometry.h"
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
static bool s_detection_valid;
static float s_detection_left;
static float s_detection_top;
static float s_detection_right;
static float s_detection_bottom;
static float s_detection_confidence;
static bool s_target_valid;
static float s_target_left;
static float s_target_top;
static float s_target_right;
static float s_target_bottom;

static const char s_index_html[] =
    "<!doctype html><html lang=zh-CN><head><meta charset=utf-8>"
    "<meta name=viewport content='width=device-width,initial-scale=1'>"
    "<title>ESP32 白球识别调试</title><style>"
    "body{margin:0;background:#101318;color:#eef;font:16px system-ui;text-align:center}"
    "main{max-width:900px;margin:auto;padding:18px}h1{font-size:22px}"
    "#v{position:relative;width:100%}img{display:block;width:100%;height:auto;"
    "background:#000;border-radius:8px}.box{display:none;position:absolute;"
    "box-sizing:border-box;border-radius:4px;box-shadow:0 0 5px #000;"
    "pointer-events:none}#ball{border:3px solid #00ff55}"
    "#target{border:3px solid #ff2bd6}.guide{position:absolute;top:0;bottom:0;"
    "width:2px;pointer-events:none}.side{background:#ffe600}.axis{background:#f22;"
    "width:3px}"
    "#s{color:#9bd;margin:10px}</style></head><body><main>"
    "<h1>ESP32 摄像头实时回传</h1><div id=s>等待视频帧…</div>"
    "<div id=v><img id=cam src=/stream alt='camera stream'>"
    "<div id=gleft class='guide side'></div><div id=axis class='guide axis'></div>"
    "<div id=gright class='guide side'></div><div id=ball class=box></div>"
    "<div id=target class=box></div></div>"
    "<script>function box(e,on,l,t,r,b){if(!on){e.style.display='none';return;}"
    "e.style.display='block';e.style.left=(100*l)+'%';e.style.top=(100*t)+'%';"
    "e.style.width=(100*(r-l))+'%';e.style.height=(100*(b-t))+'%';}"
    "function poll(){var x=new XMLHttpRequest();x.onreadystatechange=function(){"
    "if(x.readyState!==4||x.status!==200)return;try{var j=JSON.parse(x.responseText);"
    "document.getElementById('s').textContent='已接收 '+j.frames+' 帧 · JPEG '+"
    "j.bytes+' 字节 · '+(j.detected?'检测到白球':'未检测到白球')+' · '+"
    "(j.targetDetected?'检测到停止区':'未检测到停止区');"
    "document.getElementById('gleft').style.left=(100*j.guideLeft)+'%';"
    "document.getElementById('axis').style.left=(100*j.pushAxis)+'%';"
    "document.getElementById('gright').style.left=(100*j.guideRight)+'%';"
    "box(document.getElementById('ball'),j.detected,j.left,j.top,j.right,j.bottom);"
    "box(document.getElementById('target'),j.targetDetected,j.targetLeft,j.targetTop,"
    "j.targetRight,j.targetBottom);}catch(e){}};x.open('GET','/status?t='+Date.now(),true);"
    "x.send();}setInterval(poll,200);poll();</script>"
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
    bool detected;
    float left;
    float top;
    float right;
    float bottom;
    float confidence;
    bool target_detected;
    float target_left;
    float target_top;
    float target_right;
    float target_bottom;
    xSemaphoreTake(s_frame_mutex, portMAX_DELAY);
    sequence = s_frame_sequence;
    length = s_latest_length;
    detected = s_detection_valid;
    left = s_detection_left;
    top = s_detection_top;
    right = s_detection_right;
    bottom = s_detection_bottom;
    confidence = s_detection_confidence;
    target_detected = s_target_valid;
    target_left = s_target_left;
    target_top = s_target_top;
    target_right = s_target_right;
    target_bottom = s_target_bottom;
    xSemaphoreGive(s_frame_mutex);

    char json[384];
    const int written = snprintf(json, sizeof(json),
        "{\"frames\":%" PRIu32 ",\"bytes\":%u,\"detected\":%s,"
        "\"left\":%.4f,\"top\":%.4f,\"right\":%.4f,"
        "\"bottom\":%.4f,\"confidence\":%.3f,"
        "\"targetDetected\":%s,\"targetLeft\":%.4f,"
        "\"targetTop\":%.4f,\"targetRight\":%.4f,"
        "\"targetBottom\":%.4f,\"guideLeft\":%.3f,"
        "\"pushAxis\":%.3f,\"guideRight\":%.3f}",
        sequence, (unsigned)length, detected ? "true" : "false",
        left, top, right, bottom, confidence,
        target_detected ? "true" : "false", target_left, target_top,
        target_right, target_bottom, CHASSIS_RAW_CENTER_LEFT_FRACTION,
        CHASSIS_RAW_PUSH_AXIS_FRACTION,
        CHASSIS_RAW_CENTER_RIGHT_FRACTION);
    httpd_resp_set_type(request, "application/json");
    httpd_resp_set_hdr(request, "Cache-Control", "no-store");
    return httpd_resp_send(request, json, written);
}

static esp_err_t stream_handler(httpd_req_t *request)
{
    httpd_resp_set_type(request,
        "multipart/x-mixed-replace;boundary=" STREAM_BOUNDARY);
    httpd_resp_set_hdr(request, "Cache-Control", "no-store, no-cache");
    httpd_resp_set_hdr(request, "Access-Control-Allow-Origin", "*");

    uint8_t *copy = NULL;
    size_t copy_capacity = 0U;
    uint32_t last_sequence = 0U;
    esp_err_t result = ESP_OK;

    while (result == ESP_OK) {
        size_t required;
        uint32_t sequence;
        xSemaphoreTake(s_frame_mutex, portMAX_DELAY);
        required = s_latest_length;
        sequence = s_frame_sequence;
        xSemaphoreGive(s_frame_mutex);

        if (required == 0U || sequence == last_sequence) {
            vTaskDelay(pdMS_TO_TICKS(20));
            continue;
        }
        if (required > copy_capacity) {
            uint8_t *larger = heap_caps_malloc(
                required, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
            if (larger == NULL) {
                result = ESP_ERR_NO_MEM;
                break;
            }
            free(copy);
            copy = larger;
            copy_capacity = required;
        }

        size_t length = 0U;
        xSemaphoreTake(s_frame_mutex, portMAX_DELAY);
        if (s_latest_length <= copy_capacity &&
            s_frame_sequence != last_sequence) {
            length = s_latest_length;
            sequence = s_frame_sequence;
            memcpy(copy, s_latest_jpeg, length);
        }
        xSemaphoreGive(s_frame_mutex);
        if (length == 0U) continue;

        char header[128];
        const int header_length = snprintf(
            header, sizeof(header),
            "--" STREAM_BOUNDARY "\r\nContent-Type: image/jpeg\r\n"
            "Content-Length: %u\r\n\r\n", (unsigned)length);
        result = httpd_resp_send_chunk(request, header, header_length);
        if (result == ESP_OK) {
            result = httpd_resp_send_chunk(request, (const char *)copy,
                                           length);
        }
        if (result == ESP_OK) {
            result = httpd_resp_send_chunk(request, "\r\n", 2U);
        }
        last_sequence = sequence;
    }

    free(copy);
    ESP_LOGI(TAG, "browser stream disconnected");
    return result;
}

static esp_err_t start_http_server(void)
{
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.stack_size = 6144U;
    config.max_uri_handlers = 3U;
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
    if ((result = httpd_register_uri_handler(server, &index_uri)) != ESP_OK ||
        (result = httpd_register_uri_handler(server, &stream_uri)) != ESP_OK ||
        (result = httpd_register_uri_handler(server, &status_uri)) != ESP_OK) {
        httpd_stop(server);
        return result;
    }
    return ESP_OK;
}

esp_err_t camera_stream_server_init(void)
{
    s_frame_mutex = xSemaphoreCreateMutex();
    if (s_frame_mutex == NULL) return ESP_ERR_NO_MEM;

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
    if ((result = esp_wifi_set_config(WIFI_IF_AP, &wifi_config)) != ESP_OK) {
        return result;
    }
    if ((result = esp_wifi_start()) != ESP_OK) return result;
    if ((result = start_http_server()) != ESP_OK) return result;

    ESP_LOGI(TAG, "connect Wi-Fi '%s', password '%s'",
             CAMERA_AP_SSID, CAMERA_AP_PASSWORD);
    ESP_LOGI(TAG, "open http://192.168.4.1/ for camera preview");
    return ESP_OK;
}

void camera_stream_server_publish_jpeg(const uint8_t *jpeg, size_t length)
{
    if (s_frame_mutex == NULL || jpeg == NULL || length == 0U) return;

    if (length > s_latest_capacity) {
        uint8_t *larger = heap_caps_malloc(
            length, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
        if (larger == NULL) {
            ESP_LOGW(TAG, "dropping oversized %u-byte JPEG",
                     (unsigned)length);
            return;
        }
        xSemaphoreTake(s_frame_mutex, portMAX_DELAY);
        free(s_latest_jpeg);
        s_latest_jpeg = larger;
        s_latest_capacity = length;
        xSemaphoreGive(s_frame_mutex);
    }

    xSemaphoreTake(s_frame_mutex, portMAX_DELAY);
    memcpy(s_latest_jpeg, jpeg, length);
    s_latest_length = length;
    ++s_frame_sequence;
    xSemaphoreGive(s_frame_mutex);
}

void camera_stream_server_publish_detection(
    bool ball_valid, float ball_left, float ball_top,
    float ball_right, float ball_bottom, float ball_confidence,
    bool target_valid, float target_left, float target_top,
    float target_right, float target_bottom)
{
    if (s_frame_mutex == NULL) return;
    xSemaphoreTake(s_frame_mutex, portMAX_DELAY);
    s_detection_valid = ball_valid;
    s_detection_left = ball_left;
    s_detection_top = ball_top;
    s_detection_right = ball_right;
    s_detection_bottom = ball_bottom;
    s_detection_confidence = ball_confidence;
    s_target_valid = target_valid;
    s_target_left = target_left;
    s_target_top = target_top;
    s_target_right = target_right;
    s_target_bottom = target_bottom;
    xSemaphoreGive(s_frame_mutex);
}
