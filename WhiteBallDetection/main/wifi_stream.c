#include "wifi_stream.h"

#include <stdio.h>
#include <string.h>

#include "esp_event.h"
#include "esp_check.h"
#include "esp_heap_caps.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "nvs_flash.h"

#define WIFI_AP_SSID "BallDet-ESP32"
#define WIFI_AP_PASSWORD "balldet123"
#define WIFI_AP_CHANNEL 1

static const char *TAG = "wifi_stream";

static SemaphoreHandle_t s_state_lock;
static uint8_t *s_jpeg;
static size_t s_jpeg_length;
static size_t s_jpeg_capacity;
static uint16_t s_frame_width;
static uint16_t s_frame_height;
static bool s_detection_valid;
static ball_detection_t s_detection;
static uint16_t s_detection_width;
static uint16_t s_detection_height;

static const char s_index_html[] =
    "<!doctype html><html><head><meta charset='utf-8'>"
    "<meta name='viewport' content='width=device-width,initial-scale=1'>"
    "<title>BallDet</title><style>"
    "body{margin:0;background:#111;color:#eee;font-family:system-ui;text-align:center}"
    "h2{margin:12px 0 4px}.view{position:relative;display:inline-block;max-width:100vw}"
    "#cam{display:block;max-width:100vw;height:auto}#mark{position:absolute;inset:0}"
    "#status{margin:8px;font:16px monospace}</style></head><body>"
    "<h2>ESP32-S3 BallDet</h2><div class='view'>"
    "<img id='cam'><canvas id='mark'></canvas></div><div id='status'>waiting...</div>"
    "<script>const img=document.getElementById('cam'),cv=document.getElementById('mark'),"
    "ctx=cv.getContext('2d'),st=document.getElementById('status');let det=null,busy=false;"
    "function overlay(){const w=img.clientWidth,h=img.clientHeight;"
    "if(cv.width!==w||cv.height!==h){cv.width=w;cv.height=h}ctx.clearRect(0,0,w,h);"
    "if(det&&det.found){const x=det.x*w/det.width,y=det.y*h/det.height,r=det.radius*w/det.width;"
    "ctx.strokeStyle='#00ff38';ctx.lineWidth=3;ctx.beginPath();ctx.arc(x,y,r,0,Math.PI*2);ctx.stroke();"
    "ctx.fillStyle='#ff2020';ctx.beginPath();ctx.arc(x,y,3,0,Math.PI*2);ctx.fill();"
    "st.textContent=`BALL x=${det.x} y=${det.y} r=${det.radius} score=${det.score} support=${det.coverage}/32`;"
    "}else st.textContent='BALL not detected';}"
    "async function frame(){if(busy)return;busy=true;try{const r=await fetch('/frame.jpg?t='+Date.now(),{cache:'no-store'});"
    "if(r.ok){const b=await r.blob(),u=URL.createObjectURL(b);img.onload=()=>{URL.revokeObjectURL(u);overlay()};img.src=u}}"
    "catch(e){st.textContent='frame connection lost'}finally{busy=false;setTimeout(frame,30)}}"
    "async function detection(){try{const r=await fetch('/detection?t='+Date.now(),{cache:'no-store'});"
    "if(r.ok){det=await r.json();overlay()}}catch(e){}setTimeout(detection,150)}"
    "frame();detection();window.onresize=overlay;</script></body></html>";

static esp_err_t index_handler(httpd_req_t *request)
{
    httpd_resp_set_type(request, "text/html; charset=utf-8");
    httpd_resp_set_hdr(request, "Cache-Control", "no-store");
    return httpd_resp_send(request, s_index_html, HTTPD_RESP_USE_STRLEN);
}

static esp_err_t frame_handler(httpd_req_t *request)
{
    if (xSemaphoreTake(s_state_lock, pdMS_TO_TICKS(250)) != pdTRUE) {
        httpd_resp_set_status(request, "503 Service Unavailable");
        return httpd_resp_send(request, "frame busy", HTTPD_RESP_USE_STRLEN);
    }
    if (s_jpeg == NULL || s_jpeg_length == 0) {
        xSemaphoreGive(s_state_lock);
        httpd_resp_set_status(request, "503 Service Unavailable");
        return httpd_resp_send(request, "no camera frame yet",
                               HTTPD_RESP_USE_STRLEN);
    }
    httpd_resp_set_type(request, "image/jpeg");
    httpd_resp_set_hdr(request, "Cache-Control", "no-store, no-cache, must-revalidate");
    const esp_err_t error = httpd_resp_send(
        request, (const char *)s_jpeg, s_jpeg_length);
    xSemaphoreGive(s_state_lock);
    return error;
}

static esp_err_t detection_handler(httpd_req_t *request)
{
    char json[192];
    if (xSemaphoreTake(s_state_lock, pdMS_TO_TICKS(100)) != pdTRUE) {
        httpd_resp_set_status(request, "503 Service Unavailable");
        return httpd_resp_send(request, "state busy", HTTPD_RESP_USE_STRLEN);
    }
    const int length = s_detection_valid
        ? snprintf(json, sizeof(json),
                   "{\"found\":true,\"x\":%u,\"y\":%u,\"radius\":%u,"
                   "\"score\":%ld,\"coverage\":%u,\"width\":%u,\"height\":%u}",
                   s_detection.x, s_detection.y, s_detection.radius,
                   (long)s_detection.score, s_detection.coverage,
                   s_detection_width, s_detection_height)
        : snprintf(json, sizeof(json),
                   "{\"found\":false,\"width\":%u,\"height\":%u}",
                   s_frame_width, s_frame_height);
    xSemaphoreGive(s_state_lock);
    httpd_resp_set_type(request, "application/json");
    httpd_resp_set_hdr(request, "Cache-Control", "no-store");
    return httpd_resp_send(request, json, length);
}

void wifi_stream_publish_jpeg(const uint8_t *jpeg, size_t length,
                              uint16_t width, uint16_t height)
{
    if (jpeg == NULL || length == 0 || s_state_lock == NULL) return;
    if (xSemaphoreTake(s_state_lock, 0) != pdTRUE) return;
    if (length > s_jpeg_capacity) {
        uint8_t *larger = heap_caps_realloc(
            s_jpeg, length, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
        if (larger != NULL) {
            s_jpeg = larger;
            s_jpeg_capacity = length;
        }
    }
    if (s_jpeg != NULL && length <= s_jpeg_capacity) {
        memcpy(s_jpeg, jpeg, length);
        s_jpeg_length = length;
        s_frame_width = width;
        s_frame_height = height;
    }
    xSemaphoreGive(s_state_lock);
}

void wifi_stream_publish_detection(bool valid,
                                   const ball_detection_t *ball,
                                   uint16_t source_width,
                                   uint16_t source_height)
{
    if (s_state_lock == NULL || xSemaphoreTake(s_state_lock, 0) != pdTRUE) return;
    s_detection_valid = valid && ball != NULL;
    if (s_detection_valid) s_detection = *ball;
    s_detection_width = source_width;
    s_detection_height = source_height;
    xSemaphoreGive(s_state_lock);
}

esp_err_t wifi_stream_start(void)
{
    esp_err_t error = nvs_flash_init();
    if (error == ESP_ERR_NVS_NO_FREE_PAGES ||
        error == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        error = nvs_flash_init();
    }
    if (error != ESP_OK) return error;

    s_state_lock = xSemaphoreCreateMutex();
    if (s_state_lock == NULL) return ESP_ERR_NO_MEM;
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_ap();

    wifi_init_config_t init = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&init));
    wifi_config_t config = {0};
    memcpy(config.ap.ssid, WIFI_AP_SSID, strlen(WIFI_AP_SSID));
    config.ap.ssid_len = strlen(WIFI_AP_SSID);
    memcpy(config.ap.password, WIFI_AP_PASSWORD, strlen(WIFI_AP_PASSWORD));
    config.ap.channel = WIFI_AP_CHANNEL;
    config.ap.max_connection = 2;
    config.ap.authmode = WIFI_AUTH_WPA2_PSK;
    config.ap.pmf_cfg.required = false;
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &config));
    ESP_ERROR_CHECK(esp_wifi_start());

    httpd_config_t server_config = HTTPD_DEFAULT_CONFIG();
    server_config.stack_size = 8192;
    server_config.max_uri_handlers = 3;
    server_config.lru_purge_enable = true;
    httpd_handle_t server = NULL;
    ESP_RETURN_ON_ERROR(httpd_start(&server, &server_config), TAG,
                        "start HTTP server failed");
    const httpd_uri_t index_uri = {
        .uri = "/", .method = HTTP_GET, .handler = index_handler,
    };
    const httpd_uri_t frame_uri = {
        .uri = "/frame.jpg", .method = HTTP_GET, .handler = frame_handler,
    };
    const httpd_uri_t detection_uri = {
        .uri = "/detection", .method = HTTP_GET, .handler = detection_handler,
    };
    ESP_ERROR_CHECK(httpd_register_uri_handler(server, &index_uri));
    ESP_ERROR_CHECK(httpd_register_uri_handler(server, &frame_uri));
    ESP_ERROR_CHECK(httpd_register_uri_handler(server, &detection_uri));
    ESP_LOGI(TAG, "Wi-Fi AP ready: SSID=%s password=%s", WIFI_AP_SSID,
             WIFI_AP_PASSWORD);
    ESP_LOGI(TAG, "open http://192.168.4.1 in a browser");
    return ESP_OK;
}
