#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_random.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "esp_http_server.h"
#include "nvs_flash.h"
#include "cJSON.h"
#include "car_control.h"
#include "remote_state.h"
#include "tft_display.h"
#include "emoji_render.h"
#include "naiwa_render.h"
#include "mood.h"
#include "fpv.h"
#include "local_tls.h"
#include "ble_remote.h"
#include "remote_ble_state.h"

/* Conservative initial limits; chassis +x right, +y IR-facing front. */
#define MAX_TRANSLATION REMOTE_MAX_TRANSLATION
#define MAX_ROTATION REMOTE_MAX_ROTATION
static remote_state_t state;
static uint32_t ble_token;
static bool ble_connected;
static SemaphoreHandle_t mutex;
static int manual_face=FACE_AUTO;
static bool link_fault=false;
static unsigned station_count=0;
static uint16_t face_frame[128*160];
static mood_t mood;
extern const char page_start[] asm("_binary_index_html_start");
extern const char page_end[] asm("_binary_index_html_end");

/* HTTP and BLE share the same lease, motor task, limits and watchdog. */
static bool ble_command(ble_command_t c) {
    uint32_t candidate=esp_random();if(!candidate)candidate=1;
    xSemaphoreTake(mutex,portMAX_DELAY);
    bool ok=remote_ble_apply(&state,&ble_token,&manual_face,&link_fault,c,esp_timer_get_time(),candidate);
    xSemaphoreGive(mutex);
    return ok;
}
static void ble_status(uint8_t out[BLE_STATUS_SIZE]) {
    xSemaphoreTake(mutex,portMAX_DELAY);
    bool moving=state.token&&(state.x!=0||state.y!=0||state.turn!=0);
    out[0]=1;
    out[1]=(uint8_t)fminf(100,fmaxf(0,mood.dizzy));
    out[2]=(uint8_t)fminf(100,fmaxf(0,mood.fatigue));
    out[3]=(uint8_t)fminf(255,fmaxf(0,mood.idle));
    out[4]=(uint8_t)mood_face(&mood,manual_face,link_fault,moving);
    out[5]=(uint8_t)(manual_face+1);
    out[6]=link_fault;
    out[7]=ble_token && state.token==ble_token;
    xSemaphoreGive(mutex);
}
static void ble_link(bool connected) {
    xSemaphoreTake(mutex,portMAX_DELAY);
    ble_connected=connected;
    if(!connected) {
        remote_ble_disconnect(&state,&ble_token,&link_fault);
    }
    xSemaphoreGive(mutex);
}

static float approach(float a, float b, float step) {
    return a + fmaxf(-step, fminf(step, b-a));
}
static void control_task(void *arg) {
    float x=0,y=0,w=0;
    TickType_t last=xTaskGetTickCount();
    int64_t mood_time=esp_timer_get_time();
    while (true) {
        xSemaphoreTake(mutex, portMAX_DELAY);
        uint32_t previous_token=state.token;
        remote_expire(&state, esp_timer_get_time());
        if(previous_token&&!state.token)link_fault=true;
        int64_t now=esp_timer_get_time();
        bool active=state.token&&(state.x!=0||state.y!=0||state.turn!=0);
        mood_step(&mood,(now-mood_time)/1000000.0f,active,state.token?state.turn:0);
        mood_time=now;
        if (!state.token || (state.x==0 && state.y==0 && state.turn==0)) {
            x=y=w=0;
            car_control_stop();
        } else {
            x=approach(x,state.x*MAX_TRANSLATION,0.004f);
            y=approach(y,state.y*MAX_TRANSLATION,0.004f);
            w=approach(w,state.turn*MAX_ROTATION,0.02f);
            car_control_set_velocity(x,y,w);
        }
        xSemaphoreGive(mutex);
        vTaskDelayUntil(&last,pdMS_TO_TICKS(10));
    }
}
/* Never hold the control lock during SPI writes. */
static void display_task(void *arg) {
    if(tft_display_init()!=ESP_OK) {
        ESP_LOGW("remote","TFT unavailable; driving remains enabled");
        vTaskDelete(NULL);return;
    }
    int last_face=-2;unsigned last_phase=2;bool last_link=false;
    while(true) {
        xSemaphoreTake(mutex,portMAX_DELAY);
        int face=mood_face(&mood,manual_face,link_fault,state.token&&(state.x!=0||state.y!=0||state.turn!=0));
        bool linked=(station_count>0||ble_connected)&&!link_fault;
        xSemaphoreGive(mutex);
        /* Slow idle breathing, livelier driving/laughter. No heap allocation. */
        unsigned interval=(face==FACE_SLEEP?1100:face==FACE_LAUGH||face==FACE_DIZZY?350:650);
        unsigned phase=(unsigned)(esp_timer_get_time()/1000/interval)&1u;
        if(face!=last_face||phase!=last_phase||linked!=last_link) {
            naiwa_render(face_frame,face,phase,linked);
            if(tft_display_show_camera_frame(face_frame,128,160)!=ESP_OK) {
                ESP_LOGW("remote","TFT write failed; display task stopped");
                vTaskDelete(NULL);return;
            }
            last_face=face;last_phase=phase;last_link=linked;
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}
static esp_err_t root_certificate(httpd_req_t *r) {
    extern const char start[] asm("_binary_omni_root_cer_start");
    extern const char end[] asm("_binary_omni_root_cer_end");
    httpd_resp_set_type(r,"application/x-x509-ca-cert");
    httpd_resp_set_hdr(r,"Content-Disposition","attachment; filename=omni-root.cer");
    return httpd_resp_send(r,start,end-start);
}
static esp_err_t page(httpd_req_t *r) {
    httpd_resp_set_type(r,"text/html; charset=utf-8");
    httpd_resp_set_hdr(r,"Cache-Control","no-store");
    httpd_resp_set_hdr(r,"Permissions-Policy","accelerometer=(self), gyroscope=(self)");
    return httpd_resp_send(r,page_start,page_end-page_start);
}
static esp_err_t status_http(httpd_req_t *r) {
    char body[200];
    xSemaphoreTake(mutex,portMAX_DELAY);
    int face=mood_face(&mood,manual_face,link_fault,state.token&&(state.x!=0||state.y!=0||state.turn!=0));
    snprintf(body,sizeof(body),"{\"dizzy\":%.0f,\"fatigue\":%.0f,\"idle\":%.0f,\"face\":%d,\"manual\":%d,\"fault\":%s}",
        mood.dizzy,mood.fatigue,mood.idle,face,manual_face,link_fault?"true":"false");
    xSemaphoreGive(mutex);
    httpd_resp_set_type(r,"application/json");httpd_resp_set_hdr(r,"Cache-Control","no-store");
    return httpd_resp_sendstr(r,body);
}
static bool number(cJSON *j,const char *name,double *value) {
    cJSON *v=cJSON_GetObjectItemCaseSensitive(j,name);
    if (!cJSON_IsNumber(v) || !isfinite(v->valuedouble)) return false;
    *value=v->valuedouble; return true;
}
static esp_err_t api(httpd_req_t *r) {
    /* Custom header forces cross-origin browser requests through preflight.
       No CORS headers or OPTIONS route are supplied. */
    char header[8];
    if (httpd_req_get_hdr_value_str(r,"X-Remote",header,sizeof(header))!=ESP_OK || strcmp(header,"1"))
        return httpd_resp_send_err(r,HTTPD_403_FORBIDDEN,"Same-origin controller required");
    char body[256];
    if (r->content_len >= sizeof(body)) return httpd_resp_send_err(r,HTTPD_400_BAD_REQUEST,"Too large");
    int used=0;
    while (used < r->content_len) {
        int n=httpd_req_recv(r,body+used,r->content_len-used);
        if(n<=0) return ESP_FAIL;
        used+=n;
    }
    body[used]=0;
    cJSON *j=cJSON_Parse(body);
    bool ok=false;
    uint32_t token=0;
    xSemaphoreTake(mutex,portMAX_DELAY);
    int64_t now=esp_timer_get_time();
    uint32_t previous_token=state.token;
    remote_expire(&state,now);
    if(previous_token&&!state.token)link_fault=true;
    if (!strcmp(r->uri,"/api/stop")) {
        remote_stop(&state); ble_token=0; ok=true;
    } else if (!strcmp(r->uri,"/api/emoji")) {
        double f,t=0;
        number(j,"token",&t);
        if(number(j,"face",&f)&&floor(f)==f&&f>=FACE_AUTO&&f<FACE_COUNT&&(!state.token||t==state.token)) {
            manual_face=(int)f;ok=true;
        }
        /* Expression changes do not refresh the driving watchdog. */
    } else if (!strcmp(r->uri,"/api/claim")) {
        token=esp_random(); if(!token) token=1;
        ok=remote_claim(&state,token,now);
        if(ok) { link_fault=false;ble_token=0; }
    } else {
        double t,q,x,y,w;
        if(number(j,"token",&t)&&number(j,"seq",&q)&&number(j,"x",&x)&&number(j,"y",&y)&&number(j,"turn",&w)
           &&t>=1&&t<=UINT32_MAX&&floor(t)==t&&q>=1&&q<=UINT32_MAX&&floor(q)==q) {
            ok=remote_command(&state,(uint32_t)t,(uint32_t)q,x,y,w,now);
        }
    }
    xSemaphoreGive(mutex);
    cJSON_Delete(j);
    httpd_resp_set_type(r,"application/json");
    httpd_resp_set_hdr(r,"Cache-Control","no-store");
    if(!ok) httpd_resp_set_status(r,"409 Conflict");
    char reply[96];
    snprintf(reply,sizeof(reply),"{\"ok\":%s,\"token\":%lu}",ok?"true":"false",(unsigned long)token);
    return httpd_resp_sendstr(r,reply);
}
static void wifi_event(void *arg,esp_event_base_t base,int32_t id,void *data) {
    if(id==WIFI_EVENT_AP_STACONNECTED) {
        xSemaphoreTake(mutex,portMAX_DELAY);station_count++;xSemaphoreGive(mutex);
    }
    if(id==WIFI_EVENT_AP_STADISCONNECTED) {
        xSemaphoreTake(mutex,portMAX_DELAY);
        if(station_count)station_count--;
        remote_stop(&state);link_fault=true;
        xSemaphoreGive(mutex);
    }
}
void app_main(void) {
    mutex=xSemaphoreCreateMutex(); configASSERT(mutex);
    car_control_config_t car=CAR_CONTROL_DEFAULT_CONFIG();
    ESP_ERROR_CHECK(car_control_init(&car));
    configASSERT(xTaskCreate(control_task,"remote_control",3072,NULL,6,NULL)==pdPASS);
    configASSERT(xTaskCreate(display_task,"emoji_display",4096,NULL,2,NULL)==pdPASS);
    esp_err_t e=nvs_flash_init();
    if(e==ESP_ERR_NVS_NO_FREE_PAGES||e==ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase()); e=nvs_flash_init();
    }
    ESP_ERROR_CHECK(e);
    esp_err_t ble_err=ble_remote_start(ble_command,ble_link,ble_status);
    if(ble_err!=ESP_OK) ESP_LOGW("remote","BLE unavailable: %s",esp_err_to_name(ble_err));
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_ap();
    wifi_init_config_t init=WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&init));
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT,ESP_EVENT_ANY_ID,wifi_event,NULL));
    wifi_config_t config={.ap={.ssid="Omni-Remote",.password="omni2026",
        .channel=6,.max_connection=3,.authmode=WIFI_AUTH_WPA2_PSK}};
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP,&config));
    ESP_ERROR_CHECK(esp_wifi_start());
    httpd_config_t h=HTTPD_DEFAULT_CONFIG();
    h.max_open_sockets=3;h.max_uri_handlers=10;
    h.lru_purge_enable=true; h.recv_wait_timeout=1; h.send_wait_timeout=1;
    httpd_handle_t server;
    ESP_ERROR_CHECK(httpd_start(&server,&h));
    httpd_uri_t root={.uri="/",.method=HTTP_GET,.handler=page};
    ESP_ERROR_CHECK(httpd_register_uri_handler(server,&root));
    httpd_uri_t status={.uri="/api/status",.method=HTTP_GET,.handler=status_http};
    ESP_ERROR_CHECK(httpd_register_uri_handler(server,&status));
    const char *paths[]={"/api/claim","/api/command","/api/stop","/api/emoji"};
    for(int i=0;i<4;i++) {
        httpd_uri_t u={.uri=paths[i],.method=HTTP_POST,.handler=api};
        ESP_ERROR_CHECK(httpd_register_uri_handler(server,&u));
    }
    httpd_uri_t certificate={.uri="/omni-root.cer",.method=HTTP_GET,.handler=root_certificate};
    ESP_ERROR_CHECK(httpd_register_uri_handler(server,&certificate));
    httpd_ssl_config_t tls=local_tls_config(443,32770,3,5);
    tls.httpd.max_uri_handlers=10;
    httpd_handle_t secure;
    esp_err_t tls_err=httpd_ssl_start(&secure,&tls);
    if(tls_err==ESP_OK) {
        ESP_ERROR_CHECK(httpd_register_uri_handler(secure,&root));
        ESP_ERROR_CHECK(httpd_register_uri_handler(secure,&status));
        ESP_ERROR_CHECK(httpd_register_uri_handler(secure,&certificate));
        for(int i=0;i<4;i++) {
            httpd_uri_t u={.uri=paths[i],.method=HTTP_POST,.handler=api};
            ESP_ERROR_CHECK(httpd_register_uri_handler(secure,&u));
        }
        ESP_LOGI("remote","Tilt control: https://192.168.4.1 after trusting local CA");
    } else ESP_LOGW("remote","HTTPS unavailable: %s; HTTP joystick remains available",esp_err_to_name(tls_err));
    ESP_LOGI("remote","Wi-Fi Omni-Remote / omni2026 ; open http://192.168.4.1");
    esp_err_t camera=fpv_start();
    if(camera!=ESP_OK)ESP_LOGW("remote","Video unavailable (%s); driving and expressions remain available",esp_err_to_name(camera));
}
