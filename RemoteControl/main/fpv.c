#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "esp_heap_caps.h"
#include "esp_http_server.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "usb/usb_host.h"
#include "usb/uvc_host.h"
#include "fpv.h"
#include "local_tls.h"

#define JPEG_CAP (128*1024)
static uint8_t *latest,*snapshot;
static size_t jpeg_size;
static int64_t jpeg_time,last_send;
static unsigned jpeg_sequence;
static SemaphoreHandle_t lock;
static SemaphoreHandle_t send_lock;
static QueueHandle_t connections;
static TaskHandle_t owner_task;
static const char *TAG="fpv";

/* Driver keeps frame ownership. Drop rather than wait on HTTP copy. */
static bool frame_cb(const uvc_host_frame_t *f,void *ctx) {
    if(f->vs_format.format!=UVC_VS_FORMAT_MJPEG||f->data_len<4||f->data_len>JPEG_CAP||
       f->data[0]!=0xff||f->data[1]!=0xd8)return true;
    if(xSemaphoreTake(lock,0)!=pdTRUE)return true;
    int64_t now=esp_timer_get_time();
    if(now-jpeg_time>=200000) {
        memcpy(latest,f->data,f->data_len);jpeg_size=f->data_len;jpeg_time=now;jpeg_sequence++;
    }
    xSemaphoreGive(lock);return true;
}
static void stream_event(const uvc_host_stream_event_data_t *e,void *ctx) {
    if(e->type==UVC_HOST_DEVICE_DISCONNECTED) {
        xSemaphoreTake(lock,portMAX_DELAY);jpeg_size=0;xSemaphoreGive(lock);
        xTaskNotifyGive(owner_task);
    }
}
static void driver_event(const uvc_host_driver_event_data_t *e,void *ctx) {
    if(e->type==UVC_HOST_DRIVER_EVENT_DEVICE_CONNECTED)
        xQueueSend(connections,e,0);
}
/* Stream open/start/close have one owner. No queued borrowed frame pointers. */
static void camera_owner(void *arg) {
    uvc_host_driver_event_data_t event;
    while(true) {
        if(xQueueReceive(connections,&event,portMAX_DELAY)!=pdTRUE)continue;
        size_t count=event.device_connected.frame_info_num;
        if(!count||count>128)continue;
        uvc_host_frame_info_t *m=calloc(count,sizeof(*m));
        if(!m)continue;
        uint8_t address=event.device_connected.dev_addr,index=event.device_connected.uvc_stream_index;
        if(uvc_host_get_frame_list(address,index,(uvc_host_frame_info_t (*)[])m,&count)!=ESP_OK){free(m);continue;}
        size_t best=SIZE_MAX;uint64_t pixels=UINT64_MAX;
        for(size_t i=0;i<count;i++)if(m[i].format==UVC_VS_FORMAT_MJPEG&&m[i].h_res&&m[i].v_res) {
            uint64_t p=(uint64_t)m[i].h_res*m[i].v_res;
            if(p<pixels){best=i;pixels=p;}
        }
        if(best==SIZE_MAX){ESP_LOGW(TAG,"No MJPEG mode");free(m);continue;}
        uint32_t interval=m[best].default_interval;
        if(m[best].interval_type==0) {
            if(m[best].interval_max>interval)interval=m[best].interval_max;
        } else for(size_t i=0;i<m[best].interval_type&&i<CONFIG_UVC_INTERVAL_ARRAY_SIZE;i++)
            if(m[best].interval[i]>interval)interval=m[best].interval[i];
        uvc_host_stream_config_t config={
            .event_cb=stream_event,.frame_cb=frame_cb,
            .usb={.dev_addr=address,.vid=UVC_HOST_ANY_VID,.pid=UVC_HOST_ANY_PID,.uvc_stream_index=index},
            .vs_format={.h_res=m[best].h_res,.v_res=m[best].v_res,.fps=interval?10000000.0f/interval:0,.format=UVC_VS_FORMAT_MJPEG},
            .advanced={.number_of_frame_buffers=3,.frame_size=JPEG_CAP,
                .frame_heap_caps=MALLOC_CAP_SPIRAM|MALLOC_CAP_8BIT,.number_of_urbs=3,.urb_size=10*1024}};
        free(m);
        ulTaskNotifyTake(pdTRUE,0);
        uvc_host_stream_hdl_t stream=NULL;
        esp_err_t err=uvc_host_stream_open(&config,pdMS_TO_TICKS(5000),&stream);
        if(err==ESP_OK)err=uvc_host_stream_start(stream);
        if(err==ESP_OK) {
            ESP_LOGI(TAG,"MJPEG %ux%u %.1f fps, browser capped at 5 fps",config.vs_format.h_res,config.vs_format.v_res,config.vs_format.fps);
            ulTaskNotifyTake(pdTRUE,portMAX_DELAY);
        } else ESP_LOGW(TAG,"Camera start: %s; reconnect camera to retry",esp_err_to_name(err));
        if(stream)uvc_host_stream_close(stream);
        xSemaphoreTake(lock,portMAX_DELAY);jpeg_size=0;xSemaphoreGive(lock);
    }
}
static void usb_events(void *arg) {
    while(true){uint32_t flags=0;esp_err_t e=usb_host_lib_handle_events(portMAX_DELAY,&flags);
        if(e!=ESP_OK){ESP_LOGW(TAG,"USB events: %s",esp_err_to_name(e));vTaskDelay(pdMS_TO_TICKS(50));}
        if(flags&USB_HOST_LIB_EVENT_FLAGS_NO_CLIENTS)usb_host_device_free_all();}
}
static esp_err_t frame_http_inner(httpd_req_t *r) {
    httpd_resp_set_hdr(r,"Access-Control-Allow-Origin","*");
    httpd_resp_set_hdr(r,"Access-Control-Expose-Headers","X-Frame-Sequence,X-Frame-Age-Ms");
    httpd_resp_set_hdr(r,"Cache-Control","no-store");
    int64_t now=esp_timer_get_time();
    if(now-last_send<180000){httpd_resp_set_status(r,"429 Too Many Requests");return httpd_resp_sendstr(r,"Video rate limited");}
    xSemaphoreTake(lock,portMAX_DELAY);
    size_t n=jpeg_size;int64_t age=now-jpeg_time;unsigned seq=jpeg_sequence;
    if(n&&age<1000000)memcpy(snapshot,latest,n);else n=0;
    xSemaphoreGive(lock);
    if(!n){httpd_resp_set_status(r,"503 Service Unavailable");return httpd_resp_sendstr(r,"Camera not ready or frame stale");}
    last_send=now;
    char text[32];snprintf(text,sizeof(text),"%u",seq);httpd_resp_set_hdr(r,"X-Frame-Sequence",text);
    char age_text[32];snprintf(age_text,sizeof(age_text),"%lld",(long long)(age/1000));httpd_resp_set_hdr(r,"X-Frame-Age-Ms",age_text);
    httpd_resp_set_type(r,"image/jpeg");
    return httpd_resp_send(r,(const char*)snapshot,n);
}
static esp_err_t frame_http(httpd_req_t *r) {
    /* HTTP and HTTPS video tasks share one snapshot; don't block control. */
    if(xSemaphoreTake(send_lock,0)!=pdTRUE) {
        httpd_resp_set_hdr(r,"Access-Control-Allow-Origin","*");
        httpd_resp_set_status(r,"429 Too Many Requests");return httpd_resp_sendstr(r,"Video busy");
    }
    esp_err_t e=frame_http_inner(r);xSemaphoreGive(send_lock);return e;
}
esp_err_t fpv_start(void) {
    /* Allocate optional video buffers only in PSRAM, never starve motor RAM. */
    latest=heap_caps_malloc(JPEG_CAP,MALLOC_CAP_SPIRAM|MALLOC_CAP_8BIT);
    snapshot=heap_caps_malloc(JPEG_CAP,MALLOC_CAP_SPIRAM|MALLOC_CAP_8BIT);
    lock=xSemaphoreCreateMutex();send_lock=xSemaphoreCreateMutex();connections=xQueueCreate(4,sizeof(uvc_host_driver_event_data_t));
    if(!latest||!snapshot||!lock||!send_lock||!connections) {
        free(latest);free(snapshot);if(lock)vSemaphoreDelete(lock);if(connections)vQueueDelete(connections);
        if(send_lock)vSemaphoreDelete(send_lock);
        return ESP_ERR_NO_MEM;
    }
    httpd_config_t h=HTTPD_DEFAULT_CONFIG();h.server_port=81;h.ctrl_port=32769;
    h.task_priority=3;h.max_open_sockets=2;h.lru_purge_enable=true;h.send_wait_timeout=1;h.recv_wait_timeout=1;
    httpd_handle_t server;esp_err_t err=httpd_start(&server,&h);if(err!=ESP_OK)return err;
    httpd_uri_t route={.uri="/frame.jpg",.method=HTTP_GET,.handler=frame_http};
    err=httpd_register_uri_handler(server,&route);if(err!=ESP_OK)return err;
    httpd_ssl_config_t tls=local_tls_config(8443,32771,2,3);
    httpd_handle_t secure;
    err=httpd_ssl_start(&secure,&tls);
    if(err==ESP_OK)err=httpd_register_uri_handler(secure,&route);
    if(err!=ESP_OK)ESP_LOGW(TAG,"HTTPS video unavailable: %s",esp_err_to_name(err));
    const usb_host_config_t host={.skip_phy_setup=false,.intr_flags=ESP_INTR_FLAG_LOWMED};
    err=usb_host_install(&host);if(err!=ESP_OK)return err;
    if(xTaskCreate(usb_events,"usb_events",4096,NULL,4,NULL)!=pdPASS)return ESP_ERR_NO_MEM;
    if(xTaskCreate(camera_owner,"uvc_owner",4096,NULL,3,&owner_task)!=pdPASS)return ESP_ERR_NO_MEM;
    const uvc_host_driver_config_t driver={.driver_task_stack_size=4096,.driver_task_priority=4,
        .xCoreID=tskNO_AFFINITY,.create_background_task=true,.event_cb=driver_event};
    return uvc_host_install(&driver);
}
