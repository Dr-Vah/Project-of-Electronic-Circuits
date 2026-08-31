#include "camera_stream_server.h"
#include "camera_debug_page.h"

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
#define LINE_BLOCK_COUNT 4

static const char *TAG = "camera_web";
static SemaphoreHandle_t s_mutex;
static uint8_t *s_jpeg;
static size_t s_jpeg_length, s_jpeg_capacity;
static uint32_t s_frames;
static uint8_t s_threshold = 30, s_roi_top = 45, s_roi_bottom = 100, s_active_percent = 8;
static uint16_t s_width, s_height, s_centroid[4];
static uint32_t s_dark[4], s_total[4];
static bool s_active[4];

#if 0 /* Replaced by CAMERA_DEBUG_PAGE; retained temporarily for easy comparison. */
static const char s_html[] =
    "<!doctype html><html lang=zh-CN><meta charset=utf-8><meta name=viewport content='width=device-width,initial-scale=1'><title>Camera line debug</title><style>body{margin:0;background:#101318;color:#eef;font:15px system-ui}main{max-width:920px;margin:auto;padding:16px}h1{font-size:21px}canvas{width:100%;background:#000;border-radius:8px;image-rendering:pixelated}.controls,.stats{display:grid;grid-template-columns:repeat(4,minmax(0,1fr));gap:8px;margin:12px 0}.card{background:#1c222b;padding:9px;border-radius:6px}.on{color:#6f6}.off{color:#f88}label{display:block;font-size:13px}input{width:100%}small{color:#9bd}</style><main><h1>Camera line debug: binary + 4 vertical ROIs</h1><small>Black pixels are line pixels. ROI 1..4 are image left to right.</small><div class=controls><label>Threshold <b id=tv></b><input id=t type=range min=0 max=255></label><label>ROI top (%) <b id=topv></b><input id=top type=range min=0 max=99></label><label>ROI bottom (%) <b id=botv></b><input id=bot type=range min=1 max=100></label><label>Active (%) <b id=av></b><input id=a type=range min=1 max=100></label></div><canvas id=out></canvas><div id=stats class=stats></div><small id=info>Waiting for frames...</small><img id=src src=/stream hidden><script>const I=document.querySelector('#src'),C=document.querySelector('#out'),X=C.getContext('2d'),S=document.querySelector('#stats');let cfg;async function get(){try{let j=await(await fetch('/status',{cache:'no-store'})).json();cfg=j;for(const[k,id]of[['threshold','t'],['roi_top','top'],['roi_bottom','bot'],['active_percent','a']]){let e=document.querySelector('#'+id);if(document.activeElement!==e)e.value=j[k]}tv.textContent=j.threshold;topv.textContent=j.roi_top;botv.textContent=j.roi_bottom;av.textContent=j.active_percent;S.innerHTML=j.blocks.map((b,i)=>`<div class=card>ROI ${i+1}: <b class=${b.active?'on':'off'}>${b.active?'ON':'OFF'}</b><br>black ${b.percent}%<br>pixels ${b.dark}/${b.total}<br>centroid x=${b.centroid_x}</div>`).join('');info.textContent=`frames ${j.frames}, JPEG ${j.bytes} B, image ${j.width}x${j.height}`;draw()}catch(e){}}function draw(){if(!cfg||!I.naturalWidth)return;C.width=I.naturalWidth;C.height=I.naturalHeight;X.drawImage(I,0,0);let d=X.getImageData(0,0,C.width,C.height),p=d.data;for(let n=0;n<p.length;n+=4){let y=(77*p[n]+150*p[n+1]+29*p[n+2])>>8,v=y<=cfg.threshold?0:255;p[n]=p[n+1]=p[n+2]=v}X.putImageData(d,0,0);let y0=C.height*cfg.roi_top/100,y1=C.height*cfg.roi_bottom/100;X.strokeStyle='#ff3cff';X.lineWidth=Math.max(1,C.width/160);X.font=`${Math.max(10,C.width/20)}px sans-serif`;for(let i=0;i<4;i++){let x0=i*C.width/4;X.strokeRect(x0,y0,C.width/4,y1-y0);X.fillStyle='#ff3cff';X.fillText('ROI '+(i+1),x0+3,y0+14)}}I.onload=draw;for(const id of ['t','top','bot','a'])document.querySelector('#'+id).oninput=async()=>{await fetch('/config?threshold='+t.value+'&roi_top='+top.value+'&roi_bottom='+bot.value+'&active_percent='+a.value);get()};get();setInterval(get,500);</script></main>";

/* Kept separately so it can be appended after the compact page above. It makes
 * the four-channel geometry unambiguous even on browsers that cache old HTML. */
static const char s_roi_overlay_script[] =
    "<script>(()=>{window.t=document.querySelector('#t');window.top=document.querySelector('#top');window.bot=document.querySelector('#bot');window.a=document.querySelector('#a');window.tv=document.querySelector('#tv');window.topv=document.querySelector('#topv');window.botv=document.querySelector('#botv');window.av=document.querySelector('#av');window.info=document.querySelector('#info');const raw=document.querySelector('#src'),canvas=document.querySelector('#out');"
    "const label=document.createElement('small');label.textContent='Original camera image (for threshold diagnosis)';"
    "raw.hidden=false;raw.style.cssText='display:block;width:100%;margin-top:6px;border-radius:8px;background:#000';raw.before(label);for(const id of ['t','top','bot','a'])document.querySelector('#'+id).oninput=async()=>{const T=document.querySelector('#t'),P=document.querySelector('#top'),B=document.querySelector('#bot'),A=document.querySelector('#a');await fetch('/config?threshold='+T.value+'&roi_top='+P.value+'&roi_bottom='+B.value+'&active_percent='+A.value)};"
    "async function overlay(){try{const q=await fetch('/status',{cache:'no-store'});const j=await q.json();"
    "if(!canvas.width)return;const g=canvas.getContext('2d'),y0=canvas.height*j.roi_top/100,y1=canvas.height*j.roi_bottom/100,w=canvas.width/4;"
    "const colors=['#00e5ff','#ffea00','#ff5ca8','#7cff6b'];g.lineWidth=Math.max(3,canvas.width/55);g.font=`bold ${Math.max(12,canvas.width/18)}px sans-serif`;"
    "for(let i=0;i<4;i++){const x=i*w;g.fillStyle=colors[i]+'33';g.fillRect(x,y0,w,y1-y0);g.strokeStyle=colors[i];g.strokeRect(x,y0,w,y1-y0);g.fillStyle='#000';g.fillRect(x+2,y0+2,Math.min(w-4,canvas.width/3),Math.max(18,canvas.height/12));g.fillStyle=colors[i];g.fillText(`CH${i+1}  ${i*25}-${(i+1)*25}%`,x+5,y0+Math.max(15,canvas.height/15));}}catch(e){}}setInterval(overlay,120);overlay()})();</script>";
#endif

static esp_err_t index_handler(httpd_req_t *r)
{
    httpd_resp_set_type(r, "text/html; charset=utf-8");
    httpd_resp_set_hdr(r, "Cache-Control", "no-store");
    return httpd_resp_send(r, CAMERA_DEBUG_PAGE, HTTPD_RESP_USE_STRLEN);
}

static esp_err_t status_handler(httpd_req_t *r)
{
    uint32_t frames,dark[4],total[4]; size_t bytes; uint16_t width,height,centroid[4]; uint8_t th,top,bottom,active_percent; bool active[4];
    xSemaphoreTake(s_mutex,portMAX_DELAY); frames=s_frames; bytes=s_jpeg_length; width=s_width; height=s_height; th=s_threshold; top=s_roi_top; bottom=s_roi_bottom; active_percent=s_active_percent; memcpy(dark,s_dark,sizeof(dark)); memcpy(total,s_total,sizeof(total)); memcpy(centroid,s_centroid,sizeof(centroid)); memcpy(active,s_active,sizeof(active)); xSemaphoreGive(s_mutex);
    char json[640]; int n=snprintf(json,sizeof(json),"{\"frames\":%"PRIu32",\"bytes\":%u,\"width\":%u,\"height\":%u,\"threshold\":%u,\"roi_top\":%u,\"roi_bottom\":%u,\"active_percent\":%u,\"blocks\":[{\"dark\":%"PRIu32",\"total\":%"PRIu32",\"percent\":%u,\"centroid_x\":%u,\"active\":%s},{\"dark\":%"PRIu32",\"total\":%"PRIu32",\"percent\":%u,\"centroid_x\":%u,\"active\":%s},{\"dark\":%"PRIu32",\"total\":%"PRIu32",\"percent\":%u,\"centroid_x\":%u,\"active\":%s},{\"dark\":%"PRIu32",\"total\":%"PRIu32",\"percent\":%u,\"centroid_x\":%u,\"active\":%s}]}",frames,(unsigned)bytes,width,height,th,top,bottom,active_percent,dark[0],total[0],total[0]?(unsigned)(dark[0]*100U/total[0]):0,centroid[0],active[0]?"true":"false",dark[1],total[1],total[1]?(unsigned)(dark[1]*100U/total[1]):0,centroid[1],active[1]?"true":"false",dark[2],total[2],total[2]?(unsigned)(dark[2]*100U/total[2]):0,centroid[2],active[2]?"true":"false",dark[3],total[3],total[3]?(unsigned)(dark[3]*100U/total[3]):0,centroid[3],active[3]?"true":"false");
    httpd_resp_set_type(r,"application/json"); httpd_resp_set_hdr(r,"Cache-Control","no-store"); return httpd_resp_send(r,json,n);
}

static uint8_t query_u8(httpd_req_t *r,const char *key,uint8_t old,uint8_t min,uint8_t max)
{ char q[128]={0},v[8]={0},*end=NULL; if(httpd_req_get_url_query_len(r)>=sizeof(q)||httpd_req_get_url_query_str(r,q,sizeof(q))!=ESP_OK||httpd_query_key_value(q,key,v,sizeof(v))!=ESP_OK)return old; long x=strtol(v,&end,10); return end==v||*end!='\0'||x<min||x>max?old:(uint8_t)x; }

static esp_err_t config_handler(httpd_req_t *r)
{ xSemaphoreTake(s_mutex,portMAX_DELAY); s_threshold=query_u8(r,"threshold",s_threshold,0,255); s_roi_top=query_u8(r,"roi_top",s_roi_top,0,99); s_roi_bottom=query_u8(r,"roi_bottom",s_roi_bottom,1,100); s_active_percent=query_u8(r,"active_percent",s_active_percent,1,100); if(s_roi_bottom<=s_roi_top)s_roi_bottom=s_roi_top+1; xSemaphoreGive(s_mutex); return httpd_resp_sendstr(r,"{\"ok\":true}"); }

static esp_err_t stream_handler(httpd_req_t *r)
{
    httpd_resp_set_type(r,"multipart/x-mixed-replace;boundary=" STREAM_BOUNDARY); uint8_t *copy=NULL; size_t cap=0; uint32_t last=0; esp_err_t result=ESP_OK;
    while(result==ESP_OK){size_t needed;uint32_t seq;xSemaphoreTake(s_mutex,portMAX_DELAY);needed=s_jpeg_length;seq=s_frames;xSemaphoreGive(s_mutex);if(!needed||seq==last){vTaskDelay(pdMS_TO_TICKS(20));continue;}if(needed>cap){uint8_t *larger=heap_caps_malloc(needed,MALLOC_CAP_SPIRAM|MALLOC_CAP_8BIT);if(!larger){result=ESP_ERR_NO_MEM;break;}free(copy);copy=larger;cap=needed;}size_t len=0;xSemaphoreTake(s_mutex,portMAX_DELAY);if(s_jpeg_length<=cap&&s_frames!=last){len=s_jpeg_length;seq=s_frames;memcpy(copy,s_jpeg,len);}xSemaphoreGive(s_mutex);if(!len)continue;char h[128];int hn=snprintf(h,sizeof(h),"--"STREAM_BOUNDARY"\r\nContent-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n",(unsigned)len);result=httpd_resp_send_chunk(r,h,hn);if(result==ESP_OK)result=httpd_resp_send_chunk(r,(const char *)copy,len);if(result==ESP_OK)result=httpd_resp_send_chunk(r,"\r\n",2);last=seq;} free(copy); return result;
}

static esp_err_t start_http_server(void)
{ httpd_config_t c=HTTPD_DEFAULT_CONFIG(); c.stack_size=6144;c.max_uri_handlers=4;c.lru_purge_enable=true;httpd_handle_t server=NULL;esp_err_t e=httpd_start(&server,&c);if(e!=ESP_OK)return e;const httpd_uri_t a={.uri="/",.method=HTTP_GET,.handler=index_handler},b={.uri="/stream",.method=HTTP_GET,.handler=stream_handler},d={.uri="/status",.method=HTTP_GET,.handler=status_handler},f={.uri="/config",.method=HTTP_GET,.handler=config_handler};if((e=httpd_register_uri_handler(server,&a))!=ESP_OK||(e=httpd_register_uri_handler(server,&b))!=ESP_OK||(e=httpd_register_uri_handler(server,&d))!=ESP_OK||(e=httpd_register_uri_handler(server,&f))!=ESP_OK){httpd_stop(server);return e;}return ESP_OK; }

esp_err_t camera_stream_server_init(void)
{
    s_mutex=xSemaphoreCreateMutex();if(!s_mutex)return ESP_ERR_NO_MEM;esp_err_t e=nvs_flash_init();if(e==ESP_ERR_NVS_NO_FREE_PAGES||e==ESP_ERR_NVS_NEW_VERSION_FOUND){ESP_ERROR_CHECK(nvs_flash_erase());e=nvs_flash_init();}if(e!=ESP_OK)return e;if((e=esp_netif_init())!=ESP_OK)return e;if((e=esp_event_loop_create_default())!=ESP_OK)return e;if(!esp_netif_create_default_wifi_ap())return ESP_FAIL;wifi_init_config_t init=WIFI_INIT_CONFIG_DEFAULT();if((e=esp_wifi_init(&init))!=ESP_OK)return e;wifi_config_t wifi={0};memcpy(wifi.ap.ssid,CAMERA_AP_SSID,sizeof(CAMERA_AP_SSID));memcpy(wifi.ap.password,CAMERA_AP_PASSWORD,sizeof(CAMERA_AP_PASSWORD));wifi.ap.ssid_len=strlen(CAMERA_AP_SSID);wifi.ap.channel=CAMERA_AP_CHANNEL;wifi.ap.max_connection=CAMERA_AP_MAX_CLIENTS;wifi.ap.authmode=WIFI_AUTH_WPA2_PSK;wifi.ap.pmf_cfg.required=true;if((e=esp_wifi_set_mode(WIFI_MODE_AP))!=ESP_OK)return e;if((e=esp_wifi_set_config(WIFI_IF_AP,&wifi))!=ESP_OK)return e;if((e=esp_wifi_start())!=ESP_OK)return e;if((e=start_http_server())!=ESP_OK)return e;ESP_LOGI(TAG,"connect Wi-Fi '%s' password '%s', then open http://192.168.4.1/",CAMERA_AP_SSID,CAMERA_AP_PASSWORD);return ESP_OK;
}

void camera_stream_server_publish_jpeg(const uint8_t *jpeg,size_t len)
{ if(!s_mutex||!jpeg||!len)return;if(len>s_jpeg_capacity){uint8_t *larger=heap_caps_malloc(len,MALLOC_CAP_SPIRAM|MALLOC_CAP_8BIT);if(!larger){ESP_LOGW(TAG,"dropping JPEG: no PSRAM");return;}xSemaphoreTake(s_mutex,portMAX_DELAY);free(s_jpeg);s_jpeg=larger;s_jpeg_capacity=len;xSemaphoreGive(s_mutex);}xSemaphoreTake(s_mutex,portMAX_DELAY);memcpy(s_jpeg,jpeg,len);s_jpeg_length=len;++s_frames;xSemaphoreGive(s_mutex); }

void camera_stream_server_analyse_rgb565(const uint16_t *pixels,uint16_t width,uint16_t height)
{
    if (!s_mutex || !pixels || width < 4 || !height) return;

    uint8_t threshold, top, bottom, active_percent;
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    threshold = s_threshold;
    top = s_roi_top;
    bottom = s_roi_bottom;
    active_percent = s_active_percent;
    xSemaphoreGive(s_mutex);

    const uint16_t y0 = (uint32_t)height * top / 100U;
    const uint16_t y1 = (uint32_t)height * bottom / 100U;
    uint32_t dark[4] = {0}, total[4] = {0}, x_sum[4] = {0};
    for (uint16_t y = y0; y < y1; ++y) {
        for (uint16_t x = 0; x < width; ++x) {
            const unsigned block = (uint32_t)x * 4U / width;
            const uint16_t value = pixels[(size_t)y * width + x];
            const uint16_t rgb = (uint16_t)((value << 8) | (value >> 8));
            const uint8_t red = (rgb >> 11) & 31U;
            const uint8_t green = (rgb >> 5) & 63U;
            const uint8_t blue = rgb & 31U;
            const uint8_t luminance = (uint8_t)((77U * red * 255U / 31U +
                150U * green * 255U / 63U + 29U * blue * 255U / 31U) >> 8);
            ++total[block];
            if (luminance <= threshold) {
                ++dark[block];
                x_sum[block] += x;
            }
        }
    }

    xSemaphoreTake(s_mutex, portMAX_DELAY);
    s_width = width;
    s_height = height;
    for (unsigned i = 0; i < 4; ++i) {
        s_dark[i] = dark[i];
        s_total[i] = total[i];
        s_centroid[i] = dark[i] ? (uint16_t)(x_sum[i] / dark[i]) : 0;
        s_active[i] = total[i] && dark[i] * 100U >= total[i] * active_percent;
    }
    xSemaphoreGive(s_mutex);
}

bool camera_stream_server_get_line_state(camera_line_state_t *state)
{
    if(!s_mutex||!state)return false;
    bool valid;
    xSemaphoreTake(s_mutex,portMAX_DELAY);
    for(unsigned i=0;i<4;i++){
        state->active[i]=s_active[i];
        state->black_percent[i]=s_total[i]?(uint8_t)(s_dark[i]*100U/s_total[i]):0;
        state->centroid_x[i]=s_centroid[i];
    }
    valid=s_width!=0 && s_height!=0;
    xSemaphoreGive(s_mutex);
    return valid;
}
