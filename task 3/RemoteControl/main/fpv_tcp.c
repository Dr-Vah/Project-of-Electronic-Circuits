#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include "lwip/sockets.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_heap_caps.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "fpv.h"
#include "fpv_tcp.h"
#include "fpv_packet.h"

/* Separate low-priority LAN service for wx.createTCPSocket. No driving APIs.
   One viewer, bounded PSRAM snapshot, no motor/camera locks during socket I/O. */
static int listener=-1;
static uint8_t *tcp_snapshot;
static bool send_all(int fd,const uint8_t *data,size_t len,int64_t deadline) {
    while(len) {
        if(esp_timer_get_time()>=deadline)return false;
        int n=send(fd,data,len>4096?4096:len,0);
        if(n<=0)return false;
        data+=n;len-=n;
    }
    return true;
}
static void tcp_task(void *arg) {
    ESP_LOGI("fpv","tcp task started");
    while(true) {
        int fd=accept(listener,NULL,NULL);
        if(fd<0) { vTaskDelay(pdMS_TO_TICKS(250));continue; }
        struct timeval timeout={.tv_sec=0,.tv_usec=500000};
        if(setsockopt(fd,SOL_SOCKET,SO_RCVTIMEO,&timeout,sizeof(timeout))<0 ||
           setsockopt(fd,SOL_SOCKET,SO_SNDTIMEO,&timeout,sizeof(timeout))<0) {
            close(fd);continue;
        }
        int64_t last_request=esp_timer_get_time();
        while(true) {
            uint8_t request=0;
            int n=recv(fd,&request,1,0);
            if(n<0&&(errno==EAGAIN||errno==EWOULDBLOCK)) {
                if(esp_timer_get_time()-last_request<3000000)continue;
            }
            if(n!=1||request!=1)break;
            last_request=esp_timer_get_time();
            size_t len=0;uint32_t sequence=0,age=0;
            bool ready=fpv_copy_frame(tcp_snapshot,FPV_JPEG_CAP,&len,&sequence,&age);
            ESP_LOGI("fpv","frame ready=%d len=%u age=%u",ready,(unsigned)len,(unsigned)age);
            uint8_t header[FPV_HEADER_SIZE];
            fpv_header(header,(uint32_t)len,sequence,age,ready?0:1);
            int64_t deadline=esp_timer_get_time()+5000000;
            if(!send_all(fd,header,sizeof(header),deadline) ||
               (len&&!send_all(fd,tcp_snapshot,len,deadline))) {
                ESP_LOGW("fpv","frame send failed: len=%u ready=%d",(unsigned)len,ready);
                break;
            }
            /* Cap at 4 fps even if a client floods requests. */
            vTaskDelay(pdMS_TO_TICKS(250));
        }
        shutdown(fd,SHUT_RDWR);close(fd);
    }
}
esp_err_t fpv_tcp_start(void) {
    tcp_snapshot=heap_caps_malloc(FPV_JPEG_CAP,MALLOC_CAP_SPIRAM|MALLOC_CAP_8BIT);
    if(!tcp_snapshot)return ESP_ERR_NO_MEM;
    listener=socket(AF_INET,SOCK_STREAM,IPPROTO_TCP);
    if(listener<0)goto fail;
    int reuse=1;setsockopt(listener,SOL_SOCKET,SO_REUSEADDR,&reuse,sizeof(reuse));
    /* Serve the robot AP only, even if another interface is added later. */
    struct sockaddr_in address={.sin_family=AF_INET,.sin_port=htons(FPV_TCP_PORT)};
    address.sin_addr.s_addr=inet_addr("192.168.4.1");
    if(bind(listener,(struct sockaddr *)&address,sizeof(address))<0||listen(listener,1)<0)goto fail;
    if(xTaskCreate(tcp_task,"fpv_lan",4096,NULL,2,NULL)!=pdPASS)goto fail;
    ESP_LOGI("fpv","Mini program LAN video: 192.168.4.1:%d",FPV_TCP_PORT);
    return ESP_OK;
fail:
    if(listener>=0)close(listener);
    listener=-1;free(tcp_snapshot);tcp_snapshot=NULL;
    return ESP_FAIL;
}
