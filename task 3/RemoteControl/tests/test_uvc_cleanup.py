"""Compile the actual UVC allocation/cleanup functions with fault-injected USB.
No camera, board, or changes to the installed ESP-IDF are required.
"""
from pathlib import Path
import subprocess
import tempfile

source = (Path(__file__).resolve().parents[1] / 'components/usb_host_uvc/uvc_host.c').read_text()
start = source.index('static void uvc_transfers_free(')
end = source.index('/**', source.index('err:\n    uvc_transfers_free(uvc_stream);', start))
functions = source[start:end]
prefix = r'''
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
typedef int esp_err_t;
#define ESP_OK 0
#define ESP_ERR_NO_MEM 1
#define ESP_ERR_INVALID_ARG 2
#define UVC_CHECK(x,e) do { if(!(x))return (e); } while(0)
#define ESP_GOTO_ON_ERROR(call,label,tag,...) do { ret=(call);if(ret)goto label; } while(0)
#define ESP_LOGI(...) ((void)0)
#define USB_BM_ATTRIBUTES_XFER_ISOC 1
#define USB_EP_DESC_GET_XFERTYPE(p) ((p)->type)
#define USB_EP_DESC_GET_MPS(p) ((p)->mps)
#define USB_EP_DESC_GET_MULT(p) 0
typedef struct {unsigned type,mps;uint8_t bEndpointAddress;} usb_ep_desc_t;
typedef struct {void *device_handle,*context;int timeout_ms;uint8_t bEndpointAddress;
    void (*callback)(void);unsigned num_bytes;
    struct {unsigned num_bytes;} isoc_packet_desc[128];} usb_transfer_t;
typedef struct {struct {unsigned num_of_xfers;usb_transfer_t **xfers;void *dev_hdl;} constant;} uvc_stream_t;
static void isoc_transfer_callback(void) {}
static void bulk_transfer_callback(void) {}
static unsigned usb_round_up_to_mps(unsigned size,unsigned mps) {return (size+mps-1)/mps*mps;}
static void *live[8];static int attempts,fail_at,freed;static bool array_failure;
static void *tracked_alloc(size_t size) {
    if(array_failure){array_failure=false;return NULL;}
    void *p=calloc(1,size);assert(p);
    for(unsigned i=0;i<8;i++)if(!live[i]){live[i]=p;return p;}
    abort();
}
static void tracked_free(void *p) {
    if(!p)return;
    for(unsigned i=0;i<8;i++)if(live[i]==p){live[i]=NULL;free(p);return;}
    assert(!"double-free or unowned pointer");
}
static esp_err_t usb_host_transfer_alloc(size_t size,unsigned isoc,usb_transfer_t **out) {
    assert(size>0&&isoc<128);
    if(attempts++==fail_at)return ESP_ERR_NO_MEM;
    *out=tracked_alloc(sizeof(**out));return ESP_OK;
}
static void usb_host_transfer_free(usb_transfer_t *p) {if(p)freed++;tracked_free(p);}
#define malloc tracked_alloc
#define free tracked_free
'''
suffix = r'''
int main(void) {
    for(unsigned type=0;type<2;type++)for(int failure=-1;failure<4;failure++) {
        uvc_stream_t stream={0};usb_ep_desc_t ep={.type=type,.mps=64,.bEndpointAddress=0x81};
        attempts=0;freed=0;fail_at=failure;array_failure=failure==-1;
        int result=uvc_transfers_allocate(&stream,3,2048,&ep);
        assert(result==(failure==3?ESP_OK:ESP_ERR_NO_MEM));
        // Matches the caller's uvc_device_remove after allocation rollback.
        uvc_transfers_free(&stream);
        uvc_transfers_free(&stream);
        assert(!stream.constant.xfers&&!stream.constant.num_of_xfers);
        assert(freed==(failure<0?0:failure));
        for(unsigned i=0;i<8;i++)assert(!live[i]);
    }
    puts("PASS: UVC array/first/middle/last allocation failures, success, repeated cleanup (BULK + ISOC)");
}
'''
with tempfile.TemporaryDirectory(prefix='omni-uvc-test-') as tmp:
    c = Path(tmp) / 'cleanup.c'
    exe = Path(tmp) / 'cleanup.exe'
    c.write_text(prefix + functions + suffix)
    subprocess.run(['gcc', '-std=c11', '-Wall', '-Wextra', '-Werror', str(c), '-o', str(exe)], check=True)
    subprocess.run([str(exe)], check=True)
