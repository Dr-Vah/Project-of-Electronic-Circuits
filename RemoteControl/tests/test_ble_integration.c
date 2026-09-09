#include <assert.h>
#include <stdio.h>
#include "../main/remote_ble_state.h"
#include "../main/fpv_packet.h"
int main(void) {
    remote_state_t s={0};uint32_t ble=0;int face=FACE_AUTO;bool fault=false;
#define APPLY(text,time,token) remote_ble_apply(&s,&ble,&face,&fault,ble_parse(text,sizeof(text)-1),time,token)
    assert(!APPLY("#0.12,0,0!",0,1)); // No connection can auto-arm by sending speed.
    assert(remote_claim(&s,42,0));
    assert(!APPLY("#ARM!",1,100)); // HTTP owner is not displaced.
    assert(!APPLY("#FACE:3!",2,100));
    remote_ble_disconnect(&s,&ble,&fault);assert(s.token==42);
    assert(APPLY("#STOP!",3,100));assert(!s.token);
    assert(APPLY("#ARM!",4,100));assert(ble==100);
    assert(APPLY("#0.12,0,0!",5,100));assert(s.x==1);
    assert(APPLY("#FACE:3!",6,100));assert(face==3&&s.received==5);
    assert(!APPLY("#ARM!",7,101));assert(s.received==5);
    assert(!APPLY("#0.12,0,0!",300005,100));assert(!s.token&&fault);
    assert(APPLY("#FACE:-1!",300006,100));assert(face==FACE_AUTO&&!s.token);
    assert(!APPLY("#0.12,0,0!",300007,100)); // Face changes cannot resume motion.
    assert(APPLY("#ARM!",300008,101));assert(!fault);
    remote_ble_disconnect(&s,&ble,&fault);assert(!s.token&&!ble&&fault);
    uint8_t header[FPV_HEADER_SIZE];fpv_header(header,4,0x12345678,100,0);
    assert(!memcmp(header,"OVF1",4));assert(header[4]==4&&header[5]==0);
    assert(header[8]==0x78&&header[11]==0x12&&header[12]==100&&header[16]==0);
    puts("BLE/HTTP ownership, face/watchdog isolation and video header tests passed");
}
