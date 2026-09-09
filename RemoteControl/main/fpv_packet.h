#pragma once
#include <stdint.h>
#define FPV_TCP_PORT 8266
#define FPV_JPEG_CAP (128u*1024u)
#define FPV_HEADER_SIZE 20
/* Request: one byte 1. Reply: OVF1, length, sequence, age_ms, status (LE32).
   Status 0 = JPEG; 1 = no fresh frame. Error replies have zero length. */
static inline void fpv_u32(uint8_t *p,uint32_t n) {
    for(unsigned i=0;i<4;++i)p[i]=(uint8_t)(n>>(8*i));
}
static inline void fpv_header(uint8_t out[FPV_HEADER_SIZE],uint32_t len,
                              uint32_t seq,uint32_t age,uint32_t status) {
    out[0]='O';out[1]='V';out[2]='F';out[3]='1';
    fpv_u32(out+4,len);fpv_u32(out+8,seq);fpv_u32(out+12,age);fpv_u32(out+16,status);
}
