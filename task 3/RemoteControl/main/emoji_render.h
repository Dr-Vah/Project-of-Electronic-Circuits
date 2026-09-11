#pragma once
#include <stdbool.h>
#include <stdint.h>
/* Existing IDs remain stable for remote clients. */
enum { FACE_AUTO=-1, FACE_SLEEP, FACE_HAPPY, FACE_COOL, FACE_HEART, FACE_DIZZY,
       FACE_OFFLINE, FACE_CURIOUS, FACE_LAUGH, FACE_TIRED, FACE_SURPRISE,
       FACE_ANNOYED, FACE_PROUD, FACE_COUNT };
static inline int face_choose(int manual, bool fault, bool moving, bool rotating) {
    if(fault)return FACE_OFFLINE;
    if(manual>=FACE_SLEEP&&manual<FACE_COUNT)return manual;
    return rotating?FACE_DIZZY:moving?FACE_COOL:FACE_SLEEP;
}
static inline bool face_circle(int x,int y,int cx,int cy,int r) {
    return (x-cx)*(x-cx)+(y-cy)*(y-cy)<=r*r;
}
static inline bool face_heart(int x,int y,int cx,int cy) {
    x-=cx;y-=cy;
    return face_circle(x,y,-4,-3,6)||face_circle(x,y,4,-3,6)||(y>=0&&y<=13&&x>=y-13&&x<=13-y);
}
/* Logical RGB565, converted to the existing driver's wire byte order by caller. */
static inline uint16_t face_pixel(int face,int x,int y,bool blink,bool linked) {
    const uint16_t bg=0x08a3,ink=0x18c3,yellow=0xfe25,pink=0xf98b;
    if(y>=146&&y<=149&&x>=20&&x<=107)return linked?0x6f19:0xf986;
    if(!face_circle(x,y,64,72,54))return bg;
    for(int eye=0;eye<2;eye++) {
        int cx=eye?84:44,dx=x-cx,dy=y-61;
        if(face==FACE_HEART&&face_heart(x,y,cx,60))return pink;
        if(face==FACE_OFFLINE||face==FACE_DIZZY) {
            if(dx>=-9&&dx<=9&&dy>=-9&&dy<=9&&((dx-dy>=-2&&dx-dy<=2)||(dx+dy>=-2&&dx+dy<=2)))return ink;
        } else if(face==FACE_COOL) {
            if(dx>=-16&&dx<=16&&dy>=-8&&dy<=9)return ink;
            if(x>=44&&x<=84&&y>=55&&y<=58)return ink;
        } else if(face!=FACE_HEART) {
            if(face==FACE_SLEEP||blink) {if(dx>=-10&&dx<=10&&dy>=0&&dy<=3)return ink;}
            else if(face_circle(x,y,cx,60,5))return ink;
        }
    }
    if(face==FACE_SLEEP) {if(face_circle(x,y,66,94,7))return ink;}
    else if(face==FACE_OFFLINE||face==FACE_DIZZY) {if(x>=48&&x<=80&&y>=96&&y<=99)return ink;}
    else {int d=(x-64)*(x-64)+(y-81)*(y-81);if(y>=92&&d>=23*23&&d<=27*27)return ink;}
    return face==FACE_OFFLINE?0xab9f:yellow;
}
