#pragma once
#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "emoji_render.h"

#define REMOTE_MAX_TRANSLATION 0.12f
#define REMOTE_MAX_ROTATION 0.60f
typedef enum { BLE_INVALID, BLE_ARM, BLE_STOP, BLE_DRIVE, BLE_FACE } ble_command_kind_t;
typedef struct { ble_command_kind_t kind; float x, y, turn; int face; } ble_command_t;

/* One complete ASCII command per ATT write; reject junk, NaN and embedded NUL. */
static inline ble_command_t ble_parse(const void *data, size_t len) {
    ble_command_t c = {0};
    if (!data || !len || len > 31 || memchr(data, 0, len)) return c;
    char buf[32]; memcpy(buf, data, len); buf[len] = 0;
    if (!strcmp(buf, "#ARM!")) { c.kind = BLE_ARM; return c; }
    if (!strcmp(buf, "#STOP!")) { c.kind = BLE_STOP; return c; }
    if (!strncmp(buf,"#FACE:",6)) {
        char *p=buf+6;
        int sign=1,face=0,digits=0;
        if(*p=='-') { sign=-1;++p; }
        while(*p>='0'&&*p<='9'&&digits<2) { face=face*10+(*p++-'0');++digits; }
        face*=sign;
        if(!digits||*p!='!'||p[1]||face<FACE_AUTO||face>=FACE_COUNT)return c;
        c.kind=BLE_FACE;c.face=face;return c;
    }
    if (buf[0] != '#') return c;
    char *p = buf + 1, *end;
    float v[3];
    for (int i=0; i<3; ++i) {
        /* Decimal syntax only; strtof also accepts hex and whitespace. */
        char *start=p;
        if (*p=='-' || *p=='+') ++p;
        unsigned digits=0;
        while (*p>='0' && *p<='9') { ++p; ++digits; }
        if (*p=='.') { ++p; while (*p>='0' && *p<='9') { ++p; ++digits; } }
        if (!digits || *p!=(i<2?',':'!')) return c;
        v[i]=strtof(start,&end);
        if (end!=p || !isfinite(v[i])) return c;
        ++p;
    }
    if (*p || fabsf(v[0])>REMOTE_MAX_TRANSLATION ||
        fabsf(v[1])>REMOTE_MAX_TRANSLATION || fabsf(v[2])>REMOTE_MAX_ROTATION) return c;
    c.kind=BLE_DRIVE;
    c.x=v[0]/REMOTE_MAX_TRANSLATION; c.y=v[1]/REMOTE_MAX_TRANSLATION;
    c.turn=v[2]/REMOTE_MAX_ROTATION;
    return c;
}
