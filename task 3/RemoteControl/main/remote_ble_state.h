#pragma once
#include "remote_state.h"
#include "ble_protocol.h"

/* Caller holds the same mutex used by HTTP and the motor control task. */
static inline bool remote_ble_apply(remote_state_t *s,uint32_t *ble_token,
        int *manual_face,bool *fault,ble_command_t c,int64_t now,uint32_t candidate) {
    uint32_t previous=s->token;
    remote_expire(s,now);
    if(previous&&!s->token)*fault=true;
    if(c.kind==BLE_STOP) {
        remote_stop(s);*ble_token=0;return true;
    }
    if(c.kind==BLE_FACE) {
        if(c.face<FACE_AUTO||c.face>=FACE_COUNT)return false;
        if(s->token&&(!*ble_token||s->token!=*ble_token))return false;
        *manual_face=c.face;return true; // Never refresh the driving lease.
    }
    if(c.kind==BLE_ARM) {
        if(!remote_claim(s,candidate,now))return false;
        *ble_token=candidate;*fault=false;return true;
    }
    return c.kind==BLE_DRIVE && *ble_token && s->token==*ble_token &&
        remote_command(s,*ble_token,s->sequence+1,c.x,c.y,c.turn,now);
}
static inline void remote_ble_disconnect(remote_state_t *s,uint32_t *ble_token,bool *fault) {
    if(*ble_token&&s->token==*ble_token) { remote_stop(s);*fault=true; }
    *ble_token=0;
}
