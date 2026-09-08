#pragma once
#include <math.h>
#include "emoji_render.h"
typedef struct {float dizzy,fatigue,idle;bool dizzy_latched;} mood_t;
static inline float mood_limit(float x){return fmaxf(0,fminf(100,x));}
static inline void mood_step(mood_t *m,float dt,bool moving,float turn) {
    if(!isfinite(dt)||dt<=0)return;
    dt=fminf(dt,.25f);
    float spin=fabsf(turn);
    m->dizzy=mood_limit(m->dizzy+(spin>.05f?30*spin:-12)*dt);
    m->fatigue=mood_limit(m->fatigue+(moving?2:-4)*dt);
    m->idle=moving?0:fminf(60,m->idle+dt);
    if(m->dizzy>=60)m->dizzy_latched=true;
    if(m->dizzy<=25)m->dizzy_latched=false;
}
static inline int mood_face(const mood_t *m,int manual,bool fault,bool moving) {
    if(fault)return FACE_OFFLINE;
    if(manual>=0&&manual<FACE_COUNT)return manual;
    if(m->dizzy_latched)return FACE_DIZZY;
    if(m->fatigue>=75)return FACE_TIRED;
    if(m->idle>=15)return FACE_SLEEP;
    if(!moving&&m->idle>=5)return FACE_CURIOUS;
    return moving?FACE_COOL:FACE_HAPPY;
}
