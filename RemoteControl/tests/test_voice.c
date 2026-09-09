#include <assert.h>
#include <stdio.h>
#include "../main/voice_control.h"
#include "../main/mood.h"
#include "../main/ble_protocol.h"

int main(void) {
    voice_control_t v={0};
    float y,w;
    voice_sample(&v,0,0,0,0,false);
    voice_sample(&v,0,0,0,100000,false);
    assert(!v.active); /* Boot low must not mean reverse. */
    voice_sample(&v,1,0,0,110000,false);
    voice_sample(&v,1,0,0,140000,false);
    assert(v.armed);
    for (unsigned c=0;c<4;c++) {
        int64_t t=200000+c*100000;
        voice_sample(&v,0,c>>1,c&1,t,false);
        voice_sample(&v,0,c>>1,c&1,t+30000,false);
        voice_velocity(&v,&y,&w);
        assert(v.active);
        assert(y==(c==0?-VOICE_SPEED_MPS:c==3?VOICE_SPEED_MPS:0));
        assert(w==(c==2?VOICE_TURN_RAD_S:c==1?-VOICE_TURN_RAD_S:0));
    }
    voice_sample(&v,0,0,1,600000,false);
    voice_sample(&v,0,0,0,610000,false);
    assert(v.command==3); /* Ignore transient intermediate direction. */
    assert(voice_sample(&v,1,0,0,620000,false));
    assert(!v.active);
    assert(!voice_sample(&v,1,0,0,650000,false));
    voice_sample(&v,0,1,1,700000,false);
    voice_sample(&v,0,1,1,730000,false);
    assert(v.active);
    voice_cancel(&v); /* Phone claim/stop. */
    voice_sample(&v,0,1,1,800000,true);
    voice_sample(&v,0,1,1,900000,false);
    assert(!v.active && !v.armed);
    assert(voice_sample(&v,1,1,1,910000,true));
    voice_sample(&v,1,1,1,940000,false);
    assert(v.armed && !v.active);
    voice_sample(&v,0,0,0,950000,false);
    voice_sample(&v,0,0,0,980000,false);
    assert(v.active);
    /* Feed decoded voice motion into the same natural mood model as phone. */
    for (unsigned c=0;c<4;c++) {
        mood_t mood={0};
        v.command=c;
        voice_velocity(&v,&y,&w);
        mood_step(&mood,.01f,true,w/REMOTE_MAX_ROTATION);
        assert(mood_face(&mood,FACE_AUTO,false,true)==FACE_COOL);
        for(int i=0;i<510;i++) mood_step(&mood,.01f,true,w/REMOTE_MAX_ROTATION);
        assert(mood_face(&mood,FACE_AUTO,false,true)==
               ((c==1||c==2)?FACE_DIZZY:FACE_COOL));
        for(int i=0;i<400;i++) mood_step(&mood,.01f,false,0);
        assert(!mood.dizzy_latched);
        assert(mood_face(&mood,FACE_AUTO,false,false)==FACE_HAPPY);
    }
    puts("PASS: voice mapping, boot interlock, settling, stop, phone takeover and rearm");
}
