#include <assert.h>
#include <stdio.h>
#include "../main/mood.h"
static void simulate(mood_t *m,float seconds,bool moving,float turn) {
    for(int i=0;i<(int)(seconds*100);i++)mood_step(m,.01f,moving,turn);
}
int main(void) {
    mood_t m={0};assert(mood_face(&m,FACE_AUTO,false,false)==FACE_HAPPY);
    simulate(&m,4.1f,true,.5f);assert(m.dizzy_latched);
    assert(mood_face(&m,FACE_AUTO,false,true)==FACE_DIZZY);
    simulate(&m,2,false,0);assert(m.dizzy_latched);
    simulate(&m,2,false,0);assert(!m.dizzy_latched);
    assert(mood_face(&m,FACE_HEART,true,false)==FACE_OFFLINE);
    assert(mood_face(&m,FACE_HEART,false,false)==FACE_HEART);
    m=(mood_t){0};simulate(&m,6,false,0);assert(mood_face(&m,FACE_AUTO,false,false)==FACE_CURIOUS);
    simulate(&m,10,false,0);assert(mood_face(&m,FACE_AUTO,false,false)==FACE_SLEEP);
    simulate(&m,.1f,true,0);assert(mood_face(&m,FACE_AUTO,false,true)==FACE_COOL);
    simulate(&m,40,true,0);assert(m.fatigue>=75);assert(mood_face(&m,FACE_AUTO,false,true)==FACE_TIRED);
    for(int face=0;face<FACE_COUNT;face++) {
        assert(mood_face(&m,face,false,true)==face);
        assert(mood_face(&m,face,true,true)==FACE_OFFLINE);
    }
    simulate(&m,25,false,0);assert(m.fatigue==0);
    simulate(&m,100,true,-1);assert(m.fatigue==100&&m.dizzy==100);
    mood_step(&m,NAN,true,1);assert(isfinite(m.fatigue));
    puts("PASS: accumulation, recovery, hysteresis, idle sleep, fatigue, fault/manual priority, bounds");
}
