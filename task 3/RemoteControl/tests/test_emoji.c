#include <assert.h>
#include <stdio.h>
#include "../main/emoji_render.h"
#include "../main/remote_state.h"
int main(int argc,char **argv) {
    assert(face_choose(FACE_AUTO,false,false,false)==FACE_SLEEP);
    assert(face_choose(FACE_AUTO,false,true,false)==FACE_COOL);
    assert(face_choose(FACE_AUTO,false,true,true)==FACE_DIZZY);
    assert(face_choose(FACE_HEART,false,true,true)==FACE_HEART);
    assert(face_choose(FACE_HEART,true,true,true)==FACE_OFFLINE);
    remote_state_t s={0};remote_claim(&s,42,0);
    for(int i=0;i<6;i++)face_choose(i,false,true,false);
    remote_expire(&s,REMOTE_TIMEOUT_US);assert(!s.token);
    if(argc>1) {
        FILE *f=fopen(argv[1],"wb");assert(f);
        fprintf(f,"P6\n768 160\n255\n");
        for(int y=0;y<160;y++)for(int x=0;x<768;x++) {
            uint16_t p=face_pixel(x/128,x%128,y,false,true);
            unsigned char rgb[3]={((p>>11)&31)*255/31,((p>>5)&63)*255/63,(p&31)*255/31};
            fwrite(rgb,1,3,f);
        }
        fclose(f);
    }
    puts("PASS: automatic/manual expressions, fault priority, watchdog isolation");
}
