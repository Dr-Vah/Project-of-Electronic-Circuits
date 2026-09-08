#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "../main/emoji_render.h"
#include "../main/naiwa_assets.h"
#include "../main/naiwa_render.h"
static struct {uint16_t before;uint16_t pixels[128*160];uint16_t after;} buffer;
static uint16_t previous[128*160];
int main(void) {
    assert(naiwa_offsets[NAIWA_FRAMES]==sizeof(naiwa_rle));
    for(int f=0;f<NAIWA_FRAMES;f++) {
        unsigned pixels=0;
        assert(naiwa_offsets[f]<=naiwa_offsets[f+1]);
        assert((naiwa_offsets[f+1]-naiwa_offsets[f])%2==0);
        for(unsigned i=naiwa_offsets[f];i<naiwa_offsets[f+1];i+=2) {
            assert(naiwa_rle[i]>0);
            pixels+=naiwa_rle[i];
        }
        assert(pixels==NAIWA_PIXELS);
        buffer.before=0x1234;buffer.after=0xabcd;
        naiwa_render(buffer.pixels,f/2,f%2,true);
        assert(buffer.before==0x1234&&buffer.after==0xabcd);
        assert(buffer.pixels[146*128+20]==0x196f);
        if(f%2)assert(memcmp(previous,buffer.pixels,sizeof(previous))!=0);
        memcpy(previous,buffer.pixels,sizeof(previous));
    }
    naiwa_render(buffer.pixels,-1,0,false);
    assert(buffer.pixels[146*128+20]==0x86f9);
    puts("PASS: all 24 RLE frames, distinct animation phases, bounds, RGB565 wire order, link indicator");
}
