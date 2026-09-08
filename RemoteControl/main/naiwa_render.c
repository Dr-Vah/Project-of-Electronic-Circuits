#include "naiwa_render.h"
#include "naiwa_assets.h"
#include "emoji_render.h"

_Static_assert(NAIWA_FRAMES == FACE_COUNT*2, "Missing expression frames");
static uint16_t wire(uint16_t p) { return (uint16_t)((p>>8)|(p<<8)); }
void naiwa_render(uint16_t *frame, int face, unsigned phase, bool linked) {
    if(face<0 || face>=FACE_COUNT)face=FACE_HAPPY;
    unsigned sprite=(unsigned)face*2+(phase&1u), pixel=0;
    for(unsigned i=0;i<128*160;i++)frame[i]=0xffff;
    for(uint32_t i=naiwa_offsets[sprite];i+1<naiwa_offsets[sprite+1];i+=2) {
        unsigned count=naiwa_rle[i], index=naiwa_rle[i+1];
        if(count>NAIWA_PIXELS-pixel)break;
        uint16_t color=wire(naiwa_palette[index]);
        while(count--)frame[pixel++]=color;
    }
    uint16_t indicator=wire(linked?0x6f19:0xf986);
    for(unsigned y=146;y<=149;y++)for(unsigned x=20;x<=107;x++)frame[y*128+x]=indicator;
}
