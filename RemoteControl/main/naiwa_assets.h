#pragma once
#include <stdint.h>
#define NAIWA_FRAMES 24
#define NAIWA_PIXELS (128*140)
extern const uint16_t naiwa_palette[256];
extern const uint32_t naiwa_offsets[25];
extern const uint8_t naiwa_rle[287512];
