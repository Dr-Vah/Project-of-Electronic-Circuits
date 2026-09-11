#pragma once
#include <stdbool.h>
#include <stdint.h>
/* Output is 128x160 RGB565 in the TFT driver's wire byte order. */
void naiwa_render(uint16_t *frame, int face, unsigned phase, bool linked);
