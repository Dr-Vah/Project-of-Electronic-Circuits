#pragma once
#include "esp_err.h"
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
/* Optional camera/video service. Failures do not disable remote driving. */
esp_err_t fpv_start(void);
/* Copy a fresh JPEG and metadata under the short camera lock. */
bool fpv_copy_frame(uint8_t *out,size_t cap,size_t *len,uint32_t *seq,uint32_t *age_ms);
