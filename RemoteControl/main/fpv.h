#pragma once
#include "esp_err.h"
/* Optional camera/video service. Failures do not disable remote driving. */
esp_err_t fpv_start(void);
