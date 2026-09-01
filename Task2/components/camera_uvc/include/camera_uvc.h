#ifndef CAMERA_UVC_H
#define CAMERA_UVC_H

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"
#include "line_vision_control.hpp"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    bool frame_received;
    uint32_t sequence;
    int64_t frame_timestamp_us;
    line_vision_control_result_t line;
} camera_vision_sample_t;

/** Start USB host, UVC receive and the JPEG/vision processing task. */
esp_err_t camera_uvc_start(void);

/** Copy the latest complete vision sample. Returns false before the first frame. */
bool camera_uvc_get_latest(camera_vision_sample_t *sample);

#ifdef __cplusplus
}
#endif

#endif
