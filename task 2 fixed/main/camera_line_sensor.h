#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

#define CAMERA_LINE_CHANNEL_COUNT 4U

/* Channels are image-left to image-right. */
typedef struct {
    bool valid;
    bool black[CAMERA_LINE_CHANNEL_COUNT];
    uint8_t dark_percent[CAMERA_LINE_CHANNEL_COUNT];
    uint16_t centroid_x[CAMERA_LINE_CHANNEL_COUNT];
    /* -1..+1, positive means the line is image-right. Computed over the ROI
     * with both sides trimmed so sharp turns do not pull the centroid away. */
    float line_error;
    /* Dark pixel count inside the trimmed centroid ROI; 0 means no line. */
    uint32_t line_pixels;
} camera_line_state_t;

/* Starts USB-UVC capture on GPIO19/20.  The function returns after the USB
 * background tasks have been created; state remains invalid until a frame is
 * decoded successfully. */
esp_err_t camera_line_sensor_init(void);

/* Thread-safe copy of the most recently decoded four-channel state. */
bool camera_line_sensor_get_state(camera_line_state_t *state);

#ifdef __cplusplus
}
#endif
