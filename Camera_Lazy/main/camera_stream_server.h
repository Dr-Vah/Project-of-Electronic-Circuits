#pragma once

#include <stddef.h>
#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Starts a private Wi-Fi access point and an HTTP MJPEG preview server. */
esp_err_t camera_stream_server_init(void);

/* Copies one complete JPEG image. Safe to call from the frame task. */
void camera_stream_server_publish_jpeg(const uint8_t *jpeg, size_t length);

/* Analyse a decoded RGB565 frame. Outputs 1..4 correspond to image left..right. */
void camera_stream_server_analyse_rgb565(const uint16_t *pixels,
                                         uint16_t width, uint16_t height);

/* Snapshot for a later line-following main. active[0]..active[3] map to
 * the original OUT1..OUT4 order (image left to right). */
typedef struct {
    bool active[4];
    uint8_t black_percent[4];
    uint16_t centroid_x[4];
} camera_line_state_t;

bool camera_stream_server_get_line_state(camera_line_state_t *state);

#ifdef __cplusplus
}
#endif
