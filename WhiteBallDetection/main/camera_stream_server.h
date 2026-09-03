#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Starts a private Wi-Fi access point and an HTTP MJPEG preview server. */
esp_err_t camera_stream_server_init(void);

/* Copies one complete JPEG image. Safe to call from the frame task. */
void camera_stream_server_publish_jpeg(const uint8_t *jpeg, size_t length);

/* Publishes normalised boxes in the orientation of the returned raw JPEG. */
void camera_stream_server_publish_detection(
    bool ball_valid, float ball_left, float ball_top,
    float ball_right, float ball_bottom, float ball_confidence,
    bool target_valid, float target_left, float target_top,
    float target_right, float target_bottom);

#ifdef __cplusplus
}
#endif
