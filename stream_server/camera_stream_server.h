#pragma once

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

#ifdef __cplusplus
}
#endif
