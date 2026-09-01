#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Start the private Wi-Fi access point and control page. The HTTP service is
 * kept available even when frame transmission is disabled, so the user can
 * enable it from a browser at any time.
 */
esp_err_t camera_stream_server_init(bool enabled_by_default);

/** Enable or disable JPEG copying and transmission at runtime. */
void camera_stream_server_set_enabled(bool enabled);

/** Return whether JPEG transmission is currently enabled. */
bool camera_stream_server_is_enabled(void);

/* Copies one complete JPEG only while transmission is enabled. */
void camera_stream_server_publish_jpeg(const uint8_t *jpeg, size_t length);

/** Publish the four Camera_Lazy-style virtual infrared channels for the
 * browser status panel. Patterns use bit 3 for image-left through bit 0 for
 * image-right. */
void camera_stream_server_publish_virtual_ir(
    uint8_t raw_pattern, uint8_t filtered_pattern,
    const uint8_t black_percent[4], float error);

#ifdef __cplusplus
}
#endif
