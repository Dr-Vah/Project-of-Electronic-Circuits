#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "ball_detector.h"
#include "esp_err.h"

esp_err_t wifi_stream_start(void);

/* Non-blocking publisher: a frame is skipped while an HTTP client is using
 * the shared JPEG buffer, so camera processing is never held up by Wi-Fi. */
void wifi_stream_publish_jpeg(const uint8_t *jpeg, size_t length,
                              uint16_t width, uint16_t height);

void wifi_stream_publish_detection(bool valid,
                                   const ball_detection_t *ball,
                                   uint16_t source_width,
                                   uint16_t source_height);
