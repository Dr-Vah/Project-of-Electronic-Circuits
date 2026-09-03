#ifndef BALL_TRANSPORT_CONTROLLER_H
#define BALL_TRANSPORT_CONTROLLER_H

#include <stddef.h>
#include <stdint.h>

#include "black_target_detector.h"
#include "esp_err.h"
#include "white_ball_detector.h"

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t ball_transport_controller_start(void);

/** Publish one processed camera frame to the 50 Hz safety/control task. */
void ball_transport_controller_submit(const white_ball_result_t *ball,
                                      const black_target_result_t *target,
                                      size_t frame_width,
                                      size_t frame_height,
                                      int64_t timestamp_us);

#ifdef __cplusplus
}
#endif

#endif
