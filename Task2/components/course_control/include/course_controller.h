#ifndef COURSE_CONTROLLER_H
#define COURSE_CONTROLLER_H

#include <stdbool.h>

#include "esp_err.h"
#include "ultrasonic_sensor.h"

#ifdef __cplusplus
extern "C" {
#endif

/** The physical-course requirement: obstacle avoidance starts at 5 cm. */
#define TASK2_OBSTACLE_TRIGGER_CM 5.0f

esp_err_t course_controller_start(void);

/** Copy the most recent measurement taken by the controller's sensor loop. */
bool course_controller_get_ultrasonic_sample(ultrasonic_sample_t *sample);

#ifdef __cplusplus
}
#endif

#endif
