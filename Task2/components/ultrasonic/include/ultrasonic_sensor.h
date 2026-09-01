#ifndef ULTRASONIC_SENSOR_H
#define ULTRASONIC_SENSOR_H

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/* GPIO17/18 are motor direction pins in CAR_CONTROL_DEFAULT_CONFIG().
 * HC-SR04 ECHO is 5 V: level-shift/divide it to 3.3 V before GPIO47. */
#define ULTRASONIC_DEFAULT_TRIG_GPIO 21
#define ULTRASONIC_DEFAULT_ECHO_GPIO 47

typedef struct {
    int trig_gpio;
    int echo_gpio;
    uint32_t echo_timeout_us;
    float minimum_cm;
    float maximum_cm;
} ultrasonic_sensor_config_t;

#define ULTRASONIC_SENSOR_DEFAULT_CONFIG() { \
    .trig_gpio = ULTRASONIC_DEFAULT_TRIG_GPIO, \
    .echo_gpio = ULTRASONIC_DEFAULT_ECHO_GPIO, \
    .echo_timeout_us = 30000, \
    .minimum_cm = 2.0f, \
    .maximum_cm = 500.0f, \
}

typedef struct {
    enum {
        ULTRASONIC_SAMPLE_OK,
        ULTRASONIC_WAIT_RISE_TIMEOUT,
        ULTRASONIC_PULSE_TIMEOUT,
        ULTRASONIC_OUT_OF_RANGE,
    } status;
    bool valid;
    float distance_cm;
    int64_t pulse_us;
    int64_t timestamp_us;
} ultrasonic_sample_t;

esp_err_t ultrasonic_sensor_init(const ultrasonic_sensor_config_t *config);

/**
 * Take one blocking measurement. A timeout or out-of-range echo is reported
 * as a successful, invalid sample; hardware/API failures return an error.
 */
esp_err_t ultrasonic_sensor_read(ultrasonic_sample_t *sample);

#ifdef __cplusplus
}
#endif

#endif  // ULTRASONIC_SENSOR_H
