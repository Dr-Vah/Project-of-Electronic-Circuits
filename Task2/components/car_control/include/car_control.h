#ifndef CAR_CONTROL_H
#define CAR_CONTROL_H

/**
 * @file car_control.h
 * @brief Three-wheel omni-drive chassis control.
 *
 * Chassis coordinates: +x is right, +y is the camera-facing front,
 * and positive angular velocity is counter-clockwise.
 * Wheels: A is right-front, B is rear, and D is left-front.
 */

#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    CAR_WHEEL_A = 0,
    CAR_WHEEL_B,
    CAR_WHEEL_D,
    CAR_WHEEL_COUNT,
} car_wheel_t;

typedef struct {
    int pwm_gpio;
    int in1_gpio;
    int in2_gpio;
    int ea_gpio;
    int eb_gpio;
    int8_t motor_polarity;
} car_wheel_config_t;

typedef struct {
    car_wheel_config_t wheel[CAR_WHEEL_COUNT];
    float centre_to_wheel_m;
    float max_wheel_speed_mps;
    uint32_t pwm_frequency_hz;
    uint8_t pwm_resolution_bits;
    float ticks_per_meter;
    float pid_kp;
    float pid_ki;
    float pid_kd;
} car_control_config_t;

#define CAR_CONTROL_DEFAULT_CONFIG() { \
    .wheel = { \
        [CAR_WHEEL_A] = { .pwm_gpio = 16, .in1_gpio = 18, .in2_gpio = 17, .ea_gpio = 8,  .eb_gpio = 9,  .motor_polarity = 1 }, \
        [CAR_WHEEL_B] = { .pwm_gpio = 4,  .in1_gpio = 6,  .in2_gpio = 5,  .ea_gpio = 15, .eb_gpio = 7,  .motor_polarity = 1 }, \
        [CAR_WHEEL_D] = { .pwm_gpio = 14, .in1_gpio = 12, .in2_gpio = 13, .ea_gpio = 10, .eb_gpio = 11, .motor_polarity = -1 }, \
    }, \
    .centre_to_wheel_m = 0.100f, \
    .max_wheel_speed_mps = 1.0f, \
    .pwm_frequency_hz = 20000, \
    .pwm_resolution_bits = 10, \
    .ticks_per_meter = 6000.0f, \
    .pid_kp = 2.0f, \
    .pid_ki = 4.0f, \
    .pid_kd = 0.0f, \
}

esp_err_t car_control_init(const car_control_config_t *config);
esp_err_t car_control_set_wheel_speed(car_wheel_t wheel, float speed_mps);

/** Copy the latest encoder-measured wheel speeds, in chassis-positive m/s. */
esp_err_t car_control_get_wheel_speeds(float speed_mps[CAR_WHEEL_COUNT]);
esp_err_t car_control_set_velocity(float vx_mps, float vy_mps,
                                   float omega_rad_s);
esp_err_t car_control_forward(float speed_mps);

/** Direct sideways translation: positive moves left, negative moves right. */
esp_err_t car_control_strafe_left(float speed_mps);

esp_err_t car_control_turn_in_place(float omega_rad_s);
esp_err_t car_control_stop(void);

#ifdef __cplusplus
}
#endif

#endif  // CAR_CONTROL_H
