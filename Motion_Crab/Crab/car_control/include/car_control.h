#ifndef CAR_CONTROL_H
#define CAR_CONTROL_H

/**
 * @file car_control.h
 * @brief Three-wheel omni-drive chassis control.
 *
 * Chassis coordinates: +x is right, +y is the infrared-module-facing front,
 * and positive angular velocity is counter-clockwise. The MPU6050 convention
 * supplied for this car is therefore: clockwise turn -> gx < 0.
 *
 * Wheels are placed at alternating sides of a regular hexagon:
 *   A: rear, B: left-front, D: right-front.
 * The positive tangential wheel directions are counter-clockwise around the
 * chassis centre. motor_polarity compensates for the actual motor wiring.
 */

#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Fixed board wiring used by the chassis-level application. */
#define CAR_IR_OUT1_GPIO       42
#define CAR_IR_OUT2_GPIO       41
#define CAR_IR_OUT3_GPIO       40
#define CAR_IR_OUT4_GPIO       39
#define CAR_MPU6050_SDA_GPIO   1
#define CAR_MPU6050_SCL_GPIO   2

typedef enum {
    CAR_WHEEL_A = 0,  /**< Rear wheel */
    CAR_WHEEL_B,      /**< Left-front wheel */
    CAR_WHEEL_D,      /**< Right-front wheel */
    CAR_WHEEL_COUNT,
} car_wheel_t;

typedef struct {
    int pwm_gpio;
    int in1_gpio;
    int in2_gpio;
    /** +1 if IN1=1/IN2=0 drives the defined positive wheel direction;
     *  -1 if the physical motor is wired in the opposite direction. */
    int8_t motor_polarity;
} car_wheel_config_t;

typedef struct {
    car_wheel_config_t wheel[CAR_WHEEL_COUNT];
    float centre_to_wheel_m;
    float max_wheel_speed_mps;
    uint32_t pwm_frequency_hz;
    uint8_t pwm_resolution_bits;
} car_control_config_t;

/** Default board pin mapping. Adjust motor_polarity after a no-load test. */
#define CAR_CONTROL_DEFAULT_CONFIG() { \
    .wheel = { \
        [CAR_WHEEL_A] = { .pwm_gpio = 4,  .in1_gpio = 6,  .in2_gpio = 5,  .motor_polarity = 1 }, \
        [CAR_WHEEL_B] = { .pwm_gpio = 14, .in1_gpio = 12, .in2_gpio = 13, .motor_polarity = 1 }, \
        [CAR_WHEEL_D] = { .pwm_gpio = 16, .in1_gpio = 18, .in2_gpio = 17, .motor_polarity = -1 }, \
    }, \
    .centre_to_wheel_m = 0.100f, \
    .max_wheel_speed_mps = 0.50f, \
    .pwm_frequency_hz = 20000, \
    .pwm_resolution_bits = 10, \
}

/** Initialise the A/B/D motor outputs and their PWM channels. */
esp_err_t car_control_init(const car_control_config_t *config);

/**
 * Set a single tangential wheel speed in m/s.
 * Positive is the counter-clockwise tangential direction defined above.
 */
esp_err_t car_control_set_wheel_speed(car_wheel_t wheel, float speed_mps);

/**
 * General planar chassis command.
 * vx_mps: rightward velocity; vy_mps: forward velocity;
 * omega_rad_s: positive counter-clockwise, negative clockwise.
 */
esp_err_t car_control_set_velocity(float vx_mps, float vy_mps,
                                   float omega_rad_s);

/** Drive straight in the infrared-module-facing direction; negative reverses. */
esp_err_t car_control_forward(float speed_mps);

/**
 * Translate sideways using the empirically calibrated side-wheel ratio.
 * speed_mps > 0 moves left; speed_mps < 0 moves right.
 */
esp_err_t car_control_strafe_left(float speed_mps);

/** Rotate about the chassis centre; negative is clockwise and should make gx < 0. */
esp_err_t car_control_turn_in_place(float omega_rad_s);

/** Stop every wheel with zero PWM and both direction pins low. */
esp_err_t car_control_stop(void);

#ifdef __cplusplus
}
#endif

#endif  // CAR_CONTROL_H
