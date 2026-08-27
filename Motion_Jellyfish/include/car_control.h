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
    int encoder_a_gpio;
    int encoder_b_gpio;
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

/**
 * Default board pin mapping. The encoder feedback uses relative wheel travel,
 * so encoder A/B polarity does not need to be calibrated for straightening.
 * Adjust motor_polarity after a no-load motor-direction test if necessary.
 */
#define CAR_CONTROL_DEFAULT_CONFIG() { \
    /* Existing project mapping is retained: A->PWMB/E2, B->PWMD/E4, D->PWMA/E1. */ \
    .wheel = { \
        [CAR_WHEEL_A] = { .pwm_gpio = 4,  .in1_gpio = 6,  .in2_gpio = 5,  .encoder_a_gpio = 7,  .encoder_b_gpio = 15, .motor_polarity = 1 }, \
        [CAR_WHEEL_B] = { .pwm_gpio = 14, .in1_gpio = 12, .in2_gpio = 13, .encoder_a_gpio = 11, .encoder_b_gpio = 10, .motor_polarity = 1 }, \
        [CAR_WHEEL_D] = { .pwm_gpio = 16, .in1_gpio = 18, .in2_gpio = 17, .encoder_a_gpio = 8,  .encoder_b_gpio = 9,  .motor_polarity = -1 }, \
    }, \
    .centre_to_wheel_m = 0.100f, \
    .max_wheel_speed_mps = 0.50f, \
    .pwm_frequency_hz = 20000, \
    .pwm_resolution_bits = 10, \
}

/**
 * Initialise the A/B/D motor outputs, EiA/EiB quadrature inputs, PWM
 * channels, and MPU6050 on GPIO1/2.
 * The MPU6050 must be stationary during this call so that gx bias can be
 * measured for the 30-degree turn controller.
 */
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

/** Drive straight backwards. speed_mps must be non-negative. */
esp_err_t car_control_backward(float speed_mps);

/**
 * Turn clockwise 30 degrees using gx feedback, then drive rear A and left B.
 * speed_mps must be non-negative. Call car_control_finish_strafe() when the
 * desired leftward travel is complete.
 */
esp_err_t car_control_strafe_left(float speed_mps);

/**
 * Turn counter-clockwise 30 degrees using gx feedback, then drive rear A and
 * right D. speed_mps must be non-negative. Call car_control_finish_strafe()
 * when the desired rightward travel is complete.
 */
esp_err_t car_control_strafe_right(float speed_mps);

/** Stop an active strafe and use gx feedback to restore the original heading. */
esp_err_t car_control_finish_strafe(void);

/** Rotate about the chassis centre; negative is clockwise and should make gx < 0. */
esp_err_t car_control_turn_in_place(float omega_rad_s);

/** Stop every wheel immediately. */
esp_err_t car_control_stop(void);

#ifdef __cplusplus
}
#endif

#endif  // CAR_CONTROL_H
