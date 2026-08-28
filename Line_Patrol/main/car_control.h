#ifndef CAR_CONTROL_H
#define CAR_CONTROL_H

#include <stdint.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

#define CAR_IR_OUT1_GPIO 42
#define CAR_IR_OUT2_GPIO 41
#define CAR_IR_OUT3_GPIO 40
#define CAR_IR_OUT4_GPIO 39
#define CAR_MPU6050_SDA_GPIO 1
#define CAR_MPU6050_SCL_GPIO 2

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
    int8_t motor_polarity;
} car_wheel_config_t;

typedef struct {
    car_wheel_config_t wheel[CAR_WHEEL_COUNT];
    float centre_to_wheel_m;
    float max_wheel_speed_mps;
    uint32_t pwm_frequency_hz;
    uint8_t pwm_resolution_bits;
} car_control_config_t;

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

esp_err_t car_control_init(const car_control_config_t *config);
esp_err_t car_control_set_wheel_speed(car_wheel_t wheel, float speed_mps);
esp_err_t car_control_set_velocity(float vx_mps, float vy_mps,
                                   float omega_rad_s);
esp_err_t car_control_forward(float speed_mps);
esp_err_t car_control_strafe_left(float speed_mps);
esp_err_t car_control_turn_in_place(float omega_rad_s);
esp_err_t car_control_stop(void);

#ifdef __cplusplus
}
#endif

#endif
