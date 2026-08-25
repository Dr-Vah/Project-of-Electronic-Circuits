#include "car_control.h"

#include <math.h>
#include <stdbool.h>
#include <string.h>

#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_check.h"

#define SQRT3_OVER_2 0.8660254037844386f
#define STRAFE_SIDE_SPEED_RATIO 0.7f

static car_control_config_t s_config;
static bool s_ready;
static uint32_t s_max_duty;

static float clampf(float value, float low, float high)
{
    return value < low ? low : (value > high ? high : value);
}

static esp_err_t stop_wheel(car_wheel_t wheel)
{
    const car_wheel_config_t *motor = &s_config.wheel[wheel];
    ESP_RETURN_ON_ERROR(ledc_set_duty(LEDC_LOW_SPEED_MODE, (ledc_channel_t)wheel, 0),
                        "car_control", "set PWM duty failed");
    ESP_RETURN_ON_ERROR(ledc_update_duty(LEDC_LOW_SPEED_MODE, (ledc_channel_t)wheel),
                        "car_control", "update PWM duty failed");
    gpio_set_level(motor->in1_gpio, 0);
    gpio_set_level(motor->in2_gpio, 0);
    return ESP_OK;
}

esp_err_t car_control_init(const car_control_config_t *config)
{
    if (config == NULL || config->centre_to_wheel_m <= 0.0f ||
        config->max_wheel_speed_mps <= 0.0f || config->pwm_frequency_hz == 0 ||
        config->pwm_resolution_bits == 0 || config->pwm_resolution_bits > 14) {
        return ESP_ERR_INVALID_ARG;
    }

    memcpy(&s_config, config, sizeof(s_config));
    s_max_duty = (1U << s_config.pwm_resolution_bits) - 1U;

    ledc_timer_config_t timer = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution = (ledc_timer_bit_t)s_config.pwm_resolution_bits,
        .timer_num = LEDC_TIMER_0,
        .freq_hz = s_config.pwm_frequency_hz,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    ESP_RETURN_ON_ERROR(ledc_timer_config(&timer), "car_control", "configure PWM timer failed");

    for (int i = 0; i < CAR_WHEEL_COUNT; ++i) {
        const car_wheel_config_t *motor = &s_config.wheel[i];
        if (motor->motor_polarity != 1 && motor->motor_polarity != -1) {
            return ESP_ERR_INVALID_ARG;
        }

        gpio_config_t direction_pins = {
            .pin_bit_mask = (1ULL << motor->in1_gpio) | (1ULL << motor->in2_gpio),
            .mode = GPIO_MODE_OUTPUT,
            .pull_up_en = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_DISABLE,
        };
        ESP_RETURN_ON_ERROR(gpio_config(&direction_pins), "car_control", "configure direction pins failed");

        ledc_channel_config_t channel = {
            .gpio_num = motor->pwm_gpio,
            .speed_mode = LEDC_LOW_SPEED_MODE,
            .channel = (ledc_channel_t)i,
            .intr_type = LEDC_INTR_DISABLE,
            .timer_sel = LEDC_TIMER_0,
            .duty = 0,
            .hpoint = 0,
            .flags.output_invert = 0,
        };
        ESP_RETURN_ON_ERROR(ledc_channel_config(&channel), "car_control", "configure PWM channel failed");
    }

    s_ready = true;
    return car_control_stop();
}

esp_err_t car_control_set_wheel_speed(car_wheel_t wheel, float speed_mps)
{
    if (!s_ready) {
        return ESP_ERR_INVALID_STATE;
    }
    if (wheel < CAR_WHEEL_A || wheel >= CAR_WHEEL_COUNT || !isfinite(speed_mps)) {
        return ESP_ERR_INVALID_ARG;
    }

    const car_wheel_config_t *motor = &s_config.wheel[wheel];
    const float limited = clampf(speed_mps, -s_config.max_wheel_speed_mps,
                                 s_config.max_wheel_speed_mps);
    if (fabsf(limited) < 0.0001f) {
        return stop_wheel(wheel);
    }

    const bool positive = (limited * motor->motor_polarity) > 0.0f;
    gpio_set_level(motor->in1_gpio, positive ? 1 : 0);
    gpio_set_level(motor->in2_gpio, positive ? 0 : 1);

    const uint32_t duty = (uint32_t)lroundf(fabsf(limited) /
                                             s_config.max_wheel_speed_mps * s_max_duty);
    ESP_RETURN_ON_ERROR(ledc_set_duty(LEDC_LOW_SPEED_MODE, (ledc_channel_t)wheel, duty),
                        "car_control", "set PWM duty failed");
    return ledc_update_duty(LEDC_LOW_SPEED_MODE, (ledc_channel_t)wheel);
}

esp_err_t car_control_set_velocity(float vx_mps, float vy_mps, float omega_rad_s)
{
    if (!s_ready) {
        return ESP_ERR_INVALID_STATE;
    }
    if (!isfinite(vx_mps) || !isfinite(vy_mps) || !isfinite(omega_rad_s)) {
        return ESP_ERR_INVALID_ARG;
    }

    const float rotation = s_config.centre_to_wheel_m * omega_rad_s;
    const float wheel_speed[CAR_WHEEL_COUNT] = {
        vx_mps + rotation,
        -0.5f * vx_mps - SQRT3_OVER_2 * vy_mps + rotation,
        -0.5f * vx_mps + SQRT3_OVER_2 * vy_mps + rotation,
    };

    for (int i = 0; i < CAR_WHEEL_COUNT; ++i) {
        ESP_RETURN_ON_ERROR(car_control_set_wheel_speed((car_wheel_t)i, wheel_speed[i]),
                            "car_control", "set wheel speed failed");
    }
    return ESP_OK;
}

esp_err_t car_control_forward(float speed_mps)
{
    return car_control_set_velocity(0.0f, speed_mps, 0.0f);
}

esp_err_t car_control_strafe_left(float speed_mps)
{
    if (!isfinite(speed_mps)) {
        return ESP_ERR_INVALID_ARG;
    }

    /*
     * Keep the wheel-axis signs that produce the same chassis x direction,
     * with both side wheels running at half the rear-wheel speed.
     */
    ESP_RETURN_ON_ERROR(car_control_set_wheel_speed(CAR_WHEEL_A, -speed_mps),
                        "car_control", "set rear wheel speed failed");
    ESP_RETURN_ON_ERROR(car_control_set_wheel_speed(CAR_WHEEL_B,
                                                    STRAFE_SIDE_SPEED_RATIO * speed_mps),
                        "car_control", "set left wheel speed failed");
    return car_control_set_wheel_speed(CAR_WHEEL_D,
                                       STRAFE_SIDE_SPEED_RATIO * speed_mps);
}

esp_err_t car_control_turn_in_place(float omega_rad_s)
{
    return car_control_set_velocity(0.0f, 0.0f, omega_rad_s);
}

esp_err_t car_control_stop(void)
{
    if (!s_ready) {
        return ESP_ERR_INVALID_STATE;
    }

    for (int i = 0; i < CAR_WHEEL_COUNT; ++i) {
        ESP_RETURN_ON_ERROR(stop_wheel((car_wheel_t)i), "car_control", "stop wheel failed");
    }
    return ESP_OK;
}
