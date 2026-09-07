#include "car_control.h"

#include <math.h>
#include <stdbool.h>
#include <string.h>

#include "driver/gpio.h"
#include "driver/ledc.h"
#include "driver/pulse_cnt.h"
#include "esp_check.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"

#define SQRT3_OVER_2 0.8660254037844386f
#define PID_PERIOD_S 0.01f
#define SPEED_FILTER_ALPHA 0.20f

static car_control_config_t s_config;
static bool s_ready;
static uint32_t s_max_duty;
static pcnt_unit_handle_t s_pcnt_units[CAR_WHEEL_COUNT];
static esp_timer_handle_t s_pid_timer;
static float s_target_speed_mps[CAR_WHEEL_COUNT];
static float s_measured_speed_mps[CAR_WHEEL_COUNT];
static float s_pid_integral[CAR_WHEEL_COUNT];
static float s_last_error[CAR_WHEEL_COUNT];
static portMUX_TYPE s_speed_lock = portMUX_INITIALIZER_UNLOCKED;

static float clampf(float value, float low, float high)
{
    return value < low ? low : (value > high ? high : value);
}

static void set_motor_hardware(car_wheel_t wheel, float control_effort)
{
    const car_wheel_config_t *motor = &s_config.wheel[wheel];
    const float limited = clampf(control_effort, -s_config.max_wheel_speed_mps,
                                 s_config.max_wheel_speed_mps);

    if (fabsf(limited) < 0.0001f) {
        ledc_set_duty(LEDC_LOW_SPEED_MODE, (ledc_channel_t)wheel, 0);
        ledc_update_duty(LEDC_LOW_SPEED_MODE, (ledc_channel_t)wheel);
        gpio_set_level(motor->in1_gpio, 0);
        gpio_set_level(motor->in2_gpio, 0);
        return;
    }

    const bool positive = (limited * motor->motor_polarity) > 0.0f;
    gpio_set_level(motor->in1_gpio, positive ? 1 : 0);
    gpio_set_level(motor->in2_gpio, positive ? 0 : 1);

    const uint32_t duty = (uint32_t)lroundf(
        fabsf(limited) / s_config.max_wheel_speed_mps * s_max_duty);
    ledc_set_duty(LEDC_LOW_SPEED_MODE, (ledc_channel_t)wheel, duty);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, (ledc_channel_t)wheel);
}

static void pid_timer_callback(void *arg)
{
    (void)arg;
    for (int i = 0; i < CAR_WHEEL_COUNT; ++i) {
        int pulse_count = 0;
        pcnt_unit_get_count(s_pcnt_units[i], &pulse_count);
        pcnt_unit_clear_count(s_pcnt_units[i]);

        float current_speed =
            (pulse_count / s_config.ticks_per_meter) / PID_PERIOD_S;
        current_speed *= s_config.wheel[i].motor_polarity;
        portENTER_CRITICAL(&s_speed_lock);
        s_measured_speed_mps[i] +=
            SPEED_FILTER_ALPHA * (current_speed - s_measured_speed_mps[i]);
        portEXIT_CRITICAL(&s_speed_lock);

        if (fabsf(s_target_speed_mps[i]) < 0.001f) {
            set_motor_hardware((car_wheel_t)i, 0.0f);
            s_pid_integral[i] = 0.0f;
            s_last_error[i] = 0.0f;
            continue;
        }

        const float error = s_target_speed_mps[i] - current_speed;
        s_pid_integral[i] += error * PID_PERIOD_S;
        s_pid_integral[i] = clampf(s_pid_integral[i], -0.08f, 0.08f);
        const float derivative = (error - s_last_error[i]) / PID_PERIOD_S;
        s_last_error[i] = error;

        float control_effort = s_config.pid_kp * error +
                               s_config.pid_ki * s_pid_integral[i] +
                               s_config.pid_kd * derivative +
                               s_target_speed_mps[i];
        if (s_target_speed_mps[i] > 0.001f && control_effort < 0.0f) {
            control_effort = 0.0f;
        } else if (s_target_speed_mps[i] < -0.001f && control_effort > 0.0f) {
            control_effort = 0.0f;
        }
        set_motor_hardware((car_wheel_t)i, control_effort);
    }
}

esp_err_t car_control_init(const car_control_config_t *config)
{
    if (config == NULL || config->centre_to_wheel_m <= 0.0f ||
        config->max_wheel_speed_mps <= 0.0f || config->pwm_frequency_hz == 0 ||
        config->pwm_resolution_bits == 0 || config->pwm_resolution_bits > 14 ||
        config->ticks_per_meter <= 0.0f) {
        return ESP_ERR_INVALID_ARG;
    }

    memcpy(&s_config, config, sizeof(s_config));
    s_max_duty = (1U << s_config.pwm_resolution_bits) - 1U;

    const ledc_timer_config_t timer = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution = (ledc_timer_bit_t)s_config.pwm_resolution_bits,
        .timer_num = LEDC_TIMER_0,
        .freq_hz = s_config.pwm_frequency_hz,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    ESP_RETURN_ON_ERROR(ledc_timer_config(&timer), "car_control",
                        "configure PWM timer failed");

    for (int i = 0; i < CAR_WHEEL_COUNT; ++i) {
        const car_wheel_config_t *motor = &s_config.wheel[i];
        if (motor->motor_polarity != 1 && motor->motor_polarity != -1) {
            return ESP_ERR_INVALID_ARG;
        }

        const gpio_config_t direction_pins = {
            .pin_bit_mask = (1ULL << motor->in1_gpio) |
                            (1ULL << motor->in2_gpio),
            .mode = GPIO_MODE_OUTPUT,
            .pull_up_en = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_DISABLE,
        };
        ESP_RETURN_ON_ERROR(gpio_config(&direction_pins), "car_control",
                            "configure direction pins failed");

        const ledc_channel_config_t channel = {
            .gpio_num = motor->pwm_gpio,
            .speed_mode = LEDC_LOW_SPEED_MODE,
            .channel = (ledc_channel_t)i,
            .intr_type = LEDC_INTR_DISABLE,
            .timer_sel = LEDC_TIMER_0,
            .duty = 0,
            .hpoint = 0,
            .flags.output_invert = 0,
        };
        ESP_RETURN_ON_ERROR(ledc_channel_config(&channel), "car_control",
                            "configure PWM channel failed");

        const pcnt_unit_config_t unit_config = {
            .high_limit = 30000,
            .low_limit = -30000,
        };
        ESP_RETURN_ON_ERROR(pcnt_new_unit(&unit_config, &s_pcnt_units[i]),
                            "car_control", "init PCNT failed");
        const pcnt_glitch_filter_config_t filter_config = {
            .max_glitch_ns = 1000,
        };
        ESP_RETURN_ON_ERROR(
            pcnt_unit_set_glitch_filter(s_pcnt_units[i], &filter_config),
            "car_control", "set PCNT filter failed");

        const pcnt_chan_config_t chan_a_config = {
            .edge_gpio_num = motor->ea_gpio,
            .level_gpio_num = motor->eb_gpio,
        };
        pcnt_channel_handle_t pcnt_chan_a = NULL;
        ESP_RETURN_ON_ERROR(
            pcnt_new_channel(s_pcnt_units[i], &chan_a_config, &pcnt_chan_a),
            "car_control", "init PCNT channel A failed");
        ESP_RETURN_ON_ERROR(
            pcnt_channel_set_edge_action(pcnt_chan_a,
                                         PCNT_CHANNEL_EDGE_ACTION_INCREASE,
                                         PCNT_CHANNEL_EDGE_ACTION_DECREASE),
            "car_control", "set PCNT edge A failed");
        ESP_RETURN_ON_ERROR(
            pcnt_channel_set_level_action(pcnt_chan_a,
                                          PCNT_CHANNEL_LEVEL_ACTION_KEEP,
                                          PCNT_CHANNEL_LEVEL_ACTION_INVERSE),
            "car_control", "set PCNT level A failed");

        const pcnt_chan_config_t chan_b_config = {
            .edge_gpio_num = motor->eb_gpio,
            .level_gpio_num = motor->ea_gpio,
        };
        pcnt_channel_handle_t pcnt_chan_b = NULL;
        ESP_RETURN_ON_ERROR(
            pcnt_new_channel(s_pcnt_units[i], &chan_b_config, &pcnt_chan_b),
            "car_control", "init PCNT channel B failed");
        ESP_RETURN_ON_ERROR(
            pcnt_channel_set_edge_action(pcnt_chan_b,
                                         PCNT_CHANNEL_EDGE_ACTION_INCREASE,
                                         PCNT_CHANNEL_EDGE_ACTION_DECREASE),
            "car_control", "set PCNT edge B failed");
        ESP_RETURN_ON_ERROR(
            pcnt_channel_set_level_action(pcnt_chan_b,
                                          PCNT_CHANNEL_LEVEL_ACTION_INVERSE,
                                          PCNT_CHANNEL_LEVEL_ACTION_KEEP),
            "car_control", "set PCNT level B failed");

        ESP_RETURN_ON_ERROR(gpio_set_pull_mode((gpio_num_t)motor->ea_gpio,
                                               GPIO_PULLUP_ONLY),
                            "car_control", "set encoder A pull-up failed");
        ESP_RETURN_ON_ERROR(gpio_set_pull_mode((gpio_num_t)motor->eb_gpio,
                                               GPIO_PULLUP_ONLY),
                            "car_control", "set encoder B pull-up failed");
        ESP_RETURN_ON_ERROR(pcnt_unit_enable(s_pcnt_units[i]), "car_control",
                            "enable PCNT failed");
        ESP_RETURN_ON_ERROR(pcnt_unit_clear_count(s_pcnt_units[i]),
                            "car_control", "clear PCNT failed");
        ESP_RETURN_ON_ERROR(pcnt_unit_start(s_pcnt_units[i]), "car_control",
                            "start PCNT failed");
    }

    const esp_timer_create_args_t timer_args = {
        .callback = pid_timer_callback,
        .name = "pid_loop",
    };
    ESP_RETURN_ON_ERROR(esp_timer_create(&timer_args, &s_pid_timer),
                        "car_control", "create PID timer failed");
    ESP_RETURN_ON_ERROR(
        esp_timer_start_periodic(s_pid_timer,
                                 (uint64_t)(PID_PERIOD_S * 1000000.0f)),
        "car_control", "start PID timer failed");
    s_ready = true;
    return car_control_stop();
}

esp_err_t car_control_set_wheel_speed(car_wheel_t wheel, float speed_mps)
{
    if (!s_ready) {
        return ESP_ERR_INVALID_STATE;
    }
    if (wheel < CAR_WHEEL_A || wheel >= CAR_WHEEL_COUNT ||
        !isfinite(speed_mps)) {
        return ESP_ERR_INVALID_ARG;
    }

    s_target_speed_mps[wheel] =
        clampf(speed_mps, -s_config.max_wheel_speed_mps,
               s_config.max_wheel_speed_mps);
    return ESP_OK;
}

esp_err_t car_control_get_wheel_speeds(float speed_mps[CAR_WHEEL_COUNT])
{
    if (!s_ready) {
        return ESP_ERR_INVALID_STATE;
    }
    if (speed_mps == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    portENTER_CRITICAL(&s_speed_lock);
    memcpy(speed_mps, s_measured_speed_mps, sizeof(s_measured_speed_mps));
    portEXIT_CRITICAL(&s_speed_lock);
    return ESP_OK;
}

esp_err_t car_control_set_velocity(float vx_mps, float vy_mps,
                                   float omega_rad_s)
{
    if (!s_ready) {
        return ESP_ERR_INVALID_STATE;
    }
    if (!isfinite(vx_mps) || !isfinite(vy_mps) ||
        !isfinite(omega_rad_s)) {
        return ESP_ERR_INVALID_ARG;
    }

    const float rotation = s_config.centre_to_wheel_m * omega_rad_s;
    const float wheel_speed[CAR_WHEEL_COUNT] = {
        0.5f * vx_mps - SQRT3_OVER_2 * vy_mps - rotation,
        vx_mps + rotation,
        0.5f * vx_mps + SQRT3_OVER_2 * vy_mps - rotation,
    };

    for (int i = 0; i < CAR_WHEEL_COUNT; ++i) {
        ESP_RETURN_ON_ERROR(
            car_control_set_wheel_speed((car_wheel_t)i, wheel_speed[i]),
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
    return car_control_set_velocity(-speed_mps, 0.0f, 0.0f);
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
        s_target_speed_mps[i] = 0.0f;
        s_pid_integral[i] = 0.0f;
        s_last_error[i] = 0.0f;
    }
    return ESP_OK;
}

