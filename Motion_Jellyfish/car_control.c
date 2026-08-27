#include "car_control.h"

#include <math.h>
#include <stdbool.h>
#include <string.h>

#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "driver/ledc.h"
#include "esp_attr.h"
#include "esp_check.h"
#include "esp_intr_alloc.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#define SQRT3_OVER_2 0.8660254037844386f
#define MPU_ADDR_LOW                 0x68
#define MPU_ADDR_HIGH                0x69
#define MPU_REG_SMPLRT_DIV           0x19
#define MPU_REG_CONFIG               0x1A
#define MPU_REG_GYRO_CFG             0x1B
#define MPU_REG_PWR_MGMT_1           0x6B
#define MPU_REG_GYRO_XOUT_H          0x43
#define MPU_GYRO_SCALE_DPS           (250.0f / 32768.0f)
#define MPU_I2C_FREQ_HZ              (400 * 1000)
#define MPU_I2C_TIMEOUT_MS           100
#define MPU_READ_RETRIES             5
#define MPU_READ_RETRY_DELAY_MS      20
#define MPU_BIAS_SAMPLES             100
#define TURN_ANGLE_DEG               30.0f
#define TURN_RATE_RAD_S              0.50f
#define TURN_SAMPLE_MS               5
#define TURN_TIMEOUT_MS              4000
#define MPU_SETTLE_AFTER_MOTOR_MS    100
#define ENCODER_CONTROL_PERIOD_MS    20
#define ENCODER_MIN_PROGRESS_COUNTS  3.0f
#define ENCODER_CORRECTION_GAIN      1.25f
#define ENCODER_MAX_CORRECTION_RATIO 0.50f

static car_control_config_t s_config;
static bool s_ready;
static uint32_t s_max_duty;
static i2c_master_bus_handle_t s_i2c_bus;
static i2c_master_dev_handle_t s_mpu;
static bool s_imu_ready;
static float s_gx_bias_dps;
static float s_pending_heading_deg;
static SemaphoreHandle_t s_control_mutex;

typedef struct {
    int encoder_a_gpio;
    int encoder_b_gpio;
    volatile int32_t count;
    volatile uint8_t state;
} encoder_state_t;

typedef struct {
    bool active;
    bool wheel_active[CAR_WHEEL_COUNT];
    float base_speed[CAR_WHEEL_COUNT];
    int32_t start_count[CAR_WHEEL_COUNT];
} encoder_feedback_t;

static encoder_state_t s_encoder[CAR_WHEEL_COUNT];
static encoder_feedback_t s_feedback;
static TaskHandle_t s_feedback_task;
static portMUX_TYPE s_encoder_spinlock = portMUX_INITIALIZER_UNLOCKED;

/* State sequence is 00 -> 10 -> 11 -> 01 -> 00 for a positive count. */
static const int8_t DRAM_ATTR s_quadrature_delta[16] = {
    0, -1, 1, 0,
    1, 0, 0, -1,
    -1, 0, 0, 1,
    0, 1, -1, 0,
};

static const char *TAG = "car_control";

static esp_err_t set_wheel_speed_raw(car_wheel_t wheel, float speed_mps);
static esp_err_t set_velocity_raw(float vx_mps, float vy_mps, float omega_rad_s,
                                  float wheel_speed[CAR_WHEEL_COUNT]);
static esp_err_t set_velocity_with_feedback(float vx_mps, float vy_mps,
                                             float omega_rad_s, uint8_t wheel_mask);
static esp_err_t encoder_init(void);
static void feedback_disable_locked(void);
static void encoder_feedback_task(void *arg);

static float clampf(float value, float low, float high)
{
    return value < low ? low : (value > high ? high : value);
}

static esp_err_t lock_control(void)
{
    if (s_control_mutex == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    return xSemaphoreTake(s_control_mutex, pdMS_TO_TICKS(1000)) == pdTRUE
               ? ESP_OK
               : ESP_ERR_TIMEOUT;
}

static void unlock_control(void)
{
    (void)xSemaphoreGive(s_control_mutex);
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

static esp_err_t stop_all_wheels(void)
{
    for (int i = 0; i < CAR_WHEEL_COUNT; ++i) {
        ESP_RETURN_ON_ERROR(stop_wheel((car_wheel_t)i), "car_control", "stop wheel failed");
    }
    return ESP_OK;
}

static esp_err_t mpu_write_reg(uint8_t reg, uint8_t value)
{
    const uint8_t data[] = { reg, value };
    return i2c_master_transmit(s_mpu, data, sizeof(data), MPU_I2C_TIMEOUT_MS);
}

static esp_err_t mpu_read_gx_dps(float *gx_dps)
{
    uint8_t raw[2];
    esp_err_t result = ESP_FAIL;
    for (int attempt = 1; attempt <= MPU_READ_RETRIES; ++attempt) {
        result = i2c_master_transmit_receive(s_mpu, (uint8_t[]){ MPU_REG_GYRO_XOUT_H }, 1,
                                             raw, sizeof(raw), MPU_I2C_TIMEOUT_MS);
        if (result == ESP_OK) {
            const int16_t gx_raw = (int16_t)((raw[0] << 8) | raw[1]);
            *gx_dps = gx_raw * MPU_GYRO_SCALE_DPS;
            return ESP_OK;
        }

        ESP_LOGW(TAG, "gx read attempt %d/%d failed: %s", attempt, MPU_READ_RETRIES,
                 esp_err_to_name(result));
        if (attempt < MPU_READ_RETRIES) {
            vTaskDelay(pdMS_TO_TICKS(MPU_READ_RETRY_DELAY_MS));
        }
    }
    return result;
}

static esp_err_t mpu_init(void)
{
    i2c_master_bus_config_t bus_config = {
        .i2c_port = I2C_NUM_0,
        .sda_io_num = CAR_MPU6050_SDA_GPIO,
        .scl_io_num = CAR_MPU6050_SCL_GPIO,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    ESP_RETURN_ON_ERROR(i2c_new_master_bus(&bus_config, &s_i2c_bus),
                        "car_control", "create I2C bus failed");

    uint8_t address = MPU_ADDR_LOW;
    if (i2c_master_probe(s_i2c_bus, address, MPU_I2C_TIMEOUT_MS) != ESP_OK) {
        address = MPU_ADDR_HIGH;
        if (i2c_master_probe(s_i2c_bus, address, MPU_I2C_TIMEOUT_MS) != ESP_OK) {
            return ESP_ERR_NOT_FOUND;
        }
    }

    i2c_device_config_t device_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = address,
        .scl_speed_hz = MPU_I2C_FREQ_HZ,
    };
    ESP_RETURN_ON_ERROR(i2c_master_bus_add_device(s_i2c_bus, &device_config, &s_mpu),
                        "car_control", "add MPU6050 failed");
    ESP_RETURN_ON_ERROR(mpu_write_reg(MPU_REG_PWR_MGMT_1, 0x00),
                        "car_control", "wake MPU6050 failed");
    ESP_RETURN_ON_ERROR(mpu_write_reg(MPU_REG_SMPLRT_DIV, 0x04),
                        "car_control", "set MPU sample rate failed");
    ESP_RETURN_ON_ERROR(mpu_write_reg(MPU_REG_CONFIG, 0x03),
                        "car_control", "set MPU filter failed");
    ESP_RETURN_ON_ERROR(mpu_write_reg(MPU_REG_GYRO_CFG, 0x00),
                        "car_control", "set MPU gyro range failed");
    vTaskDelay(pdMS_TO_TICKS(50));

    float bias_sum = 0.0f;
    for (int i = 0; i < MPU_BIAS_SAMPLES; ++i) {
        float gx_dps;
        ESP_RETURN_ON_ERROR(mpu_read_gx_dps(&gx_dps), "car_control", "calibrate gx failed");
        bias_sum += gx_dps;
        vTaskDelay(pdMS_TO_TICKS(5));
    }
    s_gx_bias_dps = bias_sum / MPU_BIAS_SAMPLES;
    s_imu_ready = true;
    return ESP_OK;
}

static esp_err_t turn_relative_deg(float target_deg)
{
    if (!s_imu_ready || !isfinite(target_deg)) {
        return ESP_ERR_INVALID_STATE;
    }
    if (fabsf(target_deg) < 0.01f) {
        return ESP_OK;
    }

    const float direction = target_deg > 0.0f ? 1.0f : -1.0f;
    const float command_rad_s = direction * TURN_RATE_RAD_S;
    ESP_LOGI(TAG, "turn target=%.1f deg, command=%.2f rad/s, gx bias=%.2f dps",
             target_deg, command_rad_s, s_gx_bias_dps);
    ESP_RETURN_ON_ERROR(car_control_turn_in_place(command_rad_s),
                        "car_control", "start turn failed");

    float turned_deg = 0.0f;
    int64_t previous_us = esp_timer_get_time();
    const int64_t deadline_us = previous_us + TURN_TIMEOUT_MS * 1000LL;
    esp_err_t result = ESP_OK;
    while (direction * turned_deg < fabsf(target_deg)) {
        vTaskDelay(pdMS_TO_TICKS(TURN_SAMPLE_MS));
        const int64_t now_us = esp_timer_get_time();
        float gx_dps;
        result = mpu_read_gx_dps(&gx_dps);
        if (result != ESP_OK) {
            break;
        }
        turned_deg += (gx_dps - s_gx_bias_dps) * (now_us - previous_us) / 1000000.0f;
        previous_us = now_us;
        if (now_us >= deadline_us) {
            result = ESP_ERR_TIMEOUT;
            break;
        }
    }

    const esp_err_t stop_result = stop_all_wheels();
    if (result != ESP_OK) {
        ESP_LOGE(TAG, "turn failed: target=%.1f deg, signed progress=%.1f deg, error=%s",
                 target_deg, turned_deg, esp_err_to_name(result));
        return result;
    }
    ESP_LOGI(TAG, "turn complete: target=%.1f deg, measured=%.1f deg",
             target_deg, turned_deg);
    return stop_result;
}

static esp_err_t start_lateral_move(float heading_deg, float vx_mps, float vy_mps)
{
    if (!s_ready || s_pending_heading_deg != 0.0f) {
        return ESP_ERR_INVALID_STATE;
    }

    ESP_RETURN_ON_ERROR(turn_relative_deg(heading_deg), "car_control", "turn for lateral move failed");
    const uint8_t feedback_wheels = heading_deg < 0.0f
                                        ? (1U << CAR_WHEEL_A) | (1U << CAR_WHEEL_B)
                                        : (1U << CAR_WHEEL_A) | (1U << CAR_WHEEL_D);
    esp_err_t result = set_velocity_with_feedback(vx_mps, vy_mps, 0.0f,
                                                  feedback_wheels);
    if (result != ESP_OK) {
        (void)turn_relative_deg(-heading_deg);
        return result;
    }
    s_pending_heading_deg = heading_deg;
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
    if (s_control_mutex == NULL) {
        s_control_mutex = xSemaphoreCreateMutex();
        if (s_control_mutex == NULL) {
            return ESP_ERR_NO_MEM;
        }
    }
    memset(&s_feedback, 0, sizeof(s_feedback));
    s_imu_ready = false;
    s_pending_heading_deg = 0.0f;
    s_ready = false;

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

    ESP_RETURN_ON_ERROR(stop_all_wheels(), "car_control", "initial motor stop failed");
    ESP_RETURN_ON_ERROR(encoder_init(), "car_control", "initialize encoders failed");
    ESP_RETURN_ON_ERROR(mpu_init(), "car_control", "initialize MPU6050 failed");

    s_ready = true;
    if (s_feedback_task == NULL &&
        xTaskCreate(encoder_feedback_task, "encoder_feedback", 3072, NULL, 6,
                    &s_feedback_task) != pdPASS) {
        s_ready = false;
        return ESP_ERR_NO_MEM;
    }
    return ESP_OK;
}

esp_err_t car_control_set_wheel_speed(car_wheel_t wheel, float speed_mps)
{
    if (!s_ready) {
        return ESP_ERR_INVALID_STATE;
    }
    if (wheel < CAR_WHEEL_A || wheel >= CAR_WHEEL_COUNT || !isfinite(speed_mps)) {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t result = lock_control();
    if (result != ESP_OK) {
        return result;
    }
    feedback_disable_locked();
    result = set_wheel_speed_raw(wheel, speed_mps);
    unlock_control();
    return result;
}

esp_err_t car_control_set_velocity(float vx_mps, float vy_mps, float omega_rad_s)
{
    if (!s_ready) {
        return ESP_ERR_INVALID_STATE;
    }
    if (!isfinite(vx_mps) || !isfinite(vy_mps) || !isfinite(omega_rad_s)) {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t result = lock_control();
    if (result != ESP_OK) {
        return result;
    }
    feedback_disable_locked();
    float wheel_speed[CAR_WHEEL_COUNT];
    result = set_velocity_raw(vx_mps, vy_mps, omega_rad_s, wheel_speed);
    if (result != ESP_OK) {
        (void)stop_all_wheels();
    }
    unlock_control();
    return result;
}

esp_err_t car_control_forward(float speed_mps)
{
    return set_velocity_with_feedback(0.0f, speed_mps, 0.0f,
                                      (1U << CAR_WHEEL_B) | (1U << CAR_WHEEL_D));
}

esp_err_t car_control_backward(float speed_mps)
{
    if (!isfinite(speed_mps) || speed_mps < 0.0f) {
        return ESP_ERR_INVALID_ARG;
    }
    return car_control_forward(-speed_mps);
}

esp_err_t car_control_strafe_left(float speed_mps)
{
    if (!isfinite(speed_mps) || speed_mps < 0.0f) {
        return ESP_ERR_INVALID_ARG;
    }
    return start_lateral_move(-TURN_ANGLE_DEG,
                              -SQRT3_OVER_2 * speed_mps, -0.5f * speed_mps);
}

esp_err_t car_control_strafe_right(float speed_mps)
{
    if (!isfinite(speed_mps) || speed_mps < 0.0f) {
        return ESP_ERR_INVALID_ARG;
    }
    return start_lateral_move(TURN_ANGLE_DEG,
                              SQRT3_OVER_2 * speed_mps, -0.5f * speed_mps);
}

esp_err_t car_control_finish_strafe(void)
{
    if (!s_ready || s_pending_heading_deg == 0.0f) {
        return ESP_ERR_INVALID_STATE;
    }

    ESP_RETURN_ON_ERROR(car_control_stop(), "car_control", "stop lateral move failed");
    vTaskDelay(pdMS_TO_TICKS(MPU_SETTLE_AFTER_MOTOR_MS));
    ESP_RETURN_ON_ERROR(turn_relative_deg(-s_pending_heading_deg),
                        "car_control", "restore forward heading failed");
    s_pending_heading_deg = 0.0f;
    return ESP_OK;
}

static esp_err_t set_wheel_speed_raw(car_wheel_t wheel, float speed_mps)
{
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

static void IRAM_ATTR encoder_isr_handler(void *arg)
{
    encoder_state_t *encoder = (encoder_state_t *)arg;
    const uint8_t current_state =
        (uint8_t)((gpio_get_level(encoder->encoder_a_gpio) ? 2 : 0) |
                  (gpio_get_level(encoder->encoder_b_gpio) ? 1 : 0));

    portENTER_CRITICAL_ISR(&s_encoder_spinlock);
    const uint8_t transition = (uint8_t)((encoder->state << 2) | current_state);
    encoder->count += s_quadrature_delta[transition];
    encoder->state = current_state;
    portEXIT_CRITICAL_ISR(&s_encoder_spinlock);
}

static int32_t encoder_get_count(car_wheel_t wheel)
{
    int32_t count;
    portENTER_CRITICAL(&s_encoder_spinlock);
    count = s_encoder[wheel].count;
    portEXIT_CRITICAL(&s_encoder_spinlock);
    return count;
}

static esp_err_t encoder_init(void)
{
    uint64_t encoder_pin_mask = 0;
    for (int i = 0; i < CAR_WHEEL_COUNT; ++i) {
        const car_wheel_config_t *motor = &s_config.wheel[i];
        if (motor->encoder_a_gpio < 0 || motor->encoder_a_gpio >= GPIO_NUM_MAX ||
            motor->encoder_b_gpio < 0 || motor->encoder_b_gpio >= GPIO_NUM_MAX ||
            motor->encoder_a_gpio == motor->encoder_b_gpio) {
            return ESP_ERR_INVALID_ARG;
        }
        const uint64_t a_mask = 1ULL << motor->encoder_a_gpio;
        const uint64_t b_mask = 1ULL << motor->encoder_b_gpio;
        if ((encoder_pin_mask & (a_mask | b_mask)) != 0) {
            return ESP_ERR_INVALID_ARG;
        }
        encoder_pin_mask |= a_mask | b_mask;
    }

    gpio_config_t encoder_pins = {
        .pin_bit_mask = encoder_pin_mask,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_ANYEDGE,
    };
    ESP_RETURN_ON_ERROR(gpio_config(&encoder_pins), "car_control",
                        "configure encoder inputs failed");

    esp_err_t result = gpio_install_isr_service(ESP_INTR_FLAG_IRAM);
    if (result != ESP_OK && result != ESP_ERR_INVALID_STATE) {
        return result;
    }

    for (int i = 0; i < CAR_WHEEL_COUNT; ++i) {
        const car_wheel_config_t *motor = &s_config.wheel[i];
        s_encoder[i].encoder_a_gpio = motor->encoder_a_gpio;
        s_encoder[i].encoder_b_gpio = motor->encoder_b_gpio;
        portENTER_CRITICAL(&s_encoder_spinlock);
        s_encoder[i].count = 0;
        s_encoder[i].state = (uint8_t)(
            (gpio_get_level(s_encoder[i].encoder_a_gpio) ? 2 : 0) |
            (gpio_get_level(s_encoder[i].encoder_b_gpio) ? 1 : 0));
        portEXIT_CRITICAL(&s_encoder_spinlock);

        ESP_RETURN_ON_ERROR(gpio_isr_handler_add(s_encoder[i].encoder_a_gpio,
                                                 encoder_isr_handler, &s_encoder[i]),
                            "car_control", "attach encoder A interrupt failed");
        ESP_RETURN_ON_ERROR(gpio_isr_handler_add(s_encoder[i].encoder_b_gpio,
                                                 encoder_isr_handler, &s_encoder[i]),
                            "car_control", "attach encoder B interrupt failed");
    }
    return ESP_OK;
}

static void feedback_disable_locked(void)
{
    s_feedback.active = false;
    memset(s_feedback.wheel_active, 0, sizeof(s_feedback.wheel_active));
}

static void feedback_enable_locked(const float *wheel_speed, uint8_t wheel_mask)
{
    s_feedback.active = false;
    for (int i = 0; i < CAR_WHEEL_COUNT; ++i) {
        s_feedback.wheel_active[i] = (wheel_mask & (1U << i)) != 0 &&
                                     fabsf(wheel_speed[i]) >= 0.0001f;
        s_feedback.base_speed[i] = wheel_speed[i];
        s_feedback.start_count[i] = encoder_get_count((car_wheel_t)i);
    }
    s_feedback.active = wheel_mask != 0;
}

static void feedback_update_locked(void)
{
    float progress[CAR_WHEEL_COUNT] = { 0.0f };
    float progress_sum = 0.0f;
    int active_count = 0;

    for (int i = 0; i < CAR_WHEEL_COUNT; ++i) {
        if (!s_feedback.wheel_active[i]) {
            continue;
        }
        const int32_t count = encoder_get_count((car_wheel_t)i);
        const int32_t delta = (int32_t)((uint32_t)count -
                                        (uint32_t)s_feedback.start_count[i]);
        progress[i] = fabsf((float)delta);
        progress_sum += progress[i];
        ++active_count;
    }

    if (active_count < 2) {
        return;
    }
    const float mean_progress = progress_sum / (float)active_count;
    if (mean_progress < ENCODER_MIN_PROGRESS_COUNTS) {
        return;
    }

    for (int i = 0; i < CAR_WHEEL_COUNT; ++i) {
        if (!s_feedback.wheel_active[i]) {
            continue;
        }
        const float base_speed = s_feedback.base_speed[i];
        const float base_abs = fabsf(base_speed);
        const float relative_error = (mean_progress - progress[i]) / mean_progress;
        const float max_correction = base_abs * ENCODER_MAX_CORRECTION_RATIO;
        const float correction = clampf(base_abs * ENCODER_CORRECTION_GAIN *
                                            relative_error,
                                        -max_correction, max_correction);
        const float corrected_speed = base_speed +
                                      (base_speed >= 0.0f ? correction : -correction);
        if (set_wheel_speed_raw((car_wheel_t)i, corrected_speed) != ESP_OK) {
            ESP_LOGW(TAG, "encoder correction failed for wheel %d", i);
        }
    }
}

static void encoder_feedback_task(void *arg)
{
    (void)arg;
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(ENCODER_CONTROL_PERIOD_MS));
        if (!s_ready || lock_control() != ESP_OK) {
            continue;
        }
        if (s_feedback.active) {
            feedback_update_locked();
        }
        unlock_control();
    }
}

static void calculate_wheel_speeds(float vx_mps, float vy_mps, float omega_rad_s,
                                   float wheel_speed[CAR_WHEEL_COUNT])
{
    const float rotation = s_config.centre_to_wheel_m * omega_rad_s;
    wheel_speed[CAR_WHEEL_A] = vx_mps + rotation;
    wheel_speed[CAR_WHEEL_B] = -0.5f * vx_mps - SQRT3_OVER_2 * vy_mps + rotation;
    wheel_speed[CAR_WHEEL_D] = -0.5f * vx_mps + SQRT3_OVER_2 * vy_mps + rotation;
}

static esp_err_t set_velocity_raw(float vx_mps, float vy_mps, float omega_rad_s,
                                  float wheel_speed[CAR_WHEEL_COUNT])
{
    calculate_wheel_speeds(vx_mps, vy_mps, omega_rad_s, wheel_speed);
    for (int i = 0; i < CAR_WHEEL_COUNT; ++i) {
        ESP_RETURN_ON_ERROR(set_wheel_speed_raw((car_wheel_t)i, wheel_speed[i]),
                            "car_control", "set wheel speed failed");
    }
    return ESP_OK;
}

static esp_err_t set_velocity_with_feedback(float vx_mps, float vy_mps,
                                             float omega_rad_s, uint8_t wheel_mask)
{
    if (!s_ready) {
        return ESP_ERR_INVALID_STATE;
    }
    if (!isfinite(vx_mps) || !isfinite(vy_mps) || !isfinite(omega_rad_s)) {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t result = lock_control();
    if (result != ESP_OK) {
        return result;
    }

    float wheel_speed[CAR_WHEEL_COUNT];
    feedback_disable_locked();
    result = set_velocity_raw(vx_mps, vy_mps, omega_rad_s, wheel_speed);
    if (result == ESP_OK) {
        feedback_enable_locked(wheel_speed, wheel_mask);
    } else {
        (void)stop_all_wheels();
    }
    unlock_control();
    return result;
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

    esp_err_t result = lock_control();
    if (result != ESP_OK) {
        return result;
    }
    feedback_disable_locked();
    result = stop_all_wheels();
    unlock_control();
    return result;
}
