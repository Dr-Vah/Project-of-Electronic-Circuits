#include "ultrasonic_sensor.h"

#include <math.h>
#include <string.h>

#include "driver/gpio.h"
#include "esp_check.h"
#include "esp_rom_sys.h"
#include "esp_timer.h"

#define SOUND_ROUND_TRIP_CM_PER_US 0.01715f

static ultrasonic_sensor_config_t s_config;
static bool s_ready;

esp_err_t ultrasonic_sensor_init(const ultrasonic_sensor_config_t *config)
{
    if (config == NULL || config->trig_gpio < 0 || config->echo_gpio < 0 ||
        config->trig_gpio == config->echo_gpio || config->echo_timeout_us == 0 ||
        !isfinite(config->minimum_cm) || !isfinite(config->maximum_cm) ||
        config->minimum_cm < 0.0f || config->maximum_cm <= config->minimum_cm) {
        return ESP_ERR_INVALID_ARG;
    }

    memcpy(&s_config, config, sizeof(s_config));
    const gpio_config_t trigger_config = {
        .pin_bit_mask = 1ULL << s_config.trig_gpio,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_RETURN_ON_ERROR(gpio_config(&trigger_config), "ultrasonic",
                        "configure trigger pin failed");
    ESP_RETURN_ON_ERROR(gpio_set_level(s_config.trig_gpio, 0), "ultrasonic",
                        "clear trigger pin failed");

    const gpio_config_t echo_config = {
        .pin_bit_mask = 1ULL << s_config.echo_gpio,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_RETURN_ON_ERROR(gpio_config(&echo_config), "ultrasonic",
                        "configure echo pin failed");
    s_ready = true;
    return ESP_OK;
}

esp_err_t ultrasonic_sensor_read(ultrasonic_sample_t *sample)
{
    if (!s_ready) {
        return ESP_ERR_INVALID_STATE;
    }
    if (sample == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    *sample = (ultrasonic_sample_t){
        .status = ULTRASONIC_WAIT_RISE_TIMEOUT,
        .valid = false,
        .distance_cm = NAN,
        .pulse_us = 0,
        .timestamp_us = esp_timer_get_time(),
    };

    gpio_set_level(s_config.trig_gpio, 0);
    esp_rom_delay_us(2);
    gpio_set_level(s_config.trig_gpio, 1);
    esp_rom_delay_us(10);
    gpio_set_level(s_config.trig_gpio, 0);

    const int64_t wait_started_us = esp_timer_get_time();
    while (gpio_get_level(s_config.echo_gpio) == 0) {
        if (esp_timer_get_time() - wait_started_us >= s_config.echo_timeout_us) {
            sample->timestamp_us = esp_timer_get_time();
            return ESP_OK;
        }
    }

    const int64_t pulse_started_us = esp_timer_get_time();
    sample->status = ULTRASONIC_PULSE_TIMEOUT;
    while (gpio_get_level(s_config.echo_gpio) != 0) {
        if (esp_timer_get_time() - pulse_started_us >= s_config.echo_timeout_us) {
            sample->timestamp_us = esp_timer_get_time();
            return ESP_OK;
        }
    }

    sample->timestamp_us = esp_timer_get_time();
    sample->pulse_us = sample->timestamp_us - pulse_started_us;
    sample->distance_cm = sample->pulse_us * SOUND_ROUND_TRIP_CM_PER_US;
    sample->valid = sample->distance_cm >= s_config.minimum_cm &&
                    sample->distance_cm <= s_config.maximum_cm;
    sample->status = sample->valid ? ULTRASONIC_SAMPLE_OK
                                   : ULTRASONIC_OUT_OF_RANGE;
    return ESP_OK;
}

