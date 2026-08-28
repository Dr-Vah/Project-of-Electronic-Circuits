#include <stdbool.h>
#include <stdint.h>

#include "car_control.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define CONTROL_PERIOD_MS       20
#define STATUS_LOG_PERIOD_MS    500
#define LINE_LOST_STOP_MS       1200

/* Tested board: black line = 1. Change to 0 if the module output is inverted. */
#define IR_BLACK_LEVEL          1

#define FORWARD_SPEED_MPS       0.22f
#define TURN_SPEED_RAD_S        1.8f
#define SEARCH_SPEED_RAD_S      1.2f
#define TURN_DEADBAND           0.45f

static const char *TAG = "LINE_TRACK";

typedef enum {
    TRACK_FORWARD,
    TRACK_TURN_LEFT,
    TRACK_TURN_RIGHT,
    TRACK_SEARCH_LEFT,
    TRACK_SEARCH_RIGHT,
    TRACK_INTERSECTION,
    TRACK_STOPPED,
} tracking_state_t;

typedef struct {
    bool black[4];
    uint8_t pattern;
    uint8_t black_count;
    float error; /* Negative: line is left; positive: line is right. */
} infrared_data_t;

static esp_err_t infrared_init(void)
{
    gpio_config_t config = {
        .pin_bit_mask = (1ULL << CAR_IR_OUT1_GPIO) |
                        (1ULL << CAR_IR_OUT2_GPIO) |
                        (1ULL << CAR_IR_OUT3_GPIO) |
                        (1ULL << CAR_IR_OUT4_GPIO),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    return gpio_config(&config);
}

static infrared_data_t infrared_read(void)
{
    static const gpio_num_t pins[4] = {
        CAR_IR_OUT1_GPIO, CAR_IR_OUT2_GPIO,
        CAR_IR_OUT3_GPIO, CAR_IR_OUT4_GPIO,
    };
    static const float weights[4] = {-3.0f, -1.0f, 1.0f, 3.0f};

    infrared_data_t data = {0};
    float weighted_sum = 0.0f;

    for (int i = 0; i < 4; ++i) {
        data.black[i] = gpio_get_level(pins[i]) == IR_BLACK_LEVEL;
        data.pattern = (uint8_t)((data.pattern << 1) | data.black[i]);
        if (data.black[i]) {
            weighted_sum += weights[i];
            ++data.black_count;
        }
    }
    if (data.black_count != 0) {
        data.error = weighted_sum / data.black_count;
    }
    return data;
}

static tracking_state_t tracking_update(const infrared_data_t *ir,
                                        float *last_error,
                                        int64_t *lost_since_us)
{
    if (ir->black_count == 0) {
        const int64_t now_us = esp_timer_get_time();
        if (*lost_since_us == 0) {
            *lost_since_us = now_us;
        }
        if ((now_us - *lost_since_us) / 1000 >= LINE_LOST_STOP_MS) {
            ESP_ERROR_CHECK(car_control_stop());
            return TRACK_STOPPED;
        }

        if (*last_error < 0.0f) {
            ESP_ERROR_CHECK(car_control_turn_in_place(SEARCH_SPEED_RAD_S));
            return TRACK_SEARCH_LEFT;
        }
        ESP_ERROR_CHECK(car_control_turn_in_place(-SEARCH_SPEED_RAD_S));
        return TRACK_SEARCH_RIGHT;
    }

    *lost_since_us = 0;

    if (ir->black_count == 4) {
        ESP_ERROR_CHECK(car_control_forward(FORWARD_SPEED_MPS));
        return TRACK_INTERSECTION;
    }

    *last_error = ir->error;
    if (ir->error < -TURN_DEADBAND) {
        ESP_ERROR_CHECK(car_control_turn_in_place(TURN_SPEED_RAD_S));
        return TRACK_TURN_LEFT;
    }
    if (ir->error > TURN_DEADBAND) {
        ESP_ERROR_CHECK(car_control_turn_in_place(-TURN_SPEED_RAD_S));
        return TRACK_TURN_RIGHT;
    }

    ESP_ERROR_CHECK(car_control_forward(FORWARD_SPEED_MPS));
    return TRACK_FORWARD;
}

void app_main(void)
{
    car_control_config_t motor_config = CAR_CONTROL_DEFAULT_CONFIG();
    ESP_ERROR_CHECK(car_control_init(&motor_config));
    ESP_ERROR_CHECK(infrared_init());
    ESP_ERROR_CHECK(car_control_stop());

    float last_error = 0.0f;
    int64_t lost_since_us = 0;
    int64_t last_log_us = 0;

    ESP_LOGI(TAG, "line tracking started: OUT1 is left, OUT4 is right");
    ESP_LOGI(TAG, "IR black level=%d, forward=%.2f m/s, turn=%.2f rad/s",
             IR_BLACK_LEVEL, FORWARD_SPEED_MPS, TURN_SPEED_RAD_S);

    while (true) {
        const infrared_data_t ir = infrared_read();
        const tracking_state_t state =
            tracking_update(&ir, &last_error, &lost_since_us);

        const int64_t now_us = esp_timer_get_time();
        if ((now_us - last_log_us) / 1000 >= STATUS_LOG_PERIOD_MS) {
            ESP_LOGI(TAG, "IR=%d%d%d%d pattern=0x%X error=%+.1f state=%d",
                     ir.black[0], ir.black[1], ir.black[2], ir.black[3],
                     ir.pattern, ir.error, state);
            last_log_us = now_us;
        }

        vTaskDelay(pdMS_TO_TICKS(CONTROL_PERIOD_MS));
    }
}
