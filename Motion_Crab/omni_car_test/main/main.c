/**
 * @file main.c
 * @brief One-shot, low-speed smoke test for the three-wheel omni chassis.
 *
 * Keep the vehicle raised from the floor for its first run. The test waits two
 * seconds, drives forward, strafes left and right, then turns clockwise at
 * low speed. It always stops after the sequence and does not repeat.
 */

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#include "car_control.h"

#define START_DELAY_MS     2000
#define FORWARD_SPEED_MPS  0.12f
#define FORWARD_TIME_MS    1000
#define STRAFE_SPEED_MPS   0.09f
#define STRAFE_TIME_MS      1600
#define PAUSE_TIME_MS       250
#define TURN_RATE_RAD_S   -0.35f  /* Negative = clockwise, expected gx < 0. */
#define TURN_TIME_MS       700

static const char *TAG = "omni_test";

void app_main(void)
{
    car_control_config_t config = CAR_CONTROL_DEFAULT_CONFIG();
    ESP_ERROR_CHECK(car_control_init(&config));
    ESP_LOGI(TAG, "Motor test starts in %d ms; keep the chassis clear of obstacles", START_DELAY_MS);
    vTaskDelay(pdMS_TO_TICKS(START_DELAY_MS));

    ESP_LOGI(TAG, "Forward %.2f m/s for %d ms", FORWARD_SPEED_MPS, FORWARD_TIME_MS);
    ESP_ERROR_CHECK(car_control_forward(FORWARD_SPEED_MPS));
    vTaskDelay(pdMS_TO_TICKS(FORWARD_TIME_MS));

    ESP_ERROR_CHECK(car_control_stop());
    vTaskDelay(pdMS_TO_TICKS(PAUSE_TIME_MS));

    ESP_LOGI(TAG, "Strafe left %.2f m/s for %d ms", STRAFE_SPEED_MPS, STRAFE_TIME_MS);
    ESP_ERROR_CHECK(car_control_strafe_left(STRAFE_SPEED_MPS));
    vTaskDelay(pdMS_TO_TICKS(STRAFE_TIME_MS));

    ESP_ERROR_CHECK(car_control_stop());
    vTaskDelay(pdMS_TO_TICKS(PAUSE_TIME_MS));

    ESP_LOGI(TAG, "Strafe right %.2f m/s for %d ms", STRAFE_SPEED_MPS, STRAFE_TIME_MS);
    ESP_ERROR_CHECK(car_control_strafe_left(-STRAFE_SPEED_MPS));
    vTaskDelay(pdMS_TO_TICKS(STRAFE_TIME_MS));

    ESP_ERROR_CHECK(car_control_stop());
    vTaskDelay(pdMS_TO_TICKS(PAUSE_TIME_MS));

    ESP_LOGI(TAG, "Slow clockwise turn %.2f rad/s for %d ms", TURN_RATE_RAD_S, TURN_TIME_MS);
    ESP_ERROR_CHECK(car_control_turn_in_place(TURN_RATE_RAD_S));
    vTaskDelay(pdMS_TO_TICKS(TURN_TIME_MS));

    ESP_ERROR_CHECK(car_control_stop());
    ESP_LOGI(TAG, "Test complete; motors stopped");
}
