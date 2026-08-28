/**
 * Standalone TFT test.
 *
 * This test deliberately does not use the encoders or HC-SR04. It sends
 * changing fixed values once per second so the display direction, text, and
 * partial-refresh behavior can be checked independently.
 */

#include "esp_check.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "tft_display.h"

static const char *TAG = "tft_test";

void app_main(void)
{
    ESP_ERROR_CHECK(tft_display_init());
    ESP_LOGI(TAG, "ST7735S TFT test started");

    TickType_t next_wake = xTaskGetTickCount();
    int step = 0;

    while (true) {
        const tft_display_data_t data = {
            .wheel_a_rpm = 10.0f + (float)(step % 10),
            .wheel_b_rpm = 20.0f + (float)((step * 2) % 10),
            .wheel_d_rpm = 30.0f + (float)((step * 3) % 10),
            .distance_cm = 40.0f + (float)(step % 20),
        };

        ESP_ERROR_CHECK(tft_display_update(&data));
        ESP_LOGI(TAG, "A=%.1f RPM, B=%.1f RPM, D=%.1f RPM, distance=%.1f cm",
                 data.wheel_a_rpm, data.wheel_b_rpm,
                 data.wheel_d_rpm, data.distance_cm);

        ++step;
        vTaskDelayUntil(&next_wake, pdMS_TO_TICKS(1000));
    }
}
