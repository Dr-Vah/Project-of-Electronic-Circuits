#include "camera_uvc.h"
#include "car_control.h"
#include "course_controller.h"
#include "esp_check.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "tft_display.h"
#include "ultrasonic_sensor.h"

static const char *TAG = "task2";

static void display_task(void *argument)
{
    (void)argument;
    while (true) {
        ultrasonic_sample_t sample = {0};
        const ultrasonic_sample_t *sample_ptr =
            course_controller_get_ultrasonic_sample(&sample) ? &sample : NULL;
        ESP_ERROR_CHECK_WITHOUT_ABORT(
            tft_display_update_from_modules(sample_ptr));
        vTaskDelay(pdMS_TO_TICKS(200));
    }
}

void app_main(void)
{
    const car_control_config_t car_config = CAR_CONTROL_DEFAULT_CONFIG();
    const ultrasonic_sensor_config_t ultrasonic_config =
        ULTRASONIC_SENSOR_DEFAULT_CONFIG();

    ESP_ERROR_CHECK(car_control_init(&car_config));
    ESP_ERROR_CHECK(car_control_stop());
    ESP_ERROR_CHECK(ultrasonic_sensor_init(&ultrasonic_config));

    const esp_err_t display_result = tft_display_init();
    if (display_result == ESP_OK) {
        if (xTaskCreate(display_task, "tft_status", 3072, NULL, 4, NULL) !=
            pdPASS) {
            ESP_LOGW(TAG, "cannot create TFT status task");
        }
    } else {
        ESP_LOGW(TAG, "TFT unavailable: %s", esp_err_to_name(display_result));
    }

    /* Camera frames stay on the board for line following. The Wi-Fi/MJPEG
     * debug server is intentionally not started. */
    ESP_ERROR_CHECK(camera_uvc_start());
    ESP_ERROR_CHECK(course_controller_start());

    ESP_LOGI(TAG, "Task2 started: camera line following, obstacle trigger=5 cm");
}
