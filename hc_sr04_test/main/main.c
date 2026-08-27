/*
 * ESP32-S3 + HC-SR04 超声波测距模块测试程序
 *
 * 接线:
 *   HC-SR04 VCC  -> 5V
 *   HC-SR04 GND  -> GND
 *   HC-SR04 TRIG -> GPIO17
 *   HC-SR04 ECHO -> GPIO18 (见下方电平转换说明)
 *
 * 电平转换(重要):
 *   HC-SR04 的 ECHO 回波电平是 5V, 而 ESP32-S3 的 GPIO 不耐 5V。
 *   建议按下面 2 个电阻分压后再进 GPIO18:
 *     ECHO ---[1kΩ]---[GPIO18]---[2kΩ]--- GND
 *   分压后约 3.3V。如果手上模块/开发板声明支持 3.3V 逻辑可直连。
 *
 * 串口输出格式 (CSV, 10 Hz):
 *   millis(ms),distance_cm
 *   其中 distance_cm 为 -1 表示超时(距离 >5m 或未收到回波)。
 *
 * 电脑端可视化: python visualize.py <COM口>   (见工程根目录 visualize.py)
 */
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "driver/gpio.h"
#include "esp_rom_sys.h"

#define TRIG_GPIO            GPIO_NUM_17
#define ECHO_GPIO            GPIO_NUM_18
#define ECHO_TIMEOUT_US      30000   /* 超时 30ms, 对应约 5m 量程 */
#define MEASURE_INTERVAL_MS  100     /* 测距周期, HC-SR04 最快 20Hz */

static const char *TAG = "hc_sr04";

static void gpio_init(void)
{
    /* TRIG: 输出 */
    gpio_config_t io = {
        .pin_bit_mask = (1ULL << TRIG_GPIO),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io);
    gpio_set_level(TRIG_GPIO, 0);

    /* ECHO: 输入, 模块本身是推挽输出, 不需要上下拉 */
    io.pin_bit_mask = (1ULL << ECHO_GPIO);
    io.mode = GPIO_MODE_INPUT;
    gpio_config(&io);
}

static void send_trigger(void)
{
    gpio_set_level(TRIG_GPIO, 0);
    esp_rom_delay_us(2);
    gpio_set_level(TRIG_GPIO, 1);
    esp_rom_delay_us(10);           /* 触发脉冲 >= 10us */
    gpio_set_level(TRIG_GPIO, 0);
}

/* 测量一次距离并返回厘米数; 超时返回 -1 */
static float measure_cm(void)
{
    send_trigger();

    /* 等 ECHO 拉高 (超声波发出后回波到来前 ECHO 为低) */
    int64_t t0 = esp_timer_get_time();
    while (gpio_get_level(ECHO_GPIO) == 0) {
        if (esp_timer_get_time() - t0 > ECHO_TIMEOUT_US) {
            return -1.0f;
        }
    }

    /* 测量 ECHO 高电平持续时间 */
    int64_t pulse_start = esp_timer_get_time();
    while (gpio_get_level(ECHO_GPIO) == 1) {
        if (esp_timer_get_time() - pulse_start > ECHO_TIMEOUT_US) {
            return -1.0f;
        }
    }
    int64_t pulse_us = esp_timer_get_time() - pulse_start;

    /* 距离 = 声速(约 0.0343 cm/us) x 时间 / 2 (往返) */
    return (float)pulse_us * 0.01715f;
}

void app_main(void)
{
    gpio_init();
    ESP_LOGI(TAG, "HC-SR04 测试启动: TRIG=GPIO%d, ECHO=GPIO%d", TRIG_GPIO, ECHO_GPIO);
    ESP_LOGI(TAG, "输出: millis(ms),distance_cm  (-1 表示超时)");

    while (1) {
        float dist = measure_cm();
        printf("%lld,%.1f\n", (long long)(esp_timer_get_time() / 1000), dist);
        fflush(stdout);
        vTaskDelay(pdMS_TO_TICKS(MEASURE_INTERVAL_MS));
    }
}