#include "camera_display.h"

#include <stdbool.h>
#include <stddef.h>
#include <string.h>

#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_check.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define TFT_MOSI_GPIO GPIO_NUM_42
#define TFT_SCLK_GPIO GPIO_NUM_41
#define TFT_DC_GPIO   GPIO_NUM_48
#define TFT_RST_GPIO  GPIO_NUM_38
#define TFT_SPI_HOST  SPI3_HOST
#define TFT_SPI_HZ    (20 * 1000 * 1000)
#define TFT_X_OFFSET  2U
#define TFT_Y_OFFSET  1U

static const char *TAG = "camera_display";
static spi_device_handle_t s_tft;
static DMA_ATTR uint8_t s_line[CAMERA_DISPLAY_WIDTH * 2U];

static esp_err_t tft_write(bool data_mode, const void *data, size_t length)
{
    if (length == 0) return ESP_OK;
    ESP_RETURN_ON_ERROR(gpio_set_level(TFT_DC_GPIO, data_mode), TAG,
                        "set D/C failed");
    spi_transaction_t transaction = {
        .length = length * 8U,
        .tx_buffer = data,
    };
    return spi_device_polling_transmit(s_tft, &transaction);
}

static esp_err_t tft_command(uint8_t command)
{
    return tft_write(false, &command, 1);
}

static esp_err_t tft_command_data(uint8_t command,
                                  const void *data, size_t length)
{
    ESP_RETURN_ON_ERROR(tft_command(command), TAG,
                        "command 0x%02X failed", command);
    return tft_write(true, data, length);
}

static esp_err_t tft_set_window(uint16_t x0, uint16_t y0,
                                uint16_t x1, uint16_t y1)
{
    x0 += TFT_X_OFFSET;
    x1 += TFT_X_OFFSET;
    y0 += TFT_Y_OFFSET;
    y1 += TFT_Y_OFFSET;
    const uint8_t columns[] = {
        (uint8_t)(x0 >> 8), (uint8_t)x0,
        (uint8_t)(x1 >> 8), (uint8_t)x1,
    };
    const uint8_t rows[] = {
        (uint8_t)(y0 >> 8), (uint8_t)y0,
        (uint8_t)(y1 >> 8), (uint8_t)y1,
    };
    ESP_RETURN_ON_ERROR(tft_command_data(0x2A, columns, sizeof(columns)), TAG,
                        "set columns failed");
    ESP_RETURN_ON_ERROR(tft_command_data(0x2B, rows, sizeof(rows)), TAG,
                        "set rows failed");
    return tft_command(0x2C);
}

static esp_err_t tft_fill(uint16_t rgb565)
{
    const uint16_t wire = (uint16_t)((rgb565 << 8) | (rgb565 >> 8));
    uint16_t *line = (uint16_t *)s_line;
    for (size_t i = 0; i < CAMERA_DISPLAY_WIDTH; ++i) line[i] = wire;
    ESP_RETURN_ON_ERROR(tft_set_window(0, 0, CAMERA_DISPLAY_WIDTH - 1,
                                       CAMERA_DISPLAY_HEIGHT - 1), TAG,
                        "set full window failed");
    for (size_t y = 0; y < CAMERA_DISPLAY_HEIGHT; ++y) {
        ESP_RETURN_ON_ERROR(tft_write(true, s_line, sizeof(s_line)), TAG,
                            "fill row failed");
    }
    return ESP_OK;
}

esp_err_t camera_display_init(void)
{
    const gpio_config_t output_config = {
        .pin_bit_mask = (1ULL << TFT_DC_GPIO) | (1ULL << TFT_RST_GPIO),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_RETURN_ON_ERROR(gpio_config(&output_config), TAG,
                        "configure control GPIO failed");
    const spi_bus_config_t bus_config = {
        .mosi_io_num = TFT_MOSI_GPIO,
        .miso_io_num = GPIO_NUM_NC,
        .sclk_io_num = TFT_SCLK_GPIO,
        .quadwp_io_num = GPIO_NUM_NC,
        .quadhd_io_num = GPIO_NUM_NC,
        .max_transfer_sz = sizeof(s_line),
    };
    ESP_RETURN_ON_ERROR(spi_bus_initialize(TFT_SPI_HOST, &bus_config,
                                            SPI_DMA_CH_AUTO), TAG,
                        "initialize TFT SPI failed");
    const spi_device_interface_config_t device_config = {
        .clock_speed_hz = TFT_SPI_HZ,
        .mode = 0,
        .spics_io_num = -1,
        .queue_size = 1,
    };
    ESP_RETURN_ON_ERROR(spi_bus_add_device(TFT_SPI_HOST, &device_config,
                                            &s_tft), TAG,
                        "add TFT device failed");

    gpio_set_level(TFT_RST_GPIO, 1);
    vTaskDelay(pdMS_TO_TICKS(20));
    gpio_set_level(TFT_RST_GPIO, 0);
    vTaskDelay(pdMS_TO_TICKS(30));
    gpio_set_level(TFT_RST_GPIO, 1);
    vTaskDelay(pdMS_TO_TICKS(150));
    ESP_RETURN_ON_ERROR(tft_command(0x01), TAG, "software reset failed");
    vTaskDelay(pdMS_TO_TICKS(150));

    const uint8_t frame_normal[] = {0x01, 0x2C, 0x2D};
    const uint8_t frame_partial[] = {0x01, 0x2C, 0x2D, 0x01, 0x2C, 0x2D};
    const uint8_t power1[] = {0xA2, 0x02, 0x84};
    const uint8_t power2 = 0xC5;
    const uint8_t power3[] = {0x0A, 0x00};
    const uint8_t power4[] = {0x8A, 0x2A};
    const uint8_t power5[] = {0x8A, 0xEE};
    const uint8_t vcom = 0x0E;
    const uint8_t inversion = 0x07;
    const uint8_t madctl = 0xC8;
    const uint8_t pixel_format = 0x05;
    const uint8_t gamma_positive[] = {
        0x02,0x1C,0x07,0x12,0x37,0x32,0x29,0x2D,
        0x29,0x25,0x2B,0x39,0x00,0x01,0x03,0x10,
    };
    const uint8_t gamma_negative[] = {
        0x03,0x1D,0x07,0x06,0x2E,0x2C,0x29,0x2D,
        0x2E,0x2E,0x37,0x3F,0x00,0x00,0x02,0x10,
    };

    ESP_RETURN_ON_ERROR(tft_command(0x11), TAG, "sleep out failed");
    vTaskDelay(pdMS_TO_TICKS(500));
    ESP_RETURN_ON_ERROR(tft_command_data(0xB1, frame_normal, sizeof(frame_normal)), TAG, "FRMCTR1 failed");
    ESP_RETURN_ON_ERROR(tft_command_data(0xB2, frame_normal, sizeof(frame_normal)), TAG, "FRMCTR2 failed");
    ESP_RETURN_ON_ERROR(tft_command_data(0xB3, frame_partial, sizeof(frame_partial)), TAG, "FRMCTR3 failed");
    ESP_RETURN_ON_ERROR(tft_command_data(0xB4, &inversion, 1), TAG, "INVCTR failed");
    ESP_RETURN_ON_ERROR(tft_command_data(0xC0, power1, sizeof(power1)), TAG, "PWCTR1 failed");
    ESP_RETURN_ON_ERROR(tft_command_data(0xC1, &power2, 1), TAG, "PWCTR2 failed");
    ESP_RETURN_ON_ERROR(tft_command_data(0xC2, power3, sizeof(power3)), TAG, "PWCTR3 failed");
    ESP_RETURN_ON_ERROR(tft_command_data(0xC3, power4, sizeof(power4)), TAG, "PWCTR4 failed");
    ESP_RETURN_ON_ERROR(tft_command_data(0xC4, power5, sizeof(power5)), TAG, "PWCTR5 failed");
    ESP_RETURN_ON_ERROR(tft_command_data(0xC5, &vcom, 1), TAG, "VMCTR1 failed");
    ESP_RETURN_ON_ERROR(tft_command(0x20), TAG, "inversion off failed");
    ESP_RETURN_ON_ERROR(tft_command_data(0x36, &madctl, 1), TAG, "MADCTL failed");
    ESP_RETURN_ON_ERROR(tft_command_data(0x3A, &pixel_format, 1), TAG, "COLMOD failed");
    ESP_RETURN_ON_ERROR(tft_command_data(0xE0, gamma_positive, sizeof(gamma_positive)), TAG, "gamma+ failed");
    ESP_RETURN_ON_ERROR(tft_command_data(0xE1, gamma_negative, sizeof(gamma_negative)), TAG, "gamma- failed");
    ESP_RETURN_ON_ERROR(tft_command(0x13), TAG, "normal mode failed");
    ESP_RETURN_ON_ERROR(tft_command(0x29), TAG, "display on failed");
    vTaskDelay(pdMS_TO_TICKS(100));

    ESP_LOGI(TAG, "ST7735S ready: 128x160, MOSI=42 SCK=41 DC=48 RST=38");
    /* A visible blue startup screen confirms the panel independently of USB. */
    ESP_RETURN_ON_ERROR(tft_fill(0x001F), TAG, "startup fill failed");
    vTaskDelay(pdMS_TO_TICKS(500));
    return tft_fill(0x0000);
}

esp_err_t camera_display_draw_rgb565(const uint16_t *pixels,
                                     unsigned width, unsigned height)
{
    if (pixels == NULL || width == 0 || height == 0 ||
        width > CAMERA_DISPLAY_WIDTH || height > CAMERA_DISPLAY_HEIGHT) {
        return ESP_ERR_INVALID_ARG;
    }
    const uint16_t x = (CAMERA_DISPLAY_WIDTH - width) / 2U;
    const uint16_t y = (CAMERA_DISPLAY_HEIGHT - height) / 2U;
    ESP_RETURN_ON_ERROR(tft_set_window(x, y, x + width - 1,
                                       y + height - 1), TAG,
                        "set image window failed");
    const size_t row_bytes = width * sizeof(uint16_t);
    for (unsigned row = 0; row < height; ++row) {
        memcpy(s_line, pixels + (size_t)row * width, row_bytes);
        ESP_RETURN_ON_ERROR(tft_write(true, s_line, row_bytes), TAG,
                            "send image row failed");
    }
    return ESP_OK;
}
