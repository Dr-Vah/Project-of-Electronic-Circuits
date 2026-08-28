/**
 * Standalone ST7735S display driver for the LQ_TFT18SPIV33-style 128x160
 * module used by the speed/distance monitor.
 */

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_check.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "tft_display.h"

#define TFT_SDI_GPIO                GPIO_NUM_20
#define TFT_SCK_GPIO                GPIO_NUM_19
#define TFT_DC_GPIO                 GPIO_NUM_48
#define TFT_RST_GPIO                GPIO_NUM_38
#define TFT_SPI_HOST                SPI3_HOST
#define TFT_SPI_CLOCK_HZ            (20 * 1000 * 1000)
#define TFT_WIDTH                   128
#define TFT_HEIGHT                  160
#define TFT_BYTES_PER_PIXEL        2
#define TFT_X_OFFSET                2
#define TFT_Y_OFFSET                1
#define TFT_MADCTL                  0xC8

static const char *TAG = "tft_display";

/* ------------------------------ ST7735S driver ------------------------------ */

static spi_device_handle_t s_tft;
static uint8_t s_tft_line_buffer[TFT_WIDTH * TFT_BYTES_PER_PIXEL];
#define FONT_SCALE                  2
#define FONT_PIXEL_WIDTH            (5 * FONT_SCALE)
#define FONT_PIXEL_HEIGHT           (7 * FONT_SCALE)
#define TFT_TEXT_X                  4
#define TFT_TEXT_REGION_WIDTH       (TFT_WIDTH - TFT_TEXT_X)
static uint8_t s_tft_text_buffer[TFT_TEXT_REGION_WIDTH * FONT_PIXEL_HEIGHT *
                                  TFT_BYTES_PER_PIXEL];

/* 5 x 7 font, one byte per column, ASCII 32..126.  Bit 0 is the top pixel. */
static const uint8_t s_font_5x7[95][5] = {
    {0x00,0x00,0x00,0x00,0x00}, {0x00,0x00,0x5F,0x00,0x00},
    {0x00,0x07,0x00,0x07,0x00}, {0x14,0x7F,0x14,0x7F,0x14},
    {0x24,0x2A,0x7F,0x2A,0x12}, {0x23,0x13,0x08,0x64,0x62},
    {0x36,0x49,0x55,0x22,0x50}, {0x00,0x05,0x03,0x00,0x00},
    {0x00,0x1C,0x22,0x41,0x00}, {0x00,0x41,0x22,0x1C,0x00},
    {0x14,0x08,0x3E,0x08,0x14}, {0x08,0x08,0x3E,0x08,0x08},
    {0x00,0x50,0x30,0x00,0x00}, {0x08,0x08,0x08,0x08,0x08},
    {0x00,0x60,0x60,0x00,0x00}, {0x20,0x10,0x08,0x04,0x02},
    {0x3E,0x51,0x49,0x45,0x3E}, {0x00,0x42,0x7F,0x40,0x00},
    {0x42,0x61,0x51,0x49,0x46}, {0x21,0x41,0x45,0x4B,0x31},
    {0x18,0x14,0x12,0x7F,0x10}, {0x27,0x45,0x45,0x45,0x39},
    {0x3C,0x4A,0x49,0x49,0x30}, {0x01,0x71,0x09,0x05,0x03},
    {0x36,0x49,0x49,0x49,0x36}, {0x06,0x49,0x49,0x29,0x1E},
    {0x00,0x36,0x36,0x00,0x00}, {0x00,0x56,0x36,0x00,0x00},
    {0x08,0x14,0x22,0x41,0x00}, {0x14,0x14,0x14,0x14,0x14},
    {0x00,0x41,0x22,0x14,0x08}, {0x02,0x01,0x51,0x09,0x06},
    {0x32,0x49,0x79,0x41,0x3E}, {0x7E,0x11,0x11,0x11,0x7E},
    {0x7F,0x49,0x49,0x49,0x36}, {0x3E,0x41,0x41,0x41,0x22},
    {0x7F,0x41,0x41,0x22,0x1C}, {0x7F,0x49,0x49,0x49,0x41},
    {0x7F,0x09,0x09,0x09,0x01}, {0x3E,0x41,0x49,0x49,0x7A},
    {0x7F,0x08,0x08,0x08,0x7F}, {0x00,0x41,0x7F,0x41,0x00},
    {0x20,0x40,0x41,0x3F,0x01}, {0x7F,0x08,0x14,0x22,0x41},
    {0x7F,0x40,0x40,0x40,0x40}, {0x7F,0x02,0x04,0x02,0x7F},
    {0x7F,0x04,0x08,0x10,0x7F}, {0x3E,0x41,0x41,0x41,0x3E},
    {0x7F,0x09,0x09,0x09,0x06}, {0x3E,0x41,0x51,0x21,0x5E},
    {0x7F,0x09,0x19,0x29,0x46}, {0x46,0x49,0x49,0x49,0x31},
    {0x01,0x01,0x7F,0x01,0x01}, {0x3F,0x40,0x40,0x40,0x3F},
    {0x1F,0x20,0x40,0x20,0x1F}, {0x3F,0x40,0x38,0x40,0x3F},
    {0x63,0x14,0x08,0x14,0x63}, {0x03,0x04,0x78,0x04,0x03},
    {0x61,0x51,0x49,0x45,0x43}, {0x00,0x7F,0x41,0x41,0x00},
    {0x02,0x04,0x08,0x10,0x20}, {0x00,0x41,0x41,0x7F,0x00},
    {0x04,0x02,0x01,0x02,0x04}, {0x40,0x40,0x40,0x40,0x40},
    {0x00,0x01,0x02,0x04,0x00}, {0x20,0x54,0x54,0x54,0x78},
    {0x7F,0x48,0x44,0x44,0x38}, {0x38,0x44,0x44,0x44,0x20},
    {0x38,0x44,0x44,0x48,0x7F}, {0x38,0x54,0x54,0x54,0x18},
    {0x08,0x7E,0x09,0x01,0x02}, {0x0C,0x52,0x52,0x52,0x3E},
    {0x7F,0x08,0x04,0x04,0x78}, {0x00,0x44,0x7D,0x40,0x00},
    {0x20,0x40,0x44,0x3D,0x00}, {0x7F,0x10,0x28,0x44,0x00},
    {0x00,0x41,0x7F,0x40,0x00}, {0x7E,0x02,0x0C,0x02,0x7C},
    {0x7E,0x04,0x02,0x02,0x7C}, {0x38,0x44,0x44,0x44,0x38},
    {0x7C,0x14,0x14,0x14,0x08}, {0x08,0x14,0x14,0x18,0x7C},
    {0x7C,0x08,0x04,0x04,0x08}, {0x48,0x54,0x54,0x54,0x20},
    {0x04,0x3F,0x44,0x40,0x20}, {0x3C,0x40,0x40,0x20,0x7C},
    {0x1C,0x20,0x40,0x20,0x1C}, {0x3C,0x40,0x30,0x40,0x3C},
    {0x44,0x28,0x10,0x28,0x44}, {0x0C,0x50,0x50,0x50,0x3C},
    {0x44,0x64,0x54,0x4C,0x44}, {0x08,0x36,0x41,0x00,0x00},
    {0x00,0x00,0x7F,0x00,0x00}, {0x00,0x41,0x36,0x08,0x00},
    {0x08,0x04,0x08,0x10,0x08},
};

static esp_err_t tft_write(bool data_mode, const void *buffer, size_t length)
{
    if (length == 0) {
        return ESP_OK;
    }

    gpio_set_level(TFT_DC_GPIO, data_mode ? 1 : 0);
    spi_transaction_t transaction = {
        .length = length * 8,
        .tx_buffer = buffer,
    };
    return spi_device_transmit(s_tft, &transaction);
}

static esp_err_t tft_command(uint8_t command)
{
    return tft_write(false, &command, sizeof(command));
}

static esp_err_t tft_command_data(uint8_t command, const uint8_t *data, size_t length)
{
    esp_err_t result = tft_command(command);
    if (result != ESP_OK) {
        return result;
    }
    return tft_write(true, data, length);
}

static esp_err_t tft_set_window(uint16_t x0, uint16_t y0,
                                uint16_t x1, uint16_t y1)
{
    const uint16_t ax0 = x0 + TFT_X_OFFSET;
    const uint16_t ax1 = x1 + TFT_X_OFFSET;
    const uint16_t ay0 = y0 + TFT_Y_OFFSET;
    const uint16_t ay1 = y1 + TFT_Y_OFFSET;
    const uint8_t column_data[] = {
        (uint8_t)(ax0 >> 8), (uint8_t)ax0,
        (uint8_t)(ax1 >> 8), (uint8_t)ax1,
    };
    const uint8_t row_data[] = {
        (uint8_t)(ay0 >> 8), (uint8_t)ay0,
        (uint8_t)(ay1 >> 8), (uint8_t)ay1,
    };

    esp_err_t result = tft_command_data(0x2A, column_data, sizeof(column_data));
    if (result != ESP_OK) {
        return result;
    }
    result = tft_command_data(0x2B, row_data, sizeof(row_data));
    if (result != ESP_OK) {
        return result;
    }
    return tft_command(0x2C);
}

/* ST7735S uses ordinary 16-bit RGB565 pixels, MSB first. */
static void st7735_color_bytes(uint16_t color, uint8_t bytes[TFT_BYTES_PER_PIXEL])
{
    bytes[0] = (uint8_t)(color >> 8);
    bytes[1] = (uint8_t)color;
}

static esp_err_t tft_fill_rect(uint16_t x, uint16_t y, uint16_t width,
                               uint16_t height, uint16_t color)
{
    if (width == 0 || height == 0 || x >= TFT_WIDTH || y >= TFT_HEIGHT) {
        return ESP_OK;
    }
    if (x + width > TFT_WIDTH) {
        width = TFT_WIDTH - x;
    }
    if (y + height > TFT_HEIGHT) {
        height = TFT_HEIGHT - y;
    }

    uint8_t color_bytes[TFT_BYTES_PER_PIXEL];
    st7735_color_bytes(color, color_bytes);
    for (uint16_t column = 0; column < width; ++column) {
        memcpy(&s_tft_line_buffer[TFT_BYTES_PER_PIXEL * column],
               color_bytes, TFT_BYTES_PER_PIXEL);
    }

    esp_err_t result = tft_set_window(x, y, x + width - 1, y + height - 1);
    if (result != ESP_OK) {
        return result;
    }
    for (uint16_t row = 0; row < height; ++row) {
        result = tft_write(true, s_tft_line_buffer,
                           width * TFT_BYTES_PER_PIXEL);
        if (result != ESP_OK) {
            return result;
        }
    }
    return ESP_OK;
}

static esp_err_t tft_fill_screen(uint16_t color)
{
    return tft_fill_rect(0, 0, TFT_WIDTH, TFT_HEIGHT, color);
}

static esp_err_t tft_init(void)
{
    const gpio_config_t control_gpio_config = {
        .pin_bit_mask = (1ULL << TFT_DC_GPIO) | (1ULL << TFT_RST_GPIO),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_RETURN_ON_ERROR(gpio_config(&control_gpio_config), TAG,
                        "configure TFT control pins failed");

    const spi_bus_config_t bus_config = {
        .mosi_io_num = TFT_SDI_GPIO,
        .miso_io_num = GPIO_NUM_NC,
        .sclk_io_num = TFT_SCK_GPIO,
        .quadwp_io_num = GPIO_NUM_NC,
        .quadhd_io_num = GPIO_NUM_NC,
        /* The complete rendered text row is the largest SPI transaction. */
        .max_transfer_sz = sizeof(s_tft_text_buffer),
    };
    ESP_RETURN_ON_ERROR(spi_bus_initialize(TFT_SPI_HOST, &bus_config, SPI_DMA_CH_AUTO),
                        TAG, "initialize TFT SPI bus failed");

    const spi_device_interface_config_t device_config = {
        .clock_speed_hz = TFT_SPI_CLOCK_HZ,
        .mode = 0,
        .spics_io_num = -1,
        .queue_size = 1,
    };
    ESP_RETURN_ON_ERROR(spi_bus_add_device(TFT_SPI_HOST, &device_config, &s_tft),
                        TAG, "add TFT SPI device failed");

    /* Reset the ST7735S.  CS is permanently low, so commands are selected by
     * the D/C line only. */
    ESP_RETURN_ON_ERROR(tft_command(0x01), TAG, "ST7735S software reset failed");
    vTaskDelay(pdMS_TO_TICKS(150));
    gpio_set_level(TFT_RST_GPIO, 1);
    vTaskDelay(pdMS_TO_TICKS(5));
    gpio_set_level(TFT_RST_GPIO, 0);
    vTaskDelay(pdMS_TO_TICKS(20));
    gpio_set_level(TFT_RST_GPIO, 1);
    vTaskDelay(pdMS_TO_TICKS(150));

    /* ST7735S 128x160 initialization, using the common green-tab timing and
     * the 2,1 GRAM offset used by 132x162 ST7735S panels. */
    const uint8_t frame_rate_normal[] = { 0x01, 0x2C, 0x2D };
    const uint8_t frame_rate_partial[] = { 0x01, 0x2C, 0x2D, 0x01, 0x2C, 0x2D };
    const uint8_t power_control_1[] = { 0xA2, 0x02, 0x84 };
    const uint8_t power_control_2 = 0xC5;
    const uint8_t power_control_3[] = { 0x0A, 0x00 };
    const uint8_t power_control_4[] = { 0x8A, 0x2A };
    const uint8_t power_control_5[] = { 0x8A, 0xEE };
    const uint8_t vcom_control = 0x0E;
    const uint8_t madctl = TFT_MADCTL;
    const uint8_t pixel_format = 0x05; /* 16-bit RGB565 */
    const uint8_t inversion_control = 0x07;
    const uint8_t gamma_positive[] = {
        0x02, 0x1C, 0x07, 0x12, 0x37, 0x32, 0x29, 0x2D,
        0x29, 0x25, 0x2B, 0x39, 0x00, 0x01, 0x03, 0x10,
    };
    const uint8_t gamma_negative[] = {
        0x03, 0x1D, 0x07, 0x06, 0x2E, 0x2C, 0x29, 0x2D,
        0x2E, 0x2E, 0x37, 0x3F, 0x00, 0x00, 0x02, 0x10,
    };
    const uint8_t column_range[] = { 0x00, TFT_X_OFFSET,
                                     0x00, TFT_X_OFFSET + TFT_WIDTH - 1 };
    const uint8_t row_range[] = { 0x00, TFT_Y_OFFSET,
                                  0x00, TFT_Y_OFFSET + TFT_HEIGHT - 1 };

    ESP_RETURN_ON_ERROR(tft_command(0x11), TAG, "ST7735S sleep-out failed");
    vTaskDelay(pdMS_TO_TICKS(500));
    ESP_RETURN_ON_ERROR(tft_command_data(0xB1, frame_rate_normal,
                                         sizeof(frame_rate_normal)), TAG,
                        "set ST7735S frame rate 1 failed");
    ESP_RETURN_ON_ERROR(tft_command_data(0xB2, frame_rate_normal,
                                         sizeof(frame_rate_normal)), TAG,
                        "set ST7735S frame rate 2 failed");
    ESP_RETURN_ON_ERROR(tft_command_data(0xB3, frame_rate_partial,
                                         sizeof(frame_rate_partial)), TAG,
                        "set ST7735S frame rate 3 failed");
    ESP_RETURN_ON_ERROR(tft_command_data(0xB4, &inversion_control, 1), TAG,
                        "set ST7735S inversion failed");
    ESP_RETURN_ON_ERROR(tft_command_data(0xC0, power_control_1,
                                         sizeof(power_control_1)), TAG,
                        "set ST7735S power control 1 failed");
    ESP_RETURN_ON_ERROR(tft_command_data(0xC1, &power_control_2, 1), TAG,
                        "set ST7735S power control 2 failed");
    ESP_RETURN_ON_ERROR(tft_command_data(0xC2, power_control_3,
                                         sizeof(power_control_3)), TAG,
                        "set ST7735S power control 3 failed");
    ESP_RETURN_ON_ERROR(tft_command_data(0xC3, power_control_4,
                                         sizeof(power_control_4)), TAG,
                        "set ST7735S power control 4 failed");
    ESP_RETURN_ON_ERROR(tft_command_data(0xC4, power_control_5,
                                         sizeof(power_control_5)), TAG,
                        "set ST7735S power control 5 failed");
    ESP_RETURN_ON_ERROR(tft_command_data(0xC5, &vcom_control, 1), TAG,
                        "set ST7735S VCOM failed");
    ESP_RETURN_ON_ERROR(tft_command(0x20), TAG, "disable ST7735S inversion failed");
    ESP_RETURN_ON_ERROR(tft_command_data(0x36, &madctl, 1), TAG,
                        "set ST7735S orientation failed");
    ESP_RETURN_ON_ERROR(tft_command_data(0x3A, &pixel_format, 1), TAG,
                        "set ST7735S pixel format failed");
    ESP_RETURN_ON_ERROR(tft_command_data(0xE0, gamma_positive,
                                         sizeof(gamma_positive)), TAG,
                        "set ST7735S positive gamma failed");
    ESP_RETURN_ON_ERROR(tft_command_data(0xE1, gamma_negative,
                                         sizeof(gamma_negative)), TAG,
                        "set ST7735S negative gamma failed");
    ESP_RETURN_ON_ERROR(tft_command_data(0x2A, column_range, sizeof(column_range)), TAG,
                        "set ST7735S column range failed");
    ESP_RETURN_ON_ERROR(tft_command_data(0x2B, row_range, sizeof(row_range)), TAG,
                        "set ST7735S row range failed");
    ESP_RETURN_ON_ERROR(tft_command(0x13), TAG, "set ST7735S normal mode failed");
    vTaskDelay(pdMS_TO_TICKS(10));
    ESP_RETURN_ON_ERROR(tft_command(0x29), TAG, "turn ST7735S display on failed");
    vTaskDelay(pdMS_TO_TICKS(100));
    return ESP_OK;
}

/* Render one complete text row in RAM and send it in a single transaction.
 * This prevents the clear-then-draw and per-character updates that look like
 * flicker on a small SPI panel. */
static esp_err_t tft_draw_text_line(int x, int y, const char *text,
                                    uint16_t foreground, uint16_t background)
{
    uint8_t background_bytes[TFT_BYTES_PER_PIXEL];
    uint8_t foreground_bytes[TFT_BYTES_PER_PIXEL];
    st7735_color_bytes(background, background_bytes);
    st7735_color_bytes(foreground, foreground_bytes);

    if (x < 0 || y < 0 || x >= TFT_WIDTH || y + FONT_PIXEL_HEIGHT > TFT_HEIGHT) {
        return ESP_ERR_INVALID_ARG;
    }

    const uint16_t width = (uint16_t)(TFT_WIDTH - x);
    const size_t text_length = strlen(text);
    const uint16_t character_pitch = 6 * FONT_SCALE;

    for (uint16_t pixel_y = 0; pixel_y < FONT_PIXEL_HEIGHT; ++pixel_y) {
        for (uint16_t pixel_x = 0; pixel_x < width; ++pixel_x) {
            bool lit = false;
            const size_t character_index = pixel_x / character_pitch;
            const uint16_t character_x = pixel_x % character_pitch;
            if (character_index < text_length && character_x < FONT_PIXEL_WIDTH) {
                const char character = text[character_index];
                if (character >= 32 && character <= 126) {
                    const uint8_t *glyph = s_font_5x7[(int)character - 32];
                    const int source_x = character_x / FONT_SCALE;
                    const int source_y = pixel_y / FONT_SCALE;
                    lit = (glyph[source_x] & (1U << source_y)) != 0;
                }
            }
            const uint8_t *color_bytes = lit ? foreground_bytes : background_bytes;
            const size_t buffer_index =
                (pixel_y * width + pixel_x) * TFT_BYTES_PER_PIXEL;
            memcpy(&s_tft_text_buffer[buffer_index], color_bytes, TFT_BYTES_PER_PIXEL);
        }
    }

    esp_err_t result = tft_set_window((uint16_t)x, (uint16_t)y,
                                      (uint16_t)(x + width - 1),
                                      (uint16_t)(y + FONT_PIXEL_HEIGHT - 1));
    if (result != ESP_OK) {
        return result;
    }
    return tft_write(true, s_tft_text_buffer,
                     width * FONT_PIXEL_HEIGHT * TFT_BYTES_PER_PIXEL);
}

static uint16_t rgb565(uint8_t red, uint8_t green, uint8_t blue)
{
    return (uint16_t)(((red & 0xF8) << 8) |
                      ((green & 0xFC) << 3) |
                      (blue >> 3));
}

esp_err_t tft_display_init(void)
{
    return tft_init();
}

esp_err_t tft_display_update(const tft_display_data_t *data)
{
    if (data == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    const uint16_t black = rgb565(0, 0, 0);
    const uint16_t white = rgb565(255, 255, 255);
    const uint16_t cyan = rgb565(0, 220, 255);
    const uint16_t yellow = rgb565(255, 220, 0);
    const uint16_t green = rgb565(0, 255, 100);
    char line[24];
    static bool static_ui_drawn = false;
    static char previous_a[sizeof(line)] = "";
    static char previous_b[sizeof(line)] = "";
    static char previous_d[sizeof(line)] = "";
    static char previous_distance[sizeof(line)] = "";

    if (!static_ui_drawn) {
        ESP_RETURN_ON_ERROR(tft_fill_screen(black), TAG, "clear TFT failed");
        ESP_RETURN_ON_ERROR(tft_draw_text_line(TFT_TEXT_X, 3, "SPEED RPM", cyan, black), TAG,
                            "draw TFT title failed");
        ESP_RETURN_ON_ERROR(tft_draw_text_line(TFT_TEXT_X, 106, "DISTANCE", yellow, black), TAG,
                            "draw distance label failed");
        static_ui_drawn = true;
    }

    snprintf(line, sizeof(line), "A %+4.1fRPM", data->wheel_a_rpm);
    if (strcmp(previous_a, line) != 0) {
        ESP_RETURN_ON_ERROR(tft_draw_text_line(TFT_TEXT_X, 29, line, white, black), TAG,
                            "draw wheel A failed");
        strcpy(previous_a, line);
    }

    snprintf(line, sizeof(line), "B %+4.1fRPM", data->wheel_b_rpm);
    if (strcmp(previous_b, line) != 0) {
        ESP_RETURN_ON_ERROR(tft_draw_text_line(TFT_TEXT_X, 53, line, white, black), TAG,
                            "draw wheel B failed");
        strcpy(previous_b, line);
    }

    snprintf(line, sizeof(line), "D %+4.1fRPM", data->wheel_d_rpm);
    if (strcmp(previous_d, line) != 0) {
        ESP_RETURN_ON_ERROR(tft_draw_text_line(TFT_TEXT_X, 77, line, white, black), TAG,
                            "draw wheel D failed");
        strcpy(previous_d, line);
    }

    if (data->distance_cm < 0.0f) {
        strcpy(line, "NO ECHO");
    } else {
        snprintf(line, sizeof(line), "%5.1f CM", data->distance_cm);
    }
    if (strcmp(previous_distance, line) != 0) {
        ESP_RETURN_ON_ERROR(tft_draw_text_line(TFT_TEXT_X, 121, line, green, black), TAG,
                            "draw distance value failed");
        strcpy(previous_distance, line);
    }
    return ESP_OK;
}
