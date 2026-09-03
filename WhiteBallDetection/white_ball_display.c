#include "white_ball_display.h"

#include <string.h>

#include "chassis_camera_geometry.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_check.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define TFT_SDI_GPIO GPIO_NUM_42
#define TFT_SCK_GPIO GPIO_NUM_41
#define TFT_DC_GPIO GPIO_NUM_48
#define TFT_RST_GPIO GPIO_NUM_38
#define TFT_SPI_HOST SPI3_HOST
#define TFT_SPI_CLOCK_HZ (20 * 1000 * 1000)
#define TFT_WIDTH 128U
#define TFT_HEIGHT 160U
#define TFT_X_OFFSET 2U
#define TFT_Y_OFFSET 1U
/* This panel displays 0x001F as red when MADCTL.BGR is set. The JPEG decoder
 * produces RGB565, so keep the BGR bit clear to preserve red/blue order. */
#define TFT_MADCTL 0xC0U

#define COLOR_BLACK 0x0000U
#define COLOR_BOOT 0x001FU
#define COLOR_GREEN 0x07E0U
#define COLOR_CYAN 0x07FFU
#define COLOR_MAGENTA 0xF81FU
#define COLOR_YELLOW 0xFFE0U
#define COLOR_RED 0xF800U

static const char *TAG = "white_ball_display";
static spi_device_handle_t s_display;
/* Match the previously working camera_display driver: the complete row lives
 * in DMA-capable memory and is sent synchronously before it is reused. */
static DMA_ATTR uint8_t s_line[TFT_WIDTH * 2U];

static esp_err_t write_bytes(bool data, const void *bytes, size_t length)
{
    if (length == 0U) return ESP_OK;
    ESP_RETURN_ON_ERROR(gpio_set_level(TFT_DC_GPIO, data ? 1 : 0), TAG,
                        "set display D/C failed");
    spi_transaction_t transaction = {
        .length = length * 8U,
        .tx_buffer = bytes,
    };
    return spi_device_polling_transmit(s_display, &transaction);
}

static esp_err_t command(uint8_t value)
{
    return write_bytes(false, &value, 1U);
}

static esp_err_t command_data(uint8_t value, const uint8_t *data, size_t length)
{
    ESP_RETURN_ON_ERROR(command(value), TAG, "send display command failed");
    return write_bytes(true, data, length);
}

static esp_err_t set_window(uint16_t x0, uint16_t y0,
                            uint16_t x1, uint16_t y1)
{
    const uint16_t ax0 = x0 + TFT_X_OFFSET;
    const uint16_t ax1 = x1 + TFT_X_OFFSET;
    const uint16_t ay0 = y0 + TFT_Y_OFFSET;
    const uint16_t ay1 = y1 + TFT_Y_OFFSET;
    const uint8_t columns[] = {
        (uint8_t)(ax0 >> 8), (uint8_t)ax0,
        (uint8_t)(ax1 >> 8), (uint8_t)ax1,
    };
    const uint8_t rows[] = {
        (uint8_t)(ay0 >> 8), (uint8_t)ay0,
        (uint8_t)(ay1 >> 8), (uint8_t)ay1,
    };
    ESP_RETURN_ON_ERROR(command_data(0x2AU, columns, sizeof(columns)), TAG,
                        "set display columns failed");
    ESP_RETURN_ON_ERROR(command_data(0x2BU, rows, sizeof(rows)), TAG,
                        "set display rows failed");
    return command(0x2CU);
}

static void put_color(size_t x, uint16_t color)
{
    s_line[2U * x] = (uint8_t)(color >> 8);
    s_line[2U * x + 1U] = (uint8_t)color;
}

static esp_err_t fill_screen(uint16_t color)
{
    for (size_t x = 0U; x < TFT_WIDTH; ++x) put_color(x, color);
    ESP_RETURN_ON_ERROR(set_window(0U, 0U, TFT_WIDTH - 1U, TFT_HEIGHT - 1U),
                        TAG, "set clear window failed");
    for (size_t y = 0U; y < TFT_HEIGHT; ++y) {
        ESP_RETURN_ON_ERROR(write_bytes(true, s_line, sizeof(s_line)), TAG,
                            "clear display row failed");
    }
    return ESP_OK;
}

esp_err_t white_ball_display_init(void)
{
    if (s_display != NULL) return ESP_OK;
    const gpio_config_t control_gpio_config = {
        .pin_bit_mask = (1ULL << TFT_DC_GPIO) | (1ULL << TFT_RST_GPIO),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_RETURN_ON_ERROR(gpio_config(&control_gpio_config), TAG,
                        "configure display pins failed");

    const spi_bus_config_t bus_config = {
        .mosi_io_num = TFT_SDI_GPIO,
        .miso_io_num = GPIO_NUM_NC,
        .sclk_io_num = TFT_SCK_GPIO,
        .quadwp_io_num = GPIO_NUM_NC,
        .quadhd_io_num = GPIO_NUM_NC,
        .max_transfer_sz = sizeof(s_line),
    };
    ESP_RETURN_ON_ERROR(
        spi_bus_initialize(TFT_SPI_HOST, &bus_config, SPI_DMA_CH_AUTO), TAG,
        "initialize display SPI failed");
    const spi_device_interface_config_t device_config = {
        .clock_speed_hz = TFT_SPI_CLOCK_HZ,
        .mode = 0,
        .spics_io_num = -1,
        .queue_size = 1,
    };
    ESP_RETURN_ON_ERROR(
        spi_bus_add_device(TFT_SPI_HOST, &device_config, &s_display), TAG,
        "add display SPI device failed");

    gpio_set_level(TFT_RST_GPIO, 1);
    vTaskDelay(pdMS_TO_TICKS(20));
    gpio_set_level(TFT_RST_GPIO, 0);
    vTaskDelay(pdMS_TO_TICKS(30));
    gpio_set_level(TFT_RST_GPIO, 1);
    vTaskDelay(pdMS_TO_TICKS(150));

    ESP_RETURN_ON_ERROR(command(0x01U), TAG, "software reset failed");
    vTaskDelay(pdMS_TO_TICKS(150));
    ESP_RETURN_ON_ERROR(command(0x11U), TAG, "sleep out failed");
    vTaskDelay(pdMS_TO_TICKS(500));

    /* Full ST7735S green-tab initialization. The abbreviated reset/sleep-out
     * sequence is not sufficient for all LQ_TFT18SPIV33 panels. */
    const uint8_t frame_rate_normal[] = {0x01U, 0x2CU, 0x2DU};
    const uint8_t frame_rate_partial[] = {
        0x01U, 0x2CU, 0x2DU, 0x01U, 0x2CU, 0x2DU,
    };
    const uint8_t power_control_1[] = {0xA2U, 0x02U, 0x84U};
    const uint8_t power_control_2 = 0xC5U;
    const uint8_t power_control_3[] = {0x0AU, 0x00U};
    const uint8_t power_control_4[] = {0x8AU, 0x2AU};
    const uint8_t power_control_5[] = {0x8AU, 0xEEU};
    const uint8_t vcom_control = 0x0EU;
    const uint8_t inversion_control = 0x07U;
    const uint8_t gamma_positive[] = {
        0x02U, 0x1CU, 0x07U, 0x12U, 0x37U, 0x32U, 0x29U, 0x2DU,
        0x29U, 0x25U, 0x2BU, 0x39U, 0x00U, 0x01U, 0x03U, 0x10U,
    };
    const uint8_t gamma_negative[] = {
        0x03U, 0x1DU, 0x07U, 0x06U, 0x2EU, 0x2CU, 0x29U, 0x2DU,
        0x2EU, 0x2EU, 0x37U, 0x3FU, 0x00U, 0x00U, 0x02U, 0x10U,
    };
    const uint8_t columns[] = {
        0x00U, TFT_X_OFFSET,
        0x00U, TFT_X_OFFSET + TFT_WIDTH - 1U,
    };
    const uint8_t rows[] = {
        0x00U, TFT_Y_OFFSET,
        0x00U, TFT_Y_OFFSET + TFT_HEIGHT - 1U,
    };
    ESP_RETURN_ON_ERROR(command_data(0xB1U, frame_rate_normal,
                                      sizeof(frame_rate_normal)), TAG,
                        "set frame rate 1 failed");
    ESP_RETURN_ON_ERROR(command_data(0xB2U, frame_rate_normal,
                                      sizeof(frame_rate_normal)), TAG,
                        "set frame rate 2 failed");
    ESP_RETURN_ON_ERROR(command_data(0xB3U, frame_rate_partial,
                                      sizeof(frame_rate_partial)), TAG,
                        "set frame rate 3 failed");
    ESP_RETURN_ON_ERROR(command_data(0xB4U, &inversion_control, 1U), TAG,
                        "set inversion control failed");
    ESP_RETURN_ON_ERROR(command_data(0xC0U, power_control_1,
                                      sizeof(power_control_1)), TAG,
                        "set power control 1 failed");
    ESP_RETURN_ON_ERROR(command_data(0xC1U, &power_control_2, 1U), TAG,
                        "set power control 2 failed");
    ESP_RETURN_ON_ERROR(command_data(0xC2U, power_control_3,
                                      sizeof(power_control_3)), TAG,
                        "set power control 3 failed");
    ESP_RETURN_ON_ERROR(command_data(0xC3U, power_control_4,
                                      sizeof(power_control_4)), TAG,
                        "set power control 4 failed");
    ESP_RETURN_ON_ERROR(command_data(0xC4U, power_control_5,
                                      sizeof(power_control_5)), TAG,
                        "set power control 5 failed");
    ESP_RETURN_ON_ERROR(command_data(0xC5U, &vcom_control, 1U), TAG,
                        "set VCOM failed");
    const uint8_t pixel_format = 0x05U;
    const uint8_t madctl = TFT_MADCTL;
    ESP_RETURN_ON_ERROR(command_data(0x36U, &madctl, 1U), TAG,
                        "set orientation failed");
    ESP_RETURN_ON_ERROR(command_data(0x3AU, &pixel_format, 1U), TAG,
                        "set RGB565 mode failed");
    ESP_RETURN_ON_ERROR(command_data(0xE0U, gamma_positive,
                                      sizeof(gamma_positive)), TAG,
                        "set positive gamma failed");
    ESP_RETURN_ON_ERROR(command_data(0xE1U, gamma_negative,
                                      sizeof(gamma_negative)), TAG,
                        "set negative gamma failed");
    ESP_RETURN_ON_ERROR(command_data(0x2AU, columns, sizeof(columns)), TAG,
                        "set column range failed");
    ESP_RETURN_ON_ERROR(command_data(0x2BU, rows, sizeof(rows)), TAG,
                        "set row range failed");
    ESP_RETURN_ON_ERROR(command(0x20U), TAG, "disable inversion failed");
    ESP_RETURN_ON_ERROR(command(0x13U), TAG, "normal mode failed");
    ESP_RETURN_ON_ERROR(command(0x29U), TAG, "display on failed");
    vTaskDelay(pdMS_TO_TICKS(100));
    ESP_LOGI(TAG,
             "ST7735S ready: 128x160 MOSI=42 SCK=41 DC=48 RST=38 CS=GND");
    /* A blue screen proves that SPI and the panel work even before the USB
     * camera enumerates. It is replaced by the first decoded camera frame. */
    return fill_screen(COLOR_BOOT);
}

esp_err_t white_ball_display_show_rgb565(
    const uint16_t *pixels, size_t width, size_t height,
    bool rgb565_byte_swapped, const white_ball_result_t *ball,
    const black_target_result_t *target)
{
    if (pixels == NULL || width == 0U || height == 0U) {
        return ESP_ERR_INVALID_ARG;
    }
    if (s_display == NULL) return ESP_ERR_INVALID_STATE;

    size_t render_height = height * TFT_WIDTH / width;
    if (render_height == 0U) render_height = 1U;
    if (render_height > TFT_HEIGHT) render_height = TFT_HEIGHT;
    const size_t y0 = (TFT_HEIGHT - render_height) / 2U;
    const size_t chassis_left =
        (size_t)(CHASSIS_CORRECTED_CENTER_LEFT_FRACTION *
                 (TFT_WIDTH - 1U));
    const size_t push_axis =
        (size_t)(CHASSIS_CORRECTED_PUSH_AXIS_FRACTION *
                 (TFT_WIDTH - 1U));
    const size_t chassis_right =
        (size_t)(CHASSIS_CORRECTED_CENTER_RIGHT_FRACTION *
                 (TFT_WIDTH - 1U));

    bool draw_ball = ball != NULL && ball->valid;
    size_t box_left = 0U, box_right = 0U, box_top = 0U, box_bottom = 0U;
    size_t centre_x = 0U, centre_y = 0U;
    if (draw_ball) {
        box_left = ball->left * TFT_WIDTH / width;
        box_right = (ball->right + 1U) * TFT_WIDTH / width;
        if (box_right > 0U) --box_right;
        if (box_right >= TFT_WIDTH) box_right = TFT_WIDTH - 1U;
        box_top = y0 + ball->top * render_height / height;
        box_bottom = y0 + (ball->bottom + 1U) * render_height / height;
        if (box_bottom > y0) --box_bottom;
        if (box_bottom >= y0 + render_height) box_bottom = y0 + render_height - 1U;
        centre_x = (box_left + box_right) / 2U;
        centre_y = (box_top + box_bottom) / 2U;
    }

    const bool draw_target = target != NULL && target->valid;
    size_t target_left = 0U, target_right = 0U;
    size_t target_top = 0U, target_bottom = 0U;
    if (draw_target) {
        target_left = target->left * TFT_WIDTH / width;
        target_right = (target->right + 1U) * TFT_WIDTH / width;
        if (target_right > 0U) --target_right;
        if (target_right >= TFT_WIDTH) target_right = TFT_WIDTH - 1U;
        target_top = y0 + target->top * render_height / height;
        target_bottom = y0 + (target->bottom + 1U) * render_height / height;
        if (target_bottom > y0) --target_bottom;
        if (target_bottom >= y0 + render_height) {
            target_bottom = y0 + render_height - 1U;
        }
    }

    ESP_RETURN_ON_ERROR(
        set_window(0U, (uint16_t)y0, TFT_WIDTH - 1U,
                   (uint16_t)(y0 + render_height - 1U)),
        TAG, "set camera window failed");
    for (size_t output_y = 0U; output_y < render_height; ++output_y) {
        const size_t screen_y = y0 + output_y;
        const size_t source_y = output_y * height / render_height;
        for (size_t output_x = 0U; output_x < TFT_WIDTH; ++output_x) {
            const size_t source_x = output_x * width / TFT_WIDTH;
            uint16_t color = pixels[source_y * width + source_x];
            if (rgb565_byte_swapped) {
                color = (uint16_t)((color << 8) | (color >> 8));
            }
            if (output_x == chassis_left || output_x == chassis_right) {
                color = COLOR_YELLOW;
            }
            if (output_x == push_axis || output_x == push_axis + 1U) {
                color = COLOR_RED;
            }
            if (draw_ball) {
                const bool on_box =
                    output_x >= box_left && output_x <= box_right &&
                    screen_y >= box_top && screen_y <= box_bottom &&
                    (output_x <= box_left + 1U || output_x + 1U >= box_right ||
                     screen_y <= box_top + 1U || screen_y + 1U >= box_bottom);
                const bool on_cross =
                    ((screen_y == centre_y &&
                      output_x + 3U >= centre_x && output_x <= centre_x + 3U) ||
                     (output_x == centre_x &&
                      screen_y + 3U >= centre_y && screen_y <= centre_y + 3U));
                if (on_box) color = COLOR_GREEN;
                if (on_cross) color = COLOR_CYAN;
            }
            if (draw_target) {
                const bool on_target_box =
                    output_x >= target_left && output_x <= target_right &&
                    screen_y >= target_top && screen_y <= target_bottom &&
                    (output_x <= target_left + 1U ||
                     output_x + 1U >= target_right ||
                     screen_y <= target_top + 1U ||
                     screen_y + 1U >= target_bottom);
                if (on_target_box) color = COLOR_MAGENTA;
            }
            put_color(output_x, color);
        }
        ESP_RETURN_ON_ERROR(write_bytes(true, s_line, sizeof(s_line)), TAG,
                            "send camera row failed");
    }
    return ESP_OK;
}
