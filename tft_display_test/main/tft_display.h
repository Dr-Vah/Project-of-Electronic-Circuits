
#ifndef TFT_DISPLAY_H
#define TFT_DISPLAY_H

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/** Values shown by the 128x160 ST7735S display. */
typedef struct {
    float wheel_a_rpm;
    float wheel_b_rpm;
    float wheel_d_rpm;
    /* Use a negative value when the ultrasonic sensor has no echo. */
    float distance_cm;
} tft_display_data_t;

/** Initialize the ST7735S; the first update clears the display. */
esp_err_t tft_display_init(void);

/** Update the displayed values. Unchanged rows are not sent over SPI. */
esp_err_t tft_display_update(const tft_display_data_t *data);

#ifdef __cplusplus
}
#endif

#endif /* TFT_DISPLAY_H */
