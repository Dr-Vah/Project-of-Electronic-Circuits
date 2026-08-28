
#ifndef TFT_DISPLAY_H
#define TFT_DISPLAY_H

#include "esp_err.h"
#include "ultrasonic_sensor.h"

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

/* Override this at compile time when a different wheel is fitted. */
#ifndef TFT_DISPLAY_WHEEL_DIAMETER_M
#define TFT_DISPLAY_WHEEL_DIAMETER_M 0.057f
#endif

/** Initialize the ST7735S; the first update clears the display. */
esp_err_t tft_display_init(void);

/** Update the displayed values. Unchanged rows are not sent over SPI. */
esp_err_t tft_display_update(const tft_display_data_t *data);

/**
 * Read the three measured wheel speeds from car_control, convert them to RPM,
 * and display them together with an ultrasonic sample.  Passing NULL, or an
 * invalid ultrasonic sample, displays "NO ECHO" without starting another
 * blocking measurement.
 *
 * car_control_init() and ultrasonic_sensor_init() must be called before this
 * helper is used.  The wheel diameter can be overridden with
 * TFT_DISPLAY_WHEEL_DIAMETER_M (metres).
 */
esp_err_t tft_display_update_from_modules(
    const ultrasonic_sample_t *ultrasonic_sample);

#ifdef __cplusplus
}
#endif

#endif /* TFT_DISPLAY_H */

