#include <stdbool.h>
#include <stdint.h>

#include "car_control.h"
#include "tft_display.h"
#include "ultrasonic_sensor.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define CONTROL_PERIOD_MS             20
#define STATUS_LOG_PERIOD_MS          500
#define TFT_UPDATE_PERIOD_MS          200
#define LOST_CONFIRM_MS               150
#define SEARCH_TIMEOUT_MS             9000
#define FIRST_CORNER_IGNORE_LINE_MS   1000
#define NORMAL_SEARCH_IGNORE_LINE_MS  400
#define RECOVERY_GUARD_MS             300
#define REACQUIRE_SAMPLES             3
#define OPPOSITE_OUTER_SAMPLES        3
#define ULTRASONIC_SAMPLE_PERIOD_MS   50
#define OBSTACLE_TRIGGER_CM           5.0f
#define OBSTACLE_CONFIRM_SAMPLES      1
#define OBSTACLE_STOP_SETTLE_MS       150
#define OBSTACLE_CLEAR_CM             150.0f
#define OBSTACLE_CLEAR_SAMPLES        3
#define OBSTACLE_LEFT_EXTRA_TIME_MS   200
#define OBSTACLE_LINE_SAMPLES         1
#define OBSTACLE_LEFT_STRAFE_SPEED_MPS  0.08f
#define OBSTACLE_RETURN_SPEED_MPS        0.08f
#define OBSTACLE_FORWARD_SPEED_MPS    0.09f
#define OBSTACLE_STRAFE_TIMEOUT_MS    5000
#define OBSTACLE_FORWARD_TIME_MS      1300
#define OBSTACLE_RECOVERY_GUARD_MS    8000
#define ULTRASONIC_DEBUG_LOG          1

/* LQ_R4CHVB manual: black outputs low, white outputs high. */
#define IR_BLACK_LEVEL 0
#define FORWARD_SPEED_MPS       0.06f
#define ARC_FORWARD_SPEED_MPS   0.05f
#define ARC_TURN_SPEED_RAD_S    0.30f
#define INNER_TURN_SPEED_RAD_S  0.08f
#define SEARCH_SPEED_RAD_S      0.40f
#define OUTER_ERROR_THRESHOLD   1.50f

#define IR_LEFT_OUTER_MASK  0x08U
#define IR_LEFT_INNER_MASK  0x04U
#define IR_RIGHT_INNER_MASK 0x02U
#define IR_RIGHT_OUTER_MASK 0x01U
#define IR_MIDDLE_MASK (IR_LEFT_INNER_MASK | IR_RIGHT_INNER_MASK)

static const char *TAG = "LINE_TRACK";

typedef enum {
    TRACK_WAIT_START,
    TRACK_FORWARD,
    TRACK_TRIM_LEFT,
    TRACK_TRIM_RIGHT,
    TRACK_CORRECT_LEFT,
    TRACK_CORRECT_RIGHT,
    TRACK_GAP_HOLD,
    TRACK_FIRST_CORNER_RIGHT,
    TRACK_SEARCH_LEFT,
    TRACK_SEARCH_RIGHT,
    TRACK_FINISHED,
    TRACK_STOPPED,
} tracking_state_t;

typedef enum {
    DIRECTION_LEFT = -1,
    DIRECTION_NONE = 0,
    DIRECTION_RIGHT = 1,
} turn_direction_t;

typedef struct {
    int level[4];
    bool black[4];
    uint8_t raw_black_pattern;
    uint8_t pattern;
    uint8_t black_count;
    float error;
} infrared_data_t;

typedef struct {
    tracking_state_t state;
    turn_direction_t last_line_direction;
    turn_direction_t correction_direction;
    int64_t lost_since_us;
    int64_t search_since_us;
    int64_t ignore_line_until_us;
    int64_t recovery_guard_until_us;
    uint8_t reacquire_samples;
    uint8_t opposite_outer_samples;
    bool has_seen_line;
    bool first_corner_done;
} tracking_context_t;

typedef enum {
    AVOID_IDLE,
    AVOID_STRAFE_LEFT,
    AVOID_STRAFE_LEFT_EXTRA,
    AVOID_FORWARD,
    AVOID_STRAFE_RIGHT,
    AVOID_FAULT,
} avoidance_state_t;

typedef struct {
    avoidance_state_t state;
    int64_t phase_deadline_us;
    int64_t next_sample_us;
    uint8_t near_samples;
    uint8_t clear_samples;
    uint8_t line_samples;
    ultrasonic_sample_t latest_sample;
} avoidance_context_t;

static const char *avoidance_state_name(avoidance_state_t state)
{
    switch (state) {
    case AVOID_IDLE:
        return "IDLE";
    case AVOID_STRAFE_LEFT:
        return "STRAFE_LEFT";
    case AVOID_STRAFE_LEFT_EXTRA:
        return "STRAFE_LEFT_EXTRA";
    case AVOID_FORWARD:
        return "FORWARD";
    case AVOID_STRAFE_RIGHT:
        return "STRAFE_RIGHT";
    case AVOID_FAULT:
        return "FAULT";
    default:
        return "UNKNOWN";
    }
}

static void log_ultrasonic_sample(const avoidance_context_t *avoidance)
{
#if ULTRASONIC_DEBUG_LOG
    if (avoidance->latest_sample.valid) {
        ESP_LOGI(TAG,
                 "ultrasonic: time=%lld ms valid=1 distance=%.1f cm "
                 "avoid=%s near=%u clear=%u",
                 (long long)(avoidance->latest_sample.timestamp_us / 1000),
                 avoidance->latest_sample.distance_cm,
                 avoidance_state_name(avoidance->state),
                 avoidance->near_samples, avoidance->clear_samples);
    } else {
        const char *reason = "unknown";
        switch (avoidance->latest_sample.status) {
        case ULTRASONIC_WAIT_RISE_TIMEOUT:
            reason = "echo-never-high";
            break;
        case ULTRASONIC_PULSE_TIMEOUT:
            reason = "echo-stuck-high-or-too-far";
            break;
        case ULTRASONIC_OUT_OF_RANGE:
            reason = "out-of-range";
            break;
        case ULTRASONIC_SAMPLE_OK:
        default:
            break;
        }
        ESP_LOGW(TAG,
                 "ultrasonic: time=%lld ms valid=0 reason=%s pulse=%lld us "
                 "distance=invalid avoid=%s near=%u clear=%u",
                 (long long)(avoidance->latest_sample.timestamp_us / 1000),
                 reason, (long long)avoidance->latest_sample.pulse_us,
                 avoidance_state_name(avoidance->state),
                 avoidance->near_samples, avoidance->clear_samples);
    }
#else
    (void)avoidance;
#endif
}

static const char *tracking_state_name(tracking_state_t state)
{
    switch (state) {
    case TRACK_WAIT_START:
        return "WAIT_START";
    case TRACK_FORWARD:
        return "FORWARD";
    case TRACK_TRIM_LEFT:
        return "TRIM_LEFT";
    case TRACK_TRIM_RIGHT:
        return "TRIM_RIGHT";
    case TRACK_CORRECT_LEFT:
        return "CORRECT_LEFT";
    case TRACK_CORRECT_RIGHT:
        return "CORRECT_RIGHT";
    case TRACK_GAP_HOLD:
        return "GAP_HOLD";
    case TRACK_FIRST_CORNER_RIGHT:
        return "FIRST_CORNER_RIGHT";
    case TRACK_SEARCH_LEFT:
        return "SEARCH_LEFT";
    case TRACK_SEARCH_RIGHT:
        return "SEARCH_RIGHT";
    case TRACK_FINISHED:
        return "FINISHED";
    case TRACK_STOPPED:
        return "STOPPED";
    default:
        return "UNKNOWN";
    }
}

static const char *direction_name(turn_direction_t direction)
{
    switch (direction) {
    case DIRECTION_LEFT:
        return "LEFT";
    case DIRECTION_RIGHT:
        return "RIGHT";
    default:
        return "NONE";
    }
}

static bool state_is_searching(tracking_state_t state)
{
    return state == TRACK_FIRST_CORNER_RIGHT ||
           state == TRACK_SEARCH_LEFT || state == TRACK_SEARCH_RIGHT;
}

static bool majority_of_three(uint8_t history)
{
    const unsigned int ones = (history & 1U) + ((history >> 1) & 1U) +
                              ((history >> 2) & 1U);
    return ones >= 2U;
}

static esp_err_t infrared_init(void)
{
    const gpio_config_t config = {
        .pin_bit_mask = (1ULL << CAR_IR_OUT1_GPIO) |
                        (1ULL << CAR_IR_OUT2_GPIO) |
                        (1ULL << CAR_IR_OUT3_GPIO) |
                        (1ULL << CAR_IR_OUT4_GPIO),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    return gpio_config(&config);
}

static infrared_data_t infrared_read(void)
{
    static const gpio_num_t pins[4] = {
        CAR_IR_OUT4_GPIO,
        CAR_IR_OUT3_GPIO,
        CAR_IR_OUT2_GPIO,
        CAR_IR_OUT1_GPIO,
    };
    static const float weights[4] = {-3.0f, -1.0f, 1.0f, 3.0f};
    static uint8_t history[4];

    infrared_data_t data = {0};
    float weighted_sum = 0.0f;

    for (int i = 0; i < 4; ++i) {
        data.level[i] = gpio_get_level(pins[i]);
        const bool raw_black = data.level[i] == IR_BLACK_LEVEL;
        history[i] = (uint8_t)(((history[i] << 1) | raw_black) & 0x07U);
        data.black[i] = majority_of_three(history[i]);

        data.raw_black_pattern =
            (uint8_t)((data.raw_black_pattern << 1) | raw_black);
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

static tracking_state_t apply_search(tracking_context_t *context)
{
    if (context->state == TRACK_FIRST_CORNER_RIGHT ||
        context->last_line_direction == DIRECTION_RIGHT) {
        ESP_ERROR_CHECK(car_control_turn_in_place(-SEARCH_SPEED_RAD_S));
        return context->state == TRACK_FIRST_CORNER_RIGHT
                   ? TRACK_FIRST_CORNER_RIGHT
                   : TRACK_SEARCH_RIGHT;
    }

    ESP_ERROR_CHECK(car_control_turn_in_place(SEARCH_SPEED_RAD_S));
    return TRACK_SEARCH_LEFT;
}

static tracking_state_t begin_search(tracking_context_t *context,
                                     int64_t now_us)
{
    context->search_since_us = now_us;
    context->reacquire_samples = 0;
    context->correction_direction = DIRECTION_NONE;
    context->opposite_outer_samples = 0;

    if (!context->first_corner_done) {
        context->first_corner_done = true;
        context->last_line_direction = DIRECTION_RIGHT;
        context->state = TRACK_FIRST_CORNER_RIGHT;
        context->ignore_line_until_us =
            now_us + FIRST_CORNER_IGNORE_LINE_MS * 1000LL;
        ESP_LOGI(TAG, "first sustained line loss: force the 120-degree "
                      "corner search to the right");
    } else {
        if (context->last_line_direction == DIRECTION_NONE) {
            ESP_ERROR_CHECK(car_control_stop());
            context->state = TRACK_STOPPED;
            return context->state;
        }

        context->state = context->last_line_direction == DIRECTION_LEFT
                             ? TRACK_SEARCH_LEFT
                             : TRACK_SEARCH_RIGHT;
        context->ignore_line_until_us =
            now_us + NORMAL_SEARCH_IGNORE_LINE_MS * 1000LL;
    }

    return apply_search(context);
}

static tracking_state_t track_visible_line(const infrared_data_t *ir,
                                           tracking_context_t *context)
{
    turn_direction_t desired_correction = DIRECTION_NONE;
    if (ir->error <= -OUTER_ERROR_THRESHOLD) {
        desired_correction = DIRECTION_LEFT;
    } else if (ir->error >= OUTER_ERROR_THRESHOLD) {
        desired_correction = DIRECTION_RIGHT;
    }

    if (desired_correction == DIRECTION_NONE) {
        context->correction_direction = DIRECTION_NONE;
        context->opposite_outer_samples = 0;

        /* A single middle sensor means the line is only slightly off-centre. */
        if (ir->pattern == IR_LEFT_INNER_MASK) {
            ESP_ERROR_CHECK(car_control_set_velocity(
                0.0f, FORWARD_SPEED_MPS, INNER_TURN_SPEED_RAD_S));
            return TRACK_TRIM_LEFT;
        }
        if (ir->pattern == IR_RIGHT_INNER_MASK) {
            ESP_ERROR_CHECK(car_control_set_velocity(
                0.0f, FORWARD_SPEED_MPS, -INNER_TURN_SPEED_RAD_S));
            return TRACK_TRIM_RIGHT;
        }

        ESP_ERROR_CHECK(car_control_forward(FORWARD_SPEED_MPS));
        return TRACK_FORWARD;
    }

    if (context->correction_direction != DIRECTION_NONE &&
        desired_correction != context->correction_direction) {
        ++context->opposite_outer_samples;
        if (context->opposite_outer_samples < OPPOSITE_OUTER_SAMPLES) {
            ESP_ERROR_CHECK(car_control_forward(FORWARD_SPEED_MPS));
            return TRACK_FORWARD;
        }
    }

    context->correction_direction = desired_correction;
    context->opposite_outer_samples = 0;

    if (desired_correction == DIRECTION_LEFT) {
        ESP_ERROR_CHECK(car_control_set_velocity(
            0.0f, ARC_FORWARD_SPEED_MPS, ARC_TURN_SPEED_RAD_S));
        return TRACK_CORRECT_LEFT;
    }

    ESP_ERROR_CHECK(car_control_set_velocity(
        0.0f, ARC_FORWARD_SPEED_MPS, -ARC_TURN_SPEED_RAD_S));
    return TRACK_CORRECT_RIGHT;
}

static tracking_state_t tracking_update(const infrared_data_t *ir,
                                        tracking_context_t *context)
{
    const int64_t now_us = esp_timer_get_time();

    /* All-black is the finish marker only during normal tracking. */
    if (!state_is_searching(context->state) &&
        ir->raw_black_pattern == 0x0FU) {
        ESP_ERROR_CHECK(car_control_stop());
        context->state = TRACK_FINISHED;
        return context->state;
    }

    if (state_is_searching(context->state)) {
        if ((now_us - context->search_since_us) / 1000 >=
            SEARCH_TIMEOUT_MS) {
            ESP_ERROR_CHECK(car_control_stop());
            context->state = TRACK_STOPPED;
            return context->state;
        }

        /* During rotation, all-black is neither finish nor reacquisition. */
        const bool all_black = ir->raw_black_pattern == 0x0FU ||
                               ir->pattern == 0x0FU;
        const bool middle_sees_line = !all_black &&
                                      (ir->pattern & IR_MIDDLE_MASK) != 0U;
        if (now_us >= context->ignore_line_until_us && middle_sees_line) {
            ++context->reacquire_samples;
            if (context->reacquire_samples >= REACQUIRE_SAMPLES) {
                context->state = TRACK_FORWARD;
                context->lost_since_us = 0;
                context->search_since_us = 0;
                context->reacquire_samples = 0;
                context->recovery_guard_until_us =
                    now_us + RECOVERY_GUARD_MS * 1000LL;
                context->correction_direction = DIRECTION_NONE;
                ESP_ERROR_CHECK(car_control_forward(FORWARD_SPEED_MPS));
                return context->state;
            }
        } else {
            context->reacquire_samples = 0;
        }

        context->state = apply_search(context);
        return context->state;
    }

    if (ir->black_count == 0) {
        context->reacquire_samples = 0;

        if (!context->has_seen_line) {
            ESP_ERROR_CHECK(car_control_stop());
            context->state = TRACK_WAIT_START;
            return context->state;
        }

        if (context->state == TRACK_STOPPED) {
            ESP_ERROR_CHECK(car_control_stop());
            return context->state;
        }

        if (context->lost_since_us == 0) {
            context->lost_since_us = now_us;
        }

        const int64_t lost_ms =
            (now_us - context->lost_since_us) / 1000;
        if (now_us < context->recovery_guard_until_us ||
            lost_ms < LOST_CONFIRM_MS) {
            context->state = TRACK_GAP_HOLD;
            return context->state;
        }

        return begin_search(context, now_us);
    }

    context->has_seen_line = true;
    context->lost_since_us = 0;

    if (ir->error < 0.0f) {
        context->last_line_direction = DIRECTION_LEFT;
    } else if (ir->error > 0.0f) {
        context->last_line_direction = DIRECTION_RIGHT;
    }

    context->state = track_visible_line(ir, context);
    return context->state;
}

static bool avoidance_can_start(const tracking_context_t *tracking)
{
    return tracking->has_seen_line &&
           tracking->state != TRACK_FINISHED &&
           tracking->state != TRACK_STOPPED &&
           !state_is_searching(tracking->state);
}

static void avoidance_start(avoidance_context_t *avoidance)
{
    ESP_LOGW(TAG, "obstacle confirmed at %.1f cm; start left bypass",
             avoidance->latest_sample.distance_cm);
    ESP_ERROR_CHECK(car_control_stop());
    /* At 5 cm there is little room left. Let the forward motion settle before
     * starting the direct lateral translation. */
    vTaskDelay(pdMS_TO_TICKS(OBSTACLE_STOP_SETTLE_MS));
    ESP_ERROR_CHECK(
        car_control_strafe_left(OBSTACLE_LEFT_STRAFE_SPEED_MPS));
    avoidance->state = AVOID_STRAFE_LEFT;
    /* No timeout here: keep moving left until the ultrasonic clear condition
     * is observed. The return-to-line phase retains its timeout guard. */
    avoidance->phase_deadline_us = 0;
    avoidance->next_sample_us = 0;
    avoidance->near_samples = 0;
    avoidance->clear_samples = 0;
    avoidance->line_samples = 0;
}

static void avoidance_enter_fault(avoidance_context_t *avoidance,
                                  tracking_context_t *tracking,
                                  const char *reason)
{
    ESP_ERROR_CHECK(car_control_stop());
    avoidance->state = AVOID_FAULT;
    tracking->state = TRACK_STOPPED;
    ESP_LOGE(TAG, "avoidance fault: %s; motors stopped", reason);
}

/** Return true while avoidance owns the motor command. */
static bool avoidance_update(avoidance_context_t *avoidance,
                             tracking_context_t *tracking,
                             const infrared_data_t *ir, int64_t now_us)
{
    if (avoidance->state == AVOID_IDLE) {
        if (now_us >= avoidance->next_sample_us) {
            ESP_ERROR_CHECK(ultrasonic_sensor_read(&avoidance->latest_sample));
            avoidance->next_sample_us =
                avoidance->latest_sample.timestamp_us +
                ULTRASONIC_SAMPLE_PERIOD_MS * 1000LL;

            if (avoidance->latest_sample.valid &&
                avoidance->latest_sample.distance_cm <= OBSTACLE_TRIGGER_CM) {
                if (avoidance->near_samples < UINT8_MAX) {
                    ++avoidance->near_samples;
                }
            } else {
                /* Invalid/timeout data never masquerades as a near obstacle. */
                avoidance->near_samples = 0;
            }
            log_ultrasonic_sample(avoidance);
        }

        if (avoidance->near_samples >= OBSTACLE_CONFIRM_SAMPLES &&
            avoidance_can_start(tracking)) {
            avoidance_start(avoidance);
            return true;
        }
        return false;
    }

    if (avoidance->state == AVOID_FAULT) {
        return true;
    }

    switch (avoidance->state) {
    case AVOID_STRAFE_LEFT:
        if (now_us >= avoidance->next_sample_us) {
            ESP_ERROR_CHECK(ultrasonic_sensor_read(&avoidance->latest_sample));
            avoidance->next_sample_us =
                avoidance->latest_sample.timestamp_us +
                ULTRASONIC_SAMPLE_PERIOD_MS * 1000LL;

            if (avoidance->latest_sample.valid &&
                avoidance->latest_sample.distance_cm >= OBSTACLE_CLEAR_CM) {
                if (avoidance->clear_samples < UINT8_MAX) {
                    ++avoidance->clear_samples;
                }
            } else {
                avoidance->clear_samples = 0;
            }
            log_ultrasonic_sample(avoidance);
        }

        if (avoidance->clear_samples < OBSTACLE_CLEAR_SAMPLES) {
            return true;
        }

        /* Keep the existing left command briefly after the sensor first sees
         * clear space, giving the chassis a little more side clearance. */
        avoidance->state = AVOID_STRAFE_LEFT_EXTRA;
        avoidance->phase_deadline_us =
            esp_timer_get_time() + OBSTACLE_LEFT_EXTRA_TIME_MS * 1000LL;
        ESP_LOGI(TAG,
                 "avoidance: obstacle clear at %.1f cm, continue left %d ms",
                 avoidance->latest_sample.distance_cm,
                 OBSTACLE_LEFT_EXTRA_TIME_MS);
        return true;

    case AVOID_STRAFE_LEFT_EXTRA:
        if (now_us < avoidance->phase_deadline_us) {
            return true;
        }
        ESP_ERROR_CHECK(car_control_stop());
        ESP_ERROR_CHECK(car_control_forward(OBSTACLE_FORWARD_SPEED_MPS));
        avoidance->state = AVOID_FORWARD;
        avoidance->phase_deadline_us =
            esp_timer_get_time() + OBSTACLE_FORWARD_TIME_MS * 1000LL;
        ESP_LOGI(TAG, "avoidance: extra left clearance complete, pass obstacle");
        return true;

    case AVOID_FORWARD:
        if (now_us < avoidance->phase_deadline_us) {
            return true;
        }
        ESP_ERROR_CHECK(car_control_stop());
        ESP_ERROR_CHECK(
            car_control_strafe_left(-OBSTACLE_RETURN_SPEED_MPS));
        avoidance->state = AVOID_STRAFE_RIGHT;
        avoidance->phase_deadline_us =
            esp_timer_get_time() + OBSTACLE_STRAFE_TIMEOUT_MS * 1000LL;
        avoidance->line_samples = 0;
        ESP_LOGI(TAG, "avoidance: obstacle passed, strafe right to the line");
        return true;

    case AVOID_STRAFE_RIGHT:
        const bool right_outer_sees_line =
            ir->pattern != 0x0FU &&
            (ir->pattern & IR_RIGHT_OUTER_MASK) != 0U;
        if (right_outer_sees_line) {
            if (avoidance->line_samples < UINT8_MAX) {
                ++avoidance->line_samples;
            }
        } else {
            avoidance->line_samples = 0;
        }

        if (avoidance->line_samples < OBSTACLE_LINE_SAMPLES) {
            if (now_us >= avoidance->phase_deadline_us) {
                avoidance_enter_fault(avoidance, tracking,
                                      "right strafe did not find line");
            }
            return true;
        }

        ESP_ERROR_CHECK(car_control_stop());
        ESP_ERROR_CHECK(car_control_forward(FORWARD_SPEED_MPS));
        avoidance->state = AVOID_IDLE;
        avoidance->phase_deadline_us = 0;
        avoidance->near_samples = 0;
        avoidance->next_sample_us =
            esp_timer_get_time() + OBSTACLE_RECOVERY_GUARD_MS * 1000LL;

        /* Re-enter line tracking as a guarded line gap, not as a corner. */
        tracking->state = TRACK_GAP_HOLD;
        tracking->lost_since_us = esp_timer_get_time();
        tracking->search_since_us = 0;
        tracking->reacquire_samples = 0;
        tracking->correction_direction = DIRECTION_NONE;
        tracking->opposite_outer_samples = 0;
        tracking->recovery_guard_until_us =
            esp_timer_get_time() + OBSTACLE_RECOVERY_GUARD_MS * 1000LL;
        ESP_LOGI(TAG, "avoidance complete; line reacquired with pattern=0x%X",
                 ir->pattern);
        return false;

    case AVOID_FAULT:
        return true;

    case AVOID_IDLE:
    default:
        ESP_ERROR_CHECK(car_control_stop());
        avoidance->state = AVOID_IDLE;
        return false;
    }
}

void app_main(void)
{
    const car_control_config_t motor_config = CAR_CONTROL_DEFAULT_CONFIG();
    const ultrasonic_sensor_config_t ultrasonic_config =
        ULTRASONIC_SENSOR_DEFAULT_CONFIG();
    ESP_ERROR_CHECK(car_control_init(&motor_config));
    ESP_ERROR_CHECK(infrared_init());
    ESP_ERROR_CHECK(ultrasonic_sensor_init(&ultrasonic_config));
    ESP_ERROR_CHECK(tft_display_init());
    ESP_ERROR_CHECK(car_control_stop());

    tracking_context_t tracking = {
        .state = TRACK_WAIT_START,
        .last_line_direction = DIRECTION_NONE,
        .correction_direction = DIRECTION_NONE,
    };
    avoidance_context_t avoidance = {
        .state = AVOID_IDLE,
    };
    int64_t last_log_us = 0;
    int64_t last_display_update_us = 0;
    tracking_state_t previous_state = TRACK_WAIT_START;
    uint8_t previous_pattern = UINT8_MAX;

    ESP_LOGI(TAG, "course tracker started: physical left-to-right is "
                  "OUT4, OUT3, OUT2, OUT1");
    ESP_LOGI(TAG,
             "black level=%d, forward=%.2f m/s, inner trim=%.2f rad/s, "
             "arc=%.2f m/s at %.2f rad/s, search=%.2f rad/s",
             IR_BLACK_LEVEL, FORWARD_SPEED_MPS, INNER_TURN_SPEED_RAD_S,
             ARC_FORWARD_SPEED_MPS, ARC_TURN_SPEED_RAD_S,
             SEARCH_SPEED_RAD_S);
    ESP_LOGI(TAG,
             "one middle sensor trims gently; both middle sensors go "
             "straight; outer sensors correct; first "
             "sustained loss forces a right corner; all-black stops only "
             "during normal tracking");
    ESP_LOGI(TAG,
             "obstacle sensor: TRIG=GPIO%d ECHO=GPIO%d, trigger=%.1f cm "
             "for %d samples; left until %.1f cm for %d samples, "
             "forward %.0f ms, right until OUT1 sees line",
             ultrasonic_config.trig_gpio, ultrasonic_config.echo_gpio,
             OBSTACLE_TRIGGER_CM, OBSTACLE_CONFIRM_SAMPLES,
             OBSTACLE_CLEAR_CM, OBSTACLE_CLEAR_SAMPLES,
             (double)OBSTACLE_FORWARD_TIME_MS);

    while (true) {
        const infrared_data_t ir = infrared_read();
        const int64_t now_us = esp_timer_get_time();
        const bool avoiding =
            avoidance_update(&avoidance, &tracking, &ir, now_us);
        const tracking_state_t state = avoiding
                                           ? tracking.state
                                           : tracking_update(&ir, &tracking);
        const int64_t line_lost_ms = tracking.lost_since_us == 0
                                         ? 0
                                         : (now_us - tracking.lost_since_us) /
                                               1000;

        if ((now_us - last_display_update_us) / 1000 >=
            TFT_UPDATE_PERIOD_MS) {
            const esp_err_t display_result =
                tft_display_update_from_modules(&avoidance.latest_sample);
            if (display_result != ESP_OK) {
                ESP_LOGW(TAG, "TFT update failed: %s",
                         esp_err_to_name(display_result));
            }
            last_display_update_us = now_us;
        }

        if (ir.pattern != previous_pattern) {
            ESP_LOGI(TAG,
                     "sensor: level=[%d %d %d %d], black=[%d %d %d %d], "
                     "raw=0x%X filtered=0x%X error=%+.2f",
                     ir.level[0], ir.level[1], ir.level[2], ir.level[3],
                     ir.black[0], ir.black[1], ir.black[2], ir.black[3],
                     ir.raw_black_pattern, ir.pattern, ir.error);
            previous_pattern = ir.pattern;
        }

        if (state != previous_state) {
            ESP_LOGI(TAG,
                     "state: %s -> %s, last_line=%s, lost=%lld ms, "
                     "first_corner_done=%d",
                     tracking_state_name(previous_state),
                     tracking_state_name(state),
                     direction_name(tracking.last_line_direction),
                     (long long)line_lost_ms, tracking.first_corner_done);
            previous_state = state;
        }

        if ((now_us - last_log_us) / 1000 >= STATUS_LOG_PERIOD_MS) {
            ESP_LOGI(TAG,
                     "status: black=%d%d%d%d pattern=0x%X error=%+.2f "
                     "state=%s avoid=%s distance=%s%.1f cm last=%s "
                     "lost=%lld ms",
                     ir.black[0], ir.black[1], ir.black[2], ir.black[3],
                     ir.pattern, ir.error, tracking_state_name(state),
                     avoidance_state_name(avoidance.state),
                     avoidance.latest_sample.valid ? "" : "invalid/",
                     avoidance.latest_sample.distance_cm,
                     direction_name(tracking.last_line_direction),
                     (long long)line_lost_ms);
            last_log_us = now_us;
        }

        if (state == TRACK_FINISHED) {
            ESP_LOGI(TAG, "finish marker detected: all four sensors are "
                          "black; motors stopped");
            break;
        }

        vTaskDelay(pdMS_TO_TICKS(CONTROL_PERIOD_MS));
    }
}

