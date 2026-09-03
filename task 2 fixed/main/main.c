#include <stdbool.h>
#include <stdint.h>
#include <math.h>

#include "camera_line_sensor.h"
#include "car_control.h"
#include "tft_display.h"
#include "ultrasonic_sensor.h"
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
#define ULTRASONIC_SAMPLE_PERIOD_MS   50
#define OBSTACLE_TRIGGER_CM           6.5f    // 预留 1.5cm 距离，防止撞到障碍物
#define OBSTACLE_CONFIRM_SAMPLES      1
#define OBSTACLE_STOP_SETTLE_MS       150
#define OBSTACLE_CLEAR_CM             150.0f
#define OBSTACLE_CLEAR_SAMPLES        1
#define OBSTACLE_LEFT_EXTRA_TIME_MS   200
#define OBSTACLE_LINE_SAMPLES         1
#define OBSTACLE_LEFT_STRAFE_SPEED_MPS  0.06f
#define OBSTACLE_RETURN_SPEED_MPS        0.04f
#define OBSTACLE_FORWARD_SPEED_MPS    0.06f
#define OBSTACLE_STRAFE_TIMEOUT_MS    5000
#define OBSTACLE_FORWARD_TIME_MS      2400
#define OBSTACLE_RECOVERY_GUARD_MS    8000
#define LINE_RETURN_CENTER_ERROR      0.60f
#define ULTRASONIC_DEBUG_LOG          1

/* Kept for the existing logic: virtual camera black is represented as low. */
#define IR_BLACK_LEVEL 0
#define FORWARD_SPEED_MPS       0.04f
#define ARC_FORWARD_SPEED_MPS   0.035f
#define ARC_TURN_SPEED_RAD_S    0.20f
#define INNER_TURN_SPEED_RAD_S  0.05f
#define SEARCH_SPEED_RAD_S      0.25f
#define LINE_STEER_KP           0.40f
#define LINE_STEER_KD           0.08f
#define LINE_STEER_MAX_OMEGA    0.20f
#define LINE_STEER_DEADBAND     0.01f
#define LINE_ERROR_FILTER_ALPHA 0.60f

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
    TRACK_FINISH_FORWARD,
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
    /* Unfiltered centroid error this frame (-1..+1, historical sign). */
    float raw_error;
    /* Dark pixels inside the trimmed centroid ROI; 0 means no line there. */
    uint32_t line_pixels;
} infrared_data_t;

typedef struct {
    tracking_state_t state;
    turn_direction_t last_line_direction;
    int64_t lost_since_us;
    int64_t search_since_us;
    int64_t ignore_line_until_us;
    int64_t recovery_guard_until_us;
    uint8_t reacquire_samples;
    bool has_seen_line;
    bool first_corner_done;
    bool finish_detection_enabled;
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
    case TRACK_FINISH_FORWARD:
        return "FINISH_FORWARD";
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
    return camera_line_sensor_init();
}

static infrared_data_t infrared_read(void)
{
    static uint8_t history[4];
    static float error_filtered = 0.0f;
    camera_line_state_t camera_state;

    /* A camera that has not supplied a valid frame must look like white,
     * so task 1 stays safely stopped in TRACK_WAIT_START. */
    infrared_data_t data = {.level = {1, 1, 1, 1}};

    if (!camera_line_sensor_get_state(&camera_state)) {
        return data;
    }

    for (int i = 0; i < 4; ++i) {
        /* The camera is mounted facing the vehicle, so image-left/image-right
         * is the reverse of task 1's historical OUT4..OUT1 sensor order. */
        const bool raw_black =
            camera_state.black[CAMERA_LINE_CHANNEL_COUNT - 1U - (unsigned)i];
        data.level[i] = raw_black ? IR_BLACK_LEVEL : !IR_BLACK_LEVEL;
        history[i] = (uint8_t)(((history[i] << 1) | raw_black) & 0x07U);
        data.black[i] = majority_of_three(history[i]);

        data.raw_black_pattern =
            (uint8_t)((data.raw_black_pattern << 1) | raw_black);
        data.pattern = (uint8_t)((data.pattern << 1) | data.black[i]);

        if (data.black[i]) {
            ++data.black_count;
        }
    }

    /* Continuous line position from the side-trimmed ROI centroid.  Negate so
     * the sign keeps the historical meaning: positive error = line on the
     * left, negative = line on the right. */
    const float raw_error = (camera_state.line_pixels != 0)
                                ? -camera_state.line_error
                                : 0.0f;
    error_filtered +=
        LINE_ERROR_FILTER_ALPHA * (raw_error - error_filtered);
    data.error = error_filtered;
    data.raw_error = raw_error;
    data.line_pixels = camera_state.line_pixels;

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
    (void)context;

    /* PD steering from the side-trimmed ROI centroid.  The error sign keeps
     * the historical convention (positive = line on the left), mapped to a
     * rightward (negative) angular velocity.  The derivative term damps the
     * side-to-side oscillation on straight sections. */
    static float prev_error = 0.0f;
    static int64_t prev_us = 0;
    const int64_t now_us = esp_timer_get_time();
    const float dt_s = (float)(now_us - prev_us) / 1000000.0f;
    const float d_error = dt_s > 0.0f ? (ir->error - prev_error) / dt_s : 0.0f;
    prev_error = ir->error;
    prev_us = now_us;

    float omega = -LINE_STEER_KP * ir->error - LINE_STEER_KD * d_error;
    if (omega > LINE_STEER_MAX_OMEGA) {
        omega = LINE_STEER_MAX_OMEGA;
    } else if (omega < -LINE_STEER_MAX_OMEGA) {
        omega = -LINE_STEER_MAX_OMEGA;
    }

    if (fabsf(omega) < LINE_STEER_DEADBAND) {
        ESP_ERROR_CHECK(car_control_forward(FORWARD_SPEED_MPS));
        return TRACK_FORWARD;
    }

    if (fabsf(omega) <= INNER_TURN_SPEED_RAD_S) {
        ESP_ERROR_CHECK(car_control_set_velocity(
            0.0f, FORWARD_SPEED_MPS, omega));
        return omega > 0.0f ? TRACK_TRIM_LEFT : TRACK_TRIM_RIGHT;
    }

    ESP_ERROR_CHECK(car_control_set_velocity(
        0.0f, ARC_FORWARD_SPEED_MPS, omega));
    return omega > 0.0f ? TRACK_CORRECT_LEFT : TRACK_CORRECT_RIGHT;
}

static tracking_state_t tracking_update(const infrared_data_t *ir,
                                        tracking_context_t *context)
{
    const int64_t now_us = esp_timer_get_time();

    /* Once the finish marker has been entered, keep driving straight across
     * its black area and stop only after all four channels become white. */
    if (context->state == TRACK_FINISH_FORWARD) {
        if (ir->raw_black_pattern == 0x00U) {
            ESP_ERROR_CHECK(car_control_stop());
            context->state = TRACK_FINISHED;
        } else {
            ESP_ERROR_CHECK(car_control_forward(FORWARD_SPEED_MPS));
        }
        return context->state;
    }

    /* Ignore an all-black area before the first completed avoidance.  After
     * avoidance, all-black starts the straight finish-marker crossing. */
    if (context->finish_detection_enabled &&
        !state_is_searching(context->state) &&
        ir->raw_black_pattern == 0x0FU) {
        ESP_ERROR_CHECK(car_control_forward(FORWARD_SPEED_MPS));
        context->state = TRACK_FINISH_FORWARD;
        ESP_LOGI(TAG, "finish marker entered; drive forward until all channels are white");
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
           tracking->state != TRACK_FINISH_FORWARD &&
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
        /* The line is considered re-acquired once it is visible inside the
         * trimmed ROI and close to the ROI centre, instead of relying on a
         * single outer channel that can be missed or reversed. */
        const bool line_near_centre = ir->line_pixels != 0 &&
                                      fabsf(ir->raw_error) < LINE_RETURN_CENTER_ERROR;
        if (line_near_centre) {
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
        tracking->recovery_guard_until_us =
            esp_timer_get_time() + OBSTACLE_RECOVERY_GUARD_MS * 1000LL;
        tracking->finish_detection_enabled = true;
        ESP_LOGI(TAG, "avoidance complete; line reacquired with pattern=0x%X",
                 ir->pattern);
        ESP_LOGI(TAG, "finish detection enabled: all four channels at 1 will stop the car");
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
    ESP_ERROR_CHECK(tft_display_init());
    ESP_ERROR_CHECK(infrared_init());
    ESP_ERROR_CHECK(ultrasonic_sensor_init(&ultrasonic_config));
    ESP_ERROR_CHECK(car_control_stop());

    tracking_context_t tracking = {
        .state = TRACK_WAIT_START,
        .last_line_direction = DIRECTION_NONE,
    };
    avoidance_context_t avoidance = {
        .state = AVOID_IDLE,
    };
    int64_t last_log_us = 0;
    tracking_state_t previous_state = TRACK_WAIT_START;
    uint8_t previous_pattern = UINT8_MAX;

    ESP_LOGI(TAG, "course tracker started: camera image CH4, CH3, CH2, CH1 "
                  "maps to virtual OUT4, OUT3, OUT2, OUT1");
    ESP_LOGI(TAG,
             "black level=%d, forward=%.2f m/s, inner trim=%.2f rad/s, "
             "arc=%.2f m/s at %.2f rad/s, search=%.2f rad/s",
             IR_BLACK_LEVEL, FORWARD_SPEED_MPS, INNER_TURN_SPEED_RAD_S,
             ARC_FORWARD_SPEED_MPS, ARC_TURN_SPEED_RAD_S,
             SEARCH_SPEED_RAD_S);
    ESP_LOGI(TAG,
             "continuous centroid steering: line error -> omega, first "
             "sustained loss forces a right corner; all-black finish "
             "detection starts after avoidance, then drives forward until "
             "all-white");
    ESP_LOGI(TAG,
             "obstacle sensor: TRIG=GPIO%d ECHO=GPIO%d, trigger=%.1f cm "
             "for %d samples; left until %.1f cm for %d samples, "
             "forward %.0f ms, right until line returns near centre",
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
            ESP_LOGI(TAG, "finish marker crossed: all four sensors are "
                          "white; motors stopped");
            break;
        }

        vTaskDelay(pdMS_TO_TICKS(CONTROL_PERIOD_MS));
    }
}

