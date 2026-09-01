#include "course_controller.h"

#include <inttypes.h>
#include <math.h>
#include <stdbool.h>
#include <stdint.h>

#include "camera_uvc.h"
#include "car_control.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "ultrasonic_sensor.h"

#define CONTROL_PERIOD_MS 20
#define ULTRASONIC_PERIOD_MS 50
#define VISION_STALE_MS 200
#define LINE_START_VALID_FRAMES 3
#define LOST_LINE_HOLD_MS 250
#define VISION_LOG_PERIOD_MS 500
#define OBSTACLE_CLEAR_CM 150.0f
#define OBSTACLE_CLEAR_SAMPLES 3
#define STRAFE_LEFT_SPEED_MPS 0.08f
#define STRAFE_RIGHT_SPEED_MPS 0.08f
#define BYPASS_FORWARD_SPEED_MPS 0.09f
#define LEFT_EXTRA_MS 200
#define BYPASS_FORWARD_MS 1300
#define REACQUIRE_TIMEOUT_MS 8000
#define REACQUIRE_SAMPLES 3

static const char *TAG = "course";
static portMUX_TYPE s_sample_lock = portMUX_INITIALIZER_UNLOCKED;
static ultrasonic_sample_t s_latest_ultrasonic;
static bool s_has_ultrasonic;

typedef enum {
    COURSE_WAIT_LINE,
    COURSE_LINE_FOLLOW,
    COURSE_STRAFE_LEFT,
    COURSE_STRAFE_LEFT_EXTRA,
    COURSE_PASS_OBSTACLE,
    COURSE_STRAFE_RIGHT,
    COURSE_FINISHED,
    COURSE_FAULT,
} course_state_t;

typedef struct {
    course_state_t state;
    int64_t next_ultrasonic_us;
    int64_t phase_deadline_us;
    uint8_t clear_samples;
    uint8_t reacquire_samples;
    uint8_t line_start_samples;
    uint8_t lost_line_samples;
    uint32_t last_vision_sequence;
    int64_t last_valid_line_us;
    int64_t next_vision_log_us;
    float held_vx_mps;
    float held_vy_mps;
    float held_omega_rad_s;
    bool has_held_command;
    ultrasonic_sample_t ultrasonic;
} course_context_t;

static const char *state_name(course_state_t state)
{
    switch (state) {
    case COURSE_WAIT_LINE: return "WAIT_LINE";
    case COURSE_LINE_FOLLOW: return "LINE_FOLLOW";
    case COURSE_STRAFE_LEFT: return "STRAFE_LEFT";
    case COURSE_STRAFE_LEFT_EXTRA: return "STRAFE_LEFT_EXTRA";
    case COURSE_PASS_OBSTACLE: return "PASS_OBSTACLE";
    case COURSE_STRAFE_RIGHT: return "STRAFE_RIGHT";
    case COURSE_FINISHED: return "FINISHED";
    case COURSE_FAULT: return "FAULT";
    default: return "UNKNOWN";
    }
}

static void set_state(course_context_t *context, course_state_t state)
{
    if (context->state != state) {
        ESP_LOGI(TAG, "state: %s -> %s", state_name(context->state),
                 state_name(state));
        context->state = state;
    }
}

static void enter_fault(course_context_t *context, const char *reason)
{
    ESP_ERROR_CHECK_WITHOUT_ABORT(car_control_stop());
    set_state(context, COURSE_FAULT);
    ESP_LOGE(TAG, "motors stopped: %s", reason);
}

static void sample_ultrasonic_if_due(course_context_t *context,
                                     int64_t now_us)
{
    if (now_us < context->next_ultrasonic_us) {
        return;
    }
    ESP_ERROR_CHECK_WITHOUT_ABORT(
        ultrasonic_sensor_read(&context->ultrasonic));
    portENTER_CRITICAL(&s_sample_lock);
    s_latest_ultrasonic = context->ultrasonic;
    s_has_ultrasonic = true;
    portEXIT_CRITICAL(&s_sample_lock);
    context->next_ultrasonic_us =
        context->ultrasonic.timestamp_us + ULTRASONIC_PERIOD_MS * 1000LL;
}

bool course_controller_get_ultrasonic_sample(ultrasonic_sample_t *sample)
{
    if (sample == NULL) return false;
    portENTER_CRITICAL(&s_sample_lock);
    const bool available = s_has_ultrasonic;
    if (available) *sample = s_latest_ultrasonic;
    portEXIT_CRITICAL(&s_sample_lock);
    return available;
}

static bool line_is_reacquired(const camera_vision_sample_t *vision)
{
    return vision->line.virtual_ir_valid &&
           (vision->line.virtual_ir_pattern & 0x06U) != 0U;
}

static bool vision_is_fresh(const camera_vision_sample_t *vision,
                            bool has_vision, int64_t now_us)
{
    return has_vision &&
           now_us - vision->frame_timestamp_us <= VISION_STALE_MS * 1000LL;
}

static void log_vision_if_due(course_context_t *context,
                              const camera_vision_sample_t *vision,
                              int64_t now_us)
{
    if (now_us < context->next_vision_log_us) return;
    context->next_vision_log_us = now_us + VISION_LOG_PERIOD_MS * 1000LL;
    ESP_LOGI(TAG,
             "vision seq=%" PRIu32 " threshold=%u points=%u "
             "confidence=%.2f valid=%d corner=%d turning=%d midline=%d "
             "vir=0x%X/0x%X ir=[%u,%u,%u,%u] err=%.1f "
             "angle=%.1fdeg "
             "distance=%.2fm cmd=(%.3f, %.3f, %.3f)",
             vision->sequence, vision->line.threshold,
             vision->line.point_count, vision->line.confidence,
             vision->line.control_valid, vision->line.corner_detected,
             vision->line.corner_turning,
             vision->line.middle_lower_line_present,
             vision->line.virtual_ir_raw_pattern,
             vision->line.virtual_ir_pattern,
             vision->line.virtual_ir_black_percent[0],
             vision->line.virtual_ir_black_percent[1],
             vision->line.virtual_ir_black_percent[2],
             vision->line.virtual_ir_black_percent[3],
             vision->line.virtual_ir_error,
             vision->line.corner_angle_rad * 180.0f / (float)M_PI,
             vision->line.corner_distance_m,
             vision->line.command_vx_mps, vision->line.command_vy_mps,
             vision->line.command_omega_rad_s);
}

static void remember_and_apply_line_command(course_context_t *context,
                                            const camera_vision_sample_t *vision,
                                            int64_t now_us)
{
    context->held_vx_mps = vision->line.command_vx_mps;
    context->held_vy_mps = vision->line.command_vy_mps;
    context->held_omega_rad_s = vision->line.command_omega_rad_s;
    context->has_held_command = true;
    context->last_valid_line_us = now_us;
    context->lost_line_samples = 0;
    ESP_ERROR_CHECK_WITHOUT_ABORT(car_control_set_velocity(
        context->held_vx_mps, context->held_vy_mps,
        context->held_omega_rad_s));
}

static void course_update(course_context_t *context,
                          const camera_vision_sample_t *vision,
                          bool has_vision, int64_t now_us)
{
    sample_ultrasonic_if_due(context, now_us);

    switch (context->state) {
    case COURSE_WAIT_LINE: {
        ESP_ERROR_CHECK_WITHOUT_ABORT(car_control_stop());
        if (!vision_is_fresh(vision, has_vision, now_us)) {
            context->line_start_samples = 0;
            return;
        }
        if (vision->sequence == context->last_vision_sequence) return;

        context->last_vision_sequence = vision->sequence;
        log_vision_if_due(context, vision, now_us);
        if (vision->line.virtual_ir_valid) {
            if (context->line_start_samples < UINT8_MAX) {
                ++context->line_start_samples;
            }
        } else {
            context->line_start_samples = 0;
        }
        if (context->line_start_samples >= LINE_START_VALID_FRAMES) {
            remember_and_apply_line_command(context, vision, now_us);
            context->line_start_samples = 0;
            set_state(context, COURSE_LINE_FOLLOW);
        }
        return;
    }

    case COURSE_LINE_FOLLOW:
        /* The course specification requires the trigger to remain 5 cm. */
        if (context->ultrasonic.valid &&
            context->ultrasonic.distance_cm <= TASK2_OBSTACLE_TRIGGER_CM) {
            ESP_LOGW(TAG, "obstacle at %.1f cm; start bypass",
                     context->ultrasonic.distance_cm);
            ESP_ERROR_CHECK_WITHOUT_ABORT(car_control_stop());
            ESP_ERROR_CHECK_WITHOUT_ABORT(
                car_control_strafe_left(STRAFE_LEFT_SPEED_MPS));
            context->clear_samples = 0;
            set_state(context, COURSE_STRAFE_LEFT);
            return;
        }

        if (vision_is_fresh(vision, has_vision, now_us) &&
            vision->sequence != context->last_vision_sequence) {
            context->last_vision_sequence = vision->sequence;
            log_vision_if_due(context, vision, now_us);
            if (vision->line.virtual_ir_all_black) {
                ESP_ERROR_CHECK_WITHOUT_ABORT(car_control_stop());
                context->has_held_command = false;
                set_state(context, COURSE_FINISHED);
                ESP_LOGI(TAG,
                         "finish marker detected by four virtual IR channels");
                return;
            }
            if (vision->line.control_valid) {
                remember_and_apply_line_command(context, vision, now_us);
                return;
            }
            if (context->lost_line_samples < UINT8_MAX) {
                ++context->lost_line_samples;
            }
        }

        if (context->has_held_command &&
            now_us - context->last_valid_line_us <=
                LOST_LINE_HOLD_MS * 1000LL) {
            ESP_ERROR_CHECK_WITHOUT_ABORT(car_control_set_velocity(
                context->held_vx_mps, context->held_vy_mps,
                context->held_omega_rad_s));
            return;
        }

        ESP_LOGW(TAG, "line lost for %u frames; stopping",
                 context->lost_line_samples);
        ESP_ERROR_CHECK_WITHOUT_ABORT(car_control_stop());
        context->line_start_samples = 0;
        context->has_held_command = false;
        set_state(context, COURSE_WAIT_LINE);
        return;

    case COURSE_STRAFE_LEFT: {
        /* Once the sensor moves beyond the wall edge, either a far echo or
         * three consecutive no-echo samples means the forward path is clear. */
        const bool obstacle_cleared =
            (context->ultrasonic.valid &&
             context->ultrasonic.distance_cm >= OBSTACLE_CLEAR_CM) ||
            context->ultrasonic.status == ULTRASONIC_WAIT_RISE_TIMEOUT ||
            context->ultrasonic.status == ULTRASONIC_PULSE_TIMEOUT;
        if (obstacle_cleared) {
            if (context->clear_samples < UINT8_MAX) {
                ++context->clear_samples;
            }
        } else {
            context->clear_samples = 0;
        }
        if (context->clear_samples >= OBSTACLE_CLEAR_SAMPLES) {
            context->phase_deadline_us = now_us + LEFT_EXTRA_MS * 1000LL;
            set_state(context, COURSE_STRAFE_LEFT_EXTRA);
        }
        return;
    }

    case COURSE_STRAFE_LEFT_EXTRA:
        if (now_us >= context->phase_deadline_us) {
            ESP_ERROR_CHECK_WITHOUT_ABORT(
                car_control_forward(BYPASS_FORWARD_SPEED_MPS));
            context->phase_deadline_us =
                now_us + BYPASS_FORWARD_MS * 1000LL;
            set_state(context, COURSE_PASS_OBSTACLE);
        }
        return;

    case COURSE_PASS_OBSTACLE:
        if (now_us >= context->phase_deadline_us) {
            ESP_ERROR_CHECK_WITHOUT_ABORT(
                car_control_strafe_left(-STRAFE_RIGHT_SPEED_MPS));
            context->phase_deadline_us =
                now_us + REACQUIRE_TIMEOUT_MS * 1000LL;
            context->reacquire_samples = 0;
            context->last_vision_sequence =
                has_vision ? vision->sequence : 0;
            set_state(context, COURSE_STRAFE_RIGHT);
        }
        return;

    case COURSE_STRAFE_RIGHT:
        if (has_vision && vision->sequence != context->last_vision_sequence) {
            context->last_vision_sequence = vision->sequence;
            if (line_is_reacquired(vision)) {
                if (context->reacquire_samples < UINT8_MAX) {
                    ++context->reacquire_samples;
                }
            } else {
                context->reacquire_samples = 0;
            }
        }
        if (context->reacquire_samples >= REACQUIRE_SAMPLES) {
            ESP_ERROR_CHECK_WITHOUT_ABORT(car_control_stop());
            context->last_vision_sequence = 0;
            context->line_start_samples = 0;
            context->has_held_command = false;
            set_state(context, COURSE_LINE_FOLLOW);
            return;
        }
        if (now_us >= context->phase_deadline_us) {
            enter_fault(context, "camera did not reacquire the line");
        }
        return;

    case COURSE_FINISHED:
    case COURSE_FAULT:
        ESP_ERROR_CHECK_WITHOUT_ABORT(car_control_stop());
        return;
    }
}

static void course_task(void *argument)
{
    (void)argument;
    course_context_t context = {
        .state = COURSE_WAIT_LINE,
    };
    while (true) {
        camera_vision_sample_t vision = {0};
        const bool has_vision = camera_uvc_get_latest(&vision);
        course_update(&context, &vision, has_vision, esp_timer_get_time());
        vTaskDelay(pdMS_TO_TICKS(CONTROL_PERIOD_MS));
    }
}

esp_err_t course_controller_start(void)
{
    if (xTaskCreatePinnedToCore(course_task, "course_control", 4096, NULL,
                                8, NULL, tskNO_AFFINITY) != pdPASS) {
        return ESP_ERR_NO_MEM;
    }
    ESP_LOGI(TAG, "controller started; obstacle trigger fixed at %.1f cm",
             TASK2_OBSTACLE_TRIGGER_CM);
    return ESP_OK;
}
