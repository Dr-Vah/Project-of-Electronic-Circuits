#include "ball_transport_controller.h"

#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "car_control.h"
#include "chassis_camera_geometry.h"
#include "esp_check.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define CONTROL_PERIOD_MS 20
#define VISION_STALE_MS 2000
#define REQUIRED_STABLE_FRAMES 2U
#define LOST_VISION_LIMIT 5U

#define STAGING_STRAFE_GAIN 0.045f
/* BallDet is more selective but takes longer per frame. Keep chassis motion
 * conservative so one detection interval cannot create a large overshoot. */
#define ALIGN_TURN_GAIN 0.30f
#define MAX_STAGING_STRAFE_MPS 0.020f
#define NEAR_LINE_TURN_PULSE_RAD_S 0.35f
#define NEAR_LINE_TURN_PULSE_MS 55
#define LINE_BOTTOM_TOLERANCE 0.055f
#define LINE_BOTTOM_RECHECK_TOLERANCE 0.10f
#define APPROACH_SPEED_MPS 0.030f
#define PUSH_SPEED_MPS 0.045f
#define NEAR_TARGET_PUSH_SPEED_MPS 0.025f
#define BACK_AWAY_SPEED_MPS 0.030f
/* Keep rotation below the amount that can move a locked ball/target outside
 * its local tracking window between two camera results. */
#define ORANGE_SEARCH_TURN_RAD_S 0.30f

#define BALL_TARGET_ALIGNMENT_TOLERANCE 0.070f
#define PUSH_AXIS_TOLERANCE 0.055f
#define BALL_CAPTURE_Y 0.78f
#define BALL_CAPTURE_DIAMETER_PX 22.0f
#define BALL_NEAR_CAPTURE_Y 0.50f
#define BALL_NEAR_CAPTURE_DIAMETER_PX 18.0f
#define APPROACH_DROPOUT_GRACE_MS 1500
#define COMMITTED_DRIVE_TIMEOUT_MS 12000
#define BLIND_CAPTURE_MS 500
#define TARGET_VISIBLE_STOP_BOTTOM_FRACTION 0.84f
#define TARGET_SLOWDOWN_BOTTOM_FRACTION 0.60f
#define RELEASE_SETTLE_MS 300
/* With the encoder-controlled 0.03 m/s reverse command, 8.35 s is about
 * 0.25 m. Only the white-ball leg needs this clearance before turning
 * toward the orange ball; the final release remains short. */
#define WHITE_BACK_AWAY_MS 8350
#define FINAL_BACK_AWAY_MS 900
#define ORANGE_SEARCH_TIMEOUT_MS 8000

static const char *TAG = "ball_transport";
static portMUX_TYPE s_vision_lock = portMUX_INITIALIZER_UNLOCKED;

typedef enum {
    TRANSPORT_WAIT_SCENE,
    TRANSPORT_STAGE_BEHIND_BALL,
    TRANSPORT_ALIGN_APPROACH,
    TRANSPORT_APPROACH_BALL,
    TRANSPORT_ALIGN_PUSH,
    TRANSPORT_STRAIGHT_PUSH,
    TRANSPORT_SETTLE,
    TRANSPORT_BACK_AWAY,
    TRANSPORT_TURN_TO_ORANGE,
    TRANSPORT_FINISHED,
    TRANSPORT_FAULT,
} transport_state_t;

typedef struct {
    white_ball_result_t ball;
    black_target_result_t target;
    size_t width;
    size_t height;
    int64_t timestamp_us;
    uint32_t sequence;
    ball_color_t ball_color;
} transport_vision_t;

typedef struct {
    transport_state_t state;
    uint32_t last_sequence;
    uint8_t stable_frames;
    uint8_t lost_frames;
    int64_t phase_deadline_us;
    int64_t next_log_us;
    bool near_ball_seen;
    int64_t blind_capture_deadline_us;
    int64_t approach_dropout_deadline_us;
    int64_t turn_pulse_deadline_us;
    float last_visible_target_bottom;
    ball_color_t active_color;
} transport_context_t;

static transport_vision_t s_vision;
static volatile ball_color_t s_requested_color = BALL_COLOR_WHITE;

static float clampf(float value, float low, float high)
{
    return value < low ? low : (value > high ? high : value);
}

static float chassis_push_axis_error(float image_center_x)
{
    return image_center_x - CHASSIS_PUSH_AXIS_CENTER_X;
}

static const char *state_name(transport_state_t state)
{
    switch (state) {
    case TRANSPORT_WAIT_SCENE: return "WAIT_SCENE";
    case TRANSPORT_STAGE_BEHIND_BALL: return "STAGE_BEHIND_BALL";
    case TRANSPORT_ALIGN_APPROACH: return "ALIGN_APPROACH";
    case TRANSPORT_APPROACH_BALL: return "APPROACH_BALL";
    case TRANSPORT_ALIGN_PUSH: return "ALIGN_PUSH";
    case TRANSPORT_STRAIGHT_PUSH: return "STRAIGHT_PUSH";
    case TRANSPORT_SETTLE: return "SETTLE";
    case TRANSPORT_BACK_AWAY: return "BACK_AWAY";
    case TRANSPORT_TURN_TO_ORANGE: return "TURN_TO_ORANGE";
    case TRANSPORT_FINISHED: return "FINISHED";
    case TRANSPORT_FAULT: return "FAULT";
    default: return "UNKNOWN";
    }
}

ball_color_t ball_transport_controller_requested_color(void)
{
    return s_requested_color;
}

static void enter_state(transport_context_t *context,
                        transport_state_t state)
{
    if (context->state == state) return;
    ESP_LOGI(TAG, "state: %s -> %s", state_name(context->state),
             state_name(state));
    context->state = state;
    context->stable_frames = 0U;
    context->lost_frames = 0U;
    context->near_ball_seen = false;
    context->blind_capture_deadline_us = 0;
    context->approach_dropout_deadline_us = 0;
    context->turn_pulse_deadline_us = 0;
}

static bool copy_latest_vision(transport_vision_t *vision)
{
    portENTER_CRITICAL(&s_vision_lock);
    *vision = s_vision;
    portEXIT_CRITICAL(&s_vision_lock);
    return vision->sequence != 0U;
}

void ball_transport_controller_submit(const white_ball_result_t *ball,
                                      const black_target_result_t *target,
                                      ball_color_t ball_color,
                                      size_t frame_width,
                                      size_t frame_height,
                                      int64_t timestamp_us)
{
    if (ball == NULL || target == NULL || frame_width == 0U ||
        frame_height == 0U) {
        return;
    }
    portENTER_CRITICAL(&s_vision_lock);
    s_vision.ball = *ball;
    s_vision.target = *target;
    s_vision.ball_color = ball_color;
    s_vision.width = frame_width;
    s_vision.height = frame_height;
    s_vision.timestamp_us = timestamp_us;
    ++s_vision.sequence;
    if (s_vision.sequence == 0U) s_vision.sequence = 1U;
    portEXIT_CRITICAL(&s_vision_lock);
}

static bool target_and_ball_visible(const transport_vision_t *vision)
{
    return vision->ball.valid && vision->target.valid;
}

static bool staging_scene_is_valid(const transport_vision_t *vision)
{
    /* Before touching the ball, the target must appear farther away (higher
     * in the floor image) than the ball. This prevents driving from the target
     * side and pushing the ball away from the stopping zone. */
    return target_and_ball_visible(vision) &&
           vision->target.center_y + 0.04f < vision->ball.center_y;
}

static bool ball_overlaps_target(const transport_vision_t *vision)
{
    const white_ball_result_t *ball = &vision->ball;
    const black_target_result_t *target = &vision->target;
    if (!ball->valid || !target->valid) return false;

    const bool horizontal_overlap =
        ball->right >= target->left && ball->left <= target->right;
    const bool vertical_overlap =
        ball->bottom >= target->top && ball->top <= target->bottom;
    return horizontal_overlap && vertical_overlap &&
           ball->center_y >= 0.60f;
}

static void apply_velocity(float vx, float vy, float omega)
{
    ESP_ERROR_CHECK_WITHOUT_ABORT(car_control_set_velocity(vx, vy, omega));
}

static void stop_motors(void)
{
    ESP_ERROR_CHECK_WITHOUT_ABORT(car_control_stop());
}

static void update_on_new_frame(transport_context_t *context,
                                const transport_vision_t *vision,
                                int64_t now_us)
{
    if (vision->target.valid && vision->height > 0U) {
        context->last_visible_target_bottom =
            (float)(vision->target.bottom + 1U) / (float)vision->height;
    }
    const float separation =
        vision->target.center_x - vision->ball.center_x;
    const float target_axis_error =
        chassis_push_axis_error(vision->target.center_x);
    const float pair_center_x =
        0.5f * (vision->target.center_x + vision->ball.center_x);
    const float common_heading =
        chassis_push_axis_error(pair_center_x);
    const float ball_target_dy =
        vision->ball.center_y - vision->target.center_y;
    /* Extrapolate the ball-target line to the bottom of the image, where the
     * red chassis forward axis starts. Translation first makes this intercept
     * coincide with the red-line foot; rotation is handled in the next state. */
    const float line_bottom_x = ball_target_dy > 0.001f
        ? vision->ball.center_x +
              (1.0f - vision->ball.center_y) *
              (vision->ball.center_x - vision->target.center_x) /
              ball_target_dy
        : vision->ball.center_x;
    const float line_bottom_error =
        line_bottom_x - CHASSIS_PUSH_AXIS_CENTER_X;

    switch (context->state) {
    case TRANSPORT_TURN_TO_ORANGE:
        if (vision->ball_color != BALL_COLOR_ORANGE) {
            apply_velocity(0.0f, 0.0f, ORANGE_SEARCH_TURN_RAD_S);
            break;
        }
        if (vision->ball.valid) {
            /* Brake on the first valid frame and finish the controller-level
             * confirmation while stationary. Continuing to turn for another
             * vision frame used to move the newly found ball too far away. */
            stop_motors();
            if (++context->stable_frames >= REQUIRED_STABLE_FRAMES) {
                ESP_LOGI(TAG,
                         "orange ball acquired and confirmed while stopped");
                enter_state(context, TRANSPORT_WAIT_SCENE);
            }
        } else {
            context->stable_frames = 0U;
            apply_velocity(0.0f, 0.0f, ORANGE_SEARCH_TURN_RAD_S);
        }
        break;

    case TRANSPORT_WAIT_SCENE:
        if (vision->ball_color != context->active_color) {
            stop_motors();
            break;
        }
        stop_motors();
        if (staging_scene_is_valid(vision)) {
            if (++context->stable_frames >= REQUIRED_STABLE_FRAMES) {
                enter_state(context, TRANSPORT_STAGE_BEHIND_BALL);
            }
        } else {
            context->stable_frames = 0U;
        }
        break;

    case TRANSPORT_STAGE_BEHIND_BALL:
        if (!staging_scene_is_valid(vision)) {
            stop_motors();
            if (++context->lost_frames >= LOST_VISION_LIMIT) {
                enter_state(context, TRANSPORT_WAIT_SCENE);
            }
            break;
        }
        context->lost_frames = 0U;
        if (fabsf(line_bottom_error) < LINE_BOTTOM_TOLERANCE) {
            /* Phase 1: the chassis centre, ball and target are collinear.
             * Confirm while stopped, then proceed to rotation-only phase 2. */
            stop_motors();
            if (++context->stable_frames >= REQUIRED_STABLE_FRAMES) {
                enter_state(context, TRANSPORT_ALIGN_APPROACH);
            }
            break;
        }
        context->stable_frames = 0U;
        /* Low-speed continuous translation: no forward motion or rotation,
         * and no repeated kick/stop cycle. */
        apply_velocity(
            clampf(STAGING_STRAFE_GAIN * line_bottom_error,
                   -MAX_STAGING_STRAFE_MPS, MAX_STAGING_STRAFE_MPS),
            0.0f, 0.0f);
        break;

    case TRANSPORT_ALIGN_APPROACH:
        if (!staging_scene_is_valid(vision)) {
            stop_motors();
            enter_state(context, TRANSPORT_WAIT_SCENE);
            break;
        }
        if (fabsf(line_bottom_error) > LINE_BOTTOM_RECHECK_TOLERANCE) {
            /* Rotation should preserve the line through the chassis centre.
             * If perspective/slip moves it away, redo translation before
             * issuing any more turn commands. */
            stop_motors();
            enter_state(context, TRANSPORT_STAGE_BEHIND_BALL);
            break;
        }
        if (fabsf(common_heading) < PUSH_AXIS_TOLERANCE &&
            fabsf(separation) < BALL_TARGET_ALIGNMENT_TOLERANCE) {
            stop_motors();
            if (++context->stable_frames >= REQUIRED_STABLE_FRAMES) {
                enter_state(context, TRANSPORT_APPROACH_BALL);
                context->phase_deadline_us =
                    now_us + COMMITTED_DRIVE_TIMEOUT_MS * 1000LL;
                apply_velocity(0.0f, APPROACH_SPEED_MPS, 0.0f);
            }
        } else {
            context->stable_frames = 0U;
            const float turn_error = fabsf(common_heading) > 0.01f
                ? common_heading : separation;
            const float turn_omega =
                -copysignf(NEAR_LINE_TURN_PULSE_RAD_S, turn_error);
            if (context->turn_pulse_deadline_us == 0) {
                context->turn_pulse_deadline_us =
                    now_us + NEAR_LINE_TURN_PULSE_MS * 1000LL;
            }
            /* Rotation phase: no lateral or forward translation. */
            apply_velocity(0.0f, 0.0f, turn_omega);
        }
        break;

    case TRANSPORT_APPROACH_BALL:
        if (!vision->ball.valid && context->near_ball_seen &&
            vision->target.valid) {
            /* The close ball can disappear into the chassis shadow before
             * reaching the normal visual capture threshold. Continue the
             * already aligned approach for a short bounded distance, then
             * hand over to target-only push alignment. */
            if (context->blind_capture_deadline_us == 0) {
                context->blind_capture_deadline_us =
                    now_us + BLIND_CAPTURE_MS * 1000LL;
                ESP_LOGI(TAG,
                         "near ball hidden; continue capture for %d ms",
                         BLIND_CAPTURE_MS);
            }
            apply_velocity(0.0f, APPROACH_SPEED_MPS, 0.0f);
            if (now_us >= context->blind_capture_deadline_us) {
                enter_state(context, TRANSPORT_STRAIGHT_PUSH);
                apply_velocity(0.0f, PUSH_SPEED_MPS, 0.0f);
            }
            break;
        }
        if (!staging_scene_is_valid(vision)) {
            /* Three-point alignment is a commitment point. Once it has been
             * confirmed, ordinary detection dropouts must not stop or turn
             * the chassis in front of the ball. Keep driving straight and
             * hand over to the straight-push state after a bounded interval. */
            if (context->approach_dropout_deadline_us == 0) {
                context->approach_dropout_deadline_us =
                    now_us + APPROACH_DROPOUT_GRACE_MS * 1000LL;
                ESP_LOGW(TAG,
                         "approach detection dropout; stay committed for %d ms",
                         APPROACH_DROPOUT_GRACE_MS);
            }
            apply_velocity(0.0f, APPROACH_SPEED_MPS, 0.0f);
            if (now_us >= context->approach_dropout_deadline_us) {
                enter_state(context, TRANSPORT_STRAIGHT_PUSH);
                apply_velocity(0.0f, PUSH_SPEED_MPS, 0.0f);
            }
            break;
        }
        context->lost_frames = 0U;
        context->approach_dropout_deadline_us = 0;
        context->blind_capture_deadline_us = 0;
        if (vision->ball.center_y >= BALL_NEAR_CAPTURE_Y &&
            vision->ball.diameter_px >= BALL_NEAR_CAPTURE_DIAMETER_PX) {
            context->near_ball_seen = true;
        }
        apply_velocity(0.0f, APPROACH_SPEED_MPS, 0.0f);
        if (vision->ball.center_y >= BALL_CAPTURE_Y &&
            vision->ball.diameter_px >= BALL_CAPTURE_DIAMETER_PX) {
            enter_state(context, TRANSPORT_STRAIGHT_PUSH);
            apply_velocity(0.0f, PUSH_SPEED_MPS, 0.0f);
        }
        break;

    case TRANSPORT_ALIGN_PUSH:
        if (!vision->target.valid) {
            stop_motors();
            if (++context->lost_frames >= LOST_VISION_LIMIT) {
                enter_state(context, TRANSPORT_FAULT);
            }
            break;
        }
        context->lost_frames = 0U;
        /* The ball may already be under the chassis. In that case align on
         * the still-visible stopping patch instead of treating it as a lost
         * scene. */
        const float push_heading = vision->ball.valid
                                       ? common_heading
                                       : target_axis_error;
        if (fabsf(push_heading) < PUSH_AXIS_TOLERANCE) {
            /* Alignment is actionable immediately: enter the push state and
             * issue its velocity in this same control update. */
            enter_state(context, TRANSPORT_STRAIGHT_PUSH);
            context->phase_deadline_us =
                now_us + COMMITTED_DRIVE_TIMEOUT_MS * 1000LL;
            apply_velocity(0.0f, PUSH_SPEED_MPS, 0.0f);
        } else {
            context->stable_frames = 0U;
            apply_velocity(0.0f, 0.0f,
                           clampf(-ALIGN_TURN_GAIN * push_heading,
                                  -0.15f, 0.15f));
        }
        break;

    case TRANSPORT_STRAIGHT_PUSH: {
        if (!vision->target.valid) {
            /* Three-point alignment is the commitment point. A normal target
             * segmentation dropout must not interrupt or steer the push.
             * The global stale-frame and committed-drive timeouts still stop
             * the chassis if the camera actually fails or no endpoint is
             * observed within the bounded run. */
            if (++context->lost_frames == 1U) {
                ESP_LOGW(TAG,
                         "target temporarily lost; continue committed push");
            }
            apply_velocity(
                0.0f,
                context->last_visible_target_bottom >=
                        TARGET_SLOWDOWN_BOTTOM_FRACTION
                    ? NEAR_TARGET_PUSH_SPEED_MPS : PUSH_SPEED_MPS,
                0.0f);
            break;
        }
        context->lost_frames = 0U;
        if (ball_overlaps_target(vision)) {
            ESP_LOGI(TAG, "ball reached target; stop immediately");
            stop_motors();
            context->phase_deadline_us =
                now_us + RELEASE_SETTLE_MS * 1000LL;
            enter_state(context, TRANSPORT_SETTLE);
            break;
        }
        const float target_bottom = context->last_visible_target_bottom;
        if (target_bottom >= TARGET_VISIBLE_STOP_BOTTOM_FRACTION) {
            ESP_LOGI(TAG,
                     "target visible at frame bottom; stop push immediately");
            stop_motors();
            context->phase_deadline_us =
                now_us + RELEASE_SETTLE_MS * 1000LL;
            enter_state(context, TRANSPORT_SETTLE);
            break;
        }
        /* The deliberate zero lateral and angular commands make this the
         * straight portion of the run. Encoder PID holds each wheel speed. */
        apply_velocity(
            0.0f,
            target_bottom >= TARGET_SLOWDOWN_BOTTOM_FRACTION
                ? NEAR_TARGET_PUSH_SPEED_MPS : PUSH_SPEED_MPS,
            0.0f);
        break;
    }

    case TRANSPORT_SETTLE:
    case TRANSPORT_BACK_AWAY:
    case TRANSPORT_FINISHED:
    case TRANSPORT_FAULT:
        break;
    }
}

static void controller_task(void *argument)
{
    (void)argument;
    transport_context_t context = {
        .state = TRANSPORT_WAIT_SCENE,
        .active_color = BALL_COLOR_WHITE,
    };
    while (true) {
        const int64_t now_us = esp_timer_get_time();
        transport_vision_t vision = {0};
        const bool has_vision = copy_latest_vision(&vision);

        if (!has_vision ||
            now_us - vision.timestamp_us > VISION_STALE_MS * 1000LL) {
            stop_motors();
            if (context.state != TRANSPORT_FINISHED &&
                context.state != TRANSPORT_TURN_TO_ORANGE &&
                context.state != TRANSPORT_FAULT) {
                enter_state(&context, TRANSPORT_WAIT_SCENE);
            }
        } else if (vision.sequence != context.last_sequence) {
            context.last_sequence = vision.sequence;
            update_on_new_frame(&context, &vision, now_us);
        }

        if (context.state == TRANSPORT_ALIGN_APPROACH &&
            context.turn_pulse_deadline_us != 0 &&
            now_us >= context.turn_pulse_deadline_us) {
            stop_motors();
            context.turn_pulse_deadline_us = 0;
        }

        /* Complete the dropout hand-off from the fast control loop so a slow
         * next camera frame cannot create a pause between approach and push. */
        if (context.state == TRANSPORT_APPROACH_BALL &&
            context.approach_dropout_deadline_us != 0 &&
            now_us >= context.approach_dropout_deadline_us) {
            enter_state(&context, TRANSPORT_STRAIGHT_PUSH);
            apply_velocity(0.0f, PUSH_SPEED_MPS, 0.0f);
        }

        if ((context.state == TRANSPORT_APPROACH_BALL ||
             context.state == TRANSPORT_STRAIGHT_PUSH ||
             context.state == TRANSPORT_ALIGN_PUSH) &&
            context.phase_deadline_us != 0 &&
            now_us >= context.phase_deadline_us) {
            ESP_LOGE(TAG, "committed drive exceeded %d ms safety limit",
                     COMMITTED_DRIVE_TIMEOUT_MS);
            stop_motors();
            enter_state(&context, TRANSPORT_FAULT);
        }

        if (context.state == TRANSPORT_SETTLE &&
            now_us >= context.phase_deadline_us) {
            apply_velocity(0.0f, -BACK_AWAY_SPEED_MPS, 0.0f);
            const int back_away_ms =
                context.active_color == BALL_COLOR_WHITE
                    ? WHITE_BACK_AWAY_MS : FINAL_BACK_AWAY_MS;
            context.phase_deadline_us = now_us + back_away_ms * 1000LL;
            ESP_LOGI(TAG, "back away for %d ms (%s leg)", back_away_ms,
                     context.active_color == BALL_COLOR_WHITE
                         ? "about 0.25 m" : "final release");
            enter_state(&context, TRANSPORT_BACK_AWAY);
        } else if (context.state == TRANSPORT_BACK_AWAY &&
                   now_us >= context.phase_deadline_us) {
            stop_motors();
            if (context.active_color == BALL_COLOR_WHITE) {
                context.active_color = BALL_COLOR_ORANGE;
                s_requested_color = BALL_COLOR_ORANGE;
                context.phase_deadline_us =
                    now_us + ORANGE_SEARCH_TIMEOUT_MS * 1000LL;
                enter_state(&context, TRANSPORT_TURN_TO_ORANGE);
                apply_velocity(0.0f, 0.0f, ORANGE_SEARCH_TURN_RAD_S);
            } else {
                enter_state(&context, TRANSPORT_FINISHED);
            }
        } else if (context.state == TRANSPORT_TURN_TO_ORANGE &&
                   now_us >= context.phase_deadline_us) {
            ESP_LOGE(TAG, "orange-ball search timed out");
            stop_motors();
            enter_state(&context, TRANSPORT_FAULT);
        } else if (context.state == TRANSPORT_FINISHED ||
                   context.state == TRANSPORT_FAULT) {
            stop_motors();
        }

        if (now_us >= context.next_log_us) {
            context.next_log_us = now_us + 1500000LL;
            ESP_LOGI(TAG,
                     "state=%s center_band=[%+.2f,%+.2f] push_axis=%+.2f "
                     "color=%s ball=%d "
                     "ball=(%+.2f,%.2f) d=%.1f "
                     "black=%d target=(%+.2f,%.2f) area=%u",
                     state_name(context.state),
                     CHASSIS_CENTER_BAND_LEFT_X,
                     CHASSIS_CENTER_BAND_RIGHT_X,
                     CHASSIS_PUSH_AXIS_CENTER_X,
                     context.active_color == BALL_COLOR_WHITE
                         ? "white" : "orange",
                     vision.ball.valid,
                     vision.ball.center_x, vision.ball.center_y,
                     vision.ball.diameter_px, vision.target.valid,
                     vision.target.center_x, vision.target.center_y,
                     vision.target.area_px);
        }
        vTaskDelay(pdMS_TO_TICKS(CONTROL_PERIOD_MS));
    }
}

esp_err_t ball_transport_controller_start(void)
{
    /* The line-course stage already initialized the shared chassis driver. */
    if (xTaskCreatePinnedToCore(controller_task, "ball_transport", 4096,
                                NULL, 8, NULL,
                                tskNO_AFFINITY) != pdPASS) {
        car_control_stop();
        return ESP_ERR_NO_MEM;
    }
    ESP_LOGI(TAG, "controller ready; waiting for ball and black target");
    return ESP_OK;
}
