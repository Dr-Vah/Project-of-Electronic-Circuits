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

#define STAGING_STRAFE_GAIN 0.075f
/* BallDet is more selective but takes longer per frame. Keep chassis motion
 * conservative so one detection interval cannot create a large overshoot. */
#define BALL_TURN_GAIN 0.42f
#define ALIGN_TURN_GAIN 0.30f
#define MAX_STAGING_STRAFE_MPS 0.040f
#define MAX_SEARCH_TURN_RAD_S 0.22f
#define MAX_ALIGN_TURN_RAD_S 0.16f
#define APPROACH_SPEED_MPS 0.030f
#define PUSH_SPEED_MPS 0.040f
#define FINAL_PUSH_SPEED_MPS 0.025f
#define BACK_AWAY_SPEED_MPS 0.030f

#define BALL_TARGET_ALIGNMENT_TOLERANCE 0.10f
#define PUSH_AXIS_TOLERANCE 0.080f
#define PUSH_REALIGN_THRESHOLD 0.14f
#define BALL_CAPTURE_Y 0.78f
#define BALL_CAPTURE_DIAMETER_PX 22.0f
#define BALL_NEAR_CAPTURE_Y 0.50f
#define BALL_NEAR_CAPTURE_DIAMETER_PX 18.0f
#define BLIND_CAPTURE_MS 500
#define BALL_TOO_CLOSE_WHILE_STAGING_Y 0.68f
#define TARGET_NEAR_BOTTOM_FRACTION 0.75f
#define FINAL_PUSH_MS 500
#define RELEASE_SETTLE_MS 300
#define BACK_AWAY_MS 900

static const char *TAG = "ball_transport";
static portMUX_TYPE s_vision_lock = portMUX_INITIALIZER_UNLOCKED;

typedef enum {
    TRANSPORT_WAIT_SCENE,
    TRANSPORT_STAGE_BEHIND_BALL,
    TRANSPORT_ALIGN_APPROACH,
    TRANSPORT_APPROACH_BALL,
    TRANSPORT_ALIGN_PUSH,
    TRANSPORT_STRAIGHT_PUSH,
    TRANSPORT_FINAL_PUSH,
    TRANSPORT_SETTLE,
    TRANSPORT_BACK_AWAY,
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
} transport_context_t;

static transport_vision_t s_vision;

static float clampf(float value, float low, float high)
{
    return value < low ? low : (value > high ? high : value);
}

static float chassis_center_band_error(float image_center_x)
{
    if (image_center_x < CHASSIS_CENTER_BAND_LEFT_X) {
        return image_center_x - CHASSIS_CENTER_BAND_LEFT_X;
    }
    if (image_center_x > CHASSIS_CENTER_BAND_RIGHT_X) {
        return image_center_x - CHASSIS_CENTER_BAND_RIGHT_X;
    }
    return 0.0f;
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
    case TRANSPORT_FINAL_PUSH: return "FINAL_PUSH";
    case TRANSPORT_SETTLE: return "SETTLE";
    case TRANSPORT_BACK_AWAY: return "BACK_AWAY";
    case TRANSPORT_FINISHED: return "FINISHED";
    case TRANSPORT_FAULT: return "FAULT";
    default: return "UNKNOWN";
    }
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
    const float separation =
        vision->target.center_x - vision->ball.center_x;
    const float ball_axis_error =
        chassis_center_band_error(vision->ball.center_x);
    const float target_axis_error =
        chassis_push_axis_error(vision->target.center_x);
    const float pair_center_x =
        0.5f * (vision->target.center_x + vision->ball.center_x);
    const float common_heading =
        chassis_push_axis_error(pair_center_x);

    switch (context->state) {
    case TRANSPORT_WAIT_SCENE:
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
        if (fabsf(separation) < BALL_TARGET_ALIGNMENT_TOLERANCE &&
            fabsf(ball_axis_error) < PUSH_AXIS_TOLERANCE &&
            vision->ball.center_y < BALL_TOO_CLOSE_WHILE_STAGING_Y) {
            /* The scene was already confirmed in WAIT_SCENE. Once ball and
             * target share the push axis, approach immediately instead of
             * stopping for two more slow detector frames. */
            enter_state(context, TRANSPORT_APPROACH_BALL);
            apply_velocity(0.0f, APPROACH_SPEED_MPS, 0.0f);
            break;
        }
        context->stable_frames = 0U;
        apply_velocity(
            clampf(-STAGING_STRAFE_GAIN * separation,
                   -MAX_STAGING_STRAFE_MPS, MAX_STAGING_STRAFE_MPS),
            vision->ball.center_y > BALL_TOO_CLOSE_WHILE_STAGING_Y
                ? -0.025f
                : 0.0f,
            clampf(-BALL_TURN_GAIN * ball_axis_error,
                   -MAX_SEARCH_TURN_RAD_S, MAX_SEARCH_TURN_RAD_S));
        break;

    case TRANSPORT_ALIGN_APPROACH:
        if (!staging_scene_is_valid(vision)) {
            stop_motors();
            enter_state(context, TRANSPORT_WAIT_SCENE);
            break;
        }
        if (fabsf(common_heading) < PUSH_AXIS_TOLERANCE &&
            fabsf(separation) < BALL_TARGET_ALIGNMENT_TOLERANCE) {
            enter_state(context, TRANSPORT_APPROACH_BALL);
            apply_velocity(0.0f, APPROACH_SPEED_MPS, 0.0f);
        } else {
            context->stable_frames = 0U;
            apply_velocity(0.0f, 0.0f,
                           clampf(-ALIGN_TURN_GAIN * common_heading,
                                  -MAX_ALIGN_TURN_RAD_S,
                                  MAX_ALIGN_TURN_RAD_S));
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
            apply_velocity(
                0.0f, APPROACH_SPEED_MPS,
                clampf(-0.35f * target_axis_error, -0.14f, 0.14f));
            if (now_us >= context->blind_capture_deadline_us) {
                if (fabsf(target_axis_error) < PUSH_AXIS_TOLERANCE) {
                    enter_state(context, TRANSPORT_STRAIGHT_PUSH);
                    apply_velocity(0.0f, PUSH_SPEED_MPS, 0.0f);
                } else {
                    enter_state(context, TRANSPORT_ALIGN_PUSH);
                    apply_velocity(
                        0.0f, 0.0f,
                        clampf(-ALIGN_TURN_GAIN * target_axis_error,
                               -0.15f, 0.15f));
                }
            }
            break;
        }
        if (!staging_scene_is_valid(vision)) {
            stop_motors();
            if (++context->lost_frames >= LOST_VISION_LIMIT) {
                enter_state(context, TRANSPORT_WAIT_SCENE);
            }
            break;
        }
        context->lost_frames = 0U;
        context->blind_capture_deadline_us = 0;
        if (vision->ball.center_y >= BALL_NEAR_CAPTURE_Y &&
            vision->ball.diameter_px >= BALL_NEAR_CAPTURE_DIAMETER_PX) {
            context->near_ball_seen = true;
        }
        if (fabsf(separation) > 2.0f * BALL_TARGET_ALIGNMENT_TOLERANCE) {
            stop_motors();
            enter_state(context, TRANSPORT_STAGE_BEHIND_BALL);
            break;
        }
        apply_velocity(
            clampf(-0.025f * separation, -0.018f, 0.018f),
            APPROACH_SPEED_MPS,
            clampf(-0.35f * common_heading, -0.14f, 0.14f));
        if (vision->ball.center_y >= BALL_CAPTURE_Y &&
            vision->ball.diameter_px >= BALL_CAPTURE_DIAMETER_PX) {
            if (fabsf(common_heading) < PUSH_AXIS_TOLERANCE) {
                enter_state(context, TRANSPORT_STRAIGHT_PUSH);
                apply_velocity(0.0f, PUSH_SPEED_MPS, 0.0f);
            } else {
                enter_state(context, TRANSPORT_ALIGN_PUSH);
                apply_velocity(
                    0.0f, 0.0f,
                    clampf(-ALIGN_TURN_GAIN * common_heading,
                           -0.15f, 0.15f));
            }
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
            stop_motors();
            if (++context->lost_frames >= LOST_VISION_LIMIT) {
                enter_state(context, TRANSPORT_FAULT);
            }
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
        const float target_bottom =
            (float)(vision->target.bottom + 1U) / (float)vision->height;
        if (target_bottom >= TARGET_NEAR_BOTTOM_FRACTION) {
            context->phase_deadline_us =
                now_us + FINAL_PUSH_MS * 1000LL;
            enter_state(context, TRANSPORT_FINAL_PUSH);
            break;
        }
        if (fabsf(target_axis_error) > PUSH_REALIGN_THRESHOLD) {
            stop_motors();
            enter_state(context, TRANSPORT_ALIGN_PUSH);
            break;
        }
        /* The deliberate zero lateral and angular commands make this the
         * straight portion of the run. Encoder PID holds each wheel speed. */
        apply_velocity(0.0f, PUSH_SPEED_MPS, 0.0f);
        break;
    }

    case TRANSPORT_FINAL_PUSH:
        if (ball_overlaps_target(vision)) {
            ESP_LOGI(TAG, "ball reached target; stop immediately");
            stop_motors();
            context->phase_deadline_us =
                now_us + RELEASE_SETTLE_MS * 1000LL;
            enter_state(context, TRANSPORT_SETTLE);
            break;
        }
        /* Near the goal the ball and then the patch leave the camera view.
         * Finish by a bounded low-speed push instead of waiting for a visual
         * overlap that the camera cannot observe below the chassis. */
        apply_velocity(0.0f, FINAL_PUSH_SPEED_MPS, 0.0f);
        break;

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
    transport_context_t context = {.state = TRANSPORT_WAIT_SCENE};
    while (true) {
        const int64_t now_us = esp_timer_get_time();
        transport_vision_t vision = {0};
        const bool has_vision = copy_latest_vision(&vision);

        if (!has_vision ||
            now_us - vision.timestamp_us > VISION_STALE_MS * 1000LL) {
            stop_motors();
            if (context.state != TRANSPORT_FINISHED &&
                context.state != TRANSPORT_FAULT) {
                enter_state(&context, TRANSPORT_WAIT_SCENE);
            }
        } else if (vision.sequence != context.last_sequence) {
            context.last_sequence = vision.sequence;
            update_on_new_frame(&context, &vision, now_us);
        }

        if (context.state == TRANSPORT_FINAL_PUSH &&
            now_us >= context.phase_deadline_us) {
            stop_motors();
            context.phase_deadline_us =
                now_us + RELEASE_SETTLE_MS * 1000LL;
            enter_state(&context, TRANSPORT_SETTLE);
        } else if (context.state == TRANSPORT_SETTLE &&
            now_us >= context.phase_deadline_us) {
            apply_velocity(0.0f, -BACK_AWAY_SPEED_MPS, 0.0f);
            context.phase_deadline_us = now_us + BACK_AWAY_MS * 1000LL;
            enter_state(&context, TRANSPORT_BACK_AWAY);
        } else if (context.state == TRANSPORT_BACK_AWAY &&
                   now_us >= context.phase_deadline_us) {
            stop_motors();
            enter_state(&context, TRANSPORT_FINISHED);
        } else if (context.state == TRANSPORT_FINISHED ||
                   context.state == TRANSPORT_FAULT) {
            stop_motors();
        }

        if (now_us >= context.next_log_us) {
            context.next_log_us = now_us + 1500000LL;
            ESP_LOGI(TAG,
                     "state=%s center_band=[%+.2f,%+.2f] push_axis=%+.2f "
                     "white=%d "
                     "ball=(%+.2f,%.2f) d=%.1f "
                     "black=%d target=(%+.2f,%.2f) area=%u",
                     state_name(context.state),
                     CHASSIS_CENTER_BAND_LEFT_X,
                     CHASSIS_CENTER_BAND_RIGHT_X,
                     CHASSIS_PUSH_AXIS_CENTER_X,
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
    car_control_config_t car_config = CAR_CONTROL_DEFAULT_CONFIG();
    ESP_RETURN_ON_ERROR(car_control_init(&car_config), TAG,
                        "initialize three-wheel chassis failed");
    if (xTaskCreatePinnedToCore(controller_task, "ball_transport", 4096,
                                NULL, 8, NULL,
                                tskNO_AFFINITY) != pdPASS) {
        car_control_stop();
        return ESP_ERR_NO_MEM;
    }
    ESP_LOGI(TAG, "controller ready; waiting for ball and black target");
    return ESP_OK;
}
