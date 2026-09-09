/* Compile the real detector, replacing only ESP allocation/task primitives. */
#include "../../main/ball_target_detector.c"
#include "../../main/ball_transport_controller.c"

static int stop_calls, velocity_calls;
static float last_vy, last_omega;
esp_err_t car_control_stop(void) { ++stop_calls; return ESP_OK; }
esp_err_t car_control_set_velocity(float vx, float vy, float omega)
{
    (void)vx;
    last_vy = vy; last_omega = omega;
    ++velocity_calls;
    return ESP_OK;
}

#define W 320
#define H 160
#define CHECK(condition) do { if (!(condition)) { \
    fprintf(stderr, "FAIL line %d: %s\n", __LINE__, #condition); return 1; \
} } while (0)

static uint16_t pixels[W * H];

static void clear_frame(void)
{
    for (int i = 0; i < W * H; ++i) pixels[i] = 0xffff;
}

static void black_box(int x, int y, int width, int height)
{
    for (int row = y; row < y + height; ++row)
        for (int col = x; col < x + width; ++col)
            pixels[row * W + col] = 0;
}

static int independent_detection_and_mask(void)
{
    bt_detector_t d;
    bt_detector_init(&d);
    bt_detector_result_t r;
    clear_frame();
    black_box(60, 100, 20, 10);
    CHECK(bt_detector_process_rgb565(&d, pixels, W, H, &r) == ESP_OK);
    CHECK(!r.ball_valid && r.target_valid);
    CHECK(!d.white_left_mask_active);

    /* A second block appears after single-target tracking has begun. */
    black_box(260, 100, 20, 10);
    CHECK(bt_detector_process_rgb565(&d, pixels, W, H, &r) == ESP_OK);
    CHECK(d.white_left_mask_active && r.target_valid && r.target_count == 1);
    CHECK(r.target.x < W * 70 / 100);

    clear_frame();
    black_box(260, 100, 20, 10);
    CHECK(bt_detector_process_rgb565(&d, pixels, W, H, &r) == ESP_OK);
    CHECK(d.white_left_mask_active && !r.target_valid);

    d.white_delivery_complete = true;
    CHECK(bt_detector_process_rgb565(&d, pixels, W, H, &r) == ESP_OK);
    CHECK(!d.white_left_mask_active && d.orange_phase_active && r.target_valid);
    CHECK(r.target.x > W * 70 / 100);
    clear_frame();
    black_box(60, 100, 20, 10);
    CHECK(bt_detector_process_rgb565(&d, pixels, W, H, &r) == ESP_OK);
    CHECK(r.target_valid);
    /* Orange now restores the full horizontal ROI, including after reset. */
    bt_detector_init(&d);
    d.white_delivery_complete = true;
    CHECK(bt_detector_process_rgb565(&d, pixels, W, H, &r) == ESP_OK);
    CHECK(d.orange_phase_active && r.target_valid);
    return 0;
}

static int approaching_target(void)
{
    bt_detector_t d;
    bt_detector_init(&d);
    d.white_left_mask_active = true;
    bt_detector_result_t r;
    const int boxes[][4] = {
        {90, 90, 24, 10}, {80, 70, 44, 18},
        {65, 55, 70, 30}, {50, 40, 100, 44}
    };
    for (unsigned i = 0; i < sizeof(boxes) / sizeof(boxes[0]); ++i) {
        clear_frame();
        black_box(boxes[i][0], boxes[i][1], boxes[i][2], boxes[i][3]);
        CHECK(bt_detector_process_rgb565(&d, pixels, W, H, &r) == ESP_OK);
        CHECK(r.target_valid && !r.ball_valid);
        /* Stopping geometry must use this frame, with no smoothing delay. */
        CHECK(r.target.y0 == boxes[i][1]);
        const float screen_bottom = (float)(H - r.target.y0) / H;
        if (i >= 2) CHECK(screen_bottom >= 0.60f);
    }
    clear_frame();
    CHECK(bt_detector_process_rgb565(&d, pixels, W, H, &r) == ESP_OK);
    CHECK(!r.target_valid && d.white_left_mask_active);
    return 0;
}

static int reject_floor_and_line(void)
{
    bt_detector_t d;
    bt_detector_init(&d);
    bt_detector_result_t r;
    clear_frame();
    black_box(0, 0, W, H);
    CHECK(bt_detector_process_rgb565(&d, pixels, W, H, &r) == ESP_OK);
    CHECK(!r.target_valid);
    clear_frame();
    black_box(60, 80, 80, 2);
    CHECK(bt_detector_process_rgb565(&d, pixels, W, H, &r) == ESP_OK);
    CHECK(!r.target_valid);
    return 0;
}

static int camera_resolutions(void)
{
    const int sizes[][2] = {{160, 120}, {240, 160}};
    for (unsigned i = 0; i < sizeof(sizes) / sizeof(sizes[0]); ++i) {
        const int w = sizes[i][0], h = sizes[i][1];
        const int x0 = w / 8, y0 = h / 4;
        const int box_w = w / 2, box_h = h / 4;
        for (int j = 0; j < w * h; ++j) pixels[j] = 0xffff;
        for (int y = y0; y < y0 + box_h; ++y)
            for (int x = x0; x < x0 + box_w; ++x) pixels[y * w + x] = 0;
        bt_detector_t d;
        bt_detector_init(&d);
        d.white_left_mask_active = true;
        bt_detector_result_t r;
        CHECK(bt_detector_process_rgb565(&d, pixels, w, h, &r) == ESP_OK);
        CHECK(r.target_valid && !r.ball_valid);
        CHECK(r.target.x1 - r.target.x0 == box_w);
        CHECK(r.target.y1 - r.target.y0 == box_h);
        CHECK((float)(h - r.target.y0) / h >= 0.60f);
    }
    return 0;
}

static int independent_parking(void)
{
    bt_detector_t d;
    bt_detector_init(&d);
    bt_detector_result_t r;
    clear_frame();
    black_box(0, 0, W, H);
    CHECK(bt_detector_process_rgb565(&d, pixels, W, H, &r) == ESP_OK);
    CHECK(!r.target_valid && !r.ball_valid && r.black_ahead);
    CHECK(r.black_ahead_percent == 100);
    white_ball_result_t ball = {0};
    black_target_result_t target = {0};
    const transport_state_t states[] = {
        TRANSPORT_APPROACH_BALL, TRANSPORT_ALIGN_PUSH, TRANSPORT_STRAIGHT_PUSH
    };
    for (int color = BALL_COLOR_WHITE; color <= BALL_COLOR_ORANGE; ++color) {
        ball_transport_controller_submit(&ball, &target, color, W, H, 1000,
                                         r.black_ahead);
        transport_vision_t vision;
        CHECK(copy_latest_vision(&vision));
        for (unsigned i = 0; i < sizeof(states) / sizeof(states[0]); ++i) {
            transport_context_t context = {.state = states[i], .active_color = color};
            stop_calls = velocity_calls = 0;
            update_on_new_frame(&context, &vision, 1000);
            if (color == BALL_COLOR_WHITE) {
                CHECK(context.state == TRANSPORT_WHITE_EXTRA_PUSH);
                CHECK(stop_calls == 0 && velocity_calls == 1);
                CHECK(context.phase_deadline_us == 1000 + WHITE_EXTRA_PUSH_MS * 1000LL);
            } else {
                CHECK(context.state == TRANSPORT_SETTLE);
                CHECK(stop_calls == 1 && velocity_calls == 0);
                CHECK(context.phase_deadline_us == 1000 + RELEASE_SETTLE_MS * 1000LL);
            }
        }
        transport_context_t context = {.state = TRANSPORT_WAIT_SCENE};
        update_on_new_frame(&context, &vision, 1000);
        CHECK(context.state == TRANSPORT_WAIT_SCENE);
    }
    /* A single dark pixel and the ignored screen-left area do not trigger. */
    clear_frame();
    black_box(130, 60, 1, 1);
    CHECK(bt_detector_process_rgb565(&d, pixels, W, H, &r) == ESP_OK);
    CHECK(!r.black_ahead);
    clear_frame();
    black_box(240, 48, 60, 24);
    CHECK(bt_detector_process_rgb565(&d, pixels, W, H, &r) == ESP_OK);
    CHECK(!r.black_ahead);
    /* A nearby dark patch stops; the same patch far ahead does not. */
    clear_frame();
    black_box(110, 50, 40, 18);
    CHECK(bt_detector_process_rgb565(&d, pixels, W, H, &r) == ESP_OK);
    CHECK(r.black_ahead && r.black_ahead_percent >= 20);
    clear_frame();
    black_box(110, 100, 40, 18);
    CHECK(bt_detector_process_rgb565(&d, pixels, W, H, &r) == ESP_OK);
    CHECK(!r.black_ahead);
    return 0;
}

static int delivery_sequence(void)
{
    transport_context_t context = {
        .state = TRANSPORT_STRAIGHT_PUSH, .active_color = BALL_COLOR_WHITE
    };
    transport_vision_t vision = {.black_ahead = true, .ball_color = BALL_COLOR_WHITE};
    s_requested_color = BALL_COLOR_WHITE;
    int64_t now = 1000;
    update_on_new_frame(&context, &vision, now);
    CHECK(context.state == TRANSPORT_WHITE_EXTRA_PUSH);
    CHECK(last_vy == WHITE_EXTRA_PUSH_SPEED_MPS && last_omega == 0);
    const int64_t extra_end = context.phase_deadline_us;
    update_on_new_frame(&context, &vision, now + 1000);
    CHECK(context.phase_deadline_us == extra_end);
    stop_calls = velocity_calls = 0;
    update_delivery_timers(&context, extra_end - 1);
    CHECK(context.state == TRANSPORT_WHITE_EXTRA_PUSH && stop_calls == 0);
    update_delivery_timers(&context, extra_end);
    CHECK(context.state == TRANSPORT_SETTLE && stop_calls == 1);
    CHECK(velocity_calls == 0);
    CHECK(context.phase_deadline_us == extra_end + RELEASE_SETTLE_MS * 1000LL);
    now = context.phase_deadline_us;
    update_delivery_timers(&context, now);
    CHECK(context.state == TRANSPORT_BACK_AWAY && last_vy < 0);
    CHECK(context.phase_deadline_us == now + WHITE_BACK_AWAY_MS * 1000LL);
    CHECK(s_requested_color == BALL_COLOR_WHITE);
    now = context.phase_deadline_us;
    update_delivery_timers(&context, now);
    CHECK(context.state == TRANSPORT_PRETURN_ORANGE);
    CHECK(context.active_color == BALL_COLOR_ORANGE && s_requested_color == BALL_COLOR_ORANGE);
    CHECK(last_vy == 0 && last_omega > 0);
    CHECK(fabsf(ORANGE_PRETURN_LEFT_RAD_S * ORANGE_PRETURN_LEFT_MS / 1000.0f -
                30.0f * 3.14159265f / 180.0f) < 0.002f);
    const int64_t turn_end = context.phase_deadline_us;
    vision.ball_color = BALL_COLOR_ORANGE;
    vision.ball.valid = true;
    update_on_new_frame(&context, &vision, now + 1000);
    CHECK(context.state == TRANSPORT_PRETURN_ORANGE && context.phase_deadline_us == turn_end);
    stop_calls = velocity_calls = 0;
    update_delivery_timers(&context, turn_end - 1);
    CHECK(context.state == TRANSPORT_PRETURN_ORANGE && stop_calls == 0);
    update_delivery_timers(&context, turn_end);
    CHECK(context.state == TRANSPORT_TURN_TO_ORANGE && stop_calls == 1 && velocity_calls == 0);
    update_on_new_frame(&context, &vision, turn_end + 20000);
    CHECK(context.state == TRANSPORT_WAIT_SCENE);

    /* Orange retains immediate stopping and the short final release. */
    context.state = TRANSPORT_STRAIGHT_PUSH;
    update_on_new_frame(&context, &vision, turn_end + 40000);
    CHECK(context.state == TRANSPORT_SETTLE);
    now = context.phase_deadline_us;
    update_delivery_timers(&context, now);
    CHECK(context.state == TRANSPORT_BACK_AWAY);
    CHECK(context.phase_deadline_us == now + FINAL_BACK_AWAY_MS * 1000LL);
    update_delivery_timers(&context, context.phase_deadline_us);
    CHECK(context.state == TRANSPORT_FINISHED);
    return 0;
}

int main(void)
{
    int failures = independent_detection_and_mask();
    failures += approaching_target();
    failures += reject_floor_and_line();
    failures += camera_resolutions();
    failures += independent_parking();
    failures += delivery_sequence();
    if (failures) return 1;
    puts("PASS: target without ball; mask acquisition/retention/restoration; "
         "growing near target; current-frame boundary; misses; floor/line rejection; "
         "independent parking; noise/location rejection; orange full ROI; "
         "one-shot white trim/back-away/30-degree preturn/orange final release");
    return 0;
}
