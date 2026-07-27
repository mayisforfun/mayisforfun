#include "board_port.h"
#include "line_follow.h"
#include "track_fsm.h"

#define MOTOR_DIAGNOSTIC_MODE 0
#define MOTOR_TEST_SPEED      500
#define MOTOR_FORWARD_SIGN    (-1)
#define BUTTON_DEBOUNCE_TICKS 3U

volatile bool     g_ctrl_flag = false;
volatile uint32_t g_sys_ticks = 0;

volatile uint8_t  g_gray_bits         = 0;
volatile bool     g_gray_l2           = false;
volatile bool     g_gray_l1           = false;
volatile bool     g_gray_c            = false;
volatile bool     g_gray_r1           = false;
volatile bool     g_gray_r2           = false;
volatile bool     g_line_seen         = false;
volatile bool     g_line_sensor_valid = false;
volatile int16_t  g_line_position     = 0;
volatile int16_t  g_line_error        = 0;
volatile int16_t  g_line_left_speed   = 0;
volatile int16_t  g_line_right_speed  = 0;
volatile int16_t  g_motor_left_cmd    = 0;
volatile int16_t  g_motor_right_cmd   = 0;
volatile bool     g_run_enabled       = false;
volatile bool     g_button_pressed    = false;
volatile uint8_t  g_track_state       = TRACK_STRAIGHT;
volatile uint16_t g_fsm_base_speed    = 310;

#if !MOTOR_DIAGNOSTIC_MODE
static LineFollowConfig g_line_config = {
    .base_speed      = 310,
    .max_speed       = 480,
    .kp_q10          = 85,
    .ki_q10          = 0,
    .kd_q10          = 95,
    .lost_turn_speed = 180,
};
#endif

void SysTick_Handler(void)
{
    g_ctrl_flag = true;
    g_sys_ticks++;
}

static void update_gray_debug(uint8_t bits)
{
    g_gray_bits = bits;
    g_gray_l2   = (bits & (1U << 0)) != 0U;
    g_gray_l1   = (bits & (1U << 1)) != 0U;
    g_gray_c    = (bits & (1U << 2)) != 0U;
    g_gray_r1   = (bits & (1U << 3)) != 0U;
    g_gray_r2   = (bits & (1U << 4)) != 0U;
}

#if !MOTOR_DIAGNOSTIC_MODE
static void update_line_debug(const LineFollowState *state)
{
    update_gray_debug(state->raw_sensor_bits);
    g_line_seen         = state->line_seen;
    g_line_sensor_valid = state->sensor_valid;
    g_line_position     = state->position;
    g_line_error        = state->error;
    g_line_left_speed   = state->left_speed;
    g_line_right_speed  = state->right_speed;
}
#endif

static void set_motor_debug(int16_t left, int16_t right)
{
    int16_t motor_left = (int16_t) (MOTOR_FORWARD_SIGN * right);
    int16_t motor_right = (int16_t) (MOTOR_FORWARD_SIGN * left);

    g_motor_left_cmd = motor_left;
    g_motor_right_cmd = motor_right;
    Board_setMotorSpeed(motor_left, motor_right);
}

static bool button_pressed_event(void)
{
    static bool last_raw = false;
    static bool stable_state = false;
    static uint8_t stable_ticks = 0;

    bool raw = Board_readStartButton();
    g_button_pressed = raw;

    if (raw == last_raw) {
        if (stable_ticks < BUTTON_DEBOUNCE_TICKS) {
            stable_ticks++;
        }
    } else {
        stable_ticks = 0;
        last_raw = raw;
    }

    if ((stable_ticks >= BUTTON_DEBOUNCE_TICKS) && (raw != stable_state)) {
        stable_state = raw;
        if (stable_state) {
            return true;
        }
    }

    return false;
}

static void run_motor_diagnostic(void)
{
    while (1) {
        update_gray_debug(Board_readGray5());
        set_motor_debug(MOTOR_TEST_SPEED, MOTOR_TEST_SPEED);
        Board_delayMs(100);
    }
}

int main(void)
{
    Board_init();

#if MOTOR_DIAGNOSTIC_MODE
    run_motor_diagnostic();
#else
    LineFollowState line_state;
    TrackFSM track_fsm;
    uint16_t current_base_speed;

    LineFollow_init(&line_state);
    TrackFSM_init(&track_fsm);
    current_base_speed = track_fsm.base_speed;
    g_fsm_base_speed = current_base_speed;

    set_motor_debug(0, 0);

    while (1) {
        __WFI();
        if (!g_ctrl_flag) {
            continue;
        }
        g_ctrl_flag = false;

        if (button_pressed_event()) {
            g_run_enabled = !g_run_enabled;
            set_motor_debug(0, 0);
            LineFollow_init(&line_state);
            TrackFSM_init(&track_fsm);
            current_base_speed = track_fsm.base_speed;
            g_fsm_base_speed = current_base_speed;
        }

        uint8_t sensor_bits = Board_readGray5();

        if (!g_run_enabled) {
            update_gray_debug(sensor_bits);
            g_line_seen = false;
            g_line_sensor_valid = false;
            g_line_left_speed = 0;
            g_line_right_speed = 0;
            g_track_state = TRACK_STRAIGHT;
            set_motor_debug(0, 0);
            continue;
        }

        LineFollow_update(&line_state, &g_line_config,
                          sensor_bits, current_base_speed);
        update_line_debug(&line_state);

        TrackFSM_update(&track_fsm, &line_state, &current_base_speed);
        g_track_state = (uint8_t) track_fsm.state;
        g_fsm_base_speed = current_base_speed;

        set_motor_debug(line_state.left_speed, line_state.right_speed);
    }
#endif
}