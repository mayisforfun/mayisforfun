#include "board_port.h"
#include "line_follow.h"

#define MOTOR_DIAGNOSTIC_MODE 1
#define MOTOR_TEST_SPEED      500

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

#if !MOTOR_DIAGNOSTIC_MODE
static LineFollowConfig g_line_config = {
    .base_speed      = 380,
    .max_speed       = 750,
    .kp_q10          = 150,
    .ki_q10          = 0,
    .kd_q10          = 220,
    .lost_turn_speed = 260,
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

static void run_motor_diagnostic(void)
{
    while (1) {
        update_gray_debug(Board_readGray5());
        Board_setMotorSpeed(MOTOR_TEST_SPEED, MOTOR_TEST_SPEED);
        Board_delayMs(1200);

        update_gray_debug(Board_readGray5());
        Board_setMotorSpeed(0, 0);
        Board_delayMs(800);

        update_gray_debug(Board_readGray5());
        Board_setMotorSpeed(-MOTOR_TEST_SPEED, -MOTOR_TEST_SPEED);
        Board_delayMs(1200);

        update_gray_debug(Board_readGray5());
        Board_setMotorSpeed(0, 0);
        Board_delayMs(800);
    }
}

int main(void)
{
    Board_init();

#if MOTOR_DIAGNOSTIC_MODE
    run_motor_diagnostic();
#else
    LineFollowState line_state;
    LineFollow_init(&line_state);

    while (1) {
        __WFI();
        if (!g_ctrl_flag) {
            continue;
        }
        g_ctrl_flag = false;

        uint8_t sensor_bits = Board_readGray5();
        LineFollow_update(&line_state, &g_line_config,
                          sensor_bits, g_line_config.base_speed);
        update_line_debug(&line_state);
        Board_setMotorSpeed(line_state.left_speed, line_state.right_speed);
    }
#endif
}
