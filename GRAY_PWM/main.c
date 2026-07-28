#include "board_port.h"
#include "line_follow.h"
#include "track_fsm.h"
#include "encoder.h"
#include "speed_control.h"
#include "cross_detector.h"
#include "run_indicator.h"
#include "bluetooth_uart.h"

#define MOTOR_DIAGNOSTIC_MODE   0
#define ENCODER_DIAGNOSTIC_MODE 0
#define MOTOR_TEST_SPEED      500
#define ENCODER_TEST_SPEED    260
#define SPEED_CLOSED_LOOP_ENABLE 1
#define SPEED_TARGET_TICKS_PER_1000 30
#define SPEED_PID_KP_Q10      180
#define SPEED_PID_KI_Q10      0
#define SPEED_PID_KD_Q10      20
#define SPEED_PID_MAX_OUTPUT  80
#define MOTOR_FORWARD_SIGN    (-1)
#define BUTTON_DEBOUNCE_TICKS 3U
#define CROSS_STOP_TEST_ENABLE 1
#define CROSS_STOP_FULL_BLACK_TICKS 2U

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
volatile uint32_t g_button_press_count = 0;
volatile uint8_t  g_track_state       = TRACK_STRAIGHT;
volatile uint16_t g_fsm_base_speed    = 310;
volatile int32_t  g_encoder_left_delta  = 0;
volatile int32_t  g_encoder_right_delta = 0;
volatile int32_t  g_encoder_left_total  = 0;
volatile int32_t  g_encoder_right_total = 0;
volatile int16_t  g_speed_left_target_ticks = 0;
volatile int16_t  g_speed_right_target_ticks = 0;
volatile int16_t  g_speed_left_actual_ticks = 0;
volatile int16_t  g_speed_right_actual_ticks = 0;
volatile int16_t  g_speed_left_correction = 0;
volatile int16_t  g_speed_right_correction = 0;
volatile int16_t  g_speed_left_pwm = 0;
volatile int16_t  g_speed_right_pwm = 0;
volatile uint16_t g_cross_count = 0;
volatile bool     g_cross_latched = false;
volatile bool     g_cross_event = false;
volatile bool     g_cross_candidate = false;
volatile bool     g_cross_stop_test_done = false;
volatile uint8_t  g_cross_stop_black_ticks = 0;
volatile uint32_t g_bt_rx_count = 0;
volatile uint32_t g_bt_line_count = 0;
volatile uint32_t g_bt_overflow_count = 0;
volatile uint8_t  g_bt_last_cmd = BT_CMD_NONE;
volatile bool     g_bt_line_event = false;
volatile char     g_bt_last_line[BT_UART_LINE_SIZE] = {0};

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

static void update_bluetooth_debug(BluetoothUART *bt,
                                   const char *line,
                                   BluetoothCommand cmd,
                                   bool line_event)
{
    uint8_t i = 0;

    g_bt_rx_count = bt->rx_count;
    g_bt_line_count = bt->line_count;
    g_bt_overflow_count = bt->overflow_count;
    g_bt_last_cmd = (uint8_t) cmd;
    g_bt_line_event = line_event;

    if (line_event) {
        while ((i + 1U < BT_UART_LINE_SIZE) && (line[i] != '\0')) {
            g_bt_last_line[i] = line[i];
            i++;
        }
        g_bt_last_line[i] = '\0';
    }
}
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
            g_button_press_count++;
            return true;
        }
    }

    return false;
}



static int16_t clamp_i16(int32_t value, int16_t min_value, int16_t max_value)
{
    if (value > max_value) {
        return max_value;
    }
    if (value < min_value) {
        return min_value;
    }
    return (int16_t) value;
}

static int16_t speed_to_target_ticks(int16_t speed)
{
    return (int16_t) (((int32_t) speed * SPEED_TARGET_TICKS_PER_1000) / 1000);
}

static void reset_speed_debug(void)
{
    g_encoder_left_delta = 0;
    g_encoder_right_delta = 0;
    g_encoder_left_total = 0;
    g_encoder_right_total = 0;
    g_speed_left_target_ticks = 0;
    g_speed_right_target_ticks = 0;
    g_speed_left_actual_ticks = 0;
    g_speed_right_actual_ticks = 0;
    g_speed_left_correction = 0;
    g_speed_right_correction = 0;
    g_speed_left_pwm = 0;
    g_speed_right_pwm = 0;
}

static void set_motor_speed_closed_loop(SpeedPID *left_pid, SpeedPID *right_pid,
                                        int16_t left_target, int16_t right_target)
{
    int16_t left_pwm = left_target;
    int16_t right_pwm = right_target;

    g_encoder_left_delta = Encoder_getLeftTicks();
    g_encoder_right_delta = Encoder_getRightTicks();
    g_encoder_left_total += g_encoder_left_delta;
    g_encoder_right_total += g_encoder_right_delta;

    g_speed_left_actual_ticks = clamp_i16(g_encoder_left_delta, -32768, 32767);
    g_speed_right_actual_ticks = clamp_i16(g_encoder_right_delta, -32768, 32767);
    g_speed_left_target_ticks = speed_to_target_ticks(left_target);
    g_speed_right_target_ticks = speed_to_target_ticks(right_target);

#if SPEED_CLOSED_LOOP_ENABLE
    if (left_target == 0) {
        SpeedPID_reset(left_pid);
        g_speed_left_correction = 0;
    } else {
        g_speed_left_correction = SpeedPID_compute(left_pid,
            g_speed_left_target_ticks, g_speed_left_actual_ticks);
        left_pwm = clamp_i16((int32_t) left_target + g_speed_left_correction,
                             -(int16_t) g_line_config.max_speed,
                             (int16_t) g_line_config.max_speed);
    }

    if (right_target == 0) {
        SpeedPID_reset(right_pid);
        g_speed_right_correction = 0;
    } else {
        g_speed_right_correction = SpeedPID_compute(right_pid,
            g_speed_right_target_ticks, g_speed_right_actual_ticks);
        right_pwm = clamp_i16((int32_t) right_target + g_speed_right_correction,
                              -(int16_t) g_line_config.max_speed,
                              (int16_t) g_line_config.max_speed);
    }
#else
    g_speed_left_correction = 0;
    g_speed_right_correction = 0;
#endif

    g_speed_left_pwm = left_pwm;
    g_speed_right_pwm = right_pwm;
    set_motor_debug(left_pwm, right_pwm);
}
static void run_encoder_diagnostic(void)
{
    Encoder_init();
    Encoder_clearDeltas();
    set_motor_debug(0, 0);

    while (1) {
        __WFI();
        if (!g_ctrl_flag) {
            continue;
        }
        g_ctrl_flag = false;


        if (button_pressed_event()) {
            g_run_enabled = !g_run_enabled;
            Encoder_clearDeltas();
            g_encoder_left_delta = 0;
            g_encoder_right_delta = 0;
            g_encoder_left_total = 0;
            g_encoder_right_total = 0;
        }

        g_encoder_left_delta = Encoder_getLeftTicks();
        g_encoder_right_delta = Encoder_getRightTicks();
        g_encoder_left_total += g_encoder_left_delta;
        g_encoder_right_total += g_encoder_right_delta;

        if (g_run_enabled) {
            set_motor_debug(ENCODER_TEST_SPEED, ENCODER_TEST_SPEED);
        } else {
            set_motor_debug(0, 0);
        }
    }
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

#if ENCODER_DIAGNOSTIC_MODE
    run_encoder_diagnostic();
#elif MOTOR_DIAGNOSTIC_MODE
    run_motor_diagnostic();
#else
    LineFollowState line_state;
    TrackFSM track_fsm;
    SpeedPID left_speed_pid;
    SpeedPID right_speed_pid;
    CrossDetector cross_detector;
    RunIndicator run_indicator;
    BluetoothUART bt_uart;
    char bt_line[BT_UART_LINE_SIZE];
    uint16_t current_base_speed;
    uint8_t cross_stop_black_ticks = 0;

    LineFollow_init(&line_state);
    TrackFSM_init(&track_fsm);
    Encoder_init();
    Encoder_clearDeltas();
    SpeedPID_init(&left_speed_pid, SPEED_PID_KP_Q10, SPEED_PID_KI_Q10,
                  SPEED_PID_KD_Q10, SPEED_PID_MAX_OUTPUT);
    SpeedPID_init(&right_speed_pid, SPEED_PID_KP_Q10, SPEED_PID_KI_Q10,
                  SPEED_PID_KD_Q10, SPEED_PID_MAX_OUTPUT);
    CrossDetector_init(&cross_detector);
    RunIndicator_init(&run_indicator);
    BluetoothUART_init(&bt_uart);
    bt_line[0] = '\0';
    current_base_speed = track_fsm.base_speed;
    g_fsm_base_speed = current_base_speed;

    set_motor_debug(0, 0);

    while (1) {
        __WFI();
        if (!g_ctrl_flag) {
            continue;
        }
        g_ctrl_flag = false;

        BluetoothUART_poll(&bt_uart);
        if (BluetoothUART_getLine(&bt_uart, bt_line, (uint8_t) sizeof(bt_line))) {
            BluetoothCommand bt_cmd = BluetoothUART_getCommand(&bt_uart);
            update_bluetooth_debug(&bt_uart, bt_line, bt_cmd, true);
            BluetoothUART_sendLine("OK");
        } else {
            update_bluetooth_debug(&bt_uart, bt_line, BluetoothUART_getCommand(&bt_uart), false);
        }

        if (button_pressed_event()) {
            g_run_enabled = !g_run_enabled;
            set_motor_debug(0, 0);
            LineFollow_init(&line_state);
            TrackFSM_init(&track_fsm);
            CrossDetector_init(&cross_detector);
            Encoder_clearDeltas();
            SpeedPID_reset(&left_speed_pid);
            SpeedPID_reset(&right_speed_pid);
            reset_speed_debug();
            current_base_speed = track_fsm.base_speed;
            g_fsm_base_speed = current_base_speed;
            g_cross_count = 0;
            g_cross_latched = false;
            g_cross_event = false;
            g_cross_candidate = false;
            g_cross_stop_black_ticks = 0;
            cross_stop_black_ticks = 0;

            if (g_run_enabled) {
                g_cross_stop_test_done = false;
                RunIndicator_onStart(&run_indicator);
            } else {
                RunIndicator_onStop(&run_indicator);
            }
        }

        uint8_t sensor_bits = Board_readGray5();

        LineFollow_update(&line_state, &g_line_config,
                          sensor_bits, current_base_speed);
        update_line_debug(&line_state);

        CrossDetector_update(&cross_detector, sensor_bits, g_run_enabled);
        g_cross_count = cross_detector.count;
        g_cross_latched = cross_detector.latched;
        g_cross_event = cross_detector.event;
        g_cross_candidate = cross_detector.candidate;

        TrackFSM_update(&track_fsm, &line_state, &current_base_speed);
        g_track_state = (uint8_t) track_fsm.state;
        g_fsm_base_speed = current_base_speed;

#if CROSS_STOP_TEST_ENABLE
        if (g_run_enabled && ((sensor_bits & 0x1FU) == 0x1FU)) {
            if (cross_stop_black_ticks < CROSS_STOP_FULL_BLACK_TICKS) {
                cross_stop_black_ticks++;
            }
        } else {
            cross_stop_black_ticks = 0;
        }
        g_cross_stop_black_ticks = cross_stop_black_ticks;

        if (g_run_enabled && (cross_stop_black_ticks >= CROSS_STOP_FULL_BLACK_TICKS)) {
            g_run_enabled = false;
            g_cross_stop_test_done = true;
            cross_stop_black_ticks = 0;
            g_cross_stop_black_ticks = 0;
            set_motor_debug(0, 0);
            SpeedPID_reset(&left_speed_pid);
            SpeedPID_reset(&right_speed_pid);
            reset_speed_debug();
            RunIndicator_onStop(&run_indicator);
            RunIndicator_update(&run_indicator, false);
            continue;
        }
#endif

        if (!g_run_enabled) {
            g_line_left_speed = 0;
            g_line_right_speed = 0;
            set_motor_debug(0, 0);
            RunIndicator_update(&run_indicator, false);
            continue;
        }

        set_motor_speed_closed_loop(&left_speed_pid, &right_speed_pid,
                                    line_state.left_speed,
                                    line_state.right_speed);
        RunIndicator_update(&run_indicator, true);
    }
#endif
}







