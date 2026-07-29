#include "board_port.h"
#include "line_follow.h"
#include "track_fsm.h"
#include "encoder.h"
#include "speed_control.h"
#include "cross_detector.h"
#include "run_indicator.h"
#include "bluetooth_uart.h"
#include "mpu6050.h"
#include "oled_debug.h"

#define MOTOR_DIAGNOSTIC_MODE   0
#define ENCODER_DIAGNOSTIC_MODE 0
#define MPU_DIAGNOSTIC_MODE     0
#define MOTOR_TEST_SPEED      500
#define ENCODER_TEST_SPEED    260
#define SPEED_CLOSED_LOOP_ENABLE 0
#define SPEED_TARGET_TICKS_PER_1000 30
#define SPEED_PID_KP_Q10      180
#define SPEED_PID_KI_Q10      0
#define SPEED_PID_KD_Q10      20
#define SPEED_PID_MAX_OUTPUT  80
#define MOTOR_FORWARD_SIGN    (-1)
#define MOTOR_OUTPUT_SWAP     1
#define MOTOR_MIN_RUN_SPEED   300
#define BUTTON_DEBOUNCE_TICKS 3U
#define CROSS_STOP_TEST_ENABLE 1
#define CROSS_STOP_FULL_BLACK_TICKS 2U
#define BT_HEARTBEAT_TEST_ENABLE 0
#define TASK2024_ENABLE       1
#define TASK_TICKS_PER_CM     30L
#define TASK1_TEST_CM         130
#define TASK1_TEST_SPEED      380
#define TASK_STRAIGHT_SPEED   300
#define TASK_ARC_SPEED        230
#define TASK_TURN_SPEED       190
#define TASK_HEADING_ENABLE   1
#define TASK_HEADING_KP_Q10   45
#define TASK_HEADING_SIGN     1
#define TASK_STRAIGHT_LEFT_TRIM   -10
#define TASK_STRAIGHT_RIGHT_TRIM  0
#define TASK_AB_CD_CM         100
#define TASK_AC_BD_CM         128
#define TASK_ARC_CM           126
#define TASK_DIAGONAL_DEG     39
#define TASK_TURN_TOL_DEG_X100 350L
#define TASK_TURN_SIGN        1
#define TASK_LINE_RELEASE_TICKS 8U
#define TASK_LINE_ENTER_TICKS   2U
#define TASK_SEARCH_START_PERCENT 60L
#define TASK_SEARCH_SWING_DEG_X100 1200L
#define TASK_SEARCH_HALF_PERIOD_TICKS 35L
#define H_TASK2_STOP_MASK     ((1U << 1) | (1U << 2) | (1U << 3))
#define H_TASK2_STOP_TICKS    3U
#define H_TASK2_RELEASE_TICKS 10U
#define OLED_UPDATE_TICKS     10U

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
volatile bool     g_mpu_ok = false;
volatile uint8_t  g_mpu_who_am_i = 0;
volatile uint8_t  g_mpu_addr = 0;
volatile bool     g_imu_ok = false;
volatile uint8_t  g_mpu6050_who_am_i = 0;
volatile int32_t  g_mpu_gyro_z_mdps = 0;
volatile int32_t  g_mpu_gyro_z_bias_mdps = 0;
volatile int32_t  g_mpu_yaw_deg_x100 = 0;
volatile uint32_t g_mpu_init_attempts = 0;
volatile uint32_t g_mpu_read_ok_count = 0;
volatile uint32_t g_mpu_read_fail_count = 0;
volatile bool     g_mpu_init_ok_once = false;
volatile bool     g_mpu_raw_read_ok = false;
volatile int16_t  g_mpu_accel_x = 0;
volatile int16_t  g_mpu_accel_y = 0;
volatile int16_t  g_mpu_accel_z = 0;
volatile int16_t  g_mpu_gyro_x = 0;
volatile int16_t  g_mpu_gyro_y = 0;
volatile int16_t  g_mpu_gyro_z = 0;
volatile int16_t  g_imu_accel_x = 0;
volatile int16_t  g_imu_accel_y = 0;
volatile int16_t  g_imu_accel_z = 0;
volatile int16_t  g_imu_gyro_x = 0;
volatile int16_t  g_imu_gyro_y = 0;
volatile int16_t  g_imu_gyro_z = 0;
volatile uint32_t g_imu_read_count = 0;
volatile uint32_t g_imu_read_fail_count = 0;
volatile uint8_t  g_task2024_id = 0;
volatile uint8_t  g_task2024_step = 0;
volatile uint8_t  g_task2024_action = 0;
volatile int32_t  g_task2024_segment_ticks = 0;
volatile int32_t  g_task2024_target_ticks = 0;
volatile int32_t  g_task2024_yaw_target = 0;
volatile uint8_t  g_task2024_lap = 0;
volatile bool     g_task2024_search_active = false;
volatile bool     g_h_task2_active = false;
volatile bool     g_h_task2_done = false;
volatile bool     g_h_task2_released = false;
volatile uint32_t g_h_task2_start_ticks = 0;
volatile uint32_t g_h_task2_elapsed_ticks = 0;
volatile uint8_t  g_h_task2_stop_ticks = 0;
volatile uint8_t  g_h_task2_release_ticks = 0;

#if !MOTOR_DIAGNOSTIC_MODE
static LineFollowConfig g_line_config = {
    .base_speed      = 420,
    .max_speed       = 700,
    .kp_q10          = 110,
    .ki_q10          = 0,
    .kd_q10          = 145,
    .lost_turn_speed = 260,
};
#endif

static int16_t clamp_i16(int32_t value, int16_t min_value, int16_t max_value);

static void h_task2_start(void)
{
    g_h_task2_active = true;
    g_h_task2_done = false;
    g_h_task2_released = false;
    g_h_task2_start_ticks = g_sys_ticks;
    g_h_task2_elapsed_ticks = 0;
    g_h_task2_stop_ticks = 0;
    g_h_task2_release_ticks = 0;
}

static void h_task2_stop(void)
{
    g_h_task2_active = false;
    g_h_task2_done = true;
    g_h_task2_released = false;
    g_h_task2_elapsed_ticks = g_sys_ticks - g_h_task2_start_ticks;
    g_h_task2_stop_ticks = 0;
    g_h_task2_release_ticks = 0;
    g_run_enabled = false;
}

static bool h_task2_should_stop(uint8_t sensor_bits)
{
    bool stop_line_seen = ((sensor_bits & H_TASK2_STOP_MASK) == H_TASK2_STOP_MASK);

    if (!g_h_task2_released) {
        if (!stop_line_seen) {
            if (g_h_task2_release_ticks < H_TASK2_RELEASE_TICKS) {
                g_h_task2_release_ticks++;
            }
            if (g_h_task2_release_ticks >= H_TASK2_RELEASE_TICKS) {
                g_h_task2_released = true;
            }
        } else {
            g_h_task2_release_ticks = 0;
        }
        g_h_task2_stop_ticks = 0;
        return false;
    }

    if (stop_line_seen) {
        if (g_h_task2_stop_ticks < H_TASK2_STOP_TICKS) {
            g_h_task2_stop_ticks++;
        }
    } else {
        g_h_task2_stop_ticks = 0;
    }

    return g_h_task2_stop_ticks >= H_TASK2_STOP_TICKS;
}

static uint32_t h_task2_display_ticks(void)
{
    if (g_h_task2_active) {
        return g_sys_ticks - g_h_task2_start_ticks;
    }
    return g_h_task2_elapsed_ticks;
}

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
#if MOTOR_OUTPUT_SWAP
    int16_t motor_left = (int16_t) (MOTOR_FORWARD_SIGN * right);
    int16_t motor_right = (int16_t) (MOTOR_FORWARD_SIGN * left);
#else
    int16_t motor_left = (int16_t) (MOTOR_FORWARD_SIGN * left);
    int16_t motor_right = (int16_t) (MOTOR_FORWARD_SIGN * right);
#endif

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

#if TASK2024_ENABLE
typedef enum {
    TASK_ACTION_NONE = 0,
    TASK_ACTION_STRAIGHT,
    TASK_ACTION_STRAIGHT_TO_LINE,
    TASK_ACTION_ARC,
    TASK_ACTION_TURN,
    TASK_ACTION_STOP,
} Task2024ActionKind;

typedef struct {
    Task2024ActionKind kind;
    int16_t value;
    uint16_t speed;
    bool beep_after;
} Task2024Action;

typedef struct {
    bool active;
    uint8_t id;
    uint8_t step;
    uint8_t lap;
    uint8_t repeat_count;
    int32_t segment_ticks;
    int32_t target_ticks;
    int32_t yaw_target_deg_x100;
    int32_t heading_hold_deg_x100;
    uint8_t line_release_ticks;
    uint8_t line_enter_ticks;
    bool line_released;
    Task2024Action current;
} Task2024State;

static const Task2024Action g_task1_actions[] = {
    {TASK_ACTION_STRAIGHT_TO_LINE, TASK1_TEST_CM, TASK1_TEST_SPEED, true},
    {TASK_ACTION_STOP, 0, 0, false},
};

static const Task2024Action g_task2_actions[] = {
    {TASK_ACTION_STRAIGHT_TO_LINE, TASK_AB_CD_CM, TASK_STRAIGHT_SPEED, true},
    {TASK_ACTION_ARC,      TASK_ARC_CM,   TASK_ARC_SPEED,      true},
    {TASK_ACTION_STRAIGHT_TO_LINE, TASK_AB_CD_CM, TASK_STRAIGHT_SPEED, true},
    {TASK_ACTION_ARC,      TASK_ARC_CM,   TASK_ARC_SPEED,      true},
    {TASK_ACTION_STOP, 0, 0, false},
};

static const Task2024Action g_task3_actions[] = {
    {TASK_ACTION_TURN,     -TASK_DIAGONAL_DEG, TASK_TURN_SPEED,     false},
    {TASK_ACTION_STRAIGHT_TO_LINE, TASK_AC_BD_CM, TASK_STRAIGHT_SPEED, true},
    {TASK_ACTION_TURN,      TASK_DIAGONAL_DEG, TASK_TURN_SPEED,     false},
    {TASK_ACTION_ARC,       TASK_ARC_CM,       TASK_ARC_SPEED,      true},
    {TASK_ACTION_TURN,      TASK_DIAGONAL_DEG, TASK_TURN_SPEED,     false},
    {TASK_ACTION_STRAIGHT_TO_LINE, TASK_AC_BD_CM, TASK_STRAIGHT_SPEED, true},
    {TASK_ACTION_TURN,     -TASK_DIAGONAL_DEG, TASK_TURN_SPEED,     false},
    {TASK_ACTION_ARC,       TASK_ARC_CM,       TASK_ARC_SPEED,      true},
    {TASK_ACTION_STOP, 0, 0, false},
};

static int32_t abs_i32(int32_t value)
{
    return (value >= 0) ? value : -value;
}

static int32_t cm_to_ticks(int16_t cm)
{
    return ((int32_t) cm) * TASK_TICKS_PER_CM;
}

static void task_drive_heading(int16_t speed,
                               int32_t target_heading_deg_x100,
                               int16_t *left_speed,
                               int16_t *right_speed)
{
#if TASK_HEADING_ENABLE
    int32_t heading_error = g_mpu_yaw_deg_x100 - target_heading_deg_x100;
    int32_t correction = ((int32_t) TASK_HEADING_SIGN *
                          (int32_t) TASK_HEADING_KP_Q10 *
                          heading_error) / 1024L;

    *left_speed = clamp_i16((int32_t) speed + TASK_STRAIGHT_LEFT_TRIM + correction,
                            0, (int16_t) g_line_config.max_speed);
    *right_speed = clamp_i16((int32_t) speed + TASK_STRAIGHT_RIGHT_TRIM - correction,
                             0, (int16_t) g_line_config.max_speed);
#else
    (void) target_heading_deg_x100;
    *left_speed = clamp_i16((int32_t) speed + TASK_STRAIGHT_LEFT_TRIM,
                            0, (int16_t) g_line_config.max_speed);
    *right_speed = clamp_i16((int32_t) speed + TASK_STRAIGHT_RIGHT_TRIM,
                             0, (int16_t) g_line_config.max_speed);
#endif
}

static int32_t task_search_heading_offset(const Task2024State *task)
{
    int32_t phase;

    if (task->segment_ticks < ((task->target_ticks * TASK_SEARCH_START_PERCENT) / 100L)) {
        return 0;
    }

    phase = (task->segment_ticks / TASK_SEARCH_HALF_PERIOD_TICKS) & 1L;
    return (phase == 0L) ? TASK_SEARCH_SWING_DEG_X100 : -TASK_SEARCH_SWING_DEG_X100;
}

static void update_task_debug(const Task2024State *task)
{
    g_task2024_id = task->id;
    g_task2024_step = task->step;
    g_task2024_action = (uint8_t) task->current.kind;
    g_task2024_segment_ticks = task->segment_ticks;
    g_task2024_target_ticks = task->target_ticks;
    g_task2024_yaw_target = task->yaw_target_deg_x100;
    g_task2024_lap = task->lap;
    g_task2024_search_active =
        (task->current.kind == TASK_ACTION_STRAIGHT_TO_LINE) &&
        (task_search_heading_offset(task) != 0L);
}

static void Task2024_reset(Task2024State *task)
{
    task->active = false;
    task->id = 0;
    task->step = 0;
    task->lap = 0;
    task->repeat_count = 0;
    task->segment_ticks = 0;
    task->target_ticks = 0;
    task->yaw_target_deg_x100 = 0;
    task->heading_hold_deg_x100 = 0;
    task->line_release_ticks = 0;
    task->line_enter_ticks = 0;
    task->line_released = false;
    task->current.kind = TASK_ACTION_NONE;
    task->current.value = 0;
    task->current.speed = 0;
    task->current.beep_after = false;
    update_task_debug(task);
}

static const Task2024Action *Task2024_get_actions(uint8_t id, uint8_t *count)
{
    if (id == 1U) {
        *count = (uint8_t) (sizeof(g_task1_actions) / sizeof(g_task1_actions[0]));
        return g_task1_actions;
    }
    if (id == 2U) {
        *count = (uint8_t) (sizeof(g_task2_actions) / sizeof(g_task2_actions[0]));
        return g_task2_actions;
    }

    *count = (uint8_t) (sizeof(g_task3_actions) / sizeof(g_task3_actions[0]));
    return g_task3_actions;
}

static void Task2024_start(Task2024State *task, uint8_t id)
{
    Task2024_reset(task);
    task->active = true;
    task->id = id;
    task->repeat_count = (id == 4U) ? 4U : 1U;
    task->lap = 1U;
    update_task_debug(task);
}

static void Task2024_begin_action(Task2024State *task)
{
    uint8_t count = 0;
    const Task2024Action *actions = Task2024_get_actions(task->id, &count);

    if (task->step >= count) {
        task->current.kind = TASK_ACTION_STOP;
        return;
    }

    task->current = actions[task->step];
    task->segment_ticks = 0;
    task->target_ticks = 0;

    if ((task->current.kind == TASK_ACTION_STRAIGHT) ||
        (task->current.kind == TASK_ACTION_STRAIGHT_TO_LINE) ||
        (task->current.kind == TASK_ACTION_ARC)) {
        task->target_ticks = cm_to_ticks(task->current.value);
        task->heading_hold_deg_x100 = g_mpu_yaw_deg_x100;
        task->line_release_ticks = 0;
        task->line_enter_ticks = 0;
        task->line_released = false;
    } else if (task->current.kind == TASK_ACTION_TURN) {
        task->yaw_target_deg_x100 =
            g_mpu_yaw_deg_x100 + ((int32_t) task->current.value * 100L);
    }
}

static void Task2024_advance(Task2024State *task, RunIndicator *indicator)
{
    if (task->current.beep_after) {
        RunIndicator_onStart(indicator);
    }

    task->step++;

    if ((task->id == 4U) && (task->step >=
        (uint8_t) ((sizeof(g_task3_actions) / sizeof(g_task3_actions[0])) - 1U))) {
        if (task->lap < task->repeat_count) {
            task->lap++;
            task->step = 0;
        }
    }

    task->current.kind = TASK_ACTION_NONE;
}

static bool task_key_pressed_event(uint8_t key_id)
{
    static bool last_raw[4] = {false, false, false, false};
    static bool stable_state[4] = {false, false, false, false};
    static uint8_t stable_ticks[4] = {0, 0, 0, 0};
    uint8_t idx;
    bool raw;

    if ((key_id < 1U) || (key_id > 4U)) {
        return false;
    }

    idx = key_id - 1U;
    raw = Board_readTaskKey(key_id);

    if (raw == last_raw[idx]) {
        if (stable_ticks[idx] < BUTTON_DEBOUNCE_TICKS) {
            stable_ticks[idx]++;
        }
    } else {
        stable_ticks[idx] = 0;
        last_raw[idx] = raw;
    }

    if ((stable_ticks[idx] >= BUTTON_DEBOUNCE_TICKS) &&
        (raw != stable_state[idx])) {
        stable_state[idx] = raw;
        if (stable_state[idx]) {
            g_button_press_count++;
            return true;
        }
    }

    return false;
}

static void imu_init_debug(void)
{
    MPU6050RawData raw;
    MPU6050ScaledData scaled;
    int64_t bias_sum = 0;
    uint8_t samples = 0;

    if (!MPU6050_init()) {
        g_mpu_ok = false;
        g_imu_ok = false;
        (void) MPU6050_readWhoAmI((uint8_t *) &g_mpu_who_am_i);
        g_mpu6050_who_am_i = g_mpu_who_am_i;
        g_mpu_addr = MPU6050_I2C_ADDR_AD0_LOW;
        return;
    }

    g_mpu_ok = true;
    g_imu_ok = true;
    g_mpu_init_ok_once = true;
    g_mpu_addr = MPU6050_I2C_ADDR_AD0_LOW;
    (void) MPU6050_readWhoAmI((uint8_t *) &g_mpu_who_am_i);
    g_mpu6050_who_am_i = g_mpu_who_am_i;
    for (uint8_t i = 0; i < 80U; i++) {
        if (MPU6050_readRaw(&raw)) {
            MPU6050_scaleDefault(&raw, &scaled);
            bias_sum += scaled.gyro_z_mdps;
            samples++;
        }
        Board_delayMs(2);
    }

    if (samples > 0U) {
        g_mpu_gyro_z_bias_mdps = (int32_t) (bias_sum / samples);
    }
}

static void imu_update_heading(void)
{
    MPU6050RawData raw;
    MPU6050ScaledData scaled;
    int32_t corrected_z;

    if (!g_mpu_ok) {
        return;
    }

    if (!MPU6050_readRaw(&raw)) {
        g_mpu_raw_read_ok = false;
        g_mpu_read_fail_count++;
        g_imu_read_fail_count++;
        return;
    }

    g_mpu_raw_read_ok = true;
    g_mpu_read_ok_count++;
    g_imu_read_count++;
    g_mpu_accel_x = raw.accel_x;
    g_mpu_accel_y = raw.accel_y;
    g_mpu_accel_z = raw.accel_z;
    g_mpu_gyro_x = raw.gyro_x;
    g_mpu_gyro_y = raw.gyro_y;
    g_mpu_gyro_z = raw.gyro_z;
    g_imu_accel_x = raw.accel_x;
    g_imu_accel_y = raw.accel_y;
    g_imu_accel_z = raw.accel_z;
    g_imu_gyro_x = raw.gyro_x;
    g_imu_gyro_y = raw.gyro_y;
    g_imu_gyro_z = raw.gyro_z;

    MPU6050_scaleDefault(&raw, &scaled);
    g_mpu_gyro_z_mdps = scaled.gyro_z_mdps;
    corrected_z = scaled.gyro_z_mdps - g_mpu_gyro_z_bias_mdps;

    /* Loop period is 10 ms. mdps * 10ms / 10000 = degrees * 100. */
    g_mpu_yaw_deg_x100 += corrected_z / 1000L;
}

static bool Task2024_update(Task2024State *task,
                            const LineFollowState *line_state,
                            uint8_t sensor_bits,
                            RunIndicator *indicator,
                            int16_t *left_speed,
                            int16_t *right_speed)
{
    int32_t avg_ticks;
    bool mid_black;
    bool any_black;

    if (!task->active) {
        return false;
    }

    avg_ticks = (abs_i32(g_encoder_left_delta) + abs_i32(g_encoder_right_delta)) / 2L;
    task->segment_ticks += avg_ticks;
    sensor_bits &= 0x1FU;
    mid_black = (sensor_bits & ((1U << 1) | (1U << 2) | (1U << 3))) != 0U;
    any_black = sensor_bits != 0U;

    if (task->current.kind == TASK_ACTION_NONE) {
        Task2024_begin_action(task);
    }

    switch (task->current.kind) {
    case TASK_ACTION_STRAIGHT:
        if (task->segment_ticks >= task->target_ticks) {
            Task2024_advance(task, indicator);
            update_task_debug(task);
            *left_speed = 0;
            *right_speed = 0;
            return true;
        }
        task_drive_heading((int16_t) task->current.speed,
                           task->heading_hold_deg_x100,
                           left_speed, right_speed);
        break;

    case TASK_ACTION_STRAIGHT_TO_LINE:
        if (!task->line_released) {
            if (!any_black) {
                if (task->line_release_ticks < TASK_LINE_RELEASE_TICKS) {
                    task->line_release_ticks++;
                }
                if (task->line_release_ticks >= TASK_LINE_RELEASE_TICKS) {
                    task->line_released = true;
                    task->line_enter_ticks = 0;
                }
            } else {
                task->line_release_ticks = 0;
            }
        } else if (mid_black) {
            if (task->line_enter_ticks < TASK_LINE_ENTER_TICKS) {
                task->line_enter_ticks++;
            }
        } else {
            task->line_enter_ticks = 0;
        }

        if ((task->line_released &&
             (task->line_enter_ticks >= TASK_LINE_ENTER_TICKS)) ||
            (task->segment_ticks >= task->target_ticks)) {
            Task2024_advance(task, indicator);
            update_task_debug(task);
            *left_speed = 0;
            *right_speed = 0;
            return true;
        }

        task_drive_heading((int16_t) task->current.speed,
                           task->heading_hold_deg_x100 +
                               task_search_heading_offset(task),
                           left_speed, right_speed);
        break;

    case TASK_ACTION_ARC:
        if ((task->segment_ticks >= task->target_ticks) ||
            ((task->segment_ticks >= (task->target_ticks / 2L)) && !line_state->line_seen)) {
            Task2024_advance(task, indicator);
            update_task_debug(task);
            *left_speed = 0;
            *right_speed = 0;
            return true;
        }
        *left_speed = line_state->left_speed;
        *right_speed = line_state->right_speed;
        break;

    case TASK_ACTION_TURN: {
        int32_t yaw_error = task->yaw_target_deg_x100 - g_mpu_yaw_deg_x100;
        int16_t turn_speed;

        if (!g_mpu_ok || (abs_i32(yaw_error) <= TASK_TURN_TOL_DEG_X100)) {
            Task2024_advance(task, indicator);
            update_task_debug(task);
            *left_speed = 0;
            *right_speed = 0;
            return true;
        }

        turn_speed = (yaw_error > 0L) ?
            (int16_t) task->current.speed : -(int16_t) task->current.speed;
        turn_speed = (int16_t) (turn_speed * TASK_TURN_SIGN);
        *left_speed = (int16_t) -turn_speed;
        *right_speed = turn_speed;
        break;
    }

    case TASK_ACTION_STOP:
        task->active = false;
        g_run_enabled = false;
        *left_speed = 0;
        *right_speed = 0;
        RunIndicator_onStop(indicator);
        break;

    default:
        task->active = false;
        g_run_enabled = false;
        *left_speed = 0;
        *right_speed = 0;
        break;
    }

    update_task_debug(task);
    return true;
}
#endif



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

static int16_t apply_motor_min_run_speed(int16_t speed)
{
    if (speed == 0) {
        return 0;
    }
    if ((speed > 0) && (speed < MOTOR_MIN_RUN_SPEED)) {
        return MOTOR_MIN_RUN_SPEED;
    }
    if ((speed < 0) && (speed > -MOTOR_MIN_RUN_SPEED)) {
        return -MOTOR_MIN_RUN_SPEED;
    }
    return speed;
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

static void reset_control_state(LineFollowState *line_state,
                                TrackFSM *track_fsm,
                                CrossDetector *cross_detector,
                                SpeedPID *left_speed_pid,
                                SpeedPID *right_speed_pid,
                                uint16_t *current_base_speed,
                                uint8_t *cross_stop_black_ticks)
{
    set_motor_debug(0, 0);
    LineFollow_init(line_state);
    TrackFSM_init(track_fsm);
    CrossDetector_init(cross_detector);
    Encoder_clearDeltas();
    SpeedPID_reset(left_speed_pid);
    SpeedPID_reset(right_speed_pid);
    reset_speed_debug();

    *current_base_speed = track_fsm->base_speed;
    g_fsm_base_speed = *current_base_speed;
    g_cross_count = 0;
    g_cross_latched = false;
    g_cross_event = false;
    g_cross_candidate = false;
    g_cross_stop_black_ticks = 0;
    *cross_stop_black_ticks = 0;
}

static void bluetooth_send_uint32(uint32_t value)
{
    char digits[10];
    uint8_t count = 0;

    if (value == 0U) {
        BluetoothUART_sendChar('0');
        return;
    }

    while ((value > 0U) && (count < sizeof(digits))) {
        digits[count] = (char) ('0' + (value % 10U));
        value /= 10U;
        count++;
    }

    while (count > 0U) {
        count--;
        BluetoothUART_sendChar(digits[count]);
    }
}

static void bluetooth_send_status(void)
{
    BluetoothUART_sendString("STAT RUN=");
    BluetoothUART_sendChar(g_run_enabled ? '1' : '0');
    BluetoothUART_sendString(" GRAY=");
    bluetooth_send_uint32(g_gray_bits);
    BluetoothUART_sendString(" TRACK=");
    bluetooth_send_uint32(g_track_state);
    BluetoothUART_sendString(" CROSS=");
    bluetooth_send_uint32(g_cross_count);
    BluetoothUART_sendString(" LOST=");
    BluetoothUART_sendChar(g_line_seen ? '0' : '1');
    BluetoothUART_sendString(" STOP=");
    BluetoothUART_sendChar(g_cross_stop_test_done ? '1' : '0');
    BluetoothUART_sendString(" MPU=");
    BluetoothUART_sendChar(g_mpu_ok ? '1' : '0');
    BluetoothUART_sendString(" ADDR=");
    bluetooth_send_uint32(g_mpu_addr);
    BluetoothUART_sendString(" WHO=");
    bluetooth_send_uint32(g_mpu_who_am_i);
    BluetoothUART_sendString(" YAW=");
    if (g_mpu_yaw_deg_x100 < 0L) {
        BluetoothUART_sendChar('-');
        bluetooth_send_uint32((uint32_t) (-g_mpu_yaw_deg_x100 / 100L));
    } else {
        bluetooth_send_uint32((uint32_t) (g_mpu_yaw_deg_x100 / 100L));
    }
    BluetoothUART_sendString(" TASK=");
    bluetooth_send_uint32(g_task2024_id);
    BluetoothUART_sendString(" STEP=");
    bluetooth_send_uint32(g_task2024_step);
    BluetoothUART_sendString(" SEARCH=");
    BluetoothUART_sendChar(g_task2024_search_active ? '1' : '0');
    BluetoothUART_sendLine("");
}

static void bluetooth_send_heartbeat_if_needed(void)
{
#if BT_HEARTBEAT_TEST_ENABLE
    static uint32_t last_heartbeat_ticks = 0;

    if (!g_run_enabled && ((g_sys_ticks - last_heartbeat_ticks) >= 100U)) {
        last_heartbeat_ticks = g_sys_ticks;
        BluetoothUART_sendString("BT TICK ");
        bluetooth_send_uint32(g_sys_ticks);
        BluetoothUART_sendLine("");
    }
#endif
}

static void handle_bluetooth_command(BluetoothCommand cmd,
                                     LineFollowState *line_state,
                                     TrackFSM *track_fsm,
                                     CrossDetector *cross_detector,
                                     SpeedPID *left_speed_pid,
                                     SpeedPID *right_speed_pid,
                                     RunIndicator *run_indicator,
                                     uint16_t *current_base_speed,
                                     uint8_t *cross_stop_black_ticks)
{
    switch (cmd) {
    case BT_CMD_GO:
        reset_control_state(line_state, track_fsm, cross_detector,
                            left_speed_pid, right_speed_pid,
                            current_base_speed, cross_stop_black_ticks);
        g_run_enabled = true;
        g_cross_stop_test_done = false;
        RunIndicator_onStart(run_indicator);
        BluetoothUART_sendLine("OK GO");
        break;

    case BT_CMD_STOP:
        g_run_enabled = false;
        reset_control_state(line_state, track_fsm, cross_detector,
                            left_speed_pid, right_speed_pid,
                            current_base_speed, cross_stop_black_ticks);
        RunIndicator_onStop(run_indicator);
        BluetoothUART_sendLine("OK STOP");
        break;

    case BT_CMD_QUERY:
        bluetooth_send_status();
        break;

    case BT_CMD_LEFT:
        BluetoothUART_sendLine("OK LEFT");
        break;

    case BT_CMD_RIGHT:
        BluetoothUART_sendLine("OK RIGHT");
        break;

    case BT_CMD_MODE:
        BluetoothUART_sendLine("OK MODE");
        break;

    case BT_CMD_NONE:
        BluetoothUART_sendLine("OK");
        break;

    default:
        BluetoothUART_sendLine("ERR");
        break;
    }
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

    left_pwm = apply_motor_min_run_speed(left_pwm);
    right_pwm = apply_motor_min_run_speed(right_pwm);
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

static void run_mpu_diagnostic(void)
{
    LineFollowState line_state;
    MPU6050RawData raw;
    bool imu_ok;
    uint8_t who_am_i = 0;
    uint8_t imu_sample_divider = 0;

    SysTick->CTRL = 0U;
    g_ctrl_flag = false;
    LineFollow_init(&line_state);
    set_motor_debug(0, 0);
    g_mpu_addr = MPU6050_I2C_ADDR_AD0_LOW;
    g_mpu_init_attempts++;
    (void) MPU6050_readWhoAmI(&who_am_i);
    g_mpu_who_am_i = who_am_i;
    g_mpu6050_who_am_i = who_am_i;

    imu_ok = MPU6050_init();
    g_mpu_init_ok_once = imu_ok;
    g_mpu_ok = imu_ok;
    g_imu_ok = g_mpu_ok;
    if (!imu_ok) {
        Board_setBuzzer(true);
    }

    while (1) {
        uint8_t sensor_bits = Board_readGray5();

        if (imu_ok) {
            imu_sample_divider++;
            if (imu_sample_divider >= 10U) {
                imu_sample_divider = 0U;
                imu_ok = MPU6050_readRaw(&raw);
                g_mpu_raw_read_ok = imu_ok;
                g_mpu_ok = imu_ok;
                g_imu_ok = imu_ok;

                if (imu_ok) {
                    g_mpu_read_ok_count++;
                    g_imu_read_count++;
                    g_mpu_accel_x = raw.accel_x;
                    g_mpu_accel_y = raw.accel_y;
                    g_mpu_accel_z = raw.accel_z;
                    g_mpu_gyro_x = raw.gyro_x;
                    g_mpu_gyro_y = raw.gyro_y;
                    g_mpu_gyro_z = raw.gyro_z;
                    g_imu_accel_x = raw.accel_x;
                    g_imu_accel_y = raw.accel_y;
                    g_imu_accel_z = raw.accel_z;
                    g_imu_gyro_x = raw.gyro_x;
                    g_imu_gyro_y = raw.gyro_y;
                    g_imu_gyro_z = raw.gyro_z;
                } else {
                    g_mpu_read_fail_count++;
                    g_imu_read_fail_count++;
                }
            }
        }

        LineFollow_update(&line_state, &g_line_config, sensor_bits, g_line_config.base_speed);
        update_gray_debug(sensor_bits);
        set_motor_debug(0, 0);
        Board_delayMs(1);
    }
}

int main(void)
{
    Board_init();
    OLED_Debug_init();
    OLED_Debug_update(0U, false, 0U);

#if MPU_DIAGNOSTIC_MODE
    run_mpu_diagnostic();
#elif ENCODER_DIAGNOSTIC_MODE
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
#if TASK2024_ENABLE
    Task2024State task2024;
#endif
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
#if TASK2024_ENABLE
    Task2024_reset(&task2024);
    imu_init_debug();
#endif
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
#if TASK2024_ENABLE
        if (task2024.active) {
            imu_update_heading();
        }
#endif
        if (BluetoothUART_getLine(&bt_uart, bt_line, (uint8_t) sizeof(bt_line))) {
            BluetoothCommand bt_cmd = BluetoothUART_getCommand(&bt_uart);
            update_bluetooth_debug(&bt_uart, bt_line, bt_cmd, true);
            handle_bluetooth_command(bt_cmd, &line_state, &track_fsm,
                                     &cross_detector, &left_speed_pid,
                                     &right_speed_pid, &run_indicator,
                                     &current_base_speed,
                                     &cross_stop_black_ticks);
        } else {
            update_bluetooth_debug(&bt_uart, bt_line, BluetoothUART_getCommand(&bt_uart), false);
        }
        bluetooth_send_heartbeat_if_needed();

#if TASK2024_ENABLE
        for (uint8_t key_id = 1U; key_id <= 4U; key_id++) {
            if (task_key_pressed_event(key_id)) {
                reset_control_state(&line_state, &track_fsm, &cross_detector,
                                    &left_speed_pid, &right_speed_pid,
                                    &current_base_speed, &cross_stop_black_ticks);
                if (key_id == 2U) {
                    Task2024_reset(&task2024);
                    h_task2_start();
                } else {
                    g_h_task2_active = false;
                    Task2024_start(&task2024, key_id);
                }
                g_run_enabled = true;
                g_cross_stop_test_done = false;
                RunIndicator_onStart(&run_indicator);
                break;
            }
        }
#else
        if (button_pressed_event()) {
            g_run_enabled = !g_run_enabled;
            reset_control_state(&line_state, &track_fsm, &cross_detector,
                                &left_speed_pid, &right_speed_pid,
                                &current_base_speed, &cross_stop_black_ticks);

            if (g_run_enabled) {
                g_cross_stop_test_done = false;
                RunIndicator_onStart(&run_indicator);
            } else {
                RunIndicator_onStop(&run_indicator);
            }
        }
#endif

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
        if (g_run_enabled
#if TASK2024_ENABLE
            && !task2024.active
            && !g_h_task2_active
#endif
            && ((sensor_bits & 0x1FU) == 0x1FU)) {
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
#if TASK2024_ENABLE
            Task2024_reset(&task2024);
#endif
            g_line_left_speed = 0;
            g_line_right_speed = 0;
            set_motor_debug(0, 0);
            RunIndicator_update(&run_indicator, false);
            if ((g_sys_ticks % OLED_UPDATE_TICKS) == 0U) {
                OLED_Debug_update(h_task2_display_ticks(), false, sensor_bits);
            }
            continue;
        }

#if TASK2024_ENABLE
        if (g_h_task2_active && h_task2_should_stop(sensor_bits)) {
            h_task2_stop();
            set_motor_debug(0, 0);
            SpeedPID_reset(&left_speed_pid);
            SpeedPID_reset(&right_speed_pid);
            reset_speed_debug();
            RunIndicator_onStop(&run_indicator);
            RunIndicator_update(&run_indicator, false);
            OLED_Debug_update(h_task2_display_ticks(), false, sensor_bits);
            continue;
        }
#endif

#if TASK2024_ENABLE
        {
            int16_t task_left_speed = 0;
            int16_t task_right_speed = 0;

            if (Task2024_update(&task2024, &line_state, sensor_bits, &run_indicator,
                                &task_left_speed, &task_right_speed)) {
                set_motor_speed_closed_loop(&left_speed_pid, &right_speed_pid,
                                            task_left_speed,
                                            task_right_speed);
                RunIndicator_update(&run_indicator, g_run_enabled);
                continue;
            }
        }
#endif

        set_motor_speed_closed_loop(&left_speed_pid, &right_speed_pid,
                                    line_state.left_speed,
                                    line_state.right_speed);
        RunIndicator_update(&run_indicator, true);
    }
#endif
}







