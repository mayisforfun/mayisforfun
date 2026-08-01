#include "board_port.h"
#include "line_follow.h"
#include "track_fsm.h"
#include "encoder.h"
#include "speed_control.h"
#include "cross_detector.h"
#include "run_indicator.h"
#include "bluetooth_uart.h"
#include "uart.h"
#include "mpu6050.h"
#include "oled_debug.h"
#include "servo_control.h"
#include "ti_ball_control/ti_ball_control.h"

/* 速度闭环开关和单轮速度 PID 参数。
 * Q10 表示真实增益 = 参数值 / 1024。速度环只做温和修正：
 * SPEED_PID_MAX_OUTPUT 过大会和循迹转向抢控制权，导致小车摆头。
 */
#define SPEED_CLOSED_LOOP_ENABLE 1
#define SPEED_TARGET_TICKS_PER_1000 30
#define SPEED_PID_KP_Q10      90
#define SPEED_PID_KI_Q10      0
#define SPEED_PID_KD_Q10      0
#define SPEED_PID_MAX_OUTPUT  25

/* 板级电机校准参数。
 * MOTOR_FORWARD_SIGN 用来修正电机正反方向。
 * MOTOR_OUTPUT_SWAP 用来修正左右通道接反。
 * 最小运行速度用于避免 PWM 太小时电机只响不动。
 */
#define MOTOR_FORWARD_SIGN    (-1)
#define MOTOR_OUTPUT_SWAP     1
#define MOTOR_MIN_RUN_SPEED   200
#define MOTOR_MIN_RUN_SPEED_SLOW 180

/* 按键消抖：按键必须连续按下这么多个控制周期，
 * 才会被认为是一次有效按键事件。
 */
#define BUTTON_DEBOUNCE_TICKS 3U

/* 2024 风格任务参数。TASK_TICKS_PER_CM 必须在真实场地上标定，
 * 它会受到地面、电池电压、轮胎和编码器安装的影响。
 */
#define TASK2024_ENABLE       1
#define TASK_TICKS_PER_CM     30L
#define TASK1_LINE_CM         130
#define TASK1_LINE_SPEED      380
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

/* H 题停车线图案。
 * 实际停车黑线大约只有 5 cm，所以不能死等五路全黑。
 * 这里接受中心附近任意三个连续黑点，提高短线识别成功率。
 */
#define H_TASK2_STOP_LEFT3    ((1U << 0) | (1U << 1) | (1U << 2))
#define H_TASK2_STOP_MID3     ((1U << 1) | (1U << 2) | (1U << 3))
#define H_TASK2_STOP_RIGHT3   ((1U << 2) | (1U << 3) | (1U << 4))

/* H 题末端慢速区。编码器估算距离到达这里后降低基础速度，
 * 让 5 cm 短黑线更容易被稳定识别。
 */
#define H_TASK2_SLOW_START_CM 300L
#define H_TASK2_SLOW_SPEED    220U
#define H_TASK2_FORCE_STOP_CM 352L
#define H_TASK2_SLOW_SETTLE_TICKS 25U
#define H_TASK2_STOP_TICKS    2U
#define H_TASK2_RELEASE_TICKS 10U
#define OLED_UPDATE_TICKS     10U
#define K230_FRESH_TIMEOUT_TICKS 50U

/* 小球水管舵机基础参数。
 *
 * 这组参数是“水管姿态控制”的机械安全层，不是小球位置 PID 本身。
 * 后面视觉识别到小球位置后，PID 只负责算出一个角度偏移量，再调用：
 *   ServoControl_setOffsetDegX10(&ball_servo, pid_output_x10);
 *
 * 调试顺序建议：
 *   1. 先只让舵机保持中位，调 BALL_SERVO_CENTER_ANGLE_X10 或
 *      BALL_SERVO_CENTER_PULSE_US，让水管尽量水平。
 *   2. 再确认方向：如果正脉宽偏移使小球运动方向反了，修改
 *      BALL_SERVO_SIGN，不要反着改位置—速度控制公式。
 *   3. 最后再调 Kp/Kd 和超限回中的速度轨迹参数。
 *
 * 角度单位是“度 * 10”：
 *   600  = 60.0 度
 *   900  = 90.0 度
 *   1200 = 120.0 度
 *
 * 脉宽单位是 us：
 *   1000us 通常接近一端
 *   1500us 通常接近中位
 *   2000us 通常接近另一端
 *
 * BALL_SERVO_MAX_STEP_US 是每 10ms 控制周期允许变化的最大脉宽。
 * 它越大，舵机响应越快；它越小，水管动作越柔，但跟踪会更慢。
 *
 * 当前水平点暂定为1900us。新状态反馈算法不再走大角度插值，而是在
 * 1900us两侧直接限制为相同的微小脉宽偏移；1000/2000只保留为底层硬限幅。
 */
#define BALL_SERVO_MIN_ANGLE_X10     600
#define BALL_SERVO_CENTER_ANGLE_X10  900
#define BALL_SERVO_MAX_ANGLE_X10     1200
#define BALL_SERVO_MIN_PULSE_US      1000U
#define BALL_SERVO_CENTER_PULSE_US   1900U
#define BALL_SERVO_MAX_PULSE_US      2000U
#define BALL_SERVO_MAX_STEP_US       1U

/* K230/水管几何标定参数。
 * PIPE_MIDDLE_PIXEL必须是水管物理中点在1920宽图像里的center_x，不是
 * KEY1按下时的小球初始位置。默认960只是画面中心，现场应以实测值替换。
 */
#define BALL_PIPE_MIDDLE_PIXEL       960.0f
#define BALL_PIXELS_PER_CM           20.0f
#define BALL_SOFT_LIMIT_CM           10.0f
#define BALL_MAX_PULSE_OFFSET_US     20U
#define BALL_POSITION_SIGN           1
#define BALL_SERVO_SIGN              1

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
volatile uint32_t g_bt_rx_count = 0;
volatile uint32_t g_bt_line_count = 0;
volatile uint32_t g_bt_overflow_count = 0;
volatile uint8_t  g_bt_last_cmd = BT_CMD_NONE;
volatile bool     g_bt_line_event = false;
volatile char     g_bt_last_line[BT_UART_LINE_SIZE] = {0};

/* K230 视觉通信调试变量。
 *
 * K230 通过 UART1 把小球中心坐标发给 MSPM0。
 * 这些变量不是 PID 参数，而是“通信层状态”，主要给 CCS 表达式窗口调试用。
 *
 * 变量含义：
 *   g_k230_position_valid:
 *     上电后是否至少收到过一帧正确坐标。收到过就是 true。
 *
 *   g_k230_position_fresh:
 *     最近一小段时间内是否收到过新坐标。后续舵机 PID 应该优先看这个量。
 *     如果它为 false，说明视觉坐标已经超时，舵机不要继续追旧坐标。
 *
 *   g_k230_frame_event:
 *     当前控制周期内是否刚刚收到新的一帧。主循环更新调试量后会清掉它。
 *
 *   g_k230_center_x / g_k230_center_y:
 *     最近一次成功解析出的视觉坐标。
 *     水管控球时，可以根据相机安装方向选择其中一个作为小球位置反馈量。
 *
 *   g_k230_rx_count:
 *     UART1 累计收到的原始字节数量。
 *     如果它不增加，说明 MSPM0 的 PA9 没有收到 K230 发来的电平。
 *
 *   g_k230_frame_count:
 *     成功解析出的完整坐标帧数量。
 *     如果 rx_count 增加但 frame_count 不增加，说明字节到了，但协议格式不匹配。
 *
 *   g_k230_bad_frame_count:
 *     帧头、帧尾不匹配，或者 UART 收到错误时累计加一。
 *
 *   g_k230_raw_last8:
 *     最近 8 个原始字节，按真实接收顺序排列。
 *     正常固定测试帧应该显示 170,170,1,2,3,4,255,255。
 *     如果 CCS 表达式窗口不好展开数组，也可以直接看 g_k230_raw0~g_k230_raw7。
 */
volatile bool     g_k230_position_valid = false;
volatile bool     g_k230_position_fresh = false;
volatile bool     g_k230_frame_event = false;
volatile uint16_t g_k230_center_x = 0;
volatile uint16_t g_k230_center_y = 0;
volatile uint32_t g_k230_rx_count = 0;
volatile uint32_t g_k230_frame_count = 0;
volatile uint32_t g_k230_bad_frame_count = 0;
volatile uint32_t g_k230_last_frame_ticks = 0;
volatile uint8_t  g_k230_last_byte = 0;
volatile uint8_t  g_k230_parse_state = K230_PARSE_WAIT_HEAD1;
volatile uint8_t  g_k230_raw_index = 0;
volatile uint8_t  g_k230_raw_bytes[K230_UART_RAW_DEBUG_SIZE] = {0};
volatile uint8_t  g_k230_raw_last8[K230_UART_RAW_DEBUG_SIZE] = {0};
volatile uint8_t  g_k230_raw0 = 0;
volatile uint8_t  g_k230_raw1 = 0;
volatile uint8_t  g_k230_raw2 = 0;
volatile uint8_t  g_k230_raw3 = 0;
volatile uint8_t  g_k230_raw4 = 0;
volatile uint8_t  g_k230_raw5 = 0;
volatile uint8_t  g_k230_raw6 = 0;
volatile uint8_t  g_k230_raw7 = 0;
volatile uint8_t  g_k230_frame0 = 0;
volatile uint8_t  g_k230_frame1 = 0;
volatile uint8_t  g_k230_frame2 = 0;
volatile uint8_t  g_k230_frame3 = 0;
volatile uint8_t  g_k230_frame4 = 0;
volatile uint8_t  g_k230_frame5 = 0;
volatile uint8_t  g_k230_frame6 = 0;
volatile uint8_t  g_k230_frame7 = 0;
volatile uint8_t  g_k230_sync0 = 0;
volatile uint8_t  g_k230_sync1 = 0;
volatile uint8_t  g_k230_sync2 = 0;
volatile uint8_t  g_k230_sync3 = 0;
volatile uint8_t  g_k230_sync4 = 0;
volatile uint8_t  g_k230_sync5 = 0;
volatile uint8_t  g_k230_sync6 = 0;
volatile uint8_t  g_k230_sync7 = 0;
volatile uint8_t  g_k230_sync_count = 0;
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
volatile bool     g_h_task2_stop_armed = false;
volatile int32_t  g_h_task2_travel_ticks = 0;
volatile int32_t  g_h_task2_slow_start_ticks = 0;
volatile bool     g_h_task2_finish_candidate = false;
volatile bool     g_h_task2_slow_mode = false;
volatile uint8_t  g_h_task2_slow_ticks = 0;
volatile int16_t  g_ball_servo_angle_x10 = BALL_SERVO_CENTER_ANGLE_X10;
volatile uint16_t g_ball_servo_pulse_us = BALL_SERVO_CENTER_PULSE_US;
volatile uint8_t  g_ball_task = TI_BALL_TASK_IDLE;
volatile bool     g_ball_origin_valid = false;
volatile bool     g_ball_vision_valid = false;
volatile float    g_ball_position_cm = 0.0f;
volatile float    g_ball_target_cm = 0.0f;
volatile float    g_ball_velocity_cm_s = 0.0f;
volatile float    g_ball_physical_position_cm = 0.0f;
volatile float    g_ball_predicted_position_cm = 0.0f;
volatile float    g_ball_desired_velocity_cm_s = 0.0f;
volatile int16_t  g_ball_pulse_offset_us = 0;
volatile bool     g_ball_return_active = false;
volatile bool     g_ball_return_settled = false;
volatile uint8_t  g_ball_task1_phase = 0U;

/* 水管舵机的机械层配置。
 * PID不直接使用这些脉宽；它输出相对中心角的倾角，再由servo_control.c
 * 依据这里的min/center/max换算为实际PWM。
 */
static const ServoControlConfig g_ball_servo_config = {
    .min_angle_x10    = BALL_SERVO_MIN_ANGLE_X10,
    .center_angle_x10 = BALL_SERVO_CENTER_ANGLE_X10,
    .max_angle_x10    = BALL_SERVO_MAX_ANGLE_X10,
    .min_pulse_us     = BALL_SERVO_MIN_PULSE_US,
    .center_pulse_us  = BALL_SERVO_CENTER_PULSE_US,
    .max_pulse_us     = BALL_SERVO_MAX_PULSE_US,
    .max_step_us      = BALL_SERVO_MAX_STEP_US,
    .invert           = false,
};

/* 主循迹参数。
 * base_speed 是直线基础速度，max_speed 是双轮速度上限。
 * Kp 决定转向力度，Kd 用来压住左右摆头。
 * Ki 保持为 0，因为灰度循迹通常不希望长期积累转向误差。
 */
static LineFollowConfig g_line_config = {
    .base_speed      = 380,
    .max_speed       = 560,
    .kp_q10          = 80,
    .ki_q10          = 0,
    .kd_q10          = 120,
    .lost_turn_speed = 180,
};

static int16_t clamp_i16(int32_t value, int16_t min_value, int16_t max_value);
static int32_t abs_i32(int32_t value);

static uint8_t count_black_sensors5(uint8_t bits)
{
    uint8_t count = 0;

    bits &= 0x1FU;
    for (uint8_t bit = 0; bit < 5U; bit++) {
        if ((bits & (1U << bit)) != 0U) {
            count++;
        }
    }

    return count;
}

/* 判断当前灰度图案是否像 H 题末端短停车线。
 * 停车线很窄，如果强制要求 L1+C+R1 完全同时为黑，
 * 车身一摆就会漏检。因此这里接受三路连续黑、中心加足够黑区、
 * 或者任意四路黑。
 */
static bool h_task2_is_stop_line(uint8_t sensor_bits)
{
    uint8_t bits = sensor_bits & 0x1FU;

    if (((bits & H_TASK2_STOP_LEFT3) == H_TASK2_STOP_LEFT3) ||
        ((bits & H_TASK2_STOP_MID3) == H_TASK2_STOP_MID3) ||
        ((bits & H_TASK2_STOP_RIGHT3) == H_TASK2_STOP_RIGHT3)) {
        return true;
    }

    if (((bits & (1U << 2)) != 0U) && (count_black_sensors5(bits) >= 3U)) {
        return true;
    }

    return count_black_sensors5(bits) >= 4U;
}

/* 使用左右轮绝对编码器距离的平均值。
 * 这样在转弯或单侧轻微打滑时，比只相信某一侧更稳定。
 */
static int32_t h_task2_get_travel_ticks(void)
{
    return (abs_i32(g_encoder_left_total) + abs_i32(g_encoder_right_total)) / 2L;
}

/* 只重置 H 题停车线和慢速区状态。
 * 通用循迹状态和速度 PID 状态由 reset_control_state() 统一重置。
 */
static void h_task2_reset_stop_state(void)
{
    g_h_task2_travel_ticks = 0;
    g_h_task2_slow_start_ticks = H_TASK2_SLOW_START_CM * TASK_TICKS_PER_CM;
    g_h_task2_finish_candidate = false;
    g_h_task2_slow_mode = false;
    g_h_task2_slow_ticks = 0;
    g_h_task2_stop_ticks = 0;
}

/* 启动 H 题任务 2：先从起始黑线释放出来，
 * 然后在设定行驶距离之后再开始寻找末端停车线。
 */
static void h_task2_start(void)
{
    g_h_task2_active = true;
    g_h_task2_done = false;
    g_h_task2_released = false;
    g_h_task2_start_ticks = g_sys_ticks;
    g_h_task2_elapsed_ticks = 0;
    g_h_task2_release_ticks = 0;
    g_h_task2_stop_armed = false;
    h_task2_reset_stop_state();
}

/* 停止 H 题任务 2，并记录耗时，供 OLED/调试显示使用。 */
static void h_task2_stop(void)
{
    g_h_task2_active = false;
    g_h_task2_done = true;
    g_h_task2_released = false;
    g_h_task2_elapsed_ticks = g_sys_ticks - g_h_task2_start_ticks;
    g_h_task2_release_ticks = 0;
    g_run_enabled = false;
}

/* 短停车线识别状态机：
 *   1. 起步时先忽略初始黑线，直到小车完全离开它。
 *   2. 到达慢速区距离前正常行驶。
 *   3. 进入慢速区后降速，并等待几个周期让车身稳定。
 *   4. 停车图案必须连续出现若干周期，才真正停车。
 */
static bool h_task2_should_stop(uint8_t sensor_bits)
{
    bool stop_line_seen = h_task2_is_stop_line(sensor_bits);

    g_h_task2_travel_ticks = h_task2_get_travel_ticks();
    g_h_task2_slow_start_ticks = H_TASK2_SLOW_START_CM * TASK_TICKS_PER_CM;

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
        g_h_task2_stop_armed = false;
        g_h_task2_stop_ticks = 0;
        g_h_task2_finish_candidate = false;
        return false;
    }

    g_h_task2_stop_armed =
        g_h_task2_travel_ticks >= g_h_task2_slow_start_ticks;

    if (!g_h_task2_stop_armed) {
        g_h_task2_slow_mode = false;
        g_h_task2_slow_ticks = 0;
        g_h_task2_stop_ticks = 0;
        g_h_task2_finish_candidate = false;
        return false;
    }

    g_h_task2_slow_mode = true;
    if (g_h_task2_slow_ticks < H_TASK2_SLOW_SETTLE_TICKS) {
        g_h_task2_slow_ticks++;
    }

    /* 灰度终点线较短且每次覆盖的探头数量不稳定。
     * 编码器比例虽然不准确，但实测代码距离约 340 时已接近终点，
     * 因此在完成末段稳定等待后用它作为停车兜底。
     */
    if ((g_h_task2_slow_ticks >= H_TASK2_SLOW_SETTLE_TICKS) &&
        (g_h_task2_travel_ticks >=
         (H_TASK2_FORCE_STOP_CM * TASK_TICKS_PER_CM))) {
        return true;
    }

    g_h_task2_finish_candidate =
        (g_h_task2_slow_ticks >= H_TASK2_SLOW_SETTLE_TICKS) && stop_line_seen;

    if (g_h_task2_finish_candidate) {
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

/* 简单串口调试量同步。
 * 现在 uart.c 只保留最基础的“收 1 个字节、存最近 8 个字节”。
 * 为了不让 CCS 表达式窗口里的旧变量全部失效，这里继续把原始串口数据
 * 映射到 g_k230_rx_count / g_k230_raw0~7 / g_k230_last_byte 这些旧名字上。
 */
static void update_k230_debug(void)
{
    static uint32_t previous_frame_count = 0U;

    g_k230_rx_count = g_uart_rx_count;
    g_k230_last_byte = g_uart_rx_data;
    g_k230_raw_index = g_uart_rx_index;
    g_k230_frame_event = (g_k230_frame_count != previous_frame_count);
    previous_frame_count = g_k230_frame_count;

    if (g_k230_position_valid &&
        ((uint32_t) (g_sys_ticks - g_k230_last_frame_ticks) >
         K230_FRESH_TIMEOUT_TICKS)) {
        g_k230_position_fresh = false;
    }

    for (uint8_t i = 0U; i < K230_UART_RAW_DEBUG_SIZE; i++) {
        uint8_t ordered_index = (uint8_t) (g_uart_rx_index + i);
        if (ordered_index >= K230_UART_RAW_DEBUG_SIZE) {
            ordered_index = (uint8_t) (ordered_index - K230_UART_RAW_DEBUG_SIZE);
        }

        g_k230_raw_bytes[i] = g_uart_rx_buffer[i];
        g_k230_raw_last8[i] = g_uart_rx_buffer[ordered_index];
    }

    g_k230_raw0 = g_k230_raw_last8[0];
    g_k230_raw1 = g_k230_raw_last8[1];
    g_k230_raw2 = g_k230_raw_last8[2];
    g_k230_raw3 = g_k230_raw_last8[3];
    g_k230_raw4 = g_k230_raw_last8[4];
    g_k230_raw5 = g_k230_raw_last8[5];
    g_k230_raw6 = g_k230_raw_last8[6];
    g_k230_raw7 = g_k230_raw_last8[7];
}

/* Copy one coherent center_x sample from data owned by the UART ISR. */
static bool take_new_ball_center_x(uint32_t *last_frame_count,
                                   uint16_t *center_x)
{
    uint32_t primask = __get_PRIMASK();
    uint32_t frame_count;
    bool is_new;

    __disable_irq();
    frame_count = g_k230_frame_count;
    is_new = frame_count != *last_frame_count;
    if (is_new) {
        *center_x = g_k230_center_x;
        *last_frame_count = frame_count;
    }
    if (primask == 0U) {
        __enable_irq();
    }

    return is_new;
}

static void update_ball_control_debug(const TIBallControl *control)
{
    g_ball_task = (uint8_t) control->task;
    g_ball_origin_valid = control->origin_valid;
    g_ball_vision_valid = control->vision_valid;
    g_ball_position_cm = control->position_cm;
    g_ball_target_cm = control->target_cm;
    g_ball_velocity_cm_s = control->velocity_cm_s;
    g_ball_physical_position_cm = control->physical_position_cm;
    g_ball_predicted_position_cm = control->predicted_position_cm;
    g_ball_desired_velocity_cm_s = control->desired_velocity_cm_s;
    g_ball_pulse_offset_us = control->output_pulse_offset_us;
    g_ball_return_active = control->return_active;
    g_ball_return_settled = control->return_settled;
    g_ball_task1_phase = control->task1_phase;
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

#if !TASK2024_ENABLE
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
#endif

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
    {TASK_ACTION_STRAIGHT_TO_LINE, TASK1_LINE_CM, TASK1_LINE_SPEED, true},
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

    /* 控制周期是 10 ms。mdps * 10ms / 10000 = 角度 * 100。 */
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

/* 把每个控制周期的编码器 tick 换算成近似 cm/s，用于调试显示。
 * 它共用 TASK_TICKS_PER_CM，所以重新标定距离后这个显示也会更准。
 */
static int16_t speed_ticks_to_cms(int16_t ticks_per_loop)
{
    return (int16_t) (((int32_t) ticks_per_loop * 100L) / TASK_TICKS_PER_CM);
}

/* 在 PID 修正后套用最小运行 PWM。
 * 低于这个值时电机可能只响不动，会破坏编码器反馈和直线校准。
 */
static int16_t apply_motor_min_run_speed(int16_t speed)
{
    int16_t min_run_speed = g_h_task2_slow_mode ?
        MOTOR_MIN_RUN_SPEED_SLOW : MOTOR_MIN_RUN_SPEED;

    if (speed == 0) {
        return 0;
    }
    if ((speed > 0) && (speed < min_run_speed)) {
        return min_run_speed;
    }
    if ((speed < 0) && (speed > -min_run_speed)) {
        return -min_run_speed;
    }
    return speed;
}

/* 清空 OLED 和蓝牙状态里显示的编码器/速度调试量。 */
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

/* 在 GO、STOP 或任务切换前，把所有控制器恢复到确定状态。
 * 这样旧的 PID 积分、横线锁存和编码器累计值不会影响下一次运行。
 */
static void reset_control_state(LineFollowState *line_state,
                                TrackFSM *track_fsm,
                                CrossDetector *cross_detector,
                                SpeedPID *left_speed_pid,
                                SpeedPID *right_speed_pid,
                                uint16_t *current_base_speed)
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
    BluetoothUART_sendString(" K230=");
    BluetoothUART_sendChar(g_k230_position_fresh ? '1' : '0');
    BluetoothUART_sendString(" KX=");
    bluetooth_send_uint32(g_k230_center_x);
    BluetoothUART_sendString(" KY=");
    bluetooth_send_uint32(g_k230_center_y);
    BluetoothUART_sendLine("");
}

static void handle_bluetooth_command(BluetoothCommand cmd,
                                     LineFollowState *line_state,
                                     TrackFSM *track_fsm,
                                     CrossDetector *cross_detector,
                                     SpeedPID *left_speed_pid,
                                     SpeedPID *right_speed_pid,
                                     RunIndicator *run_indicator,
                                     uint16_t *current_base_speed)
{
    switch (cmd) {
    case BT_CMD_GO:
        reset_control_state(line_state, track_fsm, cross_detector,
                            left_speed_pid, right_speed_pid,
                            current_base_speed);
        g_run_enabled = true;
        RunIndicator_onStart(run_indicator);
        BluetoothUART_sendLine("OK GO");
        break;

    case BT_CMD_STOP:
        g_run_enabled = false;
        reset_control_state(line_state, track_fsm, cross_detector,
                            left_speed_pid, right_speed_pid,
                            current_base_speed);
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

    /* 先读取并累计编码器增量。
     * 循迹、H 题距离、OLED 和速度 PID 都使用同一份最新测量值。
     */
    g_encoder_left_delta = Encoder_getLeftTicks();
    g_encoder_right_delta = Encoder_getRightTicks();
    g_encoder_left_total += g_encoder_left_delta;
    g_encoder_right_total += g_encoder_right_delta;

    /* 把逻辑 PWM 目标换算成编码器 tick 目标。
     * 换算故意保持简单，方便直接在赛道上调 SPEED_TARGET_TICKS_PER_1000。
     */
    g_speed_left_actual_ticks = clamp_i16(g_encoder_left_delta, -32768, 32767);
    g_speed_right_actual_ticks = clamp_i16(g_encoder_right_delta, -32768, 32767);
    g_speed_left_target_ticks = speed_to_target_ticks(left_target);
    g_speed_right_target_ticks = speed_to_target_ticks(right_target);

#if SPEED_CLOSED_LOOP_ENABLE
    /* 只要某个轮子的目标速度为 0，就重置该轮 PID。
     * 否则历史积分会在下一次起步时突然给车一个冲击。
     */
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

    /* 最小 PWM 放在最后处理。
     * 这样 PID 仍围绕原始逻辑目标计算，而实际电机又能拿到足够启动的 PWM。
     */
    left_pwm = apply_motor_min_run_speed(left_pwm);
    right_pwm = apply_motor_min_run_speed(right_pwm);
    g_speed_left_pwm = left_pwm;
    g_speed_right_pwm = right_pwm;
    set_motor_debug(left_pwm, right_pwm);
}
int main(void)
{
    Board_init();
    OLED_Debug_init();
    OLED_Debug_update(0U, false, 0U, 0, 0, false);

    LineFollowState line_state;
    TrackFSM track_fsm;
    SpeedPID left_speed_pid;
    SpeedPID right_speed_pid;
    CrossDetector cross_detector;
    RunIndicator run_indicator;
    BluetoothUART bt_uart;
    ServoControl ball_servo;
    TIBallControl ball_control;
    TIBallControlConfig ball_control_config;
    uint32_t ball_last_frame_count = 0U;
#if TASK2024_ENABLE
    Task2024State task2024;
#endif
    char bt_line[BT_UART_LINE_SIZE];
    uint16_t current_base_speed;

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
    UART1_enableRxInterrupt();
    ServoControl_init(&ball_servo, &g_ball_servo_config);
    ball_control_config = TIBallControl_defaultConfig();
    /* 把最常现场标定的几何/方向参数集中在main.c顶部。其余状态反馈增益
     * 在ti_ball_control.c的TIBallControl_defaultConfig()中调整。
     */
    ball_control_config.pipe_middle_pixel = BALL_PIPE_MIDDLE_PIXEL;
    ball_control_config.pixels_per_cm = BALL_PIXELS_PER_CM;
    ball_control_config.soft_limit_cm = BALL_SOFT_LIMIT_CM;
    ball_control_config.max_pulse_offset_us = BALL_MAX_PULSE_OFFSET_US;
    ball_control_config.position_sign = BALL_POSITION_SIGN;
    ball_control_config.servo_sign = BALL_SERVO_SIGN;
    TIBallControl_init(&ball_control, &ball_control_config);
    update_ball_control_debug(&ball_control);
    g_ball_servo_angle_x10 = ServoControl_getCurrentAngleDegX10(&ball_servo);
    g_ball_servo_pulse_us = ServoControl_getCurrentPulseUs(&ball_servo);
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
        /* K230串口接收由中断完成，主循环不能再轮询同一个RX FIFO。
         * 否则中断和主循环会同时修改协议解析状态，造成偶发丢帧或错帧。
         */
        update_k230_debug();
        {
            uint16_t center_x;

            /* 每个K230帧只处理一次。take_new_ball_center_x会用frame_count
             * 判断是否为新帧，并在短临界区内取得一致的center_x快照。
             */
            if (take_new_ball_center_x(&ball_last_frame_count, &center_x)) {
                TIBallControl_pushVision(
                    &ball_control, center_x, g_sys_ticks);
            }
        }
#if TASK2024_ENABLE
        if (task2024.active || g_h_task2_active) {
            imu_update_heading();
        }
#endif
        if (BluetoothUART_getLine(&bt_uart, bt_line, (uint8_t) sizeof(bt_line))) {
            BluetoothCommand bt_cmd = BluetoothUART_getCommand(&bt_uart);
            update_bluetooth_debug(&bt_uart, bt_line, bt_cmd, true);
            handle_bluetooth_command(bt_cmd, &line_state, &track_fsm,
                                     &cross_detector, &left_speed_pid,
                                     &right_speed_pid, &run_indicator,
                                     &current_base_speed);
        } else {
            update_bluetooth_debug(&bt_uart, bt_line, BluetoothUART_getCommand(&bt_uart), false);
        }

        /* 舵机闭环每10ms执行一次，且放在小车g_run_enabled判断之前：
         *   1. TIBallControl_update：由小球位置算目标水管倾角；
         *   2. ServoControl_update：将目标倾角换算并缓动到实际PWM；
         *   3. 同步调试变量，方便CCS Expressions逐层定位。
         * 因此题1即使让小车保持停止，水管舵机仍会持续控制。
         */
        TIBallControl_update(&ball_control, g_sys_ticks, &ball_servo);
        ServoControl_update(&ball_servo);
        update_ball_control_debug(&ball_control);
        g_ball_servo_angle_x10 = ServoControl_getCurrentAngleDegX10(&ball_servo);
        g_ball_servo_pulse_us = ServoControl_getCurrentPulseUs(&ball_servo);

#if TASK2024_ENABLE
        for (uint8_t key_id = 1U; key_id <= 4U; key_id++) {
            if (task_key_pressed_event(key_id)) {
                reset_control_state(&line_state, &track_fsm, &cross_detector,
                                    &left_speed_pid, &right_speed_pid,
                                    &current_base_speed);
                if (key_id == 1U) {
                    /* KEY1启动小球题1：
                     *   - 小车电机保持停止；
                     *   - 丢弃按键前的旧视觉原点；
                     *   - KEY1后的第一帧center_x定义为0cm；
                     *   - 目标先设为+5cm，稳定1秒后自动切到-5cm。
                     */
                    g_run_enabled = false;
                    g_h_task2_active = false;
                    Task2024_reset(&task2024);
                    TIBallControl_requestOriginCapture(&ball_control);
                    (void) TIBallControl_setTask(
                        &ball_control,
                        TI_BALL_TASK_STATIC_PLUS5_TO_MINUS5,
                        g_sys_ticks);
                    RunIndicator_onStop(&run_indicator);
                } else if (key_id == 2U) {
                    (void) TIBallControl_setTask(
                        &ball_control, TI_BALL_TASK_IDLE, g_sys_ticks);
                    ServoControl_center(&ball_servo);
                    Task2024_reset(&task2024);
                    h_task2_start();
                    g_run_enabled = true;
                    RunIndicator_onStart(&run_indicator);
                } else {
                    (void) TIBallControl_setTask(
                        &ball_control, TI_BALL_TASK_IDLE, g_sys_ticks);
                    ServoControl_center(&ball_servo);
                    g_h_task2_active = false;
                    Task2024_start(&task2024, key_id);
                    g_run_enabled = true;
                    RunIndicator_onStart(&run_indicator);
                }
                update_ball_control_debug(&ball_control);
                break;
            }
        }
#else
        if (button_pressed_event()) {
            g_run_enabled = !g_run_enabled;
            reset_control_state(&line_state, &track_fsm, &cross_detector,
                                &left_speed_pid, &right_speed_pid,
                                &current_base_speed);

            if (g_run_enabled) {
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

        if (!g_run_enabled) {
#if TASK2024_ENABLE
            Task2024_reset(&task2024);
#endif
            g_line_left_speed = 0;
            g_line_right_speed = 0;
            set_motor_debug(0, 0);
            RunIndicator_update(&run_indicator, false);
            if ((g_sys_ticks % OLED_UPDATE_TICKS) == 0U) {
                OLED_Debug_update(h_task2_display_ticks(), false, sensor_bits,
                                  0, 0, g_h_task2_slow_mode);
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
            OLED_Debug_update(h_task2_display_ticks(), false, sensor_bits,
                              0, 0, g_h_task2_slow_mode);
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
                if ((g_sys_ticks % OLED_UPDATE_TICKS) == 0U) {
                    OLED_Debug_update(h_task2_display_ticks(), true, sensor_bits,
                                      speed_ticks_to_cms(g_speed_left_actual_ticks),
                                      speed_ticks_to_cms(g_speed_right_actual_ticks),
                                      g_h_task2_slow_mode);
                }
                continue;
            }
        }
#endif

        {
            int16_t output_left_speed = line_state.left_speed;
            int16_t output_right_speed = line_state.right_speed;

#if TASK2024_ENABLE
            if (g_h_task2_active && g_h_task2_slow_mode) {
                LineFollow_update(&line_state, &g_line_config,
                                  sensor_bits, H_TASK2_SLOW_SPEED);
                update_line_debug(&line_state);
                output_left_speed = line_state.left_speed;
                output_right_speed = line_state.right_speed;
            }
#endif

            set_motor_speed_closed_loop(&left_speed_pid, &right_speed_pid,
                                        output_left_speed,
                                        output_right_speed);
        }
        RunIndicator_update(&run_indicator, true);
        if ((g_sys_ticks % OLED_UPDATE_TICKS) == 0U) {
            OLED_Debug_update(h_task2_display_ticks(), true, sensor_bits,
                              speed_ticks_to_cms(g_speed_left_actual_ticks),
                              speed_ticks_to_cms(g_speed_right_actual_ticks),
                              g_h_task2_slow_mode);
        }
    }
}







