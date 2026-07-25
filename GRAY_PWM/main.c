#include "board_port.h"
#include "line_follow.h"
#include "mpu6050.h"

static const LineFollowConfig g_line_config = {
    .base_speed = 420,
    .max_speed = 850,
    .kp_q10 = 164,
    .ki_q10 = 8,
    .kd_q10 = 225,
    .lost_turn_speed = 320,
};

volatile bool g_imu_ok = false;
volatile uint8_t g_mpu6050_who_am_i = 0;
volatile int16_t g_imu_accel_x = 0;
volatile int16_t g_imu_accel_y = 0;
volatile int16_t g_imu_accel_z = 0;
volatile int16_t g_imu_gyro_x = 0;
volatile int16_t g_imu_gyro_y = 0;
volatile int16_t g_imu_gyro_z = 0;
volatile uint32_t g_imu_read_count = 0;
volatile uint32_t g_imu_read_fail_count = 0;
volatile uint8_t g_gray_bits = 0;
volatile bool g_gray_l2 = false;
volatile bool g_gray_l1 = false;
volatile bool g_gray_c = false;
volatile bool g_gray_r1 = false;
volatile bool g_gray_r2 = false;
volatile bool g_line_seen = false;
volatile bool g_line_sensor_valid = false;
volatile int16_t g_line_position = 0;
volatile int16_t g_line_error = 0;
volatile int16_t g_line_left_speed = 0;
volatile int16_t g_line_right_speed = 0;

static void update_imu_debug_values(const MPU6050RawData *imu_raw)
{
    g_imu_accel_x = imu_raw->accel_x;
    g_imu_accel_y = imu_raw->accel_y;
    g_imu_accel_z = imu_raw->accel_z;
    g_imu_gyro_x = imu_raw->gyro_x;
    g_imu_gyro_y = imu_raw->gyro_y;
    g_imu_gyro_z = imu_raw->gyro_z;
}

static void update_line_debug_values(const LineFollowState *line_state)
{
    g_gray_bits = line_state->raw_sensor_bits;
    g_gray_l2 = (line_state->raw_sensor_bits & (1U << 0)) != 0U;
    g_gray_l1 = (line_state->raw_sensor_bits & (1U << 1)) != 0U;
    g_gray_c = (line_state->raw_sensor_bits & (1U << 2)) != 0U;
    g_gray_r1 = (line_state->raw_sensor_bits & (1U << 3)) != 0U;
    g_gray_r2 = (line_state->raw_sensor_bits & (1U << 4)) != 0U;
    g_line_seen = line_state->line_seen;
    g_line_sensor_valid = line_state->sensor_valid;
    g_line_position = line_state->position;
    g_line_error = line_state->error;
    g_line_left_speed = line_state->left_speed;
    g_line_right_speed = line_state->right_speed;
}

int main(void)
{
    LineFollowState line_state;
    MPU6050RawData imu_raw;
    bool imu_ok;
    uint8_t who_am_i = 0;
    uint8_t imu_sample_divider = 0;

    Board_init();
    LineFollow_init(&line_state);
    (void) MPU6050_readWhoAmI(&who_am_i);
    g_mpu6050_who_am_i = who_am_i;
    imu_ok = MPU6050_init();
    g_imu_ok = imu_ok;
    if (!imu_ok) {
        Board_setBuzzer(true);
    }

    while (1) {
        uint8_t sensor_bits = Board_readGray5();

        if (imu_ok) {
            imu_sample_divider++;
            if (imu_sample_divider >= 10U) {
                imu_sample_divider = 0;
                imu_ok = MPU6050_readRaw(&imu_raw);
                g_imu_ok = imu_ok;
                if (imu_ok) {
                    update_imu_debug_values(&imu_raw);
                    g_imu_read_count++;
                } else {
                    g_imu_read_fail_count++;
                }
            }
        }

        LineFollow_update(&line_state, &g_line_config, sensor_bits);
        update_line_debug_values(&line_state);
        Board_setMotorSpeed(line_state.left_speed, line_state.right_speed);

        Board_delayMs(1);
    }
}
