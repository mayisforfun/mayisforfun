#ifndef MPU6050_H_
#define MPU6050_H_

#include <stdbool.h>
#include <stdint.h>

#define MPU6050_I2C_ADDR_AD0_LOW  0x68U
#define MPU6050_I2C_ADDR_AD0_HIGH 0x69U

typedef struct {
    int16_t accel_x;
    int16_t accel_y;
    int16_t accel_z;
    int16_t temperature;
    int16_t gyro_x;
    int16_t gyro_y;
    int16_t gyro_z;
} MPU6050RawData;

typedef struct {
    int32_t accel_x_mg;
    int32_t accel_y_mg;
    int32_t accel_z_mg;
    int32_t temperature_c_x100;
    int32_t gyro_x_mdps;
    int32_t gyro_y_mdps;
    int32_t gyro_z_mdps;
} MPU6050ScaledData;

bool MPU6050_init(void);
bool MPU6050_readWhoAmI(uint8_t *who_am_i);
bool MPU6050_readRaw(MPU6050RawData *data);
void MPU6050_scaleDefault(const MPU6050RawData *raw, MPU6050ScaledData *scaled);

#endif
