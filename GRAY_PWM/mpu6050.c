#include "mpu6050.h"

#include "ti_msp_dl_config.h"

#ifndef MPU6050_I2C_INST
#if defined(I2C_0_INST)
#define MPU6050_I2C_INST I2C_0_INST
#elif defined(I2C_INST)
#define MPU6050_I2C_INST I2C_INST
#else
#define MPU6050_I2C_INST I2C_0_INST
#endif
#endif

#ifndef MPU6050_I2C_ADDRESS
#define MPU6050_I2C_ADDRESS MPU6050_I2C_ADDR_AD0_LOW
#endif

#ifndef MPU6050_I2C_TIMEOUT
#define MPU6050_I2C_TIMEOUT 100000UL
#endif

#define MPU6050_REG_SMPLRT_DIV   0x19U
#define MPU6050_REG_CONFIG       0x1AU
#define MPU6050_REG_GYRO_CONFIG  0x1BU
#define MPU6050_REG_ACCEL_CONFIG 0x1CU
#define MPU6050_REG_ACCEL_XOUT_H 0x3BU
#define MPU6050_REG_PWR_MGMT_1   0x6BU
#define MPU6050_REG_PWR_MGMT_2   0x6CU
#define MPU6050_REG_WHO_AM_I     0x75U

static bool wait_for_status(uint32_t mask, bool set)
{
    uint32_t timeout = MPU6050_I2C_TIMEOUT;

    while (timeout-- > 0UL) {
        bool status_set = (DL_I2C_getControllerStatus(MPU6050_I2C_INST) & mask) != 0UL;

        if (status_set == set) {
            return true;
        }
    }

    return false;
}

static bool i2c_has_error(void)
{
    return (DL_I2C_getControllerStatus(MPU6050_I2C_INST) &
            DL_I2C_CONTROLLER_STATUS_ERROR) != 0UL;
}

static bool i2c_write(uint8_t reg, const uint8_t *data, uint8_t length)
{
    uint8_t tx_buffer[9];

    if (length > 7U) {
        return false;
    }

    tx_buffer[0] = reg;
    for (uint8_t i = 0; i < length; i++) {
        tx_buffer[i + 1U] = data[i];
    }

    if (!wait_for_status(DL_I2C_CONTROLLER_STATUS_IDLE, true)) {
        return false;
    }

    (void) DL_I2C_fillControllerTXFIFO(MPU6050_I2C_INST, tx_buffer, (uint16_t) length + 1U);
    DL_I2C_startControllerTransfer(MPU6050_I2C_INST,
                                   MPU6050_I2C_ADDRESS,
                                   DL_I2C_CONTROLLER_DIRECTION_TX,
                                   (uint16_t) length + 1U);

    if (!wait_for_status(DL_I2C_CONTROLLER_STATUS_BUSY, false)) {
        return false;
    }

    return !i2c_has_error();
}

static bool i2c_write_u8(uint8_t reg, uint8_t value)
{
    return i2c_write(reg, &value, 1U);
}

static bool i2c_read(uint8_t reg, uint8_t *data, uint8_t length)
{
    if ((data == 0) || (length == 0U)) {
        return false;
    }

    if (!wait_for_status(DL_I2C_CONTROLLER_STATUS_IDLE, true)) {
        return false;
    }

    (void) DL_I2C_fillControllerTXFIFO(MPU6050_I2C_INST, &reg, 1U);
    DL_I2C_startControllerTransferAdvanced(MPU6050_I2C_INST,
                                           MPU6050_I2C_ADDRESS,
                                           DL_I2C_CONTROLLER_DIRECTION_TX,
                                           1U,
                                           DL_I2C_CONTROLLER_START_ENABLE,
                                           DL_I2C_CONTROLLER_STOP_DISABLE,
                                           DL_I2C_CONTROLLER_ACK_DISABLE);

    if (!wait_for_status(DL_I2C_CONTROLLER_STATUS_BUSY, false) || i2c_has_error()) {
        return false;
    }

    DL_I2C_startControllerTransferAdvanced(MPU6050_I2C_INST,
                                           MPU6050_I2C_ADDRESS,
                                           DL_I2C_CONTROLLER_DIRECTION_RX,
                                           length,
                                           DL_I2C_CONTROLLER_START_ENABLE,
                                           DL_I2C_CONTROLLER_STOP_ENABLE,
                                           DL_I2C_CONTROLLER_ACK_DISABLE);

    for (uint8_t i = 0; i < length; i++) {
        uint32_t timeout = MPU6050_I2C_TIMEOUT;

        while (DL_I2C_isControllerRXFIFOEmpty(MPU6050_I2C_INST)) {
            if (timeout-- == 0UL) {
                return false;
            }
        }

        data[i] = DL_I2C_receiveControllerData(MPU6050_I2C_INST);
    }

    if (!wait_for_status(DL_I2C_CONTROLLER_STATUS_BUSY, false)) {
        return false;
    }

    return !i2c_has_error();
}

bool MPU6050_readWhoAmI(uint8_t *who_am_i)
{
    return i2c_read(MPU6050_REG_WHO_AM_I, who_am_i, 1U);
}

bool MPU6050_init(void)
{
    uint8_t who_am_i = 0;

    if (!i2c_write_u8(MPU6050_REG_PWR_MGMT_1, 0x80U)) {
        return false;
    }
    delay_cycles(CPUCLK_FREQ / 100U);

    if (!i2c_write_u8(MPU6050_REG_PWR_MGMT_1, 0x01U)) {
        return false;
    }
    if (!i2c_write_u8(MPU6050_REG_PWR_MGMT_2, 0x00U)) {
        return false;
    }
    if (!i2c_write_u8(MPU6050_REG_SMPLRT_DIV, 0x07U)) {
        return false;
    }
    if (!i2c_write_u8(MPU6050_REG_CONFIG, 0x03U)) {
        return false;
    }
    if (!i2c_write_u8(MPU6050_REG_GYRO_CONFIG, 0x00U)) {
        return false;
    }
    if (!i2c_write_u8(MPU6050_REG_ACCEL_CONFIG, 0x00U)) {
        return false;
    }

    return MPU6050_readWhoAmI(&who_am_i) && (who_am_i == 0x68U);
}

bool MPU6050_readRaw(MPU6050RawData *data)
{
    uint8_t buffer[14];

    if (data == 0) {
        return false;
    }

    if (!i2c_read(MPU6050_REG_ACCEL_XOUT_H, buffer, sizeof(buffer))) {
        return false;
    }

    data->accel_x = (int16_t) (((uint16_t) buffer[0] << 8) | buffer[1]);
    data->accel_y = (int16_t) (((uint16_t) buffer[2] << 8) | buffer[3]);
    data->accel_z = (int16_t) (((uint16_t) buffer[4] << 8) | buffer[5]);
    data->temperature = (int16_t) (((uint16_t) buffer[6] << 8) | buffer[7]);
    data->gyro_x = (int16_t) (((uint16_t) buffer[8] << 8) | buffer[9]);
    data->gyro_y = (int16_t) (((uint16_t) buffer[10] << 8) | buffer[11]);
    data->gyro_z = (int16_t) (((uint16_t) buffer[12] << 8) | buffer[13]);

    return true;
}

void MPU6050_scaleDefault(const MPU6050RawData *raw, MPU6050ScaledData *scaled)
{
    if ((raw == 0) || (scaled == 0)) {
        return;
    }

    scaled->accel_x_mg = ((int32_t) raw->accel_x * 1000L) / 16384L;
    scaled->accel_y_mg = ((int32_t) raw->accel_y * 1000L) / 16384L;
    scaled->accel_z_mg = ((int32_t) raw->accel_z * 1000L) / 16384L;
    scaled->temperature_c_x100 = (((int32_t) raw->temperature * 100L) / 340L) + 3653L;
    scaled->gyro_x_mdps = ((int32_t) raw->gyro_x * 1000L) / 131L;
    scaled->gyro_y_mdps = ((int32_t) raw->gyro_y * 1000L) / 131L;
    scaled->gyro_z_mdps = ((int32_t) raw->gyro_z * 1000L) / 131L;
}
