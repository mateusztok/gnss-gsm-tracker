#include "mpu6050.h"
#include <string.h>

#define MPU6050_ADDRESS (0x68U << 1)
#define MPU6050_POWER_MANAGEMENT_REGISTER 0x6BU
#define MPU6050_ACCELEROMETER_REGISTER 0x3BU

static int32_t AbsoluteDifference(int16_t first, int16_t second)
{
    int32_t difference = (int32_t)first - (int32_t)second;
    return (difference < 0) ? -difference : difference;
}

HAL_StatusTypeDef MPU6050_Init(MPU6050_t *device, I2C_HandleTypeDef *i2c)
{
    uint8_t wake_up = 0U;

    if (device == NULL || i2c == NULL) {
        return HAL_ERROR;
    }

    memset(device, 0, sizeof(*device));
    device->i2c = i2c;
    return HAL_I2C_Mem_Write(device->i2c, MPU6050_ADDRESS,
                             MPU6050_POWER_MANAGEMENT_REGISTER, 1U,
                             &wake_up, 1U, 100U);
}

HAL_StatusTypeDef MPU6050_DetectMotion(MPU6050_t *device,
                                       int32_t threshold,
                                       uint8_t *movement_detected)
{
    uint8_t raw[6];
    int16_t accel_x;
    int16_t accel_y;
    int16_t accel_z;
    int32_t delta;
    HAL_StatusTypeDef status;

    if (device == NULL || device->i2c == NULL || movement_detected == NULL ||
        threshold < 0) {
        return HAL_ERROR;
    }

    status = HAL_I2C_Mem_Read(device->i2c, MPU6050_ADDRESS,
                              MPU6050_ACCELEROMETER_REGISTER, 1U,
                              raw, sizeof(raw), 100U);
    if (status != HAL_OK) {
        return status;
    }

    accel_x = (int16_t)(((uint16_t)raw[0] << 8) | raw[1]);
    accel_y = (int16_t)(((uint16_t)raw[2] << 8) | raw[3]);
    accel_z = (int16_t)(((uint16_t)raw[4] << 8) | raw[5]);
    *movement_detected = 0U;

    if (device->has_previous_sample != 0U) {
        delta = AbsoluteDifference(accel_x, device->previous_x) +
                AbsoluteDifference(accel_y, device->previous_y) +
                AbsoluteDifference(accel_z, device->previous_z);
        *movement_detected = (delta > threshold) ? 1U : 0U;
    }

    device->previous_x = accel_x;
    device->previous_y = accel_y;
    device->previous_z = accel_z;
    device->has_previous_sample = 1U;
    return HAL_OK;
}
