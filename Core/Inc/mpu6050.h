#ifndef MPU6050_H
#define MPU6050_H

#include "stm32l4xx_hal.h"

typedef struct {
    I2C_HandleTypeDef *i2c;
    int16_t previous_x;
    int16_t previous_y;
    int16_t previous_z;
    uint8_t has_previous_sample;
} MPU6050_t;

HAL_StatusTypeDef MPU6050_Init(MPU6050_t *device, I2C_HandleTypeDef *i2c);
HAL_StatusTypeDef MPU6050_DetectMotion(MPU6050_t *device,
                                       int32_t threshold,
                                       uint8_t *movement_detected);

#endif
