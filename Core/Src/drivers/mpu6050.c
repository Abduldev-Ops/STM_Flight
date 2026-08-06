/*
 * mpu6050.c
 *
 *  Created on: Jul 17, 2026
 *      Author: akanb
 */


#include "drivers/mpu6050.h"

HAL_StatusTypeDef MPU6050_Init(I2C_HandleTypeDef *hi2c)
{
    uint8_t wake = 0x00;
    return HAL_I2C_Mem_Write(hi2c, MPU6050_ADDR, MPU6050_REG_PWR,
                              I2C_MEMADD_SIZE_8BIT, &wake, 1, HAL_MAX_DELAY);
}

HAL_StatusTypeDef MPU6050_Read(I2C_HandleTypeDef *hi2c, IMUData_t *data)
{
    uint8_t raw[14];
    HAL_StatusTypeDef status;

    status = HAL_I2C_Mem_Read(hi2c, MPU6050_ADDR, MPU6050_REG_DATA,
                               I2C_MEMADD_SIZE_8BIT, raw, 14, HAL_MAX_DELAY);
    if (status != HAL_OK) return status;

    data->accel_x = (int16_t)(raw[0]  << 8 | raw[1]);
    data->accel_y = (int16_t)(raw[2]  << 8 | raw[3]);
    data->accel_z = (int16_t)(raw[4]  << 8 | raw[5]);
    data->gyro_x  = (int16_t)(raw[8]  << 8 | raw[9]);
    data->gyro_y  = (int16_t)(raw[10] << 8 | raw[11]);
    data->gyro_z  = (int16_t)(raw[12] << 8 | raw[13]);

    return HAL_OK;
}
