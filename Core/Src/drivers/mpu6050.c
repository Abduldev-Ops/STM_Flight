/*
 * mpu6050.c
 *
 *  Created on: Jul 17, 2026
 *      Author: akanb
 */


#include "drivers/mpu6050.h"

#define MPU6050_REG_CONFIG        0x1A   /* digital low-pass filter      */
#define MPU6050_REG_GYRO_CONFIG   0x1B   /* gyro full-scale range        */
#define MPU6050_REG_ACCEL_CONFIG  0x1C   /* accel full-scale range       */

static HAL_StatusTypeDef mpu_write(I2C_HandleTypeDef *hi2c, uint8_t reg, uint8_t val)
{
    /* 100 ms timeout: a stuck bus returns an error instead of hanging forever */
    return HAL_I2C_Mem_Write(hi2c, MPU6050_ADDR, reg, I2C_MEMADD_SIZE_8BIT, &val, 1, 100);
}

HAL_StatusTypeDef MPU6050_Init(I2C_HandleTypeDef *hi2c)
{
	HAL_StatusTypeDef st;

	/* Wake up, clock from the gyro's PLL (more stable than the internal oscillator) */
	if ((st = mpu_write(hi2c, MPU6050_REG_PWR, 0x01)) != HAL_OK) return st;

	/* DLPF_CFG = 3: ~44 Hz accel / ~42 Hz gyro bandwidth, below the 50 Hz Nyquist
	 * limit of our 100 Hz sampling, so motor vibration can't alias into the angles */
	if ((st = mpu_write(hi2c, MPU6050_REG_CONFIG, 0x03)) != HAL_OK) return st;

	/* FS_SEL = 3: ±2000 deg/s -> 16.4 LSB per deg/s */
	if ((st = mpu_write(hi2c, MPU6050_REG_GYRO_CONFIG, 0x18)) != HAL_OK) return st;

	/* AFS_SEL = 2: ±8 g -> 4096 LSB per g */
	return mpu_write(hi2c, MPU6050_REG_ACCEL_CONFIG, 0x10);
}

HAL_StatusTypeDef MPU6050_Read(I2C_HandleTypeDef *hi2c, IMUData_t *data)
{
    uint8_t raw[14];
    HAL_StatusTypeDef status;
    uint32_t start = DWT->CYCCNT;

    status = HAL_I2C_Mem_Read(hi2c, MPU6050_ADDR, MPU6050_REG_DATA,
                               I2C_MEMADD_SIZE_8BIT, raw, 14, HAL_MAX_DELAY);
    if (status != HAL_OK) return status;
    data->timestamp = start;
    data->accel_x = (int16_t)(raw[0]  << 8 | raw[1]);
    data->accel_y = (int16_t)(raw[2]  << 8 | raw[3]);
    data->accel_z = (int16_t)(raw[4]  << 8 | raw[5]);
    data->gyro_x  = (int16_t)(raw[8]  << 8 | raw[9]);
    data->gyro_y  = (int16_t)(raw[10] << 8 | raw[11]);
    data->gyro_z  = (int16_t)(raw[12] << 8 | raw[13]);

    return HAL_OK;
}
