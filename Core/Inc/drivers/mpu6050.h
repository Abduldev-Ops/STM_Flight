/*
 * mpu6050.h
 *
 *  Created on: Jul 17, 2026
 *      Author: akanb
 */

//#ifndef INC_MPU6050_H_
//#define INC_MPU6050_H_

#ifndef DRIVERS_MPU6050_H
#define DRIVERS_MPU6050_H

#include "stm32l4xx_hal.h"
#include "app_types.h"

#define MPU6050_ADDR        (0x68 << 1)
#define MPU6050_REG_PWR     0x6B
#define MPU6050_REG_DATA    0x3B
#define MPU6050_WHO_AM_I    0x75

HAL_StatusTypeDef MPU6050_Init(I2C_HandleTypeDef *hi2c);
HAL_StatusTypeDef MPU6050_Read(I2C_HandleTypeDef *hi2c, IMUData_t *data);

#endif
//
//#endif /* INC_MPU6050_H_ */
