/*
 * bmp280.h
 *
 *  Created on: Aug 8, 2026
 *      Author: akanb
 */

#ifndef DRIVERS_BMP280_H
#define DRIVERS_BMP280_H

#include "stm32l4xx_hal.h"
#include "app_types.h"

#define BMP280_ADDR (0X76 << 1)
#define BMP280_REG_ID 0XD0
#define BMP280_REG_RESET 0XE0
#define BMP280_REG_CTRL 0XF4
#define BMP280_REG_CONFIG 0XF5
#define BMP280_REG_CALIB 0X88
#define BMP280_REG_DATA 0XF7

HAL_StatusTypeDef BMP280_Init(I2C_HandleTypeDef *hi2c, BMP280Calib_t *calib);
HAL_StatusTypeDef BMP280_Read(I2C_HandleTypeDef *hi2c, BMP280Calib_t *calib, BarData_t *out);

#endif /* INC_DRIVERS_BMP280_H_ */
