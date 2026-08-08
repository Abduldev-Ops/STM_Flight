/*
 * bmp280.c
 *
 *  Created on: Aug 8, 2026
 *      Author: akanb
 */


#include "drivers/bmp280.h"
#include <math.h>

// average sea level on day of writing Aug 8: 12am-10am using https://winnipeg.weatherstats.ca/charts/pressure_sea-hourly.html
#define SEA_LEVEL_PA 101421.0f

HAL_StatusTypeDef BMP280_Init(I2C_HandleTypeDef *hi2c, BMP280Calib_t *calib)
{
	uint8_t calib_raw[24];
	HAL_StatusTypeDef status;

	//read 24 bytes calib data at x88
	status = HAL_I2C_Mem_Read(hi2c, BMP280_ADDR, BMP280_REG_CALIB, I2C_MEMADD_SIZE_8BIT, calib_raw, 24, HAL_MAX_DELAY);

	if (status != HAL_OK) return status;

	//[arse calib coefficients
	calib->dig_T1 = (uint16_t)(calib_raw[1]  << 8 | calib_raw[0]);
	calib->dig_T2 = (int16_t) (calib_raw[3]  << 8 | calib_raw[2]);
	calib->dig_T3 = (int16_t) (calib_raw[5]  << 8 | calib_raw[4]);
	calib->dig_P1 = (uint16_t)(calib_raw[7]  << 8 | calib_raw[6]);
	calib->dig_P2 = (int16_t) (calib_raw[9]  << 8 | calib_raw[8]);
	calib->dig_P3 = (int16_t) (calib_raw[11] << 8 | calib_raw[10]);
	calib->dig_P4 = (int16_t) (calib_raw[13] << 8 | calib_raw[12]);
	calib->dig_P5 = (int16_t) (calib_raw[15] << 8 | calib_raw[14]);
	calib->dig_P6 = (int16_t) (calib_raw[17] << 8 | calib_raw[16]);
	calib->dig_P7 = (int16_t) (calib_raw[19] << 8 | calib_raw[18]);
	calib->dig_P8 = (int16_t) (calib_raw[21] << 8 | calib_raw[20]);
	calib->dig_P9 = (int16_t) (calib_raw[23] << 8 | calib_raw[22]);

	//set oversampling
	uint8_t ctrl = 0x27;  // osrs_t=001, osrs_p=001, mode=11
	status = HAL_I2C_Mem_Write(hi2c, BMP280_ADDR, BMP280_REG_CTRL, I2C_MEMADD_SIZE_8BIT, &ctrl, 1, HAL_MAX_DELAY);
	return status;
}

HAL_StatusTypeDef BMP280_Read(I2C_HandleTypeDef *hi2c, BMP280Calib_t *calib, BarData_t *out)
{
    uint8_t raw[6];
    HAL_StatusTypeDef status;

    // Read 6 bytes: press_msb, press_lsb, press_xlsb, temp_msb, temp_lsb, temp_xlsb
    status = HAL_I2C_Mem_Read(hi2c, BMP280_ADDR, BMP280_REG_DATA,
                               I2C_MEMADD_SIZE_8BIT, raw, 6, HAL_MAX_DELAY);
    if (status != HAL_OK) return status;

    int32_t adc_P = (int32_t)((raw[0] << 12) | (raw[1] << 4) | (raw[2] >> 4));
    int32_t adc_T = (int32_t)((raw[3] << 12) | (raw[4] << 4) | (raw[5] >> 4));

    // Bosch temperature compensation (from datasheet)
    int32_t var1 = ((((adc_T >> 3) - ((int32_t)calib->dig_T1 << 1)))
                    * ((int32_t)calib->dig_T2)) >> 11;
    int32_t var2 = (((((adc_T >> 4) - ((int32_t)calib->dig_T1))
                    * ((adc_T >> 4) - ((int32_t)calib->dig_T1))) >> 12)
                    * ((int32_t)calib->dig_T3)) >> 14;
    int32_t t_fine = var1 + var2;
    out->temperature = (float)((t_fine * 5 + 128) >> 8) / 100.0f;

    // Bosch pressure compensation (from datasheet)
    int64_t p_var1 = ((int64_t)t_fine) - 128000;
    int64_t p_var2 = p_var1 * p_var1 * (int64_t)calib->dig_P6;
    p_var2 = p_var2 + ((p_var1 * (int64_t)calib->dig_P5) << 17);
    p_var2 = p_var2 + (((int64_t)calib->dig_P4) << 35);
    p_var1 = ((p_var1 * p_var1 * (int64_t)calib->dig_P3) >> 8)
           + ((p_var1 * (int64_t)calib->dig_P2) << 12);
    p_var1 = (((((int64_t)1) << 47) + p_var1)) * ((int64_t)calib->dig_P1) >> 33;

    if (p_var1 == 0) return HAL_ERROR;

    int64_t pressure = 1048576 - adc_P;
    pressure = (((pressure << 31) - p_var2) * 3125) / p_var1;
    p_var1 = (((int64_t)calib->dig_P9) * (pressure >> 13) * (pressure >> 13)) >> 25;
    p_var2 = (((int64_t)calib->dig_P8) * pressure) >> 19;
    pressure = ((pressure + p_var1 + p_var2) >> 8) + (((int64_t)calib->dig_P7) << 4);

    float pressure_pa = (float)pressure / 256.0f;

    // Convert press to alt (barometric formula)
    out->altitude = 44330.0f * (1.0f - powf(pressure_pa / SEA_LEVEL_PA, 0.1903f));

    return HAL_OK;
}
