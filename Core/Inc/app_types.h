/*
 * app_types.h
 *
 *  Created on: Jul 17, 2026
 *      Author: akanb
 */

//#ifndef INC_APP_TYPES_H_
//#define INC_APP_TYPES_H_

#ifndef APP_TYPES_H
#define APP_TYPES_H

#include <stdint.h>

typedef struct {
    int16_t accel_x;
    int16_t accel_y;
    int16_t accel_z;
    int16_t gyro_x;
    int16_t gyro_y;
    int16_t gyro_z;
} IMUData_t;

typedef struct {
    float roll;
    float pitch;
} Attitude_t;

typedef struct {
	float altitude;
	float temperature;
} BarData_t;

typedef struct {
	//temp compensation
	uint16_t dig_T1;
	uint16_t dig_T2;
	uint16_t dig_T3;

	//press compensation
	uint16_t dig_P1;
	int16_t dig_P2;
	int16_t dig_P3;
	int16_t dig_P4;
	int16_t dig_P5;
	int16_t dig_P6;
	int16_t dig_P7;
	int16_t dig_P8;
	int16_t dig_P9;
} BMP280Calib_t;
#endif /* APP_TYPES_H */
//
//#endif /* INC_APP_TYPES_H_ */
