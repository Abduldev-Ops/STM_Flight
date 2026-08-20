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

//typedef struct {
//    float roll;
//    float pitch;
//} Attitude_t;

typedef struct {
	float altitude;
	float temperature;
} BarData_t;

typedef struct {
	//temp compensation
	uint16_t dig_T1;
	int16_t dig_T2;
	int16_t dig_T3;

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

typedef struct {
	float roll;
	float pitch;
	float gyro_x;
	float gyro_y;
	float gyro_z;
	float bias_x;
	float bias_y;
}FlightState_t;

typedef struct {
    float m1;
    float m2;
    float m3;
    float m4;
} MotorOutput_t;

typedef struct{
	float latitude;
	float longitude;
	float altitude;
	float speed;
	uint8_t fix;
	uint8_t satellites;
	uint8_t valid;
} GPSData_t;

typedef struct {
	float throttle;
	float roll;
	float pitch;
	float yaw;
	uint8_t armed;
} RCInput_t;

typedef struct __attribute__((packed)){
	uint8_t start;
	float roll;
	float pitch;
	float altitude;
	float latitude;
	float longitude;
	float throttle;
	float m1, m2, m3, m4;
	uint8_t armed;
	uint8_t gps_fix;
	uint8_t satellites;
	uint8_t checksum;
} TelemetryPckt_t;
#endif /* APP_TYPES_H */
//
//#endif /* INC_APP_TYPES_H_ */
