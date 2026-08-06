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

#endif /* APP_TYPES_H */
//
//#endif /* INC_APP_TYPES_H_ */
