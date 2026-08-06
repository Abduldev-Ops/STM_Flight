/*
 * ekf.h
 *
 *  Created on: Jul 17, 2026
 *      Author: akanb
 */

#ifndef EKF_H_
#define EKF_H_

typedef struct{
	float x[4]; //state r, p, gyro bias c, gyro bias y,
	float P[4][4];
	float Q[4][4];
	float R[2][2];
} EKF_t;


void  EKF_Init(EKF_t *ekf);
void EKF_Predict(EKF_t *ekf, float gx, float gy, float dt);
void EKF_Update(EKF_t *ekf, float roll_meas, float pitch_meas);
#endif /* INC_FLIGHT_EKF_H_ */
