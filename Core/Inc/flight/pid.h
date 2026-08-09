/*
 * pid.h
 *
 *  Created on: Jul 17, 2026
 *      Author: akanb
 */

#ifndef FLIGHT_PID_H
#define FLIGHT_PID_H

typedef struct{
	float kp, ki, kd;
	float integral;
	float prev_error;
	float integral_limit;
	float output_limit;
} PID_t;

void  PID_Init(PID_t *pid, float kp, float ki, float kd, float integral_limit, float output_limit);
float PID_Update(PID_t *pid, float setpoint, float measurement, float dt);
void  PID_Reset(PID_t *pid);


#endif /* INC_FLIGHT_PID_H_ */
