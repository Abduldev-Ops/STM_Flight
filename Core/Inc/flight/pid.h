/*
 * pid.h
 *
 *  Created on: Jul 17, 2026
 *      Author: akanb
 */

#ifndef FLIGHT_PID_H
#define FLIGHT_PID_H

#include <stdint.h>

typedef struct{
	float kp, ki, kd;
	float  i_term;
	float prev_measurement;
	float d_filtered;
	float d_cutoff_hz;
	float integral_limit;
	float output_limit;
	uint8_t first_run;
} PID_t;

void  PID_Init(PID_t *pid, float kp, float ki, float kd, float integral_limit, float output_limit);
void  PID_SetDCutoff(PID_t *pid, float cutoff_hz);
float PID_Update(PID_t *pid, float setpoint, float measurement, float dt);
void  PID_Reset(PID_t *pid);


#endif /* INC_FLIGHT_PID_H_ */
