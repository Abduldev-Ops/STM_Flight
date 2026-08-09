/*
 * pid.c
 *
 *  Created on: Jul 17, 2026
 *      Author: akanb
 */


#include "flight/pid.h"
#include <math.h>

void  PID_Init(PID_t *pid, float kp, float ki, float kd, float integral_limit, float output_limit)
{
	pid->kp = kp;
	pid->ki = ki;
	pid->kd = kd;
	pid->integral = 0.0f;
	pid->prev_error  =0.0f;
	pid->integral_limit = integral_limit;
	pid-> output_limit = output_limit;
}

float PID_Update(PID_t *pid, float setpoint, float measurement, float dt)
{
	float error;

	error = setpoint - measurement;
	pid->integral += (error * dt);

	if (pid->integral > pid->integral_limit){
		pid->integral = pid->integral_limit;
	}else if (pid->integral < -pid->integral_limit){
		pid->integral = -pid->integral_limit;
	}

	float derivative = (error - pid->prev_error) / dt;
	pid->prev_error = error;
	float output = (pid->kp * error) + (pid->ki * pid->integral) + (pid->kd * derivative);

	if (output > pid->output_limit){
		output = pid->output_limit;
	} else if (output < -pid->output_limit){
		output = -pid->output_limit;
	}

	return output;
}

void  PID_Reset(PID_t *pid)
{
	pid->integral = 0.0f;
	pid->prev_error = 0.0f;
}
