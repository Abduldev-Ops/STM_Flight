/*
 * pid.c
 *
 *  Created on: Jul 17, 2026
 *      Author: akanb
 */


#include "flight/pid.h"
#include <math.h>

#define PI_F   3.14159265f

static float clampf(float v, float lim){
	if (v >  lim) return  lim;
	if (v < -lim) return -lim;
	return v;
}

void  PID_Init(PID_t *pid, float kp, float ki, float kd, float integral_limit, float output_limit)
{
	pid->kp = kp;
	pid->ki = ki;
	pid->kd = kd;
	pid->integral_limit = integral_limit;
	pid-> output_limit = output_limit;
	pid -> d_cutoff_hz = 40.0f;
	PID_Reset(pid);
}

void PID_SetDCutoff(PID_t *pid, float cutoff_hz)
{
	pid->d_cutoff_hz = cutoff_hz;
}

float PID_Update(PID_t *pid, float setpoint, float measurement, float dt)
{
	if (dt <= 0.0f) dt = 1e-3f;                 // guard: never divide by zero

	float error = setpoint - measurement;

	/* P */
	float p_term = pid->kp * error;

	/* D on measurement: -d(measurement)/dt. Setpoint changes don't kick it. */
	float d_raw = 0.0f;
	if (pid->first_run)
	{
		pid->prev_measurement = measurement;    // no history yet: no spike
		pid->d_filtered = 0.0f;
		pid->first_run = 0;
	}
	else
	{
		d_raw = -(measurement - pid->prev_measurement) / dt;
		pid->prev_measurement = measurement;
	}
	float rc    = 1.0f / (2.0f * PI_F * pid->d_cutoff_hz);
	float alpha = dt / (rc + dt);
	pid->d_filtered += alpha * (d_raw - pid->d_filtered);
	float d_term = pid->kd * pid->d_filtered;

	/* I: tentative update in output units */
	float i_candidate = clampf(pid->i_term + pid->ki * error * dt, pid->integral_limit);

	float output = p_term + i_candidate + d_term;

	/* Conditional integration: if the output is saturated and the error would
	 * push it further into saturation, keep the old i_term (no wind-up). */
	if ((output >  pid->output_limit && error > 0.0f) ||
	    (output < -pid->output_limit && error < 0.0f))
	{
		output = p_term + pid->i_term + d_term;
	}
	else
	{
		pid->i_term = i_candidate;
	}

	return clampf(output, pid->output_limit);
}

void  PID_Reset(PID_t *pid)
{
	pid->i_term = 0.0f;
	pid->prev_measurement = 0.0f;
	pid->d_filtered = 0.0f;
	pid->first_run = 1;
}
