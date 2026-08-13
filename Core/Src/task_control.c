/*
 * task_control.c
 *
 *  Created on: Aug 8, 2026
 *      Author: akanb
 */


#include "app_types.h"
#include "app_globals.h"
#include "cmsis_os.h"
#include "flight/pid.h"
#include "stdio.h"

PID_t pid_rollrate;
PID_t pid_pitchrate;
PID_t pid_yawrate;
PID_t pid_roll_angle;
PID_t pid_pitch_angle;

void task_control(void *argument)
{
	while (state_queue == NULL || uart_mutex == NULL) osDelay(1);

	PID_Init(&pid_rollrate, 0.5f, 0.01f, 0.1f, 50.0f, 100.0f);
	PID_Init(&pid_pitchrate, 0.5f, 0.01f, 0.1f, 50.0f, 100.0f);
	PID_Init(&pid_yawrate, 0.3f, 0.01f, 0.05f, 50.0f, 100.0f);
	PID_Init(&pid_roll_angle, 2.0f, 0.0f, 0.0f, 50.0, 200.0);
	PID_Init(&pid_pitch_angle, 2.0f, 0.0f, 0.0f, 50.0, 200.0);

	const float dt= 0.01f;
	//const float THROTTLE = 50.0f;
	FlightState_t state;


	for (;;){
		if (osMessageQueueGet(control_state_queue, &state, NULL, osWaitForever) == osOK)
		{
			float gyro_x_corrected = state.gyro_x - state.bias_x;
			float gyro_y_corrected = state.gyro_y - state.bias_y;

			float roll_rate_setpoint = PID_Update(&pid_roll_angle, 0, state.roll, dt);
			float pitch_rate_setpoint = PID_Update(&pid_pitch_angle, 0, state.pitch, dt);

			float roll_out = PID_Update(&pid_rollrate, roll_rate_setpoint, gyro_x_corrected, dt);
			float pitch_out = PID_Update(&pid_pitchrate, pitch_rate_setpoint, gyro_y_corrected, dt);
			float yaw_out = PID_Update(&pid_yawrate, 0, state.gyro_z, dt);


			float base = armed ? throttle : 0.0f;

			float m1= base + pitch_out - roll_out - yaw_out;
			float m2= base - pitch_out + roll_out - yaw_out;
			float m3= base + pitch_out + roll_out + yaw_out;
			float m4= base - pitch_out - roll_out + yaw_out;

			if (!armed)
			{
				m1 = 0.0f;
				m2 = 0.0f;
				m3 = 0.0f;
				m4 = 0.0f;
			} else {
				if (m1 < 0.0f) m1 = 0.0f;
				if (m1 > 100.0f) m1 = 100.0f;
				if (m2 < 0.0f) m2 = 0.0f;
				if (m2 > 100.0f) m2 = 100.0f;
				if (m3 < 0.0f) m3 = 0.0f;
				if (m3 > 100.0f) m3 = 100.0f;
				if (m4 < 0.0f) m4 = 0.0f;
				if (m4 > 100.0f) m4 = 100.0f;
			}
			if (logging_enabled){
				osMutexAcquire(uart_mutex, osWaitForever);
				printf("[%s] T:%.0f RS:%.1f PS:%.1f M1:%.0f M2:%.0f M3:%.0f M4:%.0f\r\n", armed ? "ARM" : "DIS", throttle,
				       roll_rate_setpoint, pitch_rate_setpoint, m1, m2, m3, m4);
				osMutexRelease(uart_mutex);
			}
		}
	}
}
