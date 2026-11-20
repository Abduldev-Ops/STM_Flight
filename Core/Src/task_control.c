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
#include "flight/dshot.h"
#include "FreeRTOS.h"
#include "queue.h"

#define TELE_PACKET_START 0XCD

PID_t pid_rollrate;
PID_t pid_pitchrate;
PID_t pid_yawrate;
PID_t pid_roll_angle;
PID_t pid_pitch_angle;

TelemetryPckt_t tele;

static uint8_t calcCheckSum(TelemetryPckt_t *tele)
{
	uint8_t *bytes = (uint8_t *) tele;
	uint8_t chk = 0;
	for (int i = 1; i < sizeof(TelemetryPckt_t) - 1; i++)
		chk ^= bytes[i];
	return chk;
}


#define MOTOR_IDLE			10.0f
#define MOTOR_MAX	  		100.0f
#define THROTTLE_ACTIVE 	5.0f

static float max4(float a, float b, float c, float d) { float m = a; if (b > m) m = b; if (c > m) m = c; if (d > m) m = d; return m; }
static float min4(float a, float b, float c, float d) { float m = a; if (b < m) m = b; if (c < m) m = c; if (d < m) m = d; return m; }


#define MOTOR_TEST 0
#define PID_BYPASS 0
#define YAW_TEST 0
static float yaw_bias = 0.0f;

static void reset_all_pids(void)
{
	PID_Reset(&pid_rollrate);
	PID_Reset(&pid_pitchrate);
	PID_Reset(&pid_yawrate);
	PID_Reset(&pid_roll_angle);
	PID_Reset(&pid_pitch_angle);
}

void task_control(void *argument)
{
	while (state_queue == NULL || uart_mutex == NULL) osDelay(1);

	PID_Init(&pid_rollrate, 0.5f, 0.01f, 0.1f, 20.0f, 30.0f);
	PID_Init(&pid_pitchrate, 0.5f, 0.01f, 0.1f, 20.0f, 30.0f);
	PID_Init(&pid_yawrate, 0.3f, 0.01f, 0.05f, 20.0f, 30.0f);
	PID_Init(&pid_roll_angle, 2.0f, 0.0f, 0.0f, 50.0f, 200.0f);
	PID_Init(&pid_pitch_angle, 2.0f, 0.0f, 0.0f, 50.0f, 200.0f);

	PID_SetDCutoff(&pid_rollrate,  20.0f);
	PID_SetDCutoff(&pid_pitchrate, 20.0f);
	PID_SetDCutoff(&pid_yawrate,   20.0f);

	//const float THROTTLE = 50.0f;
	FlightState_t state;
	RCInput_t rc = {0};
	BarData_t baro = {0};
	GPSData_t gps = {0};

	for (;;){
		osMessageQueueGet(rc_queue, &rc, NULL, 0);
		if (osMessageQueueGet(control_state_queue, &state, NULL, osWaitForever) == osOK)
		{
			const uint8_t is_armed = armed;
			const float   thr      = throttle;

			float roll_rate_setpoint = 0.0f, pitch_rate_setpoint = 0.0f;
			float m1, m2, m3, m4;

			if (!is_armed){
				reset_all_pids();
				yaw_bias += 0.01f * (state.gyro_z - yaw_bias);  /* learn offset while still */
				m1 = m2 = m3 = m4 = 0.0f;
			}
			else if (thr < THROTTLE_ACTIVE) {
				reset_all_pids();

#if MOTOR_TEST
				uint32_t slot = (osKernelGetTickCount()/ 2000) % 4;
				m1 = (slot == 0) ? MOTOR_IDLE : 0.0f;
				m2 = (slot == 1) ? MOTOR_IDLE : 0.0f;
				m3 = (slot == 2) ? MOTOR_IDLE : 0.0f;
				m4 = (slot == 3) ? MOTOR_IDLE : 0.0f;
#else
				m1 = m2 = m3 = m4 = MOTOR_IDLE;
#endif
			}
			else{
#if PID_BYPASS
				m1 = m2 = m3 = m4 = (thr < MOTOR_IDLE) ? MOTOR_IDLE : thr;
#else
				float gyro_x_corrected = state.gyro_x - state.bias_x;
				float gyro_y_corrected = state.gyro_y - state.bias_y;

				float desired_roll = rc.roll;
				float desired_pitch = rc.pitch;
				float desired_yaw = rc.yaw;

				roll_rate_setpoint = PID_Update(&pid_roll_angle, desired_roll, state.roll, state.dt);
				pitch_rate_setpoint = PID_Update(&pid_pitch_angle, desired_pitch, state.pitch, state.dt);

				float roll_out = PID_Update(&pid_rollrate, roll_rate_setpoint, gyro_x_corrected, state.dt);
				float pitch_out = PID_Update(&pid_pitchrate, pitch_rate_setpoint, gyro_y_corrected, state.dt);
				float yaw_out = PID_Update(&pid_yawrate, desired_yaw, state.gyro_z - yaw_bias, state.dt);

#if YAW_TEST
				roll_out  = 0.0f;
				pitch_out = 0.0f;
#endif

				float c1= pitch_out - roll_out - yaw_out;
				float c2= -pitch_out + roll_out - yaw_out;
				float c3= pitch_out + roll_out + yaw_out;
				float c4= -pitch_out - roll_out + yaw_out;

				float cmax = max4(c1,c2,c3,c4);
				float cmin = min4(c1,c2,c3,c4);
				float span = cmax - cmin;
				float room = MOTOR_MAX - MOTOR_IDLE;

				if (span > room){
					float k = room / span;
					c1 *= k; c2 *= k; c3 *= k; c4 *= k;
					cmax *= k; cmin *= k;
				}

				float base = thr;
				if (base + cmax > MOTOR_MAX) base = MOTOR_MAX - cmax;
				if (base + cmin < MOTOR_IDLE) base = MOTOR_IDLE - cmin;

				m1 = base + c1;
				m2 = base + c2;
				m3 = base + c3;
				m4 = base + c4;
#endif
			}

			//DSHOT_SendMotors(m1, m2, m3, m4, is_armed);
			DSHOT_SetMotors(m1, m2, m3, m4, is_armed);

			xQueuePeek((QueueHandle_t)gps_queue,  &gps,  0);   // read without removing
			xQueuePeek((QueueHandle_t)baro_queue, &baro, 0);

			tele.start = TELE_PACKET_START;
			tele.roll = state.roll;
			tele.pitch = state.pitch;
			tele.altitude = baro.altitude;
			tele.latitude = gps.latitude;
			tele.longitude = gps.longitude;
			tele.throttle = thr;
			tele.m1 = m1;
			tele.m2 = m2;
			tele.m3 = m3;
			tele.m4 = m4;
			tele.armed = is_armed;
			tele.gps_fix = gps.fix;
			tele.satellites = gps.satellites;
			tele.checksum = calcCheckSum(&tele);

			xQueueOverwrite((QueueHandle_t)telemetry_queue, &tele);

			CtrlLog_t log = {
			    .armed = is_armed, .thr = thr, .roll = state.roll, .pitch = state.pitch,
			    .rs = roll_rate_setpoint, .ps = pitch_rate_setpoint,
			    .m1 = m1, .m2 = m2, .m3 = m3, .m4 = m4,
			};
			xQueueOverwrite((QueueHandle_t)log_queue, &log);   // never blocks, just replaces the latest
		}
	}
}
