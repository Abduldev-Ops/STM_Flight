/*
 * task_filter.c
 *
 *  Created on: Jul 17, 2026
 *      Author: akanb
 */


#include "app_types.h"
#include "app_globals.h"
#include "cmsis_os.h"
#include <math.h>
#include "flight/ekf.h"
#include "FreeRTOS.h"
#include "queue.h"
#include <stdio.h>

#define LEVEL_CAL_SAMPLES  200

/* ---- Accelerometer gating ------------------------------------------------
 * The accel only measures tilt when the only force on it is gravity (|a| = 1 g).
 * Thrust changes, braking and vibration add acceleration and fake a tilt. */
#define ACC_TRUST_BAND   0.10f    /* |a| within 1 ± 0.10 g: full trust              */
#define ACC_REJECT_BAND  0.30f    /* beyond 1 ± 0.30 g: ignore accel, gyro only     */
#define R_BASE           0.003f   /* noise when trusted (same value as EKF_Init)    */
#define R_MAX_SCALE      100.0f   /* R is up to 100x larger at the edge of the band */

void task_filter(void *argument)
{
    while (imu_queue == NULL || state_queue == NULL) osDelay(1);

    IMUData_t imu;
    FlightState_t state = {0};

    uint32_t prev_ts = 0;
    float dt_min = 1.0f, dt_max = 0.0f;
    uint32_t last_dt_report = osKernelGetTickCount();

    float a_min = 10.0f, a_max = 0.0f;
    uint32_t acc_derated = 0, acc_skipped = 0;

    const float ACCEL_SCALE = 4096.0f;
    const float GYRO_SCALE  = 16.4f;
    const float RAD_TO_DEG  = 180.0f / 3.14159f;
    const float DEG_TO_RAD  = 3.14159f / 180.0f;

    EKF_t ekf;
    EKF_Init(&ekf);

    float roll_offset = 0.0f, pitch_offset = 0.0f;
    int cal_count = 0;
    float a_ref = 0.0f;


    for (;;)
    {
        if (osMessageQueueGet(imu_queue, &imu, NULL, osWaitForever) == osOK)
        {
        	float dt = 0.01f;
        	if (prev_ts != 0)
        	{
        		dt= (float)(imu.timestamp - prev_ts) / (float)SystemCoreClock;
        	}
        	prev_ts = imu.timestamp;

        	if (dt < 0.001f) dt  = 0.001f;
        	if (dt > 0.05f) dt  = 0.05f;

        	if (dt < dt_min) dt_min = dt;
        	if (dt > dt_max) dt_max = dt;

        	float ax = imu.accel_x / ACCEL_SCALE;
            float ay = imu.accel_y / ACCEL_SCALE;
            float az = imu.accel_z / ACCEL_SCALE;
            float gx_dps = imu.gyro_x  / GYRO_SCALE;
            float gy_dps = imu.gyro_y  / GYRO_SCALE;
            float gz_dps = imu.gyro_z  / GYRO_SCALE;

            float roll_meas  = atan2f(ay, az);
            float pitch_meas = atan2f(-ax, sqrtf(ay*ay + az*az));

            float gx_rps = gx_dps * DEG_TO_RAD;
            float gy_rps = gy_dps * DEG_TO_RAD;

            if (cal_count < LEVEL_CAL_SAMPLES)
            {
            	roll_offset  += roll_meas;
            	pitch_offset += pitch_meas;
            	a_ref += sqrtf(ax*ax + ay*ay + az*az);
            	if (++cal_count == LEVEL_CAL_SAMPLES)
            	{
            		roll_offset  /= LEVEL_CAL_SAMPLES;
            		pitch_offset /= LEVEL_CAL_SAMPLES;
            		a_ref /= LEVEL_CAL_SAMPLES;
            		osMutexAcquire(uart_mutex, osWaitForever);
            		printf("[CAL] level offsets: a_ref %.2f, roll %.2f deg, pitch %.2f deg\r\n",
            				a_ref, roll_offset * RAD_TO_DEG, pitch_offset * RAD_TO_DEG);
            		osMutexRelease(uart_mutex);
            	}
            	continue;           /* no state output until calibrated */
            }

            roll_meas  -= roll_offset;
            pitch_meas -= pitch_offset;

            /* ---- Accelerometer gating ---- */
            float a_norm = sqrtf(ax*ax + ay*ay + az*az);          /* total acceleration in g */
            float a_err = fabsf(a_norm - a_ref);
            if (a_norm < a_min) a_min = a_norm;
            if (a_norm > a_max) a_max = a_norm;

            EKF_Predict(&ekf, gx_rps, gy_rps, dt);                 /* gyro: always */

            if (a_err < ACC_TRUST_BAND)
            {
                /* Near 1 g: accelerometer is a good tilt reference */
                EKF_SetMeasNoise(&ekf, R_BASE);
                EKF_Update(&ekf, roll_meas, pitch_meas);
            }
            else if (a_err < ACC_REJECT_BAND)
            {
                /* In between: trust it less the further |a| is from 1 g (linear ramp) */
                float t = (a_err - ACC_TRUST_BAND) / (ACC_REJECT_BAND - ACC_TRUST_BAND);   /* 0..1 */
                EKF_SetMeasNoise(&ekf, R_BASE * (1.0f + t * (R_MAX_SCALE - 1.0f)));
                EKF_Update(&ekf, roll_meas, pitch_meas);
                acc_derated++;
            }
            else
            {
                /* Far from 1 g: the accelerometer is lying about tilt. Gyro only. */
                acc_skipped++;
            }

            state.roll = ekf.x[0] * RAD_TO_DEG;
            state.pitch = ekf.x[1] * RAD_TO_DEG;
            state.gyro_x = gx_dps;
            state.gyro_y = gy_dps;
            state.gyro_z = gz_dps;
            state.bias_x = ekf.x[2] * RAD_TO_DEG;
            state.bias_y = ekf.x[3] * RAD_TO_DEG;
            state.dt = dt;

            xQueueOverwrite((QueueHandle_t)state_queue, &state);
            osMessageQueuePut(control_state_queue, &state, 0, 0);

            if (osKernelGetTickCount() - last_dt_report >= 1000)
            {
            	osMutexAcquire(uart_mutex, osWaitForever);
            	printf("[DT] min %.2f ms, max %.2f ms | |a| %.2f-%.2f g, derated %lu, skipped %lu\r\n",
            	       dt_min * 1000.0f, dt_max * 1000.0f, a_min, a_max, acc_derated, acc_skipped);
            	dt_min = 1.0f; dt_max = 0.0f;
            	a_min = 10.0f; a_max = 0.0f;
            	acc_derated = 0; acc_skipped = 0;
                osMutexRelease(uart_mutex);
                dt_min = 1.0f; dt_max = 0.0f;
                last_dt_report = osKernelGetTickCount();
            }
        }
    }
}
