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

void task_filter(void *argument)
{
    while (imu_queue == NULL || state_queue == NULL) osDelay(1);

    IMUData_t imu;
    FlightState_t state = {0};

    const float ACCEL_SCALE = 16384.0f;
    const float GYRO_SCALE  = 131.0f;
    const float RAD_TO_DEG  = 180.0f / 3.14159f;
    const float DEG_TO_RAD  = 3.14159f / 180.0f;
    const float dt           = 0.01f;

    EKF_t ekf;
    EKF_Init(&ekf);


    for (;;)
    {
        if (osMessageQueueGet(imu_queue, &imu, NULL, osWaitForever) == osOK)
        {
            float ax = imu.accel_x / ACCEL_SCALE;
            float ay = imu.accel_y / ACCEL_SCALE;
            float az = imu.accel_z / ACCEL_SCALE;
            float gx_dps = imu.gyro_x  / GYRO_SCALE;
            float gy_dps = imu.gyro_y  / GYRO_SCALE;
            float gz_dps = imu.gyro_z  / GYRO_SCALE;

            float gx_rps = gx_dps * DEG_TO_RAD;
            float gy_rps = gy_dps * DEG_TO_RAD;

            float roll_meas  = atan2f(ay, az);
            float pitch_meas = atan2f(-ax, sqrtf(ay*ay + az*az));

            EKF_Predict(&ekf, gx_rps, gy_rps, dt);
            EKF_Update(&ekf, roll_meas, pitch_meas);

            state.roll = ekf.x[0] * RAD_TO_DEG;
            state.pitch = ekf.x[1] * RAD_TO_DEG;
            state.gyro_x = gx_dps;
            state.gyro_y = gy_dps;
            state.gyro_z = gz_dps;
            state.bias_x = ekf.x[2] * RAD_TO_DEG;
            state.bias_y = ekf.x[3] * RAD_TO_DEG;

            osMessageQueuePut(state_queue, &state, 0, 0);
        }
    }
}
