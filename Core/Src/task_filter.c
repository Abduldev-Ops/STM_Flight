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
    while (imu_queue == NULL || attitude_queue == NULL) osDelay(1);

    IMUData_t imu;
    Attitude_t att = {0.0f, 0.0f};

    const float ACCEL_SCALE = 16384.0f;
    const float GYRO_SCALE  = 131.0f;
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
            float gx = imu.gyro_x  / GYRO_SCALE * 3.14159f / 180.0f;
            float gy = imu.gyro_y  / GYRO_SCALE * 3.14159f / 180.0f;

            float roll_accel  = atan2f(ay, az);
            float pitch_accel = atan2f(-ax, sqrtf(ay*ay + az*az));

            EKF_Predict(&ekf, gx, gy, dt);
            EKF_Update(&ekf, roll_accel, pitch_accel);

            att.roll  = ekf.x[0] * 180.0f / 3.14159f;
            att.pitch = ekf.x[1] * 180.0f / 3.14159f;

            osMessageQueuePut(attitude_queue, &att, 0, 0);
        }
    }
}
