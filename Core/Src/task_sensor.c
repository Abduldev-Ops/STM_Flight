/*
 * task_sensor.c
 *
 *  Created on: Jul 17, 2026
 *      Author: akanb
 */


#include "app_types.h"
#include "app_globals.h"
#include "drivers/mpu6050.h"
#include "i2c.h"
#include "cmsis_os.h"

void task_sensor(void *argument)
{
    while (uart_mutex == NULL || imu_queue == NULL || i2c_mutex == NULL)
        osDelay(1);

    IMUData_t data;

    osMutexAcquire(i2c_mutex, osWaitForever);
    MPU6050_Init(&hi2c1);
    osMutexRelease(i2c_mutex);

    for (;;)
    {
        osMutexAcquire(i2c_mutex, osWaitForever);
        MPU6050_Read(&hi2c1, &data);
        osMutexRelease(i2c_mutex);

        osMessageQueuePut(imu_queue, &data, 0, 0);
        osDelay(10);
    }
}
