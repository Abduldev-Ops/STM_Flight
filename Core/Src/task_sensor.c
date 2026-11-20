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
#include <stdio.h>

void task_sensor(void *argument)
{
    while (uart_mutex == NULL || imu_queue == NULL || i2c_mutex == NULL)
        osDelay(1);

    IMUData_t data;

    osMutexAcquire(i2c_mutex, osWaitForever);
    HAL_StatusTypeDef init_st = MPU6050_Init(&hi2c1);
    osMutexRelease(i2c_mutex);

    if (init_st != HAL_OK)
    {
        osMutexAcquire(uart_mutex, osWaitForever);
        printf("[IMU] INIT FAILED (%d)\r\n", init_st);
        osMutexRelease(uart_mutex);
    }

    uint32_t ok_count = 0, fail_count = 0;
    uint32_t last_report = osKernelGetTickCount();
    uint32_t next_wake = osKernelGetTickCount();

    for (;;)
    {
    	next_wake += 10;

        osMutexAcquire(i2c_mutex, osWaitForever);
        HAL_StatusTypeDef st = MPU6050_Read(&hi2c1, &data);
        osMutexRelease(i2c_mutex);

        if (st == HAL_OK)
        {
        	ok_count++;
        	osMessageQueuePut(imu_queue, &data, 0, 0);
        }
        else
        {
        	fail_count++;   // don't push stale/garbage data into the filter
        }

        // Report once per second; timeout 0 so a busy UART never delays

        uint32_t now = osKernelGetTickCount();
        if (now - last_report >= 1000)
        {
        	if (osMutexAcquire(uart_mutex, 0) == osOK)
        	{
        		printf("[IMU] %lu ok/s, %lu fail/s\r\n", ok_count, fail_count);
        		osMutexRelease(uart_mutex);
        	}
        	ok_count = 0;
        	fail_count = 0;
        	last_report = now;
        }
        if (osDelayUntil(next_wake) != osOK)
        {
        	next_wake = osKernelGetTickCount();
        }
    }
}
