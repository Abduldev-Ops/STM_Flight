/*
 * task_baro.c
 *
 *  Created on: Aug 8, 2026
 *      Author: akanb
 */


#include "app_types.h"
#include "app_globals.h"
#include "drivers/bmp280.h"
#include "i2c.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "cmsis_os.h"
#include <stdio.h>

#define BARO_GROUND_SAMPLES  20

void task_baro(void *argument)
{
	while (i2c_mutex == NULL || baro_queue == NULL) osDelay(1);

	BMP280Calib_t calib;

	//init bmp and read calib coeff
	osMutexAcquire(i2c_mutex, osWaitForever);
	HAL_StatusTypeDef init_st = BMP280_Init(&hi2c1, &calib);
	osMutexRelease(i2c_mutex);

	if (init_st != HAL_OK)
	{
		osMutexAcquire(uart_mutex, osWaitForever);
		printf("[BARO] INIT FAILED (%d)\r\n", init_st);
		osMutexRelease(uart_mutex);
	}

	osDelay(100);   /* first conversion at x16 oversampling takes ~40 ms */

	BarData_t baro;
	float ground_alt = 0.0f;
	int   ground_n   = 0;

	for (;;)
	{
		osMutexAcquire(i2c_mutex, osWaitForever);
		HAL_StatusTypeDef st = BMP280_Read(&hi2c1, &calib, &baro);
	    osMutexRelease(i2c_mutex);

	    if (st == HAL_OK)                       /* never publish a failed read */
	    {
	    	if (ground_n < BARO_GROUND_SAMPLES)
	    	{
	    		/* Average the take-off height. Sea-level altitude drifts with the
	    		 * weather; height above the take-off point is what flying needs. */
	    		ground_alt += baro.altitude;
	    		if (++ground_n == BARO_GROUND_SAMPLES)
	    		{
	    			ground_alt /= BARO_GROUND_SAMPLES;
	    			osMutexAcquire(uart_mutex, osWaitForever);
	    			printf("[BARO] ground reference %.2f m\r\n", ground_alt);
	    			osMutexRelease(uart_mutex);
	    		}
	    	}
	    	else
	    	{
	    		baro.altitude -= ground_alt;    /* height above take-off */
	    		xQueueOverwrite((QueueHandle_t)baro_queue, &baro);
	    	}
	    }
	    osDelay(100);
	 }
}
