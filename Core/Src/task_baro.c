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
#include "cmsis_os.h"

void task_baro(void *argument)
{
	while (i2c_mutex == NULL || baro_queue == NULL) osDelay(1);

	BMP280Calib_t calib;

	//init bmp and read calib coeff
	osMutexAcquire(i2c_mutex, osWaitForever);
	BMP280_Init(&hi2c1, &calib);
	osMutexRelease(i2c_mutex);

	BarData_t baro;

	for (;;)
	{
		osMutexAcquire(i2c_mutex, osWaitForever);
	    BMP280_Read(&hi2c1, &calib, &baro);
	    osMutexRelease(i2c_mutex);

	    osMessageQueuePut(baro_queue, &baro, 0, 0);
	    osDelay(100);  // 10Hz — barometer doesn't need high rate
	 }
}
