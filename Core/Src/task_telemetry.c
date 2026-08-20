/*
 * task_telemetry.c
 *
 *  Created on: Aug 15, 2026
 *      Author: akanb
 */


#include "app_types.h"
#include "app_globals.h"
#include "usart.h"
#include "cmsis_os.h"
#include <stdio.h>


void task_telemetry(void *argument){

	while(uart_mutex == NULL || telemetry_queue == NULL ) osDelay(1);

	osMutexAcquire(uart_mutex, osWaitForever);
	printf("[TEL] Task Started\r\n");
	osMutexRelease(uart_mutex);

	TelemetryPckt_t tele;

	for (;;)
	{
		if (osMessageQueueGet(telemetry_queue, &tele, NULL, osWaitForever) == osOK)
		{
			HAL_UART_Transmit(&hlpuart1, (uint8_t *)&tele, sizeof(TelemetryPckt_t), HAL_MAX_DELAY);
		}
		osDelay(100);
	}
}
