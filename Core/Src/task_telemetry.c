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
#include "FreeRTOS.h"
#include "queue.h"


#define TELE_PERIOD_MS  50        /* 20 Hz, matches ASSUMED_RATE_HZ in ground_gui.py */

volatile uint32_t tele_sent = 0, tele_skipped = 0;

void task_telemetry(void *argument){

	while(uart_mutex == NULL || telemetry_queue == NULL ) osDelay(1);

	TelemetryPckt_t pkt;
	uint32_t next_wake = osKernelGetTickCount();

	for (;;)
	{
		next_wake += TELE_PERIOD_MS;

		if (xQueueReceive((QueueHandle_t)telemetry_queue, &pkt, 0) == pdTRUE)
		{
			/* The whole 45-byte packet goes out under the UART mutex, so no
			 * printf text can ever be interleaved into the middle of it. */
			if (osMutexAcquire(uart_mutex, 20) == osOK)
			{
				HAL_UART_Transmit(&huart3, (uint8_t *)&pkt, sizeof(pkt), 20);
				osMutexRelease(uart_mutex);
				tele_sent++;
	        }
			else
			{
				tele_skipped++;   /* UART busy for 20 ms: drop this one, next comes in 50 ms */
	        }
	   }

	        if (osDelayUntil(next_wake) != osOK){
	            next_wake = osKernelGetTickCount();
	        }
	}
}
