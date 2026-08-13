/*
 * task_log.c
 *
 *  Created on: Jul 17, 2026
 *      Author: akanb
 */


#include "app_globals.h"
#include "cmsis_os.h"
#include <stdio.h>
#include "gpio.h"
#include "main.h"


#define ARM_HOLD_MS    2000
#define LOG_HOLD_MS    200

void task_log(void *argument)
{
    while (uart_mutex == NULL || button_sem == NULL) osDelay(1);

    for (;;)
    {
        osSemaphoreAcquire(button_sem, osWaitForever);

        uint32_t hold_start = osKernelGetTickCount();
        while (HAL_GPIO_ReadPin(B1_button_GPIO_Port, B1_button_Pin) == GPIO_PIN_RESET)
        {
        	osDelay(10);
        }

        uint32_t hold_ms = osKernelGetTickCount() - hold_start;

        if (hold_ms >=  ARM_HOLD_MS)
        {
        	if (throttle > 5.0f)
        	{
        		osMutexAcquire(uart_mutex, osWaitForever);
        		printf("[ARM] REFUSED — throttle not zero\r\n");
        		osMutexRelease(uart_mutex);
        	} else {
        		armed = !armed;
        		osMutexAcquire(uart_mutex, osWaitForever);
        		printf("[ARM] %s\r\n", armed ?  "ARMED" : "DISARMED");
        		osMutexRelease(uart_mutex);
        	}
        } else if (hold_ms >= LOG_HOLD_MS)
        {
        	logging_enabled = !logging_enabled;

        	osMutexAcquire(uart_mutex, osWaitForever);
        	printf(logging_enabled ? "[LOG] Logging ON\r\n" : "[LOG] Logging OFF\r\n");
        	osMutexRelease(uart_mutex);
        }

    }
}
