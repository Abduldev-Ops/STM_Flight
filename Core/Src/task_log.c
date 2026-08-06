/*
 * task_log.c
 *
 *  Created on: Jul 17, 2026
 *      Author: akanb
 */


#include "app_globals.h"
#include "cmsis_os.h"
#include <stdio.h>

void task_log(void *argument)
{
    while (uart_mutex == NULL || button_sem == NULL) osDelay(1);

    for (;;)
    {
        osSemaphoreAcquire(button_sem, osWaitForever);
        logging_enabled = !logging_enabled;

        osMutexAcquire(uart_mutex, osWaitForever);
        printf(logging_enabled ? "[LOG] Logging ON\r\n" : "[LOG] Logging OFF\r\n");
        osMutexRelease(uart_mutex);
    }
}
