/*
 * task_loggr.c
 *
 *  Created on: Sep 19, 2026
 *      Author: akanb
 */


#include "app_types.h"
#include "app_globals.h"
#include "cmsis_os.h"
#include "FreeRTOS.h"
#include "queue.h"
#include <stdio.h>
#include "flight/dshot.h"

void task_loggr(void *argument)
{
    while (log_queue == NULL || uart_mutex == NULL) osDelay(1);

    CtrlLog_t log;
    uint32_t next_wake = osKernelGetTickCount();

    for (;;)
    {
        next_wake += 200;                                          /* 5 Hz */

        /* Receive (not peek): if the control loop hasn't produced anything
         * new since the last print, there's nothing to print. */
        if (logging_enabled &&
            xQueueReceive((QueueHandle_t)log_queue, &log, 0) == pdTRUE)
        {
            osMutexAcquire(uart_mutex, osWaitForever);             /* fine to wait: low priority */
            printf("[%s] T:%.0f R:%.1f P:%.1f RS:%.1f PS:%.1f M1:%.0f M2:%.0f M3:%.0f M4:%.0f\r\n",
                   log.armed ? "ARM" : "DIS", log.thr, log.roll, log.pitch,
                   log.rs, log.ps, log.m1, log.m2, log.m3, log.m4);
            osMutexRelease(uart_mutex);
        }
        static uint8_t cap_printed = 0;
        if (cap_done && !cap_printed)
        {
            osMutexAcquire(uart_mutex, osWaitForever);
            DSHOT_PrintCapture();
            osMutexRelease(uart_mutex);
            cap_printed = 1;
        }
        if (osDelayUntil(next_wake) != osOK)
            next_wake = osKernelGetTickCount();
    }
}
