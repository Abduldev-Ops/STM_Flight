/*
 * task_watchdog.c
 *
 *  Created on: Jul 17, 2026
 *      Author: akanb
 */


#include "iwdg.h"
#include "cmsis_os.h"

void task_watchdog(void *argument)
{
    for (;;)
    {
        HAL_IWDG_Refresh(&hiwdg);
        osDelay(500);
    }
}
