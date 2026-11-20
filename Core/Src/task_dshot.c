/*
 * task_dshot.c
 *
 *  Created on: Sep 20, 2026
 *      Author: akanb
 */


#include "app_globals.h"
#include "cmsis_os.h"
#include "flight/dshot.h"
#include "tim.h"

static uint32_t beep_tick = 0;

void task_dshot(void *argument)
{
	DSHOT_Init();
    uint32_t next_wake = osKernelGetTickCount();
    for (;;)
    {
        next_wake += 1;                 /* 1 ms = 1 kHz */
        //if (++beep_tick >= 3000) { beep_tick = 0; DSHOT_SendCommand(1, 10); }
        DSHOT_Transmit();
        //DSHOT_TransmitDebug();      /* was DSHOT_Transmit() */
        if (osDelayUntil(next_wake) != osOK)
            next_wake = osKernelGetTickCount();
    }
}

//void task_dshot(void *argument)
//{
//    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 1000);
//    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 1000);
//    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 1000);
//    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 1000);
//
//    osDelay(4000);
//
//    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 1300);
//    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 1300);
//    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 1300);
//    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 1300);
//
//    for (;;) osDelay(1000);
//}
