/*
 * task_gps.c
 *
 *  Created on: Aug 9, 2026
 *      Author: akanb
 */


#include "app_types.h"
#include "app_globals.h"
#include "drivers/gps.h"
#include "usart.h"
#include "cmsis_os.h"
#include <string.h>
#include <stdio.h>

#define GPS_BUF_Size 128

static char gps_line[GPS_BUF_Size];
static uint8_t gps_rx_byte;
static uint8_t gps_idx = 0;
volatile uint32_t gps_byte_count = 0;

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart->Instance != USART1) return;
	gps_byte_count++;
    char c = (char)gps_rx_byte;

    if (c == '\n' || gps_idx >= GPS_BUF_Size - 1)
	{
        gps_line[gps_idx] = '\0';
        gps_idx = 0;
	 }else if (c != '\r')
	 {
		 gps_line[gps_idx++] = c;
     }

	    // Re-arm interrupt for next byte
    HAL_UART_Receive_IT(&huart1, &gps_rx_byte, 1);
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance != USART1) return;

    __HAL_UART_CLEAR_OREFLAG(huart);          // clear the overrun flag
    HAL_UART_Receive_IT(&huart1, &gps_rx_byte, 1);  // re-arm reception
}

void task_gps(void *argument)
{

	while (gps_queue == NULL || uart_mutex == NULL) osDelay(1);

	osMutexAcquire(uart_mutex, osWaitForever);
	printf("[GPS] Task started\r\n");
	osMutexRelease(uart_mutex);

//	osMutexAcquire(uart_mutex, osWaitForever);
//	printf("[GPS] ErrorCode: 0x%lX  RxState: %ld\r\n", huart1.ErrorCode, huart1.RxState);
//	osMutexRelease(uart_mutex);
	HAL_UART_Receive_IT(&huart1, &gps_rx_byte, 1);

	GPSData_t gps = {0};
    char local_line[GPS_BUF_Size];

    for (;;)
       {
    	if (logging_enabled)
    	{
    		osMutexAcquire(uart_mutex, osWaitForever);
    	    printf("[GPS] bytes:%lu line:%s\r\n", gps_byte_count, gps_line);
    	    osMutexRelease(uart_mutex);
    	}
           // Check if a complete line is ready
           if (gps_line[0] == '$')
           {
               // Copy atomically
               __disable_irq();
               strncpy(local_line, gps_line, GPS_BUF_Size);
               gps_line[0] = '\0';  // clear after copy
               __enable_irq();

               if (GPS_ParseGGA(local_line, &gps) == 1)
               {
                   osMessageQueuePut(gps_queue, &gps, 0, 0);

                   if (logging_enabled)
                   {
                       osMutexAcquire(uart_mutex, osWaitForever);
                       printf("[GPS] Lat:%.6f Lon:%.6f Sats:%d\r\n",
                              gps.latitude, gps.longitude, gps.satellites);
                       osMutexRelease(uart_mutex);
                   }
               }
           }
           osDelay(100);
       }
}
