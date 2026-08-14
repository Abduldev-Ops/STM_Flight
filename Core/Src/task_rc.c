/*
 * task_rc.c
 *
 *  Created on: Aug 13, 2026
 *      Author: akanb
 */


#include "app_types.h"
#include "app_globals.h"
#include "usart.h"
#include "cmsis_os.h"
#include <string.h>
#include <stdio.h>

#define PACKET_START 0XAB
#define RC_FAILSAFE_MS 500

typedef struct __attribute__((packed)){
	uint8_t  start;
    float    throttle;
	float    roll;
    float    pitch;
    float    yaw;
    uint8_t  armed;
    uint8_t  checksum;
} RCPacket_t;

uint8_t rcrx_byte;
uint8_t rc_buf[sizeof(RCPacket_t)];
uint8_t rc_idx = 0;
uint8_t rc_synced = 0;
volatile uint8_t rc_packet_ready = 0;

static uint8_t calcCheckSum(RCPacket_t *pkt)
{
	uint8_t *bytes = (uint8_t *) pkt;
	uint8_t chk = 0;
	for (int i = 1; i < sizeof(RCPacket_t) - 1; i++)
		chk ^= bytes[i];
	return chk;
}

void HAL_UART_RxCpltCallback_RC(UART_HandleTypeDef *huart)
{
    if (huart->Instance != LPUART1) return;

    if (!rc_synced)
    {
        // Hunt for start byte
        if (rcrx_byte == PACKET_START)
        {
            rc_buf[0] = rcrx_byte;
            rc_idx = 1;
            rc_synced = 1;
        }
    }
    else
    {
        rc_buf[rc_idx++] = rcrx_byte;
        if (rc_idx >= sizeof(RCPacket_t))
        {
            rc_idx = 0;
            rc_synced = 0;  // re-sync on next packet
            rc_packet_ready = 1;
        }
    }

    HAL_UART_Receive_IT(&hlpuart1, &rcrx_byte, 1);
}


void task_rc(void *argument)
{
	while (rc_queue == NULL || uart_mutex == NULL) osDelay(1);

	osMutexAcquire(uart_mutex, osWaitForever);
	printf("[RC] Task Started\r\n");
	osMutexRelease(uart_mutex);

	// Arm interrupt — single call only
	HAL_StatusTypeDef rc_status = HAL_BUSY;
	while (rc_status != HAL_OK)
	{
	   osDelay(10);
	   rc_status = HAL_UART_Receive_IT(&hlpuart1, &rcrx_byte, 1);
	}

	osMutexAcquire(uart_mutex, osWaitForever);
	printf("[RC] LPUART1 armed OK\r\n");
	osMutexRelease(uart_mutex);

	RCPacket_t *pkt;
	RCInput_t rc = {0};
	uint8_t failsafe_active = 0;

	for (;;)
	{
		// RC failsafe — if no packet for 500ms, disarm and zero throttle
		if (osKernelGetTickCount() - rc_last_packet_ms > RC_FAILSAFE_MS && rc_last_packet_ms > 0)
	    {
			if (!failsafe_active)
			{
				failsafe_active = 1;
				armed = 0;
				throttle = 0.0f;
				osMutexAcquire(uart_mutex, osWaitForever);
				printf("[RC] FAILSAFE — link lost, DISARMED\r\n");
				osMutexRelease(uart_mutex);
			}
	    } else if (rc_last_packet_ms > 0){
	    	failsafe_active = 0;
	    }

	    if (rc_packet_ready)
	    {
	        rc_packet_ready = 0;  // clear flag first

	        uint8_t local_buf[sizeof(RCPacket_t)];
	        __disable_irq();
	        memcpy(local_buf, rc_buf, sizeof(RCPacket_t));
	        __enable_irq();

	        pkt = (RCPacket_t *)local_buf;

	        uint8_t expected = calcCheckSum(pkt);

	        if (pkt->checksum == expected)
	        {
	            rc.throttle = pkt->throttle;
	            rc.roll     = pkt->roll;
	            rc.pitch    = pkt->pitch;
	            rc.yaw      = pkt->yaw;
	            rc.armed    = pkt->armed;

	            armed = rc.armed;
	            throttle = rc.throttle;
	            rc_last_packet_ms = osKernelGetTickCount();
	            failsafe_active = 0;

	            osMessageQueuePut(rc_queue, &rc, 0, 0);

	            static uint32_t last_debug = 0;
	            uint32_t now = osKernelGetTickCount();
	            if (now - last_debug > 1000)
	            {
	                last_debug = now;
	                osMutexAcquire(uart_mutex, osWaitForever);
	                printf("[RC] age:%lu ready:%d\r\n",
	                       now - rc_last_packet_ms, rc_packet_ready);
	                osMutexRelease(uart_mutex);
	            }
	        }
	    }
	    osDelay(5);
	}
}
