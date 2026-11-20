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
#define ARM_MAX_THROTTLE 5.0f

typedef struct __attribute__((packed)){
	uint8_t  start;
    float    throttle;
	float    roll;
    float    pitch;
    float    yaw;
    uint8_t  armed;
    uint8_t  checksum;
} RCPacket_t;

volatile uint32_t rc_isr_count = 0;
volatile uint32_t rc_good_pkts     = 0;   // checksum OK
volatile uint32_t rc_bad_pkts      = 0;   // checksum failed
volatile uint32_t rc_overwritten   = 0;   // ISR finished a packet before task read the last one
volatile uint32_t rc_uart_errors   = 0;   // ORE/FE/NE/PE from error callback
volatile uint32_t rc_last_err      = 0;   // last HAL ErrorCode seen
volatile uint32_t rc_failsafe_count = 0;

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

/* Called from USART3_IRQHandler for every received byte (ISR context). */
void RC_RxByte(uint8_t b)
{
    rc_isr_count++;
    if (!rc_synced)
    {
        // Hunt for start byte
        if (b == PACKET_START)
        {
            rc_buf[0] = b;
            rc_idx = 1;
            rc_synced = 1;
        }
    }
    else
    {
        rc_buf[rc_idx++] = b;
        if (rc_idx >= sizeof(RCPacket_t))
        {
            rc_idx = 0;
            rc_synced = 0;  // re-sync on next packet
            if (rc_packet_ready) rc_overwritten++;
            rc_packet_ready = 1;
        }
    }
    // no re-arm: the RX interrupt stays enabled permanently
}


void task_rc(void *argument)
{
	while (rc_queue == NULL || uart_mutex == NULL) osDelay(1);

	osMutexAcquire(uart_mutex, osWaitForever);
	printf("[RC] Task Started\r\n");
	osMutexRelease(uart_mutex);


	// Flush anything that arrived before we were ready, then enable RX interrupt once
	(void)USART3->SR;
	(void)USART3->DR;
	__HAL_UART_ENABLE_IT(&huart3, UART_IT_RXNE);

	osMutexAcquire(uart_mutex, osWaitForever);
	printf("[RC] USART3 RX enabled\r\n");
	osMutexRelease(uart_mutex);

	RCPacket_t *pkt;
	RCInput_t rc = {0};
	uint8_t failsafe_active = 0;
	uint8_t seen_disarm = 0;
	uint8_t arm_refused = 0;
	uint32_t last_print = 0;
	uint32_t prev_good  = 0;

	for (;;)
	{
	    uint32_t now = osKernelGetTickCount();
	    if (now - last_print > 2000)
	    {
	    	last_print = now;
	    	uint32_t good = rc_good_pkts;
	        osMutexAcquire(uart_mutex, osWaitForever);
	        printf("[RC] isr:%lu good:%lu (+%lu) bad:%lu ovw:%lu err:%lu last_err:0x%02lX fs:%lu\r\n",
	                   rc_isr_count, good, good - prev_good, rc_bad_pkts,
	                   rc_overwritten, rc_uart_errors, rc_last_err, rc_failsafe_count);
	        osMutexRelease(uart_mutex);
	        prev_good = good;
	    }

		// RC failsafe — if no packet for 500ms, disarm and zero throttle
		if (osKernelGetTickCount() - rc_last_packet_ms > RC_FAILSAFE_MS && rc_last_packet_ms > 0)
	    {
			if (!failsafe_active)
			{
				failsafe_active = 1;
				rc_failsafe_count++;
				armed = 0;
				throttle = 0.0f;
				seen_disarm = 0;
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
	        	rc_good_pkts++;
	            rc.throttle = pkt->throttle;
	            rc.roll     = pkt->roll;
	            rc.pitch    = pkt->pitch;
	            rc.yaw      = pkt->yaw;
	            rc.armed    = pkt->armed;

	            if (!rc.armed)
	            {
	            	armed = 0;
	            	seen_disarm = 1;
	            	arm_refused = 0;
	            } else if (!armed)
	            {
	            	if (seen_disarm && rc.throttle < ARM_MAX_THROTTLE)
	            	{
	            		armed = 1;
	            	}
	            	else if (!arm_refused)
	            	{
	            		arm_refused = 1;
	            		osMutexAcquire(uart_mutex, osWaitForever);
	            		printf("[ARM] REFUSED: %s\r\n",
	            				!seen_disarm ? "send DISARM first (boot/failsafe)" : "throttle not low");
	            		osMutexRelease(uart_mutex);
	            	}
	            }

	            throttle = rc.throttle;
	            rc_last_packet_ms = osKernelGetTickCount();
	            failsafe_active = 0;

	            osMessageQueuePut(rc_queue, &rc, 0, 0);
	        } else {
	        	rc_bad_pkts++;
	        }
	    }
	    osDelay(5);
	}
}
