/*
 * dshot.c
 *
 *  Created on: Aug 20, 2026
 *      Author: akanb
 */

#include "flight/dshot.h"
#include <stdint.h>
#include <stdio.h>


uint32_t dshot_buf_ch1[DSHOT_BUFFER_LENGTH];
uint32_t dshot_buf_ch2[DSHOT_BUFFER_LENGTH];
uint32_t dshot_buf_ch3[DSHOT_BUFFER_LENGTH];
uint32_t dshot_buf_ch4[DSHOT_BUFFER_LENGTH];

static volatile float last_m[4] = {0, 0, 0, 0};
static volatile uint8_t last_armed = 0;

volatile uint32_t dshot_start_fail[4] = {0};
volatile uint32_t dshot_busy_skip = 0;
volatile uint32_t dshot_sends = 0;

volatile uint16_t dshot_cmd_value  = 0;
volatile uint32_t dshot_cmd_frames = 0;

void DSHOT_SendCommand(uint16_t cmd, uint32_t repeats)
{
    dshot_cmd_value  = cmd;
    dshot_cmd_frames = repeats;
}

/* Called by task_control: store only, don't transmit. */
void DSHOT_SetMotors(float m1, float m2, float m3, float m4, uint8_t armed)
{
    last_m[0] = m1; last_m[1] = m2; last_m[2] = m3; last_m[3] = m4;
    last_armed = armed;
}

/* Called by the 1 kHz task: transmit the stored values. */
void DSHOT_Transmit(void)
{
    DSHOT_SendMotors(last_m[0], last_m[1], last_m[2], last_m[3], last_armed);
}

uint16_t DSHOT_ThrottleToValue(float throttle_pct, uint8_t armed)
{
	if (!armed || throttle_pct < 1.0f) return DSHOT_DISARMED;



	uint16_t val = (uint16_t) (DSHOT_MIN_THROTTLE + (throttle_pct / 100.0f) * (DSHOT_MAX_THROTTLE - DSHOT_MIN_THROTTLE));

	if (val > DSHOT_MAX_THROTTLE) val = DSHOT_MAX_THROTTLE;
	return val;
}

uint16_t DSHOT_MakePacket(uint16_t throttle, uint8_t telemetry)
{
	uint16_t packet = (throttle << 1) | (telemetry & 1);
	uint16_t crc = (packet ^ (packet >> 4) ^ (packet >> 8)) & 0x0F;
    return (packet << 4) | crc;
}

void DSHOT_PrepareDMABuffer(uint16_t packet, uint32_t *buf)
{
	for (int i = 15; i > -1; i--)
	{
		if (packet & (1 << i))
			{
				buf[15-i] = DSHOT_T1H;
			} else {
				buf[15-i] = DSHOT_T0H;
			}

	}
	buf[16] = 0;
	buf[17] = 0;
}

//volatile uint32_t dshot_start_fail[4] = {0};
//volatile uint32_t dshot_sends = 0;
//
//void DSHOT_SendMotors(float m1, float m2, float m3, float m4, uint8_t armed)
//{
//
////	uint8_t telemetry = 0;
////	uint16_t val1 = DSHOT_ThrottleToValue(m1, armed);
////	uint16_t dm1 = DSHOT_MakePacket(val1, telemetry);
////	DSHOT_PrepareDMABuffer(dm1, dshot_buf_ch1);
////
////	uint16_t val2 = DSHOT_ThrottleToValue(m2, armed);
////	uint16_t dm2 = DSHOT_MakePacket(val2, telemetry);
////	DSHOT_PrepareDMABuffer(dm2, dshot_buf_ch2);
////
////	uint16_t val3 = DSHOT_ThrottleToValue(m3, armed);
////	uint16_t dm3 = DSHOT_MakePacket(val3, telemetry);
////	DSHOT_PrepareDMABuffer(dm3, dshot_buf_ch3);
////
////	uint16_t val4 = DSHOT_ThrottleToValue(m4, armed);
////	uint16_t dm4 = DSHOT_MakePacket(val4, telemetry);
////	DSHOT_PrepareDMABuffer(dm4, dshot_buf_ch4);
//
//    uint16_t dm1, dm2, dm3, dm4;
//
//    if (dshot_cmd_frames > 0)
//    {
//        dshot_cmd_frames--;
//        /* commands are sent with the telemetry bit set */
//        uint16_t pkt = DSHOT_MakePacket(dshot_cmd_value, 1);
//        dm1 = dm2 = dm3 = dm4 = pkt;
//    }
//    else
//    {
//        uint8_t telemetry = 0;
//        dm1 = DSHOT_MakePacket(DSHOT_ThrottleToValue(m1, armed), telemetry);
//        dm2 = DSHOT_MakePacket(DSHOT_ThrottleToValue(m2, armed), telemetry);
//        dm3 = DSHOT_MakePacket(DSHOT_ThrottleToValue(m3, armed), telemetry);
//        dm4 = DSHOT_MakePacket(DSHOT_ThrottleToValue(m4, armed), telemetry);
//    }
//
//    DSHOT_PrepareDMABuffer(dm1, dshot_buf_ch1);
//    DSHOT_PrepareDMABuffer(dm2, dshot_buf_ch2);
//    DSHOT_PrepareDMABuffer(dm3, dshot_buf_ch3);
//    DSHOT_PrepareDMABuffer(dm4, dshot_buf_ch4);
//
//	 HAL_TIM_PWM_Stop_DMA(&htim1, TIM_CHANNEL_1);
//	 HAL_TIM_PWM_Stop_DMA(&htim1, TIM_CHANNEL_2);
//	 HAL_TIM_PWM_Stop_DMA(&htim1, TIM_CHANNEL_3);
//	 HAL_TIM_PWM_Stop_DMA(&htim1, TIM_CHANNEL_4);
//
//	 if (HAL_TIM_PWM_Start_DMA(&htim1, TIM_CHANNEL_1, dshot_buf_ch1, DSHOT_BUFFER_LENGTH) != HAL_OK)
//		 dshot_start_fail[0]++;
//	 if (HAL_TIM_PWM_Start_DMA(&htim1, TIM_CHANNEL_2, dshot_buf_ch2, DSHOT_BUFFER_LENGTH) != HAL_OK)
//		 dshot_start_fail[1]++;
//	 if (HAL_TIM_PWM_Start_DMA(&htim1, TIM_CHANNEL_3, dshot_buf_ch3, DSHOT_BUFFER_LENGTH) != HAL_OK)
//		 dshot_start_fail[2]++;
//	 if (HAL_TIM_PWM_Start_DMA(&htim1, TIM_CHANNEL_4, dshot_buf_ch4, DSHOT_BUFFER_LENGTH) != HAL_OK)
//		 dshot_start_fail[3]++;
//
//	 dshot_sends++;
//}


/* Call once, after HAL_TIM_PWM_Start() has started all four channels. */
void DSHOT_Init(void)
{
    uint16_t pkt = DSHOT_MakePacket(DSHOT_DISARMED, 0);
    DSHOT_PrepareDMABuffer(pkt, dshot_buf_ch1);
    DSHOT_PrepareDMABuffer(pkt, dshot_buf_ch2);
    DSHOT_PrepareDMABuffer(pkt, dshot_buf_ch3);
    DSHOT_PrepareDMABuffer(pkt, dshot_buf_ch4);

    /* CC DMA requests stay on permanently; a frame is sent by arming a stream */
    __HAL_TIM_ENABLE_DMA(&htim1, TIM_DMA_CC1 | TIM_DMA_CC2 | TIM_DMA_CC3 | TIM_DMA_CC4);
}

static uint8_t dshot_streams_ready(void)
{
    return htim1.hdma[TIM_DMA_ID_CC1]->State == HAL_DMA_STATE_READY &&
           htim1.hdma[TIM_DMA_ID_CC2]->State == HAL_DMA_STATE_READY &&
           htim1.hdma[TIM_DMA_ID_CC3]->State == HAL_DMA_STATE_READY &&
           htim1.hdma[TIM_DMA_ID_CC4]->State == HAL_DMA_STATE_READY;
}

static void dshot_arm(uint16_t dma_id, uint32_t *buf, volatile uint32_t *ccr, int idx)
{
    if (HAL_DMA_Start_IT(htim1.hdma[dma_id], (uint32_t)buf, (uint32_t)ccr,
                         DSHOT_BUFFER_LENGTH) != HAL_OK)
        dshot_start_fail[idx]++;
}

#define DSHOT_DMA_REQS (TIM_DIER_CC1DE | TIM_DIER_CC2DE | TIM_DIER_CC3DE | TIM_DIER_CC4DE)

/* OneShot125: tick count equals the ESC's stored PPM microseconds (1148–1832),
 * scaled by 1/8 into 143.5–229 µs. 1000 ticks = 125 µs = stop. */
static uint32_t oneshot_ticks(float pct, uint8_t armed)
{
    if (!armed || pct < 1.0f) return 1000u;
    if (pct > 100.0f) pct = 100.0f;
    return 1148u + (uint32_t)(pct * 6.84f);
}

void DSHOT_SendMotors(float m1, float m2, float m3, float m4, uint8_t armed)
{
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, oneshot_ticks(m1, armed));
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, oneshot_ticks(m2, armed));
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, oneshot_ticks(m3, armed));
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, oneshot_ticks(m4, armed));
    dshot_sends++;
}

//void DSHOT_SendMotors(float m1, float m2, float m3, float m4, uint8_t armed)
//{
//    /* never touch a buffer while its DMA is still sending */
//    if (!dshot_streams_ready()) { dshot_busy_skip++; return; }
//
//    uint16_t dm1, dm2, dm3, dm4;
//
//    if (dshot_cmd_frames > 0)
//    {
//        dshot_cmd_frames--;
//        uint16_t pkt = DSHOT_MakePacket(dshot_cmd_value, 1);
//        dm1 = dm2 = dm3 = dm4 = pkt;
//    }
//    else
//    {
//        dm1 = DSHOT_MakePacket(DSHOT_ThrottleToValue(m1, armed), 0);
//        dm2 = DSHOT_MakePacket(DSHOT_ThrottleToValue(m2, armed), 0);
//        dm3 = DSHOT_MakePacket(DSHOT_ThrottleToValue(m3, armed), 0);
//        dm4 = DSHOT_MakePacket(DSHOT_ThrottleToValue(m4, armed), 0);
//    }
//
//    DSHOT_PrepareDMABuffer(dm1, dshot_buf_ch1);
//    DSHOT_PrepareDMABuffer(dm2, dshot_buf_ch2);
//    DSHOT_PrepareDMABuffer(dm3, dshot_buf_ch3);
//    DSHOT_PrepareDMABuffer(dm4, dshot_buf_ch4);
//
//    CLEAR_BIT(htim1.Instance->DIER, DSHOT_DMA_REQS);      /* drop any stale request */
//
//    dshot_arm(TIM_DMA_ID_CC1, dshot_buf_ch1, &htim1.Instance->CCR1, 0);
//    dshot_arm(TIM_DMA_ID_CC2, dshot_buf_ch2, &htim1.Instance->CCR2, 1);
//    dshot_arm(TIM_DMA_ID_CC3, dshot_buf_ch3, &htim1.Instance->CCR3, 2);
//    dshot_arm(TIM_DMA_ID_CC4, dshot_buf_ch4, &htim1.Instance->CCR4, 3);
//
//    SET_BIT(htim1.Instance->DIER, DSHOT_DMA_REQS);        /* start all four cleanly */
//    dshot_sends++;
//
//
//}


/* ============ TEMPORARY: on-chip capture of one DShot frame on PA10 ============ */
#define CAP_MAX 48
static volatile uint32_t cap_t[CAP_MAX];
static volatile uint8_t  cap_lvl[CAP_MAX];
static volatile uint32_t cap_n = 0;
static volatile uint16_t cap_expect = 0;
volatile uint8_t cap_done = 0;
static uint32_t armed_ticks = 0;

/* Call every 1 ms from task_dshot instead of DSHOT_Transmit() */
void DSHOT_TransmitDebug(void)
{
    armed_ticks = last_armed ? armed_ticks + 1 : 0;

    if (cap_done || armed_ticks != 3000 || dshot_cmd_frames > 0)
    {
        DSHOT_Transmit();
        return;
    }

    cap_expect = DSHOT_MakePacket(DSHOT_ThrottleToValue(last_m[2], last_armed), 0);

    __disable_irq();
    CLEAR_BIT(htim1.Instance->DIER, TIM_DIER_CC3DE);      /* hold CH3's frame */
    DSHOT_SendMotors(last_m[0], last_m[1], last_m[2], last_m[3], last_armed);
    uint32_t prev = GPIOA->IDR & GPIO_PIN_10;             /* line is idle-low here */
    uint32_t t0   = DWT->CYCCNT;
    SET_BIT(htim1.Instance->DIER, TIM_DIER_CC3DE);        /* release it */
    uint32_t n    = 0;
    while ((DWT->CYCCNT - t0) < 168u * 120u)
    {
        uint32_t now = GPIOA->IDR & GPIO_PIN_10;
        if (now != prev)
        {
            if (n < CAP_MAX) { cap_t[n] = DWT->CYCCNT - t0; cap_lvl[n] = now ? 1 : 0; n++; }
            prev = now;
        }
    }
    __enable_irq();

    cap_n = n;
    cap_done = 1;
}

/* Call from a low-priority task holding the UART mutex */
void DSHOT_PrintCapture(void)
{
    uint16_t got = 0;
    int bits = 0;


    printf("[CAP] edges:%lu first:%s expect:0x%04X\r\n[CAP] high ns:",
           (unsigned long)cap_n, (cap_n && cap_lvl[0]) ? "rise" : "fall", cap_expect);
    for (uint32_t i = 0; i + 1 < cap_n; i++)
    {
        if (cap_lvl[i] == 1)
        {
            uint32_t ns = (cap_t[i + 1] - cap_t[i]) * 1000u / 168u;
            printf(" %lu", (unsigned long)ns);
            if (bits < 16) { got = (uint16_t)((got << 1) | (ns > 1875u ? 1u : 0u)); bits++; }
        }
    }
    printf("\r\n[CAP] bits:%d got:0x%04X -> %s\r\n", bits, got,
           (bits == 16 && got == cap_expect) ? "MATCH" : "MISMATCH");
}
