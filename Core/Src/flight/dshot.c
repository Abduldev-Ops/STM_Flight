/*
 * dshot.c
 *
 *  Created on: Aug 20, 2026
 *      Author: akanb
 */

#include "flight/dshot.h"
#include <stdint.h>

uint32_t dshot_buf_ch1[DSHOT_BUFFER_LENGTH];
uint32_t dshot_buf_ch2[DSHOT_BUFFER_LENGTH];
uint32_t dshot_buf_ch3[DSHOT_BUFFER_LENGTH];
uint32_t dshot_buf_ch4[DSHOT_BUFFER_LENGTH];


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

void DSHOT_SendMotors(float m1, float m2, float m3, float m4, uint8_t armed)
{
	uint8_t telemetry = 0;
	uint16_t val1 = DSHOT_ThrottleToValue(m1, armed);
	uint16_t dm1 = DSHOT_MakePacket(val1, telemetry);
	DSHOT_PrepareDMABuffer(dm1, dshot_buf_ch1);

	uint16_t val2 = DSHOT_ThrottleToValue(m2, armed);
	uint16_t dm2 = DSHOT_MakePacket(val2, telemetry);
	DSHOT_PrepareDMABuffer(dm2, dshot_buf_ch2);

	uint16_t val3 = DSHOT_ThrottleToValue(m3, armed);
	uint16_t dm3 = DSHOT_MakePacket(val3, telemetry);
	DSHOT_PrepareDMABuffer(dm3, dshot_buf_ch3);

	uint16_t val4 = DSHOT_ThrottleToValue(m4, armed);
	uint16_t dm4 = DSHOT_MakePacket(val4, telemetry);
	DSHOT_PrepareDMABuffer(dm4, dshot_buf_ch4);

	 HAL_TIM_PWM_Stop_DMA(&htim2, TIM_CHANNEL_1);
	 HAL_TIM_PWM_Stop_DMA(&htim2, TIM_CHANNEL_2);
	 HAL_TIM_PWM_Stop_DMA(&htim2, TIM_CHANNEL_3);
	 HAL_TIM_PWM_Stop_DMA(&htim2, TIM_CHANNEL_4);

	 HAL_TIM_PWM_Start_DMA(&htim2, TIM_CHANNEL_1, dshot_buf_ch1, DSHOT_BUFFER_LENGTH);
	 HAL_TIM_PWM_Start_DMA(&htim2, TIM_CHANNEL_2, dshot_buf_ch2, DSHOT_BUFFER_LENGTH);
	 HAL_TIM_PWM_Start_DMA(&htim2, TIM_CHANNEL_3, dshot_buf_ch3, DSHOT_BUFFER_LENGTH);
	 HAL_TIM_PWM_Start_DMA(&htim2, TIM_CHANNEL_4, dshot_buf_ch4, DSHOT_BUFFER_LENGTH);
}
