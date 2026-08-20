/*
 * dshot.h
 *
 *  Created on: Aug 20, 2026
 *      Author: akanb
 */

#ifndef FLIGHT_DSHOT_H
#define FLIGHT_DSHOT_H

#include "stm32l4xx_hal.h"
#include <stdint.h>
#include "tim.h"

#define DSHOT_AR 265
#define DSHOT_T0H 99
#define DSHOT_T1H 199

#define DSHOT_DISARMED       0
#define DSHOT_MIN_THROTTLE   48
#define DSHOT_MAX_THROTTLE   2047

#define DSHOT_FRAME_LENGTH 16
#define DSHOT_BUFFER_LENGTH (DSHOT_FRAME_LENGTH + 2)

extern uint32_t dshot_buf_ch1[DSHOT_BUFFER_LENGTH];
extern uint32_t dshot_buf_ch2[DSHOT_BUFFER_LENGTH];
extern uint32_t dshot_buf_ch3[DSHOT_BUFFER_LENGTH];
extern uint32_t dshot_buf_ch4[DSHOT_BUFFER_LENGTH];

uint16_t DSHOT_ThrottleToValue(float throttle_pct, uint8_t armed);
uint16_t DSHOT_MakePacket(uint16_t throttle, uint8_t telemetry);
void DSHOT_PrepareDMABuffer(uint16_t packet, uint32_t *buf);
void DSHOT_SendMotors(float m1, float m2, float m3, float m4, uint8_t armed);
#endif /* INC_FLIGHT_DSHOT_H_ */
