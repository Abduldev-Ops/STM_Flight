/*
 * dshot.h
 *
 *  Created on: Aug 20, 2026
 *      Author: akanb
 */

#ifndef FLIGHT_DSHOT_H
#define FLIGHT_DSHOT_H

#include "stm32f4xx_hal.h"
#include <stdint.h>
#include "tim.h"

#define DSHOT_ARR 1119
#define DSHOT_T0H 419
#define DSHOT_T1H 839

#define DSHOT_DISARMED       0
#define DSHOT_MIN_THROTTLE   48
#define DSHOT_MAX_THROTTLE   2047

#define DSHOT_FRAME_LENGTH 16
#define DSHOT_BUFFER_LENGTH (DSHOT_FRAME_LENGTH + 2)

extern uint32_t dshot_buf_ch1[DSHOT_BUFFER_LENGTH];
extern uint32_t dshot_buf_ch2[DSHOT_BUFFER_LENGTH];
extern uint32_t dshot_buf_ch3[DSHOT_BUFFER_LENGTH];
extern uint32_t dshot_buf_ch4[DSHOT_BUFFER_LENGTH];

extern volatile uint16_t dshot_cmd_value;
extern volatile uint32_t dshot_cmd_frames;
void DSHOT_SendCommand(uint16_t cmd, uint32_t repeats);

uint16_t DSHOT_ThrottleToValue(float throttle_pct, uint8_t armed);
uint16_t DSHOT_MakePacket(uint16_t throttle, uint8_t telemetry);
void DSHOT_PrepareDMABuffer(uint16_t packet, uint32_t *buf);
void DSHOT_SendMotors(float m1, float m2, float m3, float m4, uint8_t armed);
void DSHOT_SetMotors(float m1, float m2, float m3, float m4, uint8_t armed);
void DSHOT_Transmit(void);
void DSHOT_Init(void);
extern volatile uint32_t dshot_busy_skip;
void DSHOT_TransmitDebug(void);
void DSHOT_PrintCapture(void);
extern volatile uint8_t cap_done;
#endif /* INC_FLIGHT_DSHOT_H_ */
