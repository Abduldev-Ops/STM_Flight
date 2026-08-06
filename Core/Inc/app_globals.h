/*
 * app_globals.h
 *
 *  Created on: Jul 17, 2026
 *      Author: akanb
 */

//#ifndef INC_APP_GLOBALS_H_
//#define INC_APP_GLOBALS_H_

#ifndef APP_GLOBALS_H
#define APP_GLOBALS_H

#include "cmsis_os.h"
#include <stdint.h>

// RTOS handles — defined in freertos.c
extern osMutexId_t uart_mutex;
extern osMutexId_t i2c_mutex;
extern osMessageQueueId_t imu_queue;
extern osMessageQueueId_t attitude_queue;
extern osSemaphoreId_t button_sem;
extern volatile uint8_t logging_enabled;

#endif /* APP_GLOBALS_H */
//
//#endif /* INC_APP_GLOBALS_H_ */
