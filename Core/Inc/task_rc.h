/*
 * task_rc.h
 *
 *  Created on: Aug 13, 2026
 *      Author: akanb
 */

#ifndef TASK_RC_H
#define TASK_RC_H

void task_rc(void *argument);
extern volatile uint32_t rc_isr_count;
extern volatile uint32_t rc_good_pkts;   // checksum OK
extern volatile uint32_t rc_bad_pkts;   // checksum failed
extern volatile uint32_t rc_overwritten;   // ISR finished a packet before task read the last one
extern volatile uint32_t rc_uart_errors;   // ORE/FE/NE/PE from error callback
extern volatile uint32_t rc_last_err;   // last HAL ErrorCode seen
extern volatile uint32_t rc_failsafe_count;

#endif /* INC_TASK_RC_H_ */
