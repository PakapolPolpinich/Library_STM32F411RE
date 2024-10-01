/*
 * Systick_timer.h
 *
 *  Created on: Sep 24, 2024
 *      Author: Admin
 */

#ifndef INC_SYSTICK_TIMER_H_
#define INC_SYSTICK_TIMER_H_

#include "stm32f411re_by_belt.h"

#define COUNTFLAG       (1U << 16)
#define CLKSOURCE       (1U << 2)
#define TICKINT         (1U << 1)
#define COUNTER_ENABLE  (1U << 0)

void Systick_init(const uint32_t f_cpu, const uint32_t Hz);
void delay_ms();
uint32_t millis();
#endif /* INC_SYSTICK_TIMER_H_ */
