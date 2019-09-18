#ifndef UTIME_H
#define UTIME_H

#include "stm32g4xx_hal.h"

#include <stdint.h>

typedef struct {
	uint64_t 	upTime;	// cumulative counter
	uint16_t	counterLast;
	uint32_t	counter1us;

} utime_state_t;

extern utime_state_t utime_state;

static inline uint16_t utime_GetTime(void)
{
	return TIM6->CNT;
}


static inline uint32_t utime_now(void)
{
	// use the counter1m as a base for counting up because the timer is only 16bit, and doesn't have enough range for us resolution
	// have to get the state atomically, in case the high-frequency update interrupts us
	__disable_irq();
	uint16_t count = utime_GetTime();
	uint16_t last = utime_state.counterLast;
	uint32_t c1us = utime_state.counter1us;
	__enable_irq();

	uint16_t delta = (uint16_t)count - (uint16_t)last;
	return c1us + (uint32_t)delta;
}

void utime_sysTickUpdate(void);
void utime_init(void);


#endif // UTIME_H
