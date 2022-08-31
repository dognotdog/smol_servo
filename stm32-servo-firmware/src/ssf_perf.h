#ifndef SSF_PERF_H
#define SSF_PERF_H

#include "ssf_perf_counters.h"


#include "main.h"
#include "ssf_main.h"

#if defined(STM32G431xx) || defined(STM32G491xx)
	#include "stm32g4xx_hal.h"
#else
	#include "stm32f0xx_hal.h"
#endif

#include <stddef.h>
#include <stdint.h>

#ifdef PERF_MONITOR_ENABLED

// select which timer to use
#if defined(STM32G431xx) || defined(STM32G491xx)
	#define PERF_TIM 		TIM6
	#define PERF_EVENT_TIM	TIM1
#endif



typedef struct {
	uint32_t tickTime;
	uint32_t numCalls;	// number of calls made
	uint64_t totalTime;	// cumulative runtime
	uint32_t minTime;
	uint32_t maxTime;
} perf_channel_t;

typedef struct {
	uint64_t 	upTime;	// cumulative counter
	uint16_t	counterLast;
	uint32_t	counter1us;

	perf_channel_t channels[PERFC_NUM];
} perf_state_t;

extern perf_state_t perf_state;


void perf_init(void);
void perf_highFreqUpdate(void);

uint32_t perf_avgRuntime_us(const perf_counter_t counterId);
uint32_t perf_minRuntime_us(const perf_counter_t counterId);
uint32_t perf_maxRuntime_us(const perf_counter_t counterId);
uint32_t perf_totalRuntime_ms(const perf_counter_t counterId);

static inline uint16_t perf_getTimerValue(void)
{
	return PERF_TIM->CNT;
}

static inline uint16_t perf_getEventCount(void)
{
	return PERF_EVENT_TIM->CNT;
}


static inline uint32_t perf_now(void)
{
	// use the counter1m as a base for counting up because the timer is only 16bit, and doesn't have enough range for us resolution
	// have to get the state atomically, in case the high-frequency update interrupts us
	__disable_irq();
	uint16_t count = perf_getTimerValue();
	uint16_t last = perf_state.counterLast;
	uint32_t c1us = perf_state.counter1us;
	__enable_irq();

	uint16_t delta = (uint16_t)count - (uint16_t)last;
	return c1us + (uint32_t)delta;
}

static inline void perf_tick(const perf_counter_t counterId)
{
	perf_state.channels[counterId].tickTime = perf_now();
}



static inline void perf_tock(const perf_counter_t counterId)
{
	uint32_t tock = perf_now();

	uint32_t delta = tock - perf_state.channels[counterId].tickTime;

	perf_state.channels[counterId].minTime = umin(perf_state.channels[counterId].minTime, delta);
	perf_state.channels[counterId].maxTime = umax(perf_state.channels[counterId].maxTime, delta);

	perf_state.channels[counterId].totalTime += delta;
	perf_state.channels[counterId].numCalls++;

}

#else // PERF_MONITOR_ENABLED is false, disable perf counter

#define perf_init()
#define perf_tick(...)
#define perf_tock(...)

static inline uint32_t perf_now(void)
{
	uint32_t now = HAL_GetTick();
	return now*1000;
}

static inline uint16_t perf_getEventCount(void) { return 0; }

#endif // PERF_MONITOR_ENABLED

#endif // SSF_PERF_H
