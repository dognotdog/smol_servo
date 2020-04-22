
#include "utime.h"
#include "ssf_main.h"

utime_state_t utime_state = {};


void utime_init(void)
{
	// microsecond counter
	HAL_TIM_Base_Start(&htim6);
	// event counter
	HAL_TIM_Base_Start(&htim1);

}

void utime_sysTickUpdate(void)
{
	uint16_t count = utime_GetTime();

	uint16_t delta = (uint16_t)count - (uint16_t)utime_state.counterLast;

	utime_state.upTime += delta;
	utime_state.counter1us += delta;
	utime_state.counterLast = count;

}


