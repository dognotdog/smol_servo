
#include "debug.h"

#include "main.h"

// extern typedefs from main.c
extern TIM_HandleTypeDef htim4;


void ssf_ledInit(void)
{
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 0);
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, 0);
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 0);
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, 0);

	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);  
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);  
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);  
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);  
}


void ssf_ledIdle(void)
{
	static unsigned counter = 0;

	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, (counter/256) % 65536);
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, (counter/256+21845) % 65536);
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, (counter/256+43691) % 65536);
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, (counter/256) % 65536);

	++counter;
}