#ifndef SSF_MAIN_H
#define SSF_MAIN_H

#include "ssf_scheduler.h"

#include "stm32g4xx_hal.h"

#include <stdint.h>
#include <math.h>






extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2; // current sense
extern DMA_HandleTypeDef hdma_adc1;
extern DMA_HandleTypeDef hdma_adc2;


extern TIM_HandleTypeDef htim2;	// 3 Phase + Brake Resistor PWM
extern TIM_HandleTypeDef htim3;	// ABI Encoder Input
extern TIM_HandleTypeDef htim4;	// RGB LED

extern TIM_HandleTypeDef htim1;	// TIM2 event counter
extern TIM_HandleTypeDef htim6; // us timer

extern TIM_HandleTypeDef htim15;	// current sense offset
extern TIM_HandleTypeDef htim16;	// SPI timing

#define ADC1_NOMINAL_MAXCOUNT 		4095
#define ADC1_SHIFT					4
#define ADC1_OVERSAMPLING_COUNT		256
#define ADC1_OVERSAMPLING_FACTOR	(ADC1_OVERSAMPLING_COUNT >> ADC1_SHIFT)

#define ADC2_NOMINAL_MAXCOUNT 		4095
#define ADC2_SHIFT					0
#define ADC2_OVERSAMPLING_COUNT		16
#define ADC2_OVERSAMPLING_FACTOR	(ADC2_OVERSAMPLING_COUNT >> ADC2_SHIFT)

#define ISENSE_SAMPLE_ADC_CYCLES	25 // 12.5 sample + 12.5 conversion
#define ISENSE_SSAA_ADC_CYCLES		(ISENSE_SAMPLE_ADC_CYCLES*ADC2_OVERSAMPLING_COUNT)
#define ISENSE_SSAA_CPU_CYCLES		(ISENSE_SSAA_ADC_CYCLES*3)


#define PIN_DRVEN		GPIOA, GPIO_PIN_7
#define PIN_ENA			GPIOB, GPIO_PIN_2
#define PIN_ENB			GPIOB, GPIO_PIN_1
#define PIN_ENC			GPIOB, GPIO_PIN_0

#define HTIM_LED 		(&htim4)
#define HTIM_ENC 		(&htim3)
#define HTIM_DRV 		(&htim2)
#define HTIM_ISENSE_OFFSET	(&htim15)
#define HTIM_SPI		(&htim16)

typedef enum {
	HTIM_DRV_CH_R = TIM_CHANNEL_1,
	HTIM_DRV_CH_A = TIM_CHANNEL_4,
	HTIM_DRV_CH_B = TIM_CHANNEL_3,
	HTIM_DRV_CH_C = TIM_CHANNEL_2
} mctrl_pwmChannelId_t;




extern ssf_scheduledTask_t tasks[SCHED_COUNT];

extern void ssf_init(void);
extern void ssf_idle(void);

extern void ssf_analogInit(void);
extern float ssf_getVbus(void);
extern float ssf_getVddp(void);
extern float ssf_getVdda(void);


extern void ssf_ledInit(void);
extern void ssf_ledIdle(void);

extern void spwm_init(void);
extern void spwm_idle(void);
extern void spwm_setDrvChannel(mctrl_pwmChannelId_t ch, float normValue);
extern void spwm_enableHalfBridges(uint32_t outputMask);

extern void ssf_ui1sTask(uint32_t now_us);
// void ssf_adc2IrqHandler(ADC_HandleTypeDef *hadc);

extern void ssf_usbRxCallback(const void* data, size_t datalen);


extern float sintab(float x);

static inline float fclampf(float a, float minv, float maxv)
{
	return fminf(fmaxf(a, minv), maxv);
}

static inline uint32_t umin(uint32_t const a, uint32_t const b)
{
	return a < b ? a : b;
}

static inline uint32_t umax(uint32_t const a, uint32_t const b)
{
	return a > b ? a : b;
}

static inline uint32_t uclamp(uint32_t a, uint32_t minv, uint32_t maxv)
{
	return umin(umax(a, minv), maxv);
}


#endif // SSF_MAIN_H
