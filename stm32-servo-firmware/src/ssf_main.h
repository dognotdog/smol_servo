#ifndef SSF_MAIN_H
#define SSF_MAIN_H

#include <stdint.h>
#include <math.h>

#include "stm32g4xx_hal.h"


typedef enum {
	SCHED_NONE,
	SCHED_UI_FAST,
	SCHED_COUNT
} sch_taskId_t;

typedef void (*sch_taskFun_t)(uint32_t now_us);


typedef struct {
	sch_taskFun_t 	taskFun;
	int32_t		period_us, offset_us;
} ssf_scheduledTask_t;




extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2; // current sense
extern DMA_HandleTypeDef hdma_adc1;
extern DMA_HandleTypeDef hdma_adc2;


extern TIM_HandleTypeDef htim2;	// 3 Phase + Brake Resistor PWM
extern TIM_HandleTypeDef htim3;	// ABI Encoder Input
extern TIM_HandleTypeDef htim4;	// RGB LED

extern TIM_HandleTypeDef htim6; // us timer

extern TIM_HandleTypeDef htim15;	// current sense offset

#define ADC1_NOMINAL_MAXCOUNT 		4095
#define ADC1_SHIFT					4
#define ADC1_OVERSAMPLING_COUNT		256
#define ADC1_OVERSAMPLING_FACTOR	(ADC1_OVERSAMPLING_COUNT >> ADC1_SHIFT)

#define ADC2_NOMINAL_MAXCOUNT 		4095
#define ADC2_SHIFT					2
#define ADC2_OVERSAMPLING_COUNT		16
#define ADC2_OVERSAMPLING_FACTOR	(ADC2_OVERSAMPLING_COUNT >> ADC2_SHIFT)

#define ISENSE_SAMPLE_ADC_CYCLES	25 // 12.5 sample + 12.5 conversion
#define ISENSE_SSAA_ADC_CYCLES		(ISENSE_SAMPLE_CYCLES*ADC2_OVERSAMPLING_COUNT)
#define ISENSE_SSAA_CPU_CYCLES		(ISENSE_SSAA_ADC_CYCLES*3)


#define PIN_DRVEN		GPIOA, GPIO_PIN_7
#define PIN_ENA			GPIOB, GPIO_PIN_2
#define PIN_ENB			GPIOB, GPIO_PIN_1
#define PIN_ENC			GPIOB, GPIO_PIN_0

#define HTIM_LED 		(&htim4)
#define HTIM_ENC 		(&htim3)
#define HTIM_DRV 		(&htim2)
#define HTIM_ISENSE_OFFSET	(&htim15)

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
extern float ssf_getVdda(void);

extern void mctrl_init(void);
extern void mctrl_fastLoop(const uint16_t adcCounts[6]);
extern float* mctrl_getPhaseTable(size_t i);

extern void ssf_ledInit(void);
extern void ssf_ledIdle(void);

extern void spwm_init(void);
extern void spwm_idle(void);
extern void spwm_setDrvChannel(mctrl_pwmChannelId_t ch, float normValue);

extern void ssf_uiFastTask(uint32_t now_us);
// void ssf_adc2IrqHandler(ADC_HandleTypeDef *hadc);

extern void ssf_usbRxCallback(const void* data, size_t datalen);


extern float sintab(float x);

static inline float fclampf(float a, float minv, float maxv)
{
	return fminf(fmaxf(a, minv), maxv);
}



#endif // SSF_MAIN_H
