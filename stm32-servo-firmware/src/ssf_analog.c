
#include "debug.h"
#include "ssf_main.h"



/*
ADC Setup

	ADCs are clocked at 170/3 MHz (56.667MHz)
	So for a single channel, fs = (170/3)/(6.5+12.5) = 2.982MHz, 11.65kHz with 256x oversampling
	With 12.5 cycles, fs = (170/3)/(12.5+12.5) - 2.267MHz, or 0.44us. 100kHz PWM means we have a 10us cycle time. 16x oversampling takes 7.06us

	Assuming 100kHz PWM, it would make sense to cycle through the channels, with each channel being measured on every 3rd PWM period, for 1/3rd the rate. Using centered PWM, measure at the center of each HIGH and LOW cycle.

	channels are 17, 13, 3 for A,B,C respectively
	ADC is setup to store A,A,B,B,C,C sequence in buffer (PWM on / off center values)

	If the ADC is setup before the PWM timer is started, then ADC is started on the first trigger, which is not the beginning of counting, but the center of the first LOW period.

*/

static uint16_t __ALIGNED(4) an0_buf[4];
static uint16_t __ALIGNED(4) an1_buf[6] = {0xF0F0, 0x00F0};
// float currentSensed[6] = {};


// static float _getCurrentSenseFactor(void)
// {
// 	// default amp gain factor is G = 20 (5/10/20/40) are available
// 	// current sense shunts are Rs = 15mOhm
// 	// sense amp output is centered on VREF/2, max. 0.25V away from rail
// 	// thus +- 1.4V usable range
// 	// for 12V operation, 10A would give 150mV, factor of 10 gives full range for 8A
// 	// return Rs * G
// 	return 1.0/(0.015f*20.0f);
// }

static void _currentSenseConversionCallback(void)
{
	// float gain = _getCurrentSenseFactor();
	// for (size_t i = 0; i < 6; ++i)
	// {
	// 	float adcCount = (float)(an1_buf[i]);
	// 	currentSensed[i] = (adcCount - 0.5f*4095.0f)*(1.0f/4095.0f)*3.3f*gain;
	// 	// currentSensed[i] = adcCount;
	// }

	mctrl_fastLoop(an1_buf);

	// spwm_idle();

}

// void HAL_ADCEx_EndOfSamplingCallback(ADC_HandleTypeDef *hadc)
void _adc2ConversionCompleteCallback(ADC_HandleTypeDef *hadc)
{
	// uint32_t isr = hadc->Instance->ISR;
	// uint32_t ier = hadc->Instance->IER;
	// if (((isr & ADC_FLAG_EOSMP) == ADC_FLAG_EOSMP) && ((ier & ADC_IT_EOSMP) == ADC_IT_EOSMP))
	{
		_currentSenseConversionCallback();
		// do not reset EOSMP flags as HAL does that
	}

}
void _adc2DmaCallback(DMA_HandleTypeDef* hdma)
{
	_currentSenseConversionCallback();
}

float ssf_getVdda(void)
{
	// Voltage reference calibration
	// VDDA = 3.0 V x VREFINT_CAL / VREFINT_DATA
	float vref_count = an0_buf[3];
	float vref_ncount = vref_count * (1.0f / ADC1_OVERSAMPLING_FACTOR);
	float vref_cal = (uint32_t)(*VREFINT_CAL_ADDR);
	float vdda_mV = (vref_cal * (float)VREFINT_CAL_VREF) / (vref_ncount);

	return vdda_mV*0.001f;
}


float ssf_getVbus(void)
{
	float vbusCount = an0_buf[0];
	float vdda = ssf_getVdda();

	return vbusCount*vdda*((120.0f+5.1f)/5.1f/(ADC1_OVERSAMPLING_FACTOR*ADC1_NOMINAL_MAXCOUNT));
}


void ssf_analogInit(void)
{
	HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);

	uint32_t calibVal1 = HAL_ADCEx_Calibration_GetValue(&hadc1, ADC_SINGLE_ENDED);

	HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED);

	uint32_t calibVal2 = HAL_ADCEx_Calibration_GetValue(&hadc2, ADC_SINGLE_ENDED);

	dbg_println("ADCs calibrated to %"PRIu32",%"PRIu32"!\r\n", calibVal1, calibVal2);


	// callback could be registered here to keep track of ADC ring buffer, but we're only going to look at the latest values for now
	// HAL_DMA_RegisterCallback(&hdma_adc2, HAL_DMA_XFER_CPLT_CB_ID, _adc2DmaCallback);

	// register an ADC scan conversion complete callback
	HAL_ADC_RegisterCallback(&hadc2, HAL_ADC_CONVERSION_COMPLETE_CB_ID, _adc2ConversionCompleteCallback);


	HAL_ADC_Start_DMA(&hadc1, (uint32_t*) an0_buf, sizeof(an0_buf)/sizeof(an0_buf[0]));
	HAL_ADC_StartSampling(&hadc1);

	HAL_ADC_Start_DMA(&hadc2, (uint32_t*) an1_buf, sizeof(an1_buf)/sizeof(an1_buf[0]));
	// HAL_ADC_StartSampling(&hadc2);
	// HAL_TIM_Base_Start_IT(&htim2);
	// HAL_TIM_Base_Start_DMA(&htim3, (uint32_t*) adcBuffer, sizeof(adcBuffer)/4);

}
