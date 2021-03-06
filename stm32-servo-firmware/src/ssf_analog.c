
#include "debug.h"
#include "ssf_main.h"
#include "ssf_mctrl.h"



/*
ADC Setup

	ADCs are clocked at 170/3 MHz (56.667MHz)
	So for a single channel, fs = (170/3)/(6.5+12.5) = 2.982MHz, 11.65kHz with 256x oversampling
	With 12.5 cycles, fs = (170/3)/(12.5+12.5) - 2.267MHz, or 0.44us. 100kHz PWM means we have a 10us cycle time. 16x oversampling takes 7.06us

	Assuming 100kHz PWM, it would make sense to cycle through the channels, with each channel being measured on every 3rd PWM period, for 1/3rd the rate. Using centered PWM, measure at the center of each HIGH and LOW cycle.

	channels are 17, 13, 3 for A,B,C respectively
	ADC is setup to store A,A,B,B,C,C sequence in buffer (PWM on / off center values)

	If the ADC is setup before the PWM timer is started, then ADC is started on the first trigger, which is not the beginning of counting, but the center of the first HIGH period.

*/

static uint16_t __ALIGNED(4) an0_buf[5];
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
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
	// uint32_t isr = hadc->Instance->ISR;
	// uint32_t ier = hadc->Instance->IER;
	// if (((isr & ADC_FLAG_EOSMP) == ADC_FLAG_EOSMP) && ((ier & ADC_IT_EOSMP) == ADC_IT_EOSMP))
	if (hadc == &hadc2)
	{
		_currentSenseConversionCallback();
		// do not reset EOSMP flags as HAL does that
	}

}
void _adc2DmaCallback(DMA_HandleTypeDef* hdma)
{
	_currentSenseConversionCallback();
}

static inline float ssf_calibratedVddaVoltage(float vref_count)
{
	// Voltage reference calibration
	// VDDA = 3.0 V x VREFINT_CAL / VREFINT_DATA
	float vref_cal = (uint32_t)(*VREFINT_CAL_ADDR);
	float vout_mV = (vref_cal * (float)VREFINT_CAL_VREF) / (vref_count);

	return vout_mV*0.001f;
}

static inline float ssf_calibratedVoltage(float vcount, float vdda)
{
	return vcount*vdda*(1.0f/ADC1_NOMINAL_MAXCOUNT);
}

float ssf_getVdda(void)
{
	// Voltage reference calibration
	// VDDA = 3.0 V x VREFINT_CAL / VREFINT_DATA
	float vref_count = an0_buf[3];
	float vref_ncount = vref_count * (1.0f / ADC1_OVERSAMPLING_FACTOR);

	return ssf_calibratedVddaVoltage(vref_ncount);
}
/*
	A Zener diode in parallel with the low resistor in a voltage divider will influence the measured voltage

	a) assuming constant resistance
		treat zener as in parallel resistance with lower resistor

	b) as constant leakage current Ir
		Ihi = (Vhi-Vdiv)/(Rhi) = Ir + Vdiv/Rlo
		Vhi/Rhi - Vdiv/Rhi = Ir + Vdiv/Rlo
		Vdiv/Rhi + Vdiv/Rlo = Vhi/Rhi - Ir
		Vdiv + Vdiv*Rhi/Rlo = Vhi - Ir*Rhi
		Vdiv = (Vhi - Ir*Rhi)/(1+Rhi/Rlo)
		Vhi = Vdiv*(1+Rhi/Rlo) + Ir*Rhi


*/
float ssf_getVddp(void)
{
	float vddCount = an0_buf[4]*(1.0f/ADC1_OVERSAMPLING_FACTOR);
	float vdda = ssf_getVdda();
	float vdiv = ssf_calibratedVoltage(vddCount, vdda);	

	const float bridgeHi = 22.0e3f;
	// guess at parasitic resistance
	// zener data: VR = 1V, IRmax = 25uA
	// const float rParasitic = 60.0e3f;
	// const float bridgeLo = (5.11e3f*rParasitic)/(5.11e3f+rParasitic);
	const float bridgeLo = 5.11e3f;

	// const float Ir = 5.0e-6f;
	// const float vddp = vdiv * (1.0f + bridgeHi/bridgeLo) + Ir*bridgeHi;

	const float vddp = vdiv*((bridgeHi + bridgeLo)/bridgeLo);

	return vddp;
}

float ssf_getVbus(void)
{
	float vbusCount = an0_buf[0]*(1.0f/ADC1_OVERSAMPLING_FACTOR);
	float vdda = ssf_getVdda();
	float vdiv = ssf_calibratedVoltage(vbusCount, vdda);

	const float bridgeHi = 120.0e3f;
	// guess at parasitic resistance
	// zener data: VR = 1V, IRmax = 25uA
	// STM32 analog input impedance spec'd at ~50k input impedance with a 5p sample-and-hold capacitor
	// const float rParasitic = 500.0e3f;
	// const float bridgeLo = (5.11e3f*rParasitic)/(5.11e3f+rParasitic);
	const float bridgeLo = 5.11e3f;

	// const float Ir = 5.0e-6f;
	// const float vbus = vdiv * (1.0f + bridgeHi/bridgeLo) + Ir*bridgeHi;

	const float vbus = vdiv*((bridgeHi + bridgeLo)/bridgeLo);

	return vbus;

	// return vdiv*((bridgeHi + bridgeLo)/bridgeLo);
	// return vbusCount*vdda/(ADC1_OVERSAMPLING_FACTOR*ADC1_NOMINAL_MAXCOUNT);
}

static void _setOversamplingModeRaw(ADC_HandleTypeDef* hadc, bool enable, uint32_t ovsr, uint32_t ovss)
{
	if (enable)
	{
        MODIFY_REG(hadc->Instance->CFGR2,
                   ADC_CFGR2_OVSR | ADC_CFGR2_OVSS,
                   ADC_CFGR2_ROVSE | ovsr | ovss
                  );

	}
	else
	{
		CLEAR_BIT(hadc->Instance->CFGR2, ADC_CFGR2_ROVSE);
	}
}

static void _setOversamplingMode(ADC_HandleTypeDef* hadc, uint32_t oversamplingRatio, uint32_t oversamplingShift)
{
	bool enable = false;
	uint32_t ovsr = 0;
	switch(oversamplingRatio)
	{
		case 1:
			break;
		case 2:
			ovsr = ADC_OVERSAMPLING_RATIO_2;
			enable = true;
			break;
		case 4:
			ovsr = ADC_OVERSAMPLING_RATIO_4;
			enable = true;
			break;
		case 8:
			ovsr = ADC_OVERSAMPLING_RATIO_8;
			enable = true;
			break;
		case 16:
			ovsr = ADC_OVERSAMPLING_RATIO_16;
			enable = true;
			break;
		case 32:
			ovsr = ADC_OVERSAMPLING_RATIO_32;
			enable = oversamplingShift >= 1;
			break;
		case 64:
			ovsr = ADC_OVERSAMPLING_RATIO_64;
			enable = oversamplingShift >= 2;
			break;
		case 128:
			ovsr = ADC_OVERSAMPLING_RATIO_128;
			enable = oversamplingShift >= 3;
			break;
		case 256:
			ovsr = ADC_OVERSAMPLING_RATIO_256;
			enable = oversamplingShift >= 4;
			break;
		default:
			enable = false;
			err_println("invalid oversampling ratio for ADC oversampling: %u", oversamplingRatio);
			return;
	}

	if ((oversamplingRatio > 1) && !enable)
	{
		err_println("invalid oversampling ratio and bitshif combination for ADC oversampling: %u >> %u", oversamplingRatio, oversamplingShift);
		return;
	}

	uint32_t ovss = 0;
	switch(oversamplingShift)
	{
		case 0:
			ovss = ADC_RIGHTBITSHIFT_NONE;
			break;
		case 1:
			ovss = ADC_RIGHTBITSHIFT_1;
			break;
		case 2:
			ovss = ADC_RIGHTBITSHIFT_2;
			break;
		case 3:
			ovss = ADC_RIGHTBITSHIFT_3;
			break;
		case 4:
			ovss = ADC_RIGHTBITSHIFT_4;
			break;
		case 5:
			ovss = ADC_RIGHTBITSHIFT_5;
			break;
		case 6:
			ovss = ADC_RIGHTBITSHIFT_6;
			break;
		case 7:
			ovss = ADC_RIGHTBITSHIFT_7;
			break;
		case 8:
			ovss = ADC_RIGHTBITSHIFT_8;
			break;
		default:
			enable = false;
			err_println("invalid bishift for ADC oversampling: %u", oversamplingShift);
			return;
	}

	_setOversamplingModeRaw(hadc, enable, ovsr, ovss);
}

void ssf_analogStartRealtimeSampling(void)
{
	_setOversamplingMode(&hadc2, ADC2_OVERSAMPLING_COUNT, ADC2_SHIFT);
	HAL_ADC_Start_DMA(&hadc2, (uint32_t*) an1_buf, sizeof(an1_buf)/sizeof(an1_buf[0]));
	// HAL_ADC_Start_DMA(&hadc2, (uint32_t*) an1_buf, 3);
	// HAL_ADC_StartSampling(&hadc2);
	// HAL_TIM_Base_Start_IT(&htim2);
	// HAL_TIM_Base_Start_DMA(&htim3, (uint32_t*) adcBuffer, sizeof(adcBuffer)/4);
}

void ssf_analogStopRealtimeSampling(void)
{
	HAL_ADC_Stop_DMA(&hadc2);
	// HAL_ADC_Start_DMA(&hadc2, (uint32_t*) an1_buf, 3);
	// HAL_ADC_StartSampling(&hadc2);
	// HAL_TIM_Base_Start_IT(&htim2);
	// HAL_TIM_Base_Start_DMA(&htim3, (uint32_t*) adcBuffer, sizeof(adcBuffer)/4);
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
	// HAL_ADC_RegisterCallback(&hadc2, HAL_ADC_CONVERSION_COMPLETE_CB_ID, _adc2ConversionCompleteCallback);


	HAL_ADC_Start_DMA(&hadc1, (uint32_t*) an0_buf, sizeof(an0_buf)/sizeof(an0_buf[0]));
	HAL_ADC_StartSampling(&hadc1);

	ssf_analogStartRealtimeSampling();

}
