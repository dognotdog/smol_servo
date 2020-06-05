
#include "stm32g4xx_hal.h"
#include "stm32g4xx_hal_adc.h"


#include "debug.h"

#include "main.h"
#include "ssf_main.h"
#include "ssf_spi.h"
#include "ssf_flash.h"
#include "ssf_mctrl.h"
#include "utime.h"

#include "servo_hid_if.h"

#include <stdbool.h>
#include <stdio.h>
#include <errno.h>
#include <assert.h>

void __assert_func(const char * file, int line, const char * function, const char * expr)
{
	err_println("assertion failed: `%s` in %s, file %s:%u", expr, function, file, line);
	while (1) {};
}


static void _setPinAsGpioOutput(GPIO_TypeDef *gpio, uint32_t GPIO_Pin)
{
	// this works for single pins only
	uint32_t shift = (GPIO_Pin*GPIO_Pin);
	uint32_t modeMask = shift | (shift << 1);

	gpio->MODER = (gpio->MODER & ~modeMask) | (shift*0x01);
}

void test_chipSelects(void)
{
	_setPinAsGpioOutput(PIN_DRVSEL);
	_setPinAsGpioOutput(PIN_ASEL);
	// _setPinAsGpioOutput(PIN_SPI_NSS);

	// HAL_GPIO_WritePin(PIN_SPI_NSS, GPIO_PIN_RESET);
	// HAL_GPIO_WritePin(PIN_DRVSEL, GPIO_PIN_RESET);
	// HAL_GPIO_WritePin(PIN_ASEL, GPIO_PIN_RESET);
	// HAL_Delay(250);
	// HAL_GPIO_WritePin(PIN_DRVSEL, GPIO_PIN_SET);
	// HAL_Delay(250);
	// HAL_GPIO_WritePin(PIN_ASEL, GPIO_PIN_SET);
	// HAL_Delay(250);

	// HAL_GPIO_WritePin(PIN_SPI_NSS, GPIO_PIN_SET);
	// HAL_GPIO_WritePin(PIN_DRVSEL, GPIO_PIN_RESET);
	// HAL_GPIO_WritePin(PIN_ASEL, GPIO_PIN_RESET);
	// HAL_Delay(250);
	// HAL_GPIO_WritePin(PIN_DRVSEL, GPIO_PIN_SET);
	// HAL_Delay(250);
	// HAL_GPIO_WritePin(PIN_ASEL, GPIO_PIN_SET);
	// HAL_Delay(250);

	// HAL_GPIO_WritePin(PIN_SPI_NSS, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(PIN_DRVSEL, GPIO_PIN_SET);
	HAL_GPIO_WritePin(PIN_ASEL, GPIO_PIN_RESET);
	HAL_Delay(50);
	HAL_GPIO_WritePin(PIN_ASEL, GPIO_PIN_SET);
	HAL_Delay(50);
	HAL_GPIO_WritePin(PIN_ASEL, GPIO_PIN_RESET);
	HAL_Delay(50);
	HAL_GPIO_WritePin(PIN_ASEL, GPIO_PIN_SET);
	HAL_Delay(50);

}


void ssf_init(void)
{
	do {dbg_println("ssf_init()...");} while (0);

	ssf_flashInit();

	ssf_analogInit();
	utime_init();
	ssf_ledInit();
	spwm_init();
	ssf_spiInit();

	ssf_scheduleInit();


	mctrl_init();

	ssf_enableTasks(SCHED_UI_SLOW, SCHED_UI_SLOW, true);

}

/*
	FIXME: for some reason, calling fread() does not work right. _read is called only once, and never again, but calling _read() directly works as expected. This is a hack until it we figure out why stdio functions don't work. Writing, OTOH, appears to work just fine through fprintf().
*/
extern int _read(int32_t file, uint8_t *ptr, int32_t len);

static int _usbRead(void *ptr, size_t len)
{
	return _read(0, ptr, len);
}


char _usbcmd = 0;
static void _processUsbRx(void)
{
	// fprintf(stdout, "USB yay %u\r\n", HAL_GetTick());
	// _usbcmd = myRead();
	int readResult = _usbRead(&_usbcmd, 1);
	// dbg_println("fread() = %u, errno = %d", readResult, errno);
	if (readResult == 1)
	{
		// fwrite(&_usbcmd,1,1,stdout);

		// dbg_println("received something USB");
		switch (_usbcmd)
		{
			case 'i':
			{
				dbg_println("received mctrl reset through USB");
				mctrl_init();
				break;
			}
			default:
			{
				dbg_println("received %u through USB", _usbcmd);
				break;
			}
		}
		_usbcmd = 0;		
	}

}

void ssf_usbRxCallback(const void* _data, size_t datalen)
{
	const char* str = _data;
	dbg_println("USB RX");
	if (datalen > 0)
	{
		_usbcmd = str[0];
	}
}

// static volatile sspi_as5047_state_t _hallState;
// static volatile uint32_t			_hallReadCounter;

// void ssf_asyncReadHallSensorCallback(sspi_as5047_state_t sensorState)
// {
// 	_hallState = sensorState;
// 	++_hallReadCounter;
// }

// float ssf_getEncoderAngle(void)
// {
// 	return (float)(_hallState.ANGLEUNC & 0x3FFF) / (float)0x4000 * 2.0f*M_PI;
// }


void ssf_ui1sTask(uint32_t now_us)
{
	// dbg_println("Hello USB!");
	// // HAL_Delay(8);
	
	// spi_printTransferStatus();
	// ssf_asyncReadHallSensor();
	// sspi_as5047_state_t hallState = ssf_readHallSensor();
	sspi_as5047_state_t hallState = ssf_dbgGetLastEncoderReading();
	uint32_t readCounter = ssf_dbgGetEncoderReadCounter();
	uint32_t formatErrorCounter = 0, valueErrorCounter = 0;
	uint32_t errorCounter = ssf_dbgGetEncoderErrorCounter(&formatErrorCounter, &valueErrorCounter);
	dbg_println("AS5047D %8u reads, %8u errors (%8u fmt, %8u val)", readCounter, errorCounter, formatErrorCounter, valueErrorCounter);

	ssf_dbgPrintEncoderStatus(hallState);
	// dbg_println("HALL[%8u] 0x%04x, 0x%04x, 0x%04x, 0x%04x", readCounter, hallState.NOP, hallState.ERRFL, hallState.DIAAGC, hallState.ANGLEUNC);
	{
		sspi_drv_state_t drvState = ssf_readMotorDriver();
		// dbg_println("DRV %04x, %04x, %04x, %04x, %04x, %04x, %04x", drvState.FAULT_STATUS.reg, drvState.VGS_STATUS.reg, drvState.DRV_CTRL.reg, drvState.DRV_HS.reg, drvState.DRV_LS.reg, drvState.OCP_CTRL.reg, drvState.CSA_CTRL.reg);
		ssf_printMotorDriverFaults(drvState);
	}
	// // HAL_Delay(8);

	// bool drven = HAL_GPIO_ReadPin(PIN_DRVEN);
	// dbg_println("DRVEN %8u %u", utime_now(), drven);

	// uint32_t adcState = HAL_ADC_GetState(&hadc2);
	// uint32_t adcError = HAL_ADC_GetError(&hadc2);
	// dbg_println("ADC2 0x%08x, 0x%08x", adcState, adcError);
	// HAL_Delay(8);

	// dbg_println("iA = %8.3f %8.3f, iB = %8.3f %8.3f, iC = %8.3f %8.3f", (double)currentSensed[0], (double)currentSensed[1], (double)currentSensed[2], (double)currentSensed[3], (double)currentSensed[4], (double)currentSensed[5]);
	// dbg_println("iA = %5u %5u, iB = %5u %5u, iC = %5u %5u", (int)an1_buf[0], (int)an1_buf[1], (int)an1_buf[2], (int)an1_buf[3], (int)an1_buf[4], (int)an1_buf[5]);

	mctrl_simplePositionEstimate_t posEst = mctrl_getSimpleMotorPositionEstimate();

	dbg_println("x_est %8.3f deg, speed = %8.3f rpm", (double)(posEst.x[0][0]*180.0f/M_PI), (double)(posEst.x[1][0]*30.0f/M_PI));
	// mctrl_dbgPrintSimpleEstimatePair();
	// dbg_println("P =  %8.3g %8.3g", (double)(sqrtf(posEst.P[0][0])), (double)(sqrtf(posEst.P[0][1])));
	// dbg_println("     %8.3g %8.3g", (double)(sqrtf(posEst.P[1][0])), (double)(sqrtf(posEst.P[1][1])));
	// dbg_println("Q =  %8.3g %8.3g", (double)(sqrtf(posEst.debug.Q[0][0])), (double)(sqrtf(posEst.debug.Q[0][1])));
	// dbg_println("     %8.3g %8.3g", (double)(sqrtf(posEst.debug.Q[1][0])), (double)(sqrtf(posEst.debug.Q[1][1])));
	// dbg_println("K =  %8.3g", (double)((posEst.debug.K[0][0])));
	// dbg_println("     %8.3g", (double)((posEst.debug.K[1][0])));
	// dbg_println("R =  %8.3g", (double)(sqrtf(posEst.debug.R)));
	// dbg_println("t =  %8.3f, tm = %8.3f", (double)(1.0e-6f*posEst.timeStamp_us), (double)(1.0e-6f*posEst.lastMeasurementTimeStamp_us));

	dbg_println("VBUS = %.3f, VDDA = %.3f, VDDP = %.3f", (double)ssf_getVbus(), (double)ssf_getVdda(), (double)ssf_getVddp());


	// float* phaseCurrents0 = mctrl_getPhaseTable(2);
	// float* phaseCurrents1 = mctrl_getPhaseTable(3);

	// float maxi = 0.0f;
	// for (size_t i = 0; i < 16; ++i)
	// {
	// 	maxi = fmaxf(maxi, fabsf(phaseCurrents0[i]));
	// }

	// const char* padding = "                                ";
	// const char* line 	= "--------------------------------";


	// for (size_t i = 0; i < 16; ++i)
	// {
	// 	int lineLen = fminf(32.0f, 32.0f/maxi*fabsf(phaseCurrents0[i]));
	// 	int padLen = phaseCurrents0[i] > 0.0f ? 32 : 32-lineLen;
	// 	dbg_println("IB[%2u] = %8.3f, %8.3f %.*s%.*s", i, (double)phaseCurrents0[i], (double)phaseCurrents1[i], padLen, padding, lineLen, line);
	// }


}


void ssf_idle(void)
{
	ssf_ledIdle();

	mctrl_idle(utime_now());


	_processUsbRx();

	uint32_t now_ms = HAL_GetTick();

	servo_hid_interface_run(now_ms);


	// spwm_idle();
	// HAL_Delay(1);

	ssf_asyncReadHallSensor();

	ssf_scheduleTasks(SCHED_UI_SLOW, SCHED_UI_SLOW, utime_now());

	// test_chipSelects();

}



static volatile int irqDummyCounter;

void WWDG_IRQHandler(void)
{
	// uart_printf("WWDG_IRQHandler!\r\n");
	while (1) {++irqDummyCounter;};
}

void PVD_PVM_IRQHandler(void)
{
	// uart_printf("WWDG_IRQHandler!\r\n");
	while (1) {++irqDummyCounter;};
}


void RTC_TAMP_LSECSS_IRQHandler(void)
{
	// uart_printf("WWDG_IRQHandler!\r\n");
	while (1) {++irqDummyCounter;};
}


void RTC_WKUP_IRQHandler(void)
{
	// uart_printf("WWDG_IRQHandler!\r\n");
	while (1) {++irqDummyCounter;};
}

void FLASH_IRQHandler(void)
{
	// uart_printf("WWDG_IRQHandler!\r\n");
	while (1) {++irqDummyCounter;};
}

void RCC_IRQHandler(void)
{
	// uart_printf("WWDG_IRQHandler!\r\n");
	while (1) {++irqDummyCounter;};
}

void EXTI0_IRQHandler(void)
{
	// uart_printf("WWDG_IRQHandler!\r\n");
	while (1) {++irqDummyCounter;};
}


void EXTI1_IRQHandler(void)
{
	// uart_printf("WWDG_IRQHandler!\r\n");
	while (1) {++irqDummyCounter;};
}


void EXTI2_IRQHandler(void)
{
	// uart_printf("WWDG_IRQHandler!\r\n");
	while (1) {++irqDummyCounter;};
}


void EXTI3_IRQHandler(void)
{
	// uart_printf("WWDG_IRQHandler!\r\n");
	while (1) {++irqDummyCounter;};
}


void EXTI4_IRQHandler(void)
{
	// uart_printf("WWDG_IRQHandler!\r\n");
	while (1) {++irqDummyCounter;};
}

void DMA1_Channel3_IRQHandler(void)
{
	// uart_printf("WWDG_IRQHandler!\r\n");
	while (1) {++irqDummyCounter;};
}


void DMA1_Channel4_IRQHandler(void)
{
	// uart_printf("WWDG_IRQHandler!\r\n");
	while (1) {++irqDummyCounter;};
}


void DMA1_Channel5_IRQHandler(void)
{
	// uart_printf("WWDG_IRQHandler!\r\n");
	while (1) {++irqDummyCounter;};
}


void DMA1_Channel6_IRQHandler(void)
{
	// uart_printf("WWDG_IRQHandler!\r\n");
	while (1) {++irqDummyCounter;};
}

// void ADC1_2_IRQHandler(void)
// {
// 	// uart_printf("WWDG_IRQHandler!\r\n");
// 	while (1) {++irqDummyCounter;};
// }


// void USB_HP_IRQHandler(void)
// {
// 	// uart_printf("WWDG_IRQHandler!\r\n");
// 	while (1) {++irqDummyCounter;};
// }


// void USB_LP_IRQHandler(void)
// {
// 	// uart_printf("WWDG_IRQHandler!\r\n");
// 	while (1) {++irqDummyCounter;};
// }

void FDCAN1_IT0_IRQHandler(void)
{
	// uart_printf("WWDG_IRQHandler!\r\n");
	while (1) {++irqDummyCounter;};
}

void FDCAN1_IT1_IRQHandler(void)
{
	// uart_printf("WWDG_IRQHandler!\r\n");
	while (1) {++irqDummyCounter;};
}

void EXTI9_5_IRQHandler(void)
{
	// uart_printf("WWDG_IRQHandler!\r\n");
	while (1) {++irqDummyCounter;};
}

void TIM1_BRK_TIM15_IRQHandler(void)
{
	// uart_printf("WWDG_IRQHandler!\r\n");
	while (1) {++irqDummyCounter;};
}

// void TIM1_UP_TIM16_IRQHandler(void)
// {
// 	// uart_printf("WWDG_IRQHandler!\r\n");
// 	while (1) {++irqDummyCounter;};
// }

void TIM1_TRG_COM_TIM17_IRQHandler(void)
{
	// uart_printf("WWDG_IRQHandler!\r\n");
	while (1) {++irqDummyCounter;};
}

void TIM1_CC_IRQHandler(void)
{
	// uart_printf("WWDG_IRQHandler!\r\n");
	while (1) {++irqDummyCounter;};
}

void TIM2_IRQHandler(void)
{
	// uart_printf("WWDG_IRQHandler!\r\n");
	while (1) {++irqDummyCounter;};
}


void TIM3_IRQHandler(void)
{
	// uart_printf("WWDG_IRQHandler!\r\n");
	while (1) {++irqDummyCounter;};
}


void TIM4_IRQHandler(void)
{
	// uart_printf("WWDG_IRQHandler!\r\n");
	while (1) {++irqDummyCounter;};
}

void I2C1_EV_IRQHandler(void)
{
	// uart_printf("WWDG_IRQHandler!\r\n");
	while (1) {++irqDummyCounter;};
}

void I2C1_ER_IRQHandler(void)
{
	// uart_printf("WWDG_IRQHandler!\r\n");
	while (1) {++irqDummyCounter;};
}

void I2C2_EV_IRQHandler(void)
{
	// uart_printf("WWDG_IRQHandler!\r\n");
	while (1) {++irqDummyCounter;};
}

void I2C2_ER_IRQHandler(void)
{
	// uart_printf("WWDG_IRQHandler!\r\n");
	while (1) {++irqDummyCounter;};
}

void SPI1_IRQHandler(void)
{
	// uart_printf("WWDG_IRQHandler!\r\n");
	while (1) {++irqDummyCounter;};
}

// void SPI2_IRQHandler(void)
// {
// 	// uart_printf("WWDG_IRQHandler!\r\n");
// 	while (1) {++irqDummyCounter;};
// }

// void USART1_IRQHandler(void)
// {
// 	// uart_printf("WWDG_IRQHandler!\r\n");
// 	while (1) {++irqDummyCounter;};
// }

void USART2_IRQHandler(void)
{
	// uart_printf("WWDG_IRQHandler!\r\n");
	while (1) {++irqDummyCounter;};
}

void USART3_IRQHandler(void)
{
	// uart_printf("WWDG_IRQHandler!\r\n");
	while (1) {++irqDummyCounter;};
}

void EXTI15_10_IRQHandler(void)
{
	// uart_printf("WWDG_IRQHandler!\r\n");
	while (1) {++irqDummyCounter;};
}

void RTC_Alarm_IRQHandler(void)
{
	// uart_printf("WWDG_IRQHandler!\r\n");
	while (1) {++irqDummyCounter;};
}

void USBWakeUp_IRQHandler(void)
{
	// uart_printf("WWDG_IRQHandler!\r\n");
	while (1) {++irqDummyCounter;};
}


void TIM8_BRK_IRQHandler(void)
{
	// uart_printf("WWDG_IRQHandler!\r\n");
	while (1) {++irqDummyCounter;};
}


void TIM8_UP_IRQHandler(void)
{
	// uart_printf("WWDG_IRQHandler!\r\n");
	while (1) {++irqDummyCounter;};
}


void TIM8_TRG_COM_IRQHandler(void)
{
	// uart_printf("WWDG_IRQHandler!\r\n");
	while (1) {++irqDummyCounter;};
}


void TIM8_CC_IRQHandler(void)
{
	// uart_printf("WWDG_IRQHandler!\r\n");
	while (1) {++irqDummyCounter;};
}

void LPTIM1_IRQHandler(void)
{
	// uart_printf("WWDG_IRQHandler!\r\n");
	while (1) {++irqDummyCounter;};
}


void SPI3_IRQHandler(void)
{
	// uart_printf("WWDG_IRQHandler!\r\n");
	while (1) {++irqDummyCounter;};
}


void UART4_IRQHandler(void)
{
	// uart_printf("WWDG_IRQHandler!\r\n");
	while (1) {++irqDummyCounter;};
}


void TIM6_DAC_IRQHandler(void)
{
	// uart_printf("WWDG_IRQHandler!\r\n");
	while (1) {++irqDummyCounter;};
}


void TIM7_IRQHandler(void)
{
	// uart_printf("WWDG_IRQHandler!\r\n");
	while (1) {++irqDummyCounter;};
}


// void DMA2_Channel1_IRQHandler(void)
// {
// 	// uart_printf("WWDG_IRQHandler!\r\n");
// 	while (1) {++irqDummyCounter;};
// }


// void DMA2_Channel2_IRQHandler(void)
// {
// 	// uart_printf("WWDG_IRQHandler!\r\n");
// 	while (1) {++irqDummyCounter;};
// }


void DMA2_Channel3_IRQHandler(void)
{
	// uart_printf("WWDG_IRQHandler!\r\n");
	while (1) {++irqDummyCounter;};
}


void DMA2_Channel4_IRQHandler(void)
{
	// uart_printf("WWDG_IRQHandler!\r\n");
	while (1) {++irqDummyCounter;};
}


void DMA2_Channel5_IRQHandler(void)
{
	// uart_printf("WWDG_IRQHandler!\r\n");
	while (1) {++irqDummyCounter;};
}


void UCPD1_IRQHandler(void)
{
	// uart_printf("WWDG_IRQHandler!\r\n");
	while (1) {++irqDummyCounter;};
}


void COMP1_2_3_IRQHandler(void)
{
	// uart_printf("WWDG_IRQHandler!\r\n");
	while (1) {++irqDummyCounter;};
}


void COMP4_IRQHandler(void)
{
	// uart_printf("WWDG_IRQHandler!\r\n");
	while (1) {++irqDummyCounter;};
}


void CRS_IRQHandler(void)
{
	// uart_printf("WWDG_IRQHandler!\r\n");
	while (1) {++irqDummyCounter;};
}


void SAI1_IRQHandler(void)
{
	// uart_printf("WWDG_IRQHandler!\r\n");
	while (1) {++irqDummyCounter;};
}


void FPU_IRQHandler(void)
{
	// uart_printf("WWDG_IRQHandler!\r\n");
	while (1) {++irqDummyCounter;};
}


void RNG_IRQHandler(void)
{
	// uart_printf("WWDG_IRQHandler!\r\n");
	while (1) {++irqDummyCounter;};
}


void LPUART1_IRQHandler(void)
{
	// uart_printf("WWDG_IRQHandler!\r\n");
	while (1) {++irqDummyCounter;};
}


// void I2C3_EV_IRQHandler(void)
// {
// 	// uart_printf("WWDG_IRQHandler!\r\n");
// 	while (1) {++irqDummyCounter;};
// }


// void I2C3_ER_IRQHandler(void)
// {
// 	// uart_printf("WWDG_IRQHandler!\r\n");
// 	while (1) {++irqDummyCounter;};
// }


void DMAMUX_OVR_IRQHandler(void)
{
	// uart_printf("WWDG_IRQHandler!\r\n");
	while (1) {++irqDummyCounter;};
}


void DMA2_Channel6_IRQHandler(void)
{
	// uart_printf("WWDG_IRQHandler!\r\n");
	while (1) {++irqDummyCounter;};
}


void CORDIC_IRQHandler(void)
{
	// uart_printf("WWDG_IRQHandler!\r\n");
	while (1) {++irqDummyCounter;};
}


void FMAC_IRQHandler(void)
{
	// uart_printf("WWDG_IRQHandler!\r\n");
	while (1) {++irqDummyCounter;};
}