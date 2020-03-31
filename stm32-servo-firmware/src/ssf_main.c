
#include "stm32g4xx_hal.h"
#include "stm32g4xx_hal_adc.h"


#include "debug.h"

#include "main.h"
#include "ssf_main.h"
#include "ssf_spi.h"
#include "utime.h"

#include <stdbool.h>


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
	ssf_analogInit();
	utime_init();
	ssf_ledInit();
	spwm_init();
	ssf_spiInit();

	mctrl_init();

}

char _usbcmd = 0;
static void _processUsbRx(void)
{
	switch (_usbcmd)
	{
		case 'i':
		{
			mctrl_init();
			break;
		}
	}
	_usbcmd = 0;

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

void ssf_idle(void)
{
	ssf_ledIdle();

	_processUsbRx();

	uint32_t now_ms = HAL_GetTick();

	// spwm_idle();
	// HAL_Delay(1);


	if (((now_ms) % 1000) == 0)
	{
		// dbg_println("Hello USB!");
		// // HAL_Delay(8);

		// sspi_as5047_state_t hallState = ssf_readHallSensor();
		// dbg_println("HALL %04x, %04x, %04x, %04x", hallState.NOP, hallState.ERRFL, hallState.DIAAGC, hallState.ANGLEUNC);
		// sspi_drv_state_t drvState = ssf_readMotorDriver();
		// dbg_println("DRV %04x, %04x, %04x, %04x, %04x, %04x, %04x", drvState.FAULT_STATUS.reg, drvState.VGS_STATUS.reg, drvState.DRV_CTRL.reg, drvState.DRV_HS.reg, drvState.DRV_LS.reg, drvState.OCP_CTRL.reg, drvState.CSA_CTRL.reg);
		// ssf_printMotorDriverFaults(drvState);
		// // HAL_Delay(8);

		// bool drven = HAL_GPIO_ReadPin(PIN_DRVEN);
		// dbg_println("DRVEN %8u %u", utime_now(), drven);

		// uint32_t adcState = HAL_ADC_GetState(&hadc2);
		// uint32_t adcError = HAL_ADC_GetError(&hadc2);
		// dbg_println("ADC2 0x%08x, 0x%08x", adcState, adcError);
		// HAL_Delay(8);

		// dbg_println("iA = %8.3f %8.3f, iB = %8.3f %8.3f, iC = %8.3f %8.3f", (double)currentSensed[0], (double)currentSensed[1], (double)currentSensed[2], (double)currentSensed[3], (double)currentSensed[4], (double)currentSensed[5]);
		// dbg_println("iA = %5u %5u, iB = %5u %5u, iC = %5u %5u", (int)an1_buf[0], (int)an1_buf[1], (int)an1_buf[2], (int)an1_buf[3], (int)an1_buf[4], (int)an1_buf[5]);

		dbg_println("VBUS = %.3f, VDDA = %.3f", (double)ssf_getVbus(), (double)ssf_getVdda());

		float* phaseCurrents0 = mctrl_getPhaseTable(2);
		float* phaseCurrents1 = mctrl_getPhaseTable(3);

		float maxi = 0.0f;
		for (size_t i = 0; i < 16; ++i)
		{
			maxi = fmaxf(maxi, fabsf(phaseCurrents0[i]));
		}

		const char* padding = "                                ";
		const char* line 	= "--------------------------------";


		for (size_t i = 0; i < 16; ++i)
		{
			int lineLen = fminf(32.0f, 32.0f/maxi*fabsf(phaseCurrents0[i]));
			int padLen = phaseCurrents0[i] > 0.0f ? 32 : 32-lineLen;
			dbg_println("IB[%2u] = %8.3f, %8.3f %.*s%.*s", i, (double)phaseCurrents0[i], (double)phaseCurrents1[i], padLen, padding, lineLen, line);
		}


	}
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


void USB_HP_IRQHandler(void)
{
	// uart_printf("WWDG_IRQHandler!\r\n");
	while (1) {++irqDummyCounter;};
}


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

void TIM1_UP_TIM16_IRQHandler(void)
{
	// uart_printf("WWDG_IRQHandler!\r\n");
	while (1) {++irqDummyCounter;};
}

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

void SPI2_IRQHandler(void)
{
	// uart_printf("WWDG_IRQHandler!\r\n");
	while (1) {++irqDummyCounter;};
}

void USART1_IRQHandler(void)
{
	// uart_printf("WWDG_IRQHandler!\r\n");
	while (1) {++irqDummyCounter;};
}

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


void DMA2_Channel1_IRQHandler(void)
{
	// uart_printf("WWDG_IRQHandler!\r\n");
	while (1) {++irqDummyCounter;};
}


void DMA2_Channel2_IRQHandler(void)
{
	// uart_printf("WWDG_IRQHandler!\r\n");
	while (1) {++irqDummyCounter;};
}


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


void I2C3_EV_IRQHandler(void)
{
	// uart_printf("WWDG_IRQHandler!\r\n");
	while (1) {++irqDummyCounter;};
}


void I2C3_ER_IRQHandler(void)
{
	// uart_printf("WWDG_IRQHandler!\r\n");
	while (1) {++irqDummyCounter;};
}


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