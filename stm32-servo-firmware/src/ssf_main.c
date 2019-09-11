
#include "debug.h"

#include "main.h"
#include "ssf_main.h"
#include "ssf_spi.h"

#include <stdbool.h>


static void _setPinAsGpioOutput(GPIO_TypeDef *gpio, uint32_t GPIO_Pin)
{
	// this works for single pins only
	uint32_t shift = (GPIO_Pin*GPIO_Pin);
	uint32_t modeMask = shift | (shift << 1);

	gpio->MODER = (gpio->MODER & ~modeMask) | (shift*0x01);
}

static void test_chipSelects(void)
{
	_setPinAsGpioOutput(PIN_DRVSEL);
	_setPinAsGpioOutput(PIN_ASEL);
	_setPinAsGpioOutput(PIN_SPI_NSS);

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

	HAL_GPIO_WritePin(PIN_SPI_NSS, GPIO_PIN_RESET);
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
	ssf_ledInit();
	ssf_spiInit();
}



void ssf_idle(void)
{
	ssf_ledIdle();

	uint32_t now_ms = HAL_GetTick();

	if (((now_ms) % 1000) == 0)
	{
		dbg_println("Hello USB!");
		HAL_Delay(1);

		mspi_as5047_state_t hallState = ssf_readHallSensor();
		dbg_println("HALL %04x, %04x, %04x, %04x", hallState.NOP, hallState.ERRFL, hallState.DIAAGC, hallState.ANGLEUNC);
		mspi_drv_state_t drvState = ssf_readMotorDriver();
		dbg_println("DRV %04x, %04x, %04x, %04x, %04x, %04x, %04x", drvState.FAULT_STATUS, drvState.VGS_STATUS, drvState.DRV_CTRL, drvState.DRV_HS, drvState.DRV_LS, drvState.OCP_CTRL, drvState.CSA_CTRL);

		bool drven = HAL_GPIO_ReadPin(PIN_DRVEN);
		dbg_println("DRVEN %u", drven);

	}
	// test_chipSelects();

}