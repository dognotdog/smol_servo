#include "debug.h"

#include "main.h"
#include "ssf_main.h"
#include "ssf_spi.h"


/*
AS5047 does 16bit SPI transfers
BIT15 is an even parity on commands as well as data, and transfers are big endian.

Propagation delay of sensor is spec'd at 100us +-10us from sensing to read via SPI

At 8Mhz, SPI read of 16bits takes 2us nominally, so ~10us for the 4x16bit words seems a reasonable assumption

*/

extern SPI_HandleTypeDef hspi2;

#define _XORS(x,s) (x ^ x >> 1)

#define  _EVENPAR(x) (((~_XORS(_XORS(_XORS(_XORS(_XORS(x, 16), 8), 4), 2), 1) & 1) << 15) | (x & 0x7FFF))
// #define FLIP16(x) (((x & 0xFF00) >> 8) | ((x & 0xFF) << 8))
// #define FLIP16(x) (x)

volatile int counter = 0;

void _delay(int d) {for (counter = 0; counter < d; ++counter ) {};}

mspi_as5047_state_t ssf_readHallSensor(void)
{
	// HAL_GPIO_WritePin(PIN_DRVSEL, GPIO_PIN_SET);

	uint16_t cmd[4] = {
		(_EVENPAR(0x4001)),
		(_EVENPAR(0x7FFC)),
		(_EVENPAR(0x7FFE)),
		(_EVENPAR(0x4000))
	};

	uint16_t rx[4] = {0xFFFF,0xFFFF,0xFFFF,0xFFFF};


	// drive everything high to deselect chips
	HAL_GPIO_WritePin(PIN_DRVSEL, GPIO_PIN_SET);
	HAL_GPIO_WritePin(PIN_ASEL, GPIO_PIN_SET);
	HAL_GPIO_WritePin(PIN_SPI_NSS, GPIO_PIN_RESET);
	_delay(100);


	for (size_t i = 0; i < 4; ++i)
	{
		HAL_GPIO_WritePin(PIN_ASEL, GPIO_PIN_RESET);
		_delay(100);

		HAL_StatusTypeDef txerr = HAL_SPI_TransmitReceive(&hspi2, (void*)(cmd+i), (void*)(rx+i), 1, 1000);



		HAL_GPIO_WritePin(PIN_ASEL, GPIO_PIN_SET);
		_delay(100);
		// dbg_println("SPI err = %d", txerr);

	}

	mspi_as5047_state_t state = {
		// .NOP = FLIP16(rx[0]),
		.ERRFL = (rx[1]),
		.DIAAGC = (rx[2]),
		.ANGLEUNC = (rx[3]),
	};

	return state;
}

mspi_drv_state_t ssf_readMotorDriver(void)
{
	// HAL_GPIO_WritePin(PIN_DRVSEL, GPIO_PIN_SET);

	uint16_t cmd[8] = {
		(1 << 15) | (0 << 10),
		(1 << 15) | (1 << 10),
		(1 << 15) | (2 << 10),
		(1 << 15) | (3 << 10),
		(1 << 15) | (4 << 10),
		(1 << 15) | (5 << 10),
		(1 << 15) | (6 << 10),
		(1 << 15) | (7 << 10),
		(1 << 15) | (0 << 10),
	};

	uint16_t rx[8] = {0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF};


	// drive everything high to deselect chips
	HAL_GPIO_WritePin(PIN_DRVSEL, GPIO_PIN_SET);
	HAL_GPIO_WritePin(PIN_ASEL, GPIO_PIN_SET);
	HAL_GPIO_WritePin(PIN_SPI_NSS, GPIO_PIN_RESET);
	_delay(100);


	for (size_t i = 0; i < 8; ++i)
	{
		HAL_GPIO_WritePin(PIN_DRVSEL, GPIO_PIN_RESET);
		_delay(100);

		HAL_StatusTypeDef txerr = HAL_SPI_TransmitReceive(&hspi2, (void*)(cmd+i), (void*)(rx+i), 1, 1000);



		HAL_GPIO_WritePin(PIN_DRVSEL, GPIO_PIN_SET);
		_delay(100);
		// dbg_println("SPI err = %d", txerr);

	}

	mspi_drv_state_t state = {
		// .NOP = FLIP16(rx[0]),
		.FAULT_STATUS 	= rx[1],
		.VGS_STATUS 	= rx[2],
		.DRV_CTRL 		= rx[3],
		.DRV_HS 		= rx[4],
		.DRV_LS 		= rx[5],
		.OCP_CTRL 		= rx[6],
		.CSA_CTRL 		= rx[7],
	};

	return state;
}


static void _setPinAsGpioOutput(GPIO_TypeDef *gpio, uint32_t GPIO_Pin)
{
	// this works for single pins only
	uint32_t shift = (GPIO_Pin*GPIO_Pin);
	uint32_t modeMask = shift | (shift << 1);

	gpio->MODER = (gpio->MODER & ~modeMask) | (shift*0x01);
}

void ssf_spiInit(void)
{
	_setPinAsGpioOutput(PIN_DRVSEL);
	_setPinAsGpioOutput(PIN_ASEL);
	// Hardware NSS is garbage (tied to SPI being enabled instead of transfers) so use GPIO
	_setPinAsGpioOutput(PIN_SPI_NSS);

	HAL_GPIO_WritePin(PIN_DRVSEL, GPIO_PIN_SET);
	HAL_GPIO_WritePin(PIN_ASEL, GPIO_PIN_SET);
	HAL_GPIO_WritePin(PIN_SPI_NSS, GPIO_PIN_RESET);

	_setPinAsGpioOutput(PIN_DRVEN);
	HAL_GPIO_WritePin(PIN_DRVEN, GPIO_PIN_SET);

	// __HAL_SPI_DISABLE(&hspi2);
}
