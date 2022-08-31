#include "ssf_spi_private.h"

#include "stm32g4xx_hal.h"
#include "stm32g4xx_hal_spi.h"

#include "utime.h"
#include "debug.h"

#include <string.h>
#include <assert.h>

extern SPI_HandleTypeDef hspi2;
#define hallSpi hspi2


void sspi_initAs5047d(void)
{
	HAL_SPI_DeInit(&hallSpi);

	SPI_InitTypeDef config = {
		.Mode = SPI_MODE_MASTER,
		.Direction = SPI_DIRECTION_2LINES,
		.DataSize = SPI_DATASIZE_16BIT,
		.CLKPolarity = SPI_POLARITY_LOW,
		.CLKPhase = SPI_PHASE_2EDGE,
		.NSS = SPI_NSS_HARD_OUTPUT,
		.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_128,
		.FirstBit = SPI_FIRSTBIT_MSB,
		.TIMode = SPI_TIMODE_DISABLE,
		.CRCCalculation = SPI_CRCCALCULATION_DISABLE,
		.CRCPolynomial = 7,
		.CRCLength = SPI_CRC_LENGTH_DATASIZE,
		.NSSPMode = SPI_NSS_PULSE_DISABLE,
	};

	// hallSpi.Instance = SPI2;
	hallSpi.Init = config;
	assert(HAL_SPI_Init(&hallSpi) == HAL_OK);
}


sspi_as5047_state_t ssf_readHallSensor(void)
{
	uint16_t cmd[4] = {
		(_PAR(0x4001)),
		(_PAR(0x7FFC)),
		(_PAR(0x7FFE)),
		(_PAR(0x4000)) // NOP at the end to get response to prev command
	};

	uint16_t rx[4] = {0xFFFF,0xFFFF,0xFFFF,0xFFFF};

	spi_transfer_t transfer = {
		.len = sizeof(cmd),
		.numWords = 4,
		.src = cmd,
		.dst = rx,
		.deviceId = SSPI_DEVICE_AS5047D,
	};

	uint32_t start_us = utime_now();
	sspi_syncTransfer(transfer);
	// wait until we can start a sync transfer
	uint32_t end_us = utime_now();

	sspi_as5047_state_t state = {
		.NOP = (rx[0]),
		.ERRFL = (rx[1]),
		.DIAAGC = (rx[2]),
		.ANGLEUNC = (rx[3]),
		.start_us = start_us,
		.end_us = end_us,
	};

	return state;
}

void as5047d_readRegisters(uint16_t* registers, uint16_t* results, size_t numregs)
{
	for (size_t i = 0; i <= numregs; ++i)
	{
		spi_block_t block = {
			.numWords = 1,
			.deviceId = SSPI_DEVICE_AS5047D,
		};

		uint16_t tmp = _PAR(0x4000); // NOP
		// add NOP read to end
		if (i < numregs) 
		{
			registers[i] = _PAR(registers[i]);
			block.src = &(registers[i]);
		}
		else
		{
			block.src = &(tmp);
		}

		// first read is not used, as response is lagging cmd
		if (i == 0) {
			block.dst = &(tmp);
		}
		else
		{
			block.dst = &(results[i-1]);
		}

		sspi_transmitBlock(&block);
	}
}

bool sspi_detectAs5047d(void) {
	uint16_t cmd[4] = {
		(_PAR(0x4000)), // NOP should return 0x0000
		(_PAR(0x4003)), // PROG should return 0x0000
		(_PAR(0x4018)), // SETTINGS1 should return 0x0001
		(_PAR(0x4000)) // NOP at the end to get response to prev command
	};

	uint16_t rx[4] = {0xFFFF,0xFFFF,0xFFFF,0xFFFF};

	as5047d_readRegisters(cmd, rx, 4);

	dbg_println("AS5047D 0x4000 = 0x%04"PRIx16, rx[1]);
	dbg_println("AS5047D 0x4003 = 0x%04"PRIx16, rx[2]);
	dbg_println("AS5047D 0x4018 = 0x%04"PRIx16, rx[3]);


	bool isPresent = (rx[1] == _PAR(0)) && (rx[2] == _PAR(0)) && (rx[3] == _PAR(0x0001));

	if (isPresent)
		sspi.encDevice = SSPI_DEVICE_AS5047D;
	return isPresent;
}

