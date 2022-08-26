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
		.deviceId = SSPI_DEVICE_HALL,
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

bool sspi_detectAs5047d(void) {
	uint16_t cmd[4] = {
		(_PAR(0x4000)), // NOP should return 0x0000
		(_PAR(0x4003)), // PROG should return 0x0000
		(_PAR(0x4018)), // SETTINGS1 should return 0x0001
		(_PAR(0x4000)) // NOP at the end to get response to prev command
	};

	uint16_t rx[4] = {0xFFFF,0xFFFF,0xFFFF,0xFFFF};

	spi_transfer_t transfer = {
		.len = sizeof(cmd),
		.numWords = 4,
		.src = cmd,
		.dst = rx,
		.deviceId = SSPI_DEVICE_HALL,
	};

	sspi_syncTransfer(transfer);

	dbg_println("AS5047D 0x4000 = 0x%04"PRIx16, rx[1]);
	dbg_println("AS5047D 0x4003 = 0x%04"PRIx16, rx[2]);
	dbg_println("AS5047D 0x4018 = 0x%04"PRIx16, rx[3]);


	bool isPresent = (rx[1] == _PAR(0)) && (rx[2] == _PAR(0)) && (rx[3] == _PAR(0x0001));

	return isPresent;
}

