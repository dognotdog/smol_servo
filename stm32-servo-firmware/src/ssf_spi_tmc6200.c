#include "ssf_spi_private.h"

#include "stm32g4xx_hal.h"
#include "stm32g4xx_hal_spi.h"

#include "debug.h"

#include <string.h>
#include <assert.h>

/**
 * The TMC6200 does 40bit transfers at a time, with the first 8bit being the register address, and the last 32bit being the data. CLK is inverted, CPOL=1 CPHA=1, Mode=3.
 * 
 * MSB is READ=0, WRITE=1.
 * 
 * Registers:
 *   0x00 GCONF
 *   0x01 GSTAT
 *   0x04 IOIN
 *   0x06 OTP_PROG
 *   0x07 OTP_READ
 *   0x08 FACTORY_CONF
 *   0x09 SHORT_CONF
 *   0x0A DRV_CONF
 * 
 * GSTAT:0 indicates state after reset
 * IOIN bits 31:24 contain the version, should be 0x10
 */



extern SPI_HandleTypeDef hspi2;
#define tmc6200Spi hspi2


void sspi_initTmc6200(void)
{
	HAL_SPI_DeInit(&tmc6200Spi);

	SPI_InitTypeDef config = {
		.Mode = SPI_MODE_MASTER,
		.Direction = SPI_DIRECTION_2LINES,
		.DataSize = SPI_DATASIZE_8BIT,
		.CLKPolarity = SPI_POLARITY_HIGH,
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

	// tmc6200Spi.Instance = SPI2;
	tmc6200Spi.Init = config;
	assert(HAL_SPI_Init(&tmc6200Spi) == HAL_OK);
}

void tmc6200_readRegisters(uint8_t cmd[], uint8_t result[][5], size_t numregs)
{
	for (size_t i = 0; i < numregs; ++i)
	{
		result[i][0] = cmd[i];
		spi_block_t block = {
			.numWords = 5,
			.deviceId = SSPI_DEVICE_TMC6200,
			.src = result[i],
			.dst = result[i],
		};

		sspi_transmitBlock(&block);
	}

}

sspi_tmc_state_t _readState(void) {
	// read 5 registers
	uint8_t cmd[5] = {
		TMC6200_REG_GCONF,
		TMC6200_REG_GSTAT,
		TMC6200_REG_IOIN,
		TMC6200_REG_SHORT_CONF,
		TMC6200_REG_DRV_CONF,
	};


	uint8_t rx[5][5];
	memset(rx, 0x00, sizeof(rx));

	tmc6200_readRegisters(cmd, rx, 5);

	return (sspi_tmc_state_t){
		.GCONF.reg = (rx[0][1] << 24) | (rx[0][2] << 16) | (rx[0][3] << 8) | (rx[0][4] << 0),
		.GSTAT.reg = (rx[1][1] << 24) | (rx[1][2] << 16) | (rx[1][3] << 8) | (rx[1][4] << 0),
		.IOIN.reg = (rx[2][1] << 24) | (rx[2][2] << 16) | (rx[2][3] << 8) | (rx[2][4] << 0),
		.SHORT_CONF.reg = (rx[3][1] << 24) | (rx[3][2] << 16) | (rx[3][3] << 8) | (rx[3][4] << 0),
		.DRV_CONF.reg = (rx[4][1] << 24) | (rx[4][2] << 16) | (rx[4][3] << 8) | (rx[4][4] << 0),
	};
}

static uint32_t _writeReg(uint8_t addr, uint32_t data) {
	
	// write GCONF
	uint8_t cmd[1][5] = {
		{
			(addr & 0x7F) | 0x80, 
			data << 24, 
			data << 16, 
			data << 8, 
			data << 0
		},
	};

	uint8_t rx[5];
	memset(rx, 0xFF, sizeof(rx));

	spi_block_t block = {
		.numWords = 5,
		.deviceId = SSPI_DEVICE_TMC6200,
		.src = cmd,
		.dst = rx,
	};

	sspi_transmitBlock(&block);

	return ((uint32_t)rx[1] << 24) | ((uint32_t)rx[2] << 16) |((uint32_t)rx[3] << 8) |((uint32_t)rx[3] << 0);
}

static sspi_tmc_state_t _writeGconf(sspi_tmc_state_t state) {
	state.GCONF.reg = _writeReg(TMC6200_REG_GCONF, state.GCONF.reg);
	return state;
}

static sspi_tmc_state_t _disableDriver(void) {

	sspi_tmc_state_t state = _readState();

	state.GCONF.disable = 1;

	return _writeGconf(state);
	
}


sspi_tmc_state_t sspi_tmc_setMotorDriver3PwmMode(void) {

	sspi_tmc_state_t state = _readState();

	state.GCONF.singleline = 1;

	return _writeGconf(state);
}

bool sspi_detectTmc6200(void) {
	// read 5 registers
	sspi_tmc_state_t state = _readState();
/**
 * GSTAT:0 indicates state after reset
 * IOIN bits 31:24 contain the version, should be 0x10
 */

	dbg_println("TMC6200 IOIN       = 0x%08"PRIx32, state.IOIN.reg);

	bool isPresent = (state.IOIN.VERSION == 0x10);
	if (!isPresent)
		return false;

	_disableDriver();

	state = _readState();

	dbg_println("TMC6200 GCONF      = 0x%08"PRIx32, state.GCONF.reg);
	dbg_println("TMC6200 GSTAT      = 0x%08"PRIx32, state.GSTAT.reg);
	dbg_println("TMC6200 SHORT_CONF = 0x%08"PRIx32, state.SHORT_CONF.reg);
	dbg_println("TMC6200 DRV_CONF   = 0x%08"PRIx32, state.DRV_CONF.reg);


	isPresent = state.GCONF.disable == 1;

	if (isPresent)
	{
		sspi.drvDevice = SSPI_DEVICE_TMC6200;

		// sspi_tmc_state_t state = _readState();

		// clear reset
		state.GSTAT.reset = 1;
		_writeReg(TMC6200_REG_GSTAT, state.GSTAT.reg);

		state.GCONF.disable = 0;
		state.GCONF.singleline = 1;

		_writeReg(TMC6200_REG_GCONF, state.GCONF.reg);

	}

	return isPresent;
}

