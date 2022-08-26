#include "ssf_spi_private.h"

#include "stm32g4xx_hal.h"
#include "stm32g4xx_hal_spi.h"

#include "debug.h"

#include <string.h>
#include <assert.h>

/**
 * The TMC6200 does 40bit transfers at a time, with the first 8bit being the register address, and the last 32bit being the data.
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



#define TMC6200_REG_GCONF 		0x00
#define TMC6200_REG_GSTAT 		0x01
#define TMC6200_REG_IOIN 		0x04
#define TMC6200_REG_SHORT_CONF 	0x09
#define TMC6200_REG_DRV_CONF 	0x0A

extern SPI_HandleTypeDef hspi2;
#define tmc6200Spi hspi2


void sspi_initTmc6200(void)
{
	HAL_SPI_DeInit(&tmc6200Spi);

	SPI_InitTypeDef config = {
		.Mode = SPI_MODE_MASTER,
		.Direction = SPI_DIRECTION_2LINES,
		.DataSize = SPI_DATASIZE_8BIT,
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

	// tmc6200Spi.Instance = SPI2;
	tmc6200Spi.Init = config;
	assert(HAL_SPI_Init(&tmc6200Spi) == HAL_OK);
}

sspi_tmc_state_t _readState(void) {
	// read 5 registers
	uint8_t cmd[5][5] = {
		{TMC6200_REG_GCONF},
		{TMC6200_REG_GSTAT},
		{TMC6200_REG_IOIN},
		{TMC6200_REG_SHORT_CONF},
		{TMC6200_REG_DRV_CONF},
	};

	uint8_t rx[5][5];
	memset(rx, 0xFF, sizeof(rx));

	spi_transfer_t transfer = {
		.len = sizeof(cmd),
		.numWords = sizeof(cmd),
		.src = cmd,
		.dst = rx,
		.deviceId = SSPI_DEVICE_TMC6200,
	};

	sspi_syncTransfer(transfer);

	return (sspi_tmc_state_t){
		.GCONF.reg = (rx[0][1] << 24) | (rx[0][2] << 16) | (rx[0][3] << 8) | (rx[0][4] << 0),
		.GSTAT.reg = (rx[1][1] << 24) | (rx[1][2] << 16) | (rx[1][3] << 8) | (rx[1][4] << 0),
		.IOIN.reg = (rx[2][1] << 24) | (rx[2][2] << 16) | (rx[2][3] << 8) | (rx[2][4] << 0),
		.SHORT_CONF.reg = (rx[3][1] << 24) | (rx[3][2] << 16) | (rx[3][3] << 8) | (rx[3][4] << 0),
		.DRV_CONF.reg = (rx[4][1] << 24) | (rx[4][2] << 16) | (rx[4][3] << 8) | (rx[4][4] << 0),
	};
}

bool sspi_detectTmc6200(void) {
	// read 5 registers
	sspi_tmc_state_t state = _readState();
/**
 * GSTAT:0 indicates state after reset
 * IOIN bits 31:24 contain the version, should be 0x10
 */

	dbg_println("TMC6200 GSTAT      = 0x%08"PRIx32, state.GSTAT.reg);
	dbg_println("TMC6200 IOIN       = 0x%08"PRIx32, state.IOIN.reg);
	dbg_println("TMC6200 SHORT_CONF = 0x%08"PRIx32, state.SHORT_CONF.reg);
	dbg_println("TMC6200 DRV_CONF   = 0x%08"PRIx32, state.DRV_CONF.reg);

	bool isPresent = (state.GSTAT.reset == 1) 
				  && (state.IOIN.VERSION == 0x10);

	return isPresent;

}

extern sspi_tmc_state_t sspi_tmc_setMotorDriver3PwmMode(void) {

	sspi_tmc_state_t state = _readState();

	state.GCONF.singleline = 1;
	
	// write GCONF
	uint8_t cmd[1][5] = {
		{
			TMC6200_REG_GCONF | 0x80, 
			state.GCONF.reg << 24, 
			state.GCONF.reg << 16, 
			state.GCONF.reg << 8, 
			state.GCONF.reg << 0
		},
	};

	uint8_t rx[1][5];
	memset(rx, 0xFF, sizeof(rx));

	spi_transfer_t transfer = {
		.len = sizeof(cmd),
		.numWords = sizeof(cmd),
		.src = cmd,
		.dst = rx,
		.deviceId = SSPI_DEVICE_TMC6200,
	};

	sspi_syncTransfer(transfer);

	return state;
}
