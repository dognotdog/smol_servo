#include "ssf_spi_private.h"

#include "stm32g4xx_hal.h"
#include "stm32g4xx_hal_spi.h"

#include "debug.h"

#include <string.h>
#include <assert.h>

extern SPI_HandleTypeDef hspi2;
#define drvSpi hspi2

#define DRVCTRL_MODE_PWM3X	(0x1u << 5)
#define DRVCTRL_CLR_FLT 	(0x1u << 0)

#define CSACTRL_CAL_A		(0x1u << 4)
#define CSACTRL_CAL_B		(0x1u << 3)
#define CSACTRL_CAL_C		(0x1u << 2)

void sspi_initDrv83xx(void)
{
	HAL_SPI_DeInit(&drvSpi);

	// drvSpi.Instance = SPI2;

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

	drvSpi.Init = config;
	assert(HAL_SPI_Init(&drvSpi) == HAL_OK);
}

bool sspi_detectDrv83xx(void) {
	sspi_drv_state_t state = sspi_drv_readMotorDriver();

	dbg_println("DRV8323 FAULT_STATUS = 0x%04"PRIx16, state.FAULT_STATUS.reg);
	dbg_println("DRV8323 DRV_CTRL     = 0x%04"PRIx16, state.DRV_CTRL.reg);
	dbg_println("DRV8323 DRV_HS       = 0x%04"PRIx16, state.DRV_HS.reg);
	dbg_println("DRV8323 DRV_LS       = 0x%04"PRIx16, state.DRV_LS.reg);
	dbg_println("DRV8323 OCP_CTRL     = 0x%04"PRIx16, state.OCP_CTRL.reg);
	dbg_println("DRV8323 CSA_CTRL     = 0x%04"PRIx16, state.CSA_CTRL.reg);

	bool isPresent = (state.DRV_CTRL.reg == 0)
				  && (state.DRV_HS.reg == 0x03FF)
				  && (state.DRV_LS.reg == 0x07FF)
				  && (state.OCP_CTRL.reg == 0x0159)
				  && (state.CSA_CTRL.reg == 0x0283);

	if (isPresent)
		sspi.drvDevice = SSPI_DEVICE_DRV83XX;

	return isPresent;
}

void drv83xx_readRegisters(uint8_t* addr, uint16_t* result, size_t numregs)
{
	uint16_t* cmd = result;
	for (size_t i = 0; i < numregs; ++i)
	{
		cmd[i] = (1 << 15) | ((addr[i] & 0x0F) << 11);

		spi_block_t block = {
			.numWords = 1,
			.deviceId = SSPI_DEVICE_DRV83XX,
			.src = cmd,
			.dst = result,
		};

		sspi_transmitBlock(&block);
	}

}

uint16_t sspi_drv_writeDrvMotorDriverReg(size_t addr, uint16_t data)
{
	uint16_t cmd[1] = {
		(addr << 11) | ( data & 0x03FF),
	};

	uint16_t rx[1] = {0xFFFF};

	spi_transfer_t transfer = {
		.len = sizeof(cmd),
		.numWords = sizeof(cmd)/2,
		.src = cmd,
		.dst = rx,
		.deviceId = SSPI_DEVICE_DRV83XX,
	};

	sspi_syncTransfer(transfer);

	// return ssf_spiRW(cmd[0], SSPI_DEVICE_DRV);
	return rx[0];
}

sspi_drv_state_t sspi_drv_setMotorDriver3PwmMode(void)
{
	uint16_t rx = sspi_drv_writeDrvMotorDriverReg(2, DRVCTRL_MODE_PWM3X | DRVCTRL_CLR_FLT);

	sspi_drv_state_t state = {
		.DRV_CTRL 		= {.reg = rx},
	};

	return state;
}

sspi_drv_state_t ssf_enterMotorDriverCalibrationMode(void)
{
	sspi_drv_state_t state = sspi_drv_readMotorDriver();

	uint16_t rx = sspi_drv_writeDrvMotorDriverReg(6, state.CSA_CTRL.reg | (CSACTRL_CAL_A | CSACTRL_CAL_B | CSACTRL_CAL_C));

	state.CSA_CTRL.reg = rx;

	return state;
}

sspi_drv_state_t ssf_exitMotorDriverCalibrationMode(void)
{
	sspi_drv_state_t state = sspi_drv_readMotorDriver();

	uint16_t rx = sspi_drv_writeDrvMotorDriverReg(6, state.CSA_CTRL.reg & ~(CSACTRL_CAL_A | CSACTRL_CAL_B | CSACTRL_CAL_C));

	state.CSA_CTRL.reg = rx;

	return state;
}

sspi_drv_state_t sspi_drv_readMotorDriver(void)
{
	uint8_t cmd[7] = {
		0,1,2,3,4,5,6,
	};

	uint16_t rx[7] = {0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF};

	drv83xx_readRegisters(cmd, rx, 7);

	// for (size_t i = 0; i < 7; ++i)
	// {

	// 	rx[i] = ssf_spiRW(cmd[i], SSPI_DEVICE_DRV);
	// }

	sspi_drv_state_t state = {
		// .NOP = FLIP16(rx[0]),
		.FAULT_STATUS 	= {.reg = rx[0]},
		.VGS_STATUS 	= {.reg = rx[1]},
		.DRV_CTRL 		= {.reg = rx[2]},
		.DRV_HS 		= {.reg = rx[3]},
		.DRV_LS 		= {.reg = rx[4]},
		.OCP_CTRL 		= {.reg = rx[5]},
		.CSA_CTRL 		= {.reg = rx[6]},
	};

	return state;
}
