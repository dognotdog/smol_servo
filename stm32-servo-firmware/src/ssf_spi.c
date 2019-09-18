#include "debug.h"

#include "main.h"
#include "ssf_main.h"
#include "ssf_spi.h"


/*
AS5047 does 16bit SPI transfers
BIT15 is an even parity on commands as well as data, and transfers are big endian.

Propagation delay of sensor is spec'd at 100us +-10us from sensing to read via SPI

At 8Mhz, SPI read of 16bits takes 2us nominally, so ~10us for the 4x16bit words seems a reasonable assumption

The AS5047 clocks out a response at the NEXT transfer.
The DRV8323 responds within the SAME transfer, as it the first few bits of a command are the address plus 11 bytes data, and the response is always only 11 bytes.

on V0.1 hardware, the DRV8323 seems to have difficulty at higher clock rates, so the 128x divider was chosen for ~1.5MHz. The AS5047 has no problem with up to 5MHz

*/

extern SPI_HandleTypeDef hspi2;

#define _XORS(x,s) (x ^ x >> 1)

#define  _EVENPAR(x) (((~_XORS(_XORS(_XORS(_XORS(_XORS(x, 16), 8), 4), 2), 1) & 1) << 15) | (x & 0x7FFF))
// #define FLIP16(x) (((x & 0xFF00) >> 8) | ((x & 0xFF) << 8))
// #define FLIP16(x) (x)

volatile unsigned int counter = 0;

void _delay(unsigned int d) {for (counter = 0; counter < d; ++counter ) {};}

static struct {
	GPIO_TypeDef* 	gpio;
	uint16_t 		pin;
} _deviceSelectPinMap[] = {
	[SSPI_DEVICE_HALL] = {PIN_ASEL},
	[SSPI_DEVICE_DRV] = {PIN_DRVSEL},
};

uint16_t ssf_spiRW(uint16_t cmd, sspi_deviceId_t device)
{
	// drive everything high to deselect chips
	HAL_GPIO_WritePin(PIN_DRVSEL, GPIO_PIN_SET);
	HAL_GPIO_WritePin(PIN_ASEL, GPIO_PIN_SET);
	_delay(50);

	HAL_GPIO_WritePin(_deviceSelectPinMap[device].gpio, _deviceSelectPinMap[device].pin, GPIO_PIN_RESET);
	_delay(50);

	uint16_t result = 0xFFFF;
	// size is 1, as it is the number of transfers, not bytes, and SPI is configured in 16bit mode
	HAL_SPI_TransmitReceive(&hspi2, (void*)&cmd, (void*)&result, 1, 1000);

	HAL_GPIO_WritePin(_deviceSelectPinMap[device].gpio, _deviceSelectPinMap[device].pin, GPIO_PIN_SET);

	return result;
}

sspi_as5047_state_t ssf_readHallSensor(void)
{
	uint16_t cmd[4] = {
		(_EVENPAR(0x4001)),
		(_EVENPAR(0x7FFC)),
		(_EVENPAR(0x7FFE)),
		(_EVENPAR(0x4000)) // NOP at the end to get response to prev command
	};

	uint16_t rx[4] = {0xFFFF,0xFFFF,0xFFFF,0xFFFF};

	for (size_t i = 0; i < 4; ++i)
	{
		rx[i] = ssf_spiRW(cmd[i], SSPI_DEVICE_HALL);
	}

	sspi_as5047_state_t state = {
		// .NOP = FLIP16(rx[0]),
		.ERRFL = (rx[1]),
		.DIAAGC = (rx[2]),
		.ANGLEUNC = (rx[3]),
	};

	return state;
}

sspi_drv_state_t ssf_readMotorDriver(void)
{
	// HAL_GPIO_WritePin(PIN_DRVSEL, GPIO_PIN_SET);

	uint16_t cmd[7] = {
		(1 << 15) | (0 << 11),
		(1 << 15) | (1 << 11),
		(1 << 15) | (2 << 11),
		(1 << 15) | (3 << 11),
		(1 << 15) | (4 << 11),
		(1 << 15) | (5 << 11),
		(1 << 15) | (6 << 11),
		// (1 << 15) | (0 << 11),
	};

	uint16_t rx[7] = {0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF};


	for (size_t i = 0; i < 7; ++i)
	{

		rx[i] = ssf_spiRW(cmd[i], SSPI_DEVICE_DRV);
	}

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

uint16_t ssf_writeMotorDriverReg(size_t addr, uint16_t data)
{
	uint16_t cmd[1] = {
		(addr << 11) | ( data & 0x03FF),
	};

	return ssf_spiRW(cmd[0], SSPI_DEVICE_DRV);
}

#define DRVCTRL_MODE_PWM3X	(0x1u << 5)
#define DRVCTRL_CLR_FLT 	(0x1u << 0)

#define CSACTRL_CAL_A		(0x1u << 4)
#define CSACTRL_CAL_B		(0x1u << 3)
#define CSACTRL_CAL_C		(0x1u << 2)

sspi_drv_state_t ssf_setMotorDriver3PwmMode(void)
{
	uint16_t rx = ssf_writeMotorDriverReg(2, DRVCTRL_MODE_PWM3X | DRVCTRL_CLR_FLT);

	sspi_drv_state_t state = {
		.DRV_CTRL 		= {.reg = rx},
	};

	return state;
}

sspi_drv_state_t ssf_enterMotorDriverCalibrationMode(void)
{
	sspi_drv_state_t state = ssf_readMotorDriver();

	uint16_t rx = ssf_writeMotorDriverReg(6, state.CSA_CTRL.reg | (CSACTRL_CAL_A | CSACTRL_CAL_B | CSACTRL_CAL_C));

	state.CSA_CTRL.reg = rx;

	return state;
}

sspi_drv_state_t ssf_exitMotorDriverCalibrationMode(void)
{
	sspi_drv_state_t state = ssf_readMotorDriver();

	uint16_t rx = ssf_writeMotorDriverReg(6, state.CSA_CTRL.reg & ~(CSACTRL_CAL_A | CSACTRL_CAL_B | CSACTRL_CAL_C));

	state.CSA_CTRL.reg = rx;

	return state;
}



void  ssf_printMotorDriverFaults(sspi_drv_state_t state)
{
	if (state.FAULT_STATUS.FAULT)
		err_println("DRV8323 General Fault");
	if (state.FAULT_STATUS.VDS_OCP)
		err_println("DRV8323 VDS Monitor Overcurrent Fault");
	if (state.FAULT_STATUS.GDF)
		err_println("DRV8323 Gate Driver Fault");
	if (state.FAULT_STATUS.UVLO)
		err_println("DRV8323 Undervoltage Lockout Fault");
	if (state.FAULT_STATUS.OTSD)
		err_println("DRV8323 Overtemperature Shutdown Fault");
	if (state.FAULT_STATUS.VDS_HA)
		err_println("DRV8323 VDS Overcurrent on HS-A Fault");
	if (state.FAULT_STATUS.VDS_LA)
		err_println("DRV8323 VDS Overcurrent on LS-A Fault");
	if (state.FAULT_STATUS.VDS_HB)
		err_println("DRV8323 VDS Overcurrent on HS-B Fault");
	if (state.FAULT_STATUS.VDS_LB)
		err_println("DRV8323 VDS Overcurrent on LS-B Fault");
	if (state.FAULT_STATUS.VDS_HC)
		err_println("DRV8323 VDS Overcurrent on HS-C Fault");
	if (state.FAULT_STATUS.VDS_LC)
		err_println("DRV8323 VDS Overcurrent on LS-C Fault");

	if (state.VGS_STATUS.SA_OC)
		err_println("DRV8323 Sense Amp A Overcurrent");
	if (state.VGS_STATUS.SB_OC)
		err_println("DRV8323 Sense Amp B Overcurrent");
	if (state.VGS_STATUS.SC_OC)
		err_println("DRV8323 Sense Amp C Overcurrent");
	if (state.VGS_STATUS.OTW)
		err_println("DRV8323 Overtemperature Warning");
	if (state.VGS_STATUS.CPUV)
		err_println("DRV8323 Charge Pump Undervoltage Fault");
	if (state.VGS_STATUS.VGS_HA)
		err_println("DRV8323 Gate Drive Overcurrent on HS-A Fault");
	if (state.VGS_STATUS.VGS_LA)
		err_println("DRV8323 Gate Drive Overcurrent on LS-A Fault");
	if (state.VGS_STATUS.VGS_HB)
		err_println("DRV8323 Gate Drive Overcurrent on HS-B Fault");
	if (state.VGS_STATUS.VGS_LB)
		err_println("DRV8323 Gate Drive Overcurrent on LS-B Fault");
	if (state.VGS_STATUS.VGS_HC)
		err_println("DRV8323 Gate Drive Overcurrent on HS-C Fault");
	if (state.VGS_STATUS.VGS_LC)
		err_println("DRV8323 Gate Drive Overcurrent on LS-C Fault");
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

	ssf_setMotorDriver3PwmMode();

	// __HAL_SPI_DISABLE(&hspi2);
}
