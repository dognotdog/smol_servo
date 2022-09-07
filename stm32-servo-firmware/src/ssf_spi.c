#include "debug.h"

#include "main.h"
#include "ssf_main.h"
#include "ssf_mctrl.h"
#include "ssf_spi_private.h"
#include "utime.h"

#include <string.h>
#include <stdatomic.h>
#include <assert.h>

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
extern DMA_HandleTypeDef hdma_spi2_rx;
extern DMA_HandleTypeDef hdma_spi2_tx;

static void sspi_initGpio(void);

sspi_t sspi = {
	.currentDevice = SSPI_DEVICE_UNKNOWN,
	.blockBusy = ATOMIC_FLAG_INIT,
};


static struct {
	GPIO_TypeDef* 	gpio;
	uint16_t 		pin;
} _deviceSelectPinMap[] = {
	[SSPI_DEVICE_AS5047D] = {PIN_ASEL},
	[SSPI_DEVICE_DRV83XX] = {PIN_DRVSEL},
	[SSPI_DEVICE_TMC6200] = {PIN_DRVSEL},
};

static const struct {
	uint16_t	mode;
	uint16_t	baudrate;
	uint16_t	dataSize;
	uint16_t	ldma;
	uint16_t 	dma_rx_ccr;
	uint16_t 	dma_tx_ccr;
} _deviceSelectSpiSettings[] = {
	[SSPI_DEVICE_AS5047D] = {
		.mode = SPI_CR1_CPHA,
		.baudrate = SPI_CR1_BR_2 | SPI_CR1_BR_1, // 0b110 = 1/128
		.dataSize = SPI_CR2_DS_3 | SPI_CR2_DS_2 | SPI_CR2_DS_1 | SPI_CR2_DS_0, // 16-bit
		.ldma = 0,
		.dma_rx_ccr = DMA_CCR_PL_0 | DMA_CCR_MSIZE_0 | DMA_CCR_PSIZE_0 | DMA_CCR_MINC,
		.dma_tx_ccr = DMA_CCR_PL_0 | DMA_CCR_MSIZE_0 | DMA_CCR_PSIZE_0 | DMA_CCR_MINC | DMA_CCR_DIR,
	},
	[SSPI_DEVICE_DRV83XX] = {
		.mode = SPI_CR1_CPHA,
		.baudrate = SPI_CR1_BR_2 | SPI_CR1_BR_0, // 0b101 = 1/64
		.dataSize = SPI_CR2_DS_3 | SPI_CR2_DS_2 | SPI_CR2_DS_1 | SPI_CR2_DS_0, // 16-bit
		.ldma = 0,
		.dma_rx_ccr = DMA_CCR_PL_0 | DMA_CCR_MSIZE_0 | DMA_CCR_PSIZE_0 | DMA_CCR_MINC,
		.dma_tx_ccr = DMA_CCR_PL_0 | DMA_CCR_MSIZE_0 | DMA_CCR_PSIZE_0 | DMA_CCR_MINC | DMA_CCR_DIR,
	},
	[SSPI_DEVICE_TMC6200] = {
		.mode = SPI_CR1_CPOL | SPI_CR1_CPHA,
		.baudrate = SPI_CR1_BR_2 | SPI_CR1_BR_0, // 0b101 = 1/64
		.dataSize = SPI_CR2_DS_2 | SPI_CR2_DS_1 | SPI_CR2_DS_0, // 8-bit
		.ldma = SPI_CR2_LDMARX | SPI_CR2_LDMATX | SPI_CR2_FRXTH, // because 5 bytes per block
		.dma_rx_ccr = DMA_CCR_PL_0 | DMA_CCR_MINC,
		.dma_tx_ccr = DMA_CCR_PL_0 | DMA_CCR_MINC | DMA_CCR_DIR,
	},
};

volatile unsigned int _delayCcounter = 0;

void _delay(unsigned int d) {for (_delayCcounter = 0; _delayCcounter < d; ++_delayCcounter ) {};}

/*
	elaborate delay scheme to avoid wasting cycles:
	- as hardware NSS doesn't work, it has to be pulsed in software after each data word
	- SPI access is serialized, so we can use on timer to do this
	- callback to trigger next word transfer after elapsed time
	- effective forced delay to completion on end of transfer

*/
static void (*_timingCallback)(void) = NULL;

static void _delayWithCallback(float delay_us, void (*callback)(void))
{
	_timingCallback = callback;
	__HAL_TIM_SET_AUTORELOAD(HTIM_SPI, 170.0f*delay_us);
	HAL_TIM_OnePulse_Start_IT(HTIM_SPI, 0);
	__HAL_TIM_ENABLE(HTIM_SPI);
}



void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	// dbg_println("HAL_TIM_PeriodElapsedCallback");
	if (htim == HTIM_SPI)
	{
		if (_timingCallback)
			_timingCallback();
		_timingCallback = NULL;
	}
}
void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{
	HAL_TIM_PeriodElapsedCallback(htim);
}
void HAL_TIM_TriggerCallback(TIM_HandleTypeDef *htim)
{
	HAL_TIM_PeriodElapsedCallback(htim);
}


// void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi)
// {
// 	atomic_store(&_transferStatus, SPI_XFER_ERROR);
// 	err_println("HAL_SPI_ErrorCallback");
// 	if (sspi.currentTransfer.callback)
// 		sspi.currentTransfer.callback(&sspi.currentTransfer, false);
// 	atomic_store(&_transferStatus, SPI_XFER_IDLE);

// }

// void HAL_SPI_AbortCallback(SPI_HandleTypeDef *hspi)
// {
// 	atomic_store(&_transferStatus, SPI_XFER_ERROR);
// 	err_println("HAL_SPI_AbortCallback");
// 	if (sspi.currentTransfer.callback)
// 		sspi.currentTransfer.callback(&sspi.currentTransfer, false);
// 	atomic_store(&_transferStatus, SPI_XFER_IDLE);

// }


// static uint16_t _hallSensorTxBuf[3] = {0xFFFF, 0xFFFF, 0xFFFF};
// static uint16_t _hallSensorRxBuf[3] = {0xFFFF, 0xFFFF, 0xFFFF};

// static uint32_t _asyncReadHallSensorTick = 0;

// static void _readHallSensorCallback(bool transferOk)
// {
// 	// dbg_println("_readHallSensorCallback()...");

// 	uint32_t tock = utime_now();

// 	sspi_as5047_state_t state = {
// 		.ERRFL = (_hallSensorRxBuf[0]),
// 		.DIAAGC = (_hallSensorRxBuf[1]),
// 		.ANGLEUNC = (_hallSensorRxBuf[2]),
// 		.start_us = _asyncReadHallSensorTick,
// 		.end_us = tock,
// 	};	

// 	ssf_asyncReadHallSensorCallback(state, transferOk);
// }


// int ssf_asyncReadHallSensor(void)
// {
// 	// we don't have enough time in the mctrl function to switch SPI modes
// 	if (sspi.currentDevice != SSPI_DEVICE_AS5047D)
// 		return -2;

// 	// dbg_println("ssf_asyncReadHallSensor()...");
// 	uint16_t cmd[3] = {
// 		(_PAR(0x4001)), // ERRFL
// 		(_PAR(0x7FFC)), // DIAAGC
// 		(_PAR(0x7FFE)), // ANGLEUNC
// 	};

// 	memcpy((void*)_hallSensorTxBuf, cmd, sizeof(_hallSensorTxBuf));
// 	memset((void*)_hallSensorRxBuf, 0x0F, sizeof(_hallSensorRxBuf));

// 	_asyncReadHallSensorTick = utime_now();

// 	as5047d_readRegisters(_hallSensorTxBuf, _hallSensorRxBuf, 3, _readHallSensorCallback);

// 	return 0;
// }

void ssf_setMotorDriver3PwmMode(void)
{
	switch (sspi.drvDevice) {
		case SSPI_DEVICE_DRV83XX:
		{
			sspi_drv_setMotorDriver3PwmMode();
			break;
		}
		case SSPI_DEVICE_TMC6200:
		{
			sspi_tmc_setMotorDriver3PwmMode();
			break;
		}
		default:
		{

		}
	}
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

static bool _checkSpiEncoderRegReadOk(uint16_t reg)
{
	return (_PAR(reg) == reg) && ((reg & 0x4000) == 0);
}

bool ssf_checkSpiEncoderReadOk(sspi_as5047_state_t state, bool* formatError, bool* valueError)
{
	bool formatOk = _checkSpiEncoderRegReadOk(state.DIAAGC) 
					&& _checkSpiEncoderRegReadOk(state.ERRFL) 
					&& _checkSpiEncoderRegReadOk(state.ANGLEUNC);
	bool valueOk = ((state.ERRFL  & 0x0007) == 0) // no SPI error flags
					&& ((state.DIAAGC & 0x0100) != 0) // offset loops ready
					&& ((state.DIAAGC & 0x0200) == 0) // no cordic overflow
					&& ((state.DIAAGC & 0x0C00) != 0x0C00) // mag-hi and mag-lo not active at once
					;
	if (formatError)
		*formatError = !formatOk;
	if (valueError)
		*valueError = !valueOk;
	return formatOk && valueOk;
}

void ssf_dbgPrintEncoderStatus(sspi_as5047_state_t state)
{
	// err_println("AS5047D ERRFL    = 0x%04x.", state.ERRFL);
	// err_println("AS5047D DIAAGC   = 0x%04x.", state.DIAAGC);
	// err_println("AS5047D ANGLEUNC = 0x%04x.", state.ANGLEUNC);

	if (state.ERRFL & 0x4000)
	{
		err_println("AS5047D ERRFL not read successfully.");
	}
	else if (_PAR(state.ERRFL) == state.ERRFL)
	{
		if (state.ERRFL & 0x0001)
			err_println("AS5047D SPI Framing Error");
		if (state.ERRFL & 0x0002)
			err_println("AS5047D Invalid Command Error");
		if (state.ERRFL & 0x0004)
			err_println("AS5047D Parity Error");
	}

	if (state.DIAAGC & 0x4000)
	{
		err_println("AS5047D DIAAGC not read successfully.");
	}
	else if (_PAR(state.DIAAGC) == state.DIAAGC)
	{
		if ((state.DIAAGC & 0x0100) == 0)
			err_println("AS5047D Internal Offset Loops Not Ready");
		if (state.DIAAGC & 0x0200)
			err_println("AS5047D CORDIC Overflow");
		if (state.DIAAGC & 0x0400)
			err_println("AS5047D Magnetic Field Strength Too High");
		if (state.DIAAGC & 0x0800)
			warn_println("AS5047D Magnetic Field Strength Too Low");
		// if ((state.DIAAGC & 0x0E00) != 0)
		// {
		// 	warn_println("AS5047D NOP   =0x%04X", state.NOP);
		// 	warn_println("AS5047D ERRFL =0x%04X", state.ERRFL);
		// 	warn_println("AS5047D DIAAGC=0x%04X", state.DIAAGC);
		// }
		// dbg_println("AS5047D AGC = %u", state.DIAAGC & 0xFF);
	}

	// 0x3FF mask as msb are parity and error flags
	// dbg_println("AS5047D ANGLEUNC = %5u (%6.3f deg)", state.ANGLEUNC & 0x3FFF, (double)((float)(state.ANGLEUNC & 0x3FFF)/0x4000*360.0f));

}


static void _setPinAsGpioOutput(GPIO_TypeDef *gpio, uint32_t GPIO_Pin)
{
	// this works for single pins only
	// the pins values are 2^n, not n, as one might expect
	// that's why computing the mask this way works
	uint32_t shift = (GPIO_Pin*GPIO_Pin);
	uint32_t modeMask = shift | (shift << 1);

	gpio->MODER = (gpio->MODER & ~modeMask) | (shift*0x01);
}

// extern DMA_HandleTypeDef hdma_spi2_rx;
// static void _spiDmaCallback(DMA_HandleTypeDef* hdma)
// {
// 	dbg_println("_spiDmaCallback");
// }

void ssf_spiInit(void)
{
	_setPinAsGpioOutput(PIN_DRVSEL);
	_setPinAsGpioOutput(PIN_ASEL);
	// Hardware NSS is garbage (tied to SPI being enabled instead of transfers) so use GPIO
	// as OR gate chip expected to be not populated, don't do anything, actually, and leave this pin alone
	// _setPinAsGpioOutput(PIN_SPI_NSS);

	HAL_GPIO_WritePin(PIN_DRVSEL, GPIO_PIN_SET);
	HAL_GPIO_WritePin(PIN_ASEL, GPIO_PIN_SET);
	// HAL_GPIO_WritePin(PIN_SPI_NSS, GPIO_PIN_RESET);

	_setPinAsGpioOutput(PIN_DRVEN);
	HAL_GPIO_WritePin(PIN_DRVEN, GPIO_PIN_SET);

	// HAL_DMA_RegisterCallback(&hdma_spi2_rx, HAL_DMA_XFER_CPLT_CB_ID, _spiDmaCallback);

	// __HAL_SPI_DISABLE(&hspi2);

	sspi_initGpio();
}


#define INITIAL_TRIGGER_DMA_CH 2
#define HOLD_DONE_DMA_CH 3
#define TX_DONE_DMA_CH 4

void spi_setupTriggeredTransfer(void)
{
	// initial DMA event 
	// tim_itr1 to start TIM8, and set NSS GPIO LOW
	DMA_HandleTypeDef dma1 = {
		.Instance = DMA1_Channel4,
		.Init = {
			.Request = DMA_REQUEST_TIM2_UP,
			.Direction = DMA_MEMORY_TO_PERIPH,
			.PeriphInc = DMA_PINC_DISABLE,
			.MemInc = DMA_MINC_DISABLE,
			.PeriphDataAlignment = DMA_PDATAALIGN_WORD,
			.MemDataAlignment = DMA_MDATAALIGN_WORD,
			.Mode = DMA_NORMAL,
			.Priority = DMA_PRIORITY_LOW,
		},
	};

	// TIM8_CH1 means transfer can start, so write to SPI
	DMA_HandleTypeDef dma2 = {
		.Instance = DMA1_Channel5,
		.Init = {
			.Request = DMA_REQUEST_TIM8_CH1,
			.Direction = DMA_MEMORY_TO_PERIPH,
			.PeriphInc = DMA_PINC_DISABLE,
			.MemInc = DMA_MINC_DISABLE,
			.PeriphDataAlignment = DMA_PDATAALIGN_WORD,
			.MemDataAlignment = DMA_MDATAALIGN_WORD,
			.Mode = DMA_NORMAL,
			.Priority = DMA_PRIORITY_LOW,
		},
	};
	// DMA_GENERATOR0 is setup to trigger on SPI_TX DMA done
	// set NSS high again
	DMA_HandleTypeDef dma3 = {
		.Instance = DMA1_Channel3,
		.Init = {
			// .Request = DMA_REQUEST_SPI2_TX,
			.Request = DMA_REQUEST_GENERATOR0,
			.Direction = DMA_MEMORY_TO_PERIPH,
			.PeriphInc = DMA_PINC_DISABLE,
			.MemInc = DMA_MINC_DISABLE,
			.PeriphDataAlignment = DMA_PDATAALIGN_WORD,
			.MemDataAlignment = DMA_MDATAALIGN_WORD,
			.Mode = DMA_NORMAL,
			.Priority = DMA_PRIORITY_LOW,
		},
	};

	HAL_DMA_MuxRequestGeneratorConfigTypeDef req0 = {
		.SignalID = HAL_DMAMUX1_REQ_GEN_DMAMUX1_CH2_EVT,
		.Polarity = HAL_DMAMUX_REQ_GEN_RISING,
		.RequestNumber = 1,
	};

	if (HAL_DMA_Init(&dma1) != HAL_OK)
	{
		Error_Handler();
	}
	if (HAL_DMA_Init(&dma2) != HAL_OK)
	{
		Error_Handler();
	}
	if (HAL_DMA_Init(&dma3) != HAL_OK)
	{
		Error_Handler();
	}

	HAL_DMAEx_ConfigMuxRequestGenerator(&dma3, &req0);
}


/*******************************************************************************
 * This is the driver parts without the HAL, which needlessly complicates things for our purposes.
 * 
 * FIFO size is 32bits for RX and TX each. We want the following setup:
 *  - per-transfer config of CPOL/CPHA and word size
 *  - 1-5 words per transfer block for NSS
 *  - NSS assert, transmit, NSS deassert without software intervention for each transfer block
 * 
 * We needs SPI reads that are in lock-step with the fast mctrl loop.
 * 
 * The mctrl_fastLoop() is triggered when the ADC DMAs are completed, but we have no fixed sync point with the TIM2 PWM. Ideally, we'd kick off each transfer at the exact TIM2 trigger, but a trigger from mctrl_fastLoop() is ok for now.
 * 
 * To deassert NSS, we need disable the SPI peripheral in the RX DMA Complete handler.
 */

/**
 * deinit() will bock until SPI finishes transactions
 */
void spi_deinit(void)
{
	SPI_TypeDef* spiRegs = SPI2;

	// care must be taken the transactions are complete before disabling the SPI peripheral.

	while (spiRegs->SR & SPI_SR_FTLVL_Msk) {};
	while (spiRegs->SR & SPI_SR_BSY_Msk) {};


	spiRegs->CR1 = spiRegs->CR1 & ~SPI_CR1_SPE;

	while (spiRegs->SR & SPI_SR_FRLVL_Msk) 
	{
		uint32_t dummy = spiRegs->DR;
		(void)dummy;
	};

	// disable DMA
	spiRegs->CR2 = spiRegs->CR2 & ~(SPI_CR2_RXDMAEN | SPI_CR2_RXDMAEN);

}

static void _spiDisable(void)
{
	SPI_TypeDef* spiRegs = SPI2;

	while (spiRegs->SR & SPI_SR_FTLVL_Msk) {};
	while (spiRegs->SR & SPI_SR_BSY_Msk) {};

	spiRegs->CR1 &= ~SPI_CR1_SPE;

	while (spiRegs->SR & SPI_SR_FRLVL_Msk) 
	{
		uint32_t dummy = spiRegs->DR;
		(void)dummy;
	};

	// disable DMA
	spiRegs->CR2 &= ~(SPI_CR2_RXDMAEN | SPI_CR2_RXDMAEN);

}

sspi_deviceId_t sspi_drvType(void)
{
	return sspi.drvDevice;
}

/**
 * This only needs to be called once
 */
void sspi_initGpio(void) {
	// PB12 is NSS
	// PB13 is CLK
	// SPI peripheral controlled GPIOs
	{
		GPIO_InitTypeDef config = {
		    .Pin = /*GPIO_PIN_12|*/GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15,
		    .Mode = GPIO_MODE_AF_PP,
		    .Pull = GPIO_NOPULL,
		    .Speed = GPIO_SPEED_FREQ_VERY_HIGH,
		    .Alternate = GPIO_AF5_SPI2,
		};
	    HAL_GPIO_Init(GPIOB, &config);
	}

	// keep NSS inactive, if we manage it through the manual GPIOs
	if (0)
	{
		GPIO_InitTypeDef config = {
		    .Pin = GPIO_PIN_12,
		    .Mode = GPIO_MODE_INPUT,
		    .Pull = GPIO_PULLUP,
		    .Speed = GPIO_SPEED_FREQ_VERY_HIGH,
		};
	    HAL_GPIO_Init(GPIOB, &config);
	}

	// enable pullups on mosi/miso PB.14,PB.15 and NSS PB.12
	GPIOB->PUPDR = (GPIOB->PUPDR & ~(GPIO_PUPDR_PUPD12 | GPIO_PUPDR_PUPD14 | GPIO_PUPDR_PUPD15)) | GPIO_PUPDR_PUPD12_0 |GPIO_PUPDR_PUPD14_0 | GPIO_PUPDR_PUPD15_0;

	// reduce drive strengths on CLK B.13 and MOSI B.15
	// b00 = low, b01 = med, b10 = high, b11 = very high
	GPIOB->OSPEEDR = (GPIOB->OSPEEDR & ~(GPIO_OSPEEDR_OSPEED13 | GPIO_OSPEEDR_OSPEED15)) | 0;

}

static void _clkPinSetPol(bool pullup)
{
	// SPI_CLK = B.13
	// configure pullup/down
	uint16_t pupd = pullup ? GPIO_PUPDR_PUPD13_0 : GPIO_PUPDR_PUPD13_1;
	GPIOB->PUPDR = (GPIOB->PUPDR & ~GPIO_PUPDR_PUPD13) | pupd;
	// make it an input
	GPIOB->MODER = (GPIOB->MODER & ~GPIO_MODER_MODE13);

}

static void _clkPinAlternateFun(void) {
	// SPI_CLK = B.13
	// select alternate mode
	GPIOB->MODER = (GPIOB->MODER & ~GPIO_MODER_MODE13) | GPIO_MODER_MODE13_1;
}

void spi_initChipSelects(sspi_deviceId_t deviceId)
{

	sspi_deviceId_t inactiveDeviceId = SSPI_DEVICE_UNKNOWN;
	switch (deviceId)
	{
		case SSPI_DEVICE_TMC6200:
		case SSPI_DEVICE_DRV83XX:
		{
			inactiveDeviceId = SSPI_DEVICE_AS5047D;
			break;
		}
		case SSPI_DEVICE_AS5047D:
		{
			inactiveDeviceId = SSPI_DEVICE_TMC6200;
			break;
		}
		default:
		{
			break;
		}
	}

	assert(inactiveDeviceId != SSPI_DEVICE_UNKNOWN);

	// set inactive Pin HIGH
	{
		GPIO_TypeDef* 	inactiveGpio = _deviceSelectPinMap[inactiveDeviceId].gpio;
		uint16_t 		inactivePin = _deviceSelectPinMap[inactiveDeviceId].pin;

		// GPIO_InitTypeDef config = {
		//     .Pin = inactivePin,
		//     .Mode = GPIO_MODE_OUTPUT_PP,
		//     .Pull = GPIO_NOPULL,
		//     .Speed = GPIO_SPEED_FREQ_VERY_HIGH,
		// };
	 //    HAL_GPIO_Init(inactiveGpio, &config);

		HAL_GPIO_WritePin(inactiveGpio, inactivePin, GPIO_PIN_SET);

	}

	// init CLK to be the right polarity before we enable the SPI peripheral

	if (0)
	{
		// set active pin Hi-Z, so that NSS can pull control it.
		GPIO_TypeDef* 	activeGpio = _deviceSelectPinMap[deviceId].gpio;
		uint16_t 		activePin = _deviceSelectPinMap[deviceId].pin;

		GPIO_InitTypeDef config = {
		    .Pin = activePin,
		    .Mode = GPIO_MODE_INPUT,
		    .Pull = GPIO_NOPULL,
		    .Speed = GPIO_SPEED_FREQ_VERY_HIGH,
		};
	    HAL_GPIO_Init(activeGpio, &config);
	}
	else
	{
		// software control of chip selects
		GPIO_TypeDef* 	activeGpio = _deviceSelectPinMap[deviceId].gpio;
		uint16_t 		activePin = _deviceSelectPinMap[deviceId].pin;

		// GPIO_InitTypeDef config = {
		//     .Pin = activePin,
		//     .Mode = GPIO_MODE_OUTPUT_PP,
		//     .Pull = GPIO_NOPULL,
		//     .Speed = GPIO_SPEED_FREQ_VERY_HIGH,
		// };
	 //    HAL_GPIO_Init(activeGpio, &config);
		HAL_GPIO_WritePin(activeGpio, activePin, GPIO_PIN_SET);

	}
}

/**
 * This needs to be called every time we switch modes
 */
void spi_initPeripheral(sspi_deviceId_t deviceId) 
{

	// NOP when the device is already active
	if (deviceId == sspi.currentDevice)
		return;

	// make sure GPIO selector is high for non-selected devices
	// and Hi-Z for selected devices
	// (hardware NSS will pull that low when SPI is enabled)
	spi_initChipSelects(deviceId);

	_clkPinSetPol((_deviceSelectSpiSettings[deviceId].mode & SPI_CR1_CPOL) != 0);

	SPI_TypeDef* spiRegs = SPI2;

	uint16_t mode = _deviceSelectSpiSettings[deviceId].mode;
	uint16_t baudrate = _deviceSelectSpiSettings[deviceId].baudrate;
	uint16_t dataSize = _deviceSelectSpiSettings[deviceId].dataSize;
	uint16_t ldma = _deviceSelectSpiSettings[deviceId].ldma;

	// reset CR1 to use bidirectional, no CRC, disable
	// master mode, clock rate 1/128 (6)
	// CPOL/CPHA need to be set, too, but that depends on the attached device
	spiRegs->CR1 = mode | baudrate | SPI_CR1_MSTR;
	// software slave management
	// spiRegs->CR1 |= SPI_CR1_SSM | SPI_CR1_SSI;

	// enable SSOE so that NSS is pulled low when SPI enabled
	// don't enable any interrupts
	// set specific even/odd (ldma) and dataSize modes
	spiRegs->CR2 = /*SPI_CR2_SSOE |*/ dataSize | ldma;

	DMA_Channel_TypeDef* rx_dma = hdma_spi2_rx.Instance;
	DMA_Channel_TypeDef* tx_dma = hdma_spi2_tx.Instance;

	rx_dma->CCR = _deviceSelectSpiSettings[deviceId].dma_rx_ccr;
	tx_dma->CCR = _deviceSelectSpiSettings[deviceId].dma_tx_ccr;

	// enable RX interrupt, but don't care about TX
  	HAL_NVIC_SetPriority(DMA2_Channel1_IRQn, 2, 0);
  	HAL_NVIC_EnableIRQ(DMA2_Channel1_IRQn);
  	HAL_NVIC_DisableIRQ(DMA2_Channel2_IRQn);

	sspi.currentDevice = deviceId;
}

int spi_transmitBlockPrep(spi_block_t* block)
{
	// attempt to transmit block if idle
	// if already busy, abort attempt
	if (atomic_flag_test_and_set(&sspi.blockBusy))
	{
		if (sspi.currentBlock->callback)
		{
			sspi.currentBlock->callback(false);
		}
		return -1;
	}

	sspi.currentBlock = block;
	spi_initPeripheral(block->deviceId);

	SPI_TypeDef* spiRegs = SPI2;

	// enable the SPI peripheral 
	spiRegs->CR1 |= SPI_CR1_SPE;

	// release clock pin to peripheral
	_clkPinAlternateFun();

	// enable DMA streams

	DMA_TypeDef* dma = DMA2;
	DMA_Channel_TypeDef* rx_dma = hdma_spi2_rx.Instance;
	DMA_Channel_TypeDef* tx_dma = hdma_spi2_tx.Instance;

	// disable for changing settings
	rx_dma->CCR &= ~DMA_CCR_EN;
	tx_dma->CCR &= ~DMA_CCR_EN;

	// clear interrupt statuses
	dma->IFCR = DMA_IFCR_CGIF1 | DMA_IFCR_CTCIF1 | DMA_IFCR_CHTIF1 | DMA_IFCR_CTEIF1;
	dma->IFCR = DMA_IFCR_CGIF2 | DMA_IFCR_CTCIF2 | DMA_IFCR_CHTIF2 | DMA_IFCR_CTEIF2;


	// assert NSS after enabling peripheral, so that eg. clock polarity does not glitch
	GPIO_TypeDef* 	activeGpio = _deviceSelectPinMap[block->deviceId].gpio;
	uint16_t 		activePin = _deviceSelectPinMap[block->deviceId].pin;
	HAL_GPIO_WritePin(activeGpio, activePin, GPIO_PIN_RESET);


	// set DMA transfer size and pointers
	rx_dma->CNDTR = block->numWords;
	tx_dma->CNDTR = block->numWords;
	rx_dma->CPAR = (uint32_t)&spiRegs->DR;
	tx_dma->CPAR = (uint32_t)&spiRegs->DR;
	rx_dma->CMAR = (uint32_t)block->dst;
	tx_dma->CMAR = (uint32_t)block->src;

	// enable DMA streams
	// with transfer complete interrupt on RX
	rx_dma->CCR |= DMA_CCR_TCIE;
	rx_dma->CCR |= DMA_CCR_EN;
	tx_dma->CCR |= DMA_CCR_EN;


	// enable RX DMA
	spiRegs->CR2 |= SPI_CR2_RXDMAEN;

	block->transferComplete = false;



	return 0;
}

void spi_transmitBlockStart(spi_block_t* block)
{
	assert(block == sspi.currentBlock);

	// dbg_println("reset pin %p, %i", activeGpio, activePin);

	SPI_TypeDef* spiRegs = SPI2;

	// enable TX DMA as the last step
	spiRegs->CR2 |= SPI_CR2_TXDMAEN;
}

void spi_rxDmaCompleteHandler(DMA_TypeDef* rx_dma)
{
	// dbg_println("spi_rxDmaCompleteHandler");
	if (rx_dma->ISR & DMA_ISR_TCIF1)
	{
		// acknowledge interrupt
		rx_dma->IFCR = DMA_IFCR_CTCIF1 | DMA_IFCR_CGIF1;

		// received all data, deassert NSS before disabling the peripheral

		GPIO_TypeDef* 	activeGpio = _deviceSelectPinMap[sspi.currentBlock->deviceId].gpio;
		uint16_t 		activePin = _deviceSelectPinMap[sspi.currentBlock->deviceId].pin;
		HAL_GPIO_WritePin(activeGpio, activePin, GPIO_PIN_SET);

		DMA_Channel_TypeDef* rx_dma = hdma_spi2_rx.Instance;
		DMA_Channel_TypeDef* tx_dma = hdma_spi2_tx.Instance;

		// disable dma channels
		rx_dma->CCR &= ~DMA_CCR_EN;
		tx_dma->CCR &= ~DMA_CCR_EN;


		// disable the SPI peripheral
		_spiDisable();

		assert(sspi.currentBlock != NULL);


		sspi.currentBlock->transferComplete = true;


		if (sspi.currentBlock->callback)
		{
			sspi.currentBlock->callback(true);
		}
		atomic_flag_clear(&sspi.blockBusy);
	}
	else if (rx_dma->ISR & DMA_ISR_GIF1)
	{
		// other interrupts we just ack
		// acknowledge interrupt
		rx_dma->IFCR = DMA_IFCR_CHTIF1 | DMA_IFCR_CTEIF1 | DMA_IFCR_CGIF1;

		sspi.currentBlock->transferComplete = false;

		if (sspi.currentBlock->callback)
		{
			sspi.currentBlock->callback(false);
		}
		atomic_flag_clear(&sspi.blockBusy);
	}
}

void spi_transmitBlockWait(volatile spi_block_t* block)
{
	SPI_TypeDef* spiRegs = SPI2;



	assert(sspi.currentBlock == block);
	while (!block->transferComplete) 
	{
		// if (spiRegs->SR & SPI_SR_RXNE)
		// 	dbg_println("SPI2 RX buffer not empty...");
		// if (spiRegs->SR & SPI_SR_TXE)
		// 	dbg_println("SPI2 TX buffer empty...");
		if (spiRegs->SR & SPI_SR_FRE)
			dbg_println("SPI2 Frame format error...");
		if (spiRegs->SR & SPI_SR_OVR)
			dbg_println("SPI2 overrun error...");
		if (spiRegs->SR & SPI_SR_MODF)
			dbg_println("SPI2 mode fault...");
		if (spiRegs->SR & SPI_SR_CRCERR)
			dbg_println("SPI2 CRC error...");
		if (spiRegs->SR & SPI_SR_UDR)
			dbg_println("SPI2 underrun error...");
	};
}

/**
 * Synchronous transmission. Returns after block callback is called from
 * interrupt.
 */
int sspi_transmitBlock(spi_block_t* block) 
{
	int result = spi_transmitBlockPrep(block);
	if (0 == result)
	{
		spi_transmitBlockStart(block);
		spi_transmitBlockWait(block);		
	}
	else
	{
		err_println("failed to prep SPI block");
	}
	return result;
}

/**
 * Asynchronous transmit does not wait for tansmission to finish,
 * but leaves that to the interrupt block callback.
 */
int sspi_asyncTransmitBlock(spi_block_t* block) 
{
	int result = spi_transmitBlockPrep(block);
	if (0 == result)
	{
		spi_transmitBlockStart(block);
	}
	else
	{
		// err_println("failed to prep SPI block");
	}
	return result;
}


void spi_blockTest(void) {

	sspi_initGpio();

	sspi.currentDevice = SSPI_DEVICE_UNKNOWN;

	// bool toggle = false;
	for (size_t i = 5; true; ++i)
	{
		// HAL_GPIO_WritePin(PIN_DRVSEL, toggle);
		// toggle = !toggle;

		if (1)
		{
			sspi_detectAs5047d();
			sspi_detectTmc6200();
			sspi_detectDrv83xx();
		}
		if (0)
		{
			uint8_t src[2] = {i >> 0, 0};
			volatile uint8_t dst[2] = {0xFE, 0xFF};

			dbg_println("src = %02X %02X", src[1], src[0]);

			spi_block_t block = {
				.src = src,
				.dst = dst,
				.deviceId = SSPI_DEVICE_AS5047D,
				.numWords = 1,
			};

			if (0 != sspi_transmitBlock(&block))
				err_println("oops, failed to transmit block!");

			dbg_println("dst = %02X %02X", dst[1], dst[0]);
		}

		if (0)
		{
			uint8_t src[5] = {0, i >> 24, i >> 16, i >> 8, i >> 0};
			volatile uint8_t dst[5] = {0xFB, 0xFC, 0xFD, 0xFE, 0xFF};

			dbg_println("src = %02X %02X %02X %02X %02X", src[0], src[1], src[2], src[3], src[4]);

			spi_block_t block = {
				.src = src,
				.dst = dst,
				.deviceId = SSPI_DEVICE_TMC6200,
				.numWords = 5,
			};

			if (0 != sspi_transmitBlock(&block))
				err_println("oops, failed to transmit block!");

			dbg_println("dst = %02X %02X %02X %02X %02X", dst[0], dst[1], dst[2], dst[3], dst[4]);
		}
	}
}

void sspi_enableFastloopReads(bool enable)
{
	sspi.fastloopEnabled = enable;
}



/**
 * The fastloop functionis called for pipelined SPI calls.
 * 
 * The fastloop function is called from the fast control loop to kick of SPI transactions. 
 * 
 * The fastloop reads are as follows: We alternate between angle readouts and
 * diagnostic readouts from the driver chip and magnetic encoder
 *  0: AS5047D	ANGLEUNC
 *  1: AS5047D	ERRFL
 *  2: AS5047D	ANGLEUNC
 *  3: AS5047D	DIAAGC
 *  4: AS5047D	ANGLEUNC
 *  5: TMC6200	GSTAT 		DRV8323 0x00 (Fault Status 1)
 *  6: AS5047D	ANGLEUNC
 *  7: TMC6200	IOIN 		DRV8323 0x01 (VGS Status 2)
 *  8: AS5047D	ANGLEUNC
 *  9: TMC6200	GCONF 		DRV8323 ???

 * If the registers need to be written, we have to stop the fastloop
 * readouts, and write registers to recover from a lower priority thread.
 * 
 * The only real complication is that the AS5047D register read results
 * are returned in the next block.
 * 
 * TODO: are the angle reads latched when the read command is sent, or when the actual readout occurs (next cycle)? We should be able to figure this out later by analyzing reads in motion, for now we just assume latch on command.
 */

typedef enum {
	SSPI_FL_HALL_ANGLEU_AFTER_AN,
	SSPI_FL_HALL_ANGLEU_AFTER_S0,
	SSPI_FL_HALL_ANGLEU_AFTER_S1,
	SSPI_FL_HALL_STATUS0,
	SSPI_FL_HALL_STATUS1,
	SSPI_FL_DRV_STATUS0,
	SSPI_FL_DRV_STATUS1,
	SSPI_FL_DRV_STATUS2,
} sspi_fastloop_cycle_name_t;

static const sspi_fastloop_cycle_name_t _flCycle[] = {
	SSPI_FL_HALL_ANGLEU_AFTER_AN,
	SSPI_FL_HALL_STATUS0,
	SSPI_FL_HALL_ANGLEU_AFTER_S0,
	SSPI_FL_HALL_STATUS1,
	SSPI_FL_HALL_ANGLEU_AFTER_S1,
	SSPI_FL_DRV_STATUS0,
	SSPI_FL_HALL_ANGLEU_AFTER_AN,
	SSPI_FL_DRV_STATUS1,
	SSPI_FL_HALL_ANGLEU_AFTER_AN,
	SSPI_FL_DRV_STATUS2,
};
static size_t sspi_flCycleCounter = 0;

static struct {
	uint32_t blockTime;
	uint32_t angleTime;
	uint16_t as_angle[1];
	uint16_t as_diag[1];
	uint16_t as_err[1];
	uint16_t drv_fault[1];
	uint16_t drv_vgs[1];
	uint8_t tmc_gstat[5];
	uint8_t tmc_ioin[5];
	uint8_t tmc_gconf[5];
} sspi_fl_rx;


void _blockCallback(bool transferOk)
{
	sspi_fastloop_cycle_name_t cycle = _flCycle[sspi_flCycleCounter];
	if (transferOk)
	{
		switch (cycle)
		{
			case SSPI_FL_HALL_ANGLEU_AFTER_AN:			
			case SSPI_FL_HALL_STATUS0:		
			case SSPI_FL_HALL_STATUS1:		
			{
				// the RX result is the angle, record the time
				sspi_fl_rx.angleTime = sspi_fl_rx.blockTime;

				uint32_t tock = utime_now();

				sspi_as5047_state_t state = {
					.ERRFL = sspi_fl_rx.as_err[0],
					.DIAAGC = sspi_fl_rx.as_diag[0],
					.ANGLEUNC = sspi_fl_rx.as_angle[0],
					.start_us = sspi_fl_rx.angleTime,
					.end_us = tock,
				};	

				ssf_asyncReadHallSensorCallback(state, transferOk);

				break;
			}
			case SSPI_FL_DRV_STATUS0:
			{
				switch (sspi.drvDevice)
				{
					case SSPI_DEVICE_TMC6200:
					{
						mctrl_diag_tmc6200_gstat_rx_callback(sspi_fl_rx.tmc_gstat);
					}
					default:
					{
						break;
					}
				}
				break;
			}
			case SSPI_FL_DRV_STATUS1:
			{
				switch (sspi.drvDevice)
				{
					case SSPI_DEVICE_TMC6200:
					{
						mctrl_diag_tmc6200_ioin_rx_callback(sspi_fl_rx.tmc_ioin);
					}
					default:
					{
						break;
					}
				}
				break;
			}
			case SSPI_FL_DRV_STATUS2:
			{
				switch (sspi.drvDevice)
				{
					case SSPI_DEVICE_TMC6200:
					{
						mctrl_diag_tmc6200_gconf_rx_callback(sspi_fl_rx.tmc_gconf);
					}
					default:
					{
						break;
					}
				}
				break;
			}
			default:
			{
				break;
			}
		}
	}
}

void sspi_fastloop(void)
{
	static uint16_t as_angle_addr[1] = {_PAR(0x3FFE | AD5047D_READ_BIT)};
	static uint16_t as_diag_addr[1] = {_PAR(0x3FFC | AD5047D_READ_BIT)};
	static uint16_t as_err_addr[1] = {_PAR(0x0001 | AD5047D_READ_BIT)};
	// static uint16_t as_nop_addr[1] = {_PAR(0x0000)};
	static uint16_t drv_fault_addr[1] = {0x00 << 11};
	static uint16_t drv_vgs_addr[1] = {0x01 << 11};
	static uint8_t tmc_gstat_addr[5] = {TMC6200_REG_GSTAT};
	static uint8_t tmc_ioin_addr[5] = {TMC6200_REG_IOIN};
	static uint8_t tmc_gconf_addr[5] = {TMC6200_REG_GCONF};

	static spi_block_t as_angle_an = {
		.deviceId = SSPI_DEVICE_AS5047D,
		.src = as_angle_addr,
		.dst = sspi_fl_rx.as_angle,
		.numWords = 1,
		.callback = _blockCallback,
	};
	static spi_block_t as_angle_s0 = {
		.deviceId = SSPI_DEVICE_AS5047D,
		.src = as_angle_addr,
		.dst = sspi_fl_rx.as_diag,
		.numWords = 1,
		.callback = _blockCallback,
	};
	static spi_block_t as_angle_s1 = {
		.deviceId = SSPI_DEVICE_AS5047D,
		.src = as_angle_addr,
		.dst = sspi_fl_rx.as_err,
		.numWords = 1,
		.callback = _blockCallback,
	};

	static spi_block_t as_diag = {
		.deviceId = SSPI_DEVICE_AS5047D,
		.src = as_diag_addr,
		.dst = sspi_fl_rx.as_angle,
		.numWords = 1,
		.callback = _blockCallback,
	};
	static spi_block_t as_err = {
		.deviceId = SSPI_DEVICE_AS5047D,
		.src = as_err_addr,
		.dst = sspi_fl_rx.as_angle,
		.numWords = 1,
		.callback = _blockCallback,
	};

	static spi_block_t tmc_gstat = {
		.deviceId = SSPI_DEVICE_TMC6200,
		.src = tmc_gstat_addr,
		.dst = sspi_fl_rx.tmc_gstat,
		.numWords = 5,
		.callback = _blockCallback,
	};
	static spi_block_t tmc_ioin = {
		.deviceId = SSPI_DEVICE_TMC6200,
		.src = tmc_ioin_addr,
		.dst = sspi_fl_rx.tmc_ioin,
		.numWords = 5,
		.callback = _blockCallback,
	};
	static spi_block_t tmc_gconf = {
		.deviceId = SSPI_DEVICE_TMC6200,
		.src = tmc_gconf_addr,
		.dst = sspi_fl_rx.tmc_gconf,
		.numWords = 5,
		.callback = _blockCallback,
	};

	static spi_block_t drv_fault = {
		.deviceId = SSPI_DEVICE_DRV83XX,
		.src = drv_fault_addr,
		.dst = sspi_fl_rx.drv_fault,
		.numWords = 1,
		.callback = _blockCallback,
	};

	static spi_block_t drv_vgs = {
		.deviceId = SSPI_DEVICE_DRV83XX,
		.src = drv_vgs_addr,
		.dst = sspi_fl_rx.drv_vgs,
		.numWords = 1,
		.callback = _blockCallback,
	};

	// return early if not enabled
	if (!sspi.fastloopEnabled)
		return;

	// TODO: process values

	// increment counter
	sspi_flCycleCounter = (sspi_flCycleCounter+1) % (sizeof(_flCycle)/sizeof(*_flCycle));
	sspi_fastloop_cycle_name_t cycle = _flCycle[sspi_flCycleCounter];

	sspi_fl_rx.blockTime = utime_now();

	switch (cycle)
	{
		case SSPI_FL_HALL_ANGLEU_AFTER_AN:
		{
			sspi_fl_rx.as_angle[0] = 0xFFFF;
			sspi_asyncTransmitBlock(&as_angle_an);
			break;
		}
		case SSPI_FL_HALL_ANGLEU_AFTER_S0:
		{
			sspi_fl_rx.as_diag[0] = 0xFFFF;
			sspi_asyncTransmitBlock(&as_angle_s0);
			break;
		}
		case SSPI_FL_HALL_ANGLEU_AFTER_S1:
		{
			sspi_fl_rx.as_err[0] = 0xFFFF;
			sspi_asyncTransmitBlock(&as_angle_s1);
			break;
		}
		case SSPI_FL_HALL_STATUS0:
		{
			sspi_fl_rx.as_angle[0] = 0xFFFF;
			sspi_asyncTransmitBlock(&as_diag);
			break;
		}
		case SSPI_FL_HALL_STATUS1:
		{
			sspi_fl_rx.as_angle[0] = 0xFFFF;
			sspi_asyncTransmitBlock(&as_err);
			break;
		}
		case SSPI_FL_DRV_STATUS0:
		{
			switch (sspi.drvDevice) 
			{
				case SSPI_DEVICE_TMC6200:
				{
					memset(sspi_fl_rx.tmc_gstat, 0xFF, sizeof(sspi_fl_rx.tmc_gstat));
					sspi_asyncTransmitBlock(&tmc_gstat);
					break;
				}
				case SSPI_DEVICE_DRV83XX:
				{
					sspi_fl_rx.drv_fault[0] = 0xFFFF;
					sspi_asyncTransmitBlock(&drv_fault);
					break;
				}
				default:
				{
					// no driver, NOP
					break;
				}
			}
			break;
		}
		case SSPI_FL_DRV_STATUS1:
		{
			switch (sspi.drvDevice) 
			{
				case SSPI_DEVICE_TMC6200:
				{
					memset(sspi_fl_rx.tmc_ioin, 0xFF, sizeof(sspi_fl_rx.tmc_ioin));
					sspi_asyncTransmitBlock(&tmc_ioin);
					break;
				}
				case SSPI_DEVICE_DRV83XX:
				{
					sspi_fl_rx.drv_vgs[0] = 0xFFFF;
					sspi_asyncTransmitBlock(&drv_vgs);
					break;
				}
				default:
				{
					// no driver, NOP
					break;
				}
			}
			break;
		}
		case SSPI_FL_DRV_STATUS2:
		{
			switch (sspi.drvDevice) 
			{
				case SSPI_DEVICE_TMC6200:
				{
					memset(sspi_fl_rx.tmc_gconf, 0xFF, sizeof(sspi_fl_rx.tmc_gconf));
					sspi_asyncTransmitBlock(&tmc_gconf);
					break;
				}
				default:
				{
					// no driver, NOP
					break;
				}
			}
			break;
		}
	}

}

