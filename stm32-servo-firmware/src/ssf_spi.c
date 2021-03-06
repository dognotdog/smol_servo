#include "debug.h"

#include "main.h"
#include "ssf_main.h"
#include "ssf_spi.h"
#include "utime.h"

#include <string.h>
#include <stdatomic.h>

/*
AS5047 does 16bit SPI transfers
BIT15 is an even parity on commands as well as data, and transfers are big endian.

Propagation delay of sensor is spec'd at 100us +-10us from sensing to read via SPI

At 8Mhz, SPI read of 16bits takes 2us nominally, so ~10us for the 4x16bit words seems a reasonable assumption

The AS5047 clocks out a response at the NEXT transfer.
The DRV8323 responds within the SAME transfer, as it the first few bits of a command are the address plus 11 bytes data, and the response is always only 11 bytes.

on V0.1 hardware, the DRV8323 seems to have difficulty at higher clock rates, so the 128x divider was chosen for ~1.5MHz. The AS5047 has no problem with up to 5MHz

Async Transfer State Changes
	IDLE -> SYNC -> IDLE
		complete sync transfer
	IDLE -> IN_PROGRESS
		start of async transfer, currentTransfer set
	IN_PROGRESS | WORD_COMPLETE -> IN_DMA
		SPI bus transfer started
	IN_DMA -> WORD_COMPLETE
		transfer completed
	WORD_COMPLETE -> WAIT_LATCH
		NSS delay triggered

	things that happen are 
	 - the initial XFERCALL, which should trigger DATAIRQ
	   - XFERCALL may finish before DATAIRQ happens
	   - DATAIRQ then triggers NSSIRQ
	     - DATAIRQ may finish before NSSIRQ happens

	transfer of a new word may be started when:
		DATAIRQ and NSSIRQ and XFERCALL have all finished


	in progress     |---------------------------------------------------------------|
	  -> xfercall      |----------------------------------: - - - - - - - - - - - - |
	     -> irq          | - - - - - - - - - - - - - - - - - - |
	        -> nss

	
*/

extern SPI_HandleTypeDef hspi2;

#define _XORS(x,s) ((x) ^ ((x) >> s))

#define  _PAR(x) (((_XORS(_XORS(_XORS(_XORS((x & 0x7FFF), 8), 4), 2), 1) & 1) << 15) | (x & 0x7FFF))



typedef enum {
	SSPI_DEVICE_HALL,
	SSPI_DEVICE_DRV
} sspi_deviceId_t;

typedef enum {
	SPI_XFER_IDLE,
	SPI_XFER_SYNC,
	SPI_XFER_IN_PROGRESS,
	SPI_XFER_WAIT_XFERCALL_DATAIRQ,
	SPI_XFER_WAIT_DATAIRQ,
	// WAIT_DATAIRQ_xxx -> WORD_COMPLETE_WAIT_xxx_NSSIRQ
	SPI_XFER_WORD_COMPLETE_WAIT_XFERCALL_NSSIRQ,
	SPI_XFER_WORD_COMPLETE_WAIT_XFERCALL, // transition from here can call startWord
	SPI_XFER_WORD_COMPLETE_WAIT_NSSIRQ, // transition from here can call startWord
	SPI_XFER_WORD_COMPLETE,
	// WORD_COMPLETE_xxx -> XFER_COMPLETE_WAIT_xxx_CB
	SPI_XFER_COMPLETE_WAIT_XFERCALL_NSSIRQ_CB,
	SPI_XFER_COMPLETE_WAIT_NSSIRQ_CB,
	SPI_XFER_COMPLETE_WAIT_CB,
	SPI_XFER_COMPLETE_WAIT_NSSIRQ,
	SPI_XFER_COMPLETE_WAIT_XFERCALL_CB,
	SPI_XFER_COMPLETE_WAIT_XFERCALL_NSSIRQ,
	SPI_XFER_COMPLETE_WAIT_XFERCALL,
	SPI_XFER_ERROR,
} spi_transferStatus_t;


typedef enum {
	SPI_XFER_EVENT_NONE,
	SPI_XFER_EVENT_START,
} spi_transferEvent_t;

struct spi_transfer_s;
typedef struct spi_transfer_s spi_transfer_t;

typedef void (*spi_transferCallback_t)(const spi_transfer_t* const transfer, const bool transferOk);

struct spi_transfer_s {
	const uint16_t* src;
	uint16_t* 		dst;
	size_t			len;

	sspi_deviceId_t	deviceId;

	spi_transferCallback_t callback;
};

typedef struct {
	spi_transfer_t 	currentTransfer;
	size_t 			wordsTransferred;
} spi_t;

static spi_t spi = {
	.wordsTransferred = 0,
};

static volatile spi_transferStatus_t 	_transferStatus = SPI_XFER_IDLE;

// static bool _doWork(spi_transferEvent_t event)
// {
// 	bool done = true;

// 	switch (_transferStatus)
// 	{
// 		case SPI_XFER_???:
// 		{
// 			HAL_GPIO_WritePin(_deviceSelectPinMap[transfer.deviceId].gpio, _deviceSelectPinMap[transfer.deviceId].pin, GPIO_PIN_RESET);
// 			// _delay(25);

// 			// size is 1, as it is the number of transfers, not bytes, and SPI is configured in 16bit mode
// 			int status = HAL_SPI_TransmitReceive_DMA(&hspi2, (void*)transfer.src, (void*)transfer.dst, 1);
// 			switch(status)
// 			{
// 				case HAL_OK:
// 				{
// 					atomic_store(&_transferStatus, SPI_XFER_IN_DMA);
// 					break;
// 				}
// 				default:
// 				{
// 					err_println("HAL_SPI_TransmitReceive_DMA returned error %d", status);
// 					atomic_store(&_transferStatus, SPI_XFER_ERROR);
// 					break;
// 				}
// 			}
// 			break;
// 		}
// 	}

// 	// when not done, _doWork must be called again to advance the state machine
// 	return done;
// }





static struct {
	GPIO_TypeDef* 	gpio;
	uint16_t 		pin;
} _deviceSelectPinMap[] = {
	[SSPI_DEVICE_HALL] = {PIN_ASEL},
	[SSPI_DEVICE_DRV] = {PIN_DRVSEL},
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

static void _spi_syncTransferWord(spi_transfer_t transfer, size_t wordIndex)
{
	HAL_GPIO_WritePin(_deviceSelectPinMap[transfer.deviceId].gpio, _deviceSelectPinMap[transfer.deviceId].pin, GPIO_PIN_RESET);
	// _delay(25);

	// size is 1, as it is the number of transfers, not bytes, and SPI is configured in 16bit mode
	HAL_SPI_TransmitReceive(&hspi2, (void*)(transfer.src + wordIndex), (void*)(transfer.dst + wordIndex), 1, 1000);

	HAL_GPIO_WritePin(_deviceSelectPinMap[transfer.deviceId].gpio, _deviceSelectPinMap[transfer.deviceId].pin, GPIO_PIN_SET);
	_delay(50);

	return;
}

static void _recoverSpiError(void)
{
	// error recovery
	spi_transferStatus_t expected = SPI_XFER_ERROR;
	while (!atomic_compare_exchange_weak(&_transferStatus, &expected, SPI_XFER_IDLE)) 
	{ 
		if (expected != SPI_XFER_ERROR)
			break;
		expected = SPI_XFER_ERROR;
	};

}

static void _spiSyncTransfer(spi_transfer_t transfer)
{
	_recoverSpiError();

	// wait until we can start a sync transfer
	spi_transferStatus_t expected = SPI_XFER_IDLE;
	while (!atomic_compare_exchange_weak(&_transferStatus, &expected, SPI_XFER_SYNC)) 
	{ expected = SPI_XFER_IDLE; };

	spi.currentTransfer = transfer;
	spi.wordsTransferred = 0;

	while (spi.wordsTransferred < spi.currentTransfer.len)
	{
		_spi_syncTransferWord(spi.currentTransfer, spi.wordsTransferred);
		++spi.wordsTransferred;
	};

	atomic_store(&_transferStatus, SPI_XFER_IDLE);	
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


static void _spi_asyncStartTransferWord(spi_transfer_t transfer, size_t wordIndex)
{
	switch (_transferStatus)
	{
		case SPI_XFER_IN_PROGRESS:
		case SPI_XFER_WORD_COMPLETE:
		{
			HAL_GPIO_WritePin(_deviceSelectPinMap[transfer.deviceId].gpio, _deviceSelectPinMap[transfer.deviceId].pin, GPIO_PIN_RESET);
			// _delay(25);

			// do this before the call, as the DMA might be faster than returning from the call and could interrupt it
			atomic_store(&_transferStatus, SPI_XFER_WAIT_XFERCALL_DATAIRQ);

			// size is 1, as it is the number of transfers, not bytes, and SPI is configured in 16bit mode
			int status = HAL_SPI_TransmitReceive_DMA(&hspi2, (void*)(transfer.src+wordIndex), (void*)(transfer.dst+wordIndex), 1);
			switch(status)
			{
				case HAL_OK:
				{

					while (1)
					{
						spi_transferStatus_t expected = _transferStatus;
						spi_transferStatus_t target = SPI_XFER_ERROR;
						switch (expected)
						{
							case SPI_XFER_WAIT_XFERCALL_DATAIRQ:
								target = SPI_XFER_WAIT_DATAIRQ;
								break;

							case SPI_XFER_WORD_COMPLETE_WAIT_XFERCALL_NSSIRQ:
								target = SPI_XFER_WORD_COMPLETE_WAIT_NSSIRQ;
								break;
							case SPI_XFER_WORD_COMPLETE_WAIT_XFERCALL:
								target = SPI_XFER_WORD_COMPLETE;
								break;

							case SPI_XFER_COMPLETE_WAIT_XFERCALL_NSSIRQ_CB:
								target = SPI_XFER_COMPLETE_WAIT_NSSIRQ_CB;
								break;
							case SPI_XFER_COMPLETE_WAIT_XFERCALL_CB:
								target = SPI_XFER_COMPLETE_WAIT_CB;
								break;
							case SPI_XFER_COMPLETE_WAIT_XFERCALL_NSSIRQ:
								target = SPI_XFER_COMPLETE_WAIT_NSSIRQ;
								break;
							case SPI_XFER_COMPLETE_WAIT_XFERCALL:
								target = SPI_XFER_IDLE;
								break;
							default:
							{
								err_println("_spi_asyncStartTransferWord() post-call in invalid state %d", _transferStatus);
								target = SPI_XFER_ERROR;
								break;
							}
						}
						if (atomic_compare_exchange_strong(&_transferStatus, &expected, target))
						{
							if (target == SPI_XFER_WORD_COMPLETE)
							{
								if (spi.wordsTransferred < spi.currentTransfer.len)
								{
									_spi_asyncStartTransferWord(spi.currentTransfer, spi.wordsTransferred);									
								}
								else
								{
									while (1)
									{
										spi_transferStatus_t expected = SPI_XFER_WORD_COMPLETE;
										if (atomic_compare_exchange_strong(&_transferStatus, &expected, SPI_XFER_COMPLETE_WAIT_CB))
										{
											break;
										}
									}

									if (spi.currentTransfer.callback)
										spi.currentTransfer.callback(&spi.currentTransfer, true);

									while (1)
									{
										spi_transferStatus_t expected = SPI_XFER_COMPLETE_WAIT_CB;
										if (atomic_compare_exchange_strong(&_transferStatus, &expected, SPI_XFER_IDLE))
										{
											break;
										}
									}
								}
							}
							break;
						}

					}
					break;
				}
				default:
				{
					HAL_GPIO_WritePin(_deviceSelectPinMap[transfer.deviceId].gpio, _deviceSelectPinMap[transfer.deviceId].pin, GPIO_PIN_SET);
					err_println("HAL_SPI_TransmitReceive_DMA returned error %d", status);
					atomic_store(&_transferStatus, SPI_XFER_ERROR);
					break;
				}
			}
			break;
		}
		default:
		{
			err_println("_spi_asyncStartTransferWord() in invalid state %d", _transferStatus);
			atomic_store(&_transferStatus, SPI_XFER_ERROR);
			break;
		}
	}

	return;
}

static void _nssLatchDelayCallback(void)
{
	// dbg_println("_nssLatchDelayCallback");
	switch (_transferStatus)
	{
		case SPI_XFER_WORD_COMPLETE_WAIT_XFERCALL_NSSIRQ:
		case SPI_XFER_WORD_COMPLETE_WAIT_NSSIRQ:
		case SPI_XFER_COMPLETE_WAIT_XFERCALL_NSSIRQ_CB:
		case SPI_XFER_COMPLETE_WAIT_XFERCALL_NSSIRQ:
		case SPI_XFER_COMPLETE_WAIT_NSSIRQ_CB:
		case SPI_XFER_COMPLETE_WAIT_NSSIRQ:
		{
			while (1)
			{
				spi_transferStatus_t expected = SPI_XFER_WORD_COMPLETE_WAIT_XFERCALL_NSSIRQ;
				if (atomic_compare_exchange_strong(&_transferStatus, &expected, SPI_XFER_WORD_COMPLETE_WAIT_XFERCALL))
				{
					break;
				}
				expected = SPI_XFER_WORD_COMPLETE_WAIT_NSSIRQ;
				if (atomic_compare_exchange_strong(&_transferStatus, &expected, SPI_XFER_WORD_COMPLETE))
				{
					// if all word transfer stuff has been completed, and we have one more word to go, transfer it!
					if (spi.wordsTransferred < spi.currentTransfer.len)
						_spi_asyncStartTransferWord(spi.currentTransfer, spi.wordsTransferred);
					break;
				}
				expected = SPI_XFER_COMPLETE_WAIT_XFERCALL_NSSIRQ_CB;
				if (atomic_compare_exchange_strong(&_transferStatus, &expected, SPI_XFER_COMPLETE_WAIT_XFERCALL_CB))
				{
					break;
				}
				expected = SPI_XFER_COMPLETE_WAIT_XFERCALL_NSSIRQ;
				if (atomic_compare_exchange_strong(&_transferStatus, &expected, SPI_XFER_COMPLETE_WAIT_XFERCALL))
				{
					break;
				}
				expected = SPI_XFER_COMPLETE_WAIT_NSSIRQ_CB;
				if (atomic_compare_exchange_strong(&_transferStatus, &expected, SPI_XFER_COMPLETE_WAIT_CB))
				{
					break;
				}
				expected = SPI_XFER_COMPLETE_WAIT_NSSIRQ;
				if (atomic_compare_exchange_strong(&_transferStatus, &expected, SPI_XFER_IDLE))
				{
					break;
				}
			}
			break;
		}
		default:
		{
			err_println("_nssLatchDelayCallback() in invalid state %d", _transferStatus);
			atomic_store(&_transferStatus, SPI_XFER_ERROR);
			break;
		}
	}
}


static void _spi_finishTransferWord(void)
{
	// dbg_println("_spi_finishTransferWord() word[%u] 0x%04X", spi.wordsTransferred, spi.currentTransfer.dst[spi.wordsTransferred]);

	switch (_transferStatus)
	{
		case SPI_XFER_WAIT_XFERCALL_DATAIRQ:
		case SPI_XFER_WAIT_DATAIRQ:
		{
			// dbg_println("_spi_finishTransferWord(%d)", _transferStatus);
			HAL_GPIO_WritePin(_deviceSelectPinMap[spi.currentTransfer.deviceId].gpio, _deviceSelectPinMap[spi.currentTransfer.deviceId].pin, GPIO_PIN_SET);
			// _delay(50);

			++spi.wordsTransferred;

			while (1)
			{
				spi_transferStatus_t expected = SPI_XFER_WAIT_XFERCALL_DATAIRQ;
				if (atomic_compare_exchange_strong(&_transferStatus, &expected, SPI_XFER_WORD_COMPLETE_WAIT_XFERCALL_NSSIRQ))
				{
					break;
				}				
				expected = SPI_XFER_WAIT_DATAIRQ;
				if (atomic_compare_exchange_strong(&_transferStatus, &expected, SPI_XFER_WORD_COMPLETE_WAIT_NSSIRQ))
				{
					break;
				}				
			}


			_delayWithCallback(1.0f, _nssLatchDelayCallback);
			break;
		}
		default:
		{
			err_println("_spi_finishTransferWord() in invalid state %d", _transferStatus);
			atomic_store(&_transferStatus, SPI_XFER_ERROR);
			break;
		}
	}


	return;
}



int spi_startTransfer(spi_transfer_t transfer)
{
	// wait until we can start a sync transfer
	// spi_transferStatus_t expected = SPI_XFER_IDLE;
	// if (!atomic_compare_exchange_strong(&_currentTransfer.status, &expected, SPI_XFER_IN_PROGRESS))
	// 	return -1;

	_recoverSpiError();

	spi.currentTransfer = transfer;
	spi.wordsTransferred = 0;



	_spi_asyncStartTransferWord(transfer, spi.wordsTransferred);

	return 0;
}

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
	// atomic_store(&_transferStatus, SPI_XFER_WORD_COMPLETE);

	_spi_finishTransferWord();

	// word finished, now to determine if we need to complete or start another word


	switch(_transferStatus)
	{
		case SPI_XFER_WORD_COMPLETE_WAIT_XFERCALL_NSSIRQ:
		case SPI_XFER_WORD_COMPLETE_WAIT_XFERCALL:
		case SPI_XFER_WORD_COMPLETE_WAIT_NSSIRQ:
		{
			bool allWordsTransferred = spi.wordsTransferred == spi.currentTransfer.len;
			if (allWordsTransferred)
			{
				// if all words have been transferred, we can call the callback
				while (1)
				{
					spi_transferStatus_t expected = SPI_XFER_WORD_COMPLETE_WAIT_XFERCALL_NSSIRQ;
					if (atomic_compare_exchange_strong(&_transferStatus, &expected, SPI_XFER_COMPLETE_WAIT_XFERCALL_NSSIRQ_CB))
					{
						break;
					}
					expected = SPI_XFER_WORD_COMPLETE_WAIT_XFERCALL;
					if (atomic_compare_exchange_strong(&_transferStatus, &expected, SPI_XFER_COMPLETE_WAIT_XFERCALL_CB))
					{
						break;
					}
					expected = SPI_XFER_WORD_COMPLETE_WAIT_NSSIRQ;
					if (atomic_compare_exchange_strong(&_transferStatus, &expected, SPI_XFER_COMPLETE_WAIT_NSSIRQ_CB))
					{
						break;
					}


				}

				if (spi.currentTransfer.callback)
					spi.currentTransfer.callback(&spi.currentTransfer, true);

				while (1)
				{
					spi_transferStatus_t expected = SPI_XFER_COMPLETE_WAIT_XFERCALL_NSSIRQ_CB;
					if (atomic_compare_exchange_strong(&_transferStatus, &expected, SPI_XFER_COMPLETE_WAIT_XFERCALL_NSSIRQ))
					{
						break;
					}
					expected = SPI_XFER_COMPLETE_WAIT_XFERCALL_CB;
					if (atomic_compare_exchange_strong(&_transferStatus, &expected, SPI_XFER_COMPLETE_WAIT_XFERCALL))
					{
						break;
					}
					expected = SPI_XFER_COMPLETE_WAIT_NSSIRQ_CB;
					if (atomic_compare_exchange_strong(&_transferStatus, &expected, SPI_XFER_COMPLETE_WAIT_NSSIRQ))
					{
						break;
					}
					expected = SPI_XFER_COMPLETE_WAIT_CB;
					if (atomic_compare_exchange_strong(&_transferStatus, &expected, SPI_XFER_IDLE))
					{
						break;
					}
				}
			}
			else
			{
				// still more word transfers to go, nothing to do here, as additional transfer will be triggered after NSSIRQ or XFERCALL complete
			}
			break;
		}
		default:
		{
			err_println("HAL_SPI_TxRxCpltCallback() in invalid state %d", _transferStatus);
			atomic_store(&_transferStatus, SPI_XFER_ERROR);
			break;
		}
	}
}

void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi)
{
	atomic_store(&_transferStatus, SPI_XFER_ERROR);
	err_println("HAL_SPI_ErrorCallback");
	if (spi.currentTransfer.callback)
		spi.currentTransfer.callback(&spi.currentTransfer, false);
	atomic_store(&_transferStatus, SPI_XFER_IDLE);

}

void HAL_SPI_AbortCallback(SPI_HandleTypeDef *hspi)
{
	atomic_store(&_transferStatus, SPI_XFER_ERROR);
	err_println("HAL_SPI_AbortCallback");
	if (spi.currentTransfer.callback)
		spi.currentTransfer.callback(&spi.currentTransfer, false);
	atomic_store(&_transferStatus, SPI_XFER_IDLE);

}

// static inline uint16_t _PAR(uint16_t x)
// {
// 	uint16_t xx = x & 0x7FFF;
// 	uint16_t xxx = xx ^ (xx >> 8);
// 	xxx ^= xxx >> 4;
// 	xxx ^= xxx >> 2;
// 	xxx ^= xxx >> 1;
// 	return (((xxx) & 1) << 15) | xx;
// }

// #define FLIP16(x) (((x & 0xFF00) >> 8) | ((x & 0xFF) << 8))
// #define FLIP16(x) (x)


// uint16_t ssf_spiRW(uint16_t cmd, sspi_deviceId_t device)
// {
// 	// drive everything high to deselect chips
// 	// HAL_GPIO_WritePin(PIN_DRVSEL, GPIO_PIN_SET);
// 	// HAL_GPIO_WritePin(PIN_ASEL, GPIO_PIN_SET);
// 	// _delay(50);

// 	HAL_GPIO_WritePin(_deviceSelectPinMap[device].gpio, _deviceSelectPinMap[device].pin, GPIO_PIN_RESET);
// 	_delay(50);

// 	uint16_t result = 0xFFFF;
// 	// size is 1, as it is the number of transfers, not bytes, and SPI is configured in 16bit mode
// 	HAL_SPI_TransmitReceive(&hspi2, (void*)&cmd, (void*)&result, 1, 1000);

// 	HAL_GPIO_WritePin(_deviceSelectPinMap[device].gpio, _deviceSelectPinMap[device].pin, GPIO_PIN_SET);
// 	_delay(50);

// 	return result;
// }

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
		.len = 4,
		.src = cmd,
		.dst = rx,
		.deviceId = SSPI_DEVICE_HALL,
	};

	uint32_t start_us = utime_now();
	_spiSyncTransfer(transfer);
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

static uint16_t _hallSensorTxBuf[4] = {0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF};
static uint16_t _hallSensorRxBuf[4] = {0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF};

static uint32_t _asyncReadHallSensorTick = 0;

static void _readHallSensorCallback(const spi_transfer_t* const transfer, bool transferOk)
{
	// dbg_println("_readHallSensorCallback()...");

	uint32_t tock = utime_now();

	sspi_as5047_state_t state = {
		.NOP = (_hallSensorRxBuf[0]),
		.ERRFL = (_hallSensorRxBuf[1]),
		.DIAAGC = (_hallSensorRxBuf[2]),
		.ANGLEUNC = (_hallSensorRxBuf[3]),
		.start_us = _asyncReadHallSensorTick,
		.end_us = tock,
	};	

	ssf_asyncReadHallSensorCallback(state, transferOk);
}


int ssf_asyncReadHallSensor(void)
{
	// dbg_println("ssf_asyncReadHallSensor()...");
	uint16_t cmd[4] = {
		(_PAR(0x4001)),
		(_PAR(0x7FFC)),
		(_PAR(0x7FFE)),
		(_PAR(0x4000)) // NOP at the end to get response to prev command
	};



	// wait until we can start a sync transfer before setting buffers
	spi_transferStatus_t expected = SPI_XFER_IDLE;
	if (!atomic_compare_exchange_strong(&_transferStatus, &expected, SPI_XFER_IN_PROGRESS))
	{
		if (expected == SPI_XFER_IN_PROGRESS)
			err_println("ssf_asyncReadHallSensor() could not start, SPI read in progress");
		return -1;
	}

	memcpy((void*)_hallSensorTxBuf, cmd, sizeof(_hallSensorTxBuf));
	memset((void*)_hallSensorRxBuf, 0x0F, sizeof(_hallSensorRxBuf));


	spi_transfer_t transfer = {
		.len = 4,
		.src = _hallSensorTxBuf,
		.dst = _hallSensorRxBuf,
		.deviceId = SSPI_DEVICE_HALL,
		.callback = _readHallSensorCallback,
	};

	_asyncReadHallSensorTick = utime_now();
	int status = spi_startTransfer(transfer);


	return status;
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

	spi_transfer_t transfer = {
		.len = 7,
		.src = cmd,
		.dst = rx,
		.deviceId = SSPI_DEVICE_DRV,
	};

	_spiSyncTransfer(transfer);

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

uint16_t ssf_writeMotorDriverReg(size_t addr, uint16_t data)
{
	uint16_t cmd[1] = {
		(addr << 11) | ( data & 0x03FF),
	};

	uint16_t rx[1] = {0xFFFF};

	spi_transfer_t transfer = {
		.len = 1,
		.src = cmd,
		.dst = rx,
		.deviceId = SSPI_DEVICE_DRV,
	};

	_spiSyncTransfer(transfer);

	// return ssf_spiRW(cmd[0], SSPI_DEVICE_DRV);
	return rx[0];
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

static bool _checkSpiEncoderRegReadOk(uint16_t reg)
{
	return (_PAR(reg) == reg) && ((reg & 0x4000) == 0);
}

bool ssf_checkSpiEncoderReadOk(sspi_as5047_state_t state, bool* formatError, bool* valueError)
{
	bool formatOk = _checkSpiEncoderRegReadOk(state.NOP) 
					&& _checkSpiEncoderRegReadOk(state.DIAAGC) 
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
	dbg_println("AS5047D ANGLEUNC = %5u (%6.3f deg)", state.ANGLEUNC & 0x3FFF, (double)((float)(state.ANGLEUNC & 0x3FFF)/0x4000*360.0f));

}


static void _setPinAsGpioOutput(GPIO_TypeDef *gpio, uint32_t GPIO_Pin)
{
	// this works for single pins only
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

	ssf_setMotorDriver3PwmMode();

	// __HAL_SPI_DISABLE(&hspi2);
}

void spi_printTransferStatus(void)
{
	dbg_println("SPI transferStatus = %u", _transferStatus);
}

