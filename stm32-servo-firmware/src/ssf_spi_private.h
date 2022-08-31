#ifndef SSF_SPI_PRIVATE_H
#define SSF_SPI_PRIVATE_H

#include "ssf_spi.h"

#define _XORS(x,s) ((x) ^ ((x) >> s))

#define  _PAR(x) (((_XORS(_XORS(_XORS(_XORS((x & 0x7FFF), 8), 4), 2), 1) & 1) << 15) | (x & 0x7FFF))



/**
 * HALL is an AS5047D
 * 2-byte transfers
 * 
 * DRV83xx
 * 2-byte transfers
 * 
 * TMC6200
 * 5-byte transfers
 */
typedef enum {
	SSPI_DEVICE_UNKNOWN,
	SSPI_DEVICE_AS5047D,
	SSPI_DEVICE_DRV83XX,
	SSPI_DEVICE_TMC6200,
} sspi_deviceId_t;

struct spi_transfer_s;
struct spi_block_s;
typedef struct spi_transfer_s spi_transfer_t;
typedef struct spi_block_s spi_block_t;

typedef void (*spi_transferCallback_t)(const spi_transfer_t* const transfer, const bool transferOk);

struct spi_transfer_s {
	const void* src;
	void* 		dst;
	size_t		numWords;
	size_t		len;

	sspi_deviceId_t	deviceId;

	spi_transferCallback_t callback;
};

struct spi_block_s {
	const void* 	src;
	volatile void* 	dst;
	size_t			numWords;

	sspi_deviceId_t	deviceId;

	bool transferComplete;
};


typedef struct {
	spi_transfer_t 	currentTransfer;
	size_t 			wordsTransferred;
	sspi_deviceId_t currentDevice;
	// need to deassert/assert NSS every word per transfer
	size_t wordsPerTransfer;

	spi_block_t* currentBlock;
	bool blockIdle;

	sspi_deviceId_t encDevice;
	sspi_deviceId_t drvDevice;
} sspi_t;

extern sspi_t sspi;

extern void sspi_syncTransfer(spi_transfer_t transfer);
extern void sspi_initAs5047d(void);
extern void sspi_initDrv83xx(void);
extern void sspi_initTmc6200(void);

extern uint16_t sspi_drv_writeDrvMotorDriverReg(size_t addr, uint16_t data);
extern sspi_drv_state_t sspi_drv_setMotorDriver3PwmMode(void);
extern sspi_tmc_state_t sspi_tmc_setMotorDriver3PwmMode(void);


int sspi_transmitBlock(spi_block_t* block);

#endif // SSF_SPI_PRIVATE_H
