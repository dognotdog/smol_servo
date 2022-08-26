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
	SSPI_DEVICE_HALL,
	SSPI_DEVICE_DRV83XX,
	SSPI_DEVICE_TMC6200,
} sspi_deviceId_t;

struct spi_transfer_s;
typedef struct spi_transfer_s spi_transfer_t;

typedef void (*spi_transferCallback_t)(const spi_transfer_t* const transfer, const bool transferOk);

struct spi_transfer_s {
	const void* src;
	void* 		dst;
	size_t		numWords;
	size_t		len;

	sspi_deviceId_t	deviceId;

	spi_transferCallback_t callback;
};

extern void sspi_syncTransfer(spi_transfer_t transfer);
extern void sspi_initAs5047d(void);
extern void sspi_initDrv83xx(void);
extern void sspi_initTmc6200(void);

extern bool sspi_detectAs5047d(void);
extern bool sspi_detectDrv83xx(void);
extern bool sspi_detectTmc6200(void);

extern uint16_t sspi_drv_writeDrvMotorDriverReg(size_t addr, uint16_t data);
extern sspi_drv_state_t sspi_drv_setMotorDriver3PwmMode(void);
extern sspi_tmc_state_t sspi_tmc_setMotorDriver3PwmMode(void);

#endif // SSF_SPI_PRIVATE_H
