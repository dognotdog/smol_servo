#ifndef SSF_SPI_PRIVATE_H
#define SSF_SPI_PRIVATE_H

#include "ssf_spi.h"

#include <stdatomic.h>

#define _XORS(x,s) ((x) ^ ((x) >> s))

#define  _PAR(x) (((_XORS(_XORS(_XORS(_XORS(((x) & 0x7FFF), 8), 4), 2), 1) & 1) << 15) | ((x) & 0x7FFF))




#define TMC6200_REG_GCONF 		0x00
#define TMC6200_REG_GSTAT 		0x01
#define TMC6200_REG_IOIN 		0x04
#define TMC6200_REG_SHORT_CONF 	0x09
#define TMC6200_REG_DRV_CONF 	0x0A

#define AD5047D_READ_BIT	 	0x4000

struct spi_block_s;
typedef struct spi_block_s spi_block_t;

typedef void (*spi_blockCallback_t)(const bool transferOk);


struct spi_block_s {
	const void* 	src;
	volatile void* 	dst;
	size_t			numWords;

	sspi_deviceId_t	deviceId;

	bool transferComplete;

	spi_blockCallback_t callback;
};

typedef struct {
	sspi_deviceId_t currentDevice;

	spi_block_t* currentBlock;
	atomic_flag blockBusy;

	volatile bool fastloopEnabled;

	sspi_deviceId_t encDevice;
	sspi_deviceId_t drvDevice;


} sspi_t;

extern sspi_t sspi;

// extern void sspi_initAs5047d(void);
// extern void sspi_initDrv83xx(void);
// extern void sspi_initTmc6200(void);

extern uint16_t sspi_drv_writeDrvMotorDriverReg(size_t addr, uint16_t data);
extern sspi_drv_state_t sspi_drv_setMotorDriver3PwmMode(void);
extern sspi_tmc_state_t sspi_tmc_setMotorDriver3PwmMode(void);


int sspi_transmitBlock(spi_block_t* block);

void as5047d_readRegisters(uint16_t* registers, uint16_t* results, size_t numregs, spi_blockCallback_t callback);


#endif // SSF_SPI_PRIVATE_H
