#ifndef SSF_SPI_H
#define SSF_SPI_H

#include <stdint.h>

// SPI chip select mux pins
// DRV8323 is on PB11 on V0.1
// DRV8323 is on PC6 on V0.2
// AD5047 is on PB10

#if SSF_HARDWARE_VERSION == 0x000100
	#define PIN_DRVSEL	GPIOB, GPIO_PIN_11
	#define PIN_ASEL	GPIOB, GPIO_PIN_10
#else
	#define PIN_DRVSEL	GPIOC, GPIO_PIN_6
	#define PIN_ASEL 	GPIOB, GPIO_PIN_10
#endif

#define PIN_SPI_NSS		GPIOB, GPIO_PIN_12

#define PIN_DRVEN		GPIOA, GPIO_PIN_7

typedef struct {
	uint16_t NOP;
	uint16_t ERRFL;
	uint16_t DIAAGC;
	uint16_t ANGLEUNC;
} mspi_as5047_state_t;


typedef struct {
	// uint16_t NOP;
	uint16_t FAULT_STATUS;
	uint16_t VGS_STATUS;
	uint16_t DRV_CTRL;
	uint16_t DRV_HS;
	uint16_t DRV_LS;
	uint16_t OCP_CTRL;
	uint16_t CSA_CTRL;
} mspi_drv_state_t;


mspi_as5047_state_t ssf_readHallSensor(void);
mspi_drv_state_t ssf_readMotorDriver(void);





#endif // SSF_SPI_H
