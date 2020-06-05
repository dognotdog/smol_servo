#ifndef SSF_SPI_H
#define SSF_SPI_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

// SPI chip select mux pins
// DRV8323 is on PB11 on V0.1
// DRV8323 is on PC6 on V0.2
// AD5047 is on PB10

#if SSF_HARDWARE_VERSION == 0x000100
	#define PIN_DRVSEL	GPIOB, GPIO_PIN_11
	#define PIN_ASEL	GPIOB, GPIO_PIN_10
	#define PIN_SPI_NSS	GPIOB, GPIO_PIN_12
#elif SSF_HARDWARE_VERSION == 0x000200
	#define PIN_DRVSEL	GPIOB, GPIO_PIN_12
	#define PIN_ASEL 	GPIOB, GPIO_PIN_10
#elif SSF_HARDWARE_VERSION == 0x000300
	#define PIN_DRVSEL	GPIOC, GPIO_PIN_6
	#define PIN_ASEL 	GPIOB, GPIO_PIN_10
	#define PIN_SPI_NSS	GPIOB, GPIO_PIN_12
#endif


typedef struct {
	uint16_t NOP;
	uint16_t ERRFL;
	uint16_t DIAAGC;
	uint16_t ANGLEUNC;
	uint32_t start_us, end_us;
} sspi_as5047_state_t;


typedef struct {
	// uint16_t NOP;
	union  {
		uint16_t reg;
		struct {
			uint16_t VDS_LC:1;	// 0x0001
			uint16_t VDS_HC:1;	// 0x0002
			uint16_t VDS_LB:1;	// 0x0004
			uint16_t VDS_HB:1;	// 0x0008
			uint16_t VDS_LA:1;	// 0x0010
			uint16_t VDS_HA:1;	// 0x0020
			uint16_t OTSD:1;	// 0x0040
			uint16_t UVLO:1;	// 0x0080
			uint16_t GDF:1;		// 0x0100
			uint16_t VDS_OCP:1;	// 0x0200
			uint16_t FAULT:1;	// 0x0400
			uint16_t reserved_12_15:5;
		};
	} FAULT_STATUS;
	union { 
		uint16_t reg; 
		struct {
			uint16_t VGS_LC:1;	// 0x0001
			uint16_t VGS_HC:1;	// 0x0002
			uint16_t VGS_LB:1;	// 0x0004
			uint16_t VGS_HB:1;	// 0x0008
			uint16_t VGS_LA:1;	// 0x0010
			uint16_t VGS_HA:1;	// 0x0020
			uint16_t CPUV:1;	// 0x0040
			uint16_t OTW:1;		// 0x0080
			uint16_t SC_OC:1;	// 0x0100
			uint16_t SB_OC:1;	// 0x0200
			uint16_t SA_OC:1;	// 0x0400
		};
	} VGS_STATUS;
	union { uint16_t reg; } DRV_CTRL;
	union { uint16_t reg; } DRV_HS;
	union { uint16_t reg; } DRV_LS;
	union { uint16_t reg; } OCP_CTRL;
	union { uint16_t reg; } CSA_CTRL;
} sspi_drv_state_t;


extern int ssf_asyncReadHallSensor(void);
extern void ssf_asyncReadHallSensorCallback(sspi_as5047_state_t sensorState, bool transferOk);
extern sspi_as5047_state_t ssf_readHallSensor(void);
extern sspi_drv_state_t ssf_readMotorDriver();

bool ssf_checkSpiEncoderReadOk(sspi_as5047_state_t state, bool* formatError, bool* valueError);

extern void ssf_printMotorDriverFaults(sspi_drv_state_t state);
extern void ssf_dbgPrintEncoderStatus(sspi_as5047_state_t state);

extern sspi_drv_state_t ssf_enterMotorDriverCalibrationMode(void);
extern sspi_drv_state_t ssf_exitMotorDriverCalibrationMode(void);
extern sspi_drv_state_t ssf_setMotorDriver3PwmMode(void);

extern void spi_printTransferStatus(void);

extern void ssf_spiInit(void);




#endif // SSF_SPI_H
