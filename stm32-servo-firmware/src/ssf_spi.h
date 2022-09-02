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
#elif SSF_HARDWARE_VERSION == 0x000600
	#define PIN_DRVSEL	GPIOC, GPIO_PIN_6
	#define PIN_ASEL 	GPIOB, GPIO_PIN_10
	#define PIN_SPI_NSS	GPIOB, GPIO_PIN_12
#elif SSF_HARDWARE_VERSION == 0x000700
	#error define pins
	#define PIN_DRVSEL	GPIOC, GPIO_PIN_6
	#define PIN_ASEL 	GPIOB, GPIO_PIN_10
	#define PIN_SPI_NSS	GPIOB, GPIO_PIN_12
#endif

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

typedef struct {
	union  {
		uint32_t reg;
		struct {
			uint32_t disable:1;				// 0x00000001
			uint32_t singleline:1;			// 0x00000002
			uint32_t faultdirect:1;			// 0x00000004
			uint32_t _reserved3:1;			// 0x00000008
			uint32_t amplification_x5:1;	// 0x00000010
			uint32_t amplification_x2:1;	// 0x00000020
			uint32_t amplifier_off:1;		// 0x00000040
			uint32_t test_mode:1;			// 0x00000080
			uint32_t _reserved8_31:24;		// 0xFFFFFF00
		};
	} GCONF;
	union  {
		uint32_t reg;
		struct {
			uint32_t reset:1;		// 0x00000001
			uint32_t drv_optw:1;	// 0x00000002
			uint32_t drw_ot:1;		// 0x00000004
			uint32_t uv_cp:1;		// 0x00000008
			uint32_t shortdet_u:1;	// 0x00000010
			uint32_t s2gu:1;		// 0x00000020
			uint32_t s2vsu:1;		// 0x00000040
			uint32_t _reserved7:1;	// 0x00000080
			uint32_t shortdet_v:1;	// 0x00000100
			uint32_t s2gv:1;		// 0x00000200
			uint32_t s2vsv:1;		// 0x00000400
			uint32_t _reserved11:1;	// 0x00000800
			uint32_t shortdet_w:1;	// 0x00001000
			uint32_t s2gw:1;		// 0x00002000
			uint32_t s2vsw:1;		// 0x00004000
			uint32_t _res15_31:17;	// 0xFFFF8000
		};
	} GSTAT;
	union  {
		uint32_t reg;
		struct {
			uint32_t UL:1;			// 0x00000001
			uint32_t UH:1;			// 0x00000002
			uint32_t VL:1;			// 0x00000004
			uint32_t VH:1;			// 0x00000008
			uint32_t WL:1;			// 0x00000010
			uint32_t WH:1;			// 0x00000020
			uint32_t DRV_EN:1;		// 0x00000040
			uint32_t _reserved7:1;	// 0x00000080
			uint32_t OTPW:1;		// 0x00000100
			uint32_t OT136:1;		// 0x00000200
			uint32_t OT143:1;		// 0x00000400
			uint32_t OT150:1;		// 0x00000800
			uint32_t _res12_23:12;	// 0x00FFF000
			uint32_t VERSION:8;		// 0xFF000000
		};
	} IOIN;

	union  {
		uint32_t reg;
		struct {
			uint32_t S2VS_LEVEL:4;	// 0x0000000F
			uint32_t _res4_7:4;		// 0x000000F0
			uint32_t S2G_LEVEL:4;	// 0x00000F00
			uint32_t _res12_15:4;	// 0x0000F000
			uint32_t SHORTFILTER:2;	// 0x00030000
			uint32_t _res18_19:2;	// 0x000C0000
			uint32_t shortdelay:1;	// 0x00100000
			uint32_t _res21_23:3;	// 0x00E00000
			uint32_t RETRY:2;		// 0x03000000
			uint32_t _res26_27:2;	// 0x0C000000
			uint32_t protect_parallel:1;	// 0x10000000
			uint32_t disable_S2G:1;	// 0x20000000
			uint32_t disable_S2VS:1;// 0x40000000
			uint32_t _res31:1;		// 0x80000000
		};
	} SHORT_CONF;

	union  {
		uint32_t reg;
		struct {
			uint32_t BBMCLKS:5;		// 0x0000001F
			uint32_t _res5_15:11;	// 0x0000FFE0
			uint32_t OTSELECT:2;	// 0x00030000
			uint32_t DRVSTRENGTH:2;	// 0x000C0000
			uint32_t _res20_31:12;	// 0xFFF00000
		};
	} DRV_CONF;

} sspi_tmc_state_t;


extern int ssf_asyncReadHallSensor(void);
extern void ssf_asyncReadHallSensorCallback(sspi_as5047_state_t sensorState, bool transferOk);
extern sspi_as5047_state_t ssf_readHallSensor(void);

bool ssf_checkSpiEncoderReadOk(sspi_as5047_state_t state, bool* formatError, bool* valueError);

extern void ssf_printMotorDriverFaults(sspi_drv_state_t state);
extern void ssf_dbgPrintEncoderStatus(sspi_as5047_state_t state);

extern sspi_drv_state_t ssf_enterMotorDriverCalibrationMode(void);
extern sspi_drv_state_t ssf_exitMotorDriverCalibrationMode(void);
extern sspi_drv_state_t sspi_drv_readMotorDriver(void);
extern void ssf_setMotorDriver3PwmMode(void);

extern void spi_printTransferStatus(void);

extern void ssf_spiInit(void);
extern void sspi_enableFastloopReads(bool enable);

void sspi_fastloop(void);

// autodetect attached parts
extern bool sspi_detectAs5047d(void);
extern bool sspi_detectDrv83xx(void);
extern bool sspi_detectTmc6200(void);

extern sspi_deviceId_t sspi_drvType(void);

#endif // SSF_SPI_H
