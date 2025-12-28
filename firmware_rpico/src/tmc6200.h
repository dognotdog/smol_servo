#pragma once

#include "hardware/spi.h"

#include <stdint.h>

#define TMC_REG_GCONF 0x00
// GSTAT:0 indicates state after reset
#define TMC_REG_GSTAT 0x01
// IOIN bits 31:24 contain the version, should be 0x10
#define TMC_REG_IOIN 0x04
#define TMC_REG_OTP_PROG 0x06
#define TMC_REG_OTP_READ 0x07
#define TMC_REG_FACTORY_CONF 0x08
#define TMC_REG_SHORT_CONF 0x09
#define TMC_REG_DRV_CONF 0x0A

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
			uint32_t drv_ot:1;		// 0x00000004
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

} tmc_state_t;

int tmc6200_spi_init(spi_inst_t *spi, int mosi_pin, int nss_pin, int sclk_pin, int miso_pin);
int tmc6200_read_state(tmc_state_t* state);
int tmc6200_enable_driver(bool en);
int tmc6200_enable_3pwm_mode(bool en);


