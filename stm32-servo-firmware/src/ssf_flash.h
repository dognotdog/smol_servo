#ifndef SSF_FLASH_H
#define SSF_FLASH_H

#include <stdint.h>
#include <stdbool.h>

// Flash addresses from linker script
extern uint8_t const __freeflash_firstPage;
extern uint8_t const __freeflash_lastPage;
extern uint8_t const __freeflash_start;
extern uint8_t const __freeflash_end;
#define __freeflash_firstPage_addr (&__freeflash_firstPage)
#define __freeflash_lastPage_addr (&__freeflash_lastPage)
#define __freeflash_start_addr (&__freeflash_start)
#define __freeflash_end_addr (&__freeflash_end)

#define FLASH_CONFIG_VERSION	1

typedef struct {
	uint32_t struct_version;	// type of struct
	uint32_t data_version;		// counts up as config is changed
} device_config_v1_t;

typedef device_config_v1_t device_config_t;

void ssf_flashInit(void);

device_config_t* ssf_flash_getCurrentDeviceConfig(void);
bool ssf_flash_readDeviceConfigFromFlash(void);

#endif // SSF_FLASH_H
