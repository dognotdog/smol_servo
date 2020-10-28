#ifndef SSF_FLASH_H
#define SSF_FLASH_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

// Flash addresses from linker script
extern uint8_t const __freeflash_firstPage;
extern uint8_t const __freeflash_lastPage;
extern uint8_t const __freeflash_start;
extern uint8_t const __freeflash_end;
#define __freeflash_firstPage_addr (&__freeflash_firstPage)
#define __freeflash_lastPage_addr (&__freeflash_lastPage)
#define __freeflash_start_addr (&__freeflash_start)
#define __freeflash_end_addr (&__freeflash_end)

extern uint8_t const __configflash_firstPage;
extern uint8_t const __configflash_lastPage;
extern uint8_t const __configflash_start;
extern uint8_t const __configflash_end;
#define __configflash_firstPage_addr (&__configflash_firstPage)
#define __configflash_lastPage_addr (&__configflash_lastPage)
#define __configflash_start_addr (&__configflash_start)
#define __configflash_end_addr (&__configflash_end)

#define FLASH_CONFIG_VERSION	1
// FLASH_PAGE_SIZE defined in HAL
#define FLASH_WORD_SIZE			8

#define FLASH_CONFIG_MAX_TYPES				(16u)
#define FLASH_CONFIG_MAX_PRESERVED_TYPES	(8u)

#define FLASH_CONFIG_TYPE_DEVICECONF_V1		(0)

/*
	This is a logging file system for storing configuration data. 
	- data is stored in blocks
	- blocks cannot cross page boundaries (empty space left at end of pages if next block does not fit)
	- blocks have types
	- the last written block of a type can be preserved even if its page is erased
	- blocks have sequence numbers, highest sequence number is current
		- 0x0000 indicates spacer block
		- 0xFFFF is unwritten block
	- block flags
		- 16 block types
		- 0-7 are preserved types, 8-15 are unpreserved
	- to keep track of preserved blocks, the next page with a current preserved block must be tracked, so that when it is coming up to be erased, the block can be preserved before the page is erased 

	restrictions:
		1. all preserved blocks together not to exceed a single page size
		2. 64bit words as allocation unit
			block structs need to have 64bit sizing
		3. apparently, flash write operations block normal code execution on single bank devices

	INIT
		1. determine beginning end of used space
			scan blocks sequentially until a block whose 
			first block after empty, or 
*/

// typedef enum {
// 	FLASH_PAGE_OK,
// 	FLASH_PAGE_ERROR
// } flash_pageStatus_t;

typedef struct {
	uint16_t sequenceId;	// counts up as config is changed, skip 0x0000, 0xFFFF
	uint8_t numWords;	// number of words, zero/-1 indicates invalid block
	uint8_t flags;		// block flags | 4bit type | 4bit flags |
} flash_blockInfo_t;

typedef struct {
	flash_blockInfo_t blockInfo; // 4 bytes
	struct {
		float ratedCurrent_A;
		float peakCurrent_A;
		float rotorInertia_kgm2;
		float torqueConstant_Nm_per_A;
		float senseResistance_Ohm;
	} userParams; // 20 bytes
	struct {
		float phaseInductance_H;
		float phaseResistance_Ohm;
		float bridgeResistance_Ohm;
		uint8_t numPhases;
	} measuredParams; // 16 bytes
} device_config_v1_t;

typedef device_config_v1_t device_config_t;

void ssf_flashInit(void);
bool ssf_flash_tryWriteBlock(uint8_t flags, uintptr_t dataAddr, size_t datalen);
bool ssf_flash_tryReadLastBlock(uint8_t flags, void* dst, size_t* len);

bool ssf_flash_clearWriteError(void);
bool ssf_flash_clearConfigFlash(void);

device_config_t* ssf_flash_getCurrentDeviceConfig(void);
bool ssf_flash_readDeviceConfigFromFlash(void);

#endif // SSF_FLASH_H
