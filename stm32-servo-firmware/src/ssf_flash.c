
#include "ssf_flash.h"

#include <stddef.h>
#include "debug.h"
#include "ssf_main.h"

void ssf_flashInit(void)
{
	size_t freeBytes = (uintptr_t)(__freeflash_end_addr - __freeflash_firstPage_addr);

	dbg_println("Free Flash memory starts at 0x%08X", (uintptr_t)__freeflash_firstPage_addr);
	dbg_println("Free Flash memory ends at   0x%08X", (uintptr_t)__freeflash_end_addr);
	dbg_println("Free Flash memory contains  0x%08X (%u) bytes", freeBytes, freeBytes);

	size_t numFreePages = freeBytes/FLASH_PAGE_SIZE;
	dbg_println("                            0x%08X (%u) pages", numFreePages, numFreePages);


}
