
#include "ssf_flash.h"

#include <stddef.h>
#include "debug.h"
#include "ssf_main.h"

void ssf_flashInit(void)
{
	dbg_println("Free Flash memory starts at 0x%08X", (uintptr_t)__freeflash_firstPage_addr);
	dbg_println("Free Flash memory ends at   0x%08X", (uintptr_t)__freeflash_end_addr);
	dbg_println("Free Flash memory contains  0x%08X (%u) bytes", (uintptr_t)(__freeflash_end_addr - __freeflash_firstPage_addr), (uintptr_t)(__freeflash_end_addr - __freeflash_firstPage_addr));


}