
#include "debug.h"
#include "usbd_cdc_if.h"

#include <string.h>
#include <stdio.h>
#include <stdarg.h>
#include <stdint.h>

void usb_send(const void* const data, size_t len)
{
	int err = USBD_BUSY;
	while (err == USBD_BUSY)
		err = CDC_Transmit_FS((uint8_t*)data, len);
}


void usb_vprintf(const char* const format, va_list args)
{
    char buf[256] = "";
        
    vsnprintf(buf, sizeof(buf), format, args);
    
    size_t len = strnlen(buf, sizeof(buf));

    usb_send(buf, len);
}

void usb_printf(const char* const format, ...)
{        
    va_list args;
    va_start (args, format);
    usb_vprintf(format, args);
    va_end(args);
 }
