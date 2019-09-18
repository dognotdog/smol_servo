
#include "debug.h"
#include "usbd_cdc_if.h"

#include <string.h>
#include <stdio.h>
#include <stdarg.h>
#include <stdint.h>

void usb_send(const void* const data, size_t len)
{
    size_t sent = 0;
    while (sent < len)
    {
        size_t plen = len - sent > 64 ? 64 : len - sent;
        int err = USBD_BUSY;
        while (err == USBD_BUSY)
            err = CDC_Transmit_FS(((uint8_t*)data) + sent, plen);
        sent += plen;
    }
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
