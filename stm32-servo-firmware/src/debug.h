#ifndef DEBUG_H
#define DEBUG_H

#include <inttypes.h>

#ifdef DEBUG_USB_PRINTF

	extern void usb_printf(const char* const format, ...);

	#define dbg_println(fmt, ...) usb_printf("DBG %"PRIu32" " fmt "\r\n", HAL_GetTick(), ##__VA_ARGS__)
	#define err_println(fmt, ...) usb_printf("ERR " fmt "\r\n", ##__VA_ARGS__)

#elif defined DEBUG_UART_PRINTF

	extern void uart_printf(const char* const format, ...);

	#define dbg_println(fmt, ...) uart_printf("DBG %"PRIu32" " fmt "\r\n", HAL_GetTick(), ##__VA_ARGS__)
	#define err_println(fmt, ...) uart_printf("ERR " fmt "\r\n", ##__VA_ARGS__)

#else

	#include <stdio.h>

	#define dbg_println(fmt, ...) printf("DBG %"PRIu32" " fmt "\r\n", HAL_GetTick(), ##__VA_ARGS__)
	#define err_println(fmt, ...) printf("ERR " fmt "\r\n", ##__VA_ARGS__)

#endif // DEBUG_CUSTOM_PRINTF

#endif // DEBUG_H
