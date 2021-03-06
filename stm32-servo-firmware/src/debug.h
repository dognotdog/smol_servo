#ifndef DEBUG_H
#define DEBUG_H

#include <inttypes.h>
#include <stdarg.h>

#ifdef DEBUG_USB_PRINTF

	extern void usb_printf(const char* const format, ...);

	#define dbg_printf(fmt, ...) usb_printf(fmt, ##__VA_ARGS__)
	#define dbg_println(fmt, ...) usb_printf("DBG %"PRIu32" " fmt "\r\n", HAL_GetTick(), ##__VA_ARGS__)
	#define warn_println(fmt, ...) usb_printf("WRN %"PRIu32" " fmt "\r\n", HAL_GetTick(), ##__VA_ARGS__)
	#define err_println(fmt, ...) usb_printf("ERR %"PRIu32" " fmt "\r\n", HAL_GetTick(), ##__VA_ARGS__)

#elif defined DEBUG_UART_PRINTF

	extern void uart_printf(const char* const format, ...);

	#define dbg_printf(fmt, ...) uart_printf(fmt, ##__VA_ARGS__)
	#define dbg_println(fmt, ...) uart_printf("DBG %"PRIu32" " fmt "\r\n", HAL_GetTick(), ##__VA_ARGS__)
	#define warn_println(fmt, ...) uart_printf("WRN %"PRIu32" " fmt "\r\n", HAL_GetTick(), ##__VA_ARGS__)
	#define err_println(fmt, ...) uart_printf("ERR %"PRIu32" " fmt "\r\n", HAL_GetTick(), ##__VA_ARGS__)

#else

	#include <stdio.h>

	#define dbg_printf(fmt, ...) printf(fmt, ##__VA_ARGS__)
	#define dbg_println(fmt, ...) printf("DBG %"PRIu32" " fmt "\r\n", HAL_GetTick(), ##__VA_ARGS__)
	#define warn_println(fmt, ...) printf("WRN %"PRIu32" " fmt "\r\n", HAL_GetTick(), ##__VA_ARGS__)
	#define err_println(fmt, ...) printf("ERR %"PRIu32" " fmt "\r\n", HAL_GetTick(), ##__VA_ARGS__)

#endif // DEBUG_CUSTOM_PRINTF

#endif // DEBUG_H
