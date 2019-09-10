#ifndef DEBUG_H
#define DEBUG_H

extern void usb_printf(const char* const format, ...);

#define dbg_println(fmt, ...) usb_printf("DBG " fmt "\r\n", ##__VA_ARGS__)
// #define dbg_println(fmt, ...) usb_printf("DBG %u " fmt "\r\n", HAL_GetTick(), ##__VA_ARGS__)
#define err_println(fmt, ...) usb_printf("ERR " fmt "\r\n", ##__VA_ARGS__)


#endif // DEBUG_H
