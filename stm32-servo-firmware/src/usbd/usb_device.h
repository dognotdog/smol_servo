#ifndef USB_DEVICE_H
#define USB_DEVICE_H

#include "usbd.h"

extern USBD_HandleType *const UsbDevice;

extern void HAL_USBD_Setup(void);
void UsbDevice_Init(void);

#endif // USB_DEVICE_H
