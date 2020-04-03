#ifndef SERVO_HID_IF_H
#define SERVO_HID_IF_H

#include "usbd_hid.h"

extern USBD_HID_IfHandleType* const servo_hid_if;

extern void servo_hid_interface_run(uint32_t time_us);

#endif // SERVO_HID_IF_H
