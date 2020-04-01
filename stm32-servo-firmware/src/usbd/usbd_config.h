#ifndef USBD_CONFIG_H
#define USBD_CONFIG_H

#define USBD_MAX_IF_COUNT			2
#define USBD_EP0_BUFFER_SIZE		256
#define USBD_HS_SUPPORT             0
#define USBD_SERIAL_BCD_SIZE        24
#define USBD_CDC_NOTEP_USED         1
#define USBD_CDC_CONTROL_LINE_USED  0
#define USBD_CDC_BREAK_SUPPORT      0
#define USBD_DFU_ALTSETTINGS        0
#define USBD_DFU_ST_EXTENSION       0
#define USBD_HID_ALTSETTINGS        0
#define USBD_HID_OUT_SUPPORT        1 // enable HID out support for force feedback
#define USBD_HID_REPORT_STRINGS     0

#define STDOUT_BUFFER_SIZE 	256
#define STDIN_BUFFER_SIZE	128

#endif // USBD_CONFIG_H
