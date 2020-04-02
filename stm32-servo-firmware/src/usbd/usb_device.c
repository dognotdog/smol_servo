



#include "usb_device.h"
#include "usbd_cdc.h"
#include "usbd_hid.h"

#include <fcntl.h>
#include <stdio.h>

/*
	We have DFU interface for debugging
	and a HID interface for sensor values / force-feedback

	Initialization

	USBD_Init() inits the device

*/

static const USBD_DescriptionType _deviceConfig = {
    .Vendor = {
        .Name           = "STMicroelectronics",
        // .ID             = 0x0483,
        .ID             = 0x1D50,
    },
    .Product = {
        .Name           = "STM32 Virtual ComPort",
        // .ID             = 0x5740,
        .ID             = 0x6018,
        .Version.bcd    = 0x0100,
    },
#if (USBD_SERIAL_BCD_SIZE > 0)
    .SerialNumber       = (USBD_SerialNumberType*)UID_BASE,
#endif
    .Config = {
        .Name           = "CDC Config",
        .MaxCurrent_mA  = 100,
        .RemoteWakeup   = 0,
        .SelfPowered    = 0,
    },
};

USBD_HandleType gUsbDevice;

extern USBD_CDC_IfHandleType *const console_if;
extern USBD_HID_IfHandleType *const hid_if;

static void usbSuspendCallback(void * devHandle)
{
}

static void usbResumeCallback(void * devHandle)
{
}

void UsbDevice_Init(void)
{
	USBD_HandleType* const self = &gUsbDevice;

    // configure STDIN to be non-blocking
    int flags = fcntl(0, F_GETFL, 0);
    fcntl(0, F_SETFL, flags | O_NONBLOCK);
    // don't do extra buffering in stdlib
    setvbuf(stdin, NULL, _IONBF, 0);
    setvbuf(stdout, NULL, _IONBF, 0);
    setvbuf(stderr, NULL, _IONBF, 0);
    /* Initialize the device */
    USBD_Init(self, &_deviceConfig);


    // console_if->App = &vcpApp;
    /* All fields of Config have to be properly set up */
    console_if->Config.InEpNum  = 0x81;
    console_if->Config.OutEpNum = 0x01;
    console_if->Config.NotEpNum = 0x82;

    // hid_if->Config.InEpNum = 0x82;
    // hid_if->Config.OutEpNum = 0x02;

    // USBD_DFU_AppInit(dfu_if, 250); /* Detach can be carried out within 250 ms */

    /* Mount the interfaces to the device */
    // USBD_DFU_MountInterface(dfu_if, UsbDevice);
    USBD_CDC_MountInterface(console_if, self);
    // USBD_HID_MountInterface(hid_if, &_usbDevice);

    self->Callbacks.Suspend = usbSuspendCallback;
    self->Callbacks.Resume = usbResumeCallback;
    /* After charger detection the device connection can be made */
    USBD_Connect(self);
}
