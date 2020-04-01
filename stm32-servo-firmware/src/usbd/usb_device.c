



#include "usb_device.h"
#include "usbd_cdc.h"
#include "usbd_hid.h"

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
    // USB_ChargerType usbPort;


 //    // setup the USB device class
 //    USB_DeviceDescType desc = {
	//    	.bLength        	= sizeof(USB_DeviceDescType),
	//     .bDescriptorType    = USB_DESC_TYPE_DEVICE,
	//     .bcdUSB             = USBD_SPEC_BCD,
	//     .bDeviceClass       = 239,
	//     .bDeviceSubClass    = 2,
	//     .bDeviceProtocol    = 1,
	//     .bMaxPacketSize     = USBD_EP0_MAX_PACKET_SIZE,
	//     .idVendor           = 0x0483,
	//     .idProduct          = 0x5740,
	//     .bcdDevice          = 0xFFFF,
	//     .iManufacturer      = USBD_ISTR_VENDOR,
	//     .iProduct           = USBD_ISTR_PRODUCT,
	// #if (USBD_SERIAL_BCD_SIZE > 0)
	//     .iSerialNumber      = USBD_ISTR_SERIAL,
	// #endif
	//     .bNumConfigurations = USBD_MAX_CONFIGURATION_COUNT
 //    };

 //    USBD_DeviceDesc(self, &desc);

    /* Initialize the device */
    USBD_Init(self, &_deviceConfig);


    // console_if->App = &vcpApp;
    /* All fields of Config have to be properly set up */
    console_if->Config.InEpNum  = 0x81;
    console_if->Config.OutEpNum = 0x01;
    console_if->Config.NotEpNum = 0x8F;

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
