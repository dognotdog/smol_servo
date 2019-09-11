
#include "atmel_start.h"
#include "debug.h"
#include "mecha_pwm.h"
#include "mecha_spi.h"

#include <stdarg.h>
#include <stdio.h>


/** Buffers to receive and echo the communication bytes. */
static char __ALIGNED(4) usbd_cdc_buffer[64];
static char __ALIGNED(4) usb_linebuf[256];
static size_t usb_linebufCount;

/**
 * \brief Callback invoked when bulk OUT data received
 */
static bool usb_device_cb_bulk_out(const uint8_t ep, const enum usb_xfer_code rc, const uint32_t count)
{
	// const char hello[64] = "Hi There!\r\n";
	// cdcdf_acm_write((void*)hello, count);
	cdcdf_acm_write((uint8_t *)usbd_cdc_buffer, count);

	/* No error. */
	return false;
}

/**
 * \brief Callback invoked when bulk IN data received
 */
static bool usb_device_cb_bulk_in(const uint8_t ep, const enum usb_xfer_code rc, const uint32_t count)
{
	/* Echo data. */

	cdcdf_acm_read((uint8_t *)usbd_cdc_buffer, sizeof(usbd_cdc_buffer));

	for (size_t i = 0; i < count; ++i)
	{
		if (usb_linebufCount == sizeof(usb_linebuf))
		{
			// discard everything
		}
	}

	/* No error. */
	return false;
}

/**
 * \brief Callback invoked when Line State Change
 */
static bool usb_device_cb_state_c(usb_cdc_control_signal_t state)
{
	if (state.rs232.DTR) {
		/* Callbacks must be registered after endpoint allocation */
		cdcdf_acm_register_callback(CDCDF_ACM_CB_READ, (FUNC_PTR)usb_device_cb_bulk_out);
		cdcdf_acm_register_callback(CDCDF_ACM_CB_WRITE, (FUNC_PTR)usb_device_cb_bulk_in);
		/* Start Rx */
		cdcdf_acm_read((uint8_t *)usbd_cdc_buffer, sizeof(usbd_cdc_buffer));
	}

	/* No error. */
	return false;
}

char hello[] = "Hi There!\r\n";

void usb_vprintf(const char* const format, va_list args)
{
	char buf[256] = "";
		
	vsnprintf(buf, sizeof(buf), format, args);
	
	size_t len = strnlen(buf, sizeof(buf));

	int err = USB_BUSY;
	// while (err == USB_BUSY)
	{
		err = cdcdf_acm_write((void*)buf, len);
	};
}

void usb_printf(const char* const format, ...)
{        
	va_list args;
	va_start (args, format);
	usb_vprintf(format, args);
	va_end(args);
}


volatile extern fix1p30_t mpwm_sines[2];

int main(void)
{

	atmel_start_init();

	// while (!cdcdf_acm_is_enabled()) {
	// 	// wait cdc acm to be installed
	// };

	// cdcdf_acm_register_callback(CDCDF_ACM_CB_STATE_C, (FUNC_PTR)usb_device_cb_state_c);

	mpwm_init();
	mspi_init();


	bool ledOn = true;
	while (1) 
	{
		mspi_as5047_state_t hallSensor = mspi_readSensor();
		dbg_println("MSPI %04x, %04x, %04x", hallSensor.ERRFL, hallSensor.DIAAGC, hallSensor.ANGLEUNC);

		// dbg_println("MPWM %5i, %5i", mpwm_sines[0] >> 0, mpwm_sines[1] >> 0);
		
		gpio_set_pin_level(MECH_LED, ledOn);
		ledOn = !ledOn;
		delay_ms(200);
		// dbg_println("Hi There!");

	}

}