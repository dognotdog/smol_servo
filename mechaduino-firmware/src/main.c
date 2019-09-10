
#include "atmel_start.h"
#include "debug.h"
#include "mecha_pwm.h"

#include <stdarg.h>
#include <stdio.h>


/** Buffers to receive and echo the communication bytes. */
static uint32_t usbd_cdc_buffer[64 / 4];

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

uint16_t sin1024[32] = {0,100,200,297,391,482,568,649,723,791,851,902,945,979,1003,1018,1023,1018,1003,979,945,902,851,791,723,649,568,482,391,297,200,100};


int rotatingPwm64(const mpwm_id_t ch0, const mpwm_id_t ch1 , const uint32_t phase)
{
	uint32_t p64 = phase % 64;
	uint32_t p32 = phase % 32;

	uint32_t pwm = sin1024[p32];;

	uint32_t pwm0 = 0;
	uint32_t pwm1 = 0;

	if (p64 < 32)
		pwm0 = pwm;
	else
		pwm1 = pwm;



	mpwm_setPwm(ch0, pwm0);
	mpwm_setPwm(ch1, pwm1);

	return pwm0-pwm1;

};

int rotatingPwmHS(const mpwm_id_t ch0, const mpwm_id_t ch1 , const uint32_t phase)
{
	uint32_t p0[8] = {1,1,1,0,0,0,0,0};
	uint32_t p1[8] = {0,0,0,0,1,1,1,0};
	uint32_t p8 = phase % 8;


	mpwm_setPwm(ch0, p0[p8]*1023);
	mpwm_setPwm(ch1, p1[p8]*1023);

	return p0[p8]*1023 - p1[p8]*1023;
};

volatile extern fix1p30_t mpwm_sines[2];

int main(void)
{

	atmel_start_init();

	// while (!cdcdf_acm_is_enabled()) {
	// 	// wait cdc acm to be installed
	// };

	// cdcdf_acm_register_callback(CDCDF_ACM_CB_STATE_C, (FUNC_PTR)usb_device_cb_state_c);

	mpwm_init();

	uint32_t phase = 0;
	const uint32_t ps = 2;



	bool ledOn = true;
	while (1) 
	{
		// int p0 = rotatingPwmHS(MPWM_F0, MPWM_R0, phase);
		// int p1 = rotatingPwmHS(MPWM_F1, MPWM_R1, phase+ps);
		// dbg_println("PWM %5i, %5i", p0, p1);
		dbg_println("MPWM %5i, %5i", mpwm_sines[0] >> 0, mpwm_sines[1] >> 0);
		++phase;
		gpio_set_pin_level(MECH_LED, ledOn);
		ledOn = !ledOn;
		delay_ms(200);
		// dbg_println("Hi There!");

	}

}