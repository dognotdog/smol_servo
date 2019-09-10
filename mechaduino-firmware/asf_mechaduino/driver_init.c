/*
 * Code generated from Atmel Start.
 *
 * This file will be overwritten when reconfiguring your Atmel Start project.
 * Please copy examples or other code you want to keep to a separate file
 * to avoid losing it when reconfiguring.
 */

#include "driver_init.h"
#include <peripheral_clk_config.h>
#include <utils.h>
#include <hal_init.h>
#include <hpl_gclk_base.h>
#include <hpl_pm_base.h>

#include <hpl_adc_base.h>

struct spi_m_sync_descriptor SPI_0;

struct adc_sync_descriptor ADC_0;

struct flash_descriptor FLASH_0;

struct i2c_m_sync_desc I2C_0;

struct usart_sync_descriptor USART_0;

void ADC_0_PORT_init(void)
{

	// Disable digital pin circuitry
	gpio_set_pin_direction(PA02, GPIO_DIRECTION_OFF);

	gpio_set_pin_function(PA02, PINMUX_PA02B_ADC_AIN0);

	// Disable digital pin circuitry
	gpio_set_pin_direction(PB08, GPIO_DIRECTION_OFF);

	gpio_set_pin_function(PB08, PINMUX_PB08B_ADC_AIN2);

	// Disable digital pin circuitry
	gpio_set_pin_direction(PA04, GPIO_DIRECTION_OFF);

	gpio_set_pin_function(PA04, PINMUX_PA04B_ADC_AIN4);

	// Disable digital pin circuitry
	gpio_set_pin_direction(PA05, GPIO_DIRECTION_OFF);

	gpio_set_pin_function(PA05, PINMUX_PA05B_ADC_AIN5);
}

void ADC_0_CLOCK_init(void)
{
	_pm_enable_bus_clock(PM_BUS_APBC, ADC);
	_gclk_enable_channel(ADC_GCLK_ID, CONF_GCLK_ADC_SRC);
}

void ADC_0_init(void)
{
	ADC_0_CLOCK_init();
	ADC_0_PORT_init();
	adc_sync_init(&ADC_0, ADC, (void *)NULL);
}

void FLASH_0_CLOCK_init(void)
{

	_pm_enable_bus_clock(PM_BUS_APBB, NVMCTRL);
}

void FLASH_0_init(void)
{
	FLASH_0_CLOCK_init();
	flash_init(&FLASH_0, NVMCTRL);
}

void I2C_0_PORT_init(void)
{

	gpio_set_pin_pull_mode(PA22,
	                       // <y> Pull configuration
	                       // <id> pad_pull_config
	                       // <GPIO_PULL_OFF"> Off
	                       // <GPIO_PULL_UP"> Pull-up
	                       // <GPIO_PULL_DOWN"> Pull-down
	                       GPIO_PULL_OFF);

	gpio_set_pin_function(PA22, PINMUX_PA22C_SERCOM3_PAD0);

	gpio_set_pin_pull_mode(PA23,
	                       // <y> Pull configuration
	                       // <id> pad_pull_config
	                       // <GPIO_PULL_OFF"> Off
	                       // <GPIO_PULL_UP"> Pull-up
	                       // <GPIO_PULL_DOWN"> Pull-down
	                       GPIO_PULL_OFF);

	gpio_set_pin_function(PA23, PINMUX_PA23C_SERCOM3_PAD1);
}

void I2C_0_CLOCK_init(void)
{
	_pm_enable_bus_clock(PM_BUS_APBC, SERCOM3);
	_gclk_enable_channel(SERCOM3_GCLK_ID_CORE, CONF_GCLK_SERCOM3_CORE_SRC);
	_gclk_enable_channel(SERCOM3_GCLK_ID_SLOW, CONF_GCLK_SERCOM3_SLOW_SRC);
}

void I2C_0_init(void)
{
	I2C_0_CLOCK_init();
	i2c_m_sync_init(&I2C_0, SERCOM3);
	I2C_0_PORT_init();
}

void SPI_0_PORT_init(void)
{

	// Set pin direction to input
	gpio_set_pin_direction(PA12, GPIO_DIRECTION_IN);

	gpio_set_pin_pull_mode(PA12,
	                       // <y> Pull configuration
	                       // <id> pad_pull_config
	                       // <GPIO_PULL_OFF"> Off
	                       // <GPIO_PULL_UP"> Pull-up
	                       // <GPIO_PULL_DOWN"> Pull-down
	                       GPIO_PULL_OFF);

	gpio_set_pin_function(PA12, PINMUX_PA12D_SERCOM4_PAD0);

	gpio_set_pin_level(PB10,
	                   // <y> Initial level
	                   // <id> pad_initial_level
	                   // <false"> Low
	                   // <true"> High
	                   false);

	// Set pin direction to output
	gpio_set_pin_direction(PB10, GPIO_DIRECTION_OUT);

	gpio_set_pin_function(PB10, PINMUX_PB10D_SERCOM4_PAD2);

	gpio_set_pin_level(PB11,
	                   // <y> Initial level
	                   // <id> pad_initial_level
	                   // <false"> Low
	                   // <true"> High
	                   false);

	// Set pin direction to output
	gpio_set_pin_direction(PB11, GPIO_DIRECTION_OUT);

	gpio_set_pin_function(PB11, PINMUX_PB11D_SERCOM4_PAD3);
}

void SPI_0_CLOCK_init(void)
{
	_pm_enable_bus_clock(PM_BUS_APBC, SERCOM4);
	_gclk_enable_channel(SERCOM4_GCLK_ID_CORE, CONF_GCLK_SERCOM4_CORE_SRC);
}

void SPI_0_init(void)
{
	SPI_0_CLOCK_init();
	spi_m_sync_init(&SPI_0, SERCOM4);
	SPI_0_PORT_init();
}

void USART_0_PORT_init(void)
{

	gpio_set_pin_function(TXD, PINMUX_PB22D_SERCOM5_PAD2);

	gpio_set_pin_function(RXD, PINMUX_PB23D_SERCOM5_PAD3);
}

void USART_0_CLOCK_init(void)
{
	_pm_enable_bus_clock(PM_BUS_APBC, SERCOM5);
	_gclk_enable_channel(SERCOM5_GCLK_ID_CORE, CONF_GCLK_SERCOM5_CORE_SRC);
}

void USART_0_init(void)
{
	USART_0_CLOCK_init();
	usart_sync_init(&USART_0, SERCOM5, (void *)NULL);
	USART_0_PORT_init();
}

void delay_driver_init(void)
{
	delay_init(SysTick);
}

void TIMER_0_CLOCK_init(void)
{
	_pm_enable_bus_clock(PM_BUS_APBC, TC4);
	_gclk_enable_channel(TC4_GCLK_ID, CONF_GCLK_TC4_SRC);
}

void PWM_0_PORT_init(void)
{

	gpio_set_pin_function(VREF2, PINMUX_PA08E_TCC0_WO0);

	gpio_set_pin_function(MOT_IN3, PINMUX_PA15F_TCC0_WO5);

	gpio_set_pin_function(MOT_IN4, PINMUX_PA20F_TCC0_WO6);

	gpio_set_pin_function(MOT_IN2, PINMUX_PA21F_TCC0_WO7);
}

void PWM_0_CLOCK_init(void)
{
	_pm_enable_bus_clock(PM_BUS_APBC, TCC0);
	_gclk_enable_channel(TCC0_GCLK_ID, CONF_GCLK_TCC0_SRC);
}

void PWM_1_PORT_init(void)
{

	gpio_set_pin_function(MOT_IN1, PINMUX_PA06E_TCC1_WO0);

	gpio_set_pin_function(VREF1, PINMUX_PA07E_TCC1_WO1);
}

void PWM_1_CLOCK_init(void)
{
	_pm_enable_bus_clock(PM_BUS_APBC, TCC1);
	_gclk_enable_channel(TCC1_GCLK_ID, CONF_GCLK_TCC1_SRC);
}

void USB_DEVICE_INSTANCE_PORT_init(void)
{

	gpio_set_pin_direction(USBN,
	                       // <y> Pin direction
	                       // <id> pad_direction
	                       // <GPIO_DIRECTION_OFF"> Off
	                       // <GPIO_DIRECTION_IN"> In
	                       // <GPIO_DIRECTION_OUT"> Out
	                       GPIO_DIRECTION_OUT);

	gpio_set_pin_level(USBN,
	                   // <y> Initial level
	                   // <id> pad_initial_level
	                   // <false"> Low
	                   // <true"> High
	                   false);

	gpio_set_pin_pull_mode(USBN,
	                       // <y> Pull configuration
	                       // <id> pad_pull_config
	                       // <GPIO_PULL_OFF"> Off
	                       // <GPIO_PULL_UP"> Pull-up
	                       // <GPIO_PULL_DOWN"> Pull-down
	                       GPIO_PULL_OFF);

	gpio_set_pin_function(USBN,
	                      // <y> Pin function
	                      // <id> pad_function
	                      // <i> Auto : use driver pinmux if signal is imported by driver, else turn off function
	                      // <PINMUX_PA24G_USB_DM"> Auto
	                      // <GPIO_PIN_FUNCTION_OFF"> Off
	                      // <GPIO_PIN_FUNCTION_A"> A
	                      // <GPIO_PIN_FUNCTION_B"> B
	                      // <GPIO_PIN_FUNCTION_C"> C
	                      // <GPIO_PIN_FUNCTION_D"> D
	                      // <GPIO_PIN_FUNCTION_E"> E
	                      // <GPIO_PIN_FUNCTION_F"> F
	                      // <GPIO_PIN_FUNCTION_G"> G
	                      // <GPIO_PIN_FUNCTION_H"> H
	                      PINMUX_PA24G_USB_DM);

	gpio_set_pin_direction(USBP,
	                       // <y> Pin direction
	                       // <id> pad_direction
	                       // <GPIO_DIRECTION_OFF"> Off
	                       // <GPIO_DIRECTION_IN"> In
	                       // <GPIO_DIRECTION_OUT"> Out
	                       GPIO_DIRECTION_OUT);

	gpio_set_pin_level(USBP,
	                   // <y> Initial level
	                   // <id> pad_initial_level
	                   // <false"> Low
	                   // <true"> High
	                   false);

	gpio_set_pin_pull_mode(USBP,
	                       // <y> Pull configuration
	                       // <id> pad_pull_config
	                       // <GPIO_PULL_OFF"> Off
	                       // <GPIO_PULL_UP"> Pull-up
	                       // <GPIO_PULL_DOWN"> Pull-down
	                       GPIO_PULL_OFF);

	gpio_set_pin_function(USBP,
	                      // <y> Pin function
	                      // <id> pad_function
	                      // <i> Auto : use driver pinmux if signal is imported by driver, else turn off function
	                      // <PINMUX_PA25G_USB_DP"> Auto
	                      // <GPIO_PIN_FUNCTION_OFF"> Off
	                      // <GPIO_PIN_FUNCTION_A"> A
	                      // <GPIO_PIN_FUNCTION_B"> B
	                      // <GPIO_PIN_FUNCTION_C"> C
	                      // <GPIO_PIN_FUNCTION_D"> D
	                      // <GPIO_PIN_FUNCTION_E"> E
	                      // <GPIO_PIN_FUNCTION_F"> F
	                      // <GPIO_PIN_FUNCTION_G"> G
	                      // <GPIO_PIN_FUNCTION_H"> H
	                      PINMUX_PA25G_USB_DP);
}

/* The USB module requires a GCLK_USB of 48 MHz ~ 0.25% clock
 * for low speed and full speed operation. */
#if (CONF_GCLK_USB_FREQUENCY > (48000000 + 48000000 / 400)) || (CONF_GCLK_USB_FREQUENCY < (48000000 - 48000000 / 400))
#warning USB clock should be 48MHz ~ 0.25% clock, check your configuration!
#endif

void USB_DEVICE_INSTANCE_CLOCK_init(void)
{

	_pm_enable_bus_clock(PM_BUS_APBB, USB);
	_pm_enable_bus_clock(PM_BUS_AHB, USB);
	_gclk_enable_channel(USB_GCLK_ID, CONF_GCLK_USB_SRC);
}

void USB_DEVICE_INSTANCE_init(void)
{
	USB_DEVICE_INSTANCE_CLOCK_init();
	usb_d_init();
	USB_DEVICE_INSTANCE_PORT_init();
}

void system_init(void)
{
	init_mcu();

	// GPIO on PA17

	gpio_set_pin_level(MECH_LED,
	                   // <y> Initial level
	                   // <id> pad_initial_level
	                   // <false"> Low
	                   // <true"> High
	                   true);

	// Set pin direction to output
	gpio_set_pin_direction(MECH_LED, GPIO_DIRECTION_OUT);

	gpio_set_pin_function(MECH_LED, GPIO_PIN_FUNCTION_OFF);

	// GPIO on PA27

	gpio_set_pin_level(TX_LED,
	                   // <y> Initial level
	                   // <id> pad_initial_level
	                   // <false"> Low
	                   // <true"> High
	                   false);

	// Set pin direction to output
	gpio_set_pin_direction(TX_LED, GPIO_DIRECTION_OUT);

	gpio_set_pin_function(TX_LED, GPIO_PIN_FUNCTION_OFF);

	// GPIO on PB03

	gpio_set_pin_level(RX_LED,
	                   // <y> Initial level
	                   // <id> pad_initial_level
	                   // <false"> Low
	                   // <true"> High
	                   false);

	// Set pin direction to output
	gpio_set_pin_direction(RX_LED, GPIO_DIRECTION_OUT);

	gpio_set_pin_function(RX_LED, GPIO_PIN_FUNCTION_OFF);

	// GPIO on PB09

	gpio_set_pin_level(HEADER_SS,
	                   // <y> Initial level
	                   // <id> pad_initial_level
	                   // <false"> Low
	                   // <true"> High
	                   false);

	// Set pin direction to output
	gpio_set_pin_direction(HEADER_SS, GPIO_DIRECTION_OUT);

	gpio_set_pin_function(HEADER_SS, GPIO_PIN_FUNCTION_OFF);

	ADC_0_init();

	FLASH_0_init();

	I2C_0_init();

	SPI_0_init();

	USART_0_init();

	delay_driver_init();

	TIMER_0_CLOCK_init();

	TIMER_0_init();

	PWM_0_CLOCK_init();

	PWM_0_PORT_init();

	PWM_0_init();

	PWM_1_CLOCK_init();

	PWM_1_PORT_init();

	PWM_1_init();

	USB_DEVICE_INSTANCE_init();
}
