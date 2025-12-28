/**
 * 
 */

#include <stdio.h>

#include "pico/stdlib.h"
#include "pico/binary_info.h"

#include "hardware/i2c.h"

#include "smol_servo_rpico.h"

#include "fusb302.h"
#include "tmc6200.h"


void i2c_fusb_init(void) {
    i2c_init(FUSB_I2C, 100 * 1000);
    gpio_set_function(FUSB_I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(FUSB_I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(FUSB_I2C_SDA_PIN);
    gpio_pull_up(FUSB_I2C_SCL_PIN);
    // Make the I2C pins available to picotool
    bi_decl(bi_2pins_with_func(FUSB_I2C_SDA_PIN, FUSB_I2C_SCL_PIN, GPIO_FUNC_I2C));

}

#define LED_PIN 8

// Initialize the GPIO for the LED
void pico_led_init(void) {
    // A device like Pico that uses a GPIO for the LED will define PICO_DEFAULT_LED_PIN
    // so we can use normal GPIO functionality to turn the led on and off
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);
}

// Turn the LED on or off
void pico_set_led(bool led_on) {
    // Just set the GPIO on or off
    gpio_put(LED_PIN, led_on);
}


int main(int argc, char const *argv[])
{
    stdio_init_all();
    pico_led_init();

    // Make the SPI pins available to picotool
    bi_decl(bi_3pins_with_func(DRV_MISO_PIN, DRV_MOSI_PIN, DRV_SLCK_PIN, GPIO_FUNC_SPI));
    // Make the CS pin available to picotool
    bi_decl(bi_1pin_with_name(DRV_NSS_PIN, "DRV_SPI CS"));

    tmc6200_spi_init(DRV_SPI, DRV_MOSI_PIN, DRV_NSS_PIN, DRV_SLCK_PIN, DRV_MISO_PIN);

	i2c_fusb_init();
	fusb302_init(FUSB_I2C);

	bool led = false;
	while (1) {
		pico_set_led(led = !led);
		// printf("foo\n");
	}
	return 0;
}