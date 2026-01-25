/**
 * 
 */

#include <stdio.h>

#include "pico/stdlib.h"
#include "pico/binary_info.h"
#include "pico/critical_section.h"
#include "pico/multicore.h"

#include "hardware/i2c.h"
#include "hardware/structs/busctrl.h"
#include "hardware/structs/accessctrl.h"

#include "smol_servo.h"
#include "smol_fs.h"
#include "smol_console.h"
#include "pico_debug.h"

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

void smol_hires_time_init(void) {
	// use a critical section to block IRQs, at least
	// this should not be called once the second core is up
	critical_section_t cr;
	critical_section_init(&cr);
	critical_section_enter_blocking(&cr);
	timer_hw_t *timer_us = timer0_hw;
	timer_hw_t *timer_sysclk = timer1_hw;



	// pause timers
	bool was_paused = timer_us->pause;
	timer_us->pause = 1;
	timer_sysclk->pause = 1;

	// set 2nd timer to sysclk
	timer_sysclk->source = 1;
	// set time to match 1MHz timer
	uint64_t time_us = timer_us->timelr;
	time_us = (time_us << 32) | timer_us->timehr;
	// looking at runtime_init_clocks(), default is 150MHz, we'll just go with that
	uint64_t time_sysclk = time_us * 150u;
	timer_sysclk->timelw = time_sysclk;
	timer_sysclk->timehw = time_sysclk >> 32;

	// unpause both timers with hopefully negligible delay
	timer_us->pause = was_paused;
	timer_sysclk->pause = was_paused;

	critical_section_exit(&cr);
	critical_section_deinit(&cr);
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

static void __not_in_flash_func(_core1_init)(void) {
	// // mess with AHB priorities
	// uint32_t prio = busctrl_hw->priority;
	// // increase Core1 bus priority
	// prio = BUSCTRL_BUS_PRIORITY_PROC1_BITS;
	// busctrl_hw->priority = prio;

	smol_servo_loop_init();
	// smol_servo_loop_start();


	// disable FLASH access for CORE 1 so we don't get XIP data stalls
	hw_clear_bits(&accessctrl_hw->xip_main, 0xACCE0000 | ACCESSCTRL_XIP_MAIN_CORE1_BITS);

	while (1) {
		smol_servo_loop_idle_core1();
	}
}

int main(int argc, char const *argv[])
{
	smol_hires_time_init();

    stdio_init_all();
    pico_led_init();

    smol_fs_init();

    // Make the SPI pins available to picotool
    bi_decl(bi_3pins_with_func(DRV_MISO_PIN, DRV_MOSI_PIN, DRV_SLCK_PIN, GPIO_FUNC_SPI));
    // Make the CS pin available to picotool
    bi_decl(bi_1pin_with_name(DRV_NSS_PIN, "DRV_SPI CS"));

    tmc6200_spi_init(DRV_SPI, DRV_MOSI_PIN, DRV_NSS_PIN, DRV_SLCK_PIN, DRV_MISO_PIN);

	i2c_fusb_init();
	fusb302_init(FUSB_I2C, FUSB_INT_PIN);
	smol_usb_pd_init();

	multicore_reset_core1();
	multicore_launch_core1(_core1_init);

	bool led = false;
	while (1) {
		picobug_process_core1_queue();

		int ch = getchar_timeout_us(0);
		if (ch >= 0) {
			smol_console_process_char(ch);
		}

		smol_usb_pd_run();

		smol_servo_loop_idle_core0();

		// pico_set_led(led = !led);
		// uint64_t t_fast = timer_time_us_64(timer1_hw);
		// dbg_time_t tt = {.seconds = t_fast / 1000000u, .microseconds = t_fast % 1000000u};
		// dbg_println("foo %" PRIu32 ".%06" PRIu32, tt.seconds, tt.microseconds);
		// sleep_ms(1000);
		// printf("foo\n");
	}
	return 0;
}