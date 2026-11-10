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
#include "hardware/structs/powman.h"
#include "hardware/structs/watchdog.h"

#include "smol_servo.h"
#include "smol_fs.h"
#include "smol_config.h"
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

// set by core 1 after its init: from then on core 1 has no XIP access, and core 0 can write the flash
static volatile bool _core1_ram_only = false;

bool smol_core1_is_ram_only(void) {
	return _core1_ram_only;
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
	__dmb();
	_core1_ram_only = true;

	while (1) {
		smol_servo_loop_idle_core1();
	}
}

int main(int argc, char const *argv[])
{
	smol_hires_time_init();

    stdio_init_all();
    pico_led_init();

    // Make the SPI pins available to picotool
    bi_decl(bi_3pins_with_func(DRV_MISO_PIN, DRV_MOSI_PIN, DRV_SLCK_PIN, GPIO_FUNC_SPI));
    // Make the CS pin available to picotool
    bi_decl(bi_1pin_with_name(DRV_NSS_PIN, "DRV_SPI CS"));

    tmc6200_spi_init(DRV_SPI, DRV_MOSI_PIN, DRV_NSS_PIN, DRV_SLCK_PIN, DRV_MISO_PIN);
	// TMC6200 registers with DRV_EN low, no log output (approx. 0.5ms). The enable waits for the PD contract.
	smol_drv_init();

	// USB PD before all log output and slow init: VBUS is the only power supply. If the sink does not reply to Source_Capabilities in time, the Source sends Hard Reset and removes VBUS, and the board starts again.
	i2c_fusb_init();
	fusb302_init(FUSB_I2C, FUSB_INT_PIN);
	smol_usb_pd_init();
	smol_usb_pd_run_startup();

	// reset cause of this boot: POWMAN CHIP_RESET has the HAD_* flags (HAD_POR = power-on reset), WATCHDOG REASON tells a watchdog reset
	info_println("reset cause: CHIP_RESET 0x%08x, WATCHDOG REASON 0x%x", powman_hw->chip_reset, watchdog_hw->reason);
	smol_drv_log_status();

	// mount only: a format erases flash, and core 0 then cannot answer PD messages. smol_fs_run() does a necessary format later.
	smol_fs_init();
	// config file: applies the current sense gain to the TMC6200 (DRV_EN is still low). Without a mounted FS, smol_fs_run() loads the config after the deferred format.
	if (smol_fs_is_mounted())
		smol_config_load();

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
		smol_drv_run();
		smol_fs_run();

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