
#include "smol_servo.h"
#include "smol_servo_parameters.h"
#include "smol_fast.h"
#include "pico_debug.h"

#include "hardware/dma.h"
#include "hardware/irq.h"
#include "hardware/exception.h"
#include "hardware/pwm.h"
#include "pico/stdlib.h"
#include "pico/platform.h"

#include <stdint.h>
#include <assert.h>

static uint32_t _loop_irq_number = -1;

static int _timer1_sample_dma_channel = -1;
static uint32_t _timer1_sample_dma_buf[1] __aligned(4);


static inline void irq_set_pending_fast(uint num) {
    nvic_hw->ispr[num/32] = 1 << (num % 32);
}

static inline bool irq_get_pending_fast(uint num) {
	return 0 != nvic_hw->ispr[num/32] & 1 << (num % 32);
}

void __not_in_flash_func(smol_servo_signal_loop)(void) {
	assert_fast(_loop_irq_number != -1);
	irq_set_pending_fast(_loop_irq_number);
}

static uint32_t _bridge_pwm_values[3][3];

void __not_in_flash_func(smol_servo_loop_irq_handler)(void) {
#ifdef DEBUG_LOOP_IRQ_WITH_GPIO
	gpio_out_put_fast(DEBUG_LOOP_IRQ_WITH_GPIO, 1);
#endif
	// only execute on core 1
	assert_fast(get_core_num() == 1);
	irq_clear(_loop_irq_number);

	/**
	 * Fill previously computed PWM values with some assumptions about latency:
	 *        0     1     2     3     4     5     6     7 ...
	 * PHASE: 0     1     2     0     1     2     0     1
	 * PWM:   |w    |w    |w    |w    |w    |w    |w    |w
	 * LOOP:   |w------|         |w------|         |w------|
	 * 
	 * The LOOP might execute up to almost the end of the 3rd phase, and PWM values will be latched in an interrupt callback, so the callback at PH[0] will latch values for PH[1]. Since the PWM IRQ is higher priority than the servo loop, the  next value to apply that is written out at the beginning of LOOP is for PH[2].
	 * 
	 * Thus, we have a 5 PWM cycle latency from sampling sensor inputs to applying PWM signals
	 */

	smol_pwm_fill(_bridge_pwm_values, 2);

	/**
	 * There are a few things to sample at the beginning of the loop to have available.
	 * - `now_clocks` current PWM wrap sytem timestamp
	 * - magnetic encoder reads
	 * - current sensor reads
	 */

	// ADC values are at highest sample rate, so do that first
	float isense_values[3][2];
	uint32_t isense_times_clocks[3][2];
	smol_adc_get_isense_values(isense_values, isense_times_clocks);

	uint32_t now_clocks = _timer1_sample_dma_buf[0];



	uint32_t mag_reading_values[2];
	int32_t mag_reading_ages_ns[2];
	smol_mag_get_reading_values(mag_reading_values);
	smol_mag_get_reading_ages_ns(mag_reading_ages_ns);



	/**
	 * This is our core servo loop. We need to figure out what the next set of 3 PWM cycles should look like.
	 */



	// for sanity checking, we could check if the IRQ is pending again by the end of the function. If it is, we've taken too long and cannot keep up with the hardware trigger rate.
	assert_fast(0 == irq_get_pending_fast(_loop_irq_number));
	// assert(0 == NVIC_GetPendingIRQ(_loop_irq_number));
#ifdef DEBUG_LOOP_IRQ_WITH_GPIO
	gpio_out_put_fast(DEBUG_LOOP_IRQ_WITH_GPIO, 0);
#endif
}


void smol_irq_init(void) {
	assert(get_core_num() == 1);

	_loop_irq_number = user_irq_claim_unused(true);

	irq_set_exclusive_handler(_loop_irq_number, smol_servo_loop_irq_handler);

	irq_set_priority(ADC_IRQ, SMOL_ADC_IRQ_PRIORITY);
	irq_set_priority(BRIDGE_PWM_WRAP_IRQ, SMOL_PWM_IRQ_PRIORITY);
	irq_set_priority(_loop_irq_number, SMOL_LOOP_IRQ_PRIORITY);

	irq_set_enabled(_loop_irq_number, true);
	irq_set_enabled(ADC_IRQ, true);
	irq_set_enabled(BRIDGE_PWM_WRAP_IRQ, true);

}

static void __not_in_flash_func(_exception_dummy_handler)(void) {
 assert_fast(0);
}

static void _timer1_sample_dma_init(void) {
	assert(get_core_num() == 1);

	/**
	 * Ok, so how do we actually read that 64 bit timer correctly from an interrupt?
	 * 
	 * The registers come in [H,L] pairs, but we have to read the L registers first. To make this work via DMA, we thus have to have a READ RING config. However, that leaves us with having to reset the write address after every transfer.
	 * 
	 * OR... we only use DMA for the lower 32 bits. At 150MHz, the wrap around time is ~28s which is plenty for short term deltas, and additionally we can always reconstruct the full time with the help of a close software 64bit read.
	 */

	_timer1_sample_dma_channel = dma_claim_unused_channel(true);

	dma_channel_config_t config = dma_channel_get_default_config(_timer1_sample_dma_channel);

	channel_config_set_transfer_data_size(&config, DMA_SIZE_32);
	channel_config_set_read_increment(&config, false);
	channel_config_set_write_increment(&config, false);
	channel_config_set_dreq(&config, pwm_get_dreq(PWMA_SLICE));
	// do not chain to other channel
	channel_config_set_chain_to(&config, _timer1_sample_dma_channel);
	channel_config_set_enable(&config, true);
	channel_config_set_high_priority(&config, true);

    dma_channel_configure(
    	_timer1_sample_dma_channel, 
    	&config, 
    	_timer1_sample_dma_buf, 
    	&timer1_hw->timerawl,
    	dma_encode_endless_transfer_count(),
    	true
    );

}

void smol_servo_loop_init(void) {

	for (size_t i = 0; i < PICO_NUM_VTABLE_IRQS; ++i) {
		int prio = irq_get_priority(i);
		assert(prio > 1);
	}

	// NOTE: initially this was for debugging, now it's for catching XIP accesses post init.
	for (size_t i = MIN_EXCEPTION_NUM; i < MAX_EXCEPTION_NUM; ++i) {
		exception_set_exclusive_handler(i, _exception_dummy_handler);
		// int prio = exception_get_priority(i);
		// assert(prio > 1);
	}

	_timer1_sample_dma_init();

	smol_mag_init();
	smol_adc_init();
	smol_pwm_init();
	smol_drv_init();
	smol_irq_init();
}

void smol_servo_loop_start(void) {
    smol_mag_start();
	smol_pwm_start();
	smol_adc_start();
}

static uint64_t t_idle = 0;

void __not_in_flash_func(smol_servo_loop_idle_core1)(void) {
	// our idle function on core 1 for when there's nothing else to do.
	// NOTE: do not use functions that use long critical sections disabling interrupts, like sleep_ms()! In fact, everything here should be RAM resident to avoid XIP stalls and thus IRQ jitter.
	tight_loop_contents();
	return;
}

void smol_servo_loop_idle_core0(void) {
	uint64_t t_fast = smol_time64(timer0_hw);
	dbg_time_t tt = {.seconds = t_fast / 1000000u, .microseconds = t_fast % 1000000u};
	if (true && (t_fast - t_idle > 1000000ull)) {
		t_idle = t_fast;
		dbg_println("bar %" PRIu32 ".%06" PRIu32, tt.seconds, tt.microseconds);
		// dbg_println("adc block %" PRIu32, smol_adc_block_index());
		// dbg_println("adc pwm counter %" PRIu32, pwm_get_counter(PWM_ADC_SLICE));
		dbg_println("t_chip %.3f", smol_chip_temperature_celsius());
		dbg_println("vbus %.3f, vdd %.3f", smol_adc_vbus(), smol_adc_vdd());

		// for (size_t i = 0; i < NUM_ADC_CHANNELS; ++i) {
		// 	if (i < 9)
		// 		dbg_println("v[%u] %.3f %.3f", i, smol_adc_last_voltage(i), smol_adc_last_voltage2(i));
		// 	else
		// 		dbg_println("v[%u] %.3f", i, smol_adc_last_voltage(i));
		// }

		// sleep_ms(300);
	}
}

