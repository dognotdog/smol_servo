
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
#include <inttypes.h>
#include <math.h>

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

/**
 * Current sense zero calibration, the first phase of the servo loop start.
 *
 * The TMC6200 amplifier offset scales with the gain, so a new calibration is necessary at each power-up and after each gain change (datasheet 6.2). DRV_EN must be low: then no current flows, and each ADC sample is the zero of its channel.
 *
 * Core 1 (loop IRQ) owns the sums. It adds the difference to VSENSE_NOMINAL_ZERO_V, so that the float sums keep their resolution. Core 0 sets a request flag, and reads the sums only after the done flag.
 * There are 6 zeros: 3 phases, each with the counter 0 sample (high-side on time) and the TOP sample (low-side on time).
 */
// loops to skip after the request, for the ADC and amplifier to settle (10 ms)
#define ZERO_CAL_SKIP_LOOPS (200u)
// loops to average (approx. 205 ms at 20 kHz)
#define ZERO_CAL_SAMPLE_LOOPS (4096u)
// permitted amplifier input offset. The datasheet (6, 6.2.1) gives "a few to a few 10mV" for VINOFS. The output offset is VINOFS times the gain, so the permitted difference of a zero to VSENSE_NOMINAL_ZERO_V is this value times the gain.
#define ZERO_CAL_MAX_INPUT_OFFSET_V (0.05f)
// permitted standard deviation of the samples of one zero
#define ZERO_CAL_MAX_SIGMA_V (0.02f)

typedef enum {
	ZERO_CAL_IDLE = 0,
	ZERO_CAL_BUSY,
	ZERO_CAL_OK,
	ZERO_CAL_FAILED,
} zero_cal_state_t;

static bool _loop_started = false;
// core 0 writes true, core 1 writes false
static volatile bool _zero_cal_request = false;
// core 1 writes true, core 0 writes false
static volatile bool _zero_cal_done = false;
// core 1 only
static bool _zero_cal_active = false;
static uint32_t _zero_cal_loops = 0;
static float _zero_cal_sum[3][2];
static float _zero_cal_sum_sq[3][2];
// core 0 only
static zero_cal_state_t _zero_cal_state = ZERO_CAL_IDLE;
static float _isense_zero_v[3][2];

static inline void __not_in_flash_func(_zero_cal_accumulate)(const float values[3][2]) {
	if (_zero_cal_request) {
		for (size_t i = 0; i < 3; ++i) {
			for (size_t j = 0; j < 2; ++j) {
				_zero_cal_sum[i][j] = 0.0f;
				_zero_cal_sum_sq[i][j] = 0.0f;
			}
		}
		_zero_cal_loops = 0;
		_zero_cal_active = true;
		_zero_cal_request = false;
		return;
	}
	if (!_zero_cal_active)
		return;

	uint32_t n = _zero_cal_loops++;
	if (n < ZERO_CAL_SKIP_LOOPS)
		return;

	for (size_t i = 0; i < 3; ++i) {
		for (size_t j = 0; j < 2; ++j) {
			float d = values[i][j] - VSENSE_NOMINAL_ZERO_V;
			_zero_cal_sum[i][j] += d;
			_zero_cal_sum_sq[i][j] += d * d;
		}
	}

	if (n + 1 == ZERO_CAL_SKIP_LOOPS + ZERO_CAL_SAMPLE_LOOPS) {
		_zero_cal_active = false;
		__dmb();
		_zero_cal_done = true;
	}
}

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
	 * Thus, we have a 5 PWM cycle latency from sampling sensor inputs to applying PWM signals.
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

	_zero_cal_accumulate(isense_values);



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
	smol_irq_init();
}

/**
 * Request a new current sense zero calibration from core 1. DRV_EN must be low.
 * Returns 0 when the request is sent.
 */
static int _zero_cal_start(void) {
	if (smol_drv_is_enabled()) {
		err_println("isense zero calibration: the TMC6200 is enabled, set DRV_EN low first");
		_zero_cal_state = ZERO_CAL_FAILED;
		return -1;
	}
	_zero_cal_state = ZERO_CAL_BUSY;
	_zero_cal_done = false;
	__dmb();
	_zero_cal_request = true;
	info_println("isense zero calibration: start, gain %.0f, %u samples", smol_drv_isense_gain(), ZERO_CAL_SAMPLE_LOOPS);
	return 0;
}

/**
 * Core 0: evaluate the sums from core 1 when the calibration is complete.
 */
static void _zero_cal_run(void) {
	if ((_zero_cal_state != ZERO_CAL_BUSY) || !_zero_cal_done)
		return;
	__dmb();
	_zero_cal_done = false;

	const float n = (float)ZERO_CAL_SAMPLE_LOOPS;
	const float gain = smol_drv_isense_gain();
	const float max_offset_v = ZERO_CAL_MAX_INPUT_OFFSET_V * gain;
	bool ok = true;
	float zero[3][2];
	for (size_t i = 0; i < 3; ++i) {
		float sigma[2];
		for (size_t j = 0; j < 2; ++j) {
			float mean_d = _zero_cal_sum[i][j] / n;
			float var = _zero_cal_sum_sq[i][j] / n - mean_d * mean_d;
			sigma[j] = sqrtf(fmaxf(var, 0.0f));
			zero[i][j] = VSENSE_NOMINAL_ZERO_V + mean_d;
			if ((fabsf(mean_d) > max_offset_v) || (sigma[j] > ZERO_CAL_MAX_SIGMA_V))
				ok = false;
		}
		// input offset: the output offset divided by the gain
		info_println("isense zero phase %u: counter 0 %.4f V (sigma %.4f V), TOP %.4f V (sigma %.4f V), input offset %.1f mV",
			(unsigned)i, zero[i][0], sigma[0], zero[i][1], sigma[1], 1000.0f * (0.5f * (zero[i][0] + zero[i][1]) - VSENSE_NOMINAL_ZERO_V) / gain);
	}

	if (!ok) {
		_zero_cal_state = ZERO_CAL_FAILED;
		err_println("isense zero calibration: failed, permitted %.4f V +- %.3f V (input offset %.0f mV x gain %.0f), sigma max %.3f V",
			VSENSE_NOMINAL_ZERO_V, max_offset_v, 1000.0f * ZERO_CAL_MAX_INPUT_OFFSET_V, gain, ZERO_CAL_MAX_SIGMA_V);
		return;
	}
	for (size_t i = 0; i < 3; ++i)
		for (size_t j = 0; j < 2; ++j)
			_isense_zero_v[i][j] = zero[i][j];
	_zero_cal_state = ZERO_CAL_OK;
	info_println("isense zero calibration: done");
}

bool smol_servo_loop_isense_zero_is_busy(void) {
	return _zero_cal_state == ZERO_CAL_BUSY;
}

bool smol_servo_loop_isense_zero(float zero_v[3][2]) {
	if (_zero_cal_state != ZERO_CAL_OK)
		return false;
	for (size_t i = 0; i < 3; ++i)
		for (size_t j = 0; j < 2; ++j)
			zero_v[i][j] = _isense_zero_v[i][j];
	return true;
}

void smol_servo_loop_isense_gain_changed(void) {
	if (_zero_cal_state == ZERO_CAL_IDLE)
		return;
	// the zeros are not valid for the new gain
	_zero_cal_state = ZERO_CAL_IDLE;
	if (_loop_started)
		_zero_cal_start();
}

/**
 * Start the servo loop. The first phase is the current sense zero calibration, with DRV_EN low. A second call does only a new zero calibration.
 */
void smol_servo_loop_start(void) {
	if (smol_drv_is_enabled()) {
		err_println("servo loop start: the TMC6200 is enabled, set DRV_EN low first");
		return;
	}
	if (_loop_started) {
		_zero_cal_start();
		return;
	}
	// the request is set before the start, so that the first loop IRQ starts the calibration
	if (_zero_cal_start() != 0)
		return;
	smol_mag_start();
	smol_pwm_start();
	smol_adc_start();
	_loop_started = true;
}

static uint64_t t_idle = 0;

void __not_in_flash_func(smol_servo_loop_idle_core1)(void) {
	// our idle function on core 1 for when there's nothing else to do.
	// NOTE: do not use functions that use long critical sections disabling interrupts, like sleep_ms()! In fact, everything here should be RAM resident to avoid XIP stalls and thus IRQ jitter.
	tight_loop_contents();
	return;
}

void smol_servo_loop_idle_core0(void) {
	_zero_cal_run();

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

