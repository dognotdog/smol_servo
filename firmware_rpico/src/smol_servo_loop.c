
#include "smol_servo.h"
#include "smol_servo_parameters.h"
#include "smol_fast.h"
#include "pico_debug.h"
#include "smol_servo_loop.h"
#include "smol_motor_id.h"
#include "smol_isense_cal.h"

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
#include <string.h>

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

/**
 * Loop state, see smol_servo_loop.h.
 * - core 0 writes _req_state (smol_servo_loop_request_state).
 * - core 1 writes _state. It takes _req_state when _req_state changes. It also advances _state itself: ZERO_CAL to ZERO_CAL_DONE, and a bridge state to FAULT.
 * A request of the same state again has no effect. Core 0 requests OFF after each ZERO_CAL_DONE and FAULT.
 */
static volatile smol_loop_state_t _req_state = SMOL_LOOP_OFF;
static volatile smol_loop_state_t _state = SMOL_LOOP_OFF;
// core 1 only: the last request that core 1 took
static smol_loop_state_t _req_taken = SMOL_LOOP_OFF;
// overcurrent values for the log, core 1 writes them before FAULT
static volatile uint32_t _fault_phase = 0;
static volatile float _fault_a = 0.0f;

// states that switch the bridge
static bool __not_in_flash_func(_switches_bridge)(smol_loop_state_t state) {
	return (state == SMOL_LOOP_MOTOR_ID) || (state == SMOL_LOOP_CONTROL) || (state == SMOL_LOOP_PWM_TEST);
}

// outputs for the next loop. Zero: all half bridges off.
static smol_bridge_out_t _bridge_out;
// _bridge_out as PWM CC buffer words (smol_pwm_prepare at the loop end). Zero: all half bridges off.
static smol_bridge_cc_t _bridge_cc;

// current sense calibration. Core 0 writes it only while no bridge state runs (a bridge state needs a valid calibration).
static float _cal_zero_v[SMOL_SERVO_PHASES][2];
static float _cal_amps_per_volt = 0.0f;
static volatile bool _cal_valid = false;

static bool _loop_started = false;

// test delay at the loop end, in system clocks (smol_servo_loop_set_end_delay). Core 0 writes it, core 1 reads it.
static volatile uint32_t _end_delay_clocks = 0;
// loop end after the wrap timestamp, in system clocks, in MOTOR_ID. Core 1 writes them, core 0 sets _end_stat_reset.
static volatile bool _end_stat_reset = true;
static volatile uint32_t _end_min, _end_max, _end_n;
static volatile uint64_t _end_sum;

/**
 * Current sense offset table (smol_isense_cal.h) in MOTOR_ID, not in the calibration sweep (id_cal measures the table).
 * The table value depends on the cc of the PWM period of the sample. The ADC samples phase k in bridge PWM period k of the last 3 periods before the loop IRQ, and the output latency is 5 periods (notes/servo-loop-design.md 3.1, 3.3). Thus in loop m:
 * - phase 0 is sampled in the period of output (m - 3, k 1), phase 1 in (m - 3, k 2), phase 2 in (m - 2, k 0). Sample 1 (TOP) uses the cc of this period.
 * - sample 0 (counter 0) uses the cc of the period before (the rising edge is in that period): (m - 3, k p) for phase p.
 * The pulse test agrees: a pulse in period k 1 of the output for segment s shows in the samples of segment s + 2, in phase 0 at TOP and in phase 1 at counter 0 of the next period.
 * _cc_hist[0] is the output of loop m - 1, [1] of m - 2, [2] of m - 3. ISENSE_CC_OFF: the half bridge is off (no table value, the terminal floats).
 */
#define ISENSE_CC_OFF (0xffffu)
static uint16_t _cc_hist[3][SMOL_SERVO_PWM_PER_LOOP][SMOL_SERVO_PHASES];
static const uint8_t __not_in_flash("loop") _s1_hist[SMOL_SERVO_PHASES] = {2, 2, 1};
static const uint8_t __not_in_flash("loop") _s1_k[SMOL_SERVO_PHASES] = {1, 2, 0};

// table value in A for phase p, sample j of this loop, or 0 if the half bridge is off or the cc is outside the table
static float __not_in_flash_func(_isense_table_a)(size_t p, size_t j) {
	uint16_t cc1 = _cc_hist[_s1_hist[p]][_s1_k[p]][p];
	uint16_t cc0 = _cc_hist[2][p][p];
	if ((cc1 == ISENSE_CC_OFF) || (cc0 == ISENSE_CC_OFF))
		return 0.0f;
	uint16_t cc = (j == 0) ? cc0 : cc1;
	if (!smol_isense_cal_in_range(cc))
		return 0.0f;
	return 1e-3f * (float)smol_isense_cal_e_ma((uint32_t)p, (uint32_t)j, cc);
}

static void __not_in_flash_func(_cc_hist_push)(const smol_bridge_cc_t* cc) {
	for (size_t k = 0; k < SMOL_SERVO_PWM_PER_LOOP; ++k) {
		for (size_t p = 0; p < SMOL_SERVO_PHASES; ++p) {
			uint32_t w = cc->word[k][p];
			_cc_hist[2][k][p] = _cc_hist[1][k][p];
			_cc_hist[1][k][p] = _cc_hist[0][k][p];
			// enable cc in the high half: zero if the half bridge is off
			_cc_hist[0][k][p] = (w >> 16) ? (uint16_t)(w & 0xffffu) : ISENSE_CC_OFF;
		}
	}
}

/**
 * Current sense zero calibration (state ZERO_CAL).
 *
 * The TMC6200 amplifier offset scales with the gain, so a new calibration is necessary at each power-up and after each gain change (datasheet 6.2). DRV_EN must be low: then no current flows, and each ADC sample is the zero of its channel.
 * Core 0 clears the sums and the loop count, then requests ZERO_CAL. Core 1 skips some loops for the ADC and the amplifier to settle, then adds up the 6 samples (3 phases, counter 0 and TOP). It adds the difference to VSENSE_NOMINAL_ZERO_V, so that the float sums keep their resolution. After ZERO_CAL_LOOPS, core 1 goes to ZERO_CAL_DONE. Core 0 then evaluates the sums (_zero_cal_evaluate) and requests OFF.
 */
// loops to skip at the start (10 ms)
#define ZERO_CAL_SKIP_LOOPS (200u)
// loops to average (approx. 205 ms)
#define ZERO_CAL_SAMPLE_LOOPS (4096u)
#define ZERO_CAL_LOOPS (ZERO_CAL_SKIP_LOOPS + ZERO_CAL_SAMPLE_LOOPS)
// permitted amplifier input offset. The datasheet (6, 6.2.1) gives "a few to a few 10mV" for VINOFS. The output offset is VINOFS times the gain.
#define ZERO_CAL_MAX_INPUT_OFFSET_V (0.05f)
// permitted standard deviation of the samples of one zero
#define ZERO_CAL_MAX_SIGMA_V (0.02f)

// core 0 clears them in OFF, core 1 writes them in ZERO_CAL
static float _zero_sum[SMOL_SERVO_PHASES][2];
static float _zero_sum_sq[SMOL_SERVO_PHASES][2];
static uint32_t _zero_cal_loops = 0;

/**
 * One loop of ZERO_CAL, core 1. Returns true when the sums are complete.
 */
static bool __not_in_flash_func(_zero_cal_step)(const float isense_values[SMOL_SERVO_PHASES][2]) {
	uint32_t n = _zero_cal_loops;
	if (n >= ZERO_CAL_SKIP_LOOPS) {
		for (size_t p = 0; p < SMOL_SERVO_PHASES; ++p) {
			for (size_t j = 0; j < 2; ++j) {
				float d = isense_values[p][j] - VSENSE_NOMINAL_ZERO_V;
				_zero_sum[p][j] += d;
				_zero_sum_sq[p][j] += d * d;
			}
		}
	}
	_zero_cal_loops = n + 1;
	return n + 1 >= ZERO_CAL_LOOPS;
}

/**
 * Bridge PWM test pattern (state PWM_TEST), for a scope on the PWM and EN pins. All 3 half bridges switch with the same duty (no differential voltage), 94/88/82 % in the 3 PWM periods of a loop. It does not need the TMC6200 enabled. With DRV_EN high, the bridge outputs switch, too.
 */
static const float __not_in_flash("loop") _pwm_test_duty[SMOL_SERVO_PWM_PER_LOOP] = {0.94f, 0.88f, 0.82f};

/**
 * Overcurrent check with the calibration from core 0. Positive current flows into the motor terminal.
 * A switching spike can affect one of the 2 samples of a phase, so the check uses the smaller of the 2 magnitudes.
 */
static bool __not_in_flash_func(_overcurrent)(const float isense_values[SMOL_SERVO_PHASES][2]) {
	if (!_cal_valid)
		return false;
	for (size_t p = 0; p < SMOL_SERVO_PHASES; ++p) {
		float a0 = (isense_values[p][0] - _cal_zero_v[p][0]) * _cal_amps_per_volt;
		float a1 = (isense_values[p][1] - _cal_zero_v[p][1]) * _cal_amps_per_volt;
		a0 = (a0 < 0.0f) ? -a0 : a0;
		a1 = (a1 < 0.0f) ? -a1 : a1;
		float a = (a0 < a1) ? a0 : a1;
		if (a > SMOL_ISENSE_FAULT_A) {
			_fault_phase = p;
			_fault_a = a;
			return true;
		}
	}
	return false;
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

	smol_pwm_fill(&_bridge_cc, 2);

	/**
	 * There are a few things to sample at the beginning of the loop to have available.
	 * - `now_clocks` current PWM wrap sytem timestamp
	 * - magnetic encoder reads
	 * - current sensor reads
	 */

	// ADC values are at highest sample rate, so do that first
	float isense_values[SMOL_SERVO_PHASES][2];
	uint32_t isense_times_clocks[SMOL_SERVO_PHASES][2];
	smol_adc_get_isense_values(isense_values, isense_times_clocks);

	uint32_t now_clocks = _timer1_sample_dma_buf[0];

	uint32_t mag_reading_values[2];
	int32_t mag_reading_ages_ns[2];
	smol_mag_get_reading_values(mag_reading_values);
	smol_mag_get_reading_ages_ns(mag_reading_ages_ns);

	/**
	 * This is our core servo loop. We need to figure out what the next set of 3 PWM cycles should look like.
	 * The state (calibration, identification, control) calculates _bridge_out. The end of this loop converts it to _bridge_cc (smol_pwm_prepare), and the next loop copies that to the PWM with smol_pwm_fill().
	 */
	smol_loop_state_t req = _req_state;
	if (req != _req_taken) {
		_req_taken = req;
		_state = req;
	}

	smol_loop_state_t state = _state;
	switch (state) {
		case SMOL_LOOP_ZERO_CAL:
			smol_bridge_out_off(&_bridge_out);
			if (_zero_cal_step(isense_values)) {
				__dmb(); // the non-volatile sums must be in memory before core 0 sees ZERO_CAL_DONE and reads them
				_state = SMOL_LOOP_ZERO_CAL_DONE;
			}
			break;
		case SMOL_LOOP_MOTOR_ID: {
			float amps[SMOL_SERVO_PHASES][2];
			bool table = smol_isense_cal_valid() && smol_motor_id_uses_isense_cal();
			for (size_t p = 0; p < SMOL_SERVO_PHASES; ++p) {
				for (size_t j = 0; j < 2; ++j) {
					amps[p][j] = (isense_values[p][j] - _cal_zero_v[p][j]) * _cal_amps_per_volt;
					if (table)
						amps[p][j] -= _isense_table_a(p, j);
				}
			}
			smol_motor_id_step_t r = smol_motor_id_step(amps, &_bridge_out);
			if (r != SMOL_MOTOR_ID_RUN) {
				// an abort switches the bridge off at once, a normal end in the next loop
				if (r == SMOL_MOTOR_ID_ABORT)
					smol_pwm_all_off();
				__dmb(); // the sums must be in memory before core 0 sees MOTOR_ID_DONE and reads them
				_state = SMOL_LOOP_MOTOR_ID_DONE;
			}
			break;
		}
		case SMOL_LOOP_CONTROL:
			// TODO: closed loop control
			smol_bridge_out_off(&_bridge_out);
			break;
		case SMOL_LOOP_PWM_TEST:
			for (size_t k = 0; k < SMOL_SERVO_PWM_PER_LOOP; ++k) {
				for (size_t p = 0; p < SMOL_SERVO_PHASES; ++p) {
					_bridge_out.duty[k][p] = _pwm_test_duty[k];
					_bridge_out.enable[k][p] = true;
				}
			}
			break;
		case SMOL_LOOP_OFF:
		case SMOL_LOOP_ZERO_CAL_DONE:
		case SMOL_LOOP_MOTOR_ID_DONE:
		case SMOL_LOOP_FAULT:
		default:
			smol_bridge_out_off(&_bridge_out);
			break;
	}

	/**
	 * Overcurrent in the states that switch the bridge: bridge off, state FAULT.
	 * Not in ZERO_CAL: the bridge is off, and core 0 writes the calibration at the end of ZERO_CAL. Thus core 1 never reads the calibration while core 0 writes it, and the read needs no barrier.
	 */
	if (_switches_bridge(state) && _overcurrent(isense_values)) {
		smol_pwm_all_off();
		smol_bridge_out_off(&_bridge_out);
		// the fault values and _state are volatile, thus the compiler keeps their order. Core 0 uses the values only for the log.
		_state = SMOL_LOOP_FAULT;
	}

	// after all changes of _bridge_out: the CC words for the fill of the next loop, and the cc for the current sense table
	smol_pwm_prepare(&_bridge_out, &_bridge_cc);
	_cc_hist_push(&_bridge_cc);

	uint32_t end_delay = _end_delay_clocks;
	if (end_delay > 0)
		busy_wait_at_least_cycles(end_delay);
	if (_end_stat_reset) {
		_end_min = UINT32_MAX;
		_end_max = 0;
		_end_n = 0;
		_end_sum = 0;
		_end_stat_reset = false;
	}
	if (state == SMOL_LOOP_MOTOR_ID) {
		uint32_t dt_end = smol_time32(timer1_hw) - now_clocks;
		if (dt_end < _end_min)
			_end_min = dt_end;
		if (dt_end > _end_max)
			_end_max = dt_end;
		_end_sum += dt_end;
		_end_n += 1;
	}

	// for sanity checking, we could check if the IRQ is pending again by the end of the function. If it is, we've taken too long and cannot keep up with the hardware trigger rate.
	assert_fast(0 == irq_get_pending_fast(_loop_irq_number));
	// assert(0 == NVIC_GetPendingIRQ(_loop_irq_number));
#ifdef DEBUG_LOOP_IRQ_WITH_GPIO
	gpio_out_put_fast(DEBUG_LOOP_IRQ_WITH_GPIO, 0);
#endif
}


int smol_servo_loop_set_end_delay(uint32_t clocks) {
	if (clocks > SMOL_SERVO_LOOP_END_DELAY_MAX_CLOCKS)
		return -1;
	_end_delay_clocks = clocks;
	_end_stat_reset = true;
	smol_adc_irq_time_reset();
	return 0;
}

void smol_servo_loop_print_end_delay(void) {
	uint32_t n = _end_n;
	if (n == 0) {
		info_println("loop end delay %" PRIu32 " clocks, no MOTOR_ID loops", _end_delay_clocks);
		return;
	}
	float k = 1e6f / (float)SMOL_SERVO_LOOP_FREQ_HZ / (float)SMOL_SERVO_LOOP_PERIOD_CLOCKS;
	info_println("loop end delay %" PRIu32 " clocks, loop end after the wrap: min %.3f mean %.3f max %.3f us (%" PRIu32 " loops)", _end_delay_clocks, k * (float)_end_min, k * (float)((double)_end_sum / (double)n), k * (float)_end_max, n);
	smol_adc_print_irq_timing();
}

uint32_t __not_in_flash_func(smol_servo_loop_wrap_clocks)(void) {
	return *(volatile uint32_t*)&_timer1_sample_dma_buf[0];
}

uint32_t smol_servo_loop_end_delay(void) {
	return _end_delay_clocks;
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

#ifdef DEBUG_LOOP_IRQ_WITH_GPIO
	// output for the loop IRQ timing, see smol_servo_loop_irq_handler()
	gpio_init(DEBUG_LOOP_IRQ_WITH_GPIO);
	gpio_set_dir(DEBUG_LOOP_IRQ_WITH_GPIO, true);
#endif

	smol_mag_init();
	smol_adc_init();
	smol_pwm_init();
	smol_irq_init();
}

/**
 * Core 0 part
 */
static const char* const _state_names[SMOL_LOOP_NUM_STATES] = {
	[SMOL_LOOP_OFF] = "off",
	[SMOL_LOOP_ZERO_CAL] = "zero_cal",
	[SMOL_LOOP_ZERO_CAL_DONE] = "zero_cal_done",
	[SMOL_LOOP_MOTOR_ID] = "motor_id",
	[SMOL_LOOP_MOTOR_ID_DONE] = "motor_id_done",
	[SMOL_LOOP_CONTROL] = "control",
	[SMOL_LOOP_PWM_TEST] = "pwm_test",
	[SMOL_LOOP_FAULT] = "fault",
};

// states that need the TMC6200 enabled and a valid zero calibration
static bool _drives_bridge(smol_loop_state_t state) {
	return (state == SMOL_LOOP_MOTOR_ID) || (state == SMOL_LOOP_CONTROL);
}

const char* smol_servo_loop_state_name(smol_loop_state_t state) {
	return (state < SMOL_LOOP_NUM_STATES) ? _state_names[state] : "?";
}

smol_loop_state_t smol_servo_loop_state_from_name(const char* name) {
	for (size_t i = 0; i < SMOL_LOOP_NUM_STATES; ++i) {
		if (0 == strcmp(_state_names[i], name))
			return (smol_loop_state_t)i;
	}
	return SMOL_LOOP_NUM_STATES;
}

// in RAM: core 1 code can call it, too
smol_loop_state_t __not_in_flash_func(smol_servo_loop_state)(void) {
	return _state;
}

int smol_servo_loop_request_state(smol_loop_state_t state) {
	const char* name = smol_servo_loop_state_name(state);
	if (!_loop_started) {
		err_println("loop state %s: the servo loop has not started yet (it starts at boot)", name);
		return -1;
	}
	if ((state >= SMOL_LOOP_NUM_STATES) || (state == SMOL_LOOP_ZERO_CAL_DONE) || (state == SMOL_LOOP_MOTOR_ID_DONE) || (state == SMOL_LOOP_FAULT)) {
		err_println("loop state %s: not possible, only the loop sets it", name);
		return -1;
	}
	if ((_state == SMOL_LOOP_FAULT) && (state != SMOL_LOOP_OFF)) {
		err_println("loop state %s: fault, request off first (stop)", name);
		return -1;
	}
	if ((state == SMOL_LOOP_ZERO_CAL) && smol_drv_is_enabled()) {
		err_println("loop state %s: the TMC6200 is enabled, set DRV_EN low first", name);
		return -1;
	}
	if (_drives_bridge(state)) {
		if (!smol_drv_is_enabled()) {
			err_println("loop state %s: the TMC6200 is not enabled (drv_en)", name);
			return -1;
		}
		if (!_cal_valid) {
			err_println("loop state %s: no valid current sense zero calibration", name);
			return -1;
		}
	}
	if (state == SMOL_LOOP_ZERO_CAL) {
		// OFF for 2 loops, then clear the calibration data. No bridge state runs: the TMC6200 is disabled.
		_req_state = SMOL_LOOP_OFF;
		busy_wait_us_32(2 * 1000000 / SMOL_SERVO_LOOP_FREQ_HZ);
		_cal_valid = false;
		for (size_t p = 0; p < SMOL_SERVO_PHASES; ++p) {
			for (size_t j = 0; j < 2; ++j) {
				_zero_sum[p][j] = 0.0f;
				_zero_sum_sq[p][j] = 0.0f;
			}
		}
		_zero_cal_loops = 0;
		__dmb(); // the cleared non-volatile sums and count must be in memory before core 1 sees the ZERO_CAL request
		info_println("isense zero calibration: start, gain %.0f, %u samples", smol_drv_isense_gain(), ZERO_CAL_SAMPLE_LOOPS);
	}
	_req_state = state;
	dbg_println("loop state request %s", name);
	return 0;
}

/**
 * Zero calibration, core 0 part: mean and standard deviation of the sums, limit check, then the calibration for the loop.
 */
static void _zero_cal_evaluate(void) {
	const float n = (float)ZERO_CAL_SAMPLE_LOOPS;
	const float gain = smol_drv_isense_gain();
	const float max_offset_v = ZERO_CAL_MAX_INPUT_OFFSET_V * gain;
	bool ok = true;
	float zero[SMOL_SERVO_PHASES][2];
	for (size_t p = 0; p < SMOL_SERVO_PHASES; ++p) {
		float sigma[2];
		for (size_t j = 0; j < 2; ++j) {
			float mean_d = _zero_sum[p][j] / n;
			float var = _zero_sum_sq[p][j] / n - mean_d * mean_d;
			sigma[j] = sqrtf(fmaxf(var, 0.0f));
			zero[p][j] = VSENSE_NOMINAL_ZERO_V + mean_d;
			if ((fabsf(mean_d) > max_offset_v) || (sigma[j] > ZERO_CAL_MAX_SIGMA_V))
				ok = false;
		}
		// input offset: the output offset divided by the gain
		info_println("isense zero phase %u: counter 0 %.4f V (sigma %.4f V), TOP %.4f V (sigma %.4f V), input offset %.1f mV",
			(unsigned)p, zero[p][0], sigma[0], zero[p][1], sigma[1], 1000.0f * (0.5f * (zero[p][0] + zero[p][1]) - VSENSE_NOMINAL_ZERO_V) / gain);
	}
	if (!ok) {
		err_println("isense zero calibration: failed, permitted %.4f V +- %.3f V (input offset %.0f mV x gain %.0f), sigma max %.3f V",
			VSENSE_NOMINAL_ZERO_V, max_offset_v, 1000.0f * ZERO_CAL_MAX_INPUT_OFFSET_V, gain, ZERO_CAL_MAX_SIGMA_V);
		return;
	}
	for (size_t p = 0; p < SMOL_SERVO_PHASES; ++p)
		for (size_t j = 0; j < 2; ++j)
			_cal_zero_v[p][j] = zero[p][j];
	// positive current: into the motor terminal. The output falls for a positive current: CUR = VOFS - gain * (R * I + VINOFS) (TMC6200 datasheet 6). The pulse test on the bench agrees.
	_cal_amps_per_volt = -1.0f / (gain * ISENSE_R);
	__dmb(); // the non-volatile zeros and scale must be in memory before core 1 sees _cal_valid
	_cal_valid = true;
	info_println("isense zero calibration: done");
}

/**
 * Core 0 main loop: act on the states that the loop advanced to.
 */
static void _core0_run(void) {
	// nothing to do before the start, or when OFF is requested
	if (!_loop_started || (_req_state == SMOL_LOOP_OFF))
		return;

	smol_loop_state_t state = _state;
	switch (state) {
		case SMOL_LOOP_ZERO_CAL_DONE:
			__dmb(); // the non-volatile sums must be read after the ZERO_CAL_DONE read, not earlier by the compiler or the CPU
			_zero_cal_evaluate();
			smol_servo_loop_request_state(SMOL_LOOP_OFF);
			break;
		case SMOL_LOOP_MOTOR_ID_DONE:
			smol_motor_id_evaluate();
			smol_servo_loop_request_state(SMOL_LOOP_OFF);
			break;
		case SMOL_LOOP_FAULT:
			// the loop switched the bridge off. Switch the power stage off, too.
			smol_drv_disable();
			err_println("loop fault in state %s: overcurrent phase %u, %.2f A (limit %.1f A), TMC6200 disabled",
				smol_servo_loop_state_name(_req_state), (unsigned)_fault_phase, _fault_a, SMOL_ISENSE_FAULT_A);
			smol_servo_loop_request_state(SMOL_LOOP_OFF);
			break;
		case SMOL_LOOP_MOTOR_ID:
		case SMOL_LOOP_CONTROL:
			if (!smol_drv_is_enabled()) {
				// for example smol_drv_run() after a FAULT pin
				err_println("loop state %s: the TMC6200 was disabled", smol_servo_loop_state_name(state));
				smol_servo_loop_request_state(SMOL_LOOP_OFF);
			}
			break;
		default:
			break;
	}
}

void smol_servo_loop_print_state(void) {
	info_println("loop state %s, zero calibration %s, TMC6200 %s",
		smol_servo_loop_state_name(_state), _cal_valid ? "valid" : "not valid", smol_drv_is_enabled() ? "enabled" : "disabled");
}

bool smol_servo_loop_isense_zero_is_busy(void) {
	// core 0 requests OFF after the evaluation
	return _req_state == SMOL_LOOP_ZERO_CAL;
}

bool smol_servo_loop_isense_zero(float zero_v[3][2]) {
	if (!_cal_valid)
		return false;
	for (size_t p = 0; p < SMOL_SERVO_PHASES; ++p)
		for (size_t j = 0; j < 2; ++j)
			zero_v[p][j] = _cal_zero_v[p][j];
	return true;
}

void smol_servo_loop_isense_gain_changed(void) {
	// the zeros are not valid for the new gain. The gain change needs the TMC6200 disabled, so no bridge state runs.
	_cal_valid = false;
	if (_loop_started)
		smol_servo_loop_request_state(SMOL_LOOP_ZERO_CAL);
}

/**
 * Start the servo loop peripherals (MAG, PWM, ADC), then the zero calibration with DRV_EN low. A second call does only a new zero calibration.
 */
void smol_servo_loop_start(void) {
	if (smol_drv_is_enabled()) {
		err_println("servo loop start: the TMC6200 is enabled, set DRV_EN low first");
		return;
	}
	if (!_loop_started) {
		smol_mag_start();
		smol_pwm_start();
		smol_adc_start();
		_loop_started = true;
	}
	smol_servo_loop_request_state(SMOL_LOOP_ZERO_CAL);
}

static uint64_t t_idle = 0;

void __not_in_flash_func(smol_servo_loop_idle_core1)(void) {
	// our idle function on core 1 for when there's nothing else to do.
	// NOTE: do not use functions that use long critical sections disabling interrupts, like sleep_ms()! In fact, everything here should be RAM resident to avoid XIP stalls and thus IRQ jitter.
	tight_loop_contents();
	return;
}

void smol_servo_loop_idle_core0(void) {
	_core0_run();
	smol_motor_id_core0_run();
	smol_isense_cal_core0_run();

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

