/**
 * smol-servo motor PWM
 * 
 * For the 3-phase bridge, we're using PWM/EN channel pairs 4A/B, 5A/B, 8A/B.
 * 
 * For the optional power dissipation resistor we use PWM 3A.
 * 
 * According to 12.5.2.6 of the RP3250 Datasheet
 *   pwm_period = (TOP + 1) * (CSR_PHASE_CORRECT + 1) * (DIV_INT + DIV_FRAC / 16)
 * 
 * If we're targeting a basic servo loop frequency of 20kHz, with 3 PWM intervals each (for round-robin ADC), and we're doing "phase correct" (up/down) PWM, then we should aim for 120kHz per TOP+1, so that we can sample both the "on" and "off" periods.
 * 
 * We have a 150MHz sysclk, so pwm_period = 1250 should get us to 120kHz.
 * 
 * 1250 gives us ~10.28 bits of resolution. 
 * 
 * Now, if servo loop is 1/3rd the speed of PWM samples, we need to feed the PWM.CC from a DMA buffer.
 */

#include "smol_servo.h"
#include "smol_servo_parameters.h"
#include "smol_fast.h"
#include "pico_debug.h"

#include "hardware/pwm.h"
#include "hardware/gpio.h"

#include <assert.h>


_Static_assert((SMOL_SERVO_BRIDGE_PWM_PERIOD_CLOCKS % 2) == 0);
#define BRIDGE_PWM_TOP (SMOL_SERVO_BRIDGE_PWM_PERIOD_CLOCKS / 2 - 1)

#define BRIDGE_PWM_MID (SMOL_SERVO_BRIDGE_PWM_PERIOD_CLOCKS / 4)

#define BRIDGE_PWM_SLICES {PWMA_SLICE, PWMB_SLICE, PWMC_SLICE, PWMR_SLICE}
#define BRIDGE_PWM_COUNT (4) // A,B,C,R
#define BRIDGE_PWM_RINGBUF_NUM_ENTRIES (4)

_Static_assert(BRIDGE_PWM_RINGBUF_NUM_ENTRIES > SMOL_SERVO_PWM_PER_LOOP);

static uint16_t _pwm_cc_ringbufs[BRIDGE_PWM_COUNT][BRIDGE_PWM_RINGBUF_NUM_ENTRIES][2] __attribute__((aligned(4*BRIDGE_PWM_RINGBUF_NUM_ENTRIES)));

typedef struct {
	uint32_t ringbuf;
	uint32_t loop;
} smol_pwm_index_t;

static smol_pwm_index_t _indices;

static bool _pwm_is_started = false;

typedef struct {
	uint32_t ringbuf_num_entries;
	uint32_t pwm_per_servo_loop;
	uint32_t pwma_slice;
	uint32_t bridge_pwm_count;
	size_t bridge_pwm_slices[BRIDGE_PWM_COUNT];
	uint16_t pwm_cc_loop_bufs[BRIDGE_PWM_COUNT][SMOL_SERVO_PWM_PER_LOOP][2];

} _smol_servo_pwm_ram_vars_t;

static _smol_servo_pwm_ram_vars_t __not_in_flash("smol_pwm") _ram;

static _smol_servo_pwm_ram_vars_t _ram = {
	.ringbuf_num_entries = BRIDGE_PWM_RINGBUF_NUM_ENTRIES,
	.pwm_per_servo_loop = SMOL_SERVO_PWM_PER_LOOP,
	.pwma_slice = PWMA_SLICE,
	.bridge_pwm_count = BRIDGE_PWM_COUNT,
	.bridge_pwm_slices = BRIDGE_PWM_SLICES,

	.pwm_cc_loop_bufs = {
		// {{150, 150}, {150, 150}, {150, 150}},
		// {{150, 150}, {150, 150}, {150, 150}},
		// {{150, 150}, {150, 150}, {150, 150}},
		// {{150, 150}, {150, 150}, {150, 150}},
		{{75, 75}, {150, 150}, {225, 225}},
		{{75, 75}, {150, 150}, {225, 225}},
		{{75, 75}, {150, 150}, {225, 225}},
		{{75, 75}, {150, 150}, {225, 225}},
		// {{BRIDGE_PWM_MID, BRIDGE_PWM_MID}, {BRIDGE_PWM_MID, BRIDGE_PWM_MID}, {BRIDGE_PWM_MID, BRIDGE_PWM_MID}},
		// {{BRIDGE_PWM_MID, BRIDGE_PWM_MID}, {BRIDGE_PWM_MID, BRIDGE_PWM_MID}, {BRIDGE_PWM_MID, BRIDGE_PWM_MID}},
		// {{BRIDGE_PWM_MID, BRIDGE_PWM_MID}, {BRIDGE_PWM_MID, BRIDGE_PWM_MID}, {BRIDGE_PWM_MID, BRIDGE_PWM_MID}},
		// {{BRIDGE_PWM_MID, BRIDGE_PWM_MID}, {BRIDGE_PWM_MID, BRIDGE_PWM_MID}, {BRIDGE_PWM_MID, BRIDGE_PWM_MID}},
		// {{BRIDGE_PWM_MID, 0}, {BRIDGE_PWM_MID, 0}, {BRIDGE_PWM_MID, 0}},
		// {{BRIDGE_PWM_MID, 0}, {BRIDGE_PWM_MID, 0}, {BRIDGE_PWM_MID, 0}},
		// {{BRIDGE_PWM_MID, 0}, {BRIDGE_PWM_MID, 0}, {BRIDGE_PWM_MID, 0}},
		// {{BRIDGE_PWM_MID, 0}, {BRIDGE_PWM_MID, 0}, {BRIDGE_PWM_MID, 0}},
	},
};


void __not_in_flash_func(smol_pwm_fill)(uint32_t pwm_values[3][3], size_t starting_phase);
void smol_pwm_fill(uint32_t pwm_values[3][3], size_t starting_phase) {

}

// void  __not_in_flash_func(smol_bridge_pwm_wrap_irq_handler)(void) __attribute__ ((optimize(0)));
void  __not_in_flash_func(smol_bridge_pwm_wrap_irq_handler)(void);

void  smol_bridge_pwm_wrap_irq_handler(void) {
#ifdef DEBUG_PWM_IRQ_WITH_GPIO
	gpio_out_put_fast(DEBUG_PWM_IRQ_WITH_GPIO, 1);
#endif
	pwm_clear_irq(_ram.pwma_slice);
	/**
	 * 0      1       2       0
	 *        |       ^
	 *        v       |
	 *        IRQ(0)--'
	 */
	// this is expected to be called when the BRIDGE_PWM wraps.
	// prev indices point to what has just been done executing
	// current point to what has just been loaded
	// next is what we need to refill
	smol_pwm_index_t indices = _indices;
	uint32_t prev_ringbuf_index = indices.ringbuf;
	uint32_t prev_loop_index = indices.loop;

	uint32_t ringbuf_index = (prev_ringbuf_index + 1) % _ram.ringbuf_num_entries;
	uint32_t loop_index = (prev_loop_index + 1) % _ram.pwm_per_servo_loop;

	uint32_t next_ringbuf_index = (prev_ringbuf_index + 2) % _ram.ringbuf_num_entries;
	uint32_t next_loop_index = (prev_loop_index + 2) % _ram.pwm_per_servo_loop;

	// not using DMA but directly setting values
	for (size_t i = 0; i < _ram.bridge_pwm_count; ++i) {
		size_t slice = _ram.bridge_pwm_slices[i];
		pwm_set_both_levels_fast(slice, _ram.pwm_cc_loop_bufs[i][next_loop_index][0],  _ram.pwm_cc_loop_bufs[i][next_loop_index][1]);
	}

	indices.ringbuf = ringbuf_index;
	indices.loop = loop_index;
	_indices = indices;

	// NOTE: "0" signal condition was emperically determined to make the off-by-ones line up. This way, the servo loop starts after the phase 2 (of 0-2) current was sampled by the ADC.
	if (prev_loop_index == 0)
		smol_servo_signal_loop();

#ifdef DEBUG_PWM_IRQ_WITH_GPIO
	gpio_out_put_fast(DEBUG_PWM_IRQ_WITH_GPIO, 0);
#endif
}


void smol_servo_bridge_pwm_slice_init(int slice) {
	pwm_config config = pwm_get_default_config();
	pwm_config_set_wrap(&config, BRIDGE_PWM_TOP);
	pwm_config_set_phase_correct(&config, true);
	pwm_config_set_clkdiv_int(&config, 1);
	pwm_config_set_clkdiv_mode(&config, PWM_DIV_FREE_RUNNING);
	pwm_init(slice, &config, false);
	pwm_set_both_levels(slice, BRIDGE_PWM_TOP*2/3,  BRIDGE_PWM_TOP*1/3);
}

void smol_servo_bridge_doubletime_slice_init(int slice) {
	pwm_config config = pwm_get_default_config();
	pwm_config_set_wrap(&config, BRIDGE_PWM_TOP);
	pwm_config_set_phase_correct(&config, false);
	pwm_config_set_clkdiv_int(&config, 1);
	pwm_config_set_clkdiv_mode(&config, PWM_DIV_FREE_RUNNING);
	pwm_init(slice, &config, false);
	pwm_set_both_levels(slice, BRIDGE_PWM_TOP*2/3,  BRIDGE_PWM_TOP*1/3);
}

void smol_servo_bridge_twothirds_slice_init(int slice) {
	pwm_config config = pwm_get_default_config();
	pwm_config_set_wrap(&config, SMOL_SERVO_LOOP_PERIOD_CLOCKS/4 - 1);
	pwm_config_set_phase_correct(&config, false);
	pwm_config_set_clkdiv_int(&config, 1);
	pwm_config_set_clkdiv_mode(&config, PWM_DIV_FREE_RUNNING);
	pwm_init(slice, &config, false);
	pwm_set_both_levels(slice, BRIDGE_PWM_TOP*2/3,  BRIDGE_PWM_TOP*1/3);

	pwm_set_counter(slice, SMOL_SERVO_LOOP_PERIOD_CLOCKS/6);
}


void smol_servo_bridge_pwm_init(void) {
	smol_servo_bridge_pwm_slice_init(PWMA_SLICE);
	smol_servo_bridge_pwm_slice_init(PWMB_SLICE);
	smol_servo_bridge_pwm_slice_init(PWMC_SLICE);
	smol_servo_bridge_pwm_slice_init(PWMR_SLICE);
	smol_servo_bridge_doubletime_slice_init(PWM_DOUBLETIME_SLICE);
	smol_servo_bridge_twothirds_slice_init(PWM_TWOTHIRDS_SLICE);

	gpio_set_function(PWMA_PIN, GPIO_FUNC_PWM);
	gpio_set_function(PWMB_PIN, GPIO_FUNC_PWM);
	gpio_set_function(PWMC_PIN, GPIO_FUNC_PWM);
	gpio_set_function(PWMR_PIN, GPIO_FUNC_PWM);
	gpio_set_function(ENA_PIN, GPIO_FUNC_PWM);
	gpio_set_function(ENB_PIN, GPIO_FUNC_PWM);
	gpio_set_function(ENC_PIN, GPIO_FUNC_PWM);

	pwm_clear_irq(PWMA_SLICE);
	pwm_set_irq0_enabled(PWMA_SLICE, true);
	irq_set_exclusive_handler(BRIDGE_PWM_WRAP_IRQ, smol_bridge_pwm_wrap_irq_handler);

#ifdef DEBUG_PWM_IRQ_WITH_GPIO
	gpio_init(DEBUG_PWM_IRQ_WITH_GPIO);
	gpio_set_dir(DEBUG_PWM_IRQ_WITH_GPIO, true);
#endif
}


void smol_pwm_init(void) {
	smol_servo_bridge_pwm_init();
}

void smol_pwm_start(void) {
	pwm_set_mask_enabled(SMOL_SERVO_PWM_SLICE_MASK);
	_pwm_is_started = true;
}

bool smol_pwm_was_started(void) {
	return _pwm_is_started;
}
