#pragma once

/**
 * Servo loop states and outputs. See smol_servo_loop.c.
 */

#include "smol_servo_parameters.h"

#include "pico.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/**
 * Loop states. Core 0 requests a state with smol_servo_loop_request_state(). Core 1 executes the state in each loop IRQ (the switch in smol_servo_loop_irq_handler()).
 * A bounded state (ZERO_CAL) advances itself to its DONE state. Core 0 and core 1 code can check the state with smol_servo_loop_state().
 * To add a state: add it here, to _state_names[] and to the switch, and add its preconditions to smol_servo_loop_request_state().
 */
typedef enum {
	// bridge off
	SMOL_LOOP_OFF = 0,
	// current sense zero calibration, bridge off, DRV_EN low. Next: ZERO_CAL_DONE.
	SMOL_LOOP_ZERO_CAL,
	// the zero calibration sums are complete, bridge off. Core 0 evaluates them and requests OFF.
	SMOL_LOOP_ZERO_CAL_DONE,
	// motor identification: inductance pulse test of one terminal pair (smol_motor_id.c). Next: MOTOR_ID_DONE.
	SMOL_LOOP_MOTOR_ID,
	// the pulse test is complete or aborted, bridge off. Core 0 prints the result and requests OFF.
	SMOL_LOOP_MOTOR_ID_DONE,
	// closed loop control (not implemented, bridge off)
	SMOL_LOOP_CONTROL,
	// bridge PWM test pattern for a scope, until a new request
	SMOL_LOOP_PWM_TEST,
	// overcurrent in a state that switches the bridge, bridge off. Core 0 disables the TMC6200 and requests OFF.
	SMOL_LOOP_FAULT,
	SMOL_LOOP_NUM_STATES,
} smol_loop_state_t;

/**
 * Half bridge outputs for the 3 bridge PWM periods of the next loop.
 * duty is the high side on time, 0 to 1. If enable is false, the half bridge is off (both FETs off, output floating).
 */
typedef struct {
	float duty[SMOL_SERVO_PWM_PER_LOOP][SMOL_SERVO_PHASES];
	bool enable[SMOL_SERVO_PWM_PER_LOOP][SMOL_SERVO_PHASES];
} smol_bridge_out_t;

/**
 * The PWM CC buffer words of one loop, from smol_pwm_prepare(): (enable cc << 16) | pwm cc for each PWM period k and half bridge p. Zero: the half bridge is off.
 * The loop calculates them at its end, so that smol_pwm_fill() at the start of the next loop only copies them.
 */
typedef struct {
	uint32_t word[SMOL_SERVO_PWM_PER_LOOP][SMOL_SERVO_PHASES];
} smol_bridge_cc_t;

// PWM compare value for duty 1 (phase-correct: CC_FULL is half the period, the output is high while COUNT < CC)
#define SMOL_BRIDGE_PWM_CC_FULL (SMOL_SERVO_BRIDGE_PWM_PERIOD_CLOCKS / 2)

// compare value of a duty, the same rounding as the PWM (smol_servo_pwm.c, in RAM)
uint16_t smol_duty_to_cc(float duty);

// all half bridges off. In RAM (__not_in_flash_func), so that core 1 can call an out-of-line copy. No memset.
static inline void __not_in_flash_func(smol_bridge_out_off)(smol_bridge_out_t* out) {
	for (size_t k = 0; k < SMOL_SERVO_PWM_PER_LOOP; ++k) {
		for (size_t p = 0; p < SMOL_SERVO_PHASES; ++p) {
			out->duty[k][p] = 0.0f;
			out->enable[k][p] = false;
		}
	}
}

/**
 * Core 0 interface
 */
const char* smol_servo_loop_state_name(smol_loop_state_t state);
// returns SMOL_LOOP_NUM_STATES for an unknown name
smol_loop_state_t smol_servo_loop_state_from_name(const char* name);
smol_loop_state_t smol_servo_loop_state(void);
// checks the preconditions, then requests the state from core 1. Returns 0 on success.
int smol_servo_loop_request_state(smol_loop_state_t state);
void smol_servo_loop_print_state(void);
/**
 * Test of the ADC sample timing: busy wait in system clocks at the end of the loop (0..SMOL_SERVO_LOOP_END_DELAY_MAX_CLOCKS), so that the loop end moves later. Core 0. Returns 0 on success.
 */
int smol_servo_loop_set_end_delay(uint32_t clocks);
uint32_t smol_servo_loop_end_delay(void);
// timer 1 (system clock) timestamp of the last bridge PWM wrap (DMA), in RAM for the IRQs
uint32_t smol_servo_loop_wrap_clocks(void);
// Core 0: print the delay and the time of the loop end after the PWM wrap of the loop signal (min, mean, max in MOTOR_ID since the last set)
void smol_servo_loop_print_end_delay(void);
