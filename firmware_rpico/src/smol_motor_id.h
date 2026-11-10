#pragma once

/**
 * Motor identification, inductance pulse test (loop state MOTOR_ID). See smol_motor_id.c.
 */

#include "smol_servo_loop.h"

#include <stdbool.h>
#include <stdint.h>

typedef enum {
	SMOL_MOTOR_ID_RUN = 0,
	SMOL_MOTOR_ID_DONE,
	SMOL_MOTOR_ID_ABORT,
} smol_motor_id_step_t;

/**
 * Core 0: set the parameters and clear the sums. Call them only when the loop is not in MOTOR_ID. Return 0 on success.
 * pair: 0 = A->B, 1 = B->C, 2 = C->A.
 * Pulse test: w_ns is the width of the first pulse, cycles is the number of 9-loop cycles to average.
 * DC test: duty is the duty of X (Y has duty 0), loops is the test duration.
 */
int smol_motor_id_prepare_pulse(uint32_t pair, uint32_t w_ns, uint32_t cycles);
int smol_motor_id_prepare_dc(uint32_t pair, float duty, uint32_t loops);
/**
 * Recovery test, for a disconnected motor: all 3 terminals get the same duty (0..1) for loops (18..20000) after the settle phase. The result is the amplifier error at this duty.
 */
int smol_motor_id_prepare_rec(float duty, uint32_t loops);

/**
 * Core 1: one loop of MOTOR_ID. amps are the phase currents of this loop (calibrated, positive into the motor terminal). out is the bridge output for the next loop.
 */
smol_motor_id_step_t smol_motor_id_step(const float amps[SMOL_SERVO_PHASES][2], smol_bridge_out_t* out);

/**
 * Core 0: print the result after the loop state MOTOR_ID_DONE.
 */
void smol_motor_id_evaluate(void);

/**
 * Core 0: automatic sequence of the pulse test for all 3 pairs and 3 widths, then the fit and the 2-phase / 3-phase classification. Returns 0 if the first run started.
 */
int smol_motor_id_auto_start(uint32_t cycles);
/**
 * Core 0: recovery sweep over the duties lo..hi in steps (permille), loops per duty. Not automatic: the motor must be disconnected.
 */
int smol_motor_id_rec_start(uint32_t lo_permille, uint32_t hi_permille, uint32_t step_permille, uint32_t loops);
// cancels the automatic sequence and the recovery sweep
void smol_motor_id_auto_cancel(void);
// call it in the core 0 main loop
void smol_motor_id_core0_run(void);
