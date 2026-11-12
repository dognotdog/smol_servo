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
 * Pulse test: w_ns is the pulse width, cycles is the number of 12-loop cycles to average.
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

// Core 1: true if the loop subtracts the current sense offset table (smol_isense_cal.h) from the currents of this test. Not in the calibration sweep: it measures the table.
bool smol_motor_id_uses_isense_cal(void);

/**
 * Core 0: print the result after the loop state MOTOR_ID_DONE.
 */
void smol_motor_id_evaluate(void);

/**
 * Core 0: automatic sequence of the pulse test for all 3 pairs and 3 widths, then the fit and the 2-phase / 3-phase classification. Returns 0 if the first run started.
 */
int smol_motor_id_auto_start(uint32_t cycles);
/**
 * Core 0: recovery sweep over the PWM compare values lo..hi in steps (0..1250, duty = cc / 1250), loops per value. Not automatic. The rotor must not turn (a connected, stationary motor gives the same result).
 */
int smol_motor_id_rec_start(uint32_t lo_cc, uint32_t hi_cc, uint32_t step_cc, uint32_t loops);
/**
 * Core 0: calibration sweep in one run without a stop: warm_s seconds at warm_cc, baseline, cc lo..hi up and hi..lo down with loops per cc, baseline. The values are relative to ZERO_CAL. After the run, core 0 writes the current sense offset table (/isense_cal.bin, see smol_isense_cal.h) and prints the values. The rotor must not turn.
 */
int smol_motor_id_cal_start(uint32_t lo_cc, uint32_t hi_cc, uint32_t loops, uint32_t warm_s, uint32_t warm_cc);
// cancels the automatic sequence and the recovery sweep
void smol_motor_id_auto_cancel(void);
// call it in the core 0 main loop
void smol_motor_id_core0_run(void);
