#pragma once

/**
 * Motor identification math: line fit and 2-phase / 3-phase classification from the 3 terminal pair inductances.
 * Plain C without Pico SDK dependencies, so that a host test can use it (tools/test_motor_id_math.c).
 */

#include <stddef.h>
#include <stdbool.h>

// terminal pairs: 0 = A-B, 1 = B-C, 2 = C-A
#define SMOL_MOTOR_ID_PAIRS (3)

// a pair with a larger inductance than this is open
#define SMOL_MOTOR_ID_L_OPEN_H (0.05f)
// s = L_max / (L_mid + L_min): 2-phase above S_2PH (ideal 1.0), 3-phase below S_3PH (ideal 0.5, approx. 0.62 with 30 % saliency)
#define SMOL_MOTOR_ID_S_2PH (0.8f)
#define SMOL_MOTOR_ID_S_3PH (0.7f)

typedef enum {
	SMOL_MOTOR_TYPE_2PH = 0,
	SMOL_MOTOR_TYPE_3PH,
	// all pairs open
	SMOL_MOTOR_TYPE_NO_MOTOR,
	// only one pair is closed: one coil only
	SMOL_MOTOR_TYPE_SINGLE_COIL,
	// two pairs closed and one open: not possible with a 2-phase or a 3-phase motor
	SMOL_MOTOR_TYPE_INCONSISTENT,
	// s between S_3PH and S_2PH
	SMOL_MOTOR_TYPE_UNCLEAR,
} smol_motor_type_t;

typedef struct {
	smol_motor_type_t type;
	float s;
	// 2-phase: the terminal in both coils (0 = A, 1 = B, 2 = C), else -1
	int shared_terminal;
	/**
	 * 2-phase: coil[0] and coil[1] are the 2 coil inductances, in the order of the terminals that are not shared (the coil of the lower terminal first), coil[2] = 0.
	 * 3-phase: per-terminal wye equivalent L_A, L_B, L_C (L - M of each phase).
	 */
	float coil_h[3];
} smol_motor_class_t;

/**
 * Least squares line y = slope * x + intercept. Returns false if n < 2 or all x are equal.
 */
bool smol_motor_id_fit_line(const float* x, const float* y, size_t n, float* slope, float* intercept);

/**
 * Classify the motor from the pair inductances (H). A value <= 0, NaN or above SMOL_MOTOR_ID_L_OPEN_H is an open pair.
 */
smol_motor_class_t smol_motor_id_classify(const float pair_l_h[SMOL_MOTOR_ID_PAIRS]);

const char* smol_motor_type_name(smol_motor_type_t type);
