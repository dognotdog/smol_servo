/**
 * Host test for src/smol_motor_id_math.c.
 * cc -std=c11 -Wall -I../src test_motor_id_math.c ../src/smol_motor_id_math.c -o /tmp/test_motor_id_math && /tmp/test_motor_id_math
 */
#include "smol_motor_id_math.h"

#include <math.h>
#include <stdio.h>

static int _fails = 0;

#define CHECK(cond) do { if (!(cond)) { printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond); ++_fails; } } while (0)
#define NEAR(a, b, tol) (fabsf((a) - (b)) <= (tol))

int main(void) {
	// line fit
	{
		float x[3] = {1, 2, 3};
		float y[3] = {3, 5, 7};
		float a, b;
		CHECK(smol_motor_id_fit_line(x, y, 3, &a, &b));
		CHECK(NEAR(a, 2.0f, 1e-5f) && NEAR(b, 1.0f, 1e-5f));
		float xs[2] = {1, 1};
		CHECK(!smol_motor_id_fit_line(xs, y, 2, &a, &b));
		CHECK(!smol_motor_id_fit_line(x, y, 1, &a, &b));
	}
	// 2-phase, shared terminal B (bench: AB 2.76, BC 2.74, CA 4.71 mH)
	{
		float l[3] = {2.76e-3f, 2.74e-3f, 4.71e-3f};
		smol_motor_class_t c = smol_motor_id_classify(l);
		CHECK(c.type == SMOL_MOTOR_TYPE_2PH);
		CHECK(c.shared_terminal == 1);
		CHECK(NEAR(c.s, 0.8564f, 1e-3f));
		// coil to A first, then the coil to C
		CHECK(NEAR(c.coil_h[0], 2.76e-3f, 1e-7f) && NEAR(c.coil_h[1], 2.74e-3f, 1e-7f));
	}
	// 2-phase ideal, shared terminal A: pair BC is the series path
	{
		float l[3] = {2.0e-3f, 4.5e-3f, 2.5e-3f};
		smol_motor_class_t c = smol_motor_id_classify(l);
		CHECK(c.type == SMOL_MOTOR_TYPE_2PH);
		CHECK(c.shared_terminal == 0);
		CHECK(NEAR(c.s, 1.0f, 1e-5f));
		// non-shared terminals B and C: coil A-B (2.0) first, then C-A (2.5)
		CHECK(NEAR(c.coil_h[0], 2.0e-3f, 1e-7f) && NEAR(c.coil_h[1], 2.5e-3f, 1e-7f));
	}
	// 3-phase balanced: each pair 2 L
	{
		float l[3] = {1.0e-3f, 1.0e-3f, 1.0e-3f};
		smol_motor_class_t c = smol_motor_id_classify(l);
		CHECK(c.type == SMOL_MOTOR_TYPE_3PH);
		CHECK(NEAR(c.s, 0.5f, 1e-5f));
		for (int t = 0; t < 3; ++t)
			CHECK(NEAR(c.coil_h[t], 0.5e-3f, 1e-7f));
	}
	// 3-phase with saliency: L_A 0.65, L_B 0.35, L_C 0.5 mH. Pairs AB 1.0, BC 0.85, CA 1.15. s = 1.15 / 1.85 = 0.62
	{
		float l[3] = {1.0e-3f, 0.85e-3f, 1.15e-3f};
		smol_motor_class_t c = smol_motor_id_classify(l);
		CHECK(c.type == SMOL_MOTOR_TYPE_3PH);
		CHECK(NEAR(c.coil_h[0], 0.65e-3f, 1e-6f) && NEAR(c.coil_h[1], 0.35e-3f, 1e-6f) && NEAR(c.coil_h[2], 0.5e-3f, 1e-6f));
	}
	// unclear
	{
		float l[3] = {1.0e-3f, 1.0e-3f, 1.5e-3f};
		CHECK(smol_motor_id_classify(l).type == SMOL_MOTOR_TYPE_UNCLEAR);
	}
	// faults
	{
		float none[3] = {0.0f, -1.0f, NAN};
		CHECK(smol_motor_id_classify(none).type == SMOL_MOTOR_TYPE_NO_MOTOR);
		float one[3] = {2.6e-3f, 1.0f, 0.0f};
		CHECK(smol_motor_id_classify(one).type == SMOL_MOTOR_TYPE_SINGLE_COIL);
		float two[3] = {2.6e-3f, 2.6e-3f, 0.0f};
		CHECK(smol_motor_id_classify(two).type == SMOL_MOTOR_TYPE_INCONSISTENT);
	}
	printf("%s (%d failures)\n", _fails ? "FAIL" : "PASS", _fails);
	return _fails ? 1 : 0;
}
