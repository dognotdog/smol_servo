#include "smol_motor_id.h"

#include "smol_servo.h"
#include "smol_servo_parameters.h"
#include "pico_debug.h"
#include "smol_motor_id_math.h"

#include "hardware/sync.h"
#include "pico/platform.h"

#include <inttypes.h>
#include <math.h>
#include <stdio.h>

/**
 * Motor identification tests for one terminal pair X->Y (loop state MOTOR_ID). The third terminal Z is off (floating).
 *
 * Freewheel: X and Y low side on (duty 0). The current circulates through the 2 low side FETs and decays with L/R (approx. 1-2 ms).
 * A high side freewheel (duty 1) was tested first: the floating sense amplifiers then drift back to the bridge-off level within some ms, and the first switching event moves 2 channels by approx. 1.5 A. The low side freewheel keeps both terminals at GND.
 *
 * Settle and baseline: the zero calibration (ZERO_CAL, bridge off) does not give the zero with the bridge on. On the bench, all 3 channels move by 0.2-0.4 V (2-4 A at gain 20) when the bridge switches on, also the channel of the floating terminal, and they drift for some ms. Thus each test starts with MOTOR_ID_SETTLE_LOOPS of freewheel without current. The mean of the second half is the baseline. All values are relative to the baseline.
 *
 * Pulse test (smol_motor_id_prepare_pulse): one terminal high for a total of W in one PWM period. The pair then has U_bus across it, and the current changes by U_bus * W / L.
 * In one phase-correct period with duty W / T, the output is high for W/2 at the start and W/2 at the end of the period, and low around TOP.
 * One cycle is 9 loops (450 us). Each loop has 3 PWM periods. The pulse is in the middle period (k = 1) of the loop.
 * - loop 0: X high for W. The current X->Y goes to +d.
 * - loop 3: Y high for 2W. The current goes to -d.
 * - loop 6: X high for W. The current goes back to approx. 0.
 * The +d and the -d plateaus are 3 loops each, so the net torque impulse is approx. zero and the rotor does not move.
 * Core 1 adds the samples of each loop of the cycle (9 segments x 3 phases x 2 samples). The first cycle is not added.
 * Core 0 prints the mean waveform. The output delay (5 PWM periods) moves the plateaus relative to the segments. The waveform shows the alignment on the bench.
 *
 * DC test (smol_motor_id_prepare_dc): X duty d, Y duty 0, for a number of loops. The mean voltage d * U_bus drives a DC current X->Y of d * U_bus / R_pair. Core 1 adds the samples of the second half into segment 0. This test does not depend on the pulse timing, and it checks the output path.
 *
 * Recovery test (smol_motor_id_prepare_rec, id_rec): all 3 terminals get the same duty in all PWM periods, after the settle phase at duty 0 (all 3 low side on). The terminal voltages are equal, so no current flows, also with a motor. Each sample then shows only the amplifier error for this duty: the response to the own switching edges (ringing and recovery tail) and the common mode error of the terminal voltage. Core 1 adds the samples in 9 time bins of equal length, so that a slow drift is visible. The test is for a disconnected motor: then also small timing differences between the phases cannot drive a current.
 *
 * Abort: after the baseline, if the smaller magnitude of the 2 samples of a phase minus the baseline is above MOTOR_ID_ABORT_A, the test stops, and core 0 disables the TMC6200.
 */

#define MOTOR_ID_SEGMENTS (9u)
#define MOTOR_ID_PULSE_PERIOD (1u)
#define MOTOR_ID_ABORT_A (1.0f)
#define MOTOR_ID_W_NS_MIN (50u)
// the 2W pulse must fit in one PWM period with margin, and leave the counter 0 samples clean
#define MOTOR_ID_W_NS_MAX (4000u)
#define MOTOR_ID_CYCLES_MAX (4096u)
// 10 ms freewheel before the test, the second half is the baseline
#define MOTOR_ID_SETTLE_LOOPS (200u)
// DC test: duty of X 0..0.05, 2..20000 loops (1 s)
#define MOTOR_ID_DC_DUTY_MAX (0.05f)
#define MOTOR_ID_DC_LOOPS_MAX (20000u)
// recovery test: 18..20000 loops (1 s), 9 time bins
#define MOTOR_ID_REC_LOOPS_MIN (18u)
#define MOTOR_ID_REC_LOOPS_MAX (20000u)

typedef enum {
	MOTOR_ID_MODE_PULSE = 0,
	MOTOR_ID_MODE_DC,
	MOTOR_ID_MODE_REC,
} motor_id_mode_t;

// terminal pairs X->Y, Z is the third terminal. In RAM: core 1 has no flash access.
static const uint8_t __not_in_flash("loop") _pair_x[3] = {0, 1, 2};
static const uint8_t __not_in_flash("loop") _pair_y[3] = {1, 2, 0};
static const uint8_t __not_in_flash("loop") _pair_z[3] = {2, 0, 1};
static const char _phase_names[3] = {'A', 'B', 'C'};

// core 0 writes them before the request of MOTOR_ID, core 1 reads them in MOTOR_ID
static motor_id_mode_t _mode = MOTOR_ID_MODE_PULSE;
static uint32_t _pair = 0;
static uint32_t _w_ns = 0;
// pulse: number of cycles. DC: number of loops.
static uint32_t _count = 0;
static float _duty_w = 0.0f;
static float _duty_2w = 0.0f;
static float _duty_dc = 0.0f;
static float _duty_rec = 0.0f;

// core 0 clears them, core 1 writes them in MOTOR_ID, core 0 reads them after MOTOR_ID_DONE
static float _sum[MOTOR_ID_SEGMENTS][SMOL_SERVO_PHASES][2];
static float _sum_sq[MOTOR_ID_SEGMENTS][SMOL_SERVO_PHASES][2];
static float _base_sum[SMOL_SERVO_PHASES][2];
static float _base[SMOL_SERVO_PHASES][2];
static uint32_t _loops = 0;
// pulse: added cycles. DC, recovery: added loops.
static uint32_t _added = 0;
// recovery: added loops in each time bin
static uint32_t _bin_n[MOTOR_ID_SEGMENTS];
static bool _aborted = false;
static uint32_t _abort_phase = 0;
static float _abort_a = 0.0f;
static uint32_t _abort_loop = 0;
static float _abort_amps[SMOL_SERVO_PHASES][2];

// duty for a total high time of w_ns in one period: the output is high for 2 * CC clocks, and CC_FULL is half the period
static float _duty_for_high_ns(uint32_t w_ns) {
	float w_clocks = (float)w_ns * 1e-9f * (float)SMOL_SERVO_BRIDGE_PWM_PERIOD_CLOCKS * (float)SMOL_SERVO_BRIDGE_PWM_FREQ_HZ;
	return w_clocks / (float)SMOL_SERVO_BRIDGE_PWM_PERIOD_CLOCKS;
}

static int _check_pair(uint32_t pair) {
	if (pair > 2) {
		err_println("motor id: pair %" PRIu32 " not possible (0 = A->B, 1 = B->C, 2 = C->A)", pair);
		return -1;
	}
	return 0;
}

static void _clear(void) {
	for (size_t s = 0; s < MOTOR_ID_SEGMENTS; ++s) {
		for (size_t p = 0; p < SMOL_SERVO_PHASES; ++p) {
			for (size_t j = 0; j < 2; ++j) {
				_sum[s][p][j] = 0.0f;
				_sum_sq[s][p][j] = 0.0f;
			}
		}
	}
	for (size_t p = 0; p < SMOL_SERVO_PHASES; ++p) {
		for (size_t j = 0; j < 2; ++j) {
			_base_sum[p][j] = 0.0f;
			_base[p][j] = 0.0f;
		}
	}
	for (size_t s = 0; s < MOTOR_ID_SEGMENTS; ++s)
		_bin_n[s] = 0;
	_loops = 0;
	_added = 0;
	_aborted = false;
	_abort_phase = 0;
	_abort_a = 0.0f;
	_abort_loop = 0;
	__dmb(); // the parameters and the cleared sums must be in memory before core 1 sees the MOTOR_ID request
}

int smol_motor_id_prepare_pulse(uint32_t pair, uint32_t w_ns, uint32_t cycles) {
	if (_check_pair(pair) != 0)
		return -1;
	if ((w_ns < MOTOR_ID_W_NS_MIN) || (w_ns > MOTOR_ID_W_NS_MAX)) {
		err_println("motor id: pulse width %" PRIu32 " ns not possible (%u..%u ns)", w_ns, MOTOR_ID_W_NS_MIN, MOTOR_ID_W_NS_MAX);
		return -1;
	}
	if ((cycles < 1) || (cycles > MOTOR_ID_CYCLES_MAX)) {
		err_println("motor id: %" PRIu32 " cycles not possible (1..%u)", cycles, MOTOR_ID_CYCLES_MAX);
		return -1;
	}
	_mode = MOTOR_ID_MODE_PULSE;
	_pair = pair;
	_w_ns = w_ns;
	_count = cycles;
	_duty_w = _duty_for_high_ns(w_ns);
	_duty_2w = _duty_for_high_ns(2 * w_ns);
	_clear();
	return 0;
}

int smol_motor_id_prepare_dc(uint32_t pair, float duty, uint32_t loops) {
	if (_check_pair(pair) != 0)
		return -1;
	if (!(duty >= 0.0f) || (duty > MOTOR_ID_DC_DUTY_MAX)) {
		err_println("motor id: DC duty %.3f not possible (0..%.2f)", duty, MOTOR_ID_DC_DUTY_MAX);
		return -1;
	}
	if ((loops < 2) || (loops > MOTOR_ID_DC_LOOPS_MAX)) {
		err_println("motor id: %" PRIu32 " DC loops not possible (2..%u)", loops, MOTOR_ID_DC_LOOPS_MAX);
		return -1;
	}
	_mode = MOTOR_ID_MODE_DC;
	_pair = pair;
	_count = loops;
	_duty_dc = duty;
	_clear();
	return 0;
}

int smol_motor_id_prepare_rec(float duty, uint32_t loops) {
	if (!(duty >= 0.0f) || (duty > 1.0f)) {
		err_println("motor id: recovery duty %.3f not possible (0..1)", duty);
		return -1;
	}
	if ((loops < MOTOR_ID_REC_LOOPS_MIN) || (loops > MOTOR_ID_REC_LOOPS_MAX)) {
		err_println("motor id: %" PRIu32 " recovery loops not possible (%u..%u)", loops, MOTOR_ID_REC_LOOPS_MIN, MOTOR_ID_REC_LOOPS_MAX);
		return -1;
	}
	_mode = MOTOR_ID_MODE_REC;
	_pair = 0;
	_count = loops;
	_duty_rec = duty;
	_clear();
	return 0;
}

static __force_inline void _add(uint32_t seg, const float amps[SMOL_SERVO_PHASES][2]) {
	for (size_t p = 0; p < SMOL_SERVO_PHASES; ++p) {
		for (size_t j = 0; j < 2; ++j) {
			float a = amps[p][j] - _base[p][j];
			_sum[seg][p][j] += a;
			_sum_sq[seg][p][j] += a * a;
		}
	}
}

smol_motor_id_step_t __not_in_flash_func(smol_motor_id_step)(const float amps[SMOL_SERVO_PHASES][2], smol_bridge_out_t* out) {
	uint32_t n = _loops;
	_loops = n + 1;

	// settle and baseline, see the file comment
	if (n < MOTOR_ID_SETTLE_LOOPS) {
		if (n >= MOTOR_ID_SETTLE_LOOPS / 2) {
			for (size_t p = 0; p < SMOL_SERVO_PHASES; ++p)
				for (size_t j = 0; j < 2; ++j)
					_base_sum[p][j] += amps[p][j];
		}
		if (n == MOTOR_ID_SETTLE_LOOPS - 1) {
			const float k = 1.0f / (float)(MOTOR_ID_SETTLE_LOOPS - MOTOR_ID_SETTLE_LOOPS / 2);
			for (size_t p = 0; p < SMOL_SERVO_PHASES; ++p)
				for (size_t j = 0; j < 2; ++j)
					_base[p][j] = _base_sum[p][j] * k;
		}
	}
	else if (_mode != MOTOR_ID_MODE_REC) {
		// abort check after the baseline: the bridge goes off at once. Not in the recovery test: it has no differential voltage, and its amplifier errors are larger than the limit.
		for (size_t p = 0; p < SMOL_SERVO_PHASES; ++p) {
			float a0 = amps[p][0] - _base[p][0];
			float a1 = amps[p][1] - _base[p][1];
			a0 = (a0 < 0.0f) ? -a0 : a0;
			a1 = (a1 < 0.0f) ? -a1 : a1;
			float a = (a0 < a1) ? a0 : a1;
			if (a > MOTOR_ID_ABORT_A) {
				_abort_phase = p;
				_abort_a = a;
				_abort_loop = n;
				for (size_t q = 0; q < SMOL_SERVO_PHASES; ++q) {
					_abort_amps[q][0] = amps[q][0] - _base[q][0];
					_abort_amps[q][1] = amps[q][1] - _base[q][1];
				}
				_aborted = true;
				smol_bridge_out_off(out);
				return SMOL_MOTOR_ID_ABORT;
			}
		}
	}

	// m: loop number after the settle phase, m_next: the loop number of the output that this loop computes
	int32_t m = (int32_t)n - (int32_t)MOTOR_ID_SETTLE_LOOPS;
	int32_t m_next = m + 1;

	if (_mode == MOTOR_ID_MODE_PULSE) {
		if (m >= 0) {
			uint32_t cycle = (uint32_t)m / MOTOR_ID_SEGMENTS;
			uint32_t seg = (uint32_t)m % MOTOR_ID_SEGMENTS;
			// cycle 0 is not added: its first segments have the samples from before the first pulse
			if (cycle >= 1) {
				_add(seg, amps);
				if (seg == MOTOR_ID_SEGMENTS - 1)
					_added = cycle;
			}
			if ((cycle >= _count) && (seg == MOTOR_ID_SEGMENTS - 1)) {
				smol_bridge_out_off(out);
				return SMOL_MOTOR_ID_DONE;
			}
		}
	}
	else if (_mode == MOTOR_ID_MODE_REC) {
		if (m >= 0) {
			uint32_t bin = ((uint32_t)m * MOTOR_ID_SEGMENTS) / _count;
			_add(bin, amps);
			_bin_n[bin] += 1;
			_added = (uint32_t)m + 1;
			if ((uint32_t)m + 1 >= _count) {
				smol_bridge_out_off(out);
				return SMOL_MOTOR_ID_DONE;
			}
		}
		// all 3 terminals: same duty, low side on in the settle phase
		float d = (m_next >= 0) ? _duty_rec : 0.0f;
		for (size_t k = 0; k < SMOL_SERVO_PWM_PER_LOOP; ++k) {
			for (size_t p = 0; p < SMOL_SERVO_PHASES; ++p) {
				out->duty[k][p] = d;
				out->enable[k][p] = true;
			}
		}
		return SMOL_MOTOR_ID_RUN;
	}
	else {
		if (m >= 0) {
			if ((uint32_t)m >= _count / 2) {
				_add(0, amps);
				_added = (uint32_t)m + 1 - _count / 2;
			}
			if ((uint32_t)m + 1 >= _count) {
				smol_bridge_out_off(out);
				return SMOL_MOTOR_ID_DONE;
			}
		}
	}

	// output for the next loop: freewheel, then the pulses or the DC duty
	uint32_t x = _pair_x[_pair];
	uint32_t y = _pair_y[_pair];
	uint32_t z = _pair_z[_pair];
	for (size_t k = 0; k < SMOL_SERVO_PWM_PER_LOOP; ++k) {
		out->duty[k][x] = 0.0f;
		out->duty[k][y] = 0.0f;
		out->duty[k][z] = 0.0f;
		out->enable[k][x] = true;
		out->enable[k][y] = true;
		out->enable[k][z] = false;
	}
	if (m_next >= 0) {
		if (_mode == MOTOR_ID_MODE_PULSE) {
			uint32_t next_seg = (uint32_t)m_next % MOTOR_ID_SEGMENTS;
			if ((next_seg == 0) || (next_seg == 6))
				out->duty[MOTOR_ID_PULSE_PERIOD][x] = _duty_w;
			else if (next_seg == 3)
				out->duty[MOTOR_ID_PULSE_PERIOD][y] = _duty_2w;
		}
		else {
			for (size_t k = 0; k < SMOL_SERVO_PWM_PER_LOOP; ++k)
				out->duty[k][x] = _duty_dc;
		}
	}

	return SMOL_MOTOR_ID_RUN;
}

static void _segment_stats(size_t s, float n, float m[SMOL_SERVO_PHASES][2], float sd[SMOL_SERVO_PHASES][2]) {
	for (size_t p = 0; p < SMOL_SERVO_PHASES; ++p) {
		for (size_t j = 0; j < 2; ++j) {
			m[p][j] = _sum[s][p][j] / n;
			float var = _sum_sq[s][p][j] / n - m[p][j] * m[p][j];
			sd[p][j] = sqrtf(fmaxf(var, 0.0f));
		}
	}
}

static void _print_segment(const char* label, unsigned s, float m[SMOL_SERVO_PHASES][2], float sd[SMOL_SERVO_PHASES][2]) {
	info_println("motor id: %s%u  %7.1f %7.1f %7.1f %7.1f %7.1f %7.1f  (%5.1f %5.1f %5.1f %5.1f %5.1f %5.1f)", label, s,
		1e3f * m[0][0], 1e3f * m[0][1], 1e3f * m[1][0], 1e3f * m[1][1], 1e3f * m[2][0], 1e3f * m[2][1],
		1e3f * sd[0][0], 1e3f * sd[0][1], 1e3f * sd[1][0], 1e3f * sd[1][1], 1e3f * sd[2][0], 1e3f * sd[2][1]);
}

/**
 * Automatic sequence (id_auto), core 0: the pulse test for the 3 pairs at the widths of _auto_w_ns. A line fit of d over W gives the slope U_bus / L of each pair. Then smol_motor_id_classify() gives 2-phase or 3-phase.
 * Each run ends in MOTOR_ID_DONE. smol_motor_id_evaluate() stores d, and core 0 requests OFF. smol_motor_id_core0_run() then starts the next run.
 * A run that ends without a result (stop, fault, abort) cancels the sequence.
 */
#define AUTO_WIDTHS (5u)
#define AUTO_RUNS (SMOL_MOTOR_ID_PAIRS * AUTO_WIDTHS)
static const uint32_t _auto_w_ns[AUTO_WIDTHS] = {1000, 1500, 2000, 2500, 3000};
static struct {
	bool active;
	bool seen_running;
	bool have_result;
	uint32_t run;
	uint32_t cycles;
	float d[SMOL_MOTOR_ID_PAIRS][AUTO_WIDTHS];
	float vbus_sum;
} _auto;

static int _auto_start_run(void) {
	uint32_t pair = _auto.run / AUTO_WIDTHS;
	uint32_t w_ns = _auto_w_ns[_auto.run % AUTO_WIDTHS];
	_auto.seen_running = false;
	_auto.have_result = false;
	if (smol_motor_id_prepare_pulse(pair, w_ns, _auto.cycles) != 0)
		return -1;
	return smol_servo_loop_request_state(SMOL_LOOP_MOTOR_ID);
}

int smol_motor_id_auto_start(uint32_t cycles) {
	if (_auto.active) {
		warn_println("motor id: the automatic sequence runs");
		return -1;
	}
	_auto.run = 0;
	_auto.cycles = cycles;
	_auto.vbus_sum = 0.0f;
	_auto.active = true;
	info_println("motor id: automatic sequence, %u runs of %" PRIu32 " cycles (approx. %.1f s)",
		AUTO_RUNS, cycles, (float)AUTO_RUNS * ((float)cycles * MOTOR_ID_SEGMENTS + MOTOR_ID_SETTLE_LOOPS) / (float)SMOL_SERVO_LOOP_FREQ_HZ);
	if (_auto_start_run() != 0) {
		_auto.active = false;
		return -1;
	}
	return 0;
}

/**
 * Recovery sweep (id_rec), core 0: the recovery test at the duties lo, lo + step, .. hi (permille), in the same way as the automatic sequence.
 */
static struct {
	bool active;
	bool seen_running;
	bool have_result;
	uint32_t permille;
	uint32_t hi;
	uint32_t step;
	uint32_t loops;
} _rec;

void smol_motor_id_auto_cancel(void) {
	if (_auto.active) {
		_auto.active = false;
		warn_println("motor id: automatic sequence cancelled in run %" PRIu32, _auto.run);
	}
	if (_rec.active) {
		_rec.active = false;
		warn_println("motor id: recovery sweep cancelled at duty %" PRIu32 " permille", _rec.permille);
	}
}

static int _rec_start_run(void) {
	_rec.seen_running = false;
	_rec.have_result = false;
	if (smol_motor_id_prepare_rec((float)_rec.permille * 1e-3f, _rec.loops) != 0)
		return -1;
	return smol_servo_loop_request_state(SMOL_LOOP_MOTOR_ID);
}

int smol_motor_id_rec_start(uint32_t lo_permille, uint32_t hi_permille, uint32_t step_permille, uint32_t loops) {
	if (_auto.active || _rec.active) {
		warn_println("motor id: a sequence runs");
		return -1;
	}
	if ((lo_permille > hi_permille) || (hi_permille > 1000u) || (step_permille == 0u)) {
		err_println("motor id: recovery sweep %" PRIu32 "..%" PRIu32 " step %" PRIu32 " permille not possible", lo_permille, hi_permille, step_permille);
		return -1;
	}
	_rec.permille = lo_permille;
	_rec.hi = hi_permille;
	_rec.step = step_permille;
	_rec.loops = loops;
	uint32_t runs = (hi_permille - lo_permille) / step_permille + 1u;
	info_println("motor id: recovery sweep for a DISCONNECTED motor, duty %" PRIu32 "..%" PRIu32 " step %" PRIu32 " permille, %" PRIu32 " runs (approx. %.1f s)",
		lo_permille, hi_permille, step_permille, runs, (float)runs * (float)(loops + MOTOR_ID_SETTLE_LOOPS) / (float)SMOL_SERVO_LOOP_FREQ_HZ);
	info_println("motor id: rec lines: A0 A1 B0 B1 C0 C1 in mA minus baseline: mean and sigma of the last time bin, drift = bin 1 - bin 8");
	_rec.active = true;
	if (_rec_start_run() != 0) {
		_rec.active = false;
		return -1;
	}
	return 0;
}

static void _evaluate_rec(void) {
	const size_t last = MOTOR_ID_SEGMENTS - 1;
	if ((_bin_n[1] == 0) || (_bin_n[last] == 0)) {
		err_println("motor id: recovery test has not enough loops");
		return;
	}
	float m[SMOL_SERVO_PHASES][2];
	float sd[SMOL_SERVO_PHASES][2];
	float m1[SMOL_SERVO_PHASES][2];
	float sd1[SMOL_SERVO_PHASES][2];
	_segment_stats(last, (float)_bin_n[last], m, sd);
	_segment_stats(1, (float)_bin_n[1], m1, sd1);
	info_println("motor id: rec %.3f mean %.2f %.2f %.2f %.2f %.2f %.2f mA", _duty_rec,
		1e3f * m[0][0], 1e3f * m[0][1], 1e3f * m[1][0], 1e3f * m[1][1], 1e3f * m[2][0], 1e3f * m[2][1]);
	info_println("motor id: rec %.3f sigma %.2f %.2f %.2f %.2f %.2f %.2f mA", _duty_rec,
		1e3f * sd[0][0], 1e3f * sd[0][1], 1e3f * sd[1][0], 1e3f * sd[1][1], 1e3f * sd[2][0], 1e3f * sd[2][1]);
	info_println("motor id: rec %.3f drift %.2f %.2f %.2f %.2f %.2f %.2f mA", _duty_rec,
		1e3f * (m1[0][0] - m[0][0]), 1e3f * (m1[0][1] - m[0][1]), 1e3f * (m1[1][0] - m[1][0]),
		1e3f * (m1[1][1] - m[1][1]), 1e3f * (m1[2][0] - m[2][0]), 1e3f * (m1[2][1] - m[2][1]));
	if (_rec.active) {
		_rec.have_result = true;
		return;
	}
	// single run: baseline and all time bins
	info_println("motor id: rec baseline A %.3f %.3f B %.3f %.3f C %.3f %.3f A", _base[0][0], _base[0][1], _base[1][0], _base[1][1], _base[2][0], _base[2][1]);
	for (size_t b = 0; b < MOTOR_ID_SEGMENTS; ++b) {
		if (_bin_n[b] == 0)
			continue;
		_segment_stats(b, (float)_bin_n[b], m, sd);
		info_println("motor id: rec bin %u (%" PRIu32 " loops) %.2f %.2f %.2f %.2f %.2f %.2f mA", (unsigned)b, _bin_n[b],
			1e3f * m[0][0], 1e3f * m[0][1], 1e3f * m[1][0], 1e3f * m[1][1], 1e3f * m[2][0], 1e3f * m[2][1]);
	}
}

static void _auto_finish(void) {
	float vbus = _auto.vbus_sum / (float)AUTO_RUNS;
	float x[AUTO_WIDTHS];
	for (size_t i = 0; i < AUTO_WIDTHS; ++i)
		x[i] = (float)_auto_w_ns[i] * 1e-9f;
	float pair_l[SMOL_MOTOR_ID_PAIRS];
	for (size_t k = 0; k < SMOL_MOTOR_ID_PAIRS; ++k) {
		float slope = 0.0f;
		float intercept = 0.0f;
		pair_l[k] = 0.0f;
		bool ok = smol_motor_id_fit_line(x, _auto.d[k], AUTO_WIDTHS, &slope, &intercept);
		float res = 0.0f;
		for (size_t i = 0; i < AUTO_WIDTHS; ++i) {
			float e = _auto.d[k][i] - (slope * x[i] + intercept);
			res += e * e;
		}
		res = sqrtf(res / (float)AUTO_WIDTHS);
		if (ok && (slope > 0.0f))
			pair_l[k] = vbus / slope;
		char dbuf[16 * AUTO_WIDTHS];
		size_t dlen = 0;
		for (size_t i = 0; i < AUTO_WIDTHS; ++i)
			dlen += (size_t)snprintf(dbuf + dlen, sizeof(dbuf) - dlen, "%s%.2f", i ? " " : "", 1e3f * _auto.d[k][i]);
		info_println("motor id: pair %c-%c d %s mA, slope %.3f mA/us, intercept %.2f mA, residual %.2f mA, L %.3f mH",
			_phase_names[_pair_x[k]], _phase_names[_pair_y[k]], dbuf,
			1e-3f * slope, 1e3f * intercept, 1e3f * res, 1e3f * pair_l[k]);
	}
	smol_motor_class_t c = smol_motor_id_classify(pair_l);
	info_println("motor id: result %s, s %.3f (2-phase > %.2f, 3-phase < %.2f), VBUS %.2f V",
		smol_motor_type_name(c.type), c.s, SMOL_MOTOR_ID_S_2PH, SMOL_MOTOR_ID_S_3PH, vbus);
	if (c.type == SMOL_MOTOR_TYPE_2PH) {
		// not t: the log macros use a local variable t
		int ts = c.shared_terminal;
		int ta = (ts == 0) ? 1 : 0;
		int tb = (ts == 2) ? 1 : 2;
		info_println("motor id: shared terminal %c, coil %c-%c %.3f mH, coil %c-%c %.3f mH",
			_phase_names[ts], _phase_names[ta], _phase_names[ts], 1e3f * c.coil_h[0], _phase_names[tb], _phase_names[ts], 1e3f * c.coil_h[1]);
	}
	else if (c.type == SMOL_MOTOR_TYPE_3PH) {
		info_println("motor id: wye equivalent L-M: A %.3f mH, B %.3f mH, C %.3f mH", 1e3f * c.coil_h[0], 1e3f * c.coil_h[1], 1e3f * c.coil_h[2]);
	}
}

typedef enum {
	SEQ_WAIT = 0,
	SEQ_NEXT,
	SEQ_CANCEL,
} seq_poll_t;

// state of one run of a sequence: a run that ends without a result (stop, fault, abort) cancels the sequence
static seq_poll_t _seq_poll(bool* seen_running, bool have_result) {
	smol_loop_state_t st = smol_servo_loop_state();
	if ((st == SMOL_LOOP_MOTOR_ID) || (st == SMOL_LOOP_MOTOR_ID_DONE)) {
		*seen_running = true;
		return SEQ_WAIT;
	}
	if (st == SMOL_LOOP_FAULT)
		return SEQ_CANCEL;
	// OFF before the run started: core 1 has not taken the request yet
	if (!*seen_running || (st != SMOL_LOOP_OFF))
		return SEQ_WAIT;
	return have_result ? SEQ_NEXT : SEQ_CANCEL;
}

void smol_motor_id_core0_run(void) {
	if (_auto.active) {
		seq_poll_t r = _seq_poll(&_auto.seen_running, _auto.have_result);
		if (r == SEQ_CANCEL) {
			smol_motor_id_auto_cancel();
		}
		else if (r == SEQ_NEXT) {
			_auto.run += 1;
			if (_auto.run >= AUTO_RUNS) {
				_auto.active = false;
				_auto_finish();
			}
			else if (_auto_start_run() != 0) {
				smol_motor_id_auto_cancel();
			}
		}
	}
	if (_rec.active) {
		seq_poll_t r = _seq_poll(&_rec.seen_running, _rec.have_result);
		if (r == SEQ_CANCEL) {
			smol_motor_id_auto_cancel();
		}
		else if (r == SEQ_NEXT) {
			if (_rec.permille + _rec.step > _rec.hi) {
				_rec.active = false;
				info_println("motor id: recovery sweep done");
			}
			else {
				_rec.permille += _rec.step;
				if (_rec_start_run() != 0)
					smol_motor_id_auto_cancel();
			}
		}
	}
}

void smol_motor_id_evaluate(void) {
	__dmb(); // read the sums after the MOTOR_ID_DONE read
	uint32_t x = _pair_x[_pair];
	uint32_t y = _pair_y[_pair];
	if (_aborted) {
		err_println("motor id: abort in loop %" PRIu32 ", phase %c %.3f A (limit %.1f A), minus baseline A %.3f %.3f B %.3f %.3f C %.3f %.3f A",
			_abort_loop, _phase_names[_abort_phase], _abort_a, MOTOR_ID_ABORT_A,
			_abort_amps[0][0], _abort_amps[0][1], _abort_amps[1][0], _abort_amps[1][1], _abort_amps[2][0], _abort_amps[2][1]);
		return;
	}
	if (_added == 0) {
		err_println("motor id: no samples");
		return;
	}
	if (_mode == MOTOR_ID_MODE_REC) {
		_evaluate_rec();
		return;
	}
	const float n = (float)_added;
	float vbus = smol_adc_vbus();
	if (!_auto.active)
		info_println("motor id: baseline (bridge on, no current) A %.3f %.3f B %.3f %.3f C %.3f %.3f A",
			_base[0][0], _base[0][1], _base[1][0], _base[1][1], _base[2][0], _base[2][1]);

	float m[SMOL_SERVO_PHASES][2];
	float sd[SMOL_SERVO_PHASES][2];

	if (_mode == MOTOR_ID_MODE_DC) {
		info_println("motor id: DC pair %c->%c, duty %.4f, %" PRIu32 " loops, VBUS %.2f V", _phase_names[x], _phase_names[y], _duty_dc, _count, vbus);
		info_println("motor id: A0 A1 B0 B1 C0 C1 mean mA minus baseline (sigma mA)");
		_segment_stats(0, n, m, sd);
		_print_segment("dc", 0, m, sd);
		// TOP sample: the middle of the low time of both legs, away from the edges of X near counter 0
		float i_x = m[x][1];
		float u = _duty_dc * vbus;
		if (fabsf(i_x) > 0.0f)
			info_println("motor id: I(X) %.1f mA, I(Y) %.1f mA, I(Z) %.1f mA, mean voltage %.3f V, R pair %.3f Ohm", 1e3f * i_x, 1e3f * m[y][1], 1e3f * m[_pair_z[_pair]][1], u, u / i_x);
		else
			warn_println("motor id: no DC current on X");
		return;
	}

	info_println("motor id: pulse pair %c->%c, W %" PRIu32 " ns (duty %.4f, 2W duty %.4f), %" PRIu32 " cycles, VBUS %.2f V, BBM %" PRId32 " ns",
		_phase_names[x], _phase_names[y], _w_ns, _duty_w, _duty_2w, _added, vbus, smol_drv_bbm_ns());
	if (!_auto.active)
		info_println("motor id: seg  A0 A1 B0 B1 C0 C1 mean mA minus baseline (sigma mA)");
	// mean of both samples of X and Y in each segment
	float mx[MOTOR_ID_SEGMENTS];
	float my[MOTOR_ID_SEGMENTS];
	// per phase and sample: plateau step (segments 3 + 4) - (segments 6 + 7), / 2
	float step[SMOL_SERVO_PHASES][2] = {{0.0f}};
	for (size_t s = 0; s < MOTOR_ID_SEGMENTS; ++s) {
		_segment_stats(s, n, m, sd);
		mx[s] = 0.5f * (m[x][0] + m[x][1]);
		my[s] = 0.5f * (m[y][0] + m[y][1]);
		float sign = ((s == 3) || (s == 4)) ? 0.5f : (((s == 6) || (s == 7)) ? -0.5f : 0.0f);
		for (size_t ph = 0; ph < SMOL_SERVO_PHASES; ++ph)
			for (size_t j = 0; j < 2; ++j)
				step[ph][j] += sign * m[ph][j];
		if (!_auto.active)
			_print_segment("", (unsigned)s, m, sd);
	}
	/**
	 * Plateaus: the output of segment s shows in the samples of segment s + 2 (the pulse period, with switching spikes). The segments s + 3 and s + 4 have no switching.
	 * +d: segments 3 and 4 (after the pulse in segment 0). -d: segments 6 and 7 (after the pulse in segment 3).
	 * X carries +d and -d, Y carries -d and +d. The mean of both is d.
	 */
	float dx = 0.25f * ((mx[3] + mx[4]) - (mx[6] + mx[7]));
	float dy = 0.25f * ((my[6] + my[7]) - (my[3] + my[4]));
	float d = 0.5f * (dx + dy);
	// each half pulse loses approx. one BBM time at its rising edge
	float w = (float)_w_ns * 1e-9f;
	float w_bbm = w - 2.0f * (float)smol_drv_bbm_ns() * 1e-9f;
	info_println("motor id: d from X %.2f mA, from Y %.2f mA, mean %.2f mA, d / W %.3f mA/us", 1e3f * dx, 1e3f * dy, 1e3f * d, 1e3f * d / (w * 1e6f));
	info_println("motor id: step W %" PRIu32 " pair %c->%c A0 %.2f A1 %.2f B0 %.2f B1 %.2f C0 %.2f C1 %.2f mA",
		_w_ns, _phase_names[x], _phase_names[y], 1e3f * step[0][0], 1e3f * step[0][1], 1e3f * step[1][0], 1e3f * step[1][1], 1e3f * step[2][0], 1e3f * step[2][1]);
	if (_auto.active) {
		_auto.d[_pair][_auto.run % AUTO_WIDTHS] = d;
		_auto.vbus_sum += vbus;
		_auto.have_result = true;
		return;
	}
	if (d > 0.0f)
		info_println("motor id: L %.3f mH (W), %.3f mH (W - 2 BBM)", 1e3f * vbus * w / d, 1e3f * vbus * w_bbm / d);
	else
		warn_println("motor id: no positive current step");
}
