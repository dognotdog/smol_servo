#include "smol_motor_id_math.h"

// pair k has the terminals _pair_t0[k] and _pair_t1[k], and _pair_other[k] is not in it
static const int _pair_t0[SMOL_MOTOR_ID_PAIRS] = {0, 1, 2};
static const int _pair_t1[SMOL_MOTOR_ID_PAIRS] = {1, 2, 0};
static const int _pair_other[SMOL_MOTOR_ID_PAIRS] = {2, 0, 1};

bool smol_motor_id_fit_line(const float* x, const float* y, size_t n, float* slope, float* intercept) {
	if (n < 2)
		return false;
	float mx = 0.0f;
	float my = 0.0f;
	for (size_t i = 0; i < n; ++i) {
		mx += x[i];
		my += y[i];
	}
	mx /= (float)n;
	my /= (float)n;
	float sxx = 0.0f;
	float sxy = 0.0f;
	for (size_t i = 0; i < n; ++i) {
		sxx += (x[i] - mx) * (x[i] - mx);
		sxy += (x[i] - mx) * (y[i] - my);
	}
	if (!(sxx > 0.0f))
		return false;
	*slope = sxy / sxx;
	*intercept = my - *slope * mx;
	return true;
}

static bool _is_open(float l) {
	// also NaN
	return !(l > 0.0f) || (l > SMOL_MOTOR_ID_L_OPEN_H);
}

smol_motor_class_t smol_motor_id_classify(const float pair_l_h[SMOL_MOTOR_ID_PAIRS]) {
	smol_motor_class_t c = {.type = SMOL_MOTOR_TYPE_UNCLEAR, .s = 0.0f, .shared_terminal = -1, .coil_h = {0.0f, 0.0f, 0.0f}};

	size_t closed = 0;
	for (size_t k = 0; k < SMOL_MOTOR_ID_PAIRS; ++k)
		if (!_is_open(pair_l_h[k]))
			++closed;
	if (closed == 0) {
		c.type = SMOL_MOTOR_TYPE_NO_MOTOR;
		return c;
	}
	if (closed == 1) {
		c.type = SMOL_MOTOR_TYPE_SINGLE_COIL;
		return c;
	}
	if (closed == 2) {
		c.type = SMOL_MOTOR_TYPE_INCONSISTENT;
		return c;
	}

	// sort the pair indices by inductance: imin, imid, imax
	size_t idx[3] = {0, 1, 2};
	for (size_t i = 0; i < 2; ++i)
		for (size_t j = 0; j < 2 - i; ++j)
			if (pair_l_h[idx[j]] > pair_l_h[idx[j + 1]]) {
				size_t t = idx[j];
				idx[j] = idx[j + 1];
				idx[j + 1] = t;
			}
	float l_min = pair_l_h[idx[0]];
	float l_mid = pair_l_h[idx[1]];
	float l_max = pair_l_h[idx[2]];
	c.s = l_max / (l_mid + l_min);

	if (c.s > SMOL_MOTOR_ID_S_2PH) {
		c.type = SMOL_MOTOR_TYPE_2PH;
		size_t kmax = idx[2];
		c.shared_terminal = _pair_other[kmax];
		// the 2 coils are the 2 pairs with the shared terminal. Order: the coil to the lower non-shared terminal first.
		int t_lo = _pair_t0[kmax] < _pair_t1[kmax] ? _pair_t0[kmax] : _pair_t1[kmax];
		for (size_t k = 0; k < SMOL_MOTOR_ID_PAIRS; ++k) {
			if (k == kmax)
				continue;
			int other = (_pair_t0[k] == c.shared_terminal) ? _pair_t1[k] : _pair_t0[k];
			c.coil_h[(other == t_lo) ? 0 : 1] = pair_l_h[k];
		}
		return c;
	}
	if (c.s < SMOL_MOTOR_ID_S_3PH) {
		c.type = SMOL_MOTOR_TYPE_3PH;
		// L_t = (sum of the 2 pairs with t - the pair without t) / 2
		for (int t = 0; t < 3; ++t) {
			float with = 0.0f;
			float without = 0.0f;
			for (size_t k = 0; k < SMOL_MOTOR_ID_PAIRS; ++k) {
				if (_pair_other[k] == t)
					without = pair_l_h[k];
				else
					with += pair_l_h[k];
			}
			c.coil_h[t] = 0.5f * (with - without);
		}
		return c;
	}
	c.type = SMOL_MOTOR_TYPE_UNCLEAR;
	return c;
}

const char* smol_motor_type_name(smol_motor_type_t type) {
	switch (type) {
		case SMOL_MOTOR_TYPE_2PH: return "2-phase";
		case SMOL_MOTOR_TYPE_3PH: return "3-phase";
		case SMOL_MOTOR_TYPE_NO_MOTOR: return "no motor (all pairs open)";
		case SMOL_MOTOR_TYPE_SINGLE_COIL: return "error: single coil (one pair closed)";
		case SMOL_MOTOR_TYPE_INCONSISTENT: return "error: inconsistent (two pairs closed, one open)";
		case SMOL_MOTOR_TYPE_UNCLEAR: return "error: unclear (s between the limits)";
		default: return "?";
	}
}
