#include "smol_servo.h"

/**
 * The TMC6200 current sense amplifier is susceptible to switching spikes for 2-4us. The 60kHz PWM is ~16.67us. We sample the channels at 120kHz intervals, centered on the pulse hi and low periods, so one of them should always be at more than 4us from an edge.
 */

#define ISENSE_SETTLING_CYCLES 600

#define ISENSE_SAMPLE_CYCLES 150 // half of the 2us sampling time

void smol_isense_update_phase_current(size_t phase, float vsense[2], float amplification, const uint32_t pwm_min_cycles, const uint32_t pwm_max_cycles, const uint32_t pwm_half_period_cycles) {
	// find out if low (0) or high (1) current sample is valid
	int32_t low_margin = pwm_min_cycles - ISENSE_SAMPLE_CYCLES -ISENSE_SETTLING_CYCLES;
	int32_t high_margin = pwm_half_period_cycles - pwm_max_cycles - ISENSE_SAMPLE_CYCLES - ISENSE_SETTLING_CYCLES;

	bool sample_valid[2] = {low_margin >= 0, high_margin >= 0};

	float sample_factor[2] = {sample_valid[0], sample_valid[1]};

	float vsense = (vsense[0]*sample_factor[0] + vsense[1]*sample_factor[1]) * (1.0f / (sample_factor[0] + sample_factor[1]));

	if (sample_factor <= 0.0f) {
		// no current read sample
	}

	float isense = 

}