#include "smol_servo"

/**
 * # Modeling
 * 
 * Each phase is assumed to be an `R-L-BEMF(phase)` series circuit.
 * 
 * We also have to model zero offsets for the current sensors.
 * 
 * Sensors:
 * - current sense voltage
 * - MAG encoder position
 * - bus voltage
 * 
 * Control:
 * - Bridge PWM
 * 
 * Hidden:
 * - motor characteristics
 *   - R, L, BEMF(phase)
 *   - rotational inertia / mass
 * - driver characteristics
 *   - Bridge RDS_on
 * - rotor position / velocity
 * - torque angle
 * 
 * We have to be careful make the model observable
 * 
 * # Initialization
 * 
 * Even if we try to track every aspect of the model, we have to initialize somehow
 * 
 */

typedef struct {
	// last prediction of the model
	uint32_t t_clocks;

	uint32_t num_phases;
	float winding_r;
	float winding_l;
	float winding_bemf;

	float vsense_zero[3];
	float vsense_zero_p[3];
	float vsense_zero_s;

	float vbus;
	float vbus_p;
	float vbus_s;

	float i_d;
	float i_q;
	float torque_constant;

	float i_d_p;
	float i_q_p;
	float torque_constant_p;

	// position referencing physical 
	float rotor_position;
	float rotor_velocity;
	float rotor_position_p;
	float rotor_velocity_p;
} smol_servo_model_t;

static smol_servo_model_t _model[2];

void smol_model_init(void) {
	// current sense
	for (size_t i = 0; i < 3; ++i) {
		_model.vsense_zero[i] = VSENSE_NOMINAL_ZERO_V;
		_model.vsense_zero_p[i] = VSENSE_ZERO_V_INIT_VARIANCE;
		_model.vsense[i] = VSENSE_NOMINAL_ZERO_V;
		_model.vsense_p[i] = VSENSE_ZERO_V_INIT_VARIANCE;
	}
	_model.vsense_zero_s = VSENSE_ZERO_V_UPDATE_VARIANCE;
	// vsense_s depends on motor speed? time constant?
	_model.vsense_s = VSENSE_ZERO_V_UPDATE_VARIANCE;

}

void smol_model_predict(uint32_t t_clocks, smol_servo_model_t* new, const smol_servo_model_t* old) {
	float h = (t_clocks - old->t_clocks)*(1.0f/150.0e6f);
	float hh = h*h;
	float hhhh = hh*hh;
	new->t_clocks = t_clocks;

	/** 
	 * ## torque / current / voltage
	 * 
	 * This is a little tricky because we're expected to be in a 3-phase Y or 2-phase configuration. Then we have to take into account the QD (Park) transform to determine torque producing components.
	 * 
	 * phasor_angle: angle between current phasor and electrical_position
	 * 
	 * BEMF(phasor_angle, speed) = k * speed * cos (phasor_angle)
	 * i_q = - i * sin (phasor_angle)
	 * i_d =   i * cos (phasor_angle)
	 * torque = - k * i * sin (phasor_angle) = k * i_q
	 * i * R = U_R
	 * di / dt L = U_L
	 * 
	 * U_phase[n] = BEMF[n] + U_R[n] + U_L[n]
	 * U_phase[n] = U_bus * PWM[n]
	 * 
	 * ### Wye-Delta Transform
	 * 
	 * Z_x encodes the windings, V_x encodes the BEMF.
	 * 
	 * We measure currents at nodes and voltages across nodes:
	 *  - i_A, i_B, i_C
	 *  - U_AB, U_BC, U_CA
	 * 
	 * 1(A)        2(B)     A-----Z_AB-----V_AB-----B
	 *   \         /         \                     /
	 *  Z_1       Z_2         \                   /
	 *     \     /             \                 /
	 *    V_1   V_2            V_CA           Z_BC
	 *       \ /                 \             /
	 *        * T                 \           /
	 *        |                    \         /
	 *       V_3                   Z_CA    V_BC
	 *        |                      \     /
	 *       Z_3                      \   /
	 *        |                        \ /
	 *       3(C)                       C
	 * 
	 * n = {1,2,3}
	 * U_[n] = T + Z_[n] * i_[n] + V_[n]
	 * 
	 * Currents must sum to zero.
	 * 0 = i_1 + i_2 + i_3
	 * 
	 *          Z_AB * Z_CA
	 * Z_1 = ------------------
	 *       Z_AB + Z_BC + Z_CA
	 * 
	 *          Z_AB * Z_BC
	 * Z_2 = ------------------
	 *       Z_AB + Z_BC + Z_CA
	 *
	 *          Z_BC * Z_CA
	 * Z_3 = ------------------
	 *       Z_AB + Z_BC + Z_CA
	 * 
	 * With the magic of linear math, we can derive the BEMF voltages:
	 * 
	 * V_1 = Z_1 * ( V_AB / Z_AB -  V_CA / Z_CA)
	 * V_2 = Z_2 * ( V_BC / Z_BC -  V_AB / Z_AB)
	 * V_3 = Z_3 * ( V_CA / Z_CA -  V_BC / Z_BC)
	 * 
	 * Then we can further simplify by assuming equal impedances:
	 * 
	 *       Z_W ^ 2   Z_W
	 * Z_Y = ------- = ---
	 *       3 * Z_W    3
     *                                     V_AB - V_CA
     * V_1 = Z_Y / Z_W * ( V_AB - V_CA ) = -----------
     *                                          3
     *
     * V_[x] = BEMF_[x] = k * speed * cos(phasor_angle[x])
     * 
     * And then also assuming the voltages are balanced, with equal magnitude but 120 degrees phase shifted.
     * 
     * Phase shift formulas:
     * cos(x) = sin(x + 90deg)
     * a sin (x + p) + b sin (x + q) = c sin (x + r)
     * c^2 = a^2 + b^2 + 2*a*b cos (p - q)
     *           a sin (p) + b sin (q)
     * tan (r) = ---------------------
     *           a cos (p) + b cos (q)
     * 
     * simplified for a = b = 1:
     * c^2 = 2 + 2 cos (p - q)
     *           sin (p) + sin (q)
     * tan (r) = -----------------
     *           cos (p) + cos (q)
     *
     * and further simplified for p = 90deg, q = -30deg: 
     * c^2 = 2 + 2 cos (120deg) = 2 - 1 = 1
     *           1 - 0.5         1       sqrt(3)
     * tan (r) = ----------- = ------- = -------
     *           sqrt(3) / 2   sqrt(3)      3
     * 
     * r = atan(sqrt(3) / 3) = 30deg
     *
     * and thus:
     * 
     * V_1 = |V[xx]| / 3 * (cos(x) - cos(x - 120deg)) 
     * V_1 = |V[xx]| / 3 * sin(x + 30deg)
     * V_1 = |V[xx]| / 3 * cos(x - 60deg)
     * 
     * ### Phasors
     * 
     * Using the `magnitude * e^(j * angle)` notation might help, eg.
	 * |U| e^(j x) = T + (R + j w L) * |i| e^(j y) + |V| e^(j z)
	 * 
     * 
	 * ## rotor position
	 * 
	 * q = 2 * num_phases / (num_slots * num_poles)
	 * 
	 * mechanical_position = (electrical_position * q) % 2pi
	 * 
	 *   a = i_q * k / I
	 *   v += h * a
	 *   x += v * h + 0.5 * h^2 * a
	 */
	float rotor_acceleration = old->i_q * old->torque_constant / old->rotor_inertia;
	new->rotor_velocity = old->rotor_velocity + hh * rotor_acceleration;
	new->rotor_position = h * old->rotor_velocity + 0.5f * hh * rotor_acceleration;

	float rotor_acceleration_p = old->i_q_p + old->torque_constant_p + old->rotor_inertia_p;
	new->rotor->rotor_position_p = old->rotor_position_p + hh * old->rotor_velocity_p + 0.25f * hhhh * rotor_acceleration_p;
	new->rotor->rotor_velocity_p = old->rotor_velocity_p + hh * rotor_acceleration_p;

	// VSENSE zeroing doesn't change value, but gets more uncertain
	for (size_t i = 0; i < 3; ++i) {
		new->vsense_zero_p[i] = old->vsense_zero_p[i] + old->vsense_zero_s*dt;
	}
}

void smol_model_observe_vsense(smol_servo_model_t* new, const smol_servo_model_t* old, size_t vsense_index, float vsense, float vsense_var) {


	new->i_d[vsense_index] = ?;
	new->i_q[vsense_index] = ?;
	new->vsense_zero[vsense_index] = ?;
}
