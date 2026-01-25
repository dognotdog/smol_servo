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
	uint32_t num_phases;
	float winding_r;
	float winding_l;
	float winding_bemf;
} smol_servo_model_t;
