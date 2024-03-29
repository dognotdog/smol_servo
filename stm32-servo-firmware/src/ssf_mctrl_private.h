#ifndef SSF_MCTRL_PRIVATE_H
#define SSF_MCTRL_PRIVATE_H

#include "ssf_main.h"
#include "ssf_analog.h"
#include "ssf_linreg.h"

#include <math.h>
#include <float.h>
#include <stdbool.h>

// enum {
// 	ISENSE_ALO,
// 	ISENSE_AHI,
// 	ISENSE_BLO,
// 	ISENSE_BHI,
// 	ISENSE_CLO,
// 	ISENSE_VHI,
// 	ISENSE_COUNT,
// };

#define MCTRL_DRIVER_PHASES			3

#define PHASE_BUCKETS				16
#define NUM_STATIC_MEASUREMENTS		8
#define NUM_PHASE_MEASUREMENTS		16
#define NUM_ANGLE_MEASUREMENTS		32
#define NUM_MAPSTEP_MEASUREMENTS	(4*6*4)
#define NUM_ALIGN_WAIT_ON_CYCLES	33
#define NUM_ALIGN_WAIT_OFF_CYCLES	333

#define CALIBREADS 1000
#define NUM_INDUCTANCE_ID_CYCLES 20

#define EMFID_INITAL_SPEED	(10.0f * (2.0f*M_PI))
#define EMFID_RUN_TIME		(3.0f)


#define MEAS_SINGLE_PERIOD 		(10.0e-6f)
// #define MEAS_SAMPLES 		6
#define MEAS_FULL_PERIOD		(MEAS_SINGLE_PERIOD*ISENSE_COUNT)
#define NUM_ID_ALGO_SAMPLES		2
#define NUM_ID_ALGO_ESTIMATES 	(2*(NUM_STATIC_MEASUREMENTS - (NUM_ID_ALGO_SAMPLES-1)))
#define NUM_IDENTIFICATION_RUNS	6

#define MAX_IDENTIFICATION_REPEATS	32
#define IDELTA_MIN			0.001f
#define TARGET_VAR_NORM		0.05f





typedef enum {
	MCTRL_INIT,

	MCTRL_DRIVER_RESET_START,
	MCTRL_DRIVER_RESET_WAIT,
	MCTRL_DRIVER_RESET_COOLDOWN,

	MCTRL_DRIVER_DETECT_START,

	MCTRL_DRIVER_TEST_START,

	MCTRL_DRIVER_INIT_START,
	MCTRL_DRIVER_INIT_WAIT,
	MCTRL_DRIVER_CALIB_ENTER,
	MCTRL_DRIVER_CALIB_WAIT,
	MCTRL_DRIVER_CALIB_EXIT,
	MCTRL_DRIVER_CALIB_EXIT_WAIT,

	MCTRL_ANALOG_CALIBRATION_START,
	MCTRL_ANALOG_CALIBRATION_START_LO,
	MCTRL_ANALOG_CALIBRATION_RUN_LO,
	MCTRL_ANALOG_CALIBRATION_START_HI,
	MCTRL_ANALOG_CALIBRATION_RUN_HI,
	MCTRL_ANALOG_CALIBRATION_FINISH,


	MCTRL_ID_ALIGN_START,
	MCTRL_ID_ALIGN_WAIT,
	MCTRL_ID_ALIGN_SETTLE,
	MCTRL_ID_ALIGN_FINISH,

	MCTRL_RAMPTIME_ID_START,
	MCTRL_RAMPTIME_ID_WAIT,
	MCTRL_RAMPTIME_ID_FINISH,

	MCTRL_RESISTANCE_ID_PREPARE,
	MCTRL_RESISTANCE_ID_START,
	MCTRL_RESISTANCE_ID_RUN,
	MCTRL_RESISTANCE_ID_FINISH,

	MCTRL_INDUCTANCE_ID_PREPARE,
	MCTRL_INDUCTANCE_ID_START,
	MCTRL_INDUCTANCE_ID_RUN,
	MCTRL_INDUCTANCE_ID_FINISH,

	MCTRL_IMPEDANCE_ID_PREPARE,
	MCTRL_IMPEDANCE_ID_START,
	MCTRL_IMPEDANCE_ID_RUN,
	MCTRL_IMPEDANCE_ID_FINISH,

	MCTRL_SYSID_FINISH,
	MCTRL_SYSID_DONE,

	// pole-pair identification (angle per step)
	MCTRL_PPID_PREPARE,
	MCTRL_PPID_START,
	MCTRL_PPID_RUN,
	MCTRL_PPID_FINISH,

	MCTRL_COGID_PREPARE,
	MCTRL_COGID_START,
	MCTRL_COGID_RUN,
	MCTRL_COGID_FINISH,

	// step nonlinearities
	MCTRL_MAPSTEP_PREPARE,
	MCTRL_MAPSTEP_START,
	MCTRL_MAPSTEP_RUN,
	MCTRL_MAPSTEP_FINISH,


	MCTRL_EMF_PREPARE,
	MCTRL_EMF_STALL_RAMP,
	MCTRL_EMF_STALL_EVAL,
	MCTRL_EMF_START,
	MCTRL_EMF_RAMP,
	MCTRL_EMF_DECELERATE,
	MCTRL_EMF_RUN,
	MCTRL_EMF_FINISH,


	MCTRL_DEMO,
	// MCTRL_
} mctrl_state_t;

typedef enum {
	MCTRL_MOT_UNKNOWN,
	MCTRL_MOT_2PH,
	MCTRL_MOT_3PH_SIN,
	MCTRL_MOT_3PH_TRAP,
	MCTRL_MOT_
} mctrl_motor_type_t;

// bridge status low, high impedance (off), and high
typedef enum {
	MCTRL_BRIDGE_LO = -1,
	MCTRL_BRIDGE_ZZ,
	MCTRL_BRIDGE_HI,
} mctrl_bridge_activation_t;


typedef struct {
	struct {
		float maxCurrent;
		float staticIdentificationDutyCycle;
		size_t maxRampCycles;
		mctrl_bridge_activation_t idSequence[NUM_IDENTIFICATION_RUNS][MCTRL_DRIVER_PHASES];
	} sysId;
} mctrl_params_t;

typedef struct {
	float pwm[3];
	float u[3];
} mctrl_pwm_t;

typedef struct {
	struct {
		struct {
			float Lest[NUM_IDENTIFICATION_RUNS];
			float Rest[NUM_IDENTIFICATION_RUNS];
			float Lvar[NUM_IDENTIFICATION_RUNS];
			float Rvar[NUM_IDENTIFICATION_RUNS];
		} phases;
		mctrl_motor_type_t motorType;
		size_t stepsPerRev;
	} sysParamEstimates;

	float angleSum;
	float currentSqrSum[ISENSE_NUMCHANNELS];

	// EMF stall ramp
	float stallSpeed;
	incrementalLinreg_t emfRegressions[ISENSE_NUMCHANNELS];

	size_t idRunCounter;
	// float adcZeroCalibs[ISENSE_COUNT];
	volatile size_t calibCounter;
	volatile float lastMeasurement[NUM_STATIC_MEASUREMENTS][ISENSE_NUMCHANNELS];
	volatile float lastVbus[NUM_STATIC_MEASUREMENTS];

	float stepmap_i[NUM_MAPSTEP_MEASUREMENTS];
	float stepmap_a[NUM_MAPSTEP_MEASUREMENTS];

	float phasePwm[NUM_PHASE_MEASUREMENTS];
	float phaseCurrents[NUM_PHASE_MEASUREMENTS][ISENSE_NUMCHANNELS];

	float demoPhasedCurrents[ISENSE_NUMCHANNELS][PHASE_BUCKETS];
	float phase;
	mctrl_pwm_t pwm;
	size_t counter;

	struct {
		// debugging counters
		uint16_t lastEventCount;
		uint16_t lastEventCountDelta;
		uint32_t lastControlStepTime_us;
		uint32_t lastControlDelta_us;

		float iiPhaseEstimate;
	} debug;

} mctrl_controller_t;


extern mctrl_params_t mctrl_params;

extern volatile mctrl_state_t mctrl_state;
extern mctrl_controller_t mctrl;

static inline void mctrl_enableBridges(mctrl_bridge_activation_t bridges[MCTRL_DRIVER_PHASES])
{
	uint32_t enables = ((bridges[0] != MCTRL_BRIDGE_ZZ) << 0)
					 | ((bridges[1] != MCTRL_BRIDGE_ZZ) << 1)
					 | ((bridges[2] != MCTRL_BRIDGE_ZZ) << 2);
	spwm_enableHalfBridges(enables);
}

static inline size_t mctrl_loBridge(const mctrl_bridge_activation_t bridges[MCTRL_DRIVER_PHASES])
{
	return (bridges[0] == MCTRL_BRIDGE_LO)*0
		 + (bridges[1] == MCTRL_BRIDGE_LO)*1
		 + (bridges[2] == MCTRL_BRIDGE_LO)*2;
}

static inline size_t mctrl_hiBridge(const mctrl_bridge_activation_t bridges[MCTRL_DRIVER_PHASES])
{
	return (bridges[0] == MCTRL_BRIDGE_HI)*0
		 + (bridges[1] == MCTRL_BRIDGE_HI)*1
		 + (bridges[2] == MCTRL_BRIDGE_HI)*2;
}

void mctrl_updateSimpleSensorEstimate(uint32_t now_us);
float mctrl_getSimpleMotorSpeedEstimate(void);

mctrl_pwm_t mctrl_setPhasorPwmSin(float phase, float vRmsPhase, mctrl_motor_type_t motorType);
void mctrl_getPhasorVoltagesSin(float phase, mctrl_motor_type_t motorType, float v[3]);

void mctrl_fastloopProcessHallSensor(void);

#endif // SSF_MCTRL_PRIVATE_H
