#ifndef SSF_MCTRL_PRIVATE_H
#define SSF_MCTRL_PRIVATE_H

#include "ssf_main.h"

#include <math.h>
#include <float.h>
#include <stdbool.h>

enum {
	ISENSE_ALO,
	ISENSE_AHI,
	ISENSE_BLO,
	ISENSE_BHI,
	ISENSE_CLO,
	ISENSE_VHI,
	ISENSE_COUNT,
};

#define PHASE_BUCKETS				16
#define NUM_STATIC_MEASUREMENTS		8
#define NUM_ALIGN_WAIT_ON_CYCLES	33
#define NUM_ALIGN_WAIT_OFF_CYCLES	333

#define CALIBREADS 3000




#define MEAS_SINGLE_PERIOD 	10.0e-6f
// #define MEAS_SAMPLES 		6
#define MEAS_FULL_PERIOD	(MEAS_SINGLE_PERIOD*ISENSE_COUNT)
#define NUM_ID_ALGO_SAMPLES	2
#define NUM_ID_ALGO_ESTIMATES (2*(NUM_STATIC_MEASUREMENTS - (NUM_ID_ALGO_SAMPLES-1)))
#define NUM_IDENTIFICATION_RUNS	3

#define MAX_IDENTIFICATION_REPEATS	32
#define IDELTA_MIN			0.001f
#define TARGET_VAR_NORM		0.05f





typedef enum {
	MCTRL_INIT,

	MCTRL_DRIVER_RESET_START,
	MCTRL_DRIVER_RESET_WAIT,

	MCTRL_DRIVER_INIT_START,
	MCTRL_DRIVER_INIT_WAIT,
	MCTRL_DRIVER_CALIB_ENTER,
	MCTRL_DRIVER_CALIB_WAIT,
	MCTRL_DRIVER_CALIB_EXIT,
	MCTRL_DRIVER_CALIB_EXIT_WAIT,

	MCTRL_ANALOG_CALIBRATION_RUN,
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

	MCTRL_SYSID_DONE,

	MCTRL_DEMO,
	// MCTRL_
} mctrl_state_t;

typedef enum {
	MCTRL_MOT_UNKNOWN,
	MCTRL_MOT_2PH,
	MCTRL_MOT_3PH,
	MCTRL_MOT_
} mctrl_motor_type_t;

typedef struct {
	struct {
		float maxCurrent;
		float staticIdentificationDutyCycle;
		size_t maxRampCycles;
	} sysId;
} mctrl_params_t;

typedef struct {
	struct {
		struct {
			float Lest[3];
			float Rest[3];
			float Lvar[3];
			float Rvar[3];
		} phases;
	} sysParamEstimates;

	size_t idRunCounter;
	float adcZeroCalibs[ISENSE_COUNT];
	volatile size_t calibCounter;
	volatile float lastMeasurement[NUM_STATIC_MEASUREMENTS][ISENSE_COUNT];
	volatile float lastVbus[NUM_STATIC_MEASUREMENTS];

	float phasedCurrents[ISENSE_COUNT][PHASE_BUCKETS];
	float phase;
	size_t counter;

	// debugging counters
	uint16_t lastEventCount;
	uint16_t lastEventCountDelta;
	uint32_t lastControlStepTime_us;
	uint32_t lastControlDelta_us;

} mctrl_controller_t;


extern mctrl_params_t mctrl_params;

extern volatile mctrl_state_t mctrl_state;
extern mctrl_controller_t mctrl;



#endif // SSF_MCTRL_PRIVATE_H
