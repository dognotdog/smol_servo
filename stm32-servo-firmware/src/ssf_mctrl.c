
#include "ssf_main.h"
#include "ssf_spi.h"
#include "debug.h"

#include <math.h>
#include <stdbool.h>
#include <string.h>


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
#define NUM_STATIC_MEASUREMENTS		4
#define NUM_ALIGN_WAIT_ON_CYCLES	33
#define NUM_ALIGN_WAIT_OFF_CYCLES	333

#define CALIBREADS 3000

static float _phasedCurrents[6][PHASE_BUCKETS];
static float _phase = 0.0f;
static size_t _counter = 0;

static float _adcZeroCalibs[6];
static volatile size_t _calibCounter = 0;

static volatile float _lastMeasurement[NUM_STATIC_MEASUREMENTS][ISENSE_COUNT];



/*
Phase Recovery

	ia = i*cos(x)
	ib = i*sin(x)

	ia^2 + ib^2 = i^2
	x = atan(ib,ia)

Motor Identification

	Static: 
		Hold 2 outputs low, pulse one output high, cycle through each and record values.

		Resistance / Inductance
			Inductance: varying length single (or triple, in this case) pulses per phase
			u = L * di/dt

			Resistance: steady-state current value after many pulses


		Configuration:
			3-phase or 2-phase 

	Dynamic:
		Torque Constant?


Our dual-slope PWM (up & down) runs at 100kHz per slope, so 50kHz for a full PWM cycle.

We make a measurement per 100kHz, aka 10us, twice per channel, each round completed every 60us


Single Dimensional Kalman Estimation

Measurements are x taken sigma^2 variance, and x1 is our current estimate, and x2 is a new measurement, the new x = x1 + K (x2 - x1), where K = sigma1^2 / (sigma1^2 + sigma2^2) and the new sigma^2 = (1 - K) sigma1^2


*/

#define MEAS_SINGLE_PERIOD 	10.0e-6f
#define MEAS_SAMPLES 		6
#define MEAS_FULL_PERIOD	(MEAS_SINGLE_PERIOD*MEAS_SAMPLES)

#define NUM_IDENTIFICATION_RUNS	3

#define MAX_IDENTIFICATION_REPEATS	32
#define IDELTA_MIN			0.001f
#define TARGET_VAR_NORM		0.05

typedef enum {
	MCTRL_INIT,
	MCTRL_ANALOG_CALIBRATION_RUN,
	MCTRL_ANALOG_CALIBRATION_FINISH,
	MCTRL_INDUCTANCE_IDA_START,
	MCTRL_INDUCTANCE_IDA_ALIGN,
	MCTRL_INDUCTANCE_IDA_ALIGN_WAIT,
	MCTRL_INDUCTANCE_IDA_RUN,
	MCTRL_INDUCTANCE_IDA_FINISH,
	MCTRL_DEMO,
	MCTRL_
} mctrl_state_t;


static size_t _idRunCounter = 0;

static volatile mctrl_state_t _state = MCTRL_INIT;

static float _kalmanInductanceMeans[3];
static float _kalmanInductanceVars[3];


static inline float _calcInductance(float di, float dt, float u)
{
	float L = u / di * dt;
	return L;
}

void mctrl_init(void)
{
	// reset state
	_state = MCTRL_INIT;

	spwm_setDrvChannel(HTIM_DRV_CH_R, 0.0f);
	spwm_setDrvChannel(HTIM_DRV_CH_B, 0.0f);
	spwm_setDrvChannel(HTIM_DRV_CH_A, 0.0f);
	spwm_setDrvChannel(HTIM_DRV_CH_C, 0.0f);

	memset(_adcZeroCalibs, 0, sizeof(_adcZeroCalibs));
	_calibCounter = 0;
	// allow things to settle a bit
	HAL_Delay(20);

	// first is analog recalibration
	// put the DRV8323 into calibration mode
	// switchover in DRV8323 takes 100us
	// do calibration twice, as otherwise we seem to get flip-flopping between ~2047 and ~2070 ADC counts, it is more stable this way, but not perfect, for some reason SOC still seems to be unstable
	ssf_enterMotorDriverCalibrationMode();

	HAL_Delay(2);
	ssf_exitMotorDriverCalibrationMode();
	ssf_enterMotorDriverCalibrationMode();

	HAL_Delay(2);

	// then read out ADCs for zero calib
	// average 16 reads (2 per fastLoop)
	_state = MCTRL_ANALOG_CALIBRATION_RUN;

	// wait for calibration measurements to finish from fastLoop
	while (_state != MCTRL_ANALOG_CALIBRATION_FINISH) {};


	// average out the 16 reads for an adc count calibration
	for (size_t i = 0; i < 3; ++i)
	{
		float calib = (_adcZeroCalibs[2*i]+_adcZeroCalibs[2*i+1])*(1.0f/CALIBREADS);
		_adcZeroCalibs[2*i] = calib;
		_adcZeroCalibs[2*i+1] = calib;

		dbg_println("SO[%u] calibrated to 0A = %.3f counts", i, (double)calib);
	}

	ssf_exitMotorDriverCalibrationMode();


	// driver calibration is now done, we can move on to motor identification
	// do identification runs for each phase
	for (_idRunCounter = 0; _idRunCounter < NUM_IDENTIFICATION_RUNS; ++_idRunCounter)
	{
		float inductanceEstimate = 0.0f;
		float inductanceVariance = 0.0f;
		for (size_t k = 0; k < MAX_IDENTIFICATION_REPEATS; ++k)
		{
			memset((void*)_lastMeasurement, 0, sizeof(_lastMeasurement));
			_state = MCTRL_INDUCTANCE_IDA_START;
			while (_state != MCTRL_INDUCTANCE_IDA_FINISH) {};



			// for (size_t j = 0; j < NUM_STATIC_MEASUREMENTS; ++j)
			// {
			// 	for (size_t i = 0; i < 3; ++i)
			// 	{

			// 		dbg_println("SO[%u] reads %8.3f,  %8.3f", i, (double)_lastMeasurement[j][2*i], (double) _lastMeasurement[j][2*i+1]);
			// 	}
			// }

			float u = 12.0;
			float inductances[(NUM_STATIC_MEASUREMENTS-1)*2] = {0.0f};

			size_t numValidInductances = 0;

			for (size_t j = 1; j < NUM_STATIC_MEASUREMENTS; ++j)
			{

				float delta0 = _lastMeasurement[j-1][0+2*_idRunCounter] - _lastMeasurement[j-0][2*_idRunCounter];
				float delta1 = _lastMeasurement[j-1][1+2*_idRunCounter] - _lastMeasurement[j-0][1+2*_idRunCounter];

				if (delta0 >= IDELTA_MIN) 
				{
					float L0 = _calcInductance(delta0, MEAS_FULL_PERIOD, u);
					inductances[numValidInductances++] = L0;
				}
				if (delta1 >= IDELTA_MIN) 
				{
					float L1 = _calcInductance(delta1, MEAS_FULL_PERIOD, u);
					inductances[numValidInductances++] = L1;
				}
			}

			float measurementMean = 0.0;
			for (size_t i = 0; i < numValidInductances; ++i)
			{
				float L = inductances[i];
				measurementMean += L;
			}

			measurementMean *= 1.0f/(numValidInductances);

			float measurementVar = 0.0f;
			for (size_t i = 0; i < numValidInductances; ++i)
			{
				float L = inductances[i];
				float LE = (L - measurementMean);
				measurementVar += LE*LE;
			}

			measurementVar *= 1.0f/(numValidInductances);


			if (k == 0)
			{
				// set initial values
				inductanceEstimate = measurementMean;
				inductanceVariance = measurementVar;
			}
			else
			{
				// update estimate via Kalman Filter
				/*
				K = sigma1^2 / (sigma1^2 + sigma2^2)
				x = x1 + K (x2 - x1)
				sigma^2 = (1 - K) sigma1^2
				*/
				float K = inductanceVariance / (inductanceVariance +  measurementVar);
				inductanceEstimate += K * (measurementMean - inductanceEstimate);
				inductanceVariance *= 1.0f - K;
			}

			_kalmanInductanceMeans[_idRunCounter] = inductanceEstimate;
			_kalmanInductanceVars[_idRunCounter] = inductanceVariance;

			dbg_println("Kalman Estimate is  %.3f mH, sigma = %.3f mH", (double)(inductanceEstimate*1e3), (double)(sqrtf(inductanceVariance)*1e3));

			if (inductanceVariance < inductanceEstimate*inductanceEstimate*TARGET_VAR_NORM*TARGET_VAR_NORM)
			{
				dbg_println("Reached required limit.");
				break;
			}
		}
	}



	_state = MCTRL_DEMO;
}

float* mctrl_getPhaseTable(size_t i)
{
	return _phasedCurrents[i];
}


static float _getCurrentSenseFactor(void)
{
	// default amp gain factor is G = 20 (5/10/20/40) are available
	// current sense shunts are Rs = 15mOhm
	// sense amp output is centered on VREF/2, max. 0.25V away from rail
	// thus +- 1.4V usable range
	// for 12V operation, 10A would give 150mV, factor of 10 gives full range for 8A
	// return Rs * G
	return 1.0/(0.015f*20.0f);
}


static void _convertAdcToCurrent(volatile float currents[6], const uint16_t adcCounts[6])
{
	float gain = (1.0f/4095.0f)*3.3f*_getCurrentSenseFactor();
	for (size_t i = 0; i < 6; ++i)
	{
		float adcCount = (float)(adcCounts[i]);

		float current = (adcCount - _adcZeroCalibs[i])*gain;
		currents[i] = current;
	}
}

void mctrl_fastLoop(const uint16_t adcCounts[6])
{

	switch (_state)
	{
		case MCTRL_ANALOG_CALIBRATION_RUN:
		{
			if (_calibCounter < CALIBREADS/2)
			{
				for (size_t i = 0; i < 6; ++i)
				{
					_adcZeroCalibs[i] += adcCounts[i];
				}
				++_calibCounter;	
			}
			else
			{
				_state = MCTRL_ANALOG_CALIBRATION_FINISH;
			}

			break;
		}
		case MCTRL_INDUCTANCE_IDA_START:
		{
			// full PWM on the tested phase for a single fastLoop cycle to nudge it into alignment
			if (_calibCounter < NUM_ALIGN_WAIT_OFF_CYCLES)
			{
				spwm_setDrvChannel(HTIM_DRV_CH_A, 0.0);
				spwm_setDrvChannel(HTIM_DRV_CH_B, 0.0);
				spwm_setDrvChannel(HTIM_DRV_CH_C, 0.0);

				++_calibCounter;
			}
			else
			{
				// spwm_setDrvChannel(HTIM_DRV_CH_A, _idRunCounter != 0);
				// spwm_setDrvChannel(HTIM_DRV_CH_B, _idRunCounter != 1);
				// spwm_setDrvChannel(HTIM_DRV_CH_C, _idRunCounter != 2);


				_calibCounter = 0;
				_state = MCTRL_INDUCTANCE_IDA_ALIGN;
			}

			break;
		}
		case MCTRL_INDUCTANCE_IDA_ALIGN:
		{
			// turn current off again
			// wait for some time to complete alignment
			if (_calibCounter < NUM_ALIGN_WAIT_ON_CYCLES)
			{
				float ramp = 1.0f - fabsf(_calibCounter - NUM_ALIGN_WAIT_ON_CYCLES/2);
				spwm_setDrvChannel(HTIM_DRV_CH_A, ramp*(_idRunCounter != 0));
				spwm_setDrvChannel(HTIM_DRV_CH_B, ramp*(_idRunCounter != 1));
				spwm_setDrvChannel(HTIM_DRV_CH_C, ramp*(_idRunCounter != 2));

				++_calibCounter;
			}
			else
			{
				spwm_setDrvChannel(HTIM_DRV_CH_A, 0.0);
				spwm_setDrvChannel(HTIM_DRV_CH_B, 0.0);
				spwm_setDrvChannel(HTIM_DRV_CH_C, 0.0);

				_calibCounter = 0;
				_state = MCTRL_INDUCTANCE_IDA_ALIGN_WAIT;
			}
			break;
		}
		case MCTRL_INDUCTANCE_IDA_ALIGN_WAIT:
		{
			// wait for some time to complete alignment
			if (_calibCounter < NUM_ALIGN_WAIT_OFF_CYCLES)
			{
				++_calibCounter;
			}
			else
			{
				spwm_setDrvChannel(HTIM_DRV_CH_A, _idRunCounter != 0);
				spwm_setDrvChannel(HTIM_DRV_CH_B, _idRunCounter != 1);
				spwm_setDrvChannel(HTIM_DRV_CH_C, _idRunCounter != 2);

				_calibCounter = 0;
				_state = MCTRL_INDUCTANCE_IDA_RUN;
			}
			break;
		}
		case MCTRL_INDUCTANCE_IDA_RUN:
		{
			if (_calibCounter < NUM_STATIC_MEASUREMENTS)
			{
				_convertAdcToCurrent(_lastMeasurement[_calibCounter], adcCounts);
				++_calibCounter;
			}
			else
			{
				spwm_setDrvChannel(HTIM_DRV_CH_A, 0.0);
				spwm_setDrvChannel(HTIM_DRV_CH_B, 0.0);
				spwm_setDrvChannel(HTIM_DRV_CH_C, 0.0);

				_state = MCTRL_INDUCTANCE_IDA_FINISH;
			}
			break;
		}
		case MCTRL_DEMO:
		{
			float currents[6] = {0.0f};
			_convertAdcToCurrent(currents, adcCounts);

			// store phase currents for debugging purposes
			for (size_t i = 0; i < 6; ++i)
			{
				size_t pidx = ((int)(_phase*((PHASE_BUCKETS)/(M_PI*2.0f)))) % PHASE_BUCKETS;
				_phasedCurrents[i][pidx] = currents[i];
			}

			// "control" right now is just constant speed moves

			const float step = M_PI/(100.0e3/6.0/10)*0.25f;

			// keep the brake resistor off
			spwm_setDrvChannel(HTIM_DRV_CH_R, 0.0f);

			// keep B at 50% for 2-phase drivering
			spwm_setDrvChannel(HTIM_DRV_CH_B, 0.5f);

			// vary A,C with sines
			spwm_setDrvChannel(HTIM_DRV_CH_A, 0.5f+0.1f*sintab(_phase));
			spwm_setDrvChannel(HTIM_DRV_CH_C, 0.5f+0.1f*sintab(_phase + M_PI*0.5f));

			++_counter;
			_phase += step;

			break;
		}
	}

}

