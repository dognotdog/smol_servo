
#include "ssf_main.h"
#include "ssf_spi.h"

#include <math.h>
#include <stdbool.h>

#define PHASE_BUCKETS	16

static float _phasedCurrents[6][PHASE_BUCKETS];
static float _phase = 0.0f;
static size_t _counter = 0;

static float _adcZeroCalibs[6];
static volatile size_t _calibCounter = 0;

static volatile bool _doCalibration = false;

static volatile bool _isAnalogCalibrationComplete = false;



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
			
			Resistance: steady-state current value after many pulses


		Configuration:
			3-phase or 2-phase 

	Dynamic:
		Torque Constant?

*/


void mctrl_init(void)
{
	// start by putting the DRV8323 into calibration mode
	// switchover in DRV8323 takes 100us
	ssf_enterMotorDriverCalibrationMode();

	HAL_Delay(2);

	// then read out ADCs for zero calib
	// average 16 reads (2 per fastLoop)
	_doCalibration = true;
	while (_calibCounter < 8)
	{};


	// average out the 16 reads for an adc count calibration
	for (size_t i = 0; i < 3; ++i)
	{
		float calib = (_adcZeroCalibs[2*i]+_adcZeroCalibs[2*i+1])*(1.0f/16.0f);
		_adcZeroCalibs[2*i] = calib;
		_adcZeroCalibs[2*i+1] = calib;
	}

	ssf_exitMotorDriverCalibrationMode();

	_isAnalogCalibrationComplete = true;
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

void mctrl_fastLoop(const uint16_t adcCounts[6])
{



	if (!_isAnalogCalibrationComplete && _doCalibration)
	{
		if (_calibCounter < 8)
		{
			for (size_t i = 0; i < 6; ++i)
			{
				_adcZeroCalibs[i] += adcCounts[i];
			}
			++_calibCounter;	
		}
	}
	else
	{
		float gain = (1.0f/4095.0f)*3.3f*_getCurrentSenseFactor();
		float currents[6] = {0.0f};
		for (size_t i = 0; i < 6; ++i)
		{
			float adcCount = (float)(adcCounts[i]);

			float current = (adcCount - _adcZeroCalibs[i])*gain;
			currents[i] = current;
		}

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

	}


}

