
#include "ssf_mctrl_private.h"

#include "ssf_main.h"
#include "ssf_analog.h"
#include "ssf_spi.h"
#include "debug.h"
#include "ssf_perf.h"
#include "ssf_linreg.h"
#include "utime.h"

#include <math.h>
#include <float.h>
#include <stdbool.h>
#include <string.h>
#include <assert.h>
#include <stdlib.h>


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
				V = L * di/dt + R * i

			step response is
				V u(t) = L * di(t)/dt + R * i(t)
				-> i(t) = V / R * (1 - e^(-R/L * t))


			laplace:
				-> V 1/s = L * (s i(s) - i(0-)) + R * i(s)
				-> V/s = (L * s + R) i(s)

				                 V
				-> i(s) = ---------------
				          s * (L * s + R)

				               V/L
				-> i(s) = -------------
				          s * (s + R/L)

				               V/R
				-> i(s) = -------------
				          s * (s * L/R + 1)

			z transform:
                       z
				-> V ----- = L * (1 - z^-1) i(z) / T + R i(z)
				     z - 1

					   z
				-> V ----- = (L / T * (1 - z^-1) + R) * i(z)
					 z - 1

				              z               1
				-> i(z) = V ----- * ----------------------
				            z - 1   L / T * (1 - z^-1) + R

                              z                1
				-> i(z) = V ----- * -------------------------
				            z - 1    L / T + R - L / T * z^-1

                              z             T / L
				-> i(z) = V ----- * --------------------
				            z - 1   1 + R * T / L - z^-1

                                        z              1
				-> i(z) = T * V / L * ----- * ---------------------
				                      z - 1   1 + R * T / L - z^-1

                                         1                1
				-> i(z) = T * V / L * -------- * --------------------
				                      1 - z^-1   1 + R * T / L - z^-1

                                         1               1
				-> i(z) = T * V / L * -------- * ------------------
				                      1 - z^-1   (L + R*T)/L - z^-1

                              V           1                1
				-> i(z) = --------- * -------- * --------------------
				          (L/T + R)   1 - z^-1   1 - L/(L + R*T) z^-1

                              V                           1
				-> i(z) = --------- * -------------------------------------------
				          (L/T + R)   1 - 2 * L/(L + R*T) z^-1 + L/(L + R*T) z^-2

			differential form
				        V - R i
				di/dt = -------
				           L

				                        V            R
				(1 - z^-1)/T i(z) = L{ --- u(t) } - --- * i(z)
				                        L            L

				1-z^-1         V    1       R
				------ i(z) = --- ------ - --- * i(z)
				   T           L  1-z^-1    L

				 / 1-z^-1    R  \          V    1     
				|  ------ + ---  | i(z) = --- ------
				 \    T      L  /          L  1-z^-1

				 / L (1-z^-1) + R T \          V    1     
				|  ----------------  | i(z) = --- ------
				 \       T L        /          L  1-z^-1

				        V    1             1
				i(z) = --- ------ * ----------------
				        T  1-z^-1   L (1-z^-1) + R T

				        V      1            1
				i(z) = --- * ------ * --------------
				       L T   1-z^-1       R T
				                      1 + --- - z^-1
				                           L

				        V      1            1
				i(z) = --- * ------ * --------------
				       L T   1-z^-1   L + R T
				                      ------- - z^-1
				                         L

				           V          1            1
				i(z) = ---------- * ------ * ----------------
				       T (L + RT)   1-z^-1          L
				                             1 - ------- z^-1
				                                 L + R T

				           V                           1
				i(z) = ---------- * ---------------------------------------
				       T (L + RT)                L                L
				                    1 - ( 1 + ------- ) z^-1 - ------- z^-2
				                              L + R T          L + R T


				             V                           
				-> num = ---------- 
				         T (L + RT)   

				                      L                L
				-> den = 1 - ( 1 + ------- ) z^-1 - ------- z^-2
				                   L + R T          L + R T




			Resistance: steady-state current value after many pulses


		Configuration:
			3-phase or 2-phase 

	Dynamic:
		Torque Constant?


Our dual-slope PWM (up & down) runs at 100kHz per slope, so 50kHz for a full PWM cycle.

We make a measurement per 100kHz, aka 10us, twice per channel, each round completed every 60us


Single Dimensional Kalman Estimation

Measurements are x taken sigma^2 variance, and x1 is our current estimate, and x2 is a new measurement, the new x = x1 + K (x2 - x1), where K = sigma1^2 / (sigma1^2 + sigma2^2) and the new sigma^2 = (1 - K) sigma1^2

Simultaneous L/R estimation

Using discrete samples i0,i1,i2,i3, and deriving ii1 = i2-i0 / 2T, ii2 = i3-i1 / 2T, iii = ii2-ii1 / T with sampling period T based on

	d i   V - R i
	--- = -------
	d t      L

	d^2 i     R d i
	----- = - -----
	d t^2     L d t

combining those two we get

               V
	L = ---------------
		ii - i iii / ii

           V ii
	L = ------------
		ii^2 - i iii 

                 i2 - i1
                 -------
                    T 
	L = V -----------------------
		  (i2 - i1)^2   i (ii2-ii1) 
		  ----------- - -----------
		      T^2            T
         
                        i2 - i1 
	L = V -----------------------------------
		  (i2 - i1)^2   (i2+i1) (i3-i1-i2+i0) 
		  ----------- - ---------------------
		       T                 4 T

                        i2 - i1 
	L = T V -----------------------------------
		                  (i2+i1) (i3-i1-i2+i0) 
		    (i2 - i1)^2 - ---------------------
		                            4

we can find that
	                    i2 - i1
	L = 2 T V ---------------------------
	          i2^2 - i0 i2 - i1 i3 + i1^2

	          i2 - i0 - i3 + i1
	R = V ---------------------------
	      i2^2 - i0 i2 - i1 i3 + i1^2

and with
	Var[x^2] = ( Var[x] + E[x]^2 )^2 - 2*E[x]^2
as well as 
	Var[X/Y] = E[(X/Y)^2] - E^2[X/Y]

	Var[L] = (2 T V)^2 (  )






Finding ABBC config values from measurements

Assuming an RH/RL being equal at Rt, Rp being being equal cross phases, and Ra,Rb,Rc being the measured values on each phase for 2 phases:
	Rp = (Ra - 4Rt + sqrt(Ra^2 + 4 Rt^2)) / 2 
	Rp = 2 Rb - 3 Rt

*/


mctrl_params_t mctrl_params = {
	.sysId = {
		.maxCurrent = 1.0f,
		.staticIdentificationDutyCycle = 0.1f,
		.maxRampCycles = NUM_STATIC_MEASUREMENTS,
		.idSequence = {
			{MCTRL_BRIDGE_LO, MCTRL_BRIDGE_HI, MCTRL_BRIDGE_ZZ},
			{MCTRL_BRIDGE_ZZ, MCTRL_BRIDGE_LO, MCTRL_BRIDGE_HI},
			{MCTRL_BRIDGE_HI, MCTRL_BRIDGE_ZZ, MCTRL_BRIDGE_LO},
			{MCTRL_BRIDGE_HI, MCTRL_BRIDGE_LO, MCTRL_BRIDGE_ZZ},
			{MCTRL_BRIDGE_ZZ, MCTRL_BRIDGE_HI, MCTRL_BRIDGE_LO},
			{MCTRL_BRIDGE_LO, MCTRL_BRIDGE_ZZ, MCTRL_BRIDGE_HI},
		},
	},
};

mctrl_controller_t mctrl = {
	.debug = {.lastEventCount = 0, },
};


volatile mctrl_state_t mctrl_state = MCTRL_INIT;

// static float _kalmanInductanceMeans[3];
// static float _kalmanInductanceVars[3];
// static float _kalmanResistanceMeans[3];
// static float _kalmanResistanceVars[3];

#if SSF_HARDWARE_VERSION == 0x000100

// emulate VBUS sensing on v0.1 hardware 
static inline float ssf_getVbus(void)
{
	return 12.0f;
}

#endif



static inline float _calcInductance(float di, float dt, float u)
{
	float L = u / di * dt;
	return L;
}


// reverse indexing of i_n
float _calculateInductance_n3(float i0, float i1, float i2, float V, float T, bool* isValid)
{
	// float den = i1*i1 - i0*i2;
	// float nom = T*V*(i0-i1);
	// *isValid = fabsf(den) > fabsf(nom)*FLT_EPSILON;
	// float L = -nom/den;

	float i = i1;
	// float i = (1.0f/3.0f)*(i0+i1+i2);
	float ii = 0.5f*(i2-i0);
	float iii = i2 - 2.0f*i1 + i0;

	float den = ii*ii - i*iii;
	float nom = T*V*(ii);
	*isValid = fabsf(den) > fabsf(nom)*FLT_EPSILON;
	float L = nom/den;

	return L;
}
float _calculateResistance_n3(float i0, float i1, float i2, float V, float T, bool* isValid)
{
	// float den = i1*i1 - i0*i2;
	// float nom = V*(2.0f*i1 - i0 - i2);
	// *isValid = fabsf(den) > fabsf(nom)*FLT_EPSILON;
	// float R = -nom/den;

	float i = i1;
	// float i = (1.0f/3.0f)*(i0+i1+i2);
	float ii = 0.5f*(i2-i0);
	float iii = i2 - 2.0f*i1 + i0;

	float den = ii*ii - i*iii;
	float nom = -V*(iii);
	*isValid = fabsf(den) > fabsf(nom)*FLT_EPSILON;
	float R = nom/den;

	return R;
}

float _calculateTimeConstant_n3(float i0, float i1, float i2, float V, float T, bool* isValid)
{
	float den = 2.0f*i1 - i0 - i2;
	float nom = 0.5*T*(i2-i0);
	*isValid = fabsf(den) > fabsf(nom)*FLT_EPSILON;
	float tau = nom/den;

	return tau;
}

float _calculateInductance(float i0, float i1, float i2, float i3, float V, float T, bool* isValid)
{
	float i = 0.5*(i1+i2);
	float ii1 = 0.5*(i2-i0);
	float ii2 = 0.5*(i3-i1);
	float iii = ii2-ii1;
	float ii = i2-i1;

	float den = ii*ii - i*iii;
	float nom = T*V*(ii);
	*isValid = fabsf(den) > fabsf(nom)*FLT_EPSILON;
	float L = -nom/den;

	return L;
}

float _calculateTimeConstant(float i0, float i1, float i2, float i3, float V, float T, bool* isValid)
{
	// float i = 0.5*(i1+i2);
	float ii1 = 0.5*(i2-i0);
	float ii2 = 0.5*(i3-i1);
	float iii = ii2-ii1;
	float ii = i2-i1;

	float den = iii;
	*isValid = fabsf(den) > fabsf(T*ii)*FLT_EPSILON;
	float tau = -T*ii/den;

	return tau;
}


float _calculateInductance2(float i0, float i1, float i2, float i3, float V, float T, bool* isValid)
{
	// float i = 0.5*(i1+i2);
	float ii1 = 0.5*(i2-i0);
	float ii2 = 0.5*(i3-i1);
	float ii = i2-i1;

	float den = i1*ii2-i2*ii1;
	float nom = T*V*(ii);
	*isValid = fabsf(den) > fabsf(nom)*FLT_EPSILON;

	float L = nom/den;

	return L;
}


float _calculateResistance(float i0, float i1, float i2, float i3, float V, float T)
{
	float i = 0.5*(i1+i2);
	float ii1 = 0.5*(i2-i0);
	float ii2 = 0.5*(i3-i1);
	float iii = ii2-ii1;
	float ii = i2-i1;

	float R = V*(iii)/(ii*ii - i*iii);

	return R;
}

float _calculateResistance2(float i0, float i1, float i2, float i3, float V, float T)
{
	// float i = 0.5*(i1+i2);
	float ii1 = 0.5*(i2-i0);
	float ii2 = 0.5*(i3-i1);
	float iii = ii2-ii1;
	// float ii = i2-i1;

	float R = V*(iii)/(i2*ii1-i1*ii2);

	return R;
}

void mctrl_init(void)
{
	mctrl_state = MCTRL_DRIVER_RESET_START;
}

float* mctrl_getPhaseTable(size_t i)
{
	return mctrl.demoPhasedCurrents[i];
}


static float _getCurrentSenseFactor(void)
{
	// default amp gain factor is G = 20 (5/10/20/40) are available
	// current sense shunts are Rs = 15mOhm
	// sense amp output is centered on VREF/2, max. 0.25V away from rail
	// thus +- 1.4V usable range
	// also, the DRV8323 is inverting
	// for 12V operation, 10A would give 150mV, factor of 10 gives full range for 8A
	// return Rs * G
#if SSF_HARDWARE_VERSION < 0x00000600
	return -1.0f/(0.015f*20.0f);
#else
	// TMC6200 uses a 5x amplifier by default
	// sense resistor is Rs = 5mOhm
	return 1.0f/(0.005f*5.0f);
#endif
}


static void _convertAdcToCurrent(volatile float currents[ISENSE_COUNT], const uint16_t adcCounts[ISENSE_COUNT], /*const float lowSideOnFraction[ISENSE_COUNT],*/ const float vdda)
{
	float gain = (1.0f/ssfa_isenseMaxCount())*vdda*_getCurrentSenseFactor();
	for (size_t i = 0; i < ISENSE_COUNT; ++i)
	{
		float adcCount = (float)(adcCounts[i]);

		float current = (adcCount - mctrl.adcZeroCalibs[i])*gain;
		currents[i] = current;
	}
}

static void _compute2PhPwm(const float phase, float pwm[3])
{
	float sinx = sintab(phase);
	float cosx = sintab(phase + M_PI*0.5f);
	float b = 0.5f-pwm[1]*0.25f*(sinx+cosx);
	float a = b+pwm[0]*M_SQRT1_2*(cosx);
	float c = b+pwm[2]*M_SQRT1_2*(sinx);
	pwm[0] = a;
	pwm[1] = b;
	pwm[2] = c;
}

static void _compute3PhPwmSin(const float phase, float pwm[3])
{
	float sinx0 = sintab(phase);
	float sinx1 = sintab(phase + M_PI*2.0f/3.0f);
	float sinx2 = sintab(phase + M_PI*4.0f/3.0f);
	float a = 0.5f*pwm[0]*(sinx0);
	float b = 0.5f*pwm[1]*(sinx1);
	float c = 0.5f*pwm[2]*(sinx2);
	pwm[0] = 0.5f+a;
	pwm[1] = 0.5f+b;
	pwm[2] = 0.5f+c;
}

static void _compute3PhPwmSin3(const float phase, float pwm[3])
{
	float sinx0 = sintab(phase);
	float sinx1 = sintab(phase + M_PI*2.0f/3.0f);
	float sinx2 = sintab(phase + M_PI*4.0f/3.0f);

	float a = 0.5f*pwm[0]*(sinx0);
	float b = 0.5f*pwm[1]*(sinx1);
	float c = 0.5f*pwm[2]*(sinx2);
	pwm[0] = 0.5f+a;
	pwm[1] = 0.5f+b;
	pwm[2] = 0.5f+c;
}

static void _compute3PhPwmTrap(const float phase, float pwm[3])
{
	float x2pi = fmodf(phase, 2.0f*M_PI);
	float xtab = (x2pi+2.0f*M_PI)*(6.0f/(2.0f*M_PI));
	float x[3] = {
		fmodf(xtab, 6.0f),
		fmodf(xtab + 2.0f, 6.0f),
		fmodf(xtab + 4.0f, 6.0f)
	};

	// float tab[6] = {0.5,0.0,-0.5,-0.5,0.0,0.5};

	for (size_t i = 0; i < 3; ++i)
	{
		float y = 0.5f;;
		int xi = (int)x[i];
		switch(xi)
		{
			case 0:
				y = 0.5f;
				break;
			case 1:
				y = 0.5f - 1.0f*(x[i] - xi);
				break;
			case 2:
			case 3:
				y = -0.5f;
				break;
			case 4:
				y = -0.5f + 1.0f*(x[i] - xi);
				break;
			case 5:
				y = 0.5f;
				break;
		}
		pwm[i] = 0.5 + pwm[i]*y;
	}
}

/**
 * This function is supposed to compute the 2ph and 3ph target output voltages for maximum voltage swing
 */
void mctrl_getPhasorVoltagesSin(float phase, mctrl_motor_type_t motorType, float v[3])
{
	switch(motorType)
	{
		case MCTRL_MOT_2PH:
		{
			float sinv = M_SQRT2*sintab(phase + M_PI*1.25f); // 135deg behind A
			float sina = sintab(phase + M_PI*0.0f);
			float sinb = sintab(phase + M_PI*0.5f);
			float sinu = sinv + sina;
			float sinw = sinv + sinb;
			v[0] = sinu;
			v[1] = sinv;
			v[2] = sinw;

			break;
		}
		case MCTRL_MOT_3PH_SIN:
		{
			float sina = sintab(phase + M_PI*0.0f/3.0f);
			float sinb = sintab(phase + M_PI*2.0f/3.0f);
			float sinc = sintab(phase + M_PI*4.0f/3.0f);

			float sinmin = fminf(fminf(sina, sinb), sinc);
			float sinmax = fmaxf(fmaxf(sina, sinb), sinc);
			// phase for N at 3*omega
			float subphase = fmodf(phase*(3.0f/(2.0f*M_PI)), 1.0f);
			// create N around zero so we can later create phase voltages around 0.5 bus voltage
			float n = (subphase >= 0.5f) ? -0.5*M_SQRT3 + sinmax : 0.5*M_SQRT3 + sinmin;

			float sinu = -n + sina;
			float sinv = -n + sinb;
			float sinw = -n + sinc;

			v[0] = sinu;
			v[1] = sinv;
			v[2] = sinw;
			
			break;
		}
		case MCTRL_MOT_3PH_TRAP:
		{
			float ph = fmodf(phase/(M_PI*2.0f)*6.0f, 6.0f);
			float ph1 = fmodf(ph, 1.0f);
			switch ((int)ph)
			{
				case 0:
				{
					v[0] = ph1;
					v[1] = 0.0f;
					v[2] = 1.0f;
					break;
				}
				case 1:
				{
					v[0] = 1.0f;
					v[1] = 0.0f;
					v[2] = 1.0f - ph1;
					break;
				}
				case 2:
				{
					v[0] = 1.0f;
					v[1] = ph1;
					v[2] = 0.0f;
					break;
				}
				case 3:
				{
					v[0] = 1.0f - ph1;
					v[1] = 1.0f;
					v[2] = 0.0f;
					break;
				}
				case 4:
				{
					v[0] = 0.0f;
					v[1] = 1.0f;
					v[2] = ph1;
					break;
				}
				case 5:
				{
					v[0] = 0.0f;
					v[1] = 1.0f - ph1;
					v[2] = 1.0f;
					break;
				}
			}
			break;
		}
		default:
		{
			v[0] = 0.0f;
			v[1] = 0.0f;
			v[2] = 0.0f;
			break;
		}
	}
}

mctrl_pwm_t mctrl_setPhasorPwmSin(float phase, float vRmsPhase, mctrl_motor_type_t motorType)
{
	mctrl_pwm_t pwm = {};
	mctrl_getPhasorVoltagesSin(phase, motorType, pwm.u);

	float dc = M_SQRT1_2*vRmsPhase;

	pwm.pwm[0] = 0.5f+dc*pwm.u[0];
	pwm.pwm[1] = 0.5f+dc*pwm.u[1];
	pwm.pwm[2] = 0.5f+dc*pwm.u[2];

	spwm_setDrvChannel(HTIM_DRV_CH_A, pwm.pwm[0]);
	spwm_setDrvChannel(HTIM_DRV_CH_B, pwm.pwm[1]);
	spwm_setDrvChannel(HTIM_DRV_CH_C, pwm.pwm[2]);
			
	return pwm;
}


void mctrl_fastLoop(const uint16_t adcCounts[ISENSE_NUMCHANNELS])
{
	// check if we're taking too long to execute, which is bad
	if ((mctrl.debug.lastEventCount > 12) && (mctrl.debug.lastEventCountDelta != 6))
	{
		// err_println("mctrl.debug.lastEventCountDelta not 6 but %u", mctrl.debug.lastEventCountDelta);
		assert(0);
	}
	// assert((mctrl.debug.lastEventCount == 0) || (mctrl.debug.lastEventCountDelta == 6));

	uint32_t now = utime_now();
	uint16_t eventCount = perf_getEventCount();

	mctrl.debug.lastControlDelta_us = now - mctrl.debug.lastControlStepTime_us;
	mctrl.debug.lastControlStepTime_us = now;

	mctrl.debug.lastEventCountDelta = eventCount - mctrl.debug.lastEventCount;
	mctrl.debug.lastEventCount = eventCount;

	// float alpha = 0.0f;

	sspi_fastloop();

	mctrl_fastloopProcessHallSensor();

	switch (mctrl_state)
	{
		case MCTRL_ANALOG_CALIBRATION_RUN_LO:
		{
			if (mctrl.calibCounter < CALIBREADS/2)
			{
				for (size_t i = 0; i < ISENSE_NUMCHANNELS; ++i)
				{
					ssfa_isenseCalibrateAdcZeros(adcCounts, false);
				}
				++mctrl.calibCounter;	
			}
			else
			{
				mctrl_state = MCTRL_ANALOG_CALIBRATION_FINISH;
			}

			break;
		}
		case MCTRL_ANALOG_CALIBRATION_RUN_HI:
		{
			if (mctrl.calibCounter < CALIBREADS/2)
			{
				for (size_t i = 0; i < ISENSE_NUMCHANNELS; ++i)
				{
					ssfa_isenseCalibrateAdcZeros(adcCounts, true);
				}
				++mctrl.calibCounter;	
			}
			else
			{
				mctrl_state = MCTRL_ANALOG_CALIBRATION_FINISH;
			}

			break;
		}
		case MCTRL_ID_ALIGN_START:
		{
			// turn everything off to to start
			if (mctrl.calibCounter < NUM_ALIGN_WAIT_OFF_CYCLES)
			{
				spwm_setDrvChannel(HTIM_DRV_CH_A, 0.0);
				spwm_setDrvChannel(HTIM_DRV_CH_B, 0.0);
				spwm_setDrvChannel(HTIM_DRV_CH_C, 0.0);

				++mctrl.calibCounter;
			}
			else
			{
				mctrl.calibCounter = 0;
				mctrl_state = MCTRL_ID_ALIGN_WAIT;
			}

			break;
		}
		case MCTRL_ID_ALIGN_WAIT:
		{
			// ramp up current to align motor
			// wait for some time to complete alignment
			if (mctrl.calibCounter < NUM_ALIGN_WAIT_ON_CYCLES)
			{
				// ramp up the current to not cause a huge jerk bringing the rotor into alignment
				float ramp = 1.0f - fabsf((float)(mctrl.calibCounter - NUM_ALIGN_WAIT_ON_CYCLES/2));

				const mctrl_bridge_activation_t* bridges = mctrl_params.sysId.idSequence[mctrl.idRunCounter];
				spwm_setDrvChannel(HTIM_DRV_CH_A, ramp*(bridges[0] == MCTRL_BRIDGE_HI));
				spwm_setDrvChannel(HTIM_DRV_CH_B, ramp*(bridges[1] == MCTRL_BRIDGE_HI));
				spwm_setDrvChannel(HTIM_DRV_CH_C, ramp*(bridges[2] == MCTRL_BRIDGE_HI));

				++mctrl.calibCounter;
			}
			else
			{
				spwm_setDrvChannel(HTIM_DRV_CH_A, 0.0);
				spwm_setDrvChannel(HTIM_DRV_CH_B, 0.0);
				spwm_setDrvChannel(HTIM_DRV_CH_C, 0.0);

				mctrl.calibCounter = 0;
				mctrl_state = MCTRL_ID_ALIGN_SETTLE;
			}
			break;
		}
		case MCTRL_ID_ALIGN_SETTLE:
		{
			// wait for some time to complete alignment and have current die down
			if (mctrl.calibCounter < NUM_ALIGN_WAIT_OFF_CYCLES)
			{
				spwm_setDrvChannel(HTIM_DRV_CH_A, 0.0);
				spwm_setDrvChannel(HTIM_DRV_CH_B, 0.0);
				spwm_setDrvChannel(HTIM_DRV_CH_C, 0.0);
				++mctrl.calibCounter;
			}
			else
			{
				mctrl.calibCounter = 0;
				mctrl_state = MCTRL_ID_ALIGN_FINISH;
			}
			break;
		}

		case MCTRL_RAMPTIME_ID_START:
		{
			const mctrl_bridge_activation_t* bridges = mctrl_params.sysId.idSequence[mctrl.idRunCounter];
			spwm_setDrvChannel(HTIM_DRV_CH_A, (bridges[0] == MCTRL_BRIDGE_HI));
			spwm_setDrvChannel(HTIM_DRV_CH_B, (bridges[1] == MCTRL_BRIDGE_HI));
			spwm_setDrvChannel(HTIM_DRV_CH_C, (bridges[2] == MCTRL_BRIDGE_HI));

			mctrl.calibCounter = 0;
			mctrl_state = MCTRL_RAMPTIME_ID_WAIT;

			break;
		}
		case MCTRL_RAMPTIME_ID_WAIT:
		{
			// turn current off again
			// wait for some time to complete alignment
			if (mctrl.calibCounter < mctrl_params.sysId.maxRampCycles)
			{
				const mctrl_bridge_activation_t* bridges = mctrl_params.sysId.idSequence[mctrl.idRunCounter];
				size_t loBridgeId = mctrl_loBridge(bridges);

				spwm_setDrvChannel(HTIM_DRV_CH_A, (bridges[0] == MCTRL_BRIDGE_HI));
				spwm_setDrvChannel(HTIM_DRV_CH_B, (bridges[1] == MCTRL_BRIDGE_HI));
				spwm_setDrvChannel(HTIM_DRV_CH_C, (bridges[2] == MCTRL_BRIDGE_HI));

				float vdda = ssf_getVdda();
				_convertAdcToCurrent(mctrl.lastMeasurement[mctrl.calibCounter], adcCounts, vdda);

				float i0 = mctrl.lastMeasurement[mctrl.calibCounter][(2*loBridgeId+0) % ISENSE_COUNT];
				float i1 = mctrl.lastMeasurement[mctrl.calibCounter][(2*loBridgeId+1)  % ISENSE_COUNT];

				float i = 0.5*(i0+i1);

				// stop if we exceed max current
				if (fabsf(i) > mctrl_params.sysId.maxCurrent)
				{
					spwm_setDrvChannel(HTIM_DRV_CH_A, 0.0);
					spwm_setDrvChannel(HTIM_DRV_CH_B, 0.0);
					spwm_setDrvChannel(HTIM_DRV_CH_C, 0.0);

					mctrl_state = MCTRL_RAMPTIME_ID_FINISH;
				}
				else
					++mctrl.calibCounter;
			}
			else
			{
				spwm_setDrvChannel(HTIM_DRV_CH_A, 0.0);
				spwm_setDrvChannel(HTIM_DRV_CH_B, 0.0);
				spwm_setDrvChannel(HTIM_DRV_CH_C, 0.0);

				mctrl_state = MCTRL_RAMPTIME_ID_FINISH;
			}
			break;
		}
		case MCTRL_INDUCTANCE_ID_START:
		{
			// wait for things to settle
			if (mctrl.calibCounter < NUM_ALIGN_WAIT_OFF_CYCLES)
			{
				spwm_setDrvChannel(HTIM_DRV_CH_A, 0.0);
				spwm_setDrvChannel(HTIM_DRV_CH_B, 0.0);
				spwm_setDrvChannel(HTIM_DRV_CH_C, 0.0);

				++mctrl.calibCounter;
			}
			else
			{
				const mctrl_bridge_activation_t* bridges = mctrl_params.sysId.idSequence[mctrl.idRunCounter];
				spwm_setDrvChannel(HTIM_DRV_CH_A, (bridges[0] == MCTRL_BRIDGE_HI));
				spwm_setDrvChannel(HTIM_DRV_CH_B, (bridges[1] == MCTRL_BRIDGE_HI));
				spwm_setDrvChannel(HTIM_DRV_CH_C, (bridges[2] == MCTRL_BRIDGE_HI));

				mctrl.calibCounter = 0;
				mctrl_state = MCTRL_INDUCTANCE_ID_RUN;
			}
			break;
		}
		case MCTRL_INDUCTANCE_ID_RUN:
		{
			if (mctrl.calibCounter < NUM_STATIC_MEASUREMENTS)
			{
				float vdda = ssf_getVdda();
				mctrl.lastVbus[mctrl.calibCounter] = ssf_getVbus();
				_convertAdcToCurrent(mctrl.lastMeasurement[mctrl.calibCounter], adcCounts, vdda);
				++mctrl.calibCounter;
			}
			else
			{
				spwm_setDrvChannel(HTIM_DRV_CH_A, 0.0);
				spwm_setDrvChannel(HTIM_DRV_CH_B, 0.0);
				spwm_setDrvChannel(HTIM_DRV_CH_C, 0.0);

				mctrl_state = MCTRL_INDUCTANCE_ID_FINISH;
			}
			break;
		}
		case MCTRL_RESISTANCE_ID_START:
		{
			// turn on PWM and wait for it to settle
			const mctrl_bridge_activation_t* bridges = mctrl_params.sysId.idSequence[mctrl.idRunCounter];
			float dc = mctrl_params.sysId.staticIdentificationDutyCycle;

			if (mctrl.calibCounter < NUM_ALIGN_WAIT_OFF_CYCLES)
			{
				spwm_setDrvChannel(HTIM_DRV_CH_A, dc*(bridges[0] == MCTRL_BRIDGE_HI));
				spwm_setDrvChannel(HTIM_DRV_CH_B, dc*(bridges[1] == MCTRL_BRIDGE_HI));
				spwm_setDrvChannel(HTIM_DRV_CH_C, dc*(bridges[2] == MCTRL_BRIDGE_HI));


				++mctrl.calibCounter;
			}
			if (mctrl.calibCounter < 2*NUM_ALIGN_WAIT_OFF_CYCLES)
			{
				spwm_setDrvChannel(HTIM_DRV_CH_A, dc*(bridges[0] == MCTRL_BRIDGE_HI));
				spwm_setDrvChannel(HTIM_DRV_CH_B, dc*(bridges[1] == MCTRL_BRIDGE_HI));
				spwm_setDrvChannel(HTIM_DRV_CH_C, dc*(bridges[2] == MCTRL_BRIDGE_HI));


				++mctrl.calibCounter;
			}
			else
			{
				mctrl.calibCounter = 0;
				mctrl_state = MCTRL_RESISTANCE_ID_RUN;
			}

			break;
		}
		case MCTRL_RESISTANCE_ID_RUN:
		{
			// measure steady-state current
			if (mctrl.calibCounter < NUM_STATIC_MEASUREMENTS)
			{
				float vdda = ssf_getVdda();
				mctrl.lastVbus[mctrl.calibCounter] = ssf_getVbus();
				_convertAdcToCurrent(mctrl.lastMeasurement[mctrl.calibCounter], adcCounts, vdda);
				++mctrl.calibCounter;
			}
			else
			{
				spwm_setDrvChannel(HTIM_DRV_CH_A, 0.0);
				spwm_setDrvChannel(HTIM_DRV_CH_B, 0.0);
				spwm_setDrvChannel(HTIM_DRV_CH_C, 0.0);

				mctrl_state = MCTRL_RESISTANCE_ID_FINISH;
			}
			break;
		}
		case MCTRL_IMPEDANCE_ID_START:
		{
			if (mctrl.calibCounter < NUM_ALIGN_WAIT_OFF_CYCLES)
			{
				spwm_setDrvChannel(HTIM_DRV_CH_A, 0.0);
				spwm_setDrvChannel(HTIM_DRV_CH_B, 0.0);
				spwm_setDrvChannel(HTIM_DRV_CH_C, 0.0);

				++mctrl.calibCounter;
			}
			else
			{
				mctrl.calibCounter = 0;
				// set PWM from phase table, which is +-
				// current path is from idRunCounter+1 down to idRunCounter
				// the idRunCounter phase must be LOW when PWM is positive
				// idRunCounter+1 phase must be HIGH when PWM is negative
				size_t i = mctrl.calibCounter % NUM_PHASE_MEASUREMENTS;
				float dc = mctrl.phasePwm[i];
				const mctrl_bridge_activation_t* bridges = mctrl_params.sysId.idSequence[mctrl.idRunCounter];
				float dca = -(bridges[0] == MCTRL_BRIDGE_LO)*dc + (bridges[0] == MCTRL_BRIDGE_HI)*dc;
				float dcb = -(bridges[1] == MCTRL_BRIDGE_LO)*dc + (bridges[1] == MCTRL_BRIDGE_HI)*dc;
				float dcc = -(bridges[2] == MCTRL_BRIDGE_LO)*dc + (bridges[2] == MCTRL_BRIDGE_HI)*dc;
				spwm_setDrvChannel(HTIM_DRV_CH_A, dca);
				spwm_setDrvChannel(HTIM_DRV_CH_B, dcb);
				spwm_setDrvChannel(HTIM_DRV_CH_C, dcc);

				mctrl_state = MCTRL_IMPEDANCE_ID_RUN;
			}
			break;
		}
		case MCTRL_IMPEDANCE_ID_RUN:
		{
			// measure steady-state current
			if (mctrl.calibCounter < NUM_PHASE_MEASUREMENTS*NUM_INDUCTANCE_ID_CYCLES)
			{
				size_t i = (mctrl.calibCounter+1) % NUM_PHASE_MEASUREMENTS;
				float dc = mctrl.phasePwm[i];
				const mctrl_bridge_activation_t* bridges = mctrl_params.sysId.idSequence[mctrl.idRunCounter];
				float dca = -(bridges[0] == MCTRL_BRIDGE_LO)*dc + (bridges[0] == MCTRL_BRIDGE_HI)*dc;
				float dcb = -(bridges[1] == MCTRL_BRIDGE_LO)*dc + (bridges[1] == MCTRL_BRIDGE_HI)*dc;
				float dcc = -(bridges[2] == MCTRL_BRIDGE_LO)*dc + (bridges[2] == MCTRL_BRIDGE_HI)*dc;
				spwm_setDrvChannel(HTIM_DRV_CH_A, dca);
				spwm_setDrvChannel(HTIM_DRV_CH_B, dcb);
				spwm_setDrvChannel(HTIM_DRV_CH_C, dcc);

				// start measuring after half through the period
				if (mctrl.calibCounter >= NUM_PHASE_MEASUREMENTS*NUM_INDUCTANCE_ID_CYCLES/2)
				{
					float vdda = ssf_getVdda();
					size_t j = (mctrl.calibCounter) % NUM_PHASE_MEASUREMENTS;
					float isense[ISENSE_COUNT] = {};
					_convertAdcToCurrent(isense, adcCounts, vdda);
					for (size_t i = 0; i < ISENSE_COUNT; ++i)
					{
						mctrl.phaseCurrents[j][i] += isense[i];
					}		
				}

				++mctrl.calibCounter;
			}
			else
			{
				// turn everything low again once we're done
				spwm_setDrvChannel(HTIM_DRV_CH_A, 0.0f);
				spwm_setDrvChannel(HTIM_DRV_CH_B, 0.0f);
				spwm_setDrvChannel(HTIM_DRV_CH_C, 0.0f);

				mctrl_state = MCTRL_IMPEDANCE_ID_FINISH;
			}
			break;
		}
		case MCTRL_PPID_START:
		{
			mctrl.calibCounter = now;
			switch(mctrl.sysParamEstimates.motorType)
			{
				case MCTRL_MOT_2PH:
				{
					float dc = mctrl_params.sysId.staticIdentificationDutyCycle;
					size_t step = mctrl.counter % 4;
					float p0[4] = { 0.5f, 0.5f, -0.5f, -0.5f};
					float p1[4] = {-0.5f, 0.5f,  0.5f, -0.5f};
					spwm_setDrvChannel(HTIM_DRV_CH_A, 0.5f + dc*p0[step]);
					spwm_setDrvChannel(HTIM_DRV_CH_B, 0.5f);
					spwm_setDrvChannel(HTIM_DRV_CH_C, 0.5f + dc*p1[step]);

					break;
				}
				case MCTRL_MOT_3PH_SIN:
				case MCTRL_MOT_3PH_TRAP:
				{
					float dc = mctrl_params.sysId.staticIdentificationDutyCycle;
					size_t step = mctrl.counter % 6;
					// float p0[6] = { 0.5f, 0.5f, 0.0f,-0.5f,-0.5f, 0.0f};
					// float p1[6] = {-0.5f, 0.0f, 0.5f, 0.5f, 0.0f,-0.5f};
					// float p2[6] = { 0.0f,-0.5f,-0.5f, 0.0f, 0.5f, 0.5f};
					float p0[6] = { 0.5f, 0.5f,-0.5f,-0.5f,-0.5f, 0.5f};
					float p1[6] = {-0.5f,-0.5f,-0.5f, 0.5f, 0.5f, 0.5f};
					float p2[6] = {-0.5f, 0.5f, 0.5f, 0.5f,-0.5f,-0.5f};
					spwm_setDrvChannel(HTIM_DRV_CH_A, 0.5f + dc*p0[step]);
					spwm_setDrvChannel(HTIM_DRV_CH_B, 0.5f + dc*p1[step]);
					spwm_setDrvChannel(HTIM_DRV_CH_C, 0.5f + dc*p2[step]);

					break;
				}
				default:
				{
					break;
				}
			}

			mctrl_state = MCTRL_PPID_RUN;
			break;
		}
		case MCTRL_PPID_RUN:
		{
			if (now - mctrl.calibCounter > 100000u)
			{
				spwm_setDrvChannel(HTIM_DRV_CH_A, 0.0f);
				spwm_setDrvChannel(HTIM_DRV_CH_B, 0.0f);
				spwm_setDrvChannel(HTIM_DRV_CH_C, 0.0f);

				mctrl_state = MCTRL_PPID_FINISH;
			}
			break;
		}
		case MCTRL_MAPSTEP_START:
		{
			mctrl.calibCounter = now;

			float dc = 2.0f*mctrl_params.sysId.staticIdentificationDutyCycle;

			mctrl_pwm_t pwm = mctrl_setPhasorPwmSin(mctrl.counter*(2.0f*M_PI/NUM_MAPSTEP_MEASUREMENTS), dc, mctrl.sysParamEstimates.motorType);
			mctrl.pwm = pwm;


			mctrl_state = MCTRL_MAPSTEP_RUN;
			break;
		}
		case MCTRL_MAPSTEP_RUN:
		{
			if (now - mctrl.calibCounter > 100000u)
			{
				// spwm_setDrvChannel(HTIM_DRV_CH_A, 0.0f);
				// spwm_setDrvChannel(HTIM_DRV_CH_B, 0.0f);
				// spwm_setDrvChannel(HTIM_DRV_CH_C, 0.0f);

				mctrl_state = MCTRL_MAPSTEP_FINISH;
			}
			break;
		}
		case MCTRL_EMF_STALL_RAMP:
		{
			// increase speed until we stall
			uint32_t dt_us = now - mctrl.calibCounter;
			float estSpeed = mctrl_getSimpleMotorSpeedEstimate();
			float dc = mctrl_params.sysId.staticIdentificationDutyCycle;
			float speed = 5.0*2.0f*M_PI*1.0e-6*(dt_us);
			mctrl.phase = fmodf(mctrl.phase + speed*MEAS_FULL_PERIOD, 2.0f*M_PI);

			mctrl_setPhasorPwmSin(mctrl.phase, dc, mctrl.sysParamEstimates.motorType);

			// limit time to 60s
			if ((dt_us > 60u*1000000u) 
				|| (  (fabsf(estSpeed) < 0.5f*mctrl.stallSpeed) 
					&& fabsf(mctrl.stallSpeed) > 0.1*2.0f*M_PI))
			{
				mctrl.calibCounter = now;
				mctrl_state = MCTRL_EMF_STALL_EVAL;
			}
			else
			{
				mctrl.stallSpeed = fmaxf(fabsf(estSpeed), mctrl.stallSpeed);
			}
			break;
		}
		case MCTRL_EMF_START:
		{
			mctrl.calibCounter = now;
			mctrl.phase = 0.0f;
			float dc = mctrl_params.sysId.staticIdentificationDutyCycle;

			mctrl_setPhasorPwmSin(mctrl.phase, dc, mctrl.sysParamEstimates.motorType);

			mctrl_state = MCTRL_EMF_RAMP;
			break;
		}
		case MCTRL_EMF_RAMP:
		{
			float dc = mctrl_params.sysId.staticIdentificationDutyCycle;
			float maxSpeed = 0.9f*mctrl.stallSpeed*(1.0f+mctrl.idRunCounter)/NUM_STATIC_MEASUREMENTS*(mctrl.sysParamEstimates.stepsPerRev/4);
			float speed = maxSpeed*fminf(1.0f, 1.0e-6*(now - mctrl.calibCounter));
			// alpha = EMFID_INITAL_SPEED;
			mctrl.phase = fmodf(mctrl.phase + speed*MEAS_FULL_PERIOD, 2.0f*M_PI);

			mctrl_setPhasorPwmSin(mctrl.phase, dc, mctrl.sysParamEstimates.motorType);

			if (now - mctrl.calibCounter > 1000000u)
			{
				mctrl.calibCounter = now;
				mctrl_state = MCTRL_EMF_RUN;
			}
			break;
		}
		case MCTRL_EMF_RUN:
		{
			float maxSpeed = 0.9f*mctrl.stallSpeed*(1.0f+mctrl.idRunCounter)/NUM_STATIC_MEASUREMENTS*(mctrl.sysParamEstimates.stepsPerRev/4);
			float dc = mctrl_params.sysId.staticIdentificationDutyCycle;
			float speed = maxSpeed*1.0f;
			mctrl.phase = fmodf(mctrl.phase + speed*MEAS_FULL_PERIOD, 2.0f*M_PI);

			mctrl_setPhasorPwmSin(mctrl.phase, dc, mctrl.sysParamEstimates.motorType);

			float vdda = ssf_getVdda();
			float currents[ISENSE_COUNT] = {0.0f};
			_convertAdcToCurrent(currents, adcCounts, vdda);

			for (size_t i = 0; i < ISENSE_COUNT; ++i)
			{
				mctrl.currentSqrSum[i] += currents[i]*currents[i];
			}
			++mctrl.counter;

			if (now - mctrl.calibCounter > (unsigned)(1.0e6f*EMFID_RUN_TIME))
			{
				mctrl.calibCounter = now;
				mctrl_state = MCTRL_EMF_DECELERATE;
			}
			break;
		}
		case MCTRL_EMF_DECELERATE:
		{
			float maxSpeed = 0.9f*mctrl.stallSpeed*(1.0f+mctrl.idRunCounter)/NUM_STATIC_MEASUREMENTS*(mctrl.sysParamEstimates.stepsPerRev/4);
			float dc = mctrl_params.sysId.staticIdentificationDutyCycle;
			float speed = maxSpeed*fmaxf(0.0f, 1.0e-6*(1000000u - (now - mctrl.calibCounter)));
			// alpha = -EMFID_INITAL_SPEED;
			mctrl.phase = fmodf(mctrl.phase + speed*MEAS_FULL_PERIOD, 2.0f*M_PI);

			mctrl_setPhasorPwmSin(mctrl.phase, dc, mctrl.sysParamEstimates.motorType);

			if (now - mctrl.calibCounter > 1000000u)
			{
				spwm_setDrvChannel(HTIM_DRV_CH_A, 0.0f);
				spwm_setDrvChannel(HTIM_DRV_CH_B, 0.0f);
				spwm_setDrvChannel(HTIM_DRV_CH_C, 0.0f);
				mctrl.calibCounter = now;
				mctrl_state = MCTRL_EMF_FINISH;
			}
			break;
		}
		case MCTRL_DEMO:
		{
			float vdda = ssf_getVdda();
			float currents[ISENSE_COUNT] = {0.0f};
			_convertAdcToCurrent(currents, adcCounts, vdda);

			// store phase currents for debugging purposes
			for (size_t i = 0; i < ISENSE_COUNT; ++i)
			{
				size_t pidx = ((int)(mctrl.phase*((PHASE_BUCKETS)/(M_PI*2.0f)))) % PHASE_BUCKETS;
				mctrl.demoPhasedCurrents[i][pidx] = currents[i];
			}

			// "control" right now is just constant speed moves

			const float step = M_PI/(100.0e3/6.0/10)*0.05f;

			// keep the brake resistor off
			spwm_setDrvChannel(HTIM_DRV_CH_R, 0.0f);

			float dc = 0.1f;
			mctrl_setPhasorPwmSin(mctrl.phase, dc, mctrl.sysParamEstimates.motorType);
			// float pwm[3] = {dc, dc, dc};

			// switch(mctrl.sysParamEstimates.motorType)
			// {
			// 	case MCTRL_MOT_2PH:
			// 	{
			// 		_compute2PhPwm(mctrl.phase, pwm);

			// 		break;
			// 	}
			// 	case MCTRL_MOT_3PH:
			// 	{
			// 		_compute3PhPwmSin(mctrl.phase, pwm);
					
			// 		break;
			// 	}
			// 	default:
			// 	{
			// 		break;
			// 	}
			// }


			// spwm_setDrvChannel(HTIM_DRV_CH_A, pwm[0]);
			// spwm_setDrvChannel(HTIM_DRV_CH_B, pwm[1]);
			// spwm_setDrvChannel(HTIM_DRV_CH_C, pwm[2]);

			++mctrl.counter;
			mctrl.phase = fmodf(mctrl.phase + step, 2.0f*M_PI);

			break;
		}
		default:
		{
			break;
		}
	}

	mctrl_updateSimpleSensorEstimate(utime_now());
}

