
#include "ssf_main.h"
#include "ssf_spi.h"
#include "debug.h"
#include "ssf_perf.h"

#include <math.h>
#include <float.h>
#include <stdbool.h>
#include <string.h>



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

Assuming an RH/RL being equal at Rt, Rp being being equal cross phases, and Ra,Rb,Rc being the measured values on each phase:
	Rp = (Ra - 4Rt + sqrt(Ra^2 + 4 Rt^2)) / 2 
	Rp = 2 Rb - 3 Rt

*/

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

static float _phasedCurrents[6][PHASE_BUCKETS];
static float _phase = 0.0f;
static size_t _counter = 0;

static float _adcZeroCalibs[6];
static volatile size_t _calibCounter = 0;

static volatile float _lastMeasurement[NUM_STATIC_MEASUREMENTS][ISENSE_COUNT];




#define MEAS_SINGLE_PERIOD 	10.0e-6f
#define MEAS_SAMPLES 		6
#define MEAS_FULL_PERIOD	(MEAS_SINGLE_PERIOD*MEAS_SAMPLES)
#define NUM_ID_ALGO_SAMPLES	3
#define NUM_ID_ALGO_ESTIMATES (2*(NUM_STATIC_MEASUREMENTS - (NUM_ID_ALGO_SAMPLES-1)))
#define NUM_IDENTIFICATION_RUNS	3

#define MAX_IDENTIFICATION_REPEATS	32
#define IDELTA_MIN			0.001f
#define TARGET_VAR_NORM		0.05f

#define RESISTANCE_ID_DC	1.0f

typedef enum {
	MCTRL_INIT,

	MCTRL_ANALOG_CALIBRATION_RUN,
	MCTRL_ANALOG_CALIBRATION_FINISH,

	MCTRL_ID_ALIGN_START,
	MCTRL_ID_ALIGN_WAIT,
	MCTRL_ID_ALIGN_SETTLE,
	MCTRL_ID_ALIGN_FINISH,

	MCTRL_RAMPTIME_ID_START,
	MCTRL_RAMPTIME_ID_WAIT,
	MCTRL_RAMPTIME_ID_FINISH,

	MCTRL_INDUCTANCE_ID_START,
	MCTRL_INDUCTANCE_ID_RUN,
	MCTRL_INDUCTANCE_ID_FINISH,

	MCTRL_RESISTANCE_ID_START,
	MCTRL_RESISTANCE_ID_RUN,
	MCTRL_RESISTANCE_ID_FINISH,

	MCTRL_DEMO,
	MCTRL_
} mctrl_state_t;

typedef struct {
	struct {
		float maxCurrent;
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
	// debugging counters
	uint16_t lastEventCount;
	uint16_t lastEventCountDelta;
	uint32_t lastControlStepTime_us;
	uint32_t lastControlDelta_us;
} mctrl_controller_t;

static mctrl_params_t _params = {
	.sysId = {
		.maxCurrent = 1.0f,
		.maxRampCycles = NUM_STATIC_MEASUREMENTS,
	},
};

static mctrl_controller_t _mctrl;

static size_t _idRunCounter = 0;

static volatile mctrl_state_t _state = MCTRL_INIT;

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

typedef struct {
    float sumx;		// sum of x     
    float sumx2;	// sum of x**2  
    float sumxy;	// sum of x * y 
    float sumy;		// sum of y     
    float sumy2;	// sum of y**2 
    size_t n;		// number of samples

} incrementalLinreg_t;

typedef struct {
	float intercept, slope, rsqr;
	bool ok;
} linregResult_t;

incrementalLinreg_t linreg_addSample(incrementalLinreg_t self, float x, float y)
{
	self.sumx  += x;       
	self.sumx2 += x*x;  
	self.sumxy += x*y;
	self.sumy  += y;      
	self.sumy2 += y*y; 
	++self.n;
	return self;
}

linregResult_t linreg_solve(incrementalLinreg_t self)
{
	float denom = (self.n * self.sumx2 - (self.sumx*self.sumx));
	if (fabsf(denom) <= FLT_EPSILON) 
	{
	    return (linregResult_t){.ok = false};
	}
	float nom = (self.n*self.sumxy - self.sumx*self.sumy);
	float den = (self.n*self.sumx2 - self.sumx*self.sumx) * (self.n*self.sumy2 - self.sumy*self.sumy);
	linregResult_t result = {
		.intercept = (self.n * self.sumxy  -  self.sumx*self.sumy) / denom,
		.slope = (self.sumy * self.sumx2  -  self.sumx*self.sumxy) / denom,
	    .rsqr =  nom*nom / den,
	    .ok = true,
	};
	return result;
}

// https://stackoverflow.com/questions/5083465/fast-efficient-least-squares-fit-algorithm-in-c
static int linreg(size_t n, const float x[], const float y[], float* m, float* b, float* r)
{
    float sumx = 0.0;	// sum of x     
    float sumx2 = 0.0;	// sum of x**2  
    float sumxy = 0.0;	// sum of x * y 
    float sumy = 0.0;	// sum of y     
    float sumy2 = 0.0;	// sum of y**2 

    for (size_t i = 0; i < n; ++i)
    { 
        sumx  += x[i];       
        sumx2 += (x[i]*x[i]);  
        sumxy += x[i] * y[i];
        sumy  += y[i];      
        sumy2 += (y[i]*y[i]); 
    } 

    float denom = (n * sumx2 - (sumx*sumx));
    if (fabsf(denom) <= FLT_EPSILON) 
    {
        *m = 0.0f;
        *b = 0.0f;
        if (r)
         	*r = 0.0f;
        return -1;
    }

    *m = (n * sumxy  -  sumx*sumy) / denom;
    *b = (sumy * sumx2  -  sumx*sumxy) / denom;
    if (r) 
    {
    	float nom = (n*sumxy - sumx*sumy);
    	float den = ((n*sumx2 - sumx*sumx) *
              (n*sumy2 - sumy*sumy));
        *r =  nom*nom / den;
    }

    return 0; 
}



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
	float i = 0.5*(i1+i2);
	float ii1 = 0.5*(i2-i0);
	float ii2 = 0.5*(i3-i1);
	float iii = ii2-ii1;
	// float ii = i2-i1;

	float R = V*(iii)/(i2*ii1-i1*ii2);

	return R;
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
	// do not enter calibration mode again, sample ADCs on microcontroller to determine zero offset at output of current sense amps
	// ssf_enterMotorDriverCalibrationMode();

	HAL_Delay(2);

	// then read out ADCs for zero calib
	// average 16 reads (2 per fastLoop)
	_state = MCTRL_ANALOG_CALIBRATION_RUN;

	// wait for calibration measurements to finish from fastLoop
	while (_state != MCTRL_ANALOG_CALIBRATION_FINISH) {};


	// average out the reads for an adc count calibration
	for (size_t i = 0; i < 3; ++i)
	{
		float calib = (_adcZeroCalibs[2*i]+_adcZeroCalibs[2*i+1])*(1.0f/CALIBREADS);
		
		// dbg_println("    calib0 = %.3f, calib1 = %.3f", (double)(_adcZeroCalibs[2*i]*(2.0f/CALIBREADS)), (double)(_adcZeroCalibs[2*i+1]*(2.0f/CALIBREADS)));
		
		_adcZeroCalibs[2*i] = calib;
		_adcZeroCalibs[2*i+1] = calib;

		dbg_println("SO[%u] calibrated to 0A = %.3f counts", i, (double)calib);
	}

	// ssf_exitMotorDriverCalibrationMode();

	spwm_enableHalfBridges(0x0);

	// driver calibration is now done, we can move on to motor identification
	// do identification runs for each phase
	for (_idRunCounter = 0; _idRunCounter < NUM_IDENTIFICATION_RUNS; ++_idRunCounter)
	{
		// align rotor with a gentle nudge
		_state = MCTRL_ID_ALIGN_START;
		while (_state != MCTRL_ID_ALIGN_FINISH) {};

		int lowBridge = _idRunCounter;
		int highBridge = (_idRunCounter + 1) % 3;

		spwm_enableHalfBridges((1 << lowBridge) | (1 << highBridge));

		// ballpark measure how fast current ramps up
		memset((void*)_lastMeasurement, 0, sizeof(_lastMeasurement));
		_calibCounter = 0;
		_state = MCTRL_RAMPTIME_ID_START;
		while (_state != MCTRL_RAMPTIME_ID_FINISH) {};

		size_t numSampleExceedingCurrentLimit = _calibCounter;

		if (numSampleExceedingCurrentLimit >= NUM_STATIC_MEASUREMENTS)
		{
			{
				float i0 = _lastMeasurement[NUM_STATIC_MEASUREMENTS-1][2*_idRunCounter+0];
				float i1 = _lastMeasurement[NUM_STATIC_MEASUREMENTS-1][2*_idRunCounter+1];

				float i = 0.5*(i0+i1);

				float time = (NUM_STATIC_MEASUREMENTS-1)*(float)MEAS_FULL_PERIOD + _idRunCounter*2*MEAS_SINGLE_PERIOD;
				dbg_println("could not reach %.3f, max current reached is %.3f after %8.0f us", (double)_params.sysId.maxCurrent, (double)i, (double)(time*1e6f));

			}

			{
				// run full-PWM resistance estimation
				memset((void*)_lastMeasurement, 0, sizeof(_lastMeasurement));
				_calibCounter = 0;
				_state = MCTRL_RESISTANCE_ID_START;
				while (_state != MCTRL_RESISTANCE_ID_FINISH) {};

				float i = 0.0f;

				for (size_t j = NUM_STATIC_MEASUREMENTS/2; j < NUM_STATIC_MEASUREMENTS; ++j)
				{
					float i0 = _lastMeasurement[j][2*_idRunCounter+0];
					float i1 = _lastMeasurement[j][2*_idRunCounter+1];

					i += (i0+i1);

				}
				i *= 1.0f/NUM_STATIC_MEASUREMENTS;

				float u = ssf_getVbus();
				float R = u/i;

				dbg_println("steady state current is %1.3f @ %8.3f for %.3f R", (double)i, (double)u, (double)(R));

			}

		}
		else
		{
			float i0 = _lastMeasurement[numSampleExceedingCurrentLimit][2*_idRunCounter+0];
			float i1 = _lastMeasurement[numSampleExceedingCurrentLimit][2*_idRunCounter+1];

			float i = 0.5*(i0+i1);
			float time = numSampleExceedingCurrentLimit*(float)MEAS_FULL_PERIOD + _idRunCounter*2*MEAS_SINGLE_PERIOD;

			dbg_println("reached %.3f after %8.0f us", (double)i, (double)(time*1e6f));

		}


		float inductanceEstimate = 0.0f;
		float inductanceVariance = 0.0f;
		float resistanceEstimate = 0.0f;
		float resistanceVariance = 0.0f;
		float tcEstimate = 0.0f;
		float tcVariance = 0.0f;
		for (size_t k = 0; k < MAX_IDENTIFICATION_REPEATS; ++k)
		{
			memset((void*)_lastMeasurement, 0, sizeof(_lastMeasurement));
			_calibCounter = 0;
			_state = MCTRL_INDUCTANCE_ID_START;
			while (_state != MCTRL_INDUCTANCE_ID_FINISH) {};


			float Lest[NUM_ID_ALGO_ESTIMATES] = {};
			float Rest[NUM_ID_ALGO_ESTIMATES] = {};

			size_t numValidLMeasurements = 0;
			size_t numValidRMeasurements = 0;

			float TauEst[NUM_ID_ALGO_ESTIMATES] = {};
			size_t numValidTaus = 0;

			// for (size_t j = 0; j < NUM_STATIC_MEASUREMENTS; ++j)
			// {
			// 	for (size_t i = 0; i < 3; ++i)
			// 	{

			// 		dbg_println("SO[%u] reads %8.3f,  %8.3f", i, (double)_lastMeasurement[j][2*i], (double) _lastMeasurement[j][2*i+1]);
			// 	}
			// }

			incrementalLinreg_t linregData = {};

			for (size_t j = 0; j+1 < NUM_STATIC_MEASUREMENTS; ++j)
			{
				const float h = MEAS_FULL_PERIOD;
				const float dh = MEAS_SINGLE_PERIOD;
				float i0 = _lastMeasurement[j+0][2*_idRunCounter];
				float i0a = _lastMeasurement[j+0][2*_idRunCounter+1];
				float i1 = _lastMeasurement[j+1][2*_idRunCounter];
				float i1a = _lastMeasurement[j+1][2*_idRunCounter+1];
				float di = (i1-i0)/h;
				float dia = (i1a-i0a)/h;

				linregData = linreg_addSample(linregData, j*h + 0.0f*dh, logf(di));
				linregData = linreg_addSample(linregData, j*h + 1.0f*dh, logf(dia));

				// dbg_println("  di[%u] = %8.3f A/s, dia = %8.3f A/s", _idRunCounter, (double)(di), (double)(dia));
			}

			linregResult_t lr = linreg_solve(linregData);

			float currentRate = expf(-lr.slope);
			// float vbus = ssf_getVbus();

			dbg_println("LINREG tau[%u] = %8.3f ms, R^2 = %8.3f, h = %u us (%u count)", _idRunCounter, (double)(currentRate*1e3f), (double)(lr.rsqr), _mctrl.lastControlDelta_us, _mctrl.lastEventCountDelta);

			for (size_t j = 0; j+NUM_ID_ALGO_SAMPLES-1 < NUM_STATIC_MEASUREMENTS; ++j)
			{
#if NUM_ID_ALGO_SAMPLES == 3
				// measurements are done in batches of 6, A,A,B,B,C,C
				// in the system identification run, we'd look at one channel each
				// over a number of repeated measurements NUM_STATIC_MEASUREMENTS

				float i0 = _lastMeasurement[j+0][2*_idRunCounter];
				float i1 = _lastMeasurement[j+1][2*_idRunCounter];
				float i2 = _lastMeasurement[j+2][2*_idRunCounter];

				float i0a = _lastMeasurement[j+0][2*_idRunCounter+1];
				float i1a = _lastMeasurement[j+1][2*_idRunCounter+1];
				float i2a = _lastMeasurement[j+2][2*_idRunCounter+1];

				bool validL0 = false, validL1 = false;
				bool validR0 = false, validR1 = false;
				float vbus = ssf_getVbus();
				float L0 = _calculateInductance_n3(i0, i1, i2, vbus, MEAS_FULL_PERIOD, &validL0);
				float L1 = _calculateInductance_n3(i0a, i1a, i2a, vbus, MEAS_FULL_PERIOD, &validL1);
				float R0 = _calculateResistance_n3(i0, i1, i2, vbus, MEAS_FULL_PERIOD, &validR0);
				float R1 = _calculateResistance_n3(i0a, i1a, i2a, vbus, MEAS_FULL_PERIOD, &validR1);

				bool tauValid0, tauValid1;
				float tau0 = _calculateTimeConstant_n3(i0, i1, i2, vbus, MEAS_FULL_PERIOD, &tauValid0);
				float tau1 = _calculateTimeConstant_n3(i0a, i1a, i2a, vbus, MEAS_FULL_PERIOD, &tauValid1);

				// dbg_println("  [%u] L0 = %8.3f mH, L1 = %8.3f mH", _idRunCounter, (double)(L0*1e3), (double)(L1*1e3));
				// dbg_println(" [%u] R0 = %8.3f R,  R1 = %8.3f R", _idRunCounter, (double)(R0*1e0), (double)(R1*1e0));
				// dbg_println(" [%u] t0 = %8.3f ms, t1 = %8.3f ms", _idRunCounter, (double)(tau0*1e3), (double)(tau1*1e3));

				if (validL0)
				{
					Lest[numValidLMeasurements++] = L0;
				}
				if (validL1)
				{
					Lest[numValidLMeasurements++] = L1;
				}
				if (validR0)
				{
					Rest[numValidRMeasurements++] = R0;
				}
				if (validR1)
				{
					Rest[numValidRMeasurements++] = R1;
				}

				if (tauValid0)
				{
					TauEst[numValidTaus++] = tau0;
				}
				if (tauValid1)
				{
					TauEst[numValidTaus++] = tau1;
				}

#elif NUM_ID_ALGO_SAMPLES == 4
				float i0 = _lastMeasurement[j+0][2*_idRunCounter];
				float i1 = _lastMeasurement[j+1][2*_idRunCounter];
				float i2 = _lastMeasurement[j+2][2*_idRunCounter];
				float i3 = _lastMeasurement[j+3][2*_idRunCounter];

				float i0a = _lastMeasurement[j+0][2*_idRunCounter+1];
				float i1a = _lastMeasurement[j+1][2*_idRunCounter+1];
				float i2a = _lastMeasurement[j+2][2*_idRunCounter+1];
				float i3a = _lastMeasurement[j+3][2*_idRunCounter+1];

				bool valid0 = false, valid1 = false;
				float vbus = ssf_getVbus();
				float L = _calculateInductance(i0, i1, i2, i3, vbus, MEAS_FULL_PERIOD, &valid0);
				float La = _calculateInductance(i0a, i1a, i2a, i3a, vbus, MEAS_FULL_PERIOD, &valid1);
				float R = _calculateResistance(i0, i1, i2, i3, vbus, MEAS_FULL_PERIOD);
				float Ra = _calculateResistance(i0a, i1a, i2a, i3a, vbus, MEAS_FULL_PERIOD);

				bool tauValid0, tauValid1;
				float tau = _calculateTimeConstant(i0, i1, i2, i3, vbus, MEAS_FULL_PERIOD, &tauValid0);
				float taua = _calculateTimeConstant(i0a, i1a, i2a, i3a, vbus, MEAS_FULL_PERIOD, &tauValid1);

				if (valid0)
				{
					Lest[numValidMeasurements] = L;
					Rest[numValidMeasurements] = R;
					++numValidMeasurements;					
				}
				if (valid1)
				{
					Lest[numValidMeasurements] = La;
					Rest[numValidMeasurements] = Ra;
					++numValidMeasurements;					
				}

				if (tauValid0)
				{
					TauEst[numValidTaus] = tau;
					++numValidTaus;					
				}
				if (tauValid1)
				{
					TauEst[numValidTaus] = taua;
					++numValidTaus;					
				}
#endif

				// dbg_println("L reads %8.3f, %8.3f mH", (double)(L*1e3f), (double)(La*1e3f));
				// dbg_println("R reads %8.3f, %8.3f R", (double)(R), (double)(Ra));

			}



			float Lavg = 0.0;
			float Lvar = 0.0;
			float Ravg = 0.0;
			float Rvar = 0.0;

			for (size_t j = 0; j < numValidLMeasurements; ++j)
			{
				Lavg += Lest[j];
				Lvar += Lest[j]*Lest[j];
			}

			for (size_t j = 0; j < numValidRMeasurements; ++j)
			{
				Ravg += Rest[j];
				Rvar += Rest[j]*Rest[j];
			}


			Lavg *= 1.0f/numValidLMeasurements;
			Lvar *= 1.0f/numValidLMeasurements;

			Lvar -= Lavg*Lavg;

			Ravg *= 1.0f/numValidRMeasurements;
			Rvar *= 1.0f/numValidLMeasurements;

			Rvar -= Ravg*Ravg;

			if (k == 0)
			{
				inductanceEstimate = Lavg;
				inductanceVariance = Lvar;	
				resistanceEstimate = Ravg;
				resistanceVariance = Rvar;	
			}
			else
			{
				// dbg_println("  Lest  %8.3f mH, sigma = %8.3f mH", (double)(Lavg*1e3), (double)(sqrtf(Lvar)*1e3));
				// dbg_println("  Rest  %8.3f R,  sigma = %8.3f R", (double)(Ravg*1e0), (double)(sqrtf(Rvar)*1e0));


				// update estimate via Kalman Filter
				/*
				K = sigma1^2 / (sigma1^2 + sigma2^2)
				x = x1 + K (x2 - x1)
				sigma^2 = (1 - K) sigma1^2
				*/
				if (numValidLMeasurements > 1)
				{
					float KL = inductanceVariance / (inductanceVariance +  Lvar);
					inductanceEstimate += KL * (Lavg - inductanceEstimate);
					inductanceVariance *= 1.0f - KL;	
				}
				if (numValidRMeasurements > 1)
				{
					float KR = resistanceVariance / (resistanceVariance +  Rvar);
					resistanceEstimate += KR * (Ravg - resistanceEstimate);
					resistanceVariance *= 1.0f - KR;					
				}

				// dbg_println("  KL = %8.3f, KR = %8.3f", (double)(KL), (double)(KR));

			}

			float tavg = 0.0f, tvar = 0.0f;
			for (size_t j = 0; j < numValidTaus; ++j)
			{
				tavg += TauEst[j];
				tvar += TauEst[j]*TauEst[j];
			}

			tavg *= 1.0f/numValidTaus;
			tvar *= 1.0f/numValidTaus;

			tvar -= tavg*tavg;

			if (k == 0)
			{
				tcEstimate = tavg;
				tcVariance = tvar;	
			}
			else if (numValidTaus > 1)
			{
				float KT = tcVariance / (tcVariance +  tvar);
				tcEstimate += KT * (tavg - tcEstimate);
				tcVariance *= 1.0f - KT;

				// dbg_println("  KL = %8.3f, KR = %8.3f", (double)(KL), (double)(KR));

			}

			// dbg_println("Kalman Lest[%u] is  %8.3f mH, sigma = %8.3f mH", _idRunCounter, (double)(inductanceEstimate*1e3), (double)(sqrtf(inductanceVariance)*1e3));
			// dbg_println("Kalman Rest[%u] is  %8.3f R,  sigma = %8.3f R", _idRunCounter, (double)(resistanceEstimate*1e0), (double)(sqrtf(resistanceVariance)*1e0));

			if ((inductanceVariance < inductanceEstimate*inductanceEstimate*TARGET_VAR_NORM*TARGET_VAR_NORM) && (resistanceVariance < resistanceEstimate*resistanceEstimate*TARGET_VAR_NORM*TARGET_VAR_NORM))
			{
				dbg_println("Reached required limit.");
				break;
			}


		}

		dbg_println("Kalman Lest[%u] is  %8.3f mH, sigma = %8.3f mH", _idRunCounter, (double)(inductanceEstimate*1e3), (double)(sqrtf(inductanceVariance)*1e3));
		dbg_println("Kalman Rest[%u] is  %8.3f R,  sigma = %8.3f R", _idRunCounter, (double)(resistanceEstimate*1e0), (double)(sqrtf(resistanceVariance)*1e0));
		dbg_println("Kalman Tau[%u] is   %8.3f ms, sigma = %8.3f ms", _idRunCounter, (double)(tcEstimate*1e3), (double)(sqrtf(tcVariance)*1e3));

		_mctrl.sysParamEstimates.phases.Lest[_idRunCounter] = inductanceEstimate;
		_mctrl.sysParamEstimates.phases.Lvar[_idRunCounter] = inductanceVariance;
		_mctrl.sysParamEstimates.phases.Rest[_idRunCounter] = resistanceEstimate;
		_mctrl.sysParamEstimates.phases.Rvar[_idRunCounter] = resistanceVariance;

	}

	// we measured the phase values, lets see what the winding resistance would be in ABBC config

	// {
	// 	float Ra = _mctrl.sysParamEstimates.phases.Rest[0];
	// 	float Rb = _mctrl.sysParamEstimates.phases.Rest[1];
	// 	float Rsw = -(8*Ra*Rb - 16*Rb*Rb)/(4*Ra + 16*Rb);
	// 	float Rp = 2*Rb - 3*Rsw;
	// 	dbg_println("Rsw[AB] is %8.3f R", (double)Rsw);
	// 	dbg_println("Rp[AB]  is %8.3f R", (double)Rp);
	// }
	// {
	// 	float Ra = _mctrl.sysParamEstimates.phases.Rest[2];
	// 	float Rb = _mctrl.sysParamEstimates.phases.Rest[1];
	// 	float Rsw = -(8*Ra*Rb - 16*Rb*Rb)/(4*Ra + 16*Rb);
	// 	float Rp = 2*Rb - 3*Rsw;
	// 	dbg_println("Rsw[BC] is %8.3f R", (double)Rsw);
	// 	dbg_println("Rp[BC]  is %8.3f R", (double)Rp);
	// }

	{
		float Ra = _mctrl.sysParamEstimates.phases.Rest[0];
		float Rb = _mctrl.sysParamEstimates.phases.Rest[1];
		float Rs = 0.015f;
		float Rt = -2*(Rs*Rs + Ra*Rs - 3*Rb*Rs + 2*Rb*Rb - Ra*Rb)/(3*Rs + Ra - 4*Rb);
		float Rp = 2*Rb - 3*Rt - 2*Rs;
		dbg_println("Rt[AB] is %8.3f R", (double)Rt);
		dbg_println("Rp[AB] is %8.3f R", (double)Rp);
	}
	{
		float Ra = _mctrl.sysParamEstimates.phases.Rest[2];
		float Rb = _mctrl.sysParamEstimates.phases.Rest[1];
		float Rs = 0.015f;
		float Rt = -2*(Rs*Rs + Ra*Rs - 3*Rb*Rs + 2*Rb*Rb - Ra*Rb)/(3*Rs + Ra - 4*Rb);
		float Rp = 2*Rb - 3*Rt - 2*Rs;
		dbg_println("Rt[BC] is %8.3f R", (double)Rt);
		dbg_println("Rp[BC] is %8.3f R", (double)Rp);
	}

	{
		// with only 2 phases (3rd disabled) active we have
		// Rc = 2*Rp + 2*Rt + Rs 
		// Ra/b = Rp + 2*Rt + Rs
		// Rp = Ra/b - 2*Rt - Rs
		// Rp = 0.5*Rc - Rt - 0.5*Rs
		// 0.5*Rc - Rt - 0.5*Rs = Ra/b - 2*Rt - Rs
		// Rt = Ra/b - 0.5*Rc - 0.5*Rs
		float Ra = _mctrl.sysParamEstimates.phases.Rest[0];
		float Rc = _mctrl.sysParamEstimates.phases.Rest[2];
		float Rs = 0.015f;
		float Rt = Ra - 0.5f*Rc - 0.5*Rs;
		float Rp = 0.5*Rc - Rt - 0.5*Rs;
		dbg_println("Rt[A/C] is %8.3f R", (double)Rt);
		dbg_println("Rp[A/C] is %8.3f R", (double)Rp);
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
	// also, the DRV8323 is inverting
	// for 12V operation, 10A would give 150mV, factor of 10 gives full range for 8A
	// return Rs * G
	return -1.0f/(0.015f*20.0f);
}


static void _convertAdcToCurrent(volatile float currents[6], const uint16_t adcCounts[6], const float vdda)
{
	float gain = (1.0f/(ADC2_OVERSAMPLING_FACTOR*ADC2_NOMINAL_MAXCOUNT))*vdda*_getCurrentSenseFactor();
	for (size_t i = 0; i < 6; ++i)
	{
		float adcCount = (float)(adcCounts[i]);

		float current = (adcCount - _adcZeroCalibs[i])*gain;
		currents[i] = current;
	}
}

void mctrl_fastLoop(const uint16_t adcCounts[6])
{
	uint32_t now = perf_now();
	uint16_t eventCount = perf_getEventCount();

	_mctrl.lastControlDelta_us = now - _mctrl.lastControlStepTime_us;
	_mctrl.lastControlStepTime_us = now;

	_mctrl.lastEventCountDelta = eventCount - _mctrl.lastEventCount;
	_mctrl.lastEventCount = eventCount;

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
		case MCTRL_ID_ALIGN_START:
		{
			// turn everything off to to start
			if (_calibCounter < NUM_ALIGN_WAIT_OFF_CYCLES)
			{
				spwm_setDrvChannel(HTIM_DRV_CH_A, 0.0);
				spwm_setDrvChannel(HTIM_DRV_CH_B, 0.0);
				spwm_setDrvChannel(HTIM_DRV_CH_C, 0.0);

				++_calibCounter;
			}
			else
			{
				_calibCounter = 0;
				_state = MCTRL_ID_ALIGN_WAIT;
			}

			break;
		}
		case MCTRL_ID_ALIGN_WAIT:
		{
			// ramp up current to align motor
			// wait for some time to complete alignment
			if (_calibCounter < NUM_ALIGN_WAIT_ON_CYCLES)
			{
				// ramp up the current to not cause a huge jerk bringing the rotor into alignment
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
				_state = MCTRL_ID_ALIGN_SETTLE;
			}
			break;
		}
		case MCTRL_ID_ALIGN_SETTLE:
		{
			// wait for some time to complete alignment and have current die down
			if (_calibCounter < NUM_ALIGN_WAIT_OFF_CYCLES)
			{
				spwm_setDrvChannel(HTIM_DRV_CH_A, 0.0);
				spwm_setDrvChannel(HTIM_DRV_CH_B, 0.0);
				spwm_setDrvChannel(HTIM_DRV_CH_C, 0.0);
				++_calibCounter;
			}
			else
			{
				_calibCounter = 0;
				_state = MCTRL_ID_ALIGN_FINISH;
			}
			break;
		}

		case MCTRL_RAMPTIME_ID_START:
		{
			spwm_setDrvChannel(HTIM_DRV_CH_A, (_idRunCounter != 0));
			spwm_setDrvChannel(HTIM_DRV_CH_B, (_idRunCounter != 1));
			spwm_setDrvChannel(HTIM_DRV_CH_C, (_idRunCounter != 2));
			_calibCounter = 0;
			_state = MCTRL_RAMPTIME_ID_WAIT;

			break;
		}
		case MCTRL_RAMPTIME_ID_WAIT:
		{
			// turn current off again
			// wait for some time to complete alignment
			if (_calibCounter < _params.sysId.maxRampCycles)
			{
				spwm_setDrvChannel(HTIM_DRV_CH_A, (_idRunCounter != 0));
				spwm_setDrvChannel(HTIM_DRV_CH_B, (_idRunCounter != 1));
				spwm_setDrvChannel(HTIM_DRV_CH_C, (_idRunCounter != 2));

				float vdda = ssf_getVdda();
				_convertAdcToCurrent(_lastMeasurement[_calibCounter], adcCounts, vdda);

				float i0 = _lastMeasurement[_calibCounter][2*_idRunCounter+0];
				float i1 = _lastMeasurement[_calibCounter][2*_idRunCounter+1];

				float i = 0.5*(i0+i1);

				// stop if we exceed max current
				if (fabsf(i) > _params.sysId.maxCurrent)
				{
					spwm_setDrvChannel(HTIM_DRV_CH_A, 0.0);
					spwm_setDrvChannel(HTIM_DRV_CH_B, 0.0);
					spwm_setDrvChannel(HTIM_DRV_CH_C, 0.0);

					_state = MCTRL_RAMPTIME_ID_FINISH;
				}
				else
					++_calibCounter;
			}
			else
			{
				spwm_setDrvChannel(HTIM_DRV_CH_A, 0.0);
				spwm_setDrvChannel(HTIM_DRV_CH_B, 0.0);
				spwm_setDrvChannel(HTIM_DRV_CH_C, 0.0);

				_state = MCTRL_RAMPTIME_ID_FINISH;
			}
			break;
		}
		case MCTRL_INDUCTANCE_ID_START:
		{
			// wait for things to settle
			if (_calibCounter < NUM_ALIGN_WAIT_OFF_CYCLES)
			{
				spwm_setDrvChannel(HTIM_DRV_CH_A, 0.0);
				spwm_setDrvChannel(HTIM_DRV_CH_B, 0.0);
				spwm_setDrvChannel(HTIM_DRV_CH_C, 0.0);

				++_calibCounter;
			}
			else
			{
				// run 5% current to figure out the resistance
				float dc = 1.0f;
				spwm_setDrvChannel(HTIM_DRV_CH_A, dc*(_idRunCounter != 0));
				spwm_setDrvChannel(HTIM_DRV_CH_B, dc*(_idRunCounter != 1));
				spwm_setDrvChannel(HTIM_DRV_CH_C, dc*(_idRunCounter != 2));

				_calibCounter = 0;
				_state = MCTRL_INDUCTANCE_ID_RUN;
			}
			break;
		}
		case MCTRL_INDUCTANCE_ID_RUN:
		{
			if (_calibCounter < NUM_STATIC_MEASUREMENTS)
			{
				float vdda = ssf_getVdda();
				_convertAdcToCurrent(_lastMeasurement[_calibCounter], adcCounts, vdda);
				++_calibCounter;
			}
			else
			{
				spwm_setDrvChannel(HTIM_DRV_CH_A, 0.0);
				spwm_setDrvChannel(HTIM_DRV_CH_B, 0.0);
				spwm_setDrvChannel(HTIM_DRV_CH_C, 0.0);

				_state = MCTRL_INDUCTANCE_ID_FINISH;
			}
			break;
		}
		case MCTRL_RESISTANCE_ID_START:
		{
			// turn on PWM and wait for it to settle
			if (_calibCounter < NUM_ALIGN_WAIT_OFF_CYCLES)
			{
				float dc = RESISTANCE_ID_DC;
				spwm_setDrvChannel(HTIM_DRV_CH_A, dc*(_idRunCounter != 0));
				spwm_setDrvChannel(HTIM_DRV_CH_B, dc*(_idRunCounter != 1));
				spwm_setDrvChannel(HTIM_DRV_CH_C, dc*(_idRunCounter != 2));


				++_calibCounter;
			}
			else
			{
				_calibCounter = 0;
				_state = MCTRL_RESISTANCE_ID_RUN;
			}

			break;
		}
		case MCTRL_RESISTANCE_ID_RUN:
		{
			// measure steady-state current
			if (_calibCounter < NUM_STATIC_MEASUREMENTS)
			{
				float vdda = ssf_getVdda();
				_convertAdcToCurrent(_lastMeasurement[_calibCounter], adcCounts, vdda);
				++_calibCounter;
			}
			else
			{
				spwm_setDrvChannel(HTIM_DRV_CH_A, 0.0);
				spwm_setDrvChannel(HTIM_DRV_CH_B, 0.0);
				spwm_setDrvChannel(HTIM_DRV_CH_C, 0.0);

				_state = MCTRL_RESISTANCE_ID_FINISH;
			}
			break;
		}
		case MCTRL_DEMO:
		{
			float vdda = ssf_getVdda();
			float currents[6] = {0.0f};
			_convertAdcToCurrent(currents, adcCounts, vdda);

			// store phase currents for debugging purposes
			for (size_t i = 0; i < 6; ++i)
			{
				size_t pidx = ((int)(_phase*((PHASE_BUCKETS)/(M_PI*2.0f)))) % PHASE_BUCKETS;
				_phasedCurrents[i][pidx] = currents[i];
			}

			// "control" right now is just constant speed moves

			const float step = M_PI/(100.0e3/6.0/10)*0.5f;

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
		default:
		{
			break;
		}
	}

}

