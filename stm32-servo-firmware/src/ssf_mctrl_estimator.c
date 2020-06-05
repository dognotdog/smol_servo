
#include <stdbool.h>
#include <stdatomic.h>
#include <stddef.h>
#include <string.h>
#include <assert.h>

#include "ssf_mctrl_private.h"
#include "ssf_mctrl.h"
#include "ssf_main.h"
#include "ssf_spi.h"
#include "debug.h"

typedef struct {
	uint32_t timeStamp_us;
	uint32_t timeSpan_us;
	uint16_t encoderPosition; // 16bit per radian (encoder readout shifted!)
	uint16_t encoderDelta;
	uint16_t encoderDeltaDelta;
} ssf_encoderPosition_t;

typedef struct {
	float val, var;
} ssf_kalmanValue_t;


static mctrl_simplePositionEstimate_t _simpleMotorPositionEstimate = {.timeStamp_us = 0};

static uint32_t _lastDebugTimestamp = 0;
static mctrl_simplePositionEstimate_t _est0 = {.timeStamp_us = 0};
static mctrl_simplePositionEstimate_t _est1 = {.timeStamp_us = 0};

static volatile atomic_int_fast32_t _encoderPosUpdateCounter = 0;
static ssf_encoderPosition_t _lastEncoderPosition;



static volatile sspi_as5047_state_t _hallState;
static volatile uint32_t			_hallReadCounter;
static volatile uint32_t			_hallErrorCounter;
static volatile uint32_t			_hallFormatErrorCounter;
static volatile uint32_t			_hallValueErrorCounter;

bool ssf_atomicTryGetEncoderPosition(ssf_encoderPosition_t* val)
{
	atomic_int_fast32_t initialCount = atomic_load(&_encoderPosUpdateCounter);
	*val = _lastEncoderPosition;
	atomic_int_fast32_t finalCount = atomic_load(&_encoderPosUpdateCounter);

	if (initialCount == finalCount)
	{
		return true;
	}
	return false;
}

static ssf_kalmanValue_t _singleVarKalmanMeasurement(const ssf_kalmanValue_t v1, const ssf_kalmanValue_t v2)
{
	/*
		update estimate via Kalman Filter (#1 is old estimate, #2 is new measurement)
			K = sigma1^2 / (sigma1^2 + sigma2^2)
			x = x1 + K (x2 - x1)
			sigma^2 = (1 - K) sigma1^2
	*/

	const float K = v1.var/(v1.var + v2.var);
	ssf_kalmanValue_t v = {
		.val = v1.val + K*(v2.val - v1.val),
		.var = (1.0f - K) * v1.var,
	};
	return v;
}

/*
Matrix multiplication indices

     k ->          j ->          j ->
   .-----.       .-----.       .-----.
   |     |       |     |       |     |
 i |     |     k |     |     i |     |
   |     |  X    |     |  =    |     |
 | |     |     | |     |     | |     |
 v |     |     v |     |     v |     |
   '-----'       '-----'       '-----'

*/

#define MMUL(dst, a, b, li, lj, lk) 				\
{													\
	static_assert(sizeof(dst)/sizeof(dst[0])== (li), "1st dimension of dst mismatch");\
	static_assert(sizeof(dst[0])/sizeof(dst[0][0]) == (lj), "2nd dimension of dst mismatch");\
	static_assert(sizeof(a)/sizeof(a[0]) == (li), "1st dimension of a mismatch");\
	static_assert(sizeof(a[0])/sizeof(a[0][0]) == (lk), "2nd dimension of a mismatch");\
	static_assert(sizeof(b)/sizeof(b[0]) == (lk), "1st dimension of b mismatch");\
	static_assert(sizeof(b[0])/sizeof(b[0][0]) == (lj), "2nd dimension of b mismatch");\
	for (size_t i = 0; i < (li); ++i)				\
	{												\
		for (size_t j = 0; j < (lj); ++j)			\
		{											\
			for (size_t k = 0; k < (lk); ++k)		\
			{										\
				(dst)[i][j] += (a)[i][k]*(b)[k][j];	\
			}										\
		}											\
	}												\
}

#define MMULT(dst, a, bT, li, lj, lk) 			\
{												\
	static_assert(sizeof(dst)/sizeof(dst[0]) == (li), "1st dimension of dst mismatch");\
	static_assert(sizeof(dst[0])/sizeof(dst[0][0]) == (lj), "2nd dimension of dst mismatch");\
	static_assert(sizeof(a)/sizeof(a[0]) == (li), "1st dimension of a mismatch");\
	static_assert(sizeof(a[0])/sizeof(a[0][0]) == (lk), "2nd dimension of a mismatch");\
	static_assert(sizeof(bT)/sizeof(bT[0]) == (lj), "1st dimension of b mismatch");\
	static_assert(sizeof(bT[0])/sizeof(bT[0][0]) == (lk), "2nd dimension of b mismatch");\
	for (size_t i = 0; i < (li); ++i)			\
	{											\
		for (size_t j = 0; j < (lj); ++j)		\
		{										\
			for (size_t k = 0; k < (lk); ++k)	\
			{									\
				(dst)[i][j] += (a)[i][k]*(bT)[j][k]; \
			}									\
		}										\
	}											\
}


// ixk x kxj = ixj
// ixj x kxi = ixi
// j == k
// ixj x jxj x jxi
#define MTRANS(dst, a, b, li, ljk)		    \
{											\
	static_assert(sizeof(dst)/sizeof(dst[0]) == (li), "1st dimension of dst mismatch");\
	static_assert(sizeof(dst[0])/sizeof(dst[0][0]) == (li), "2nd dimension of dst mismatch");\
	static_assert(sizeof(a)/sizeof(a[0]) == (li), "1st dimension of a mismatch");\
	static_assert(sizeof(a[0])/sizeof(a[0][0]) == (ljk), "2nd dimension of a mismatch");\
	static_assert(sizeof(b)/sizeof(b[0]) == (ljk), "1st dimension of b mismatch");\
	static_assert(sizeof(b[0])/sizeof(b[0][0]) == (ljk), "2nd dimension of b mismatch");\
	float ab[li][ljk] = {};					\
	MMUL(ab, (a), (b), (li), (ljk), (ljk));	\
	MMULT((dst), ab, (a), (li), (li), (ljk));\
}

#define MADD(dst, a, b, li, lj) 				\
{												\
	static_assert(sizeof(dst)/sizeof(dst[0]) == (li), "1st dimension of dst mismatch");\
	static_assert(sizeof(dst[0])/sizeof(dst[0][0]) == (lj), "2nd dimension of dst mismatch");\
	static_assert(sizeof(a)/sizeof(a[0]) == (li), "1st dimension of a mismatch");\
	static_assert(sizeof(a[0])/sizeof(a[0][0]) == (lj), "2nd dimension of a mismatch");\
	for (size_t i = 0; i < (li); ++i)			\
	{											\
		for (size_t j = 0; j < (lj); ++j)		\
		{										\
			(dst)[i][j] = (a)[i][j] + (b)[i][j];\
		}										\
	}											\
}

#define MSUB(dst, a, b, li, lj) 				\
{												\
	static_assert(sizeof(dst)/sizeof(dst[0]) == (li), "1st dimension of dst mismatch");\
	static_assert(sizeof(dst[0])/sizeof(dst[0][0]) == (lj), "2nd dimension of dst mismatch");\
	static_assert(sizeof(a)/sizeof(a[0]) == (li), "1st dimension of a mismatch");\
	static_assert(sizeof(a[0])/sizeof(a[0][0]) == (lj), "2nd dimension of a mismatch");\
	static_assert(sizeof(b)/sizeof(b[0]) == (li), "1st dimension of b mismatch");\
	static_assert(sizeof(b[0])/sizeof(b[0][0]) == (lj), "2nd dimension of b mismatch");\
	for (size_t i = 0; i < (li); ++i)			\
	{											\
		for (size_t j = 0; j < (lj); ++j)		\
		{										\
			(dst)[i][j] = (a)[i][j] - (b)[i][j];\
		}										\
	}											\
}

#define MMULS(dst, a, b, li, lj) 			\
{											\
	static_assert(sizeof(dst)/sizeof(dst[0]) == (li), "1st dimension of dst mismatch");\
	static_assert(sizeof(dst[0])/sizeof(dst[0][0]) == (lj), "2nd dimension of dst mismatch");\
	static_assert(sizeof(a)/sizeof(a[0]) == (li), "1st dimension of a mismatch");\
	static_assert(sizeof(a[0])/sizeof(a[0][0]) == (lj), "2nd dimension of a mismatch");\
	for (size_t i = 0; i < (li); ++i)		\
	{										\
		for (size_t j = 0; j < (lj); ++j)	\
		{									\
			(dst)[i][j] = (a)[i][j]*(b);	\
		}									\
	}										\
}

// static void _mmul2x2(float dst[2][2], const float a[2][2], const float b[2][2])
// {
// 	memset(dst, 0, sizeof(float)*4);
// 	MMUL(dst, a, b, 2, 2, 2);
// }

// static void _mmul2x2t(float dst[2][2], const float a[2][2], const float bT[2][2])
// {
// 	// multiply A by the transpose of B
// 	memset(dst, 0, sizeof(float)*4);
// 	MMULT(dst, a, bT, 2, 2, 2);
// }

// static void _mtransform2x2(float dst[2][2], const float a[2][2], const float b[2][2])
// {
// 	float ab[2][2] = {};
// 	_mmul2x2(ab, a, b);
// 	_mmul2x2t(dst, ab, a);
// }

static inline float _modAngle(const float a)
{
	return fmodf(fmodf(a, 2.0f*M_PI) + M_PI + (a < 0.0f)*(2.0f*M_PI), 2.0f*M_PI) - M_PI;
}


static mctrl_simplePositionEstimate_t _estimateSimpleMotorPosition(const mctrl_simplePositionEstimate_t prev, uint32_t now_us)
{
	// static ssf_encoderPosition_t oldEncoderPos = {.timeSpan_us = -1};
	ssf_encoderPosition_t encoderPos = {.timeSpan_us = -1};
	bool haveMeasurement = ssf_atomicTryGetEncoderPosition(&encoderPos);

	// float h = 1.0e-6f*(now_us - prev.timeStamp_us);
	float h = 60.0e-6f;

	float A[2][2] = { 
		{1.0f, h   },
		{0.0f, 1.0f},
	};

	// lets give x a bigger uncertainyy
	float q11 = 0.2f*M_PI*h; //*h*0.5f;
	float q22 = 2.0f*M_PI*h;// + h*fabsf(alpha);
	float Q[2][2] = { 
		{q11*q11, 0.0f},
		{0.0f, q22*q22},
	};

	float H[1][2] = { 
		{1.0f, 0.0f},
	};


	// a priori update
	// x = A*x + B*u
	float x_pre[2][1] = {};

	MMUL(x_pre, A, prev.x, 2,1,2)
	// x_pre[0][0] = _modAngle(x_pre[0][0]);

	// P + A*P*A' + Q
	float APA[2][2] = {};
	MTRANS(APA, A, prev.P, 2,2);
	float P_pre[2][2] = {};
	MADD(P_pre, APA, Q, 2,2);

	float I[2][2] = {
		{1.0f, 0.0f}, 
		{0.0f, 1.0f}
	};

	mctrl_simplePositionEstimate_t result = {
		.timeStamp_us = now_us,
		.lastMeasurementTimeStamp_us = prev.lastMeasurementTimeStamp_us,
	};

	memcpy(result.debug.x_pre, x_pre, sizeof(result.debug.x_pre));

	if (haveMeasurement && (prev.lastMeasurementTimeStamp_us != encoderPos.timeStamp_us))
	{
		result.lastMeasurementTimeStamp_us = encoderPos.timeStamp_us;

		float lambda = 1.0e-6f*(now_us - encoderPos.timeStamp_us);
		float rawEncoderPos = (float)encoderPos.encoderPosition/0x10000*2.0f*M_PI;

		// extrapolate reading to now
		float zRaw = rawEncoderPos + lambda * x_pre[1][0];
		// mod angle z to be in the range  x +- pi
		float z = x_pre[0][0] + _modAngle(zRaw - x_pre[0][0]);

		float Rsqrt = 0.0222f;// + 2.0f*M_PI*lambda;
		float R = Rsqrt*Rsqrt;

		// K = P*H'*(H*P*H' + R)^-1
		float hph[1][1] = {};
		MTRANS(hph, H, P_pre, 1,2);
		float hph_r = hph[0][0] + R;
		float PHt[2][1] = {};
		MMULT(PHt, P_pre, H, 2, 1, 2);
		float K[2][1] = {};
		MMULS(K, PHt, 1.0f/hph_r, 2,1);

		// x = x + K*(z - H*x)
		float Hx[1][1] = {};
		MMUL(Hx, H, x_pre, 1,1,2)
		float Kz_Hx[2][1] = {};
		MMULS(Kz_Hx, K, z - Hx[0][0], 2,1);


		float x_post[2][1] = {};
		MADD(x_post, x_pre, Kz_Hx, 2,1);

		// x_post[0][0] = _modAngle(x_post[0][0]);

		// P = (I-K*H)*P
		float KH[2][2] = {};
		MMUL(KH, K,H, 2,2,1);

		float I_KH[2][2] = {};
		MSUB(I_KH, I, KH, 2,2);

		float P_post[2][2] = {};
		MMUL(P_post, I_KH, P_pre, 2,2,2);

		memcpy(result.x, x_post, sizeof(result.x));
		memcpy(result.P, P_post, sizeof(result.P));

		memcpy(result.debug.K, K, sizeof(result.debug.K));
		memcpy(result.debug.z, &z, sizeof(result.debug.z));
		memcpy(result.debug.x_post, x_post, sizeof(result.debug.x_post));
		memcpy(result.debug.Kz_Hx, Kz_Hx, sizeof(result.debug.Kz_Hx));
		memcpy(&result.debug.Q, Q, sizeof(result.debug.Q));
		result.debug.R = R;
	}
	else
	{
		memcpy(result.x, x_pre, sizeof(result.x));
		memcpy(result.P, P_pre, sizeof(result.P));

		memcpy(result.debug.K, prev.debug.K, sizeof(result.debug.K));
		memcpy(result.debug.z, prev.debug.z, sizeof(result.debug.z));
		memcpy(&result.debug.Q, Q, sizeof(result.debug.Q));
		result.debug.R = prev.debug.R;
	}

	// remove cross correlations for better numerical stability
	// result.P[1][0] = 0.0f;
	// result.P[0][1] = 0.0f;

	result.x[0][0] = _modAngle(result.x[0][0]);

	if ((now_us -_lastDebugTimestamp > 1000000u) && haveMeasurement && (prev.lastMeasurementTimeStamp_us != encoderPos.timeStamp_us))
	{
		_est0 = prev;
		_est1 = result;
		_lastDebugTimestamp = now_us;
	}


	return result;
}

mctrl_simplePositionEstimate_t mctrl_getSimpleMotorPositionEstimate(void)
{
	return _simpleMotorPositionEstimate;
}

void mctrl_updateSimpleSensorEstimate(uint32_t now_us)
{
	mctrl_simplePositionEstimate_t newEstimate = _estimateSimpleMotorPosition(_simpleMotorPositionEstimate, now_us);

	_simpleMotorPositionEstimate = newEstimate;
}

void ssf_asyncReadHallSensorCallback(sspi_as5047_state_t sensorState, bool transferOk)
{
	uint32_t timeSpan = sensorState.end_us - sensorState.start_us;
	uint32_t midTime = sensorState.start_us + timeSpan/2u;

	bool formatError = false, valueError = false;

	if (ssf_checkSpiEncoderReadOk(sensorState, &formatError, &valueError))
	{
		uint16_t encoderRead = (sensorState.ANGLEUNC & 0x3FFF) << 2;
		uint16_t delta = encoderRead - _lastEncoderPosition.encoderPosition;
		uint16_t delta2 = delta - _lastEncoderPosition.encoderDelta;

		ssf_encoderPosition_t pos = {
			.timeStamp_us = midTime,
			.timeSpan_us = timeSpan,
			.encoderPosition = encoderRead,
			.encoderDelta = delta,
			.encoderDeltaDelta = delta2,
		};

		_lastEncoderPosition = pos;
		atomic_store(&_encoderPosUpdateCounter, _encoderPosUpdateCounter+1);
	}
	else if (transferOk)
	{
		++_hallErrorCounter;
		if (formatError)
			++_hallFormatErrorCounter;
		if (valueError)
			++_hallValueErrorCounter;
		// ssf_dbgPrintEncoderStatus(sensorState); 
	}

	_hallState = sensorState;
	++_hallReadCounter;

}

float ssf_getEncoderAngle(void)
{
	return (float)(uint32_t)_lastEncoderPosition.encoderPosition * (2.0f*M_PI / (float)0x10000);
}

/**
	Try to get encoder angle, only succeeds if no update happens during readout
*/
bool ssf_atomicTryGetEncoderAngle(float* val, uint32_t* time_us)
{
	size_t initialCount = atomic_load(&_encoderPosUpdateCounter);
	float angle =  ssf_getEncoderAngle();
	uint32_t time = _lastEncoderPosition.timeStamp_us;
	size_t finalCount = atomic_load(&_encoderPosUpdateCounter);

	if (initialCount == finalCount)
	{
		*val = angle;
		*time_us = time;
		return true;
	}
	return false;
}


uint32_t ssf_dbgGetEncoderReadCounter(void)
{
	return _hallReadCounter;
}
uint32_t ssf_dbgGetEncoderErrorCounter(uint32_t* formatErrors, uint32_t* valueErrors)
{
	if (formatErrors)
		*formatErrors = _hallFormatErrorCounter;
	if (valueErrors)
		*valueErrors = _hallValueErrorCounter;
	return _hallErrorCounter;
}


sspi_as5047_state_t ssf_dbgGetLastEncoderReading(void)
{
	return _hallState;
}


void mctrl_dbgPrintSimpleEstimate(mctrl_simplePositionEstimate_t est)
{
	dbg_printf("x = [%.7e; %.7e]\r\n", 
		(double)((est.x[0][0])), 
		(double)((est.x[1][0]))
		);
	dbg_printf("x_pre  = [%.7e; %.7e]\r\n", 
		(double)((est.debug.x_pre[0][0])), 
		(double)((est.debug.x_pre[1][0]))
		);
	dbg_printf("x_post = [%.7e; %.7e]\r\n", 
		(double)((est.debug.x_post[0][0])), 
		(double)((est.debug.x_post[1][0]))
		);


	dbg_printf("z = [%.7e]\r\n", 
		(double)((est.debug.z[0][0]))
		);
	dbg_printf("Kz_Hx = [%.7e; %.7e]\r\n", 
		(double)((est.debug.Kz_Hx[0][0])), 
		(double)((est.debug.Kz_Hx[1][0]))
		);
	dbg_printf("P = [%.7e %.7e; %.7e %.7e]\r\n", 
		(double)((est.P[0][0])), 
		(double)((est.P[0][1])),
		(double)((est.P[1][0])), 
		(double)((est.P[1][1]))
		);
	dbg_printf("Q = [%.7e %.7e; %.7e %.7e]\r\n", 
		(double)((est.debug.Q[0][0])), 
		(double)((est.debug.Q[0][1])),
		(double)((est.debug.Q[1][0])), 
		(double)((est.debug.Q[1][1]))
		);
	dbg_printf("K = [%.7e; %.7e]\r\n", 
		(double)((est.debug.K[0][0])), 
		(double)((est.debug.K[1][0]))
		);
	dbg_printf("R = [%.7e]\r\n", 
		(double)((est.debug.R))
		);
	dbg_printf("t =  %.7e\r\ntm = %.7e\r\n", (double)(1.0e-6f*est.timeStamp_us), (double)(1.0e-6f*est.lastMeasurementTimeStamp_us));

}

float mctrl_getSimpleMotorSpeedEstimate(void)
{
	return _simpleMotorPositionEstimate.x[1][0];
}

void mctrl_dbgPrintSimpleEstimatePair(void)
{
	static uint32_t lastPrintout = 0;

	if (lastPrintout != _lastDebugTimestamp)
	{
		mctrl_dbgPrintSimpleEstimate(_est0);
		mctrl_dbgPrintSimpleEstimate(_est1);
		lastPrintout = _lastDebugTimestamp;
	}
}

