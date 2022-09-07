#ifndef SSF_MCTRL_H
#define SSF_MCTRL_H

#include "ssf_spi.h"

#include <stdint.h>
#include <stddef.h>

typedef struct {
	float x[2][1];
	float P[2][2];
	struct {
		float z[1][1];
		float x_pre[2][1];
		float x_post[2][1];
		float Kz_Hx[2][1];
		float R;
		float Q[2][2];
		float K[2][1];
	} debug;
	uint32_t timeStamp_us;
	uint32_t lastMeasurementTimeStamp_us;
} mctrl_simplePositionEstimate_t;


// constrain angle to +-pi
float mctrl_modAngle(const float a);

extern float ssf_getEncoderAngle(void);
bool ssf_atomicTryGetEncoderAngle(float* val, uint32_t* time_us);
extern uint32_t ssf_dbgGetEncoderReadCounter(void);
extern uint32_t ssf_dbgGetEncoderErrorCounter(uint32_t* formatErrors, uint32_t* valueErrors);
extern sspi_as5047_state_t ssf_dbgGetLastEncoderReading(void);

extern void mctrl_init(void);
extern void mctrl_fastLoop(const uint16_t adcCounts[6]);
extern float* mctrl_getPhaseTable(size_t i);
extern void mctrl_idle(uint32_t now_us);

extern mctrl_simplePositionEstimate_t mctrl_getSimpleMotorPositionEstimate(void);

extern void mctrl_dbgPrintSimpleEstimatePair(void);
extern void mctrl_dbgPrintSimpleEstimate(mctrl_simplePositionEstimate_t est);

extern void mctrl_diag_tmc6200_gstat_rx_callback(uint8_t rx[5]);
extern void mctrl_diag_tmc6200_ioin_rx_callback(uint8_t rx[5]);
extern void mctrl_diag_tmc6200_gconf_rx_callback(uint8_t rx[5]);

#endif // SSF_MCTRL_H
