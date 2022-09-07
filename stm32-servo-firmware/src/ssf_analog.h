#ifndef SSF_ANALOG_H
#define SSF_ANALOG_H

#include <stdbool.h>
#include <stdint.h>

typedef enum {
	ISENSE_CH_A_LO,
	ISENSE_CH_A_HI,
	ISENSE_CH_B_LO,
	ISENSE_CH_B_HI,
	ISENSE_CH_C_LO,
	ISENSE_CH_C_HI,
	ISENSE_NUMCHANNELS
} isense_channels_t;


void ssfa_isenseCalibrateAdcZeros(const uint16_t adcCounts[ISENSE_NUMCHANNELS], bool bridgeState);

void ssfa_isenseResetAdcZeroCalibration(void);

#endif // SSF_ANALOG_H
