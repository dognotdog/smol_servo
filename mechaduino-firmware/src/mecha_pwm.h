#ifndef MECHA_PWM_H
#define MECHA_PWM_H


#include <stdint.h>

typedef enum {
	// MPWM_INVALID,
	MPWM_V0,
	MPWM_F0,
	MPWM_R0,
	MPWM_V1,
	MPWM_F1,
	MPWM_R1,
	MPWM_NUM_CHANNELS
} mpwm_id_t;

typedef int16_t fix1p14_t;
typedef int32_t fix1p30_t;
typedef uint32_t fix2u30_t;

typedef int32_t fix5p24_t;
typedef int32_t fix15p16_t;


void mpwm_init(void);

void mpwm_setPwm(const mpwm_id_t ch, const uint32_t pwm);
void mpwm_setPeriodAndDuty(const uint32_t period, const uint32_t onTimes[MPWM_NUM_CHANNELS]);



#endif // MECHA_PWM_H
