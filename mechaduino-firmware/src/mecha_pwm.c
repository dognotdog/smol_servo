/*
Motor Driver Config:
12	PA7		VREF1	V0	TCC1/1	(TCC1/1 TCC3/5)
11	PA6		IN1		F0 	TCC1/0	(TCC1/0 TCC3/4)
30	PA21	IN2		R0 	TCC0/7	(TC7/1  TCC0/7)
24	PA15	IN3		F1 	TCC0/5	(TC3/1  TCC0/5)
29	PA20	IN4		R1 	TCC0/6	(TC7/0  TCC0/6)
13	PA8		VREF2	V1 	TCC0/0	(TCC0/0 TCC1/2)

On the motor driver, Rs = 0.15Ohm, and Imax = Vref/(Av * Rs), where according to the datasheet Av is about a factor of 10.0:
	Imax	Vref	x1024	x256
	0.1A	0.15V	46		12
	0.5A	0.75V	232		58
	1.0A	1.5V	465		116
	2.2A	3.3V	1023	255
*/

#include "sam.h"

#include "mecha_pwm.h"

#include <stddef.h>

#define MPWM_BITS	10
#define MPWM_MAX	((1 << MPWM_BITS)-1)
#define MPWM_VREF	((int)(0.5*MPWM_MAX*10.0*0.15/3.3 + 0.5))


static const struct {
	Tcc* tcc;
	size_t ch;
} _chMap[MPWM_NUM_CHANNELS] = {
	[MPWM_V0] = {TCC1,1},
	[MPWM_F0] = {TCC1,0},
	[MPWM_R0] = {TCC0,3},
	[MPWM_V1] = {TCC0,0},
	[MPWM_F1] = {TCC0,1},
	[MPWM_R1] = {TCC0,2},
};

static const uint32_t _revMap[2][4] = {
	[0] = {MPWM_V1, MPWM_F1, MPWM_R1, MPWM_R0},
	[1] = {MPWM_F0, MPWM_V0},
};


void mpwm_setPeriodAndDuty(const uint32_t period, const uint32_t onTimes[MPWM_NUM_CHANNELS])
{
	TCC0->PERB.bit.PERB = period;
	for (size_t i = 0; i < 4; ++i)
	{
		size_t ch = _revMap[0][i];
		TCC0->CCB[i].bit.CCB = onTimes[ch];
	}

	TCC1->PERB.bit.PERB = period;
	for (size_t i = 0; i < 2; ++i)
	{
		size_t ch = _revMap[1][i];
		TCC1->CCB[i].bit.CCB = onTimes[ch];
	}
}

void mpwm_setPwm(const mpwm_id_t ch, const uint32_t pwm)
{
	_chMap[ch].tcc->CCB[_chMap[ch].ch].bit.CCB = pwm;
}



fix1p14_t sin1p14[32] = {0,1606,3196,4756,6270,7723,9102,10394,11585,12665,13623,14449,15137,15679,16069,16305,16384,16305,16069,15679,15137,14449,13623,12665,11585,10394,9102,7723,6270,4756,3196,1606};


// normalized input fixed point sine lookup
fix1p30_t sinfixn(fix2u30_t xn)
{
	// normalized x
	// 2^30 represents 2pi
	// 2^29 represents pi

	// with a 32 entry table for 0..pi, 2^5 are integer entries
	// 2^24 is the fractional part
	int32_t xint = xn >> 24;
	fix5p24_t xfrac24 = xn & 0xFFFFFF;

	// sin1p14 is in a Q1.14 format, and multiplying that with a fraction of Q1.16 gives Q1.30
	fix15p16_t xf = (xfrac24+128) >> 8;
	fix15p16_t xf1 = (1 << 16) - xf;

	int32_t xl0 = xint     % 32;
	int32_t xl1 = (xint+1) % 32;
	fix1p14_t y0 = sin1p14[xl0];
	fix1p14_t y1 = sin1p14[xl1];

	int sign = xint < 32 ? 1 : -1;
	fix1p30_t y = sign*(y0*xf1 + y1*xf);
	return y;
}

// normalized input fixed point sine lookup
fix1p30_t _sinfixn(fix1p30_t xn)
{
	// normalized x
	// 2^30 represents 2pi (64 / 2^6)
	// 2^29 represents pi  (32 / 2^5)

	// with a 32 entry table for 0..pi, 2^5 are integer entries
	// 2^24 is the fractional part
	int32_t xint = xn >> 24;

	// sin1p14 is in a Q1.14 format, and multiplying that with a fraction of Q1.16 gives Q1.30

	int32_t xl0 = xint     % 32;
	fix1p14_t y0 = sin1p14[xl0];

	int sign = xint < 32 ? 1 : -1;
	fix1p30_t y = sign*y0 << 16;
	return y;
}


static fix1p30_t step  = (1L << 30)/100; // 1/1000
static fix1p30_t phase = (1L << 30)/4; // 1/4
static fix1p30_t rotation = 0;
fix1p30_t mpwm_sines[2];

void TC4_Handler(void)
{	
	// clear TC interrupt flag, otherwise the interrupt keeps being continously activated in NVIC
	while (TC4->COUNT16.STATUS.bit.SYNCBUSY) {};
	TC4->COUNT16.INTFLAG.bit.MC0 = 1;

	fix1p30_t x0 = rotation;
	fix1p30_t x1 = (rotation + phase) % (1 << 30);

	fix1p30_t y0 = sinfixn(x0);
	fix1p30_t y1 = sinfixn(x1);
	int sign0 = y0 < 0 ? -1 : 1;
	int sign1 = y1 < 0 ? -1 : 1;

	int32_t pwmF0 = MPWM_MAX*(sign0 > 0);
	int32_t pwmR0 = MPWM_MAX*(sign0 < 0);
	int32_t pwmV0 = (((sign0*y0 + (1 << (29-MPWM_BITS))) >> (30-MPWM_BITS))*MPWM_VREF) >> MPWM_BITS;

	// need to go from Q1.30 to Q1.10


	int32_t pwmF1 = MPWM_MAX*(sign1 > 0);
	int32_t pwmR1 = MPWM_MAX*(sign1 < 0);
	int32_t pwmV1 = (((sign1*y1 + (1 << (29-MPWM_BITS))) >> (30-MPWM_BITS))*MPWM_VREF) >> MPWM_BITS;


	mpwm_setPwm(MPWM_F0, pwmF0);
	mpwm_setPwm(MPWM_R0, pwmR0);
	mpwm_setPwm(MPWM_V0, pwmV0);
	mpwm_setPwm(MPWM_F1, pwmF1);
	mpwm_setPwm(MPWM_R1, pwmR1);
	mpwm_setPwm(MPWM_V1, pwmV1);

	mpwm_sines[0] = pwmV0;
	mpwm_sines[1] = pwmV1;
	rotation = (rotation + step) % (1 << 30);
}

void mpwm_init(void)
{
	// wait for all registers to sync
	// while (TCC0->SYNCBUSY.reg & TCC_SYNCBUSY_MASK) {};
	// while (TCC1->SYNCBUSY.reg & TCC_SYNCBUSY_MASK) {};

	// enable lock update to enable double buffering
	TCC0->CTRLBCLR.bit.LUPD = 1;
	TCC1->CTRLBCLR.bit.LUPD = 1;
	uint32_t onTimes[MPWM_NUM_CHANNELS] = {
		[MPWM_V0] = 0,
		[MPWM_F0] = MPWM_MAX, // turn R0 and F0 on for "braking" (both halves low)
		[MPWM_R0] = MPWM_MAX,
		[MPWM_V1] = 0,
		[MPWM_F1] = MPWM_MAX, // turn R1 and F1 on for "braking" (both halves low)
		[MPWM_R1] = MPWM_MAX,
	};
	mpwm_setPeriodAndDuty(MPWM_MAX, onTimes);

	TCC0->CTRLA.bit.ENABLE=1;
	TCC1->CTRLA.bit.ENABLE=1;

	// setup TC4 for IRQ triggering the fast update interrupt
	while (TC4->COUNT16.STATUS.bit.SYNCBUSY) {};
	TC4->COUNT16.CTRLA.bit.ENABLE = 0;
	while (TC4->COUNT16.STATUS.bit.SYNCBUSY) {};
	TC4->COUNT16.CTRLA.bit.MODE = TC_CTRLA_MODE_COUNT16_Val;
	while (TC4->COUNT16.STATUS.bit.SYNCBUSY) {};
	TC4->COUNT16.CTRLA.bit.WAVEGEN = TC_CTRLA_WAVEGEN_MFRQ_Val;
	while (TC4->COUNT16.STATUS.bit.SYNCBUSY) {};
	// TC4->COUNT16.CTRLA.bit.PRESCALER = TC_CTRLA_PRESCALER_DIV16_Val;
	// while (TC4->COUNT16.STATUS.bit.SYNCBUSY) {};
	// set to count to 4800 for a 10kHz update rate
	TC4->COUNT16.CC[0].bit.CC = 2047;
	TC4->COUNT16.CC[1].bit.CC = 2047;

	NVIC_EnableIRQ(TC4_IRQn);
	while (TC4->COUNT16.STATUS.bit.SYNCBUSY) {};
	TC4->COUNT16.INTENSET.bit.MC0 = 1;

	while (TC4->COUNT16.STATUS.bit.SYNCBUSY) {};
	TC4->COUNT16.CTRLA.bit.ENABLE = 1;
	while (TC4->COUNT16.STATUS.bit.SYNCBUSY) {};

}
