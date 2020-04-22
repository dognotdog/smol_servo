#ifndef SSF_PERF_COUNTERS_H
#define SSF_PERF_COUNTERS_H

#include "ssf_main.h"

// MCU variations:
// STM32F030x6
// STM32F070x6
// STM32G431xx


typedef enum {
	PERFC_SCHEDULED_LAST = SCHED_COUNT, // first entries are scheduled idle loop calls
	PERFC_IRQ_1MS,
	PERFC_IRQ_16MS,
	PERFC_IRQ_256MS,
	PERFC_IDLE,
	PERFC_NUM
} perf_counter_t;


#endif // SSF_PERF_COUNTERS_H
