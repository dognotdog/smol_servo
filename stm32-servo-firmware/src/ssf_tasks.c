

#include "ssf_main.h"


ssf_scheduledTask_t tasks[SCHED_COUNT] = {
	[SCHED_UI_SLOW] = {ssf_ui1sTask, 1000000, 0},
};

