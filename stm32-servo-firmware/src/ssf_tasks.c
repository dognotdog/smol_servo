

#include "ssf_main.h"


ssf_scheduledTask_t tasks[SCHED_COUNT] = {
	[SCHED_UI_FAST] = {ssf_uiFastTask, 20000, 0},
};

