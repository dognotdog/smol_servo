
#include "ssf_main.h"

#include <stdbool.h>
#include <stddef.h>

static struct{
	uint32_t lastCall;
	bool	isEnabled;
} taskStates[SCHED_COUNT];




void ssf_scheduleInit(void)
{
	// init scheduler state
	// set lastCall times to initial offsets
	for (size_t i = SCHED_NONE; i < SCHED_COUNT; ++i)
	{
		taskStates[i].lastCall = tasks[i].offset_us;
	}

}

void ssf_scheduleTasks(sch_taskId_t first, sch_taskId_t last, uint32_t now_us)
{
	for (size_t i = first; i <= last; ++i)
	{
		uint32_t callTime = taskStates[i].lastCall + tasks[i].period_us;
		if ((int32_t)now_us - (int32_t)callTime >= 0)
		{
			if (taskStates[i].isEnabled && tasks[i].taskFun)
			{
				tasks[i].taskFun(now_us);
			}
			// last call time is updated regardless of task being enabled
			taskStates[i].lastCall = callTime;
		}
	}
}

