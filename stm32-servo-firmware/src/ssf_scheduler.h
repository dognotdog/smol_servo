#ifndef SSF_SCHEDULER_H
#define SSF_SCHEDULER_H

#include <stdint.h>
#include <stdbool.h>

typedef enum {
	SCHED_NONE,
	SCHED_UI_SLOW,
	SCHED_COUNT
} sch_taskId_t;

typedef void (*sch_taskFun_t)(uint32_t now_us);


typedef struct {
	sch_taskFun_t 	taskFun;
	int32_t		period_us, offset_us;
} ssf_scheduledTask_t;


void ssf_scheduleInit(void);
void ssf_enableTasks(sch_taskId_t first, sch_taskId_t last, bool enable);
void ssf_scheduleTasks(sch_taskId_t first, sch_taskId_t last, uint32_t now_us);

#endif // SSF_SCHEDULER_H
