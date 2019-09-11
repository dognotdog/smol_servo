#ifndef SSF_MAIN_H
#define SSF_MAIN_H

#include <stdint.h>

typedef enum {
	SCHED_NONE,
	SCHED_UI_FAST,
	SCHED_COUNT
} sch_taskId_t;

typedef void (*sch_taskFun_t)(uint32_t now_us);


typedef struct {
	sch_taskFun_t 	taskFun;
	int32_t		period_us, offset_us;
} ssf_scheduledTask_t;


extern ssf_scheduledTask_t tasks[SCHED_COUNT];

extern void ssf_init(void);
extern void ssf_idle(void);


extern void ssf_ledInit(void);
extern void ssf_ledIdle(void);


extern void ssf_uiFastTask(uint32_t now_us);

#endif // SSF_MAIN_H
