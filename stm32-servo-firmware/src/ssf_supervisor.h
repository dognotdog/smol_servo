#ifndef SSF_SUPERVISOR_H
#define SSF_SUPERVISOR_H

#include <stdbool.h>

typedef enum 
{
	SSF_SUP_FLAG_INVALID,
	SSF_SUP_FLAG_SPI_READY,
	SSF_SUP_FLAG_DRV8323_PRESENT,
	SSF_SUP_FLAG_TMC6200_PRESENT,
	SSF_SUP_FLAG_3V3_OK,
	SSF_SUP_FLAG_VDD_OK,
	SSF_SUP_FLAG_VBUS_OK,
	SSF_SUP_FLAG_COUNT,
} ssf_supervisor_flag_t;

void ssf_supervisorInit(void);

bool ssf_supervisorCheckFlag(ssf_supervisor_flag_t flag);
void ssf_supervisorWriteFlag(ssf_supervisor_flag_t flag, bool value);

static inline void ssf_supervisorClearFlag(ssf_supervisor_flag_t flag) 
{
	ssf_supervisorWriteFlag(flag, false);
}

static inline void ssf_supervisorSetFlag(ssf_supervisor_flag_t flag)
{
	ssf_supervisorWriteFlag(flag, true);
}

#endif // SSF_SUPERVISOR_H
