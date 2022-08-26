#include "ssf_supervisor.h"

#include <string.h>

/**
 * This module runs supervision and safety checks
 */

static bool _supflags[SSF_SUP_FLAG_COUNT] = {false};

void ssf_supervisorInit(void)
{
	memset(_supflags, 0, sizeof(_supflags));
}

bool ssf_supervisorCheckFlag(ssf_supervisor_flag_t flag)
{
	return _supflags[flag];
}

void ssf_supervisorWriteFlag(ssf_supervisor_flag_t flag, bool value)
{
	_supflags[flag] = value;
}
