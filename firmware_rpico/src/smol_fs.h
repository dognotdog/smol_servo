#pragma once

#include "lfs.h"

#include <stdbool.h>

lfs_t* smol_lfs(void);

// mount only. A necessary format is deferred to smol_fs_run().
void smol_fs_init(void);
// core 0 main loop: the deferred format, after the PD contract and the core 1 init
void smol_fs_run(void);
bool smol_fs_is_mounted(void);
