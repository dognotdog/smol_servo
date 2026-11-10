#include "smol_servo.h"

#include "hardware/flash.h"
#include "pico/flash.h"
#include "pico/time.h"
#include "pico/platform/panic.h"


#include "pico_debug.h"
#include "smol_config.h"

#include "lfs.h"

#include <stddef.h>
#include <string.h>

extern char __flash_binary_start;
extern char __flash_storage_begin;
extern char __flash_storage_end;

#define FS_BLOCK_SIZE (FLASH_SECTOR_SIZE)
#define FS_TIMEOUT_MS (100)


// variables used by the filesystem
static lfs_t lfs;
static bool _mounted = false;

bool smol_fs_is_mounted(void) {
	return _mounted;
}

lfs_t* smol_lfs(void) {
	return &lfs;
}

static int _flash_read(const struct lfs_config *c, lfs_block_t block, lfs_off_t off, void *buffer, lfs_size_t size) {
	const uint8_t* flash_begin = &__flash_storage_begin;
	memcpy(buffer, flash_begin + block * c->block_size + off, size);

	return LFS_ERR_OK;
}

typedef struct {
	uintptr_t offs;
	const void* buffer;
	size_t size;
} _program_params_t;

static void _safe_prog(void* context) {
	_program_params_t* param = context;
	flash_range_program(param->offs, param->buffer, param->size);
}

static int _flash_prog(const struct lfs_config *c, lfs_block_t block, lfs_off_t off, const void *buffer, lfs_size_t size) {
	uintptr_t flash_begin = (uintptr_t)&__flash_binary_start;
	uintptr_t storage_begin = (uintptr_t)&__flash_storage_begin;
	uintptr_t flash_offs = (storage_begin - flash_begin) + block * c->block_size + off;

	_program_params_t param = {
		.offs = flash_offs,
		.buffer = buffer,
		.size = size,
	};

	int result = flash_safe_execute(_safe_prog, &param, FS_TIMEOUT_MS);
	if (result != PICO_OK) {
		err_println("FLASH write failed with error %i", result);
		return LFS_ERR_IO;
	}

	const uint8_t* written_block = (const uint8_t*)flash_begin + flash_offs;

	result = memcmp(buffer, written_block, size);
	if (result != 0) {
		err_println("FLASH write verification failed.");		
		return LFS_ERR_CORRUPT;
	}

	return LFS_ERR_OK;
}

static void _safe_erase(void* context) {
	uintptr_t* offs = context;
	flash_range_erase(*offs, 1);
}

static int _flash_erase(const struct lfs_config *c, lfs_block_t block) {
	assert(c->block_size == FS_BLOCK_SIZE);
	uintptr_t flash_begin = (uintptr_t)&__flash_binary_start;
	uintptr_t storage_begin = (uintptr_t)&__flash_storage_begin;
	uintptr_t flash_offs = (storage_begin - flash_begin) + block * c->block_size;

	int result = flash_safe_execute(_safe_erase, &flash_offs, FS_TIMEOUT_MS);
	if (result != PICO_OK) {
		err_println("FLASH erase failed with error %i.", result);	
		return LFS_ERR_IO;
	}

	const uint8_t* erased_block = (const uint8_t*)flash_begin + flash_offs;

	for (size_t i = 0; i < c->block_size; ++i) {
		if (erased_block[i] != 0xFF) {
			err_println("FLASH erase verification failed.");	
			return LFS_ERR_CORRUPT;
		}
	}

	return LFS_ERR_OK;
}

static int _flash_sync(const struct lfs_config *c) {
	return LFS_ERR_OK;
}


// configuration of the filesystem. block_count is not a const expression, smol_fs_init() sets it.
static struct lfs_config _cfg = {
	// block device operations
	.read  = _flash_read,
	.prog  = _flash_prog,
	.erase = _flash_erase,
	.sync  = _flash_sync,

	// block device configuration
	.read_size = 16,
	.prog_size = FLASH_PAGE_SIZE,
	.block_size = FS_BLOCK_SIZE,
	.cache_size = FLASH_PAGE_SIZE,
	.lookahead_size = 16,
	.block_cycles = 500,
};

/**
 * Format deferral, see notes/littlefs-deferred-format.md.
 * A flash erase disables the core 0 interrupts, and core 0 cannot answer USB PD messages during that time. A format before the PD contract can cause a Hard Reset and a reboot loop.
 * Also, the build sets PICO_FLASH_ASSUME_CORE1_SAFE=1: a flash write is safe only after core 1 runs from RAM only.
 */
// the PD contract voltage must not change for this time before the format. This includes an EPR entry after the first SPR contract (EPR entry delay + SinkEPREnterTimer + PSTransitionTimer is approx. 1.7 s).
#define FS_FORMAT_PD_SETTLE_US (3000000)

static bool _format_pending = false;
// contract voltage at the last smol_fs_run(), and the time of its last change
static uint32_t _settle_mv = 0;
static uint64_t _settle_since_us = 0;

/**
 * Mount the filesystem. A mount only reads the flash.
 * If the mount fails, the format does not occur here. smol_fs_run() does the format later, see _format_pending.
 */
void smol_fs_init(void) {
	_cfg.block_count = (uintptr_t)(&__flash_storage_end - &__flash_storage_begin) / FS_BLOCK_SIZE;

	int err = lfs_mount(&lfs, &_cfg);
	_mounted = (err == 0);
	if (_mounted) {
		info_println("FS mounted, %u blocks of %u bytes", (unsigned)_cfg.block_count, (unsigned)_cfg.block_size);
		return;
	}

	// this occurs on the first boot, or when the FS is corrupt
	_format_pending = true;
	_settle_mv = smol_usb_pd_contract_mv();
	_settle_since_us = time_us_64();
	warn_println("FS mount failed (%d), format and config load deferred until the PD contract is stable for %u ms and core 1 runs from RAM", err, (unsigned)(FS_FORMAT_PD_SETTLE_US / 1000));
}

/**
 * Core 0 main loop function. Does the deferred format when all these conditions are true:
 * 1. A PD contract exists (smol_usb_pd_contract_mv() > 0).
 * 2. The contract voltage did not change for FS_FORMAT_PD_SETTLE_US.
 * 3. Core 1 runs from RAM only (smol_core1_is_ram_only()).
 * After the format, it mounts the FS and loads the config. main() does not load the config while the format is pending.
 * A failed format or mount after the format is fatal (panic()).
 */
void smol_fs_run(void) {
	if (!_format_pending)
		return;

	uint64_t now_us = time_us_64();
	uint32_t contract_mv = smol_usb_pd_contract_mv();
	if (contract_mv != _settle_mv) {
		_settle_mv = contract_mv;
		_settle_since_us = now_us;
		return;
	}
	if ((contract_mv == 0) || !smol_core1_is_ram_only())
		return;
	if (now_us - _settle_since_us < FS_FORMAT_PD_SETTLE_US)
		return;

	_format_pending = false;
	info_println("FS format start, PD contract %u mV", (unsigned)contract_mv);
	uint64_t start_us = time_us_64();
	int err = lfs_format(&lfs, &_cfg);
	uint32_t format_ms = (uint32_t)((time_us_64() - start_us) / 1000);
	// a failed format or mount is fatal: the flash does not operate correctly. The config is not loaded, so smol_drv_enable() stays blocked.
	if (err)
		panic("FS format failed (%d) after %u ms", err, (unsigned)format_ms);
	err = lfs_mount(&lfs, &_cfg);
	if (err)
		panic("FS mount after format failed (%d)", err);
	_mounted = true;
	info_println("FS formatted in %u ms and mounted, %u blocks of %u bytes", (unsigned)format_ms, (unsigned)_cfg.block_count, (unsigned)_cfg.block_size);

	// main() did not load the config, because the FS was not mounted
	smol_config_load();
}


