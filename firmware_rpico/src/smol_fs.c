#include "smol_servo.h"

#include "hardware/flash.h"
#include "pico/flash.h"


#include "pico_debug.h"

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


void smol_fs_init(void) {
	// configuration of the filesystem is provided by this struct
	static struct lfs_config cfg = {
	    // block device operations
	    .read  = _flash_read,
	    .prog  = _flash_prog,
	    .erase = _flash_erase,
	    .sync  = _flash_sync,

	    // block device configuration
	    .read_size = 16,
	    .prog_size = FLASH_PAGE_SIZE,
	    .block_size = FS_BLOCK_SIZE,
	    // NOTE: not a const expression, so can't set block_count during init
	    // .block_count = (uintptr_t)(&__flash_storage_end - &__flash_storage_begin) / FS_BLOCK_SIZE,
	    .cache_size = FLASH_PAGE_SIZE,
	    .lookahead_size = 16,
	    .block_cycles = 500,
	};

	cfg.block_count = (uintptr_t)(&__flash_storage_end - &__flash_storage_begin) / FS_BLOCK_SIZE;

    // mount the filesystem
    int err = lfs_mount(&lfs, &cfg);

    // reformat if we can't mount the filesystem
    // this should only happen on the first boot
    if (err) {
        lfs_format(&lfs, &cfg);
        lfs_mount(&lfs, &cfg);
    }

}


