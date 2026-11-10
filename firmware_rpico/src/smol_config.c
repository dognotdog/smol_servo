/**
 * smol-servo configuration file, see smol_config.h.
 *
 * Reading uses the lwjson stream parser (https://github.com/MaJerle/lwjson). The file is fed to the parser one character at a time, so only a small read buffer is necessary.
 *
 * Writing uses snprintf(), because the file is a flat object.
 *
 * Flash writes: the build sets PICO_FLASH_ASSUME_CORE1_SAFE=1, so flash_safe_execute() does not stop core 1. A write is thus safe only after core 1 has finished its init and runs from RAM only (smol_core1_is_ram_only()). Also, a flash erase disables the interrupts of core 0 for up to several 10 ms. During this time core 0 cannot answer USB PD messages.
 */

#include "smol_config.h"
#include "smol_servo.h"
#include "smol_servo_parameters.h"
#include "smol_fs.h"
#include "pico_debug.h"

#include "lwjson/lwjson.h"

#include <errno.h>
#include <stddef.h>
#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define CONFIG_PATH "/config.json"
#define CONFIG_TMP_PATH "/config.json.new"
#define CONFIG_READ_CHUNK (64)
#define CONFIG_MAX_SIZE (2048)

typedef bool (*value_check_fn)(int32_t value);

typedef struct {
	const char* key;
	size_t offset;
	value_check_fn check;
	const char* valid_text;
} config_int_key_t;

static bool _check_isense_gain(int32_t value) {
	return (value == 5) || (value == 10) || (value == 20);
}

static const config_int_key_t _int_keys[] = {
	{"isense_gain", offsetof(smol_config_t, isense_gain), _check_isense_gain, "5, 10 or 20"},
};

#define NUM_INT_KEYS (sizeof(_int_keys) / sizeof(_int_keys[0]))

static smol_config_t _defaults(void) {
	static const float amplifications[] = ISENSE_AMPLIFICATIONS;
	smol_config_t cfg = {
		.isense_gain = (int32_t)amplifications[ISENSE_AMPLIFICATION_CODE],
	};
	return cfg;
}

static smol_config_t _config;
static bool _config_initialized = false;
// true after smol_config_load() read the FS without errors (or found no file) and applied the values. smol_drv_enable() needs it.
static bool _config_loaded = false;

bool smol_config_is_loaded(void) {
	return _config_loaded;
}

const smol_config_t* smol_config(void) {
	if (!_config_initialized) {
		_config = _defaults();
		_config_initialized = true;
	}
	return &_config;
}

static const config_int_key_t* _find_int_key(const char* key) {
	for (size_t i = 0; i < NUM_INT_KEYS; ++i) {
		if (0 == strcmp(_int_keys[i].key, key))
			return &_int_keys[i];
	}
	return NULL;
}

static bool _parse_int(const char* text, int32_t* value) {
	char* end = NULL;
	errno = 0;
	long v = strtol(text, &end, 10);
	if ((end == text) || (*end != '\0') || (errno != 0) || (v < INT32_MIN) || (v > INT32_MAX))
		return false;
	*value = (int32_t)v;
	return true;
}

/**
 * Set a key in cfg from its text value. Returns 0 on success, else -1 (cfg unchanged).
 */
static int _set_value(smol_config_t* cfg, const char* key, const char* text) {
	if (0 == strcmp(key, "version")) {
		int32_t version = 0;
		if (!_parse_int(text, &version)) {
			warn_println("config: version '%s' is not an integer", text);
			return -1;
		}
		if (version > SMOL_CONFIG_VERSION)
			warn_println("config: file version %" PRId32 " is newer than %d", version, SMOL_CONFIG_VERSION);
		return 0;
	}

	const config_int_key_t* k = _find_int_key(key);
	if (!k) {
		warn_println("config: unknown key '%s' ignored", key);
		return -1;
	}

	int32_t value = 0;
	if (!_parse_int(text, &value)) {
		warn_println("config: %s value '%s' is not an integer", key, text);
		return -1;
	}
	if (k->check && !k->check(value)) {
		warn_println("config: %s value %" PRId32 " is not valid (%s)", key, value, k->valid_text);
		return -1;
	}
	*(int32_t*)((uint8_t*)cfg + k->offset) = value;
	return 0;
}

typedef struct {
	smol_config_t cfg;
	uint32_t key_errors;
	bool root_not_object;
} load_context_t;

static void _json_event(lwjson_stream_parser_t* jsp, lwjson_stream_type_t type) {
	load_context_t* ctx = lwjson_stream_get_user_data(jsp);

	// the root start event comes before the parser pushes the root to its stack. The root end event comes after the pop, so it also has stack_pos 0.
	if (jsp->stack_pos == 0) {
		if ((type == LWJSON_STREAM_TYPE_ARRAY) || (type == LWJSON_STREAM_TYPE_ARRAY_END))
			ctx->root_not_object = true;
		return;
	}

	// only values of the root object: stack is [OBJECT, KEY]. Deeper events belong to a nested value.
	if ((jsp->stack_pos != 2) || (jsp->stack[0].type != LWJSON_STREAM_TYPE_OBJECT) || (jsp->stack[1].type != LWJSON_STREAM_TYPE_KEY))
		return;

	const char* key = jsp->stack[1].meta.name;
	switch (type) {
		case LWJSON_STREAM_TYPE_NUMBER:
			if (_set_value(&ctx->cfg, key, jsp->data.prim.buff) != 0)
				ctx->key_errors++;
			break;
		case LWJSON_STREAM_TYPE_OBJECT:
		case LWJSON_STREAM_TYPE_ARRAY:
			warn_println("config: %s has a nested value, ignored", key);
			ctx->key_errors++;
			break;
		case LWJSON_STREAM_TYPE_STRING:
		case LWJSON_STREAM_TYPE_TRUE:
		case LWJSON_STREAM_TYPE_FALSE:
		case LWJSON_STREAM_TYPE_NULL:
			warn_println("config: %s value type is not supported, ignored", key);
			ctx->key_errors++;
			break;
		default:
			break;
	}
}

static int _apply(const smol_config_t* cfg) {
	if (smol_drv_set_isense_gain(cfg->isense_gain) != 0) {
		err_println("config: isense_gain %" PRId32 " not applied", cfg->isense_gain);
		return -1;
	}
	return 0;
}

int smol_config_load(void) {
	// the gain change is not allowed while the driver is enabled. The loaded state does not change.
	if (smol_drv_is_enabled()) {
		err_println("config: load not allowed while the driver is enabled");
		return -1;
	}
	_config_loaded = false;

	smol_config_t defaults = _defaults();
	_config = defaults;
	_config_initialized = true;

	if (!smol_fs_is_mounted()) {
		warn_println("config: no filesystem, defaults in use");
		_apply(&_config);
		return -1;
	}

	lfs_t* lfs = smol_lfs();
	lfs_file_t file;
	int err = lfs_file_open(lfs, &file, CONFIG_PATH, LFS_O_RDONLY);
	if (err == LFS_ERR_NOENT) {
		info_println("config: no %s, defaults in use", CONFIG_PATH);
		if (_apply(&_config) != 0)
			return -1;
		_config_loaded = true;
		return 0;
	}
	if (err < 0) {
		err_println("config: open %s failed (%d), defaults in use", CONFIG_PATH, err);
		_apply(&_config);
		return -1;
	}

	static lwjson_stream_parser_t parser;
	load_context_t ctx = {.cfg = defaults};
	lwjson_stream_init(&parser, _json_event);
	lwjson_stream_set_user_data(&parser, &ctx);

	char buf[CONFIG_READ_CHUNK];
	size_t total = 0;
	lwjsonr_t result = lwjsonSTREAMWAITFIRSTCHAR;
	bool done = false;
	bool syntax_error = false;
	while (!done && !syntax_error) {
		lfs_ssize_t n = lfs_file_read(lfs, &file, buf, sizeof(buf));
		if (n < 0) {
			err_println("config: read failed (%d)", (int)n);
			syntax_error = true;
			break;
		}
		if (n == 0)
			break;
		total += (size_t)n;
		if (total > CONFIG_MAX_SIZE) {
			err_println("config: file is larger than %d bytes", CONFIG_MAX_SIZE);
			syntax_error = true;
			break;
		}
		for (lfs_ssize_t i = 0; i < n; ++i) {
			result = lwjson_stream_parse(&parser, buf[i]);
			if (result == lwjsonSTREAMDONE) {
				done = true;
				break;
			}
			if ((result != lwjsonSTREAMINPROG) && (result != lwjsonSTREAMWAITFIRSTCHAR)) {
				err_println("config: JSON error %d at byte %u", (int)result, (unsigned)(total - (size_t)n + (size_t)i));
				syntax_error = true;
				break;
			}
		}
	}
	lfs_file_close(lfs, &file);

	if (!done && !syntax_error) {
		err_println("config: %s is incomplete", CONFIG_PATH);
		syntax_error = true;
	}
	if (ctx.root_not_object) {
		err_println("config: the root of %s is not an object", CONFIG_PATH);
		syntax_error = true;
	}

	if (syntax_error) {
		err_println("config: defaults in use");
		_apply(&_config);
		return -1;
	}

	_config = ctx.cfg;
	info_println("config: %s read, %u bytes, %" PRIu32 " key errors", CONFIG_PATH, (unsigned)total, ctx.key_errors);
	if (_apply(&_config) != 0)
		return -1;
	if (ctx.key_errors != 0)
		return -1;
	_config_loaded = true;
	return 0;
}

static int _format(const smol_config_t* cfg, char* buf, size_t size) {
	int n = snprintf(buf, size, "{\n  \"version\": %d", SMOL_CONFIG_VERSION);
	for (size_t i = 0; (i < NUM_INT_KEYS) && (n > 0) && ((size_t)n < size); ++i) {
		int32_t value = *(const int32_t*)((const uint8_t*)cfg + _int_keys[i].offset);
		n += snprintf(buf + n, size - (size_t)n, ",\n  \"%s\": %" PRId32, _int_keys[i].key, value);
	}
	if ((n > 0) && ((size_t)n < size))
		n += snprintf(buf + n, size - (size_t)n, "\n}\n");
	if ((n < 0) || ((size_t)n >= size))
		return -1;
	return n;
}

int smol_config_save(void) {
	if (!smol_fs_is_mounted()) {
		err_println("config: no filesystem");
		return -1;
	}
	if (!smol_core1_is_ram_only()) {
		err_println("config: core 1 init is not complete, flash write not allowed");
		return -1;
	}

	char text[256];
	int len = _format(smol_config(), text, sizeof(text));
	if (len < 0) {
		err_println("config: text buffer too small");
		return -1;
	}

	// write a new file, then rename it: littlefs replaces the old file in one step
	lfs_t* lfs = smol_lfs();
	lfs_file_t file;
	int err = lfs_file_open(lfs, &file, CONFIG_TMP_PATH, LFS_O_WRONLY | LFS_O_CREAT | LFS_O_TRUNC);
	if (err < 0) {
		err_println("config: open %s failed (%d)", CONFIG_TMP_PATH, err);
		return -1;
	}
	lfs_ssize_t written = lfs_file_write(lfs, &file, text, (lfs_size_t)len);
	err = lfs_file_close(lfs, &file);
	if ((written != len) || (err < 0)) {
		err_println("config: write failed (%d, %d)", (int)written, err);
		lfs_remove(lfs, CONFIG_TMP_PATH);
		return -1;
	}
	err = lfs_rename(lfs, CONFIG_TMP_PATH, CONFIG_PATH);
	if (err < 0) {
		err_println("config: rename failed (%d)", err);
		return -1;
	}
	info_println("config: %s written, %d bytes", CONFIG_PATH, len);
	return 0;
}

int smol_config_set(const char* key, const char* value) {
	smol_config_t cfg = *smol_config();
	if (_set_value(&cfg, key, value) != 0)
		return -1;
	if (0 == strcmp(key, "isense_gain")) {
		if (smol_drv_set_isense_gain(cfg.isense_gain) != 0)
			return -1;
	}
	_config = cfg;
	return 0;
}

void smol_config_print(void) {
	char text[256];
	if (_format(smol_config(), text, sizeof(text)) < 0) {
		err_println("config: text buffer too small");
		return;
	}
	info_println("config in use:\n%s", text);
}
