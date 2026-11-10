#pragma once

#include <stdbool.h>
#include <stdint.h>

/**
 * smol-servo configuration, stored as a flat JSON object in /config.json on the littlefs flash filesystem.
 *
 * Example:
 * {
 *   "version": 1,
 *   "isense_gain": 20
 * }
 *
 * Rules:
 * - Only a flat object of "key": value pairs. A nested object or array as a value is ignored with a warning.
 * - A missing key, or a key with an invalid value, keeps its default value.
 * - An unknown key is ignored with a warning.
 * - On a JSON syntax error, all keys keep their default values.
 *
 * All functions run on core 0.
 */

#define SMOL_CONFIG_VERSION (1)

typedef struct {
	// TMC6200 current sense amplifier gain: 5, 10 or 20
	int32_t isense_gain;
} smol_config_t;

const smol_config_t* smol_config(void);

// read /config.json and apply the values. Call after smol_fs_init(). Not allowed while the driver is enabled. Returns 0 when the file was read without errors, or when there is no file.
int smol_config_load(void);
// true after the last smol_config_load() returned 0. smol_drv_enable() is blocked until then.
bool smol_config_is_loaded(void);
// write /config.json. Allowed only while core 1 runs from RAM only. Returns 0 on success.
int smol_config_save(void);
// set one key from a string value and apply it. Returns 0 on success.
int smol_config_set(const char* key, const char* value);
void smol_config_print(void);
