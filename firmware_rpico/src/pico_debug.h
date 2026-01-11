#pragma once

#include <inttypes.h>
#include <stdarg.h>

#include "pico/time.h"

// define PICOBUG_ID in .c module before including this header
#ifndef PICOBUG_ID
#define PICOBUG_ID (DBG_ID_DEFAULT)
#endif

typedef enum {
	DBG_ID_DEFAULT = 0,
	DBG_ID_BSP,
	DBG_ID_DRV,
	DBG_ID_MAG,
	DBG_ID_USB,
	DBG_ID_ADC,
	DBG_ID_PWM,
	DBG_ID_LOOP,
	DBG_ID_COUNT,
} dbg_id_t;

typedef enum {
	DEBUG_LEVEL_DBG = 0,
	DEBUG_LEVEL_INF,
	DEBUG_LEVEL_WRN,
	DEBUG_LEVEL_ERR,
} dbg_level_t;


typedef struct {
	uint32_t seconds;
	uint32_t microseconds;
} dbg_time_t;

static inline dbg_time_t dbg_time(void) {
	// do a direct read of the microsecond timer, no API
    uint32_t lo = timer0_hw->timelr;
    uint32_t hi = timer0_hw->timehr;
    const uint64_t us_time = ((uint64_t) hi << 32u) | lo;
    // the convert to seconds and microseconds for nicer printf
	const uint32_t seconds = us_time / 1000000u;
	const uint32_t microseconds = us_time % 1000000u;
	return (dbg_time_t){.seconds = seconds, .microseconds = microseconds};
}

extern void picobug_set_level(int id, dbg_level_t level);

extern int picobug_printf (dbg_id_t id, dbg_level_t level, const char *__restrict fmt, ...) _ATTRIBUTE ((__format__ (__printf__, 3, 4)));

// call picobug_process_core1_queue() to process the debug print queue from core
extern void picobug_process_core1_queue(void);

// extern int picobug_vprintf (const char *, __VALIST) _ATTRIBUTE ((__format__ (__printf__, 1, 0)));

#define dbg_printf(fmt, ...) picobug_printf(0, DEBUG_LEVEL_DBG, fmt, ##__VA_ARGS__)

#define dbg_println(fmt, ...) do { \
	const dbg_time_t t = dbg_time(); \
	picobug_printf(PICOBUG_ID, DEBUG_LEVEL_DBG, "DBG %"PRIu32".%06"PRIu32" " fmt "\r\n", t.seconds, t.microseconds, ##__VA_ARGS__); \
} while (0)
#define info_println(fmt, ...) do { \
	const dbg_time_t t = dbg_time(); \
	picobug_printf(PICOBUG_ID, DEBUG_LEVEL_INF, "INF %"PRIu32".%06"PRIu32" " fmt "\r\n", t.seconds, t.microseconds, ##__VA_ARGS__); \
} while (0)
#define warn_println(fmt, ...) do { \
	const dbg_time_t t = dbg_time(); \
	picobug_printf(PICOBUG_ID, DEBUG_LEVEL_WRN, "WRN %"PRIu32".%06"PRIu32" " fmt "\r\n", t.seconds, t.microseconds, ##__VA_ARGS__); \
} while (0)
#define err_println(fmt, ...) do { \
	const dbg_time_t t = dbg_time(); \
	picobug_printf(PICOBUG_ID, DEBUG_LEVEL_ERR, "ERR %"PRIu32".%06"PRIu32" " fmt "\r\n", t.seconds, t.microseconds, ##__VA_ARGS__); \
} while (0)
