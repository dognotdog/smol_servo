#pragma once

#include <inttypes.h>
#include <stdarg.h>
#include <stdio.h>

#include "pico/time.h"

typedef struct {
	uint32_t seconds;
	uint32_t microseconds;
} dbg_time_t;

static inline dbg_time_t dbg_time(void) {
	const uint64_t us_time = time_us_64();
	const uint32_t seconds = us_time / 1000000u;
	const uint32_t microseconds = us_time % 1000000u;
	return (dbg_time_t){.seconds = seconds, .microseconds = microseconds};
}

#define dbg_printf(fmt, ...) printf(fmt, ##__VA_ARGS__)
#define dbg_println(fmt, ...) do { \
	const dbg_time_t t = dbg_time(); \
	printf("DBG %"PRIu32".%06"PRIu32" " fmt "\r\n", t.seconds, t.microseconds, ##__VA_ARGS__); \
} while (0)
#define info_println(fmt, ...) do { \
	const dbg_time_t t = dbg_time(); \
	printf("INF %"PRIu32".%06"PRIu32" " fmt "\r\n", t.seconds, t.microseconds, ##__VA_ARGS__); \
} while (0)
#define warn_println(fmt, ...) do { \
	const dbg_time_t t = dbg_time(); \
	printf("WRN %"PRIu32".%06"PRIu32" " fmt "\r\n", t.seconds, t.microseconds, ##__VA_ARGS__); \
} while (0)
#define err_println(fmt, ...) do { \
	const dbg_time_t t = dbg_time(); \
	printf("ERR %"PRIu32".%06"PRIu32" " fmt "\r\n", t.seconds, t.microseconds, ##__VA_ARGS__); \
} while (0)
