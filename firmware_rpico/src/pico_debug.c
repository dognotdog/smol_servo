
#include "pico_debug.h"

#include <stdio.h>
#include <stdarg.h>
#include <stdbool.h>
#include <stdatomic.h>

#define PICOBUG_MAX_NUM_IDS (256)
#define PICOBUG_LEVEL_BITS (2)
#define PICOBUG_LEVEL_MASK ((1u << PICOBUG_LEVEL_BITS) - 1u)
#define PICOBUG_STORAGE_BYTES (((PICOBUG_MAX_NUM_IDS*PICOBUG_LEVEL_BITS) + 7) / 8)
#define PICOBUG_STORAGE_U32 ((PICOBUG_STORAGE_BYTES + 3) / 4)

#define PICOBUG_BUFSIZE 128
#define PICOBUG_BUF_MAX_COUNT 32

static char _picobug_buf[PICOBUG_BUF_MAX_COUNT][PICOBUG_BUFSIZE];
static volatile uint32_t _buf_free_index;

static char* _queued_buffers[PICOBUG_BUF_MAX_COUNT];
static volatile uint32_t _buf_queued_index;
static volatile uint32_t _queue_processed_index;

// the first 32 IDs are active in early boot,
// the rest have to be activated specifically
static uint32_t _ids_active[PICOBUG_STORAGE_U32] = {
	UINT32_MAX, 
	UINT32_MAX,
};

_Static_assert((int)DEBUG_LEVEL_ERR <= (1 << PICOBUG_LEVEL_BITS));
_Static_assert((int)DBG_ID_COUNT <= PICOBUG_MAX_NUM_IDS);

static bool _is_id_active(dbg_id_t id, dbg_level_t level) {
	assert(id < DBG_ID_COUNT);
	const size_t leveled_id = id * PICOBUG_LEVEL_BITS;
	const size_t word = leveled_id / (8u * 4u);
	const size_t bit = leveled_id - word * (8u * 4u);

	// errors are never suppressed
	return (level == DEBUG_LEVEL_ERR) || ((_ids_active[word] >> bit) & PICOBUG_LEVEL_MASK);
}

void picobug_set_level(int id, dbg_level_t level) {
	assert(id < DBG_ID_COUNT);
	const size_t leveled_id = id * PICOBUG_LEVEL_BITS;
	const size_t word = leveled_id / (8u * 4u);
	const size_t bit = leveled_id - word * (8u * 4u);
	const uint32_t storage = _ids_active[word];
	const uint32_t mask = PICOBUG_LEVEL_MASK << bit;
	const uint32_t val = (uint32_t)level << bit;
	_ids_active[word] = (storage & ~mask) | val;
}

// static bool _cas(volatile uint32_t* val, uint32_t expected, uint32_t desired) {
// 	__DSB();

// 	_irq_disable();

// 	__ISB();
// }

static uint32_t _inc_index(volatile uint32_t* index) {
	uint32_t expected = *index;
	uint32_t desired = (expected + 1) % PICOBUG_BUF_MAX_COUNT;

	*index = desired;
	return expected;

	// TODO: fix atomics?
	// while (!atomic_compare_exchange_strong(index, &expected, desired)) {
	// 	expected = *index;
	// 	desired = (expected + 1) % PICOBUG_BUF_MAX_COUNT;
	// }
	// return expected;
}

char* picobug_request_buf(void) {
	uint32_t i = _inc_index(&_buf_free_index);
	return _picobug_buf[i];
}

void picobug_queue_buf(char* buf) {
	uint32_t buf_index = (buf - _picobug_buf[0]) / PICOBUG_BUFSIZE;

	uint32_t queue_index = _inc_index(&_buf_queued_index);
	_queued_buffers[queue_index] = _picobug_buf[buf_index];
}

int picobug_vprintf_core1(const char *fmt, __VALIST args) _ATTRIBUTE ((__format__ (__printf__, 1, 0)));

int picobug_vprintf_core1(const char *fmt, __VALIST args) {
	// from core 1, which we use for realtime stuff, we don't want to be stuck in USB/UART critical sections. So, we shove everything into a queue.
	char* buf = picobug_request_buf();

	int result = vsnprintf(buf, PICOBUG_BUFSIZE, fmt, args);

	picobug_queue_buf(buf);

	return result;
}

int picobug_printf (dbg_id_t id, dbg_level_t level, const char *__restrict fmt, ...) {
	if (!_is_id_active(id, level))
		return 0;

	va_list args;
	va_start(args, fmt);
	int result = 0;
	if (get_core_num() == 0) {
		result = vprintf(fmt, args);
	}
	else
		result = picobug_vprintf_core1(fmt, args);
	va_end(args);
	return result;
}


void picobug_process_core1_queue(void) {
	uint32_t q = _buf_queued_index;
	for (size_t i = 0; i < PICOBUG_BUF_MAX_COUNT; ++i) {
		uint32_t qi = (q + i) % PICOBUG_BUF_MAX_COUNT;
		// uint32_t queue_index = _inc_index(_queue_processed_index);
		char* null_ptr = NULL;
		char* buf_ptr = _queued_buffers[qi];
		_queued_buffers[qi] = NULL;
		// char* buf_ptr = atomic_exchange(&_queued_buffers[qi], null_ptr);
		if (buf_ptr == NULL) {
			continue;
		}
		printf("[C1] %.*s", PICOBUG_BUFSIZE, buf_ptr);
	}
}
