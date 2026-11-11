#include "smol_isense_cal.h"

#include "smol_servo.h"
#include "smol_servo_loop.h"
#include "smol_config.h"
#include "smol_fs.h"
#include "pico_debug.h"

#include "hardware/sync.h"

#include <inttypes.h>
#include <math.h>
#include <stddef.h>
#include <string.h>

#define ISENSE_CAL_PATH "/isense_cal.bin"
#define ISENSE_CAL_TMP_PATH "/isense_cal.tmp"
// table lines that core 0 prints in one call of smol_isense_cal_core0_run()
#define ISENSE_CAL_PRINT_LINES (8u)

smol_isense_cal_t smol_isense_cal_image;
static volatile bool _valid = false;

static struct {
	bool printing;
	uint32_t i;
} _show;

bool smol_isense_cal_valid(void) {
	return _valid;
}

static bool _bridge_runs(void) {
	smol_loop_state_t s = smol_servo_loop_state();
	return (s == SMOL_LOOP_MOTOR_ID) || (s == SMOL_LOOP_CONTROL) || (s == SMOL_LOOP_PWM_TEST);
}

static int16_t _to_i16_ma(float a) {
	float ma = 1e3f * a;
	ma = (ma > 0.0f) ? (ma + 0.5f) : (ma - 0.5f);
	if (ma > 32767.0f)
		return 32767;
	if (ma < -32768.0f)
		return -32768;
	return (int16_t)ma;
}

/**
 * Sizes of the 3 arrays in the file. The image has the arrays at their maximum size, the file has them at n_knots. Thus the file is written and read in 4 parts (header, E, err, sd) with one row of each channel and sample at a time.
 */
typedef int (*_row_io_fn)(void* buf, size_t bytes, void* ctx);

static int _rows(uint32_t n_knots, _row_io_fn io, void* ctx) {
	smol_isense_cal_t* c = &smol_isense_cal_image;
	if (io(&c->h, sizeof(c->h), ctx) != 0)
		return -1;
	for (size_t p = 0; p < SMOL_ISENSE_CAL_CHANNELS; ++p)
		for (size_t j = 0; j < 2; ++j)
			if (io(c->e_ma[p][j], n_knots * sizeof(int16_t), ctx) != 0)
				return -1;
	for (size_t p = 0; p < SMOL_ISENSE_CAL_CHANNELS; ++p)
		for (size_t j = 0; j < 2; ++j)
			if (io(c->err_ma[p][j], (n_knots - 1u) * sizeof(int16_t), ctx) != 0)
				return -1;
	for (size_t p = 0; p < SMOL_ISENSE_CAL_CHANNELS; ++p)
		for (size_t j = 0; j < 2; ++j)
			if (io(c->sd_ma[p][j], (n_knots - 1u) * sizeof(int16_t), ctx) != 0)
				return -1;
	return 0;
}

typedef struct {
	lfs_t* lfs;
	lfs_file_t* file;
	uint32_t crc;
	size_t bytes;
} _io_ctx_t;

static int _write_part(void* buf, size_t bytes, void* vctx) {
	_io_ctx_t* ctx = vctx;
	lfs_ssize_t n = lfs_file_write(ctx->lfs, ctx->file, buf, (lfs_size_t)bytes);
	if (n != (lfs_ssize_t)bytes)
		return -1;
	ctx->crc = lfs_crc(ctx->crc, buf, bytes);
	ctx->bytes += bytes;
	return 0;
}

static int _read_part(void* buf, size_t bytes, void* vctx) {
	_io_ctx_t* ctx = vctx;
	lfs_ssize_t n = lfs_file_read(ctx->lfs, ctx->file, buf, (lfs_size_t)bytes);
	if (n != (lfs_ssize_t)bytes)
		return -1;
	ctx->crc = lfs_crc(ctx->crc, buf, bytes);
	ctx->bytes += bytes;
	return 0;
}

static int _save(void) {
	if (!smol_fs_is_mounted()) {
		err_println("isense cal: no filesystem, table not saved");
		return -1;
	}
	if (!smol_core1_is_ram_only()) {
		err_println("isense cal: core 1 init is not complete, flash write not allowed");
		return -1;
	}
	// write a new file, then rename it: littlefs replaces the old file in one step
	lfs_t* lfs = smol_lfs();
	lfs_file_t file;
	int err = lfs_file_open(lfs, &file, ISENSE_CAL_TMP_PATH, LFS_O_WRONLY | LFS_O_CREAT | LFS_O_TRUNC);
	if (err < 0) {
		err_println("isense cal: open %s failed (%d)", ISENSE_CAL_TMP_PATH, err);
		return -1;
	}
	_io_ctx_t ctx = {.lfs = lfs, .file = &file, .crc = 0xffffffffu, .bytes = 0};
	int r = _rows(smol_isense_cal_image.h.n_knots, _write_part, &ctx);
	uint32_t crc = ctx.crc;
	if ((r == 0) && (lfs_file_write(lfs, &file, &crc, sizeof(crc)) != (lfs_ssize_t)sizeof(crc)))
		r = -1;
	err = lfs_file_close(lfs, &file);
	if ((r != 0) || (err < 0)) {
		err_println("isense cal: write failed (%d)", err);
		lfs_remove(lfs, ISENSE_CAL_TMP_PATH);
		return -1;
	}
	err = lfs_rename(lfs, ISENSE_CAL_TMP_PATH, ISENSE_CAL_PATH);
	if (err < 0) {
		err_println("isense cal: rename failed (%d)", err);
		return -1;
	}
	info_println("isense cal: %s written, %u bytes", ISENSE_CAL_PATH, (unsigned)(ctx.bytes + sizeof(crc)));
	return 0;
}

int smol_isense_cal_load(void) {
	if (_bridge_runs()) {
		err_println("isense cal: load not allowed while the bridge switches");
		return -1;
	}
	_valid = false;
	__dmb();
	if (!smol_fs_is_mounted()) {
		warn_println("isense cal: no filesystem, no table in use");
		return -1;
	}
	lfs_t* lfs = smol_lfs();
	lfs_file_t file;
	int err = lfs_file_open(lfs, &file, ISENSE_CAL_PATH, LFS_O_RDONLY);
	if (err == LFS_ERR_NOENT) {
		info_println("isense cal: no %s, no table in use (run id_cal)", ISENSE_CAL_PATH);
		return -1;
	}
	if (err < 0) {
		err_println("isense cal: open %s failed (%d)", ISENSE_CAL_PATH, err);
		return -1;
	}
	smol_isense_cal_t* c = &smol_isense_cal_image;
	_io_ctx_t ctx = {.lfs = lfs, .file = &file, .crc = 0xffffffffu, .bytes = 0};
	int r = _read_part(&c->h, sizeof(c->h), &ctx);
	const smol_isense_cal_header_t* h = &c->h;
	if ((r == 0) && ((h->magic != SMOL_ISENSE_CAL_MAGIC) || (h->version != SMOL_ISENSE_CAL_VERSION) || (h->header_bytes != sizeof(*h))
		|| (h->step_cc != SMOL_ISENSE_CAL_STEP_CC) || (h->n_knots < 2u) || (h->n_knots > SMOL_ISENSE_CAL_KNOTS_MAX))) {
		err_println("isense cal: %s has a wrong header (magic 0x%08" PRIx32 ", version %u, %u knots, step %u)", ISENSE_CAL_PATH, h->magic, h->version, h->n_knots, h->step_cc);
		r = -2;
	}
	if (r == 0) {
		// _rows() reads the header again: go back to the start
		lfs_file_rewind(lfs, &file);
		ctx.crc = 0xffffffffu;
		ctx.bytes = 0;
		r = _rows(h->n_knots, _read_part, &ctx);
		uint32_t crc = 0;
		if ((r == 0) && (lfs_file_read(lfs, &file, &crc, sizeof(crc)) != (lfs_ssize_t)sizeof(crc)))
			r = -1;
		if ((r == 0) && (crc != ctx.crc)) {
			err_println("isense cal: %s has a wrong CRC", ISENSE_CAL_PATH);
			r = -2;
		}
		if (r == -1)
			err_println("isense cal: %s is too short", ISENSE_CAL_PATH);
	}
	lfs_file_close(lfs, &file);
	if (r != 0)
		return -1;
	int32_t gain = smol_config()->isense_gain;
	if ((int32_t)h->isense_gain != gain) {
		err_println("isense cal: the table is for gain %u, the config has gain %" PRId32 ". No table in use (run id_cal)", h->isense_gain, gain);
		return -1;
	}
	__dmb(); // the table must be in memory before core 1 sees _valid
	_valid = true;
	info_println("isense cal: %s read, cc %u..%u, %u knots, gain %u, VBUS %.2f V", ISENSE_CAL_PATH,
		h->lo_cc, h->lo_cc + (h->n_knots - 1u) * h->step_cc, h->n_knots, h->isense_gain, 1e-3f * (float)h->vbus_mv);
	return 0;
}

/**
 * Least squares fit of a piecewise linear function (hat functions at the knots) to the points. The normal equations are tridiagonal and the same for all 6 channels. The Thomas algorithm solves them.
 */
int smol_isense_cal_build(const smol_isense_cal_header_t* hin, uint32_t lo_cc, uint32_t n_cc, smol_isense_cal_point_fn fn, void* ctx) {
	if (_bridge_runs()) {
		err_println("isense cal: build not allowed while the bridge switches");
		return -1;
	}
	if (n_cc < SMOL_ISENSE_CAL_STEP_CC + 1u) {
		err_println("isense cal: %" PRIu32 " points are not enough for a table (min. %u)", n_cc, SMOL_ISENSE_CAL_STEP_CC + 1u);
		return -1;
	}
	uint32_t nk = (n_cc - 1u + SMOL_ISENSE_CAL_STEP_CC - 1u) / SMOL_ISENSE_CAL_STEP_CC + 1u;
	if (nk > SMOL_ISENSE_CAL_KNOTS_MAX) {
		err_println("isense cal: %" PRIu32 " knots are too many (max. %u)", nk, SMOL_ISENSE_CAL_KNOTS_MAX);
		return -1;
	}
	_valid = false;
	__dmb();

	/**
	 * The matrix is the same for all channels: one forward elimination of the matrix (den, cp), then for each channel the right side and the back substitution. Thus only one right side is in RAM.
	 */
	static float den[SMOL_ISENSE_CAL_KNOTS_MAX];
	static float off[SMOL_ISENSE_CAL_KNOTS_MAX];
	static float cp[SMOL_ISENSE_CAL_KNOTS_MAX];
	static float rhs[SMOL_ISENSE_CAL_KNOTS_MAX];
	memset(den, 0, sizeof(den));
	memset(off, 0, sizeof(off));
	float mean[SMOL_ISENSE_CAL_CHANNELS][2];
	float sd[SMOL_ISENSE_CAL_CHANNELS][2];
	const float inv_step = 1.0f / (float)SMOL_ISENSE_CAL_STEP_CC;
	for (uint32_t n = 0; n < n_cc; ++n) {
		uint32_t i = n / SMOL_ISENSE_CAL_STEP_CC;
		if (i > nk - 2u)
			i = nk - 2u;
		float f = (float)(n - i * SMOL_ISENSE_CAL_STEP_CC) * inv_step;
		float g = 1.0f - f;
		den[i] += g * g;
		den[i + 1u] += f * f;
		off[i] += g * f;
	}
	// Thomas algorithm, forward elimination of the matrix (symmetric: sub-diagonal = off)
	cp[0] = off[0] / den[0];
	for (uint32_t k = 1; k < nk; ++k) {
		den[k] -= off[k - 1u] * cp[k - 1u];
		cp[k] = (k < nk - 1u) ? off[k] / den[k] : 0.0f;
	}

	smol_isense_cal_t* c = &smol_isense_cal_image;
	memset(c, 0, sizeof(*c));
	c->h = *hin;
	c->h.magic = SMOL_ISENSE_CAL_MAGIC;
	c->h.version = SMOL_ISENSE_CAL_VERSION;
	c->h.header_bytes = sizeof(c->h);
	c->h.lo_cc = (uint16_t)lo_cc;
	c->h.step_cc = SMOL_ISENSE_CAL_STEP_CC;
	c->h.n_knots = (uint16_t)nk;
	for (size_t q = 0; q < SMOL_ISENSE_CAL_CHANNELS * 2u; ++q) {
		memset(rhs, 0, sizeof(rhs));
		for (uint32_t n = 0; n < n_cc; ++n) {
			uint32_t i = n / SMOL_ISENSE_CAL_STEP_CC;
			if (i > nk - 2u)
				i = nk - 2u;
			float f = (float)(n - i * SMOL_ISENSE_CAL_STEP_CC) * inv_step;
			fn(n, mean, sd, ctx);
			float y = mean[q / 2u][q % 2u];
			rhs[i] += (1.0f - f) * y;
			rhs[i + 1u] += f * y;
		}
		rhs[0] /= den[0];
		for (uint32_t k = 1; k < nk; ++k)
			rhs[k] = (rhs[k] - off[k - 1u] * rhs[k - 1u]) / den[k];
		for (uint32_t k = nk - 1u; k-- > 0;)
			rhs[k] -= cp[k] * rhs[k + 1u];
		for (uint32_t k = 0; k < nk; ++k)
			c->e_ma[q / 2u][q % 2u][k] = _to_i16_ma(rhs[k]);
	}

	// err and sd of each interval, with the integer interpolation of the loop (the rounding is in err)
	for (uint32_t n = 0; n < n_cc; ++n) {
		uint32_t i = n / SMOL_ISENSE_CAL_STEP_CC;
		if (i > nk - 2u)
			i = nk - 2u;
		fn(n, mean, sd, ctx);
		for (size_t p = 0; p < SMOL_ISENSE_CAL_CHANNELS; ++p) {
			for (size_t j = 0; j < 2; ++j) {
				float e = 1e3f * mean[p][j] - (float)smol_isense_cal_e_ma((uint32_t)p, (uint32_t)j, lo_cc + n);
				int16_t ae = _to_i16_ma(1e-3f * fabsf(e));
				int16_t as = _to_i16_ma(sd[p][j]);
				if (ae > c->err_ma[p][j][i])
					c->err_ma[p][j][i] = ae;
				if (as > c->sd_ma[p][j][i])
					c->sd_ma[p][j][i] = as;
			}
		}
	}

	info_println("isense cal: table cc %" PRIu32 "..%" PRIu32 ", %" PRIu32 " knots, gain %u, VBUS %.2f V", lo_cc, lo_cc + (nk - 1u) * SMOL_ISENSE_CAL_STEP_CC, nk,
		c->h.isense_gain, 1e-3f * (float)c->h.vbus_mv);
	int r = _save();
	__dmb(); // the table must be in memory before core 1 sees _valid
	_valid = true;
	if (r != 0)
		warn_println("isense cal: the new table is in use, but not saved");
	return r;
}

void smol_isense_cal_show(void) {
	if (!_valid) {
		info_println("isense cal: no table in use");
		return;
	}
	const smol_isense_cal_header_t* h = &smol_isense_cal_image.h;
	info_println("isense cal: cc %u..%u step %u, %u knots, cc full %u, gain %u, VBUS %.2f V, id_cal %u loops per cc, warmup %u s at cc %u",
		h->lo_cc, h->lo_cc + (h->n_knots - 1u) * h->step_cc, h->step_cc, h->n_knots, h->cc_full, h->isense_gain, 1e-3f * (float)h->vbus_mv,
		h->loops_per_cc, h->warm_s, h->warm_cc);
	info_println("isense cal: base start %d %d %d %d %d %d mA, end %d %d %d %d %d %d mA",
		h->base_start_ma[0][0], h->base_start_ma[0][1], h->base_start_ma[1][0], h->base_start_ma[1][1], h->base_start_ma[2][0], h->base_start_ma[2][1],
		h->base_end_ma[0][0], h->base_end_ma[0][1], h->base_end_ma[1][0], h->base_end_ma[1][1], h->base_end_ma[2][0], h->base_end_ma[2][1]);
	info_println("isense cal: lines: knot cc, then E A0 A1 B0 B1 C0 C1, then err and sd of the interval to the next knot, mA");
	_show.i = 0;
	_show.printing = true;
}

void smol_isense_cal_core0_run(void) {
	if (!_show.printing)
		return;
	const smol_isense_cal_t* c = &smol_isense_cal_image;
	const uint32_t nk = c->h.n_knots;
	for (uint32_t l = 0; (l < ISENSE_CAL_PRINT_LINES) && (_show.i < nk); ++l) {
		uint32_t k = _show.i;
		uint32_t cc = c->h.lo_cc + k * c->h.step_cc;
		if (k + 1u < nk) {
			info_println("isense cal: cc %4" PRIu32 " E %6d %6d %6d %6d %6d %6d err %4d %4d %4d %4d %4d %4d sd %4d %4d %4d %4d %4d %4d", cc,
				c->e_ma[0][0][k], c->e_ma[0][1][k], c->e_ma[1][0][k], c->e_ma[1][1][k], c->e_ma[2][0][k], c->e_ma[2][1][k],
				c->err_ma[0][0][k], c->err_ma[0][1][k], c->err_ma[1][0][k], c->err_ma[1][1][k], c->err_ma[2][0][k], c->err_ma[2][1][k],
				c->sd_ma[0][0][k], c->sd_ma[0][1][k], c->sd_ma[1][0][k], c->sd_ma[1][1][k], c->sd_ma[2][0][k], c->sd_ma[2][1][k]);
		}
		else {
			info_println("isense cal: cc %4" PRIu32 " E %6d %6d %6d %6d %6d %6d", cc,
				c->e_ma[0][0][k], c->e_ma[0][1][k], c->e_ma[1][0][k], c->e_ma[1][1][k], c->e_ma[2][0][k], c->e_ma[2][1][k]);
		}
		_show.i = k + 1u;
	}
	if (_show.i >= nk)
		_show.printing = false;
}
