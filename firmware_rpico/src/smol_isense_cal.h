#pragma once

/**
 * Current sense offset calibration table (amplifier error E over the PWM compare value cc), stored as the binary file /isense_cal.bin on littlefs.
 * id_cal (smol_motor_id_cal_start) measures it, then core 0 fits the table, writes the file and loads it. main() loads the file at boot.
 * The model and the measurement are in notes/amplifier-recovery.md (sections 4 and 5).
 *
 * Table: knots every SMOL_ISENSE_CAL_STEP_CC counts from lo_cc. Per channel and sample (A0 A1 B0 B1 C0 C1) one value in mA per knot. Between the knots, linear interpolation.
 * - Sample 0 (counter 0, terminal high): relative to ZERO_CAL.
 * - Sample 1 (TOP, terminal low): relative to the static low level (the mean of the start and the end baseline of id_cal).
 * For each interval between 2 knots, the file also has err (the largest difference between the measured mean and the interpolation) and sd (the largest standard deviation of one loop), in mA.
 *
 * File layout (little endian): smol_isense_cal_header_t, int16_t E[3][2][n_knots], int16_t err[3][2][n_knots - 1], int16_t sd[3][2][n_knots - 1], uint32_t CRC (lfs_crc over all bytes before it, start value 0xffffffff).
 *
 * RAM: the image of the largest file (SMOL_ISENSE_CAL_KNOTS_MAX knots), 2908 bytes, and 1280 bytes for the fit. The loop uses only E (912 bytes for 76 knots).
 */

#include <stdbool.h>
#include <stdint.h>

#define SMOL_ISENSE_CAL_MAGIC (0x41435349u) // "ISCA"
#define SMOL_ISENSE_CAL_VERSION (1u)
#define SMOL_ISENSE_CAL_STEP_SHIFT (4u)
#define SMOL_ISENSE_CAL_STEP_CC (1u << SMOL_ISENSE_CAL_STEP_SHIFT)
// cc 0..1250 in steps of 16 needs up to 80 knots
#define SMOL_ISENSE_CAL_KNOTS_MAX (80u)
#define SMOL_ISENSE_CAL_CHANNELS (3u)

typedef struct {
	uint32_t magic;
	uint16_t version;
	uint16_t header_bytes;
	// first knot, knot spacing, number of knots, PWM compare value for duty 1
	uint16_t lo_cc;
	uint16_t step_cc;
	uint16_t n_knots;
	uint16_t cc_full;
	// conditions of the measurement: TMC6200 amplifier gain, VBUS, id_cal parameters
	uint16_t isense_gain;
	uint16_t vbus_mv;
	uint16_t loops_per_cc;
	uint16_t warm_s;
	uint16_t warm_cc;
	uint16_t reserved;
	// baselines of id_cal (all low sides on, no switching), relative to ZERO_CAL, mA
	int16_t base_start_ma[SMOL_ISENSE_CAL_CHANNELS][2];
	int16_t base_end_ma[SMOL_ISENSE_CAL_CHANNELS][2];
} smol_isense_cal_header_t;

typedef struct {
	smol_isense_cal_header_t h;
	int16_t e_ma[SMOL_ISENSE_CAL_CHANNELS][2][SMOL_ISENSE_CAL_KNOTS_MAX];
	int16_t err_ma[SMOL_ISENSE_CAL_CHANNELS][2][SMOL_ISENSE_CAL_KNOTS_MAX - 1];
	int16_t sd_ma[SMOL_ISENSE_CAL_CHANNELS][2][SMOL_ISENSE_CAL_KNOTS_MAX - 1];
} smol_isense_cal_t;

/**
 * Core 0 writes the image only while _valid is false. Core 1 may read E while smol_isense_cal_valid() is true.
 */
extern smol_isense_cal_t smol_isense_cal_image;
bool smol_isense_cal_valid(void);

/**
 * Table value in mA for channel p (0..2), sample j (0, 1) at compare value cc. Outside the knot range, the value of the nearest end knot.
 * Inline, so that core 1 can use it from RAM. Call it only when smol_isense_cal_valid() is true.
 */
static inline int32_t smol_isense_cal_e_ma(uint32_t p, uint32_t j, uint32_t cc) {
	const smol_isense_cal_t* c = &smol_isense_cal_image;
	const int16_t* e = c->e_ma[p][j];
	int32_t d = (int32_t)cc - (int32_t)c->h.lo_cc;
	if (d <= 0)
		return e[0];
	uint32_t i = (uint32_t)d >> SMOL_ISENSE_CAL_STEP_SHIFT;
	if (i >= (uint32_t)c->h.n_knots - 1u)
		return e[c->h.n_knots - 1u];
	int32_t f = d & (int32_t)(SMOL_ISENSE_CAL_STEP_CC - 1u);
	return e[i] + (((e[i + 1u] - e[i]) * f) >> SMOL_ISENSE_CAL_STEP_SHIFT);
}

/**
 * Input of the table fit: for point i (cc = lo_cc + i), the mean amplifier error and the standard deviation of one loop, in A, relative to the references above.
 */
typedef void (*smol_isense_cal_point_fn)(uint32_t i, float mean_a[SMOL_ISENSE_CAL_CHANNELS][2], float sd_a[SMOL_ISENSE_CAL_CHANNELS][2], void* ctx);

/**
 * Core 0: fit the table to the points lo_cc .. lo_cc + n_cc - 1, write the file and use the new table. h gives the conditions of the measurement (the function sets magic, version, sizes and knots).
 * Not allowed while a bridge state runs. Returns 0 on success. On a file error, the new table stays in use (not saved).
 */
int smol_isense_cal_build(const smol_isense_cal_header_t* h, uint32_t lo_cc, uint32_t n_cc, smol_isense_cal_point_fn fn, void* ctx);

/**
 * Core 0: read the file. Without a file, or with an invalid file, or with a different amplifier gain than the config, no table is in use. Returns 0 when a table is in use.
 */
int smol_isense_cal_load(void);

// Core 0: print the header and the table (cal_show). The table lines print in smol_isense_cal_core0_run().
void smol_isense_cal_show(void);
void smol_isense_cal_core0_run(void);
