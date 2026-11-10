/**
 * smol-servo TMC6200 gate driver setup
 *
 * Init and enable are two steps:
 *
 * 1. smol_drv_init() runs early in main(), before the USB PD sink starts. It sets DRV_EN low, so that the power stage stays off (all motor outputs floating), and writes the registers. The TMC6200 keeps its register values over an MCU reboot, so the datasheet (section 8) requires to disable the driver and write all used registers again. smol_drv_init() writes no log output, because blocking log output before the PD contract can cause a Hard Reset from the Source (see notes/usb-pd-reboot-loop.md). smol_drv_log_status() prints the result later.
 *
 * 2. smol_drv_enable() sets DRV_EN high and enables the driver. It needs a PD contract of at least DRV_VS_MIN_MV, because before the contract VBUS is 5V and the charge pump has an undervoltage (GSTAT.uv_cp). The GSTAT reset flag can only be cleared while DRV_EN is high.
 *
 * VS operational range (datasheet 11.1): 10V to 60V. 8V is the lower limit with reduced specification: short to GND protection does not work, and the limit depends on the MOSFET gate threshold and the load current. Thus a 9V contract is allowed, with a warning.
 *
 * smol_drv_run() runs in the main loop. It disables the driver when the contract voltage goes below DRV_VS_MIN_MV or when the FAULT pin is high.
 *
 * All TMC6200 SPI access is on core 0.
 */

#include "smol_servo.h"
#include "smol_config.h"
#include "smol_servo_parameters.h"
#include "pico_debug.h"

#include "tmc6200.h"

#include "hardware/gpio.h"

#include <inttypes.h>

#define TMC6200_VERSION 0x10

// lowest PD contract voltage for the enable: reduced specification below 10V (datasheet 11.1)
#define DRV_VS_MIN_MV (9000)
#define DRV_VS_FULL_SPEC_MV (10000)

#define GCONF_WRITE_MASK 0x000000FFu
#define DRV_CONF_WRITE_MASK 0x000F001Fu
// GSTAT flags that prevent the enable: drv_ot, uv_cp, s2g*, s2vs*. drv_otpw and shortdet_* are for information only.
#define GSTAT_ERROR_MASK 0x0000666Cu
#define GSTAT_ALL_MASK 0x0000777Fu

_Static_assert((ISENSE_AMPLIFICATION_CODE >= 0) && (ISENSE_AMPLIFICATION_CODE <= 3));
_Static_assert((TMC_BBMCLKS >= 0) && (TMC_BBMCLKS <= 15));
_Static_assert((TMC_DRVSTRENGTH >= 0) && (TMC_DRVSTRENGTH <= 3));
_Static_assert((TMC_OTSELECT >= 0) && (TMC_OTSELECT <= 3));

typedef enum {
	DRV_INIT_NOT_RUN = 0,
	DRV_INIT_OK,
	DRV_INIT_READ_FAILED,
	DRV_INIT_NOT_FOUND,
	DRV_INIT_READBACK_FAILED,
} drv_init_result_t;

static drv_init_result_t _init_result = DRV_INIT_NOT_RUN;
// register values before and after the init writes, for smol_drv_log_status()
static tmc_state_t _state_before;
static tmc_state_t _state_after;
static tmc_state_t _config;
static bool _drv_enabled = false;

static void _log_state(const char* label, const tmc_state_t* state) {
	dbg_println("TMC6200 %s GCONF 0x%08" PRIx32 " GSTAT 0x%08" PRIx32 " IOIN 0x%08" PRIx32 " SHORT_CONF 0x%08" PRIx32 " DRV_CONF 0x%08" PRIx32,
		label, state->GCONF.reg, state->GSTAT.reg, state->IOIN.reg, state->SHORT_CONF.reg, state->DRV_CONF.reg);
}

static bool _config_matches(const tmc_state_t* state) {
	bool gconf_ok = ((state->GCONF.reg ^ _config.GCONF.reg) & GCONF_WRITE_MASK) == 0;
	bool drv_conf_ok = ((state->DRV_CONF.reg ^ _config.DRV_CONF.reg) & DRV_CONF_WRITE_MASK) == 0;
	return gconf_ok && drv_conf_ok;
}

static void _write_config(void) {
	tmc6200_write_register(TMC_REG_GCONF, _config.GCONF.reg);
	tmc6200_write_register(TMC_REG_DRV_CONF, _config.DRV_CONF.reg);
}

/**
 * Set DRV_EN low and write the configuration with GCONF.disable = 1. No log output, see the file comment.
 */
void smol_drv_init(void) {
	_init_result = DRV_INIT_NOT_RUN;
	_drv_enabled = false;

	// DRV_EN low: power stage off
	gpio_init(DRVEN);
	gpio_put(DRVEN, false);
	gpio_set_dir(DRVEN, GPIO_OUT);

	// FAULT is high on a driver error. The pull-down keeps the pin low when the TMC6200 has no power.
	gpio_init(DRFAULT);
	gpio_set_dir(DRFAULT, GPIO_IN);
	gpio_pull_down(DRFAULT);

	if (tmc6200_read_state(&_state_before) != 0) {
		_init_result = DRV_INIT_READ_FAILED;
		return;
	}

	if (_state_before.IOIN.VERSION != TMC6200_VERSION) {
		_init_result = DRV_INIT_NOT_FOUND;
		return;
	}

	_config = _state_before;

	_config.GCONF.reg &= ~GCONF_WRITE_MASK;
	_config.GCONF.disable = 1;
	// PWM/EN pairs: H input is the PWM signal, L input is the enable
	_config.GCONF.singleline = 1;
	_config.GCONF.faultdirect = 0;
	_config.GCONF.amplification_x5 = (ISENSE_AMPLIFICATION_CODE >> 0) & 1;
	_config.GCONF.amplification_x1 = (ISENSE_AMPLIFICATION_CODE >> 1) & 1;
	_config.GCONF.amplifier_off = 0;
	_config.GCONF.test_mode = 0;

	_config.DRV_CONF.BBMCLKS = TMC_BBMCLKS;
	_config.DRV_CONF.OTSELECT = TMC_OTSELECT;
	_config.DRV_CONF.DRVSTRENGTH = TMC_DRVSTRENGTH;

	_write_config();

	if (tmc6200_read_state(&_state_after) != 0) {
		_init_result = DRV_INIT_READ_FAILED;
		return;
	}

	_init_result = _config_matches(&_state_after) ? DRV_INIT_OK : DRV_INIT_READBACK_FAILED;
}

/**
 * Print the result of smol_drv_init(). Call it after the PD startup.
 */
void smol_drv_log_status(void) {
	switch (_init_result) {
		case DRV_INIT_NOT_RUN:
			err_println("TMC6200 init did not run");
			return;
		case DRV_INIT_READ_FAILED:
			err_println("TMC6200 read failed");
			return;
		case DRV_INIT_NOT_FOUND:
			err_println("TMC6200 not found, IOIN version 0x%02x", (unsigned)_state_before.IOIN.VERSION);
			return;
		default:
			break;
	}

	_log_state("before init", &_state_before);
	_log_state("after init", &_state_after);

	if (_init_result == DRV_INIT_READBACK_FAILED) {
		err_println("TMC6200 readback GCONF 0x%08" PRIx32 " DRV_CONF 0x%08" PRIx32 ", expected GCONF 0x%08" PRIx32 " DRV_CONF 0x%08" PRIx32,
			_state_after.GCONF.reg, _state_after.DRV_CONF.reg, _config.GCONF.reg, _config.DRV_CONF.reg);
		return;
	}

	info_println("TMC6200 configured, default gain %.0f, BBMCLKS %d, DRVSTRENGTH %d", smol_drv_isense_gain(), TMC_BBMCLKS, TMC_DRVSTRENGTH);
}

static int _gain_to_code(int32_t gain) {
	switch (gain) {
		case 5: return 0;
		case 10: return 1;
		case 20: return 3;
		default: return -1;
	}
}

/**
 * Current sense amplifier gain of the configuration (5, 10 or 20).
 */
float smol_drv_isense_gain(void) {
	static const float amplifications[] = ISENSE_AMPLIFICATIONS;
	uint32_t code = (_config.GCONF.amplification_x1 << 1) | _config.GCONF.amplification_x5;
	if (_init_result == DRV_INIT_NOT_RUN)
		code = ISENSE_AMPLIFICATION_CODE;
	return amplifications[code];
}

/**
 * Set the current sense amplifier gain (5, 10 or 20). Allowed only while the driver is disabled. A new current sense zero calibration is necessary after a change, because the offset scales with the gain (datasheet 6.2.1).
 * Returns 0 on success.
 */
int smol_drv_set_isense_gain(int32_t gain) {
	int code = _gain_to_code(gain);
	if (code < 0) {
		err_println("TMC6200 gain %" PRId32 " is not valid (5, 10 or 20)", gain);
		return -1;
	}
	if (_drv_enabled) {
		err_println("TMC6200 gain change not allowed while the driver is enabled");
		return -1;
	}
	if (_init_result != DRV_INIT_OK) {
		err_println("TMC6200 gain: init not complete");
		return -1;
	}

	_config.GCONF.amplification_x5 = (code >> 0) & 1;
	_config.GCONF.amplification_x1 = (code >> 1) & 1;
	tmc6200_write_register(TMC_REG_GCONF, _config.GCONF.reg);

	tmc_state_t state;
	if ((tmc6200_read_state(&state) != 0) || !_config_matches(&state)) {
		err_println("TMC6200 gain: GCONF readback 0x%08" PRIx32 ", expected 0x%08" PRIx32, state.GCONF.reg, _config.GCONF.reg);
		return -1;
	}
	info_println("TMC6200 current sense gain %" PRId32 ", GCONF 0x%08" PRIx32, gain, state.GCONF.reg);
	smol_servo_loop_isense_gain_changed();
	return 0;
}

bool smol_drv_is_ready(void) {
	return _init_result == DRV_INIT_OK;
}

bool smol_drv_is_enabled(void) {
	return _drv_enabled;
}

/**
 * Set DRV_EN low and GCONF.disable = 1. DRV_EN low switches off the power stage at once.
 */
void smol_drv_disable(void) {
	gpio_put(DRVEN, false);
	_drv_enabled = false;
	if (_init_result == DRV_INIT_OK)
		tmc6200_write_register(TMC_REG_GCONF, _config.GCONF.reg);
}

/**
 * Enable the power stage. Call it while all bridge EN inputs are low, before PWM starts.
 * Returns 0 when the driver is enabled, else -1. On an error, DRV_EN stays low.
 */
int smol_drv_enable(void) {
	if (_init_result != DRV_INIT_OK) {
		err_println("TMC6200 enable: init not complete");
		return -1;
	}
	// the config sets the current sense gain. On the first boot, the config load waits for the deferred FS format.
	if (!smol_config_is_loaded()) {
		err_println("TMC6200 enable: config not loaded");
		return -1;
	}
	if (smol_servo_loop_isense_zero_is_busy()) {
		err_println("TMC6200 enable: current sense zero calibration is busy");
		return -1;
	}

	uint32_t vs_mv = smol_usb_pd_contract_mv();
	if (vs_mv < DRV_VS_MIN_MV) {
		err_println("TMC6200 enable: PD contract %" PRIu32 " mV, need %d mV", vs_mv, DRV_VS_MIN_MV);
		return -1;
	}
	if (vs_mv < DRV_VS_FULL_SPEC_MV)
		warn_println("TMC6200 enable: VS %" PRIu32 " mV is below %d mV, short to GND protection does not work", vs_mv, DRV_VS_FULL_SPEC_MV);

	// the registers can be at the reset values again after an undervoltage reset of the TMC6200
	tmc_state_t state;
	if (tmc6200_read_state(&state) != 0) {
		err_println("TMC6200 enable: read failed");
		return -1;
	}
	if (!_config_matches(&state)) {
		warn_println("TMC6200 enable: configuration lost (GCONF 0x%08" PRIx32 " DRV_CONF 0x%08" PRIx32 "), write again", state.GCONF.reg, state.DRV_CONF.reg);
		_write_config();
	}

	gpio_put(DRVEN, true);

	// clear the reset flag and the old error flags: this is possible only while DRV_EN is high
	tmc6200_write_register(TMC_REG_GSTAT, GSTAT_ALL_MASK);

	if (tmc6200_read_state(&state) != 0) {
		gpio_put(DRVEN, false);
		err_println("TMC6200 enable: read failed");
		return -1;
	}

	bool config_ok = _config_matches(&state);
	bool gstat_ok = (state.GSTAT.reg & GSTAT_ERROR_MASK) == 0;
	bool fault_ok = !gpio_get(DRFAULT);
	if (!config_ok || !gstat_ok || !fault_ok) {
		gpio_put(DRVEN, false);
		_log_state("enable failed", &state);
		err_println("TMC6200 enable: config %s, GSTAT errors 0x%08" PRIx32 ", FAULT pin %s",
			config_ok ? "ok" : "wrong", state.GSTAT.reg & GSTAT_ERROR_MASK, fault_ok ? "low" : "high");
		return -1;
	}
	if (state.GSTAT.reset)
		warn_println("TMC6200 enable: GSTAT reset flag did not clear");

	tmc_state_t enabled = _config;
	enabled.GCONF.disable = 0;
	tmc6200_write_register(TMC_REG_GCONF, enabled.GCONF.reg);

	if ((tmc6200_read_state(&state) != 0) || state.GCONF.disable) {
		smol_drv_disable();
		err_println("TMC6200 enable: GCONF.disable did not clear");
		return -1;
	}

	_drv_enabled = true;
	_log_state("enabled", &state);
	info_println("TMC6200 enabled, VS %" PRIu32 " mV", vs_mv);
	return 0;
}

/**
 * Main loop check on core 0: disable on a low PD contract voltage or on a FAULT.
 */
void smol_drv_run(void) {
	if (!_drv_enabled)
		return;

	uint32_t vs_mv = smol_usb_pd_contract_mv();
	if (vs_mv < DRV_VS_MIN_MV) {
		smol_drv_disable();
		warn_println("TMC6200 disabled: PD contract %" PRIu32 " mV", vs_mv);
		return;
	}

	if (gpio_get(DRFAULT)) {
		// read GSTAT first: DRV_EN low can clear the flags. The TMC6200 has already switched off the affected bridge.
		tmc_state_t state;
		int read_result = tmc6200_read_state(&state);
		smol_drv_disable();
		if (read_result == 0)
			_log_state("fault", &state);
		err_println("TMC6200 disabled: FAULT pin high");
	}
}
