#include "smol_console.h"
#include "smol_servo.h"

#include "pico_debug.h"
#include "smol_config.h"
#include "smol_motor_id.h"
#include "smol_servo_usb_pd.h"
#include "tmc6200.h"

#include <stdio.h>
#include <inttypes.h>
#include <stdlib.h>
#include <stddef.h>
#include <string.h>
#include <stdbool.h>

#define SMOL_CONSOLE_INPUT_BUF_BYTES (128)

static char _input_buf[SMOL_CONSOLE_INPUT_BUF_BYTES];
static size_t _input_cursor = 0;

static void _process_command(char* cmd, size_t cmd_len, char* args, size_t args_len) {
	dbg_println("process_command: %.*s", cmd_len, cmd);
	// the servo loop starts at boot (main.c). "state zero_cal" does a new zero calibration.
	if (0 == strcmp("stop", cmd)) {
		smol_motor_id_auto_cancel();
		smol_servo_loop_request_state(SMOL_LOOP_OFF);
	}
	else if (0 == strcmp("state", cmd)) {
		// state: print, state <name>: request a loop state
		if (args[0] == 0) {
			smol_servo_loop_print_state();
		}
		else {
			smol_loop_state_t state = smol_servo_loop_state_from_name(args);
			if (state == SMOL_LOOP_NUM_STATES)
				warn_println("unknown loop state: %s (off, zero_cal, motor_id (use id_pulse), control, pwm_test)", args);
			else
				smol_servo_loop_request_state(state);
		}
	}
	else if ((0 == strcmp("id_pulse", cmd)) || (0 == strcmp("id_dc", cmd))) {
		// id_pulse [pair 0..2] [w_ns] [cycles], defaults 0 500 64
		// id_dc [pair 0..2] [duty_permille] [loops], defaults 0 20 2000
		// pair 0 = A->B, 1 = B->C, 2 = C->A
		bool pulse = (0 == strcmp("id_pulse", cmd));
		unsigned long v[3] = {0, pulse ? 500 : 20, pulse ? 64 : 2000};
		char* p = args;
		for (size_t i = 0; i < 3 && *p; ++i) {
			char* end = NULL;
			v[i] = strtoul(p, &end, 10);
			if (end == p) {
				warn_println("usage: id_pulse [pair 0..2] [w_ns] [cycles], id_dc [pair 0..2] [duty_permille] [loops]");
				return;
			}
			p = end;
			while (*p == ' ')
				++p;
		}
		if (smol_servo_loop_state() == SMOL_LOOP_MOTOR_ID) {
			warn_println("%s: a motor id test runs", cmd);
			return;
		}
		int r = pulse ? smol_motor_id_prepare_pulse((uint32_t)v[0], (uint32_t)v[1], (uint32_t)v[2])
			: smol_motor_id_prepare_dc((uint32_t)v[0], (float)v[1] * 1e-3f, (uint32_t)v[2]);
		if (r == 0)
			smol_servo_loop_request_state(SMOL_LOOP_MOTOR_ID);
	}
	else if (0 == strcmp("id_auto", cmd)) {
		// id_auto [cycles], default 2048: pulse test of all pairs, fit and 2-phase / 3-phase classification
		unsigned long cycles = 2048;
		if (args[0]) {
			char* end = NULL;
			cycles = strtoul(args, &end, 10);
			if (end == args) {
				warn_println("usage: id_auto [cycles]");
				return;
			}
		}
		smol_motor_id_auto_start((uint32_t)cycles);
	}
	else if (0 == strcmp("id_rec", cmd)) {
		// id_rec [lo_cc] [hi_cc] [step_cc] [loops], defaults 0 1250 1 1000: amplifier recovery sweep over the PWM compare values (duty = cc / 1250), all 3 terminals at the same duty. The rotor must not turn.
		unsigned long v[4] = {0, 1250, 1, 1000};
		char* p = args;
		for (size_t i = 0; i < 4 && *p; ++i) {
			char* end = NULL;
			v[i] = strtoul(p, &end, 10);
			if (end == p) {
				warn_println("usage: id_rec [lo_cc 0..1250] [hi_cc] [step_cc] [loops]");
				return;
			}
			p = end;
			while (*p == ' ')
				++p;
		}
		if (smol_servo_loop_state() == SMOL_LOOP_MOTOR_ID) {
			warn_println("%s: a motor id test runs", cmd);
			return;
		}
		smol_motor_id_rec_start((uint32_t)v[0], (uint32_t)v[1], (uint32_t)v[2], (uint32_t)v[3]);
	}
	else if (0 == strcmp("id_cal", cmd)) {
		// id_cal [lo_cc] [hi_cc] [loops_per_cc] [warm_s] [warm_cc], defaults 25 1225 100 10 625: calibration sweep up and down in one run, all 3 terminals at the same duty. The rotor must not turn.
		unsigned long v[5] = {25, 1225, 100, 10, 625};
		char* p = args;
		for (size_t i = 0; i < 5 && *p; ++i) {
			char* end = NULL;
			v[i] = strtoul(p, &end, 10);
			if (end == p) {
				warn_println("usage: id_cal [lo_cc 0..1250] [hi_cc] [loops_per_cc] [warm_s] [warm_cc]");
				return;
			}
			p = end;
			while (*p == ' ')
				++p;
		}
		if (smol_servo_loop_state() == SMOL_LOOP_MOTOR_ID) {
			warn_println("%s: a motor id test runs", cmd);
			return;
		}
		smol_motor_id_cal_start((uint32_t)v[0], (uint32_t)v[1], (uint32_t)v[2], (uint32_t)v[3], (uint32_t)v[4]);
	}
	else if (0 == strcmp("drv_io", cmd)) {
		// TMC6200 input pin levels and status, also while a loop state runs
		tmc_state_t st;
		if (tmc6200_read_state(&st) != 0) {
			err_println("drv_io: read failed");
			return;
		}
		info_println("drv_io: UH %u UL %u VH %u VL %u WH %u WL %u DRV_EN %u, GSTAT 0x%08" PRIx32 ", GCONF 0x%08" PRIx32,
			st.IOIN.UH, st.IOIN.UL, st.IOIN.VH, st.IOIN.VL, st.IOIN.WH, st.IOIN.WL, st.IOIN.DRV_EN, st.GSTAT.reg, st.GCONF.reg);
	}
	else if (0 == strcmp("drv_en", cmd)) {
		smol_drv_enable();
	}
	else if (0 == strcmp("drv_dis", cmd)) {
		smol_drv_disable();
		info_println("TMC6200 disabled");
	}
	else if (0 == strcmp("pd_req", cmd)) {
		// pd_req: print the source capabilities and the contract. pd_req <mV>: new contract at this voltage (9000 mV or more). pd_req max: boot default (highest voltage).
		if (args[0] == 0) {
			smol_usb_pd_print_status();
			return;
		}
		uint32_t mv = 0;
		if (0 != strcmp("max", args)) {
			char* end = NULL;
			unsigned long v = strtoul(args, &end, 10);
			while (end && (*end == ' '))
				++end;
			if ((end == args) || (end && (*end != 0)) || (v == 0)) {
				warn_println("usage: pd_req [<mV> | max]");
				return;
			}
			mv = (uint32_t)v;
		}
		if (smol_drv_is_enabled())
			warn_println("pd_req: the TMC6200 is enabled. Use drv_dis before a voltage change");
		smol_usb_pd_request_mv(mv);
	}
	else if (0 == strcmp("cfg", cmd)) {
		smol_config_print();
	}
	else if (0 == strcmp("cfg_set", cmd)) {
		// args: <key> <value>
		char* value = strchr(args, ' ');
		if (!value) {
			warn_println("usage: cfg_set <key> <value>");
			return;
		}
		*value++ = 0;
		if (smol_config_set(args, value) == 0)
			info_println("config: %s = %s (not saved)", args, value);
	}
	else if (0 == strcmp("cfg_save", cmd)) {
		smol_config_save();
	}
	else if (0 == strcmp("cfg_load", cmd)) {
		smol_config_load();
	}
	else {
		warn_println("unknown command: %s", cmd);
	}
}

static void smol_console_process_input_line(char* line, size_t linelen) {
	dbg_println("process_line: %.*s", linelen, line);
	if (linelen == 0)
		return;
	char* cmd_end = strpbrk(line, " ");
	size_t args_len = 0;
	size_t cmd_len = 0;
 	if (!cmd_end) {
 		cmd_end = line + linelen;
 		cmd_len = linelen;
 	}
 	else {
 		cmd_len = cmd_end - line;
 		args_len = (linelen - cmd_len) - 1;
 		// terminate the command, so that it compares as a string. args stays terminated by the line end.
 		*cmd_end = 0;
 	}

 	static char empty_args[] = "";
 	_process_command(line, cmd_len, (args_len > 0) ? cmd_end + 1 : empty_args, args_len);
}

void smol_console_process_char(int ch) {
	bool is_newline = (ch == '\n') || (ch == '\r');
	if ((ch == '\b') && (_input_cursor > 0)) {
		--_input_cursor;
	}
	if (_input_cursor < SMOL_CONSOLE_INPUT_BUF_BYTES) {

		if (ch == '\b') {
			// already backspaced
		}
		else if (!is_newline) {
			_input_buf[_input_cursor] = ch;
			++_input_cursor;
		}
		printf("\r%.*s", _input_cursor, _input_buf);
		// if we're ending with a newline, 
		if (is_newline) {
			printf("\r\n");
			// terminate line
			_input_buf[_input_cursor] = 0;
			smol_console_process_input_line(_input_buf, _input_cursor);
			// reset input line buffer
			_input_cursor = 0;
			_input_buf[0] = 0;
		}
	}
	else {
		printf("\a");
	}
}