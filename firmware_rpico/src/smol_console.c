#include "smol_console.h"
#include "smol_servo.h"

#include "pico_debug.h"
#include "smol_config.h"

#include <stdio.h>
#include <stddef.h>
#include <string.h>
#include <stdbool.h>

#define SMOL_CONSOLE_INPUT_BUF_BYTES (128)

static char _input_buf[SMOL_CONSOLE_INPUT_BUF_BYTES];
static size_t _input_cursor = 0;

static void _process_command(char* cmd, size_t cmd_len, char* args, size_t args_len) {
	dbg_println("process_command: %.*s", cmd_len, cmd);
	if (0 == strcmp("s", cmd)) {
		smol_servo_loop_start();
	}
	else if (0 == strcmp("stop", cmd)) {
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
				warn_println("unknown loop state: %s (off, zero_cal, motor_id, control, pwm_test)", args);
			else
				smol_servo_loop_request_state(state);
		}
	}
	else if (0 == strcmp("drv_en", cmd)) {
		smol_drv_enable();
	}
	else if (0 == strcmp("drv_dis", cmd)) {
		smol_drv_disable();
		info_println("TMC6200 disabled");
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