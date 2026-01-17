#include "smol_console.h"

#include <stdio.h>
#include <stddef.h>
#include <string.h>
#include <stdbool.h>

#define SMOL_CONSOLE_INPUT_BUF_BYTES (128)

static char _input_buf[SMOL_CONSOLE_INPUT_BUF_BYTES];
static size_t _input_cursor = 0;

static void smol_console_process_input_line(char* line, size_t linelen) {
	char* cmd_end = strpbrk(line, " ");
 	if (!cmd_end)
 		cmd_end = line + linelen;
}

void smol_console_process_char(int ch) {
	bool is_newline = ch == '\n';
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
			smol_console_process_input_line(_input_buf, _input_cursor - 1);
			// reset input line buffer
			_input_cursor = 0;
			_input_buf[0] = 0;
		}
	}
	else {
		printf("\a");
	}
}