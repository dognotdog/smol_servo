#include "as5047d.h"
#include "pico_debug.h"

#include "hardware/gpio.h"
#include "hardware/spi.h"
#include "hardware/structs/spi.h"

#include <string.h>

static spi_inst_t *m_spi = NULL;
static int m_spi_mosi_pin;
static int m_spi_nss_pin;
static int m_spi_sclk_pin;
static int m_spi_miso_pin;

// number of detection attempts. Each failed attempt writes a warning to the log.
#define AS5047D_DETECT_ATTEMPTS (3)

int as5047d_spi_init(spi_inst_t *spi, int mosi_pin, int nss_pin, int sclk_pin, int miso_pin) {
	m_spi = spi;
	m_spi_mosi_pin = mosi_pin;
	m_spi_nss_pin = nss_pin;
	m_spi_sclk_pin = sclk_pin;
	m_spi_miso_pin = miso_pin;

	/*
	 * A warm reset of the RP2350 (SWD reset, flash, watchdog) does not remove the power of the AS5047D.
	 * When the MAG DMA runs (after smol_pwm_start()), a frame is active for approx. 90% of the time. The reset cuts that frame.
	 * The board has no pull-up on CSn (MAG_NSS), and the RP2350 pad reset state is pull-down. Thus CSn stays low, and the AS5047D keeps the cut frame open until this init.
	 * The first CSn rising edge ends the cut frame. The AS5047D then sets PARERR in ERRFL, and EF in the next answer.
	 * Thus: first set CLK and MOSI to a driven low, then set CSn high with SIO. This ends the cut frame before the first real frame. as5047d_detect() then reads ERRFL to clear the error.
	 * The levels are the same as the SPI idle levels (mode 1, CPOL=0), so the change to the SPI function below causes no edge.
	 */
	gpio_init(sclk_pin);
	gpio_put(sclk_pin, 0);
	gpio_set_dir(sclk_pin, GPIO_OUT);

	gpio_init(mosi_pin);
	gpio_put(mosi_pin, 0);
	gpio_set_dir(mosi_pin, GPIO_OUT);

	gpio_init(nss_pin);
	gpio_put(nss_pin, 1);
	gpio_set_dir(nss_pin, GPIO_OUT);

	// CSn high time tCSn >= 350ns
	sleep_us(10);

	// we have a limit of 350ns for the initial clock edge after NSS is asserted according to the AS5047D datasheet, limiting us to about 1.428MHz. This sample at 80kHz, this just works out.
	spi_init(m_spi, 1428571);
	spi_set_format(spi, 16, SPI_CPOL_0, SPI_CPHA_1, SPI_MSB_FIRST);

	gpio_set_function(miso_pin, GPIO_FUNC_SPI);
	gpio_set_function(sclk_pin, GPIO_FUNC_SPI);
	gpio_set_function(mosi_pin, GPIO_FUNC_SPI);
	gpio_set_function(nss_pin, GPIO_FUNC_SPI);

	return 0;
}

/**
 * Send numregs commands, then one NOP.
 * The AS5047D sends the answer to a command in the next frame. Thus rx[i] is the answer to registers[i].
 * If prev_rx is not NULL, it gets the answer of the first frame, which is the answer to the command before this call.
 */
static int _read_registers(const uint16_t* registers, uint16_t* rx, size_t numregs, uint16_t* prev_rx) {
	uint16_t first_rx = 0xFFFF;

	for (size_t i = 0; i < numregs; ++i) {
		uint16_t* dst = (i == 0) ? &first_rx : &(rx[i-1]);
		int read_result = spi_write16_read16_blocking(m_spi, &(registers[i]), dst, 1);

		if (read_result != 1) {
			return -1;
		}
		sleep_us(1);
	}

	uint16_t nop = (_PAR(MAG_READ_FLAG | MAG_REG_NOP));

	// NOP command to read last register
	int read_result = spi_write16_read16_blocking(m_spi, &nop, rx + numregs - 1, 1);

	if (read_result != 1) {
		return -1;
	}

	if (prev_rx)
		*prev_rx = first_rx;

	return 0;
}

static int _detect_attempt(int attempt) {
	enum {
		CMD_ERRFL_CLEAR,
		CMD_NOP,
		CMD_ERRFL,
		CMD_PROG,
		CMD_SETTINGS1,
		CMD_COUNT
	};
	const uint16_t cmd[CMD_COUNT] = {
		[CMD_ERRFL_CLEAR] = (_PAR(MAG_READ_FLAG | MAG_REG_ERRFL)), // read clears the errors from before this attempt
		[CMD_NOP]         = (_PAR(MAG_READ_FLAG | MAG_REG_NOP)), // should return 0x0000
		[CMD_ERRFL]       = (_PAR(MAG_READ_FLAG | MAG_REG_ERRFL)), // should return 0x0000
		[CMD_PROG]        = (_PAR(MAG_READ_FLAG | MAG_REG_PROG)), // should return 0x0000
		[CMD_SETTINGS1]   = (_PAR(MAG_READ_FLAG | MAG_REG_SETTINGS1)), // should return 0x0001 (with parity 0x8001)
	};

	uint16_t rx[CMD_COUNT];
	for (size_t i = 0; i < CMD_COUNT; ++i)
		rx[i] = 0xFFFF;
	uint16_t prev_rx = 0xFFFF;

	int read_result = _read_registers(cmd, rx, CMD_COUNT, &prev_rx);

	dbg_println("AS5047D attempt %d: prev 0x%04x, ERRFL before 0x%04x, NOP 0x%04x, ERRFL 0x%04x, PROG 0x%04x, SETTINGS1 0x%04x",
		attempt, prev_rx, rx[CMD_ERRFL_CLEAR], rx[CMD_NOP], rx[CMD_ERRFL], rx[CMD_PROG], rx[CMD_SETTINGS1]);

	// error flags from before this attempt: not a detection failure, because the read clears them
	if ((read_result == 0) && (rx[CMD_ERRFL_CLEAR] != _PAR(0)))
		info_println("AS5047D attempt %d: ERRFL 0x%04x before the detection frames (cleared)", attempt, rx[CMD_ERRFL_CLEAR]);

	bool is_present = (read_result == 0)
		&& (rx[CMD_NOP] == _PAR(0))
		&& (rx[CMD_ERRFL] == _PAR(0))
		&& (rx[CMD_PROG] == _PAR(0))
		&& (rx[CMD_SETTINGS1] == _PAR(0x0001));

	if (!is_present) {
		warn_println("AS5047D attempt %d failed: read %d, NOP 0x%04x, ERRFL 0x%04x, PROG 0x%04x, SETTINGS1 0x%04x",
			attempt, read_result, rx[CMD_NOP], rx[CMD_ERRFL], rx[CMD_PROG], rx[CMD_SETTINGS1]);
	}

	return is_present ? 0 : -1;
}

int as5047d_detect(void) {
	bool is_present = false;
	int attempt = 1;
	for (; attempt <= AS5047D_DETECT_ATTEMPTS; ++attempt) {
		if (0 == _detect_attempt(attempt)) {
			is_present = true;
			break;
		}
	}

	info_println("AS5047D detected: %u (attempts %d)", is_present, is_present ? attempt : AS5047D_DETECT_ATTEMPTS);

	return is_present ? 0 : -1;
}
