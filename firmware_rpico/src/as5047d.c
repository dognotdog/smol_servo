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

int as5047d_spi_init(spi_inst_t *spi, int mosi_pin, int nss_pin, int sclk_pin, int miso_pin) {
	m_spi = spi;
	m_spi_mosi_pin = mosi_pin;
	m_spi_nss_pin = nss_pin;
	m_spi_sclk_pin = sclk_pin;
	m_spi_miso_pin = miso_pin;

	// we have a limit of 350ns for the initial clock edge after NSS is asserted according to the AS5047D datasheet, limiting us to about 1.428MHz. This sample at 80kHz, this just works out.
	spi_init(m_spi, 1428571);
	spi_set_format(spi, 16, SPI_CPOL_0, SPI_CPHA_1, SPI_MSB_FIRST);

    gpio_set_function(miso_pin, GPIO_FUNC_SPI);
    gpio_set_function(sclk_pin, GPIO_FUNC_SPI);
    gpio_set_function(mosi_pin, GPIO_FUNC_SPI);
    gpio_set_function(nss_pin, GPIO_FUNC_SPI);
}

static int _read_registers(uint16_t* registers, uint16_t* rx, size_t numregs) {

	// int read_result = spi_write16_read16_blocking(m_spi, registers, rx, numregs);

	// if (read_result != numregs) {
	// 	return -1;
	// }

	for (size_t i = 0; i < numregs; ++i) {
		int read_result = spi_write16_read16_blocking(m_spi, &(registers[i]), &(rx[i]), 1);

		if (read_result != 1) {
			return -1;
		}
		sleep_us(1);
	}

	// shift RX to account for reads being shifted by 16 bits
	memmove(rx, rx + 1, 2 * (numregs-1));

	uint16_t nop = (_PAR(0x4000));


	// NOP command to read last register
	int read_result = spi_write16_read16_blocking(m_spi, &nop, rx + numregs - 1, 1);

	if (read_result != numregs) {
		return -1;
	}

	return 0;
}

int as5047d_detect(void) {
	uint16_t cmd[4] = {
		(_PAR(0x4000)), // NOP should return 0x0000
		(_PAR(0x4001)), // ERRFL
		(_PAR(0x4003)), // PROG should return 0x0000
		(_PAR(0x4018)), // SETTINGS1 should return 0x0001
	};

	uint16_t rx[4] = {0xFFFF,0xFFFF,0xFFFF,0xFFFF};

	_read_registers(cmd, rx, 4);

	dbg_println("AS5047D 0x4000: 0x%04x", rx[0]);
	dbg_println("AS5047D 0x4001: 0x%04x", rx[1]);
	dbg_println("AS5047D 0x4003: 0x%04x", rx[2]);
	dbg_println("AS5047D 0x4018: 0x%04x", rx[3]);

	bool is_present = (rx[0] == _PAR(0)) && (rx[2] == _PAR(0)) && (rx[3] == _PAR(0x0001));
	info_println("AS5047D detected: %u", is_present);

	return is_present ? 0 : -1;
}