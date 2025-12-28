#include "tmc6200.h"

#include <stddef.h>
#include <stdint.h>

#include "hardware/gpio.h"
#include "pico/binary_info.h"

/**
 * The TMC6200 does 40bit transfers at a time, with the first 8bit being the register address, and the last 32bit being the data. CLK is inverted, CPOL=1 CPHA=1, Mode=3.
 * 
 * MSB is READ=0, WRITE=1.
 * 
 */

#define TMC_RW_BLOCK_SIZE 5
#define TMC_STATE_NUM_REGISTERS 5

static spi_inst_t *m_spi = NULL;
int m_spi_mosi_pin;
int m_spi_nss_pin;
int m_spi_sclk_pin;
int m_spi_miso_pin;

int tmc6200_read_registers(uint8_t cmd[], uint8_t result[][5], size_t numregs) {
	for (size_t i = 0; i < numregs; ++i)
	{
		result[i][0] = cmd[i];
		int read_result = spi_write_read_blocking(m_spi, result[i],result[i], TMC_RW_BLOCK_SIZE);
		if (read_result != 0)
			return read_result;
	}
	return 0;
}

int tmc6200_write_register(uint8_t addr, uint32_t data) {
	uint8_t cmd[TMC_RW_BLOCK_SIZE] = {
		(addr & 0x7F) | 0x80, 
		data << 24, 
		data << 16, 
		data << 8, 
		data << 0,
	};

	uint8_t rx[5] = {0};
	int read_result = spi_write_read_blocking(m_spi, cmd, rx, TMC_RW_BLOCK_SIZE);
	return read_result;
}

int tmc6200_read_state(tmc_state_t* state) {
	assert(state);
	uint8_t cmd[TMC_STATE_NUM_REGISTERS] = {
		TMC_REG_GCONF,
		TMC_REG_GSTAT,
		TMC_REG_IOIN,
		TMC_REG_SHORT_CONF,
		TMC_REG_DRV_CONF,
	};

	uint8_t rx[TMC_STATE_NUM_REGISTERS][TMC_RW_BLOCK_SIZE] = {{0}};

	int read_result = tmc6200_read_registers(cmd, rx, TMC_STATE_NUM_REGISTERS);

	if (read_result != 0) {
		return read_result;
	}

	state->GCONF.reg = (rx[0][1] << 24) | (rx[0][2] << 16) | (rx[0][3] << 8) | (rx[0][4] << 0);
	state->GSTAT.reg = (rx[1][1] << 24) | (rx[1][2] << 16) | (rx[1][3] << 8) | (rx[1][4] << 0);
	state->IOIN.reg = (rx[2][1] << 24) | (rx[2][2] << 16) | (rx[2][3] << 8) | (rx[2][4] << 0);
	state->SHORT_CONF.reg = (rx[3][1] << 24) | (rx[3][2] << 16) | (rx[3][3] << 8) | (rx[3][4] << 0);
	state->DRV_CONF.reg = (rx[4][1] << 24) | (rx[4][2] << 16) | (rx[4][3] << 8) | (rx[4][4] << 0);

	return 0;
}

int _write_gconf(tmc_state_t* state) {
	return tmc6200_write_register(TMC_REG_GCONF, state->GCONF.reg);
}

int tmc6200_enable_driver(bool en) {
	tmc_state_t state;
	int read_result = tmc6200_read_state(&state);
	if (read_result != 0)
		return read_result;

	state.GCONF.disable = en;

	return _write_gconf(&state);
}

int tmc6200_enable_3pwm_mode(bool en) {
	tmc_state_t state;
	int read_result = tmc6200_read_state(&state);
	if (read_result != 0)
		return read_result;

	state.GCONF.singleline = en;

	return _write_gconf(&state);
}

int tmc6200_spi_init(spi_inst_t *spi, int mosi_pin, int nss_pin, int sclk_pin, int miso_pin) {
	m_spi = spi;
	m_spi_mosi_pin = mosi_pin;
	m_spi_nss_pin = nss_pin;
	m_spi_sclk_pin = sclk_pin;
	m_spi_miso_pin = miso_pin;

	spi_init(m_spi, 1000 * 1000);
	spi_set_format(spi, 8, SPI_CPOL_1, SPI_CPHA_1, SPI_MSB_FIRST);

    gpio_set_function(miso_pin, GPIO_FUNC_SPI);
    gpio_set_function(sclk_pin, GPIO_FUNC_SPI);
    gpio_set_function(mosi_pin, GPIO_FUNC_SPI);

    gpio_init(nss_pin);
    gpio_set_dir(nss_pin, GPIO_OUT);
    gpio_put(nss_pin, 1);

}