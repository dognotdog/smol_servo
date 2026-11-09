#include "tmc6200.h"

#include <stddef.h>
#include <stdint.h>

#include "hardware/gpio.h"
#include "hardware/sync.h"
#include "pico/binary_info.h"

/**
 * The TMC6200 does 40bit transfers at a time, with the first 8bit being the register address, and the last 32bit being the data. CLK is inverted, CPOL=1 CPHA=1, Mode=3.
 * 
 * MSB is READ=0, WRITE=1.
 * 
 * CSN (NSS) is the SPI1 CSn pin function, driven by the SPI peripheral. The datasheet (4.1, 4.2) requires CSN to stay low for the whole 40-bit datagram, and latches the datagram when CSN goes high. The PL022 sends 8-bit frames, so one datagram is 5 frames. In SPI mode 3 (CPHA=1), the PL022 keeps CSn low between back-to-back frames, and sets it high only when the TX FIFO is empty after the last frame. Thus all 5 bytes go into the 8-entry TX FIFO with interrupts disabled, so that an interrupt cannot empty the FIFO in the middle of the datagram.
 *
 * Read data comes back in the same datagram as the read address (datasheet 4.1.1). The first byte received is the SPI status.
 *
 * Timing (datasheet 4.3): fSCK max 3.5MHz with the internal oscillator, CSN high time > 2 tCLK + 10ns between datagrams.
 *
 * The current sense circuitry is susceptible to switching spikes for 2-4us. The 60kHz PWM is ~16.67us. We sample the channels at 120kHz intervals, centered on the pulse hi and low periods, so one of them should always be at more than 4us from an edge.
 * 
 */

#define TMC_RW_BLOCK_SIZE 5
#define TMC_STATE_NUM_REGISTERS 5

static spi_inst_t *m_spi = NULL;
static int m_spi_mosi_pin;
static int m_spi_nss_pin;
static int m_spi_sclk_pin;
static int m_spi_miso_pin;

/**
 * Send one 40-bit datagram and receive the 40-bit reply into the same buffer.
 */
static void _tmc6200_transfer(uint8_t buf[TMC_RW_BLOCK_SIZE]) {
	assert(m_spi);
	spi_hw_t* hw = spi_get_hw(m_spi);

	// BSY is set while a frame is sent or the TX FIFO is not empty: after this, the TX FIFO is empty and CSn is high
	while (spi_is_busy(m_spi))
		tight_loop_contents();
	// discard old data in the RX FIFO
	while (spi_is_readable(m_spi))
		(void)hw->dr;

	// the empty TX FIFO has space for the 5 bytes, so the writes do not wait
	uint32_t irq_state = save_and_disable_interrupts();
	for (size_t i = 0; i < TMC_RW_BLOCK_SIZE; ++i)
		hw->dr = buf[i];
	restore_interrupts(irq_state);

	// the RX FIFO (8 entries) holds the 5 bytes of the reply
	for (size_t i = 0; i < TMC_RW_BLOCK_SIZE; ++i) {
		while (!spi_is_readable(m_spi))
			tight_loop_contents();
		buf[i] = (uint8_t)hw->dr;
	}
	// wait for the end of the last frame, so that CSn is high and the datagram is latched
	while (spi_is_busy(m_spi))
		tight_loop_contents();
}

int tmc6200_read_registers(uint8_t cmd[], uint8_t result[][5], size_t numregs) {
	for (size_t i = 0; i < numregs; ++i)
	{
		result[i][0] = cmd[i] & 0x7F; // read access: W bit = 0
		result[i][1] = 0;
		result[i][2] = 0;
		result[i][3] = 0;
		result[i][4] = 0;
		_tmc6200_transfer(result[i]);
	}
	return 0;
}

int tmc6200_write_register(uint8_t addr, uint32_t data) {
	uint8_t buf[TMC_RW_BLOCK_SIZE] = {
		(addr & 0x7F) | 0x80, // write access: W bit = 1
		(data >> 24) & 0xFF,
		(data >> 16) & 0xFF,
		(data >> 8) & 0xFF,
		(data >> 0) & 0xFF,
	};
	_tmc6200_transfer(buf);
	return 0;
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

	// fSCK max is 3.5MHz with the internal oscillator
	spi_init(m_spi, 1000 * 1000);
	// SPI mode 3. CPHA=1 is also necessary for the PL022 to keep CSn low between the 5 frames of a datagram.
	spi_set_format(spi, 8, SPI_CPOL_1, SPI_CPHA_1, SPI_MSB_FIRST);

	gpio_set_function(miso_pin, GPIO_FUNC_SPI);
	gpio_set_function(sclk_pin, GPIO_FUNC_SPI);
	gpio_set_function(mosi_pin, GPIO_FUNC_SPI);
	// CSn driven by the SPI peripheral, see _tmc6200_transfer()
	gpio_set_function(nss_pin, GPIO_FUNC_SPI);

	return 0;
}