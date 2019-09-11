
#include "atmel_start.h"
#include "hal_spi_m_sync.h"
#include "hal_io.h"

#include "mecha_spi.h"


/*
AS5047 does 16bit SPI transfers
BIT15 is an even parity on commands as well as data, and transfers are big endian.

Propagation delay of sensor is spec'd at 100us +-10us from sensing to read via SPI

At 8Mhz, SPI read of 16bits takes 2us nominally, so ~10us for the 4x16bit words seems a reasonable assumption

*/





#define _XORS(x,s) (x ^ x >> 1)

// #define  _EVENPAR(x) (x)
#define  _EVENPAR(x) (((~_XORS(_XORS(_XORS(_XORS(_XORS(x, 16), 8), 4), 2), 1) & 1) << 15) | (x & 0x7FFF))
#define FLIP16(x) (((x & 0xFF00) >> 8) | ((x & 0xFF) << 8))
// #define FLIP16(x) (x)

uint16_t mspi_wr(uint16_t cmd)
{
	struct io_descriptor *io = NULL;
	spi_m_sync_get_io_descriptor(&SPI_0, &io);


	gpio_set_pin_level(HEADER_SS, false);
	io_write(io, (void*)&cmd, 2);
	gpio_set_pin_level(HEADER_SS, true);
	uint16_t result = 0;
	io_read(io, (void*)&result, 2);

	return result;

};

mspi_as5047_state_t mspi_readSensor(void)
{
	uint16_t cmd[4] = {
		FLIP16(_EVENPAR(0x4001)),
		FLIP16(_EVENPAR(0x7FFC)),
		FLIP16(_EVENPAR(0x7FFE)),
		FLIP16(_EVENPAR(0x4000))
	};

	uint16_t rx[4] = {0xFFFF,0xFFFF,0xFFFF,0xFFFF};



	gpio_set_pin_level(HEADER_SS, true);
	spi_m_sync_enable(&SPI_0);
	// delay_us(1);
	for (size_t i = 0; i < 4; ++i)
	{
		struct spi_xfer xfer = {
			.txbuf = (void*)(cmd+i),
			.rxbuf = (void*)(rx+i),
			.size = 2
		};
		gpio_set_pin_level(HEADER_SS, false);
		// delay_us(1);
		spi_m_sync_transfer(&SPI_0, &xfer);
		gpio_set_pin_level(HEADER_SS, true);
		// delay_us(1);

	}

	spi_m_sync_disable(&SPI_0);

	mspi_as5047_state_t state = {
		.ERRFL = FLIP16(rx[1]),
		.DIAAGC = FLIP16(rx[2]),
		.ANGLEUNC = FLIP16(rx[3]),
	};

	return state;
}

void mspi_init(void)
{
	// SPI (sync) baudrate = FCPU/2/(BAUDREG+1)
	// at 48MHz:
	// 8MHz = 2
	// 6MHz = 3
	// 4.8Mhz = 4
	// 4MHz = 5
	// 3MHz = 7
	// 2MHz = 11 
	// 1Mhz = 23
	spi_m_sync_disable(&SPI_0);
	spi_m_sync_set_baudrate(&SPI_0, 2);
	spi_m_sync_set_data_order(&SPI_0, SPI_DATA_ORDER_MSB_1ST);
	spi_m_sync_set_mode(&SPI_0, SPI_MODE_1);

	// bring SS high as default
	gpio_set_pin_level(HEADER_SS, true);

	// spi_m_sync_enable(&SPI_0);

}