#include "smol_servo.h"

#include "as5047d.h"

#include "hardware/gpio.h"
#include "hardware/pwm.h"
#include "hardware/dma.h"
#include "hardware/spi.h"
#include "hardware/structs/spi.h"

#define MAG_DMA_BUF_LENGTH (4)
#define MAG_DMA_RING_BITS (3)
_Static_assert((1 << MAG_DMA_RING_BITS) == (2 * MAG_DMA_BUF_LENGTH));

// servo loop driven DMA buffering
// 
static uint16_t __not_in_flash("MAG") _tx_loop_buf[MAG_DMA_BUF_LENGTH];
static uint16_t _tx_loop_buf[MAG_DMA_BUF_LENGTH] __aligned(sizeof(_tx_loop_buf));

/**
 * starting the buffers at index 0, with +1/6th offset of the servo loop PWM, we observed on MOSI:
 *   ... [M] ANGLEUNC MAG ANGLEUNC DIAAGC [M] ...
 * where [M] is the marker for the servo loop callback.
 * 
 * In order to preserve the sequence below, we add a -1 offset to the DMA buffer index setup. The rx_loop_buf index is additionally offset because reads are delayed by 1 transaction, but we'd like to keep the same buffer positions. This means that we're operating with"
         ... [M]                                    [M] ...
 * MOSI:      |  DIAAGC  ANGLEUNC   MAG    ANGLEUNC  |
 * MISO:      | ANGLEUNC  DIAAGC  ANGLEUNC   MAG     |
 * BUF:       |   [3]      [0]      [1]      [2]     |
 * 
 * Knowing ANGLEUNC time is important if we want to properly model time progression in our servo control model.
 * 
 * We have observed via scoping the NSS and PWM signals that NSS goes low within a few NS of of WRAP, as would be expected, and NSS stays high for ~490ns, with SPI initalized at a `1428571` baud rate.
 * 
 * At 20kHz, we have 50us between samples. The AS5047D latches values for the next read when NSS is deasserted, so timings for the ANGLE reads are:
 * [1] @ -25us - ~490ns = -25.49us
 * [3] @ -50us - ~490ns = -50.49us
 * 
 * 
 */

static uint16_t _tx_loop_buf[MAG_DMA_BUF_LENGTH] = {
	_PAR(MAG_READ_FLAG | MAG_REG_DIAAGC),
	_PAR(MAG_READ_FLAG | MAG_REG_ANGLEUNC),
	_PAR(MAG_READ_FLAG | MAG_REG_MAG),
	_PAR(MAG_READ_FLAG | MAG_REG_ANGLEUNC),
};

static uint16_t _rx_loop_buf[MAG_DMA_BUF_LENGTH];
static uint16_t _rx_loop_buf[MAG_DMA_BUF_LENGTH] __aligned(sizeof(_rx_loop_buf));

static int _mag_tx_dma_channel = -1;
static int _mag_rx_dma_channel = -1;

static size_t _angle_indices[2] = {3,1};

// AS5047D has an intrinsic ~100us (90-110us per datasheet) processing delay.
#define MAG_SENSOR_DELAY_NS (100 * 1000)

static uint32_t _angle_delays_ns[2] = {MAG_SENSOR_DELAY_NS + 50490, MAG_SENSOR_DELAY_NS + 25490};

static bool _mag_is_present = false;

void __not_in_flash_func(smol_mag_get_reading_values)(uint32_t values[2]) {
	for (size_t i = 0; i < 2; ++i) {
		values[i] = _rx_loop_buf[_angle_indices[i]];
	}
}

void __not_in_flash_func(smol_mag_get_reading_ages_ns)(int32_t ages[2]) {
	for (size_t i = 0; i < 2; ++i) {
		ages[i] = _angle_delays_ns[i];
	}
}


static void _rx_dma_init(void) {
	_mag_rx_dma_channel = dma_claim_unused_channel(true);

	dma_channel_config_t rx_config = dma_channel_get_default_config(_mag_rx_dma_channel);
	channel_config_set_transfer_data_size(&rx_config, DMA_SIZE_16);
	channel_config_set_ring(&rx_config, true, MAG_DMA_RING_BITS);
	channel_config_set_read_increment(&rx_config, false);
	channel_config_set_write_increment(&rx_config, true);

	channel_config_set_dreq(&rx_config, spi_get_dreq(MAG_SPI, false));
	// do not chain to other channel
	channel_config_set_chain_to(&rx_config, _mag_rx_dma_channel);
	channel_config_set_enable(&rx_config, true);
	// channel_config_set_high_priority(&rx_config, true);

    dma_channel_configure(
    	_mag_rx_dma_channel, 
    	&rx_config, 
    	_rx_loop_buf + MAG_DMA_BUF_LENGTH - 2, // reads are offset
    	&((spi_hw_t*)MAG_SPI)->dr,
    	// dma_encode_transfer_count_with_self_trigger(1),
       	dma_encode_endless_transfer_count(),
    	true
    );

    // enable RX DMA
    *hw_set_alias(&((spi_hw_t*)MAG_SPI)->dmacr) = SPI_SSPDMACR_RXDMAE_BITS;

}

static void _tx_dma_init(void) {
	_mag_tx_dma_channel = dma_claim_unused_channel(true);

	dma_channel_config_t tx_config = dma_channel_get_default_config(_mag_tx_dma_channel);
	channel_config_set_transfer_data_size(&tx_config, DMA_SIZE_16);
	channel_config_set_ring(&tx_config, false, MAG_DMA_RING_BITS);
	channel_config_set_read_increment(&tx_config, true);
	channel_config_set_write_increment(&tx_config, false);

	// 2x faster than the PWM phase correct interrupts
	channel_config_set_dreq(&tx_config, pwm_get_dreq(PWM_TWOTHIRDS_SLICE));

	// do not chain to other channel
	channel_config_set_chain_to(&tx_config, _mag_tx_dma_channel);
	channel_config_set_enable(&tx_config, true);
	// channel_config_set_high_priority(&tx_config, true);

    dma_channel_configure(
    	_mag_tx_dma_channel, 
    	&tx_config, 
    	&((spi_hw_t*)MAG_SPI)->dr,
    	_tx_loop_buf + MAG_DMA_BUF_LENGTH - 1,
    	// dma_encode_transfer_count_with_self_trigger(1),
    	dma_encode_endless_transfer_count(),
    	true
    );

}

static void _mag_dma_init(void) {
	assert(get_core_num() == 1);

	_rx_dma_init();
	_tx_dma_init();
}

void smol_mag_init(void) {
    as5047d_spi_init(MAG_SPI, MAG_MOSI_PIN, MAG_NSS_PIN, MAG_SLCK_PIN, MAG_MISO_PIN);

	_mag_is_present = 0 == as5047d_detect();

	if (_mag_is_present) {
		_mag_dma_init();
	}
}

void smol_mag_start(void) {
}