/**
 * smol-servo ADC setup
 * 
 * We're running up/down PWM at 60kHz (120kHz) and need to sample at both 0 and TOP+1 counts.
 * 
 * However, the PWM module only allows DMA requests on counter wrap, so we have to setup a spare PWM or other timer mechanism.
 * 
 * An ADC samples takes 2us, so we should trigger 1us before counter wrap (150 cycles).
 * 
 * NOTE: it may be possible to use a "negative" counter value instead of retarding the PWM counter by 150 cycles from software to get the correct phase alignment.
 * 
 * Additionally, we should interleave the 120kHz 3-phase current reads with
 * the voltage and auxiliary sensors. We have a total of 9 channels anyway,
 * might as well make use of it.
 * 
 *  0    TOP    0    TOP    0    TOP   :0 
 *  |     |     |     |     |     |    :|  ...
 *  I0    I0    I1    I1    I2    I2   :I0
 *     V0    V1    V2   VDIV  VBUS  TEMP
 * 
 * Thus the order of channels:
 * 0,3, 0,4, 1,5, 1,6, 2,7, 2,8
 * 
 * This would result in a 240kHz sample rate. We could insert additional reads to denoise the non-current sense reads:
 * 0,3,3,3, 0,4,4,4, 1,5,5,5, 1,6,6,6, 2,7,7,7, 2,8,8,8
 *
 * It does not make sense to increase the number of current sense reads because of potential switching noise. The current sense points are at 0 and TOP, so one of the two is picked depending on proximity of the switching time. One will always be relatively far away, the other closer (until we get to "step" speeds where polarity switches each PWM period)
 * 
 * What makes sense to set up? 120kHz is 1250 cycles, 240kHz is 625 cycles. Since we don't have an integer number of cycles to go higher, we'd have to feed the PWM TOP with 312/313 alternatingly. One option is to setup a DMA for TOP that, the other could be to use PH_RET to retard it by one cycle every other period.
 * 
 * If we feed TOP, we can do DMA chaining to trigger the ADC.
 * 
 * Going with 240kHz, we do not need to alternate counts.
 * 
 * We also have the 8 entry ADC FIFO, so don't need to do DMAs as long as we read out the FIFO often enough.
 * 
 * We'll start with 240kHz ADC reads for simplicity. DMA to ADC.CS will trigger ADC reads.
 * 
 * PWM[6].WRAP --.
 *               |
 *               V
 * CS_RINGBUF -> DMA -> ADC.CS -> ADC.READ -.
 *                                          |
 *                                        1 / 4
 *                                          |
 *                                          V
 *                            ADC.FCS -> adc_fifo_threshold_irq()
 *                                             
 * 
 * 
 */

#include "smol_servo.h"
#include "smol_servo_parameters.h"
#include "smol_fast.h"
#include "pico_debug.h"

#include "hardware/platform_defs.h"
#include "hardware/dma.h"
#include "hardware/adc.h"
#include "hardware/pwm.h"

#include <stdint.h>
#include <string.h>
#include <assert.h>

// 4 ADC reads for every interrupt 
#define ADC_PATTERN_BLOCK_SIZE 4
#define ADC_PATTERN_NUM_BLOCKS 3

#define ADC_RINGBUF_NUM_BLOCKS (4)
#define ADC_RINGBUF_WORDS (ADC_PATTERN_BLOCK_SIZE*ADC_RINGBUF_NUM_BLOCKS)
#define ADC_RINGBUS_BITS (6)
_Static_assert((1 << ADC_RINGBUS_BITS) == (4 * ADC_RINGBUF_WORDS));

#define ADC_PWM_TOP (SMOL_SERVO_ADC_PERIOD_CLOCKS-1)

#define ADC_FIFO_THRESHOLD (4)
// start with N-1 for the first block to align correctly with bridge PWM
#define ADC_FIFO_STARTING_THRESHOLD (ADC_FIFO_THRESHOLD-1)

#define ADC_CHANNEL_VBUS (3)
#define ADC_CHANNEL_VDD (4)
#define ADC_CHANNEL_TEMPERATURE (8)

#define ADC_IRQ_PERIOD_CLOCKS_NOMINAL (ADC_FIFO_THRESHOLD*SMOL_SERVO_ADC_PERIOD_CLOCKS)
// allow a 2us deviation
#define ADC_IRQ_PERIOD_CLOCKS_ALLOWANCE (300)
#define ADC_IRQ_PERIOD_CLOCKS_MIN (ADC_IRQ_PERIOD_CLOCKS_NOMINAL - ADC_IRQ_PERIOD_CLOCKS_ALLOWANCE)
#define ADC_IRQ_PERIOD_CLOCKS_MAX (ADC_IRQ_PERIOD_CLOCKS_NOMINAL + ADC_IRQ_PERIOD_CLOCKS_ALLOWANCE)

static int __not_in_flash("ADC") _adc_irq_period_clocks_min = ADC_IRQ_PERIOD_CLOCKS_MIN;
static int __not_in_flash("ADC") _adc_irq_period_clocks_max = ADC_IRQ_PERIOD_CLOCKS_MAX;

static int _adc_cs_dma_channel = -1;

static uint32_t _block_index = 0;
static uint32_t _ringbuf_index = 0;

static uint32_t _adc_cs_buf[ADC_PATTERN_NUM_BLOCKS][ADC_PATTERN_BLOCK_SIZE];

static uint32_t _adc_cs_ringbuf[ADC_RINGBUF_NUM_BLOCKS][ADC_PATTERN_BLOCK_SIZE] __attribute__((aligned(4*ADC_RINGBUF_WORDS)));

static volatile uint32_t _adc_read_buf[NUM_ADC_CHANNELS][2];
static volatile uint32_t _adc_time_buf[NUM_ADC_CHANNELS][2];

uint32_t smol_adc_block_index(void) {
	return _block_index;
}

static uint32_t _prev_irq_t_clocks = 0;

void __not_in_flash_func(smol_adc_fifo_threshold_irq_handler)(void) {
#ifdef DEBUG_ADC_FIFO_THRES_WITH_GPIO
	gpio_out_put_fast(DEBUG_ADC_FIFO_THRES_WITH_GPIO, 1);
#endif
	// we expect this to be called every ADC_FIFO_THRESHOLD samples
#ifdef DEBUG_ADC_FIFO_THRES
	dbg_println("adc fifo!");
#endif
	/**
	 * ADC block interrupt handler
	 * 
	 * store samples and times
	 */
	static bool is_first_run = true;

	uint32_t block_complete_index = _block_index;
	uint32_t ringbuf_complete_index = _ringbuf_index;

	uint32_t t_clocks = smol_time32(timer1_hw);
	uint32_t dt = t_clocks - _prev_irq_t_clocks;
	// Block of ADC reads has completed.
	// 1. Gather ADC reads
	// 2. Refill CS pattern ringbuf

	uint32_t prev_block_index = (block_complete_index + ADC_PATTERN_NUM_BLOCKS - 1) % ADC_PATTERN_NUM_BLOCKS;
	uint32_t next_block_index = (block_complete_index + 1) % ADC_PATTERN_NUM_BLOCKS;
	uint32_t next_ringbuf_index = (ringbuf_complete_index + 1) % ADC_RINGBUF_NUM_BLOCKS;

	if (is_first_run) {
		// let's see if this fixes the first run assert
		// NOTE: first run is N-1 entries we ignore.
		for (size_t i = 0; i < ADC_FIFO_STARTING_THRESHOLD; ++i) {
			volatile int val = adc_hw->fifo;
			// adc_fifo_get();
		}
		// update FIFO threshold
		uint32_t fcs = adc_hw->fcs;
		fcs = (fcs & ~ADC_FCS_THRESH_BITS) | (ADC_FIFO_THRESHOLD << ADC_FCS_THRESH_LSB);

		adc_hw->fcs = fcs;
		// *hw_set_alias(&adc_hw->fcs) = ADC_FCS_UNDER_BITS | ADC_FCS_OVER_BITS;
		assert_fast(0 == (fcs & ADC_FCS_OVER_BITS));
		assert_fast(0 == (fcs & ADC_FCS_UNDER_BITS));
		is_first_run = false;
	}
	else
	{
		assert_fast(dt > _adc_irq_period_clocks_min);
		assert_fast(dt < _adc_irq_period_clocks_max);
		// gather ADC read results from FIFO

		// sanity check that we have exactly 4 samples waiting and no under/overflows
		uint32_t fcs = adc_hw->fcs;
		uint32_t cs = adc_hw->cs;
		assert_fast(0 == (fcs & ADC_FCS_OVER_BITS));
		assert_fast(0 == (fcs & ADC_FCS_UNDER_BITS));
		assert_fast(0 == (cs & ADC_CS_ERR_STICKY_BITS));
		uint32_t fifo_level = (fcs & ADC_FCS_LEVEL_BITS) >> ADC_FCS_LEVEL_LSB;
		// assert_fast(fifo_level >= ADC_FIFO_THRESHOLD - 3);
		// assert_fast(fifo_level >= ADC_FIFO_THRESHOLD - 2);
		// assert_fast(fifo_level >= ADC_FIFO_THRESHOLD - 1);
		assert_fast(fifo_level >= ADC_FIFO_THRESHOLD);
		// assert_fast(fifo_level <= ADC_FIFO_THRESHOLD + 2);
		// assert_fast(fifo_level <= ADC_FIFO_THRESHOLD + 1);
		assert_fast(fifo_level == ADC_FIFO_THRESHOLD);
		// assert_fast(0);

		// read FIFO into local buffer first...
		_Static_assert(ADC_PATTERN_BLOCK_SIZE == ADC_FIFO_THRESHOLD);
		uint16_t samples[ADC_PATTERN_BLOCK_SIZE] = {0};

		for (size_t i = 0; i < ADC_PATTERN_BLOCK_SIZE; ++i) {
			uint16_t val = adc_hw->fifo;
			// assert(0 == (val & 0x8000)); // assert no error bit set
			samples[i] = val;
		}

		// ...then assign samples to correct ADC channels.
		// samples contain two evenly spaced current reads [0,2] and two aux reads in between [1,3]
		// 
		assert_fast(block_complete_index < ADC_PATTERN_NUM_BLOCKS);
		_adc_read_buf[prev_block_index][0] = samples[0];
		_adc_read_buf[prev_block_index][1] = samples[2];
		_adc_read_buf[2*prev_block_index + 3][0] = samples[1];
		_adc_read_buf[2*prev_block_index + 4][0] = samples[3];

		// TIME: we're executed roughly at the end of the ADC sampling of the 4th block, so at about +1us (150clk) past center of sample.
		_adc_time_buf[prev_block_index][0] = t_clocks - 3 *SMOL_SERVO_ADC_PERIOD_CLOCKS - 150;
		_adc_time_buf[prev_block_index][1] = t_clocks - 1 *SMOL_SERVO_ADC_PERIOD_CLOCKS - 150;
		_adc_time_buf[2*prev_block_index + 3][0] = t_clocks - 2 *SMOL_SERVO_ADC_PERIOD_CLOCKS - 150;
		_adc_time_buf[2*prev_block_index + 4][0] = t_clocks - 0 *SMOL_SERVO_ADC_PERIOD_CLOCKS - 150;
	}

	// refill CS pattern. This is a software op because of the power-of-two restriction on DMA ringbuffer sizes, but we have a 3x factor for our 3-phase pattern.
	// we're refilling the block we just executed, so that this isn't timing critical for the next conversion block.
	for (size_t i = 0; i < ADC_PATTERN_BLOCK_SIZE; ++i) {
		_adc_cs_ringbuf[ringbuf_complete_index][i] = _adc_cs_buf[block_complete_index][i];
	}
	// memcpy(_adc_cs_ringbuf[ringbuf_complete_index], _adc_cs_buf[block_complete_index], sizeof(uint32_t)*ADC_PATTERN_BLOCK_SIZE);

	// update indices for next round.
	_block_index = next_block_index;
	_ringbuf_index = next_ringbuf_index;
	_prev_irq_t_clocks = t_clocks;

#ifdef DEBUG_ADC_FIFO_THRES_WITH_GPIO
	gpio_out_put_fast(DEBUG_ADC_FIFO_THRES_WITH_GPIO, 0);
	// if (prev_block_index == 2) {
	// 	gpio_out_put_fast(DEBUG_ADC_FIFO_THRES_WITH_GPIO, 0);
	// 	gpio_out_put_fast(DEBUG_ADC_FIFO_THRES_WITH_GPIO, 1);
	// 	gpio_out_put_fast(DEBUG_ADC_FIFO_THRES_WITH_GPIO, 1);
	// 	gpio_out_put_fast(DEBUG_ADC_FIFO_THRES_WITH_GPIO, 0);
	// }
#endif
}


static uint32_t smol_adc_cs_for_read(uint32_t ch) {
	assert(ch < 9);
	uint32_t ainsel     = ch << ADC_CS_AINSEL_LSB;
	uint32_t err_clear  = 1u << ADC_CS_ERR_STICKY_LSB;
	uint32_t start_once = 1u << ADC_CS_START_ONCE_LSB;
	uint32_t ts_en      = 1u << ADC_CS_TS_EN_LSB;
	uint32_t en         = 1u << ADC_CS_EN_LSB;

	return ainsel | err_clear | start_once | ts_en | en;
}


int smol_adc_cs_buf_init(void) {
	// init interleaved read pattern
	// [0,2] are bridge current sense values on channels 0-2
	// [1,3] are aux reads on channels 3-8
	for (size_t i = 0; i < ADC_PATTERN_NUM_BLOCKS; ++i) {
		_Static_assert(ADC_PATTERN_BLOCK_SIZE == 4);
		_adc_cs_buf[i][0] = smol_adc_cs_for_read(i);
		_adc_cs_buf[i][1] = smol_adc_cs_for_read(2*i + 3);
		_adc_cs_buf[i][2] = smol_adc_cs_for_read(i);
		_adc_cs_buf[i][3] = smol_adc_cs_for_read(2*i + 4);
	}
	// pre-fill ringbuffer
	for (size_t i = 0; i < ADC_RINGBUF_NUM_BLOCKS; ++i) {
		memcpy(_adc_cs_ringbuf[i], _adc_cs_buf[i % ADC_PATTERN_NUM_BLOCKS], 4 * ADC_PATTERN_BLOCK_SIZE);
	}
}

void __not_in_flash_func(smol_adc_dma_irq_handler)(void) {
	dma_channel_acknowledge_irq0(_adc_cs_dma_channel);
	dbg_println("adc cs dma!");
}

void __not_in_flash_func(smol_adc_pwm_irq_handler)(void) {
	pwm_clear_irq(PWM_ADC_SLICE);
	dbg_println("adc wrap!");
}

void __not_in_flash_func(smol_adc_timing_irq_handler)(void) {
#ifdef DEBUG_ADC_TIMING_WITH_GPIO
	gpio_out_put_fast(DEBUG_ADC_TIMING_WITH_GPIO, 1);
#endif
	pwm_clear_irq(PWM_SERVO_LOOP_SLICE);
	// dbg_println("adc wrap!");
#ifdef DEBUG_ADC_TIMING_WITH_GPIO
	gpio_out_put_fast(DEBUG_ADC_TIMING_WITH_GPIO, 0);
#endif
}


int smol_adc_dma_init(void) {
	assert(get_core_num() == 1);

	_adc_cs_dma_channel = dma_claim_unused_channel(true);

	dma_channel_config_t config = dma_channel_get_default_config(_adc_cs_dma_channel);
	// ringbuf byte count is 4 * 8 = 32, which is 5 bits.
	channel_config_set_transfer_data_size(&config, DMA_SIZE_32);
	channel_config_set_ring(&config, false, ADC_RINGBUS_BITS);
	channel_config_set_read_increment(&config, true);
	channel_config_set_write_increment(&config, false);
	channel_config_set_dreq(&config, pwm_get_dreq(PWM_ADC_SLICE));
	// do not chain to other channel
	channel_config_set_chain_to(&config, _adc_cs_dma_channel);
	channel_config_set_enable(&config, true);
	channel_config_set_high_priority(&config, true);

    // dma_channel_set_read_addr(_adc_cs_dma_channel, _adc_cs_ringbuf, false);
    // dma_channel_set_write_addr(_adc_cs_dma_channel, &adc_hw->cs, false);
    // dma_channel_set_transfer_count(_adc_cs_dma_channel, dma_encode_transfer_count_with_self_trigger(1), false);
    // dma_channel_set_transfer_count(_adc_cs_dma_channel, dma_encode_endless_transfer_count(), false);
	// dma_channel_set_config(_adc_cs_dma_channel, &config, true);

    dma_channel_configure(
    	_adc_cs_dma_channel, 
    	&config, 
    	&adc_hw->cs,
    	_adc_cs_ringbuf[0] + 1, // start first block with N-1 reads for PWM alignment
    	dma_encode_transfer_count_with_self_trigger(1),
    	true
    );

#ifdef DEBUG_DMA_ADC
    dma_channel_set_irq0_enabled(_adc_cs_dma_channel, true);
	irq_set_exclusive_handler(DMA_IRQ_0, smol_adc_dma_irq_handler);
	irq_set_priority(DMA_IRQ_0, 1);
	irq_set_enabled(DMA_IRQ_0, true);
#endif
}

int smol_adc_pwm_init(void) {
	{
		pwm_config config = pwm_get_default_config();
		pwm_config_set_wrap(&config, ADC_PWM_TOP);
		pwm_config_set_phase_correct(&config, false);
		pwm_config_set_clkdiv_mode(&config, PWM_DIV_FREE_RUNNING);
		pwm_init(PWM_ADC_SLICE, &config, false);

		pwm_set_counter(PWM_ADC_SLICE, SMOL_SERVO_ADC_ADVANCE_COUNT);
	}

#ifdef DEBUG_ADC_TIMING_WITH_GPIO
	gpio_init(DEBUG_ADC_TIMING_WITH_GPIO);
	gpio_set_dir(DEBUG_ADC_TIMING_WITH_GPIO, true);
	{
		pwm_config config = pwm_get_default_config();
		pwm_config_set_wrap(&config, SMOL_SERVO_BRIDGE_PWM_PERIOD_CLOCKS-1);
		pwm_config_set_phase_correct(&config, false);
		pwm_config_set_clkdiv_mode(&config, PWM_DIV_FREE_RUNNING);
		pwm_init(PWM_SERVO_LOOP_SLICE, &config, false);

		pwm_set_counter(PWM_SERVO_LOOP_SLICE, SMOL_SERVO_ADC_ADVANCE_COUNT);

	}
	pwm_clear_irq(PWM_ADC_SLICE);
	pwm_set_irq1_enabled(PWM_SERVO_LOOP_SLICE, true);

	irq_set_exclusive_handler(PWM_IRQ_WRAP_1, smol_adc_timing_irq_handler);
	irq_set_priority(PWM_IRQ_WRAP_1, 0);
	irq_set_enabled(PWM_IRQ_WRAP_1, true);
#endif

#ifdef DEBUG_PWM_ADC_GPIO
	gpio_set_function(DEBUG_PWM_ADC_GPIO, GPIO_FUNC_PWM);
	gpio_set_function(DEBUG_PWM_ADC_GPIO+1, GPIO_FUNC_PWM);
	pwm_set_both_levels(PWM_ADC_SLICE, 300, 300);
#endif

#ifdef DEBUG_PWM_ADC_IRQ
	pwm_clear_irq(PWM_ADC_SLICE);
	pwm_set_irq1_enabled(PWM_ADC_SLICE, true);

	irq_set_exclusive_handler(PWM_IRQ_WRAP_1, smol_adc_pwm_irq_handler);
	irq_set_priority(PWM_IRQ_WRAP_1, 1);
	irq_set_enabled(PWM_IRQ_WRAP_1, true);
#endif
}

void smol_adc_init(void) {
	assert(get_core_num() == 1);

	adc_init();
	adc_set_temp_sensor_enabled(true);

	adc_gpio_init(SOA_SENSE_PIN);
	adc_gpio_init(SOB_SENSE_PIN);
	adc_gpio_init(SOC_SENSE_PIN);

	adc_gpio_init(VBUS_SENSE_PIN);
	adc_gpio_init(VDD_SENSE_PIN);

	adc_gpio_init(ADCA_SENSE_PIN);
	adc_gpio_init(ADCB_SENSE_PIN);
	adc_gpio_init(ADCC_SENSE_PIN);

	// setup FIFO to trigger IRQ every 4 samples, but no DMA.
	// Since we already have a FIFO, we can just get the samples from there directly instead of using our own ring buffer.
	adc_fifo_setup(true, false, ADC_FIFO_STARTING_THRESHOLD, true, false);
	adc_fifo_drain();
	// clear errors
	*hw_set_alias(&adc_hw->fcs) = ADC_FCS_UNDER_BITS | ADC_FCS_OVER_BITS;
	assert(0 == (adc_hw->fcs & ADC_FCS_OVER_BITS));
	assert(0 == (adc_hw->fcs & ADC_FCS_UNDER_BITS));

	adc_irq_set_enabled(true);
	irq_set_exclusive_handler(ADC_IRQ, smol_adc_fifo_threshold_irq_handler);

	smol_adc_cs_buf_init();
	smol_adc_pwm_init();
	smol_adc_dma_init();


#ifdef DEBUG_ADC_FIFO_THRES_WITH_GPIO
	gpio_init(DEBUG_ADC_FIFO_THRES_WITH_GPIO);
	gpio_set_dir(DEBUG_ADC_FIFO_THRES_WITH_GPIO, true);
#endif

}

void smol_adc_start(void) {
	// PWM has to be running so we can do the counter retarding.
	// assert(smol_pwm_was_started());
	// However, since we're "advancing" the counter, this is a NOP, as the advance is done by setting the counter register before enabling the PWM slice.
}

#define ADC_COUNT_TO_VOLTAGE (ADC_REF_VOLTAGE / 4095.0f)

static float _adc_to_temperature_celsius(uint16_t sample) {
	float vbe = 0.706f;
	float slope = -1.721e-3f;
	float adc_voltage = sample * ADC_COUNT_TO_VOLTAGE;
	float t_celsius = 27.0f + (adc_voltage - vbe) / slope;
	return t_celsius;
}

void __not_in_flash_func(smol_adc_get_isense_values)(float values[3][2], uint32_t times[3][2]) {
	for (size_t i = 0; i < 3; ++i) {
		uint16_t sample0 = _adc_read_buf[i][0];
		uint16_t sample1 = _adc_read_buf[i][1];
		values[i][0] = sample0 * ADC_COUNT_TO_VOLTAGE;
		values[i][1] = sample1 * ADC_COUNT_TO_VOLTAGE;
		times[i][0] = _adc_time_buf[i][0];
		times[i][1] = _adc_time_buf[i][1];
	}
}

float smol_adc_last_voltage(size_t channel) {
	assert(channel < NUM_ADC_CHANNELS);
	uint16_t sample = _adc_read_buf[channel][0];
	return sample * ADC_COUNT_TO_VOLTAGE;
}

float smol_adc_last_voltage2(size_t channel) {
	assert(channel < NUM_ADC_CHANNELS);
	uint16_t sample = _adc_read_buf[channel][1];
	return sample * ADC_COUNT_TO_VOLTAGE;
}

float smol_adc_vbus(void) {
	uint16_t sample = _adc_read_buf[ADC_CHANNEL_VBUS][0];
	return sample * ADC_COUNT_TO_VOLTAGE / ADC_VBUS_DIV;
}

float smol_adc_vdd(void) {
	uint16_t sample = _adc_read_buf[ADC_CHANNEL_VDD][0];
	return sample * ADC_COUNT_TO_VOLTAGE / ADC_VDD_DIV;
}

float smol_chip_temperature_celsius(void) {
	uint16_t sample = _adc_read_buf[ADC_CHANNEL_TEMPERATURE][0];
	return _adc_to_temperature_celsius(sample);
}

