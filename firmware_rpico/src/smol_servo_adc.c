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

#include "smol_servo_parameters.h"
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

#define ADC_PWM_TOP (SMOL_SERVO_ADC_PERIOD_CLOCKS-1)

#define ADC_FIFO_THRESHOLD (4)

static int _adc_cs_dma_channel = -1;

static uint32_t _block_index = 0;
static uint32_t _ringbuf_index = 0;

static uint32_t _adc_cs_buf[ADC_PATTERN_NUM_BLOCKS][ADC_PATTERN_BLOCK_SIZE];

static uint32_t _adc_cs_ringbuf[ADC_RINGBUF_NUM_BLOCKS][ADC_PATTERN_BLOCK_SIZE] __attribute__((aligned(4*ADC_RINGBUF_WORDS)));

static uint32_t _adc_read_buf[NUM_ADC_CHANNELS][2];

uint32_t smol_adc_block_index(void) {
	return _block_index;
}

void smol_adc_fifo_threshold_irq_handler(void) {
	// we expect this to be called every ADC_FIFO_THRESHOLD samples
	// dbg_println("adc!");
	uint32_t block_complete_index = _block_index;
	uint32_t ringbuf_complete_index = _ringbuf_index;
	// Block of ADC reads has completed.
	// 1. Gather ADC reads
	// 2. Refill CS pattern ringbuf

	uint32_t next_block_index = (block_complete_index + 1) % ADC_PATTERN_NUM_BLOCKS;
	uint32_t next_ringbuf_index = (ringbuf_complete_index + 1) % ADC_RINGBUF_NUM_BLOCKS;

	// refill CS pattern. This is a software op because of the power-of-two restriction on DMA ringbuffer sizes, but we have a 3x factor for our 3-phase pattern.
	// we're refilling the block we just executed, so that this isn't timing critical for the next conversion block.
	memcpy(_adc_cs_ringbuf[ringbuf_complete_index], _adc_cs_buf[block_complete_index], sizeof(uint32_t)*ADC_PATTERN_BLOCK_SIZE);

	// update indices for next round.
	_block_index = next_block_index;
	_ringbuf_index = next_ringbuf_index;

	// gather ADC read results from FIFO

	// sanity check that we have exactly 4 samples waiting.
	assert(adc_fifo_get_level() == ADC_FIFO_THRESHOLD);

	// read FIFO into local buffer first...
	uint16_t samples[ADC_FIFO_THRESHOLD] = {0};

	for (size_t i = 0; i < ADC_FIFO_THRESHOLD; ++i) {
		uint16_t val = adc_fifo_get();
		assert(0 == (val & 0x8000)); // assert no error bit set
		samples[i] = val;
	}

	// ...then assign samples to correct ADC channels.
	// samples contain two evenly spaced current reads [0,2] and two aux reads in between [1,3]
	_adc_read_buf[block_complete_index][0] = samples[0];
	_adc_read_buf[block_complete_index][1] = samples[2];
	_adc_read_buf[2*block_complete_index + 3][0] = samples[1];
	_adc_read_buf[2*block_complete_index + 4][0] = samples[3];
}


static uint32_t smol_adc_cs_for_read(uint32_t ch) {
	assert(ch < 9);
	uint32_t ainsel = ch << 12;
	uint32_t err_clear = 1 << 10;
	uint32_t start_once = 1 << 2;
	uint32_t ts_en = 1 < 1;
	uint32_t en = 1 < 0;

	return ainsel | err_clear | start_once | ts_en | en;
}


int smol_adc_cs_buf_init(void) {
	// init interleaved read pattern
	// [0,2] are bridge current sense values on channels 0-2
	// [1,3] are aux reads on channels 3-8
	for (size_t i = 0; i < ADC_PATTERN_NUM_BLOCKS; ++i) {
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

void smol_adc_dma_irq_handler(void) {
	dma_channel_acknowledge_irq0(_adc_cs_dma_channel);
	dbg_println("adc cs dma!");
}

void smol_adc_pwm_irq_handler(void) {
	pwm_clear_irq(PWM_ADC_SLICE);
	dbg_println("adc wrap!");
}


int smol_adc_dma_init(void) {
	_adc_cs_dma_channel = dma_claim_unused_channel(true);

	dma_channel_config_t config = dma_channel_get_default_config(_adc_cs_dma_channel);
	// ringbuf byte count is 4 * 8 = 32, which is 5 bits.
	channel_config_set_ring(&config, true, 5);
	channel_config_set_read_increment(&config, true);
	channel_config_set_write_increment(&config, false);
	channel_config_set_dreq(&config, pwm_get_dreq(PWM_ADC_SLICE));
	channel_config_set_enable(&config, true);

	dma_channel_set_config(_adc_cs_dma_channel, &config, false);

    dma_channel_set_read_addr(_adc_cs_dma_channel, _adc_cs_ringbuf, false);
    dma_channel_set_write_addr(_adc_cs_dma_channel, &adc_hw->cs, false);
    // trigger on the last config (since DREQ is setup, it will wait for DREQ?)
    dma_channel_set_transfer_count(_adc_cs_dma_channel, dma_encode_transfer_count_with_self_trigger(ADC_PATTERN_BLOCK_SIZE), true);

#ifdef DEBUG_DMA_ADC
    dma_channel_set_irq0_enabled(_adc_cs_dma_channel, true);
	irq_set_exclusive_handler(DMA_IRQ_0, smol_adc_dma_irq_handler);
	irq_set_priority(DMA_IRQ_0, 0);
	irq_set_enabled(DMA_IRQ_0, true);
#endif
}

int smol_adc_pwm_init(void) {
	pwm_config config = pwm_get_default_config();
	pwm_config_set_wrap(&config, ADC_PWM_TOP);
	pwm_config_set_phase_correct(&config, false);
	pwm_config_set_clkdiv_mode(&config, PWM_DIV_FREE_RUNNING);
	pwm_init(PWM_ADC_SLICE, &config, false);

#ifdef DEBUG_PWM_ADC
	gpio_set_function(28, GPIO_FUNC_PWM);
	gpio_set_function(29, GPIO_FUNC_PWM);
	pwm_set_both_levels(PWM_ADC_SLICE, ADC_PWM_TOP * 1 / 3, ADC_PWM_TOP * 2 / 3);

	pwm_clear_irq(PWM_ADC_SLICE);
	pwm_set_irq1_enabled(PWM_ADC_SLICE, true);

	irq_set_exclusive_handler(PWM_IRQ_WRAP_1, smol_adc_pwm_irq_handler);
	irq_set_priority(PWM_IRQ_WRAP_1, 0);
	irq_set_enabled(PWM_IRQ_WRAP_1, true);
#endif
}

int smol_adc_init(void) {
	adc_init();

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
	adc_fifo_setup(true, false, ADC_FIFO_THRESHOLD, true, false);
	adc_irq_set_enabled(true);
	irq_set_exclusive_handler(ADC_IRQ, smol_adc_fifo_threshold_irq_handler);

	smol_adc_cs_buf_init();
	smol_adc_pwm_init();
	smol_adc_dma_init();

	// setup trigger PWM

}

void smol_adc_start(void) {
	// PWM has to be running so we can do the counter adjustment.
	assert(smol_pwm_was_started());

	// make ADC reads centered on PWM wrap by retarding the ADC read timer
	// TODO: check if this could be done by initing PWM.CTR with a "negative" value instead.
	for (size_t i = 0; i < SMOL_SERVO_ADC_RETARD_COUNT; ++i) {
		pwm_retard_count(PWM_ADC_SLICE);
	}
}

