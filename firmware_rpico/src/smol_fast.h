#pragma once

/**
 * RAM resident functions / macros
 */

#include "hardware/timer.h"
#include "hardware/pwm.h"
#include "hardware/sync.h"
#include "pico/platform.h"

#define smol_time64(timer)           \
	({                               \
		uint32_t irq = save_and_disable_interrupts(); \
	    uint32_t lo = timer->timelr; \
	    uint32_t hi = timer->timehr; \
	    restore_interrupts(irq); \
		uint64_t result = ((uint64_t) hi << 32u) | lo; \
		result; \
	})

#define smol_time32(timer) \
	({                     \
	    timer->timerawl;   \
	})


#define gpio_out_put_fast(pin, val) \
	({ pico_default_asm_volatile ("mcrr p0, #4, %0, %1, c0" : : "r" (pin), "r" (val)); })

static uint32_t __not_in_flash("smol_fast") _pwm_c0_cc_b_lsb = PWM_CH0_CC_B_LSB;
static uint32_t __not_in_flash("smol_fast") _pwm_c0_cc_a_lsb = PWM_CH0_CC_A_LSB;

#define pwm_set_both_levels_fast(slice_num, level_a, level_b) \
	({ pwm_hw->slice[slice_num].cc = (((uint)level_b) << _pwm_c0_cc_b_lsb) | (((uint)level_a) << _pwm_c0_cc_a_lsb); })

#define pwm_clear_irq_fast(slice_num) \
	({ pwm_hw->intr = 1u << slice_num; })

// static inline uint64_t __not_in_flash_func(smol_time64)(timer_hw_t *timer) {
//     // use lo,hi order for latched read
//     uint32_t lo = timer->timelr;
//     uint32_t hi = timer->timehr;
//     return ((uint64_t) hi << 32u) | lo;
// }
