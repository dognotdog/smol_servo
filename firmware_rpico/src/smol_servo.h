#pragma once


#include "hardware/timer.h"
#include "pico/platform.h"

#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>

void smol_adc_init();
void smol_pwm_init();
void smol_servo_loop_init(void);

void smol_pwm_start();
void smol_adc_start();
void smol_servo_loop_start(void);

void smol_bridge_pwm_wrap_irq_handler(void);
void smol_adc_fifo_threshold_irq_handler(void);

void smol_servo_loop_idle(void);

bool smol_pwm_was_started(void);

float smol_chip_temperature_celsius(void);
float smol_adc_vbus(void);
float smol_adc_vdd(void);
float smol_adc_last_voltage(size_t channel);
float smol_adc_last_voltage2(size_t channel);

// debugging / watchdog
uint32_t smol_adc_block_index(void);

#define smol_time64(timer)           \
	({                               \
	    uint32_t lo = timer->timelr; \
	    uint32_t hi = timer->timehr; \
		uint64_t result =((uint64_t) hi << 32u) | lo; \
		result; \
	})


#define gpio_out_put_fast(pin, val) \
	({ pico_default_asm_volatile ("mcrr p0, #4, %0, %1, c0" : : "r" (pin), "r" (val)); })

#define pwm_set_both_levels_fast(slice_num, level_a, level_b) \
	({ pwm_hw->slice[slice_num].cc = (((uint)level_b) << PWM_CH0_CC_B_LSB) | (((uint)level_a) << PWM_CH0_CC_A_LSB); })

// static inline uint64_t __not_in_flash_func(smol_time64)(timer_hw_t *timer) {
//     // use lo,hi order for latched read
//     uint32_t lo = timer->timelr;
//     uint32_t hi = timer->timehr;
//     return ((uint64_t) hi << 32u) | lo;
// }
