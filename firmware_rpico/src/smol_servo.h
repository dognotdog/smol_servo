#pragma once

#include <stdbool.h>
#include <stdint.h>

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


// debugging / watchdog
uint32_t smol_adc_block_index(void);