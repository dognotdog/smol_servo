#pragma once

#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>

void smol_usb_pd_init(void);
void smol_usb_pd_run(void);

void smol_drv_init(void);

void smol_adc_init(void);
void smol_pwm_init(void);
void smol_servo_loop_init(void);

void smol_pwm_start(void);
void smol_adc_start(void);
void smol_servo_loop_start(void);
void smol_servo_signal_loop(void);

void smol_servo_loop_idle_core0(void);
void smol_servo_loop_idle_core1(void);

bool smol_pwm_was_started(void);

float smol_chip_temperature_celsius(void);
float smol_adc_vbus(void);
float smol_adc_vdd(void);
float smol_adc_last_voltage(size_t channel);
float smol_adc_last_voltage2(size_t channel);

// debugging / watchdog
uint32_t smol_adc_block_index(void);
