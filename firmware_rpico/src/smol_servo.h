#pragma once

#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>

void smol_usb_pd_init(void);
void smol_usb_pd_run(void);

void smol_drv_init(void);
void smol_mag_init(void);

void smol_adc_init(void);
void smol_pwm_init(void);
void smol_servo_loop_init(void);

void smol_mag_start(void);
void smol_pwm_start(void);
void smol_adc_start(void);
void smol_servo_loop_start(void);
void smol_servo_signal_loop(void);

void smol_servo_loop_idle_core0(void);
void smol_servo_loop_idle_core1(void);

bool smol_pwm_was_started(void);
void smol_pwm_fill(uint32_t pwm_values[3][3], size_t starting_phase);

float smol_chip_temperature_celsius(void);
float smol_adc_vbus(void);
float smol_adc_vdd(void);
float smol_adc_last_voltage(size_t channel);
float smol_adc_last_voltage2(size_t channel);
void smol_adc_get_isense_values(float values[3][2], uint32_t times_clocks[3][2]);

void smol_mag_get_reading_values(uint32_t values[2]);
void smol_mag_get_reading_ages_ns(int32_t ages[2]);

// debugging / watchdog
uint32_t smol_adc_block_index(void);
