#pragma once

#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>

#include "smol_servo_loop.h"

bool smol_core1_is_ram_only(void);

void smol_usb_pd_init(void);
void smol_usb_pd_run(void);
void smol_usb_pd_run_startup(void);
uint32_t smol_usb_pd_contract_mv(void);

void smol_drv_init(void);
void smol_drv_log_status(void);
bool smol_drv_is_ready(void);
int smol_drv_enable(void);
int smol_drv_set_isense_gain(int32_t gain);
float smol_drv_isense_gain(void);
int smol_drv_set_switching(int32_t bbmclks, int32_t drvstrength);
// break-before-make time of the configuration (BBMCLKS times 42 ns, typ.)
int32_t smol_drv_bbm_ns(void);
void smol_drv_disable(void);
bool smol_drv_is_enabled(void);
void smol_drv_run(void);
void smol_mag_init(void);

void smol_adc_init(void);
void smol_pwm_init(void);
void smol_servo_loop_init(void);

void smol_mag_start(void);
void smol_pwm_start(void);
void smol_adc_start(void);
void smol_servo_loop_start(void);
void smol_servo_signal_loop(void);
// current sense zero calibration, see smol_servo_loop.c
bool smol_servo_loop_isense_zero_is_busy(void);
bool smol_servo_loop_isense_zero(float zero_v[3][2]);
void smol_servo_loop_isense_gain_changed(void);

void smol_servo_loop_idle_core0(void);
void smol_servo_loop_idle_core1(void);

bool smol_pwm_was_started(void);
void smol_pwm_fill(const smol_bridge_out_t* out, size_t starting_phase);
void smol_pwm_all_off(void);

float smol_chip_temperature_celsius(void);
float smol_adc_vbus(void);
float smol_adc_vdd(void);
float smol_adc_last_voltage(size_t channel);
float smol_adc_last_voltage2(size_t channel);
void smol_adc_get_isense_values(float values[3][2], uint32_t times_clocks[3][2]);

void smol_mag_get_reading_values(uint32_t values[2]);
void smol_mag_get_reading_ages_ns(int32_t ages[2]);

float smol_sin(float x);

// debugging / watchdog
uint32_t smol_adc_block_index(void);
