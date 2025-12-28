#pragma once

pico_board_cmake_set(PICO_PLATFORM, rp2350)

// --- RP2350 VARIANT ---
#define PICO_RP2350B 1

// #include "hardware/spi.h"

/**
 * Board config for the `smol-servo-rp-nodrv` and `smol-servo-nomcu-tmc6200` combo.
 * 
 * .1in pin headers on MCU board:
 * 
 *      J5           J7           J6        J8
 * .---------.  .---------.  .---------.  .----.
 * | 01 | 02 |  | 01 | 02 |  | 01 | 02 |  | 01 |
 * | 03 | 04 |  | 03 | 04 |  | 03 | 04 |  | 02 |
 *  ...
 * | 09 | 10 |  | 09 | 10 |  | 09 | 10 |  | 05 |
 * '---------'  '---------'  '---------'  '----'
 * 
 *  J2                  J9                  J4
 * .----------------.  .----------------.  .--------------------.
 * | 02 | 04 ... 10 |  | 02 | 04 ... 10 |  | GND  | GND  | GND  |
 * | 01 | 03 ... 09 |  | 01 | 03 ... 09 |  | VBUS | VBUS | VBUS |
 * '----------------'  '----------------'  '--------------------'
 * 
 * Comments below map header pins to GPIO
 * 
 */

// UART console
#define SERIAL_CONSOLE_TX_PIN 6 // J9-04
#define SERIAL_CONSOLE_RX_PIN 7 // J9-06

// FUSB302 USB PD controller
#define FUSB_I2C i2c0
#define FUSB_I2C_SDA_PIN 0
#define FUSB_I2C_SCL_PIN 1
#define FUSB_INT_PIN     2

// TMC6200 SPI
#define DRV_SPI spi1
#define DRV_MISO_PIN 28 // J7-03
#define DRV_NSS_PIN  29 // J7-05
#define DRV_SLCK_PIN 30 // J7-07
#define DRV_MOSI_PIN 31 // J7-09

#define DRVEN   35 // J6-09
#define DRFAULT 34 // J6-10

#define MAG_SPI spi0
#define MAG_MISO_PIN 36 // J7-10
#define MAG_NSS_PIN  37 // J7-04
#define MAG_SLCK_PIN 38 // J7-06
#define MAG_MOSI_PIN 39 // J7-08


// 3ph bridge driver
#define PWMR_PIN 22 // J5-08
#define PWMA_PIN 24 // J5-03
#define ENA_PIN  25 // J5-05
#define PWMB_PIN 26 // J5-07
#define ENB_PIN  27 // J5-09
#define PWMC_PIN 32 // J5-04
#define ENC_PIN  33 // J5-06

#define SOA_SENSE_PIN 40 // J6-03
#define SOB_SENSE_PIN 41 // J6-05
#define SOC_SENSE_PIN 42 // J6-07


#define VBUS_SENSE_PIN 43 // J8-04
#define VDD_SENSE_PIN  44 // J8-05

pico_board_cmake_set_default(PICO_RP2350_A2_SUPPORTED, 1)
#ifndef PICO_RP2350_A2_SUPPORTED
#define PICO_RP2350_A2_SUPPORTED 1
#endif
