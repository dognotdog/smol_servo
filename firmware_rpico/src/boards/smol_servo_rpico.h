#pragma once

pico_board_cmake_set(PICO_PLATFORM, rp2350)

// --- RP2350 VARIANT ---
#define PICO_RP2350A 0
#define PICO_RP2350B 1

// #define PICO_BOOT_STAGE2_CHOOSE_W25Q080 1

// RP2354 has a W25Q16JV 16Mbit chip in the package
pico_board_cmake_set_default(PICO_FLASH_SIZE_BYTES, (2 * 1024 * 1024))
#ifndef PICO_FLASH_SIZE_BYTES
#define PICO_FLASH_SIZE_BYTES (2 * 1024 * 1024)
#endif

#ifndef PICO_FLASH_SPI_CLKDIV
#define PICO_FLASH_SPI_CLKDIV 2
#endif

// board detect?
#define RASPBERRYPI_SMOL_SERVO_RPICO

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

#ifndef PICO_DEFAULT_UART_TX_PIN
#define PICO_DEFAULT_UART_TX_PIN SERIAL_CONSOLE_TX_PIN
#endif
#ifndef PICO_DEFAULT_UART_RX_PIN
#define PICO_DEFAULT_UART_RX_PIN SERIAL_CONSOLE_RX_PIN
#endif
#define uart_default uart1

// FUSB302 USB PD controller
#define FUSB_I2C i2c0
#define FUSB_I2C_SDA_PIN 0
#define FUSB_I2C_SCL_PIN 1
#define FUSB_INT_PIN     2

// External I2C
#define EXT_I2C i2c1
#define EXT_I2C_SDA_PIN 14 // J9-08
#define EXT_I2C_SCL_PIN 15 // J9-10

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

// AP2020 LED
#define LED_SPI spi1
#define LED_SCLK_PIN 10
#define LED_MOSI_PIN 11


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

#define ADCA_SENSE_PIN 45 // J8-01
#define ADCB_SENSE_PIN 46 // J8-02
#define ADCC_SENSE_PIN 47 // J8-03

// PWM allocation
#define PWMR_SLICE 3
#define PWMA_SLICE 4
#define PWMB_SLICE 5
#define PWM_ADC_SLICE 6
#define PWM_SERVO_LOOP_SLICE 7
#define PWMC_SLICE 8

pico_board_cmake_set_default(PICO_RP2350_A2_SUPPORTED, 1)
#ifndef PICO_RP2350_A2_SUPPORTED
#define PICO_RP2350_A2_SUPPORTED 1
#endif
