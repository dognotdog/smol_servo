#pragma once

#include <stdint.h>
#include <stdbool.h>

#include "pico/stdlib.h"
#include "hardware/i2c.h"


// 7-bit address
#define FUSB302_I2C_ADDR 0x22

#define FUSB_SWITCHES0_MEAS_CC_LSB (2)
#define FUSB_SWITCHES0_MEAS_CC1_MASK 0x04
#define FUSB_SWITCHES0_MEAS_CC2_MASK 0x08

#define FUSB_MEASURE_MEAS_VBUS_MASK 0x40
#define FUSB_MEASURE_MDAC_MASK      0x3f

#define FUSB_STATUS1_RX_EMPTY_MASK 0x20

#define FUSB_STATUS0_VBUSOK_MASK (0x80)
#define FUSB_STATUS0_COMP_MASK   (0x20)
#define FUSB_CONTROL0_INT_MASK_MASK (0x20)

#define FUSB_CONTROL3_AUTO_RETRY_MASK 0x01
#define FUSB_CONTROL3_N_RETRIES_MASK 0x06
#define FUSB_CONTROL3_N_RETRIES_LSB (1)

#define FUSB_SWITCHES1_AUTO_CRC_MASK 0x04

#define FUSB_RESET_PD_MASK 0x02

#define FUSB_FIFO_TX_ON 0xA1
#define FUSB_FIFO_TX_SYNC1 0x12
#define FUSB_FIFO_TX_SYNC2 0x13
#define FUSB_FIFO_TX_SYNC3 0x1B
#define FUSB_FIFO_TX_PACKSYM 0x80
#define FUSB_FIFO_TX_JAM_CRC 0xFF
#define FUSB_FIFO_TX_EOP 0x14
#define FUSB_FIFO_TX_OFF 0xFE

typedef enum {
    FUSB_REG_DEVICE_ID = 0x01,
    FUSB_REG_SWITCHES0,
    FUSB_REG_SWITCHES1,
    FUSB_REG_MEASURE,
    FUSB_REG_SLICE,
    FUSB_REG_CONTROL0,
    FUSB_REG_CONTROL1,
    FUSB_REG_CONTROL2,
    FUSB_REG_CONTROL3,
    FUSB_REG_MASK1,
    FUSB_REG_POWER,
    FUSB_REG_RESET,
    FUSB_REG_OCP,
    FUSB_REG_MASKA,
    FUSB_REG_MASKB,
    FUSB_REG_CONTROL4,
    FUSB_REG_STATUS0A = 0x3C,
    FUSB_REG_STATUS1A,
    FUSB_REG_INTERRUPTA = 0x3E,
    FUSB_REG_INTERRUPTB = 0x3F,
    FUSB_REG_STATUS0,
    FUSB_REG_STATUS1,
    FUSB_REG_INTERRUPT = 0x42,
    FUSB_REG_FIFO,
} fusb302_reg_addr_t;


int fusb302_init(i2c_inst_t* i2c, int irq_pin);

int fusb302_write_reg(fusb302_reg_addr_t reg, uint8_t value);
int fusb302_read_reg(fusb302_reg_addr_t reg, uint8_t* value);

int fusb302_rx(void* rx_data, size_t rx_len);
int fusb302_tx(const void* tx_data, size_t tx_len);
int fusb302_software_reset(void);
int fusb302_hard_reset(void);


int fusb302_measure_vbus(float vbus_threshold, bool* input_is_higher);
int fusb302_check_cc1(uint8_t* bc_lvl);
int fusb302_check_cc2(uint8_t* bc_lvl);
int fusb302_measure_cc(float* cc1, float* cc2);
bool fusb302_have_irq(void);
bool fusb302_have_rx(void);

