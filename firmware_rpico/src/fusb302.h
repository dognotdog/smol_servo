#pragma once

#include <stdint.h>
#include <stdbool.h>

#include "pico/stdlib.h"
#include "hardware/i2c.h"


// 7-bit address
#define FUSB302_I2C_ADDR 0x22

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
    FUSB_REG_INTERRUPTA,
    FUSB_REG_INTERRUPTB,
    FUSB_REG_STATUS0,
    FUSB_REG_STATUS1,
    FUSB_REG_INTERRUPT,
    FUSB_REG_FIFO,
} fusb302_reg_addr_t;


int fusb302_init(i2c_inst_t* i2c);

int fusb302_write_reg(fusb302_reg_addr_t reg, uint8_t value);
int fusb302_read_reg(fusb302_reg_addr_t reg, uint8_t* value);
