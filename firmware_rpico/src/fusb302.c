/**
 * FUSB302 Driver
 */

#include "fusb302.h"

#include <stdbool.h>
#include <stdio.h>

#include "pico_debug.h"


static i2c_inst_t* m_fusb_i2c_bus;

int fusb302_init(i2c_inst_t* i2c) {
    dbg_println("fusb302_init()...");
    m_fusb_i2c_bus = i2c;

    uint8_t device_id = 0;
    int result = fusb302_read_reg(FUSB_REG_DEVICE_ID, &device_id);

    if (!result) {
        err_println("FUSB302 device_id read failed.");
        return result;
    }

    info_println("FUSB302 device_id 0x%02X", device_id);

    return 0;
}

int fusb302_write_reg(fusb302_reg_addr_t reg, uint8_t value) {
    absolute_time_t timeout = make_timeout_time_ms(100);
    uint8_t bytes[2] = {reg, value};
    int result = i2c_write_blocking_until(m_fusb_i2c_bus, FUSB302_I2C_ADDR, bytes, 2, false, timeout);
    if (result != 2)
        err_println("fusb302_write_reg failed: i2c_write_blocking_until()=%i.", result);
    return result;
}
int fusb302_read_reg(fusb302_reg_addr_t reg, uint8_t* value) {
    absolute_time_t timeout = make_timeout_time_ms(100);
    uint8_t reg_byte = reg;
    int result = i2c_write_blocking_until(m_fusb_i2c_bus, FUSB302_I2C_ADDR, &reg_byte, 1, true, timeout);
    if (result != 1) {
        err_println("fusb302_read_reg failed: i2c_write_blocking_until()=%i.\n", result);
        return result;
    }
    return i2c_read_blocking_until(m_fusb_i2c_bus, FUSB302_I2C_ADDR, value, 1, false, timeout);
}
