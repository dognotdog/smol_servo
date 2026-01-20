/**
 * FUSB302 Driver
 */

#include "fusb302.h"

#include <stdbool.h>
#include <stdio.h>

#include "pico_debug.h"

static i2c_inst_t* m_fusb_i2c_bus;
static int _irq_gpio = -1;

int fusb302_chip_init(void) {

    uint8_t device_id = 0;
    int result = fusb302_read_reg(FUSB_REG_DEVICE_ID, &device_id);

    if (result != 0) {
        err_println("FUSB302 device_id read failed.");
        return result;
    }

    info_println("FUSB302 device_id 0x%02X", device_id);

    result = fusb302_software_reset();
    if (result != 0) {
        err_println("FUSB302 RESET failed.");
        return result;
    }

    // power on
    result = fusb302_write_reg(FUSB_REG_POWER, 0x0F);
    if (result != 0) {
        err_println("FUSB302 POWER set failed.");
        return result;
    }

    uint8_t status0 = 0;
    result = fusb302_read_reg(FUSB_REG_STATUS0, &status0);

    if (result != 0) {
        err_println("FUSB302 STATUS0 read failed.");
        return result;
    }

    bool vbus_ok = 0 != (status0 & FUSB_STATUS0_VBUSOK_MASK);
    dbg_println("FUSB VBUSOK %u", vbus_ok);

    // enable host interrupts
    uint8_t control0 = 0;
    result = fusb302_read_reg(FUSB_REG_CONTROL0, &control0);
    if (result != 0) {
        err_println("FUSB302 CONTROL0 read failed.");
        return result;
    }
    control0 &= ~FUSB_CONTROL0_INT_MASK_MASK;
    result = fusb302_write_reg(FUSB_REG_CONTROL0, control0);
    if (result != 0) {
        err_println("FUSB302 CONTROL0 write failed.");
        return result;
    }

    // enable auto retries
    uint8_t control3 = 0;
    result = fusb302_read_reg(FUSB_REG_CONTROL3, &control3);
    if (result != 0) {
        err_println("FUSB302 CONTROL3 read failed.");
        return result;
    }
    control3 &= ~FUSB_CONTROL3_N_RETRIES_MASK;
    control3 |= 3 << FUSB_CONTROL3_N_RETRIES_LSB;
    control3 |= FUSB_CONTROL3_AUTO_RETRY_MASK;
    result = fusb302_write_reg(FUSB_REG_CONTROL3, control3);
    if (result != 0) {
        err_println("FUSB302 CONTROL3 write failed.");
        return result;
    }

    return result;
}

int fusb302_init(i2c_inst_t* i2c, int irq_gpio) {
    dbg_println("fusb302_init()...");
    m_fusb_i2c_bus = i2c;
    _irq_gpio = irq_gpio;

    if (_irq_gpio != -1) {
        gpio_init(_irq_gpio);
        gpio_pull_up(_irq_gpio);
    }

    int result = fusb302_chip_init();

    return result;
}

bool fusb302_have_irq(void) {
    // active low interrupt
    if (_irq_gpio != -1)
        return !gpio_get(_irq_gpio);
    else
        return 0;
}

int fusb302_write_reg(fusb302_reg_addr_t reg, uint8_t value) {
    absolute_time_t timeout = make_timeout_time_ms(100);
    uint8_t bytes[2] = {reg, value};
    int result = i2c_write_blocking_until(m_fusb_i2c_bus, FUSB302_I2C_ADDR, bytes, 2, false, timeout);
    if (result != 2) {
        err_println("fusb302_write_reg failed: i2c_write_blocking_until()=%i.", result);
        return result;
    }
    return 0;
}
int fusb302_read_reg(fusb302_reg_addr_t reg, uint8_t* value) {
    absolute_time_t timeout = make_timeout_time_ms(100);
    uint8_t reg_byte = reg;
    int result = i2c_write_blocking_until(m_fusb_i2c_bus, FUSB302_I2C_ADDR, &reg_byte, 1, true, timeout);
    if (result != 1) {
        err_println("fusb302_read_reg failed: i2c_write_blocking_until()=%i.\n", result);
        return result;
    }
    result = i2c_read_blocking_until(m_fusb_i2c_bus, FUSB302_I2C_ADDR, value, 1, false, timeout);
    if (result != 1) {
        err_println("fusb302_read_reg failed: i2c_read_blocking_until()=%i.\n", result);
        return result;
    }
    return 0;
}

int fusb302_check_cc1(uint8_t* bc_lvl) {
    // set to measure CC1
    uint8_t switches0 = 0;
    assert(0 == fusb302_read_reg(FUSB_REG_SWITCHES0, &switches0));

    switches0 &= ~(FUSB_SWITCHES0_MEAS_CC2_MASK);
    switches0 |= (FUSB_SWITCHES0_MEAS_CC1_MASK);
    assert(0 == fusb302_write_reg(FUSB_REG_SWITCHES0, switches0));

    uint8_t status0 = 0;
    assert(0 == fusb302_read_reg(FUSB_REG_STATUS0, &status0));
    *bc_lvl = status0 & 0x03;

    return 0;
}

int fusb302_check_cc2(uint8_t* bc_lvl) {
    // set to measure CC1
    uint8_t switches0 = 0;
    assert(0 == fusb302_read_reg(FUSB_REG_SWITCHES0, &switches0));

    switches0 &= ~(FUSB_SWITCHES0_MEAS_CC1_MASK);
    switches0 |= (FUSB_SWITCHES0_MEAS_CC2_MASK);
    assert(0 == fusb302_write_reg(FUSB_REG_SWITCHES0, switches0));

    uint8_t status0 = 0;
    assert(0 == fusb302_read_reg(FUSB_REG_STATUS0, &status0));
    *bc_lvl = status0 & 0x03;

    return 0;
}


int fusb302_measure_cc(float* cc1, float* cc2) {

    // disable VBUS measurement
    uint8_t measure = 0;
    assert(0 == fusb302_read_reg(FUSB_REG_MEASURE, &measure));
    measure &= ~FUSB_MEASURE_MEAS_VBUS_MASK;
    assert(0 == fusb302_write_reg(FUSB_REG_MEASURE, measure));

    // set to measure CC1
    uint8_t switches0 = 0;
    assert(0 == fusb302_read_reg(FUSB_REG_SWITCHES0, &switches0));
    switches0 &= ~(FUSB_SWITCHES0_MEAS_CC2_MASK);
    switches0 |= (FUSB_SWITCHES0_MEAS_CC1_MASK);
    assert(0 == fusb302_write_reg(FUSB_REG_SWITCHES0, switches0));


    sleep_us(250);
    assert(0 == fusb302_read_reg(FUSB_REG_MEASURE, &measure));
    uint8_t mdac1 = measure & FUSB_MEASURE_MDAC_MASK;
    *cc1 = 0.042f + mdac1 * 0.042f;

    // measure CC2
    switches0 &= ~(FUSB_SWITCHES0_MEAS_CC1_MASK);
    switches0 |= (FUSB_SWITCHES0_MEAS_CC2_MASK);
    assert(0 == fusb302_write_reg(FUSB_REG_SWITCHES0, switches0));
    sleep_us(250);
    assert(0 == fusb302_read_reg(FUSB_REG_MEASURE, &measure));
    uint8_t mdac2 = measure & FUSB_MEASURE_MDAC_MASK;
    *cc2 = 0.042f + mdac2 * 0.042f;

    return 0;
}

int fusb302_measure_vbus(float vbus_threshold, bool* input_is_higher) {
    // disable MEAS_CCx to do VBUS measurement
    uint8_t switches0 = 0;
    assert(0 == fusb302_read_reg(FUSB_REG_SWITCHES0, &switches0));

    uint32_t initial_cc_state = switches0 & (FUSB_SWITCHES0_MEAS_CC1_MASK | FUSB_SWITCHES0_MEAS_CC2_MASK);

    switches0 &= ~(FUSB_SWITCHES0_MEAS_CC1_MASK | FUSB_SWITCHES0_MEAS_CC2_MASK);
    assert(0 == fusb302_write_reg(FUSB_REG_SWITCHES0, switches0));

    uint8_t measure = 0;
    assert(0 == fusb302_read_reg(FUSB_REG_MEASURE, &measure));

    uint8_t measure_initial = measure;

    // enable VBUS measurement
    measure |= FUSB_MEASURE_MEAS_VBUS_MASK;
    measure &= ~FUSB_MEASURE_MDAC_MASK;
    // TODO: initial PCB's VBUS_LV didn't work right.
    uint8_t threshold_int = vbus_threshold / 0.42f - 1;
    measure |= threshold_int;
    assert(0 == fusb302_write_reg(FUSB_REG_MEASURE, measure));
    dbg_println("MEASURE:MDAC = %u", threshold_int);

    // read result
    sleep_us(250);
    uint8_t status0 = 0;
    assert(0 == fusb302_read_reg(FUSB_REG_STATUS0, &status0));

    bool comp = (0 != (status0 & FUSB_STATUS0_COMP_MASK));
    dbg_println("STATUS0:COMP = %u", comp);

    *input_is_higher = comp;

    // disable measurement
    measure = measure_initial;
    assert(0 == fusb302_write_reg(FUSB_REG_MEASURE, measure));

    switches0 |= initial_cc_state;
    assert(0 == fusb302_write_reg(FUSB_REG_SWITCHES0, switches0));

    return 0;
}

bool fusb302_have_rx(void) {
    uint8_t status1 = 0;
    assert(0 == fusb302_read_reg(FUSB_REG_STATUS1, &status1));

    return 0 == (status1 & FUSB_STATUS1_RX_EMPTY_MASK);
}

int fusb302_rx(void* rx_data, size_t rx_len) {
    for (size_t i = 0; i < rx_len; ++i) {
        uint8_t fifo = 0;
        int result = fusb302_read_reg(FUSB_REG_FIFO, &fifo);
        if (result != 0)
            return result;
        ((uint8_t*)rx_data)[i] = fifo;
    }
    return 0;
}

int fusb302_tx(const void* tx_data, size_t tx_len) {
    const uint8_t* tx_bytes = tx_data;
    for (size_t i = 0; i < tx_len; ++i) {
        uint8_t fifo = 0;
        int result = fusb302_write_reg(FUSB_REG_FIFO, tx_bytes[i]);
        if (result != 0)
            return result;
    }
    return 0;
}

int fusb302_software_reset(void) {
    return fusb302_write_reg(FUSB_REG_RESET, 0x01);
}

int fusb302_hard_reset(void) {
    int result = fusb302_software_reset();
    result = fusb302_chip_init();
    return result;
}


