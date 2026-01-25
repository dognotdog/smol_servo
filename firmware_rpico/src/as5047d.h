#pragma once

#include "hardware/spi.h"

// Parity for AS5047D
#define _XORS(x,s) ((x) ^ ((x) >> s))

#define  _PAR(x) (((_XORS(_XORS(_XORS(_XORS(((x) & 0x7FFF), 8), 4), 2), 1) & 1) << 15) | ((x) & 0x7FFF))

#define MAG_READ_FLAG    (0x4000)
#define MAG_REG_ERRFL    (0x0001)
#define MAG_REG_DIAAGC   (0x3FFC)
#define MAG_REG_MAG      (0x3FFD)
#define MAG_REG_ANGLEUNC (0x3FFE)
#define MAG_REG_ANGLECOM (0x3FFF)

int as5047d_spi_init(spi_inst_t *spi, int mosi_pin, int nss_pin, int sclk_pin, int miso_pin);
void as5047d_dma_init(void);

int as5047d_detect(void);
