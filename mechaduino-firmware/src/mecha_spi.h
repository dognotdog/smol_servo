#ifndef MECHA_SPI_H
#define MECHA_SPI_H

#include <stdint.h>

typedef struct {
	// uint16_t NOP;
	uint16_t ERRFL;
	uint16_t DIAAGC;
	uint16_t ANGLEUNC;
} mspi_as5047_state_t;


void mspi_init(void);

mspi_as5047_state_t mspi_readSensor(void);


#endif // MECHA_SPI_H
