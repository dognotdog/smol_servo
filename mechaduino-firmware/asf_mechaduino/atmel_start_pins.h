/*
 * Code generated from Atmel Start.
 *
 * This file will be overwritten when reconfiguring your Atmel Start project.
 * Please copy examples or other code you want to keep to a separate file
 * to avoid losing it when reconfiguring.
 */
#ifndef ATMEL_START_PINS_H_INCLUDED
#define ATMEL_START_PINS_H_INCLUDED

#include <hal_gpio.h>

// SAMD21 has 8 pin functions

#define GPIO_PIN_FUNCTION_A 0
#define GPIO_PIN_FUNCTION_B 1
#define GPIO_PIN_FUNCTION_C 2
#define GPIO_PIN_FUNCTION_D 3
#define GPIO_PIN_FUNCTION_E 4
#define GPIO_PIN_FUNCTION_F 5
#define GPIO_PIN_FUNCTION_G 6
#define GPIO_PIN_FUNCTION_H 7

#define PA02 GPIO(GPIO_PORTA, 2)
#define PA04 GPIO(GPIO_PORTA, 4)
#define PA05 GPIO(GPIO_PORTA, 5)
#define MOT_IN1 GPIO(GPIO_PORTA, 6)
#define VREF1 GPIO(GPIO_PORTA, 7)
#define VREF2 GPIO(GPIO_PORTA, 8)
#define PA12 GPIO(GPIO_PORTA, 12)
#define MOT_IN3 GPIO(GPIO_PORTA, 15)
#define MECH_LED GPIO(GPIO_PORTA, 17)
#define MOT_IN4 GPIO(GPIO_PORTA, 20)
#define MOT_IN2 GPIO(GPIO_PORTA, 21)
#define PA22 GPIO(GPIO_PORTA, 22)
#define PA23 GPIO(GPIO_PORTA, 23)
#define USBN GPIO(GPIO_PORTA, 24)
#define USBP GPIO(GPIO_PORTA, 25)
#define TX_LED GPIO(GPIO_PORTA, 27)
#define RX_LED GPIO(GPIO_PORTB, 3)
#define PB08 GPIO(GPIO_PORTB, 8)
#define HEADER_SS GPIO(GPIO_PORTB, 9)
#define PB10 GPIO(GPIO_PORTB, 10)
#define PB11 GPIO(GPIO_PORTB, 11)
#define TXD GPIO(GPIO_PORTB, 22)
#define RXD GPIO(GPIO_PORTB, 23)

#endif // ATMEL_START_PINS_H_INCLUDED
