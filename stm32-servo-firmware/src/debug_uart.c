
#include <stdio.h>
#include <stddef.h>
#include <string.h>
#include <stdarg.h>

#include "stm32g4xx_hal.h"
#include "stm32g4xx_hal_uart.h"

#include "debug.h"

extern UART_HandleTypeDef huart1;

void uart_send(const void* const data, size_t len)
{
	HAL_UART_Transmit(&huart1, (unsigned char*)data, len, 500);
}

void uart_vprintf(const char* const format, va_list args)
{
    char buf[256] = "";
        
    vsnprintf(buf, sizeof(buf), format, args);
    
    size_t len = strnlen(buf, sizeof(buf));

    uart_send(buf, len);
}

void uart_printf(const char* const format, ...)
{        
    va_list args;
    va_start (args, format);
    uart_vprintf(format, args);
    va_end(args);
}

extern void debug_uart_init(void)
{
    HAL_NVIC_SetPriority(USART1_IRQn, 2, 0);
    HAL_NVIC_EnableIRQ(USART1_IRQn);

}

