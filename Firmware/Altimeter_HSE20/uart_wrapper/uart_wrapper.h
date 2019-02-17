/*
 * xprintf_config.h
 *
 *  Created on: 2018/02/22
 *      Author: feunoir
 */

#ifndef UART_WRAPPER_H_
#define UART_WRAPPER_H_

#include "stm32l4xx_hal.h"
#include "xprintf.h"

void uart_init(UART_HandleTypeDef* huart);
uint8_t uart_getc(void);
void uart_putc(uint8_t c);
void uart_puts(char *str);

#endif /* UART_WRAPPER_H_ */
