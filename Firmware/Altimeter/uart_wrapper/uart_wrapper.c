/*
 * uart_wrapper.c
 *
 *  Created on: 2018/02/22
 *      Author: feunoir
 */

#include "uart_wrapper.h"

static UART_HandleTypeDef *huart_cobs;

void uart_init(UART_HandleTypeDef* huart){
	huart_cobs = huart;
	xdev_out(uart_putc);
	xdev_in(uart_getc);
}

uint8_t uart_getc(void)
{
	uint8_t c = 0;
	char buf[1];
	HAL_UART_Receive(huart_cobs, (uint8_t *)buf, sizeof(buf), 0xFFFF);
	c = buf[0];
	return c;
}
void uart_putc(uint8_t c)
{
	char buf[1];
	buf[0] = c;
	HAL_UART_Transmit(huart_cobs, (uint8_t *)buf, sizeof(buf), 0xFFFF);
}
void uart_puts(char *str)
{
	while (*str) {
		uart_putc(*str++);
	}
}
