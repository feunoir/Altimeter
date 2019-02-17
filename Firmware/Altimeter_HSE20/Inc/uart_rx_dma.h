/*
 * UARTRx_DMA.h
 *
 *  Created on: 2017/12/06
 *      Author: feunoir
 */

#ifndef UART_RX_DMA_H_
#define UART_RX_DMA_H_

#include "main.h"
#include "stm32l4xx_hal.h"
#include "stdbool.h"

/*
* The STM32 makes receiving chars into a large circular buffer simple
* and requires no CPU time. The UART receiver DMA must be setup as CIRCULAR.
*/
#define CIRC_BUF_SZ       512  /* must be power of two */
#define DMA_WRITE_PTR ( (CIRC_BUF_SZ - huart_cobs->hdmarx->Instance->CNDTR) & (CIRC_BUF_SZ - 1) )


void msgrx_init(UART_HandleTypeDef *huart);
bool msgrx_circ_buf_is_empty(void);
uint8_t msgrx_circ_buf_get(void);


#endif /* UART_RX_DMA_H_ */
