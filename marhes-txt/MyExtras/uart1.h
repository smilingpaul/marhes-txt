/*
 * uart.h
 *
 *  Created on: Jun 18, 2010
 *      Author: Titus
 */

#ifndef UART_H_
#define UART_H_

#include "LPC2378.h"

/*************************************************************************
 *             Definitions
 *************************************************************************/
// Peripheral Power
#define PCONP_UART0			(1<<3)
#define PCONP_UART1			(1<<4)
#define PCONP_UART2			(1<<24)
#define PCONP_UART3			(1<<25)
// Peripheral clock divider
#define PCLKSEL0_UART0_DIV1	(1<<6)
#define PCLKSEL0_UART1_DIV1 (1<<8)
// UART PINS
#define PINSEL0_TXD0		(1<<4)
#define PINSEL0_RXD0		(1<<6)
#define PINSEL0_TXD1 		(1<<30)
#define PINSEL1_RXD1		(1<<0)
// Line control register (LCR)
#define UART_DLAB			(1<<7)
#define UART_8BIT			3
// FIFO control register (FCR)
#define UART_FIFO_EN		(1<<0)
#define UART_RX_FIFO_RST	(1<<1)
#define UART_TX_FIFO_RST	(1<<2)
#define UART_RX_TRIG1		(0<<6)
#define UART_RX_TRIG4		(1<<6)
#define UART_RX_TRIG8		(2<<6)
#define UART_RX_TRIG14		(3<<6)
// Transmit enable register (TER)
#define UART_TXEN			(1<<7)
// Interrupt Enable Register
#define UART_RXDAIE			(1<<0)
#define UART_THREIE			(1<<1)
#define UART_RXLSIE			(1<<2)

// TX and RX Buffers
#define UART0_TX_BUF_SIZE	128
#define UART0_RX_BUF_SIZE	256
#define UART1_TX_BUF_SIZE	128
#define UART1_RX_BUF_SIZE	256

/*************************************************************************
 *             Function declarations
 *************************************************************************/

void Uart0Init(void);
void Uart0TxChar(char data);
void Uart1Init(void);
void Uart1TxChar(char data);


#endif /* UART_H_ */
