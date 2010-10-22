/*
 * uart0.h
 *
 *  Created on: Jun 18, 2010
 *      Author: Titus
 */

#ifndef UART0_H_
#define UART0_H_

#include "app_types.h"
#include "armVIC.h"
#include "LPC23xx.h"

/*************************************************************************
 *             Definitions
 *************************************************************************/
// Peripheral Power
#define PCONP_UART0			(1<<3)
// Peripheral clock divider
#define PCLKSEL0_UART0_DIV1	(1<<6)
// UART PINS
#define PINSEL0_TXD0		(1<<4)
#define PINSEL0_RXD0		(1<<6)
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
// Interrupt Identification Register (IIR)
#define UART_NO_INT			(1<<0)
#define UART_THRE_INT		(1<<1)
#define UART_RXDA_INT		(1<<2)
#define UART_RXLS_INT		(3<<1)
#define UART_CTOI_INT		(3<<2)
#define UART_INT_MASK 		0x0E
// Line Status Register (LSR)
#define UART_RDR			(1<<0)
#define UART_THRE			(1<<5)
#define UART_TEMT			(1<<6)

// FIFO Sizes
#define UART_TX_FIFO		16

// TX and RX Buffers
#define UART0_TX_BUF_SIZE	256
#define UART0_RX_BUF_SIZE	256

// Largest size Rx string to return
#define UART0_RX_STR_SIZE	32

/*************************************************************************
 *             Function declarations
 *************************************************************************/

void Uart0Init(void);
void Uart0TxString(const char *data);
int Uart0TxBufUsed(void);
int Uart0TxBufSpace(void);
char Uart0RxChar(void);
char* Uart0RxString(char* dest);
uint8_t Uart0RxDataReady(void);
int Uart0RxBufUsed(void);
int Uart0RxBufSpace(void);
void Uart0ISR(void) __attribute__((naked));

#endif /* UART0_H_ */
