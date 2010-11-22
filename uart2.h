/*
 * uart2.h
 *
 *  Created on: Jun 18, 2010
 *      Author: Titus
 */

#ifndef UART2_H_
#define UART2_H_

#include "app_types.h"
#include "armVIC.h"
#include "LPC23xx.h"

/*************************************************************************
 *             Definitions
 *************************************************************************/
// Peripheral Power
#define PCONP_UART2			(1<<24)
// Peripheral clock divider
#define PCLKSEL1_UART2_DIV1	(1<<16)
// UART PINS
#define PINSEL0_TXD2		(1<<20)
#define PINSEL0_RXD2		(1<<22)
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
#define UART2_TX_BUF_SIZE	128
#define UART2_RX_BUF_SIZE	256

// Largest size Rx string to return
#define UART2_RX_STR_SIZE	32

/*************************************************************************
 *             Function declarations
 *************************************************************************/

void Uart2Init(void);
void Uart2TxString(const char *data);
void Uart2TxArr(uint8_t *data, uint8_t numBytes);
int Uart2TxBufUsed(void);
int Uart2TxBufSpace(void);
char Uart2RxChar(void);
char* Uart2RxString(char* dest);
uint8_t Uart2RxDataReady(void);
int Uart2RxBufUsed(void);
int Uart2RxBufSpace(void);
void Uart2ISR(void) __attribute__((naked));

#endif /* UART2_H_ */
