/*
 * uart.c
 *
 *  Created on: Jun 18, 2010
 *      Author: Titus
 */

#include "LPC2378.h"
#include "uart.h"

unsigned char uart0_rx_buffer[UART0_RX_BUF_SIZE];
unsigned char uart0_tx_buffer[UART0_TX_BUF_SIZE];
unsigned short uart0_rx_insert, uart0_rx_extract;
unsigned short uart0_tx_insert, uart0_tx_extract;
int uart0_tx_running;

// Setup UART0 for the IMU at 38400 baud on P0.2 and P0.3
void Uart0Init(void)
{
	// 1. Turn on UART0 peripheral
	PCONP |= PCONP_UART0;

	// 2. Select the peripheral clock divider
	//    For baud rate of 38400 use divider of 1 so PCLK = 72MHz
	//    BR = PCLK / (16 * (256*DLM+DLL) * (1 + DIV/MULT))
	//    DLM = 0, DLL = 78, DIV = 1, MULT = 2
	//    So BR = 38461.5
	PCLKSEL0 |= PCLKSEL0_UART0_DIV1;

	// 3. Set the DLAB bit to write to the baud registers
	UART0_LCR = UART_DLAB;
	UART0_DLL = 78;
	UART0_DLM = 0;
	UART0_FDR = (2<<4) | (1<<0);
	UART0_LCR = UART_8BIT;

	// 4. Enable UART FIFO
	UART0_FCR |= UART_FIFO_EN | UART_RX_FIFO_RST | UART_TX_FIFO_RST;

	// 5. Enable Peripheral Pins P0.2(TXD0) & P0.3(RXD0)
	PINSEL0 |= PINSEL0_TXD0 | PINSEL0_RXD0;

	// 6. Enable transmit (no handshaking required)
	UART0_TER |= UART_TXEN;

	// 7. Setup Interrupts (work on later)
	VICIntSelect &= ~VIC_CHAN_NUM_UART0;		// Change to IRQ
	VICIntEnClr |= VIC_CHAN_NUM_UART0; 			// Disable interrupt
	VICVectAddr6 = (unsigned long)Uart0ISR;		// Assign the ISR
	VICVectPriority6 = 0xF;						// Set the priority
	VICIntEnable |= VIC_CHAN_NUM_UART0;			// Enable the interrupt
	UART0_IER |= UART_RXDAIE;					// Enable RX Data interrupt
	uart0_rx_insert = uart0_rx_extract = 0;		// Initialize buffer positions
	uart0_tx_insert = uart0_tx_extract = 0;
	uart0_tx_running = 0;
}

void Uart0TxChar(char data)
{

	UART0_THR = data;				// Load data into transmit buffer
}

void Uart0ISR(void)
{

}

// Setup UART1 for the GPS at 19200 baud on P0.15 and P0.16
void Uart1Init(void)
{
	// 1. Turn on UART1 peripheral
	PCONP |= PCONP_UART1;

	// 2. Select the peripheral clock divider
	//    For baud rate of 19200 use divider of 1 so PCLK = 72MHz
	//    BR = PCLK / (16 * (256*DLM+DLL) * (1 + DIV/MULT))
	//    DLM = 0, DLL = 156, DIV = 1, MULT = 2
	//    So BR = 19230.8
	PCLKSEL0 |= PCLKSEL0_UART1_DIV1;

	// 3. Set the DLAB bit to write to the baud registers
	UART1_LCR |= UART_DLAB;
	UART1_DLL = 156;
	UART1_DLL = 0;
	UART1_FDR = (2<<4) | (1<<0);

	// 4. Enable UART FIFO
	UART1_FCR |= UART_FIFO_EN;

	// 5. Enable Peripheral Pins P0.15(TXD1) & P0.16(RXD1)
	PINSEL0 |= PINSEL0_TXD1;
	PINSEL1 |= PINSEL1_RXD1;

	// 6. Enable transmit (no handshaking required)
	UART1_TER |= UART_TXEN;

	// 7. Setup Interrupts (work on later)
}

void Uart1TxChar(char data)
{
	UART1_LCR &= ~UART_DLAB;		// Clear DLAB bit to write to Transmit holding register
	UART1_THR = data;				// Load data into transmit buffer
}
