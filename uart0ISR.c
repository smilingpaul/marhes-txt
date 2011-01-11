/*
 * uart0ISR.c
 *
 *  Created on: Jul 5, 2010
 *      Author: Titus
 */

#include "uart0.h"

extern uint8_t uart0_rx_buffer[UART0_RX_BUF_SIZE];
extern uint8_t uart0_tx_buffer[UART0_TX_BUF_SIZE];
extern uint16_t uart0_rx_insert, uart0_rx_extract;
extern uint16_t uart0_tx_insert, uart0_tx_extract;
extern int uart0_tx_running;

void Uart0ISR(void)
{
	uint8_t int_id;
	int count = 0;

//	ISR_ENTRY();

	// Take care of all interrupts
	while (((int_id = U0IIR) & UART_NO_INT) == 0)
	{
		switch(int_id & UART_INT_MASK)
		{
			case UART_CTOI_INT:
			case UART_RXDA_INT:
				do {
					uint16_t tempCounter;
					uart0_rx_buffer[uart0_rx_insert] = U0RBR;
					tempCounter = (uart0_rx_insert + 1) % UART0_RX_BUF_SIZE;

					if (tempCounter != uart0_rx_extract)
						uart0_rx_insert = tempCounter;
				} while (U0LSR & UART_RDR);
				break;
			case UART_THRE_INT:
				// If there is more data in the buffer to write
				if(uart0_tx_insert != uart0_tx_extract)
				{
					// While there is more data, write up to 16 bytes in the TX FIFO
					while(uart0_tx_extract != uart0_tx_insert && count < UART_TX_FIFO)
					{
						U0THR = uart0_tx_buffer[uart0_tx_extract++];
						uart0_tx_extract %= UART0_TX_BUF_SIZE;
						count++;
					}
				}
				else
				{
					uart0_tx_running = 0;
				}
				break;
			case UART_RXLS_INT:
				U0LSR;
				break;
		}
	}

	VICVectAddr = 0x00000000;
//	ISR_EXIT();
}
