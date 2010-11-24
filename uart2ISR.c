/*
 * uart2ISR.c
 *
 *  Created on: Jul 5, 2010
 *      Author: Titus
 */

#include "uart2.h"

extern uint8_t uart2_rx_buffer[UART2_RX_BUF_SIZE];
extern uint8_t uart2_tx_buffer[UART2_TX_BUF_SIZE];
extern uint16_t uart2_rx_insert, uart2_rx_extract;
extern uint16_t uart2_tx_insert, uart2_tx_extract;
extern int uart2_tx_running;

void Uart2ISR(void)
{
	uint8_t int_id;
	int count = 0;

	ISR_ENTRY();

	// Take care of all interrupts
	while (((int_id = U2IIR) & UART_NO_INT) == 0)
	{
		switch(int_id & UART_INT_MASK)
		{
			case UART_CTOI_INT:
			case UART_RXDA_INT:
				do {
					uint16_t tempCounter;
					uart2_rx_buffer[uart2_rx_insert] = U2RBR;
					tempCounter = (uart2_rx_insert + 1) % UART2_RX_BUF_SIZE;

					if (tempCounter != uart2_rx_extract)
						uart2_rx_insert = tempCounter;
				} while (U2LSR & UART_RDR);
				break;
			case UART_THRE_INT:
				// If there is more data in the buffer to write
				if(uart2_tx_insert != uart2_tx_extract)
				{
					// While there is more data, write up to 16 bytes in the TX FIFO
					while(uart2_tx_extract != uart2_tx_insert && count < UART_TX_FIFO)
					{
						U2THR = uart2_tx_buffer[uart2_tx_extract++];
						uart2_tx_extract %= UART2_TX_BUF_SIZE;
						count++;
					}
				}
				else
				{
					uart2_tx_running = 0;
				}
				break;
			case UART_RXLS_INT:
				U2LSR;
				break;
		}
	}

	VICVectAddr = 0x00000000;
	ISR_EXIT();
}
