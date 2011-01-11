/*
 * uart2.c
 *
 *  Created on: Jun 18, 2010
 *      Author: Titus
 */

#include "uart2.h"

uint8_t uart2_rx_buffer[UART2_RX_BUF_SIZE];
uint8_t uart2_tx_buffer[UART2_TX_BUF_SIZE];
uint16_t uart2_rx_insert, uart2_rx_extract;
uint16_t uart2_tx_insert, uart2_tx_extract;
int uart2_tx_running;

// Setup UART2 for the GPS at 19200 baud on P0.10 and P0.11
void Uart2Init(void)
{
	// 1. Turn on UART2 peripheral
	PCONP |= PCONP_UART2;

	// 2. Select the peripheral clock divider
	//    For baud rate of 19200 use divider of 1 so PCLK = 72MHz
	//    BR = PCLK / (16 * (256*DLM+DLL) * (1 + DIV/MULT))
	//    DLM = 0, DLL = 78, DIV = 1, MULT = 2
	//    So BR = 38461.5
	PCLKSEL0 |= PCLKSEL1_UART2_DIV1;

	// 3. Set the DLAB bit to write to the baud registers
	U2LCR = UART_DLAB;
	U2DLL = 26;
	U2DLM = 0;
	U2FDR = 0x21;
	U2LCR = UART_8BIT;

	// 4. Enable UART FIFO
	U2FCR |= UART_FIFO_EN | UART_RX_FIFO_RST | UART_TX_FIFO_RST;

	// 5. Enable Peripheral Pins P0.10(TXD2) & P0.11(RXD2)
	PINSEL0 |= PINSEL0_TXD2 | PINSEL0_RXD2;

	// 6. Enable transmit (no handshaking required)
	U2TER |= UART_TXEN;

	// 7. Setup Interrupts
	VICIntSelect &= ~VIC_CHAN_TO_MASK(VIC_CHAN_NUM_UART2);	// Change to IRQ
	VICIntEnClr |= VIC_CHAN_TO_MASK(VIC_CHAN_NUM_UART2); 	// Disable interrupt
	VICVectAddr28 = (uint32_t)(void *)Uart2ISR;				// Assign the ISR
	VICVectPriority28 = 0xE;									// Set the priority
	VICIntEnable |= VIC_CHAN_TO_MASK(VIC_CHAN_NUM_UART2);	// Enable the interrupt
	U2IER = UART_RXDAIE;									// Enable RX Data interrupt
	uart2_rx_insert = uart2_rx_extract = 0;					// Initialize buffer positions
	uart2_tx_insert = uart2_tx_extract = 0;
	uart2_tx_running = 0;
}

void Uart2TxString(const char *data)
{
	register char ch;

	if (*data == 0)
		return;

	U2IER &= ~UART_THREIE;					// Disable THR Interrupt

	// If currently running add to the buffer, and if not, write to the
	// Transmit Holding Register and set the transmit buffer flag
	// 16 Byte TX FIFO
	if(uart2_tx_running)
	{
		while((ch = *data))						// Write all chars to buffer
		{
			uart2_tx_buffer[uart2_tx_insert++] = ch;
			data++;
			uart2_tx_insert %= UART2_TX_BUF_SIZE;
		}
	}
	else
	{
		uart2_tx_running = 1;					// Write the TX running flag
		int count = 0;

		// Can only initially write 16 bytes to the FIFO
		while((ch = *data) && (count < 16))		// While there are characters left to write
		{										// Null terminated string stops the loop
			U2THR = ch;						// Load data into transmit buffer
			data++;
			count++;
		}

		// Write the rest of the bytes to the TX Buffer
		while((ch = *data))
		{
			uart2_tx_buffer[uart2_tx_insert++] = ch;
			data++;
			uart2_tx_insert %= UART2_TX_BUF_SIZE;
		}
	}
	U2IER |= UART_THREIE;					// Enable THR Interrupt
}

void Uart2TxArr(uint8_t *data, uint8_t numBytes)
{
	U2IER &= ~UART_THREIE;					// Disable THR Interrupt

	// If currently running add to the buffer, and if not, write to the
	// Transmit Holding Register and set the transmit buffer flag
	// 16 Byte TX FIFO
	if(uart2_tx_running)
	{
		int count = 0;
		while(count < numBytes)						// Write all chars to buffer
		{
			uart2_tx_buffer[uart2_tx_insert++] = *data;
			data++;
			count++;
			uart2_tx_insert %= UART2_TX_BUF_SIZE;
		}
	}
	else
	{
		uart2_tx_running = 1;					// Write the TX running flag
		int count = 0;

		// Can only initially write 16 bytes to the FIFO
		while((count < numBytes) && (count < 16))	// While there is data left to write
		{
			U2THR = *data;					        // Load data into transmit buffer
			data++;
			count++;
		}

		// Write the rest of the bytes to the TX Buffer
		while(count < numBytes)
		{
			uart2_tx_buffer[uart2_tx_insert++] = *data;
			data++;
			count++;
			uart2_tx_insert %= UART2_TX_BUF_SIZE;
		}
	}
	U2IER |= UART_THREIE;					// Enable THR Interrupt
}

int Uart2TxBufUsed(void)
{
	int space = uart2_tx_insert - uart2_tx_extract;
	if (space < 0)
		space += UART2_TX_BUF_SIZE;
	return space;
}

int Uart2TxBufSpace(void)
{
	return UART2_TX_BUF_SIZE - Uart2TxBufUsed();
}

char Uart2RxChar(void)
{
	register char data = uart2_rx_buffer[uart2_rx_extract++];
	uart2_rx_extract %= UART2_RX_BUF_SIZE;

	return data;
}

char* Uart2RxString(char* dest)
{
	int i = 0;

	while (Uart2RxDataReady() && i < UART2_RX_STR_SIZE - 1)
	{
		*dest = Uart2RxChar();
		dest++;
		i++;
	}
	*dest = '\0';
	return dest;
}

uint8_t Uart2RxDataReady(void)
{
	if (uart2_rx_extract == uart2_rx_insert)
		return 0;
	else
		return 1;
}

int Uart2RxBufUsed(void)
{
	int used = uart2_rx_insert - uart2_rx_extract;
	if (used < 0)
		used += UART2_RX_BUF_SIZE;
	return used;
}

int Uart2RxBufSpace(void)
{
	return UART2_RX_BUF_SIZE - Uart2RxBufUsed();
}

