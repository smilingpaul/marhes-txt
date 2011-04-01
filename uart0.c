/*
 * uart0.c
 *
 *  Created on: Jun 18, 2010
 *      Author: Titus
 */

#include "uart0.h"

uint8_t uart0_rx_buffer[UART0_RX_BUF_SIZE];
uint8_t uart0_tx_buffer[UART0_TX_BUF_SIZE];
uint16_t uart0_rx_insert, uart0_rx_extract;
uint16_t uart0_tx_insert, uart0_tx_extract;
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
	U0LCR = UART_DLAB;
	U0DLL = 52;//78;//26;
	U0DLM = 0;
	U0FDR = 0x21;
	U0LCR = UART_8BIT;

	// 4. Enable UART FIFO
	U0FCR |= UART_FIFO_EN | UART_RX_FIFO_RST | UART_TX_FIFO_RST;

	// 5. Enable Peripheral Pins P0.2(TXD0) & P0.3(RXD0)
	PINSEL0 |= PINSEL0_TXD0 | PINSEL0_RXD0;

	// 6. Enable transmit (no handshaking required)
	U0TER |= UART_TXEN;

	// 7. Setup Interrupts
	VICIntSelect &= ~VIC_CHAN_TO_MASK(VIC_CHAN_NUM_UART0);			// Change to IRQ
	VICIntEnClr |= VIC_CHAN_TO_MASK(VIC_CHAN_NUM_UART0); 				// Disable interrupt
	VICVectAddr6 = (uint32_t)(void *)Uart0ISR;	// Assign the ISR
	VICVectPriority6 = 0xE;							// Set the priority
	VICIntEnable |= VIC_CHAN_TO_MASK(VIC_CHAN_NUM_UART0);				// Enable the interrupt
	U0IER = UART_RXDAIE;							// Enable RX Data interrupt
	uart0_rx_insert = uart0_rx_extract = 0;			// Initialize buffer positions
	uart0_tx_insert = uart0_tx_extract = 0;
	uart0_tx_running = 0;
}

void Uart0TxString(const char *data)
{
	register char ch;

	if (*data == 0)
		return;

	U0IER &= ~UART_THREIE;					// Disable THR Interrupt

	// If currently running add to the buffer, and if not, write to the
	// Transmit Holding Register and set the transmit buffer flag
	// 16 Byte TX FIFO
	if(uart0_tx_running)
	{
		while((ch = *data))						// Write all chars to buffer
		{
			uart0_tx_buffer[uart0_tx_insert++] = ch;
			data++;
			uart0_tx_insert %= UART0_TX_BUF_SIZE;
		}
	}
	else
	{
		uart0_tx_running = 1;					// Write the TX running flag
		int count = 0;

		// Can only initially write 16 bytes to the FIFO
		while((ch = *data) && (count < 16))		// While there are characters left to write
		{										// Null terminated string stops the loop
			U0THR = ch;						// Load data into transmit buffer
			data++;
			count++;
		}

		// Write the rest of the bytes to the TX Buffer
		while((ch = *data))
		{
			uart0_tx_buffer[uart0_tx_insert++] = ch;
			data++;
			uart0_tx_insert %= UART0_TX_BUF_SIZE;
		}
	}
	U0IER |= UART_THREIE;					// Enable THR Interrupt
}

void Uart0TxArr(uint8_t *data, uint8_t numBytes)
{
	U0IER &= ~UART_THREIE;					// Disable THR Interrupt

	// If currently running add to the buffer, and if not, write to the
	// Transmit Holding Register and set the transmit buffer flag
	// 16 Byte TX FIFO
	if(uart0_tx_running)
	{
		int count = 0;
		while(count < numBytes)						// Write all chars to buffer
		{
			uart0_tx_buffer[uart0_tx_insert++] = *data;
			data++;
			count++;
			uart0_tx_insert %= UART0_TX_BUF_SIZE;
		}
	}
	else
	{
		uart0_tx_running = 1;					// Write the TX running flag
		int count = 0;

		// Can only initially write 16 bytes to the FIFO
		while((count < numBytes) && (count < 16))	// While there is data left to write
		{
			U0THR = *data;					        // Load data into transmit buffer
			data++;
			count++;
		}

		// Write the rest of the bytes to the TX Buffer
		while(count < numBytes)
		{
			uart0_tx_buffer[uart0_tx_insert++] = *data;
			data++;
			count++;
			uart0_tx_insert %= UART0_TX_BUF_SIZE;
		}
	}
	U0IER |= UART_THREIE;					// Enable THR Interrupt
}

int Uart0TxBufUsed(void)
{
	int space = uart0_tx_insert - uart0_tx_extract;
	if (space < 0)
		space += UART0_TX_BUF_SIZE;
	return space;
}

int Uart0TxBufSpace(void)
{
	return UART0_TX_BUF_SIZE - Uart0TxBufUsed();
}

char Uart0RxChar(void)
{
	register char data = uart0_rx_buffer[uart0_rx_extract++];
	uart0_rx_extract %= UART0_RX_BUF_SIZE;

	return data;
}

char* Uart0RxString(char* dest)
{
	int i = 0;

	while (Uart0RxDataReady() && i < UART0_RX_STR_SIZE - 1)
	{
		*dest = Uart0RxChar();
		dest++;
		i++;
	}
	*dest = '\0';
	return dest;
}

uint8_t Uart0RxDataReady(void)
{
	if (uart0_rx_extract == uart0_rx_insert)
		return 0;
	else
		return 1;
}

int Uart0RxBufUsed(void)
{
	int used = uart0_rx_insert - uart0_rx_extract;
	if (used < 0)
		used += UART0_RX_BUF_SIZE;
	return used;
}

int Uart0RxBufSpace(void)
{
	return UART0_RX_BUF_SIZE - Uart0RxBufUsed();
}
