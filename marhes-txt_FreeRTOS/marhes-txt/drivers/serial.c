/*
    FreeRTOS V7.0.1 - Copyright (C) 2011 Real Time Engineers Ltd.
	

    ***************************************************************************
     *                                                                       *
     *    FreeRTOS tutorial books are available in pdf and paperback.        *
     *    Complete, revised, and edited pdf reference manuals are also       *
     *    available.                                                         *
     *                                                                       *
     *    Purchasing FreeRTOS documentation will not only help you, by       *
     *    ensuring you get running as quickly as possible and with an        *
     *    in-depth knowledge of how to use FreeRTOS, it will also help       *
     *    the FreeRTOS project to continue with its mission of providing     *
     *    professional grade, cross platform, de facto standard solutions    *
     *    for microcontrollers - completely free of charge!                  *
     *                                                                       *
     *    >>> See http://www.FreeRTOS.org/Documentation for details. <<<     *
     *                                                                       *
     *    Thank you for using FreeRTOS, and thank you for your support!      *
     *                                                                       *
    ***************************************************************************


    This file is part of the FreeRTOS distribution.

    FreeRTOS is free software; you can redistribute it and/or modify it under
    the terms of the GNU General Public License (version 2) as published by the
    Free Software Foundation AND MODIFIED BY the FreeRTOS exception.
    >>>NOTE<<< The modification to the GPL is included to allow you to
    distribute a combined work that includes FreeRTOS without being obliged to
    provide the source code for proprietary components outside of the FreeRTOS
    kernel.  FreeRTOS is distributed in the hope that it will be useful, but
    WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
    or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
    more details. You should have received a copy of the GNU General Public
    License and the FreeRTOS license exception along with FreeRTOS; if not it
    can be viewed here: http://www.freertos.org/a00114.html and also obtained
    by writing to Richard Barry, contact details for whom are available on the
    FreeRTOS WEB site.

    1 tab == 4 spaces!

    http://www.FreeRTOS.org - Documentation, latest information, license and
    contact details.

    http://www.SafeRTOS.com - A version that is certified for use in safety
    critical systems.

    http://www.OpenRTOS.com - Commercial support, development, porting,
    licensing and training services.
*/

/*
	Changes from V2.4.0

		+ Made serial ISR handling more complete and robust.

	Changes from V2.4.1

		+ Split serial.c into serial.c and serialISR.c.  serial.c can be 
		  compiled using ARM or THUMB modes.  serialISR.c must always be
		  compiled in ARM mode.
		+ Another small change to cSerialPutChar().

	Changed from V2.5.1

		+ In cSerialPutChar() an extra check is made to ensure the post to
		  the queue was successful if then attempting to retrieve the posted
		  character.

*/

/* 
	BASIC INTERRUPT DRIVEN SERIAL PORT DRIVER FOR UART0. 

	This file contains all the serial port components that can be compiled to
	either ARM or THUMB mode.  Components that must be compiled to ARM mode are
	contained in serialISR.c.
*/

/* Standard includes. */
#include <stdlib.h>

/* Scheduler includes. */
#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"

/* Demo application includes. */
#include "serial.h"
#include "app_types.h"

/*-----------------------------------------------------------*/

/* Constants to setup and access the UART. */
#define serPCONP_UART0			    (1<<3)
#define serPCLKSEL0_UART0_DIV1  (1<<6)
#define serPCONP_UART1			    (1<<4)
#define serPCLKSEL0_UART1_DIV1  (1<<8)
#define serPINSEL0_TXD0		(1<<4)
#define serPINSEL0_RXD0		(1<<6)
#define serPINSEL0_TXD1		(1<<30)
#define serPINSEL1_RXD1		(1<<0)
#define serDLAB							( ( unsigned char ) 0x80 )
#define serENABLE_INTERRUPTS			( ( unsigned char ) 0x03 )
#define serNO_PARITY					( ( unsigned char ) 0x00 )
#define ser1_STOP_BIT					( ( unsigned char ) 0x00 )
#define ser8_BIT_CHARS					( ( unsigned char ) 0x03 )
#define serFIFO_ON						( ( unsigned char ) 0x01 )
#define serCLEAR_FIFO					( ( unsigned char ) 0x06 )
#define serWANTED_CLOCK_SCALING			( ( unsigned long ) 16 )

/* Constants to setup and access the VIC. */
#define serUART0_VIC_CHANNEL			( ( unsigned long ) 0x0006 )
#define serUART0_VIC_CHANNEL_BIT		( ( unsigned long ) 0x0040 )
#define serUART0_VIC_ENABLE				( ( unsigned long ) 0x0020 )
#define serUART1_VIC_CHANNEL			( ( unsigned long ) 0x0007 )
#define serUART1_VIC_CHANNEL_BIT		( ( unsigned long ) 0x0080 )
#define serUART1_VIC_ENABLE				( ( unsigned long ) 0x0020 )
#define serCLEAR_VIC_INTERRUPT			( ( unsigned long ) 0 )

#define serINVALID_QUEUE				( ( xQueueHandle ) 0 )
#define serHANDLE						( ( xComPortHandle ) 1 )
#define serNO_BLOCK						( ( portTickType ) 0 )

/*-----------------------------------------------------------*/

/* Queues used to hold received characters, and characters waiting to be
transmitted. */
static xQueueHandle xRxedChars[2]; 
static xQueueHandle xCharsForTx[2]; 

/*-----------------------------------------------------------*/

/* Communication flag between the interrupt service routine and serial API. */
static volatile long *plTHREEmpty[2];

const uint32_t portHandles[2] = {0, 1};

/**
 * Lookup the baud rate from the enum.
 */
static unsigned long prvBaud( eBaud eWantedBaud ); 

static unsigned long prvBaud( eBaud eWantedBaud )
{
	switch( eWantedBaud )
    {
		case ser50			:	return 50UL;
		case ser75			:	return 75UL;
		case ser110			:	return 110UL;
		case ser134			:	return 134UL;
		case ser150			:	return 150UL;
		case ser200			:	return 200UL;
		case ser300			:	return 300UL;
		case ser600			:	return 600UL;
		case ser1200		:	return 1200UL;
		case ser1800		:	return 1800UL;
		case ser2400		:	return 2400UL;
		case ser4800		:	return 4800UL;
		case ser19200		:	return 19200UL;
		case ser38400		:	return 38400UL;
		case ser57600		:	return 57600UL;
		case ser115200  :	return 115200UL;
		default         : return 9600UL;
    }
}

/* 
 * The queues are created in serialISR.c as they are used from the ISR.
 * Obtain references to the queues and THRE Empty flag. 
 */
void vSerialISRCreateQueues(	unsigned portBASE_TYPE port, unsigned portBASE_TYPE uxQueueLength, xQueueHandle *pxRxedChars, 
								xQueueHandle *pxCharsForTx, long volatile **pplTHREEmptyFlag );

/*-----------------------------------------------------------*/

xComPortHandle xSerialPortInit( eCOMPort ePort, eBaud eWantedBaud, eParity eWantedParity, eDataBits eWantedDataBits, eStopBits        
                                eWantedStopBits, unsigned portBASE_TYPE uxBufferLength )
{
unsigned long ulDivisor, ulWantedClock;
xComPortHandle xReturn;
extern void ( vUART_ISR_Wrapper )( void );
unsigned char ucPort;

  ucPort = ( unsigned long ) ePort;
  
  if (ucPort == 0)
  {
  	/* The queues are used in the serial ISR routine, so are created from
	     serialISR.c (which is always compiled to ARM mode. */
	  vSerialISRCreateQueues( 0, uxBufferLength, &xRxedChars[0], &xCharsForTx[0], &plTHREEmpty[0] );
    
    if( 
		  ( xRxedChars[0] != serINVALID_QUEUE ) && 
		  ( xCharsForTx[0] != serINVALID_QUEUE ) 
	    )
	  {
		  portENTER_CRITICAL();
		  {
    		PCONP |= serPCONP_UART0;
    		PCLKSEL0 |= serPCLKSEL0_UART0_DIV1;
		
			  /* Setup the baud rate:  Calculate the divisor value. */
			  ulWantedClock = prvBaud(eWantedBaud) * serWANTED_CLOCK_SCALING;
			  ulDivisor = configCPU_CLOCK_HZ / ulWantedClock;

			  /* Set the DLAB bit so we can access the divisor. */
			  U0LCR |= serDLAB;

			  /* Setup the divisor. */
			  U0DLL = ( unsigned char ) ( ulDivisor & ( unsigned long ) 0xff );
			  ulDivisor >>= 8;
			  U0DLM = ( unsigned char ) ( ulDivisor & ( unsigned long ) 0xff );
			  //U0FDR = 0x21;

			  /* Turn on the FIFO's and clear the buffers. */
			  U0FCR = ( serFIFO_ON | serCLEAR_FIFO );

			  /* Setup transmission format. */
			  U0LCR = serNO_PARITY | ser1_STOP_BIT | ser8_BIT_CHARS;
			
			  PINSEL0 |= serPINSEL0_TXD0 | serPINSEL0_RXD0;

			  /* Setup the VIC for the UART. */
			  VICIntSelect &= ~( serUART0_VIC_CHANNEL_BIT );
			  VICIntEnable |= serUART0_VIC_CHANNEL_BIT;
			  VICVectAddr6 = ( long ) vUART_ISR_Wrapper;
			  VICVectCntl6 = serUART0_VIC_CHANNEL;// | serUART0_VIC_ENABLE;

			  /* Enable UART0 interrupts. */
			  U0IER |= serENABLE_INTERRUPTS;
			  
			  xReturn = ( xComPortHandle ) &portHandles[0];
		  }
		  portEXIT_CRITICAL();
	  }
  }
  
  if (ucPort == 1)
  {
  	/* The queues are used in the serial ISR routine, so are created from
	     serialISR.c (which is always compiled to ARM mode. */
	  vSerialISRCreateQueues( 1, uxBufferLength, &xRxedChars[1], &xCharsForTx[1], &plTHREEmpty[1] );
    
    if( 
		  ( xRxedChars[1] != serINVALID_QUEUE ) && 
		  ( xCharsForTx[1] != serINVALID_QUEUE ) 
	    )
	  {
		  portENTER_CRITICAL();
		  {
    		PCONP |= serPCONP_UART1;
    		PCLKSEL0 |= serPCLKSEL0_UART1_DIV1;
		
			  /* Setup the baud rate:  Calculate the divisor value. */
			  ulWantedClock = prvBaud(eWantedBaud) * serWANTED_CLOCK_SCALING;
			  ulDivisor = configCPU_CLOCK_HZ / ulWantedClock;

			  /* Set the DLAB bit so we can access the divisor. */
			  U1LCR |= serDLAB;

			  /* Setup the divisor. */
			  U1DLL = ( unsigned char ) ( ulDivisor & ( unsigned long ) 0xff );
			  ulDivisor >>= 8;
			  U1DLM = ( unsigned char ) ( ulDivisor & ( unsigned long ) 0xff );
			  
			  /* Turn on the FIFO's and clear the buffers. */
			  U1FCR = ( serFIFO_ON | serCLEAR_FIFO );

			  /* Setup transmission format. */
			  U1LCR = serNO_PARITY | ser1_STOP_BIT | ser8_BIT_CHARS;
			
			  PINSEL0 |= serPINSEL0_TXD1;
        PINSEL1 |= serPINSEL1_RXD1;

			  /* Setup the VIC for the UART. */
			  VICIntSelect &= ~( serUART1_VIC_CHANNEL_BIT );
			  VICIntEnable |= serUART1_VIC_CHANNEL_BIT;
			  VICVectAddr7 = ( long ) vUART_ISR_Wrapper;
			  VICVectCntl7 = serUART1_VIC_CHANNEL;// | serUART1_VIC_ENABLE;

			  /* Enable UART1 interrupts. */
			  U1IER |= serENABLE_INTERRUPTS;
			  
			  xReturn = ( xComPortHandle ) &portHandles[1];
		  }
		  portEXIT_CRITICAL();
	  }
  }
  
  return xReturn;
}

xComPortHandle xSerialPortInitMinimal( unsigned long ulWantedBaud, unsigned portBASE_TYPE uxQueueLength )
{
unsigned long ulDivisor, ulWantedClock;
xComPortHandle xReturn = serHANDLE;
extern void ( vUART_ISR_Wrapper )( void );

	/* The queues are used in the serial ISR routine, so are created from
	serialISR.c (which is always compiled to ARM mode. */
	vSerialISRCreateQueues( 0, uxQueueLength, &xRxedChars[0], &xCharsForTx[0], &plTHREEmpty[0] );

	if( 
		( xRxedChars[0] != serINVALID_QUEUE ) && 
		( xCharsForTx[0] != serINVALID_QUEUE ) && 
		( ulWantedBaud != ( unsigned long ) 0 ) 
	  )
	{
		portENTER_CRITICAL();
		{
  		PCONP |= serPCONP_UART0;
  		PCLKSEL0 |= serPCLKSEL0_UART0_DIV1;
		
			/* Setup the baud rate:  Calculate the divisor value. */
			ulWantedClock = ulWantedBaud * serWANTED_CLOCK_SCALING;
			ulDivisor = configCPU_CLOCK_HZ / ulWantedClock;

			/* Set the DLAB bit so we can access the divisor. */
			U0LCR |= serDLAB;

			/* Setup the divisor. */
			U0DLL = ( unsigned char ) ( ulDivisor & ( unsigned long ) 0xff );
			ulDivisor >>= 8;
			U0DLM = ( unsigned char ) ( ulDivisor & ( unsigned long ) 0xff );
			//U0FDR = 0x21;

			/* Turn on the FIFO's and clear the buffers. */
			U0FCR = ( serFIFO_ON | serCLEAR_FIFO );

			/* Setup transmission format. */
			U0LCR = serNO_PARITY | ser1_STOP_BIT | ser8_BIT_CHARS;
			
			PINSEL0 |= serPINSEL0_TXD0 | serPINSEL0_RXD0;

			/* Setup the VIC for the UART. */
			VICIntSelect &= ~( serUART0_VIC_CHANNEL_BIT );
			VICIntEnable |= serUART0_VIC_CHANNEL_BIT;
			VICVectAddr6 = ( long ) vUART_ISR_Wrapper;
			VICVectCntl6 = serUART0_VIC_CHANNEL | serUART0_VIC_ENABLE;

			/* Enable UART0 interrupts. */
			U0IER |= serENABLE_INTERRUPTS;
		}
		portEXIT_CRITICAL();
	}
	else
	{
		xReturn = ( xComPortHandle ) 0;
	}

	return xReturn;
}
/*-----------------------------------------------------------*/

signed portBASE_TYPE xSerialGetChar( xComPortHandle pxPort, signed char *pcRxedChar, portTickType xBlockTime )
{
	/* The port handle is not required as this driver only supports UART0. */
	//( void ) pxPort;
	uint32_t uiPort = *((uint32_t*)pxPort);

	/* Get the next character from the buffer.  Return false if no characters
	are available, or arrive before xBlockTime expires. */
	if( xQueueReceive( xRxedChars[uiPort], pcRxedChar, xBlockTime ) )
	{
		return pdTRUE;

	}
	else
	{
		return pdFALSE;
	}
}
/*-----------------------------------------------------------*/

void vSerialPutString( xComPortHandle pxPort, const signed char * const pcString, unsigned short usStringLength )
{
signed char *pxNext;
unsigned short cnt = 0;

	/* NOTE: This implementation does not handle the queue being full as no
	block time is used! */

	/* The port handle is not required as this driver only supports UART0. */
	( void ) pxPort;
	( void ) usStringLength;
  
	/* Send each character in the string, one at a time. */
	portENTER_CRITICAL();
	{
	  pxNext = ( signed char * ) pcString;
	  while( cnt < usStringLength )
	  {
		  xSerialPutChar( pxPort, *pxNext, serNO_BLOCK );
		  pxNext++;
		  cnt++;
	  }
	}
	portEXIT_CRITICAL();
}
/*-----------------------------------------------------------*/

signed portBASE_TYPE xSerialPutChar( xComPortHandle pxPort, signed char cOutChar, portTickType xBlockTime )
{
signed portBASE_TYPE xReturn;

	/* This demo driver only supports one port so the parameter is not used. */
	//( void ) pxPort;
  uint32_t uiPort = *((uint32_t*)pxPort);
  
	portENTER_CRITICAL();
	{
		/* Is there space to write directly to the UART? */
		if( *plTHREEmpty[uiPort] == ( long ) pdTRUE )
		{
			/* We wrote the character directly to the UART, so was 
			successful. */
			*plTHREEmpty[uiPort] = pdFALSE;
			if (uiPort == 0)
			  U0THR = cOutChar;
 			if (uiPort == 1)
			  U1THR = cOutChar;
			xReturn = pdPASS;
		}
		else 
		{
			/* We cannot write directly to the UART, so queue the character.
			Block for a maximum of xBlockTime if there is no space in the
			queue. */
			xReturn = xQueueSend( xCharsForTx[uiPort], &cOutChar, xBlockTime );

			/* Depending on queue sizing and task prioritisation:  While we 
			were blocked waiting to post interrupts were not disabled.  It is 
			possible that the serial ISR has emptied the Tx queue, in which
			case we need to start the Tx off again. */
			if( ( *plTHREEmpty[uiPort] == ( long ) pdTRUE ) && ( xReturn == pdPASS ) )
			{
				xQueueReceive( xCharsForTx[uiPort], &cOutChar, serNO_BLOCK );
				*plTHREEmpty[uiPort] = pdFALSE;
			  if (uiPort == 0)
			    U0THR = cOutChar;
   			if (uiPort == 1)
			    U1THR = cOutChar;
			}
		}
	}
	portEXIT_CRITICAL();

	return xReturn;
}
/*-----------------------------------------------------------*/

void vSerialClose( xComPortHandle xPort )
{
	/* Not supported as not required by the demo application. */
	( void ) xPort;
}
/*-----------------------------------------------------------*/





	
