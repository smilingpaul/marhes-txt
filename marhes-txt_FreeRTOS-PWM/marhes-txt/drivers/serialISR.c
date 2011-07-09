/**
 @file serialISR.c
  
 @brief Includes the serial ISR and a queue initialization function.
        
 This code is from the FreeRTOS Demo application. It is modified to use both
 serial ports 0 and 1. It handles serial interrupts to efficiently send and 
 receive data.
 
 @author Titus Appel

 @version 1.0

 @date 2010/07/25
 
 @note Must be compiled in ARM mode because of ISR.

 Contact: titus.appel@gmail.com
*/

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
	BASIC INTERRUPT DRIVEN SERIAL PORT DRIVER FOR UART0. 

	This file contains all the serial port components that must be compiled
	to ARM mode.  The components that can be compiled to either ARM or THUMB
	mode are contained in serial.c.

*/

/* Standard includes. */
#include <stdlib.h>

/* Scheduler includes. */
#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"

/* Demo application includes. */
#include "serial.h"

/*-----------------------------------------------------------*/

/* Constant to access the VIC. */
#define serCLEAR_VIC_INTERRUPT			( ( unsigned long ) 0 )

/* Constants to determine the ISR source. */
#define serSOURCE_THRE					( ( unsigned char ) 0x02 )
#define serSOURCE_RX_TIMEOUT			( ( unsigned char ) 0x0c )
#define serSOURCE_ERROR					( ( unsigned char ) 0x06 )
#define serSOURCE_RX					( ( unsigned char ) 0x04 )
#define serINTERRUPT_SOURCE_MASK		( ( unsigned char ) 0x0f )

/// Queue used to hold received characters
static xQueueHandle xRxedChars[2]; 

/// Queue used to hold characters waiting to be transmitted
static xQueueHandle xCharsForTx[2]; 

/// Communication flag between the interrupt service routine and serial API
static volatile long lTHREEmpty[2];

/*-----------------------------------------------------------*/

/* 
 * The queues are created in serialISR.c as they are used from the ISR.
 * Obtain references to the queues and THRE Empty flag. 
 */
/**
 @brief Creates the RX and TX queues in the ISR so it can use them. Returns
        their pointers so serial.c can used them also.
 @param[in] port                The serial port queues to create
 @param[in] uxQueueLength       The length of the queues
 @param[out] *pxRxedChars       Queue used to hold received characters
 @param[out] *pxCharsForTx      Queue to hold characters waiting transmittion
 @param[out] **pplTHREEmptyFlag Communication flag between the interrupt service 
                                routine and serial API
*/
void vSerialISRCreateQueues(	unsigned portBASE_TYPE port, \
     unsigned portBASE_TYPE uxQueueLength, xQueueHandle *pxRxedChars, \
     xQueueHandle *pxCharsForTx, long volatile **pplTHREEmptyFlag );

/* UART interrupt service routine entry point. */
void vUART_ISR_Wrapper( void ) __attribute__ ((naked));

/* UART interrupt service routine handler. */
void vUART_ISR_Handler( void ) __attribute__ ((noinline));

/*-----------------------------------------------------------*/

/**
 @brief Creates the RX and TX queues in the ISR so it can use them. Returns
        their pointers so serial.c can used them also.
 @param[in] port                The serial port queues to create
 @param[in] uxQueueLength       The length of the queues
 @param[out] *pxRxedChars       Queue used to hold received characters
 @param[out] *pxCharsForTx      Queue to hold characters waiting transmittion
 @param[out] **pplTHREEmptyFlag Communication flag between the interrupt service 
                                routine and serial API
*/
void vSerialISRCreateQueues(	unsigned portBASE_TYPE port, \
     unsigned portBASE_TYPE uxQueueLength, xQueueHandle *pxRxedChars, \
     xQueueHandle *pxCharsForTx, long volatile **pplTHREEmptyFlag )
{
	/* Create the queues used to hold Rx and Tx characters. */
	xRxedChars[port] = xQueueCreate( uxQueueLength, ( unsigned portBASE_TYPE ) sizeof( signed char ) );
	xCharsForTx[port] = xQueueCreate( uxQueueLength + 1, ( unsigned portBASE_TYPE ) sizeof( signed char ) );

	/* Pass back a reference to the queues so the serial API file can 
	post/receive characters. */
	*pxRxedChars = xRxedChars[port];
	*pxCharsForTx = xCharsForTx[port];

	/* Initialise the THRE empty flag - and pass back a reference. */
	lTHREEmpty[port] = ( long ) pdTRUE;
	*pplTHREEmptyFlag = &lTHREEmpty[port];
}
/*-----------------------------------------------------------*/

/**
 @brief Wrapper for the serial interrupt vector.
 
 FreeRTOS can save the context (all registers, stack positions, etc.) 
 so everything doesn't get screwed up because the ARM doesn't do that for you.
*/
void vUART_ISR_Wrapper( void )
{
	/* Save the context of the interrupted task. */
	portSAVE_CONTEXT();

	/* Call the handler.  This must be a separate function from the wrapper
	to ensure the correct stack frame is set up. */
	__asm volatile ("bl vUART_ISR_Handler");

	/* Restore the context of whichever task is going to run next. */
	portRESTORE_CONTEXT();
}
/*-----------------------------------------------------------*/

/**
 @brief Serial ISR
 
 This ISR handles both UART0 and 1. The ISR handles data that wouldn't fit in 
 the ISR by writing the data written to the queue. The received data is put onto
 the queue. Error interrupts are not handled, the interrupts are just cleared.
 @note Use the wrapper function instead so the context gets saved.
*/
void vUART_ISR_Handler( void )
{
signed char cChar;
portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;

	/* What caused the interrupt? */
	switch( U0IIR & serINTERRUPT_SOURCE_MASK )
	{
		case serSOURCE_ERROR :	/* Not handling this, but clear the interrupt. */
								cChar = U0LSR;
								break;

		case serSOURCE_THRE	:	/* The THRE is empty.  If there is another
								character in the Tx queue, send it now. */
								if( xQueueReceiveFromISR( xCharsForTx[0], &cChar, &xHigherPriorityTaskWoken ) == pdTRUE )
								{
									U0THR = cChar;
								}
								else
								{
									/* There are no further characters 
									queued to send so we can indicate 
									that the THRE is available. */
									lTHREEmpty[0] = pdTRUE;
								}
								break;

		case serSOURCE_RX_TIMEOUT :
		case serSOURCE_RX	:	/* A character was received.  Place it in 
								the queue of received characters. */
								cChar = U0RBR;
								xQueueSendFromISR( xRxedChars[0], &cChar, &xHigherPriorityTaskWoken );
								break;

		default				:	/* There is nothing to do, leave the ISR. */
								break;
	}
	
  switch( U1IIR & serINTERRUPT_SOURCE_MASK )
	{
		case serSOURCE_ERROR :	/* Not handling this, but clear the interrupt. */
								cChar = U1LSR;
								break;

		case serSOURCE_THRE	:	/* The THRE is empty.  If there is another
								character in the Tx queue, send it now. */
								if( xQueueReceiveFromISR( xCharsForTx[1], &cChar, &xHigherPriorityTaskWoken ) == pdTRUE )
								{
									U1THR = cChar;
								}
								else
								{
									/* There are no further characters 
									queued to send so we can indicate 
									that the THRE is available. */
									lTHREEmpty[1] = pdTRUE;
								}
								break;

		case serSOURCE_RX_TIMEOUT :
		case serSOURCE_RX	:	/* A character was received.  Place it in 
								the queue of received characters. */
								cChar = U1RBR;
								xQueueSendFromISR( xRxedChars[1], &cChar, &xHigherPriorityTaskWoken );
								break;

		default				:	/* There is nothing to do, leave the ISR. */
								break;
	}

	if( xHigherPriorityTaskWoken )
	{
		portYIELD_FROM_ISR();
	}

	/* Clear the ISR in the VIC. */
	VICVectAddr = serCLEAR_VIC_INTERRUPT;
}






	
