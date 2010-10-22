/*
 * encoder.c
 *
 *  Created on: Jun 19, 2010
 *      Author: Titus
 *
 *  Description:
 *  	These routines initialize Timers 0 and 3 for capture mode to
 *  	count the encoder ticks to measure the velocity of the TXT-1.
 *  	The encoders being used are the US Digital E7MS optical encoder
 *  	which is currently discontinued.  The E7MS has 400 CPR (counts
 *  	per revolution).  These are quadrature encoders so you can measure
 *  	the direction also.  They have an A and a B signal.  We will use
 *  	the A signal to measure the speed with the capture pins.  The B
 *  	signal will be used with general purpose pins for direction.
 *  	There are two ways to measure how fast the encoders are moving:
 *  	1. 	Count the time in between pulses or measure the period of the
 *  	    signal.  This will give ms/tick
 *  	2. 	Count the number of ticks for a specific time interval.  This
 *  		will give ticks/ms
 *  	This routine uses the 2nd method since it is not inversely related.
 *  	With the 1st method really slow speeds will give large times and
 *  	could overflow the timer counter.  With the 2nd method this problem
 *  	doesn't happen.  Direction is found with a interrupt on the A line
 *  	and then reading the B line.  The B line being high or low determines
 *  	the direction of rotation.
 *
 *  	Note: 	Timer 3 is used for the rear encoders.  The rear encoders
 *  			are a future improvement.  In order to use these R120 and R121
 *  			need to be removed on the dev board.
 *
 *  Pin definition:
 *  	P3.23 - CAP0.0 - Front Right Encoder A Signal
 *    	P0.23 - CAP3.0 - Front Left Encoder A Signal
 */

#include "encoder.h"

// Variable declaration
uint32_t ticks[SIZE_ENCODER_ARR];

/*************************************************************************
 * Function Name: EncoderInit
 * Parameters: void
 * Return: void
 *
 * Description: Initializes the Timer 0 and 3 peripherals to count rising
 * 				pulses of the encoder A signals.  The B lines are
 * 				configured as an input to determine direction. Because the
 * 				Capture port can only count one channel at a time. We use
 * 				timer 1 to control the sample period for each channel.
 *
 *************************************************************************/
void EncoderInit(void)
{
	// 1. Power up Timer 0 and 3 for capture mode and Timer 1 as a timer
	PCONP |= PCONP_PCTIM0 | PCONP_PCTIM1 | PCONP_PCTIM3;

	// 2. Make the peripheral clocks 72 MHz = divided by one
	PCLKSEL0 |= PCLKSEL0_TIM0_DIV1 | PCLKSEL0_TIM1_DIV1;
	PCLKSEL1 |= PCLKSEL1_TIM3_DIV1;

	// 3. Select pin functions.  Make P3.23 be CAP0.0 and P3.24 be CAP0.1.
	//    Make P0.23 be CAP3.0 and P0.24 be CAP3.1.
	PINSEL7 |= (PINSEL7_CAP00);
	PINSEL1 |= (PINSEL1_CAP30);

	// 4. Setup timer counter
	T0TCR = TCR_CR;							// Reset timer0 counter
	T0CTCR = CTCR_CM_RF | CTCR_CAP_SEL_0;	// Timer is counter mode on rise/fall
											// Start the counting on channel 0
	T0PR = 0;								// Increment TC after every rise/fall

	T3TCR = TCR_CR;							// Reset timer3 counter
	T3CTCR = CTCR_CM_RF | CTCR_CAP_SEL_0;	// Timer is counter mode on rise/fall
											// Start the counting on channel 0
	T3PR = 0;								// Increment TC after every rise/fall

//	// 5. Configure inputs to other signal of quadrature encoders to
//	//    find out the direction of the wheels (input = 0)
//	FIO2DIR &= ~(0x00000C0);				// Front B signals as inputs
////	FIO2DIR &= ~(0x0000300);				// Rear B signals as inputs

	// 6. Setup timer1 for a sample period of 20msec to switch between channels
	//    for counting enabling the interrupt
	T1TCR = TCR_CR;							// Reset timer1 counter
	T1CTCR = CTCR_TM;						// Timer 1 is in timer mode
	T1MR0 = MCR_20MS;						// Match at 20ms
	T1MCR = MCR_MR0I | MCR_MR0S | MCR_MR0R;	// On match, interrupt,reset,stop TC

	// Setup T1 Interrupt
	VICIntSelect &= ~VIC_CHAN_TO_MASK(VIC_CHAN_NUM_Timer1);	// Change to IRQ
	VICIntEnClr |= VIC_CHAN_TO_MASK(VIC_CHAN_NUM_Timer1); 	// Disable interrupt
	VICVectAddr5 = (uint32_t)(void *)EncoderISR;			// Assign the ISR
	VICVectPriority5 = 0xF;									// Set the priority
	VICIntEnable |= VIC_CHAN_TO_MASK(VIC_CHAN_NUM_Timer1);	// Enable the interrupt

	// 7. Enable the Timer counters
	T0TCR = TCR_CE;
	T3TCR = TCR_CE;
	T1TCR = TCR_CE;
}

uint32_t EncoderCount(uint8_t channel)
{
	uint32_t count;
	count = ticks[channel];
	return count;
}
