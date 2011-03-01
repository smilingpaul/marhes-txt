/*
 * timer2.c
 *
 *  Created on: Feb 26, 2011
 *      Author: titus
 */

#include "timer.h"
#include "timer2.h"

void Timer2Init(void)
{
	PCONP |= PCONP_PCTIM2;
	PCLKSEL1 |= PCLKSEL1_TIM2_DIV1;

	// Setup timer2
	T2TCR = TCR_CR;							// Reset timer2 counter
	T2CTCR = CTCR_TM;						// Timer 2 is in timer mode
	T2PR = PR_1MS;							// Set TC increment every 1ms
	T2MR0 = MCR_1S;							// Match at 1s (ODOM_COMB)
//	T2MR1 = MCR_2S;							// Match at 2s (CMD_VEL)
//	T2MR2 = MCR_10S;						// Match at 10s (BATTERY MSG)
//	T2MR3 =									// (PWM Timer)

	// On match, interrupt and on MR2 reset TC
	T2MCR = MCR_MR0I | MCR_MR0R | MCR_MR0S;//MCR_MR1I | MCR_MR2I | MCR_MR2R;

	// Setup T2 Interrupt
	VICIntSelect &= ~VIC_CHAN_TO_MASK(VIC_CHAN_NUM_Timer2);	// Change to IRQ
	VICIntEnClr |= VIC_CHAN_TO_MASK(VIC_CHAN_NUM_Timer2); 	// Disable interrupt
	VICVectAddr26 = (uint32_t)(void *)Timer2ISR;			// Assign the ISR
	VICVectPriority26 = 0xD;								// Set the priority
	VICIntEnable |= VIC_CHAN_TO_MASK(VIC_CHAN_NUM_Timer2);	// Enable the INT

	// Enable the timer counter
	T2TCR = TCR_CE;
}

