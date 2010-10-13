/*
 * encoderISR.c
 *
 *  Created on: Aug 21, 2010
 *      Author: Titus
 */

#include "encoder.h"

extern uint32_t frontRightTicks, frontLeftTicks;
//extern uint32_t rearRightTicks, rearLeftTicks;
extern char inputChannel;

void EncoderISR(void)
{
	ISR_ENTRY();

	// Check for MR0 Interrupt
	if(T1IR | IR_MR0)
	{
		FIO0PIN ^= (1<<21);
		// Stop Counters
		T0TCR = 0;
		T3TCR = 0;

		// First check which channel is being read
		// If the channel is 0, store the right encoder counts and
		// then change to channel 1, reset and enable the TC
		// If the channel is 1, do the opposite
		if(inputChannel == 0)
		{
			// Store the right encoder tick counts
			frontRightTicks = T0TC;
//			rearRightTicks = T3TC;

			// Switch to channel 1 as input
			T0CTCR = CTCR_CM_RF | CTCR_CAP_SEL_1;
//			T3CTCR = CTCR_CM_RF | CTCR_CAP_SEL_1;

			inputChannel = 1;
		}
		else
		{
			// Store the left encoder tick counts
			frontLeftTicks = T0TC;
//			rearLeftTicks = T3TC;

			// Switch to channel 0 as input
			T0CTCR = CTCR_CM_RF | CTCR_CAP_SEL_0;
//			T3CTCR = CTCR_CM_RF | CTCR_CAP_SEL_0;

			inputChannel = 0;
		}

		// Reset and enable the TC
		T0TCR = TCR_CR;
//		T3TCR = TCR_CR;
		T0TCR = TCR_CE;
//		T3TCR = TCR_CE;
		T1TCR = TCR_CE;

		// Clear MR0 interrupt
		T1IR |= IR_MR0;
	}
/*	else if(T1IR | IR_MR1)*/
/*	{*/
/*		FIO0PIN ^= (1<<21);*/
/*	}*/

	VICVectAddr = 0x00000000;
	ISR_EXIT();
}
